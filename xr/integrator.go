package xr

import (
	"sort"
	"sync"
)

// Snapshot 是某一时刻全部设备状态的一致快照，不可变。
type Snapshot struct {
	// Devices 按 UID 排序，保证确定性。
	Devices []DeviceState
	// Time 是快照对应的单调时钟时间戳。
	Time int64
	// Space 是快照姿态所在的参考空间。
	Space SpaceID
}

// Device 按 UID 查找设备状态。
func (s *Snapshot) Device(uid DeviceUID) (DeviceState, bool) {
	for _, d := range s.Devices {
		if d.Info.UID == uid {
			return d, true
		}
	}
	return DeviceState{}, false
}

// deviceEntry 是单台设备的内部状态，热插拔期间保持稳定身份与绑定。
type deviceEntry struct {
	info      DeviceInfo
	attached  bool
	tracking  TrackingState
	latest    TimedPose
	hasSample bool
	lastSeq   uint64
	hasSeq    bool
	bindings  map[string]string

	// 追踪恢复平滑：恢复时刻记录的预测姿态与恢复窗口。
	recovering    bool
	recoveryFrom  Pose
	recoveryStart int64
}

// Integrator 是统一会话与追踪集成层的核心：
// 接入多来源帧流，归一设备模型，产出一致快照。
//
// 并发模型：Ingest/Attach/Detach/RelocateSpace 与 Snapshot 可并发调用；
// Snapshot 仅在构建期间短暂持锁，返回后即为不可变快照，
// 慢读取方不会阻塞更新推进。
type Integrator struct {
	cfg       Config
	predictor *predictor
	spaces    *SpaceManager

	mu      sync.Mutex
	devices map[DeviceUID]*deviceEntry
	// correction 是传感器空间到本地空间的校正，
	// 本地空间重定位时更新，保证设备世界姿态连续。
	correction Pose
}

func NewIntegrator(cfg Config) *Integrator {
	return &Integrator{
		cfg:        cfg,
		predictor:  newPredictor(cfg.Predict),
		spaces:     NewSpaceManager(),
		devices:    map[DeviceUID]*deviceEntry{},
		correction: IdentityPose(),
	}
}

// Spaces 返回参考空间管理器。
func (it *Integrator) Spaces() *SpaceManager { return it.spaces }

// Config 返回集成层配置。
func (it *Integrator) Config() Config { return it.cfg }

// Attach 接入一台设备；相同 UID 重复接入保持稳定身份与既有绑定。
func (it *Integrator) Attach(info DeviceInfo) {
	it.mu.Lock()
	defer it.mu.Unlock()
	if e, ok := it.devices[info.UID]; ok {
		// 重新接入：身份与绑定保留，追踪状态复位等待新数据。
		e.info = info
		e.attached = true
		e.tracking = TrackingStateNotTracked
		e.hasSeq = false
		e.hasSample = false
		e.recovering = false
		return
	}
	it.devices[info.UID] = &deviceEntry{
		info:     info,
		attached: true,
		tracking: TrackingStateNotTracked,
		bindings: map[string]string{},
	}
}

// Detach 移除一台设备；移除期间到达的数据被丢弃，不影响其他设备。
// 设备条目保留，重新接入时身份与绑定不丢失。
func (it *Integrator) Detach(uid DeviceUID) {
	it.mu.Lock()
	defer it.mu.Unlock()
	if e, ok := it.devices[uid]; ok {
		e.attached = false
		e.tracking = TrackingStateNotTracked
		e.hasSample = false
		e.recovering = false
	}
}

// SetBinding 设置设备绑定配置。
func (it *Integrator) SetBinding(uid DeviceUID, key, value string) {
	it.mu.Lock()
	defer it.mu.Unlock()
	if e, ok := it.devices[uid]; ok {
		e.bindings[key] = value
	}
}

// Binding 读取设备绑定配置。
func (it *Integrator) Binding(uid DeviceUID, key string) (string, bool) {
	it.mu.Lock()
	defer it.mu.Unlock()
	if e, ok := it.devices[uid]; ok {
		v, ok := e.bindings[key]
		return v, ok
	}
	return "", false
}

// Ingest 消费一帧数据。
// 规则：
//   - 未接入或已移除设备的帧直接丢弃，不会错位到其他设备；
//   - 重复帧（序号不递增）丢弃；
//   - 时间戳不晚于当前最新采样的迟到帧丢弃，不得覆盖较新姿态；
//   - 追踪丢失帧只更新追踪状态，不触碰姿态采样。
func (it *Integrator) Ingest(f Frame) {
	it.mu.Lock()
	defer it.mu.Unlock()
	e, ok := it.devices[f.UID]
	if !ok || !e.attached {
		return
	}
	if e.hasSeq && f.Seq <= e.lastSeq {
		return
	}
	e.lastSeq = f.Seq
	e.hasSeq = true

	if f.Tracking != TrackingStateTracked {
		e.tracking = TrackingStateNotTracked
		return
	}
	if e.hasSample && f.Pose.Time <= e.latest.Time {
		return
	}
	// 追踪恢复：记录恢复窗口起点姿态，避免输出跳变。
	if e.tracking == TrackingStateNotTracked && e.hasSample && it.cfg.Predict.Enabled {
		e.recovering = true
		e.recoveryFrom = it.predictor.predict(e.latest, f.Pose.Time).Pose
		e.recoveryStart = f.Pose.Time
	}
	e.tracking = TrackingStateTracked
	e.latest = f.Pose
	e.hasSample = true
}

// RelocateSpace 调整参考空间原点。
// 调整本地空间时同步更新传感器校正，保证设备世界姿态连续、不跳变；
// 已锚定内容的世界姿态由 SpaceManager 保证不变。
func (it *Integrator) RelocateSpace(id SpaceID, newOrigin Pose) {
	it.mu.Lock()
	defer it.mu.Unlock()
	if id == SpaceLocal {
		old := it.spaces.Origin(SpaceLocal)
		it.correction = newOrigin.Inverse().Compose(old).Compose(it.correction)
	}
	it.spaces.Relocate(id, newOrigin)
}

// Snapshot 返回当前时刻在指定参考空间下的一致快照。
// 快照内所有设备基于同一时刻与同一空间换算，不会新旧混杂。
func (it *Integrator) Snapshot(space SpaceID, now int64) Snapshot {
	it.mu.Lock()
	entries := make([]*deviceEntry, 0, len(it.devices))
	for _, e := range it.devices {
		if e.attached {
			entries = append(entries, e)
		}
	}
	sort.Slice(entries, func(i, j int) bool { return entries[i].info.UID < entries[j].info.UID })

	out := Snapshot{Time: now, Space: space}
	for _, e := range entries {
		ds := DeviceState{
			Info:     e.info,
			Attached: true,
			Tracking: e.tracking,
		}
		if e.tracking == TrackingStateTracked && e.hasSample {
			p := it.predictor.predict(e.latest, now)
			if e.recovering {
				blend := it.cfg.Predict.RecoveryBlend
				t := float64(now-e.recoveryStart) / 1e9 / blend
				if blend <= 0 || t >= 1 {
					e.recovering = false
				} else if t > 0 {
					p.Pose = LerpPose(e.recoveryFrom, p.Pose, t)
				} else {
					p.Pose = e.recoveryFrom
				}
			}
			ds.Pose = it.toSpace(p, space)
		}
		// 追踪丢失：Pose 保持零值，以 Tracking 标志区分无效结果。
		out.Devices = append(out.Devices, ds)
	}
	it.mu.Unlock()
	return out
}

// toSpace 把本地空间下的（预测后）姿态换算到目标空间，
// 位置与朝向做完整刚体变换，速度仅做旋转变换。
func (it *Integrator) toSpace(p TimedPose, space SpaceID) TimedPose {
	// 组合旋转：target^-1 * origin[local] * correction。
	localToWorld := it.spaces.Origin(SpaceLocal)
	worldToTarget := it.spaces.Origin(space).Inverse()
	combined := worldToTarget.Compose(localToWorld).Compose(it.correction)
	return TimedPose{
		Pose: combined.Compose(p.Pose),
		Velocity: Velocity{
			Linear:  combined.Orientation.Rotate(p.Velocity.Linear),
			Angular: combined.Orientation.Rotate(p.Velocity.Angular),
		},
		Time: p.Time,
	}
}
