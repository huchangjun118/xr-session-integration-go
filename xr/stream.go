package xr

import "math"

// Frame 是来源上报的一帧原始追踪数据。
type Frame struct {
	UID      DeviceUID
	Type     DeviceType
	Tracking TrackingState
	Pose     TimedPose
	// Seq 是来源侧的单调序号，用于识别重复帧。
	Seq uint64
}

// Source 是进程内模拟输入流，按确定顺序产出帧。
type Source interface {
	// Next 返回下一帧；ok 为 false 表示流已耗尽。
	Next() (Frame, bool)
}

// SliceSource 回放固定帧序列，用于脚本化测试与复现。
type SliceSource struct {
	frames []Frame
	i      int
}

func NewSliceSource(frames []Frame) *SliceSource { return &SliceSource{frames: frames} }

func (s *SliceSource) Next() (Frame, bool) {
	if s.i >= len(s.frames) {
		return Frame{}, false
	}
	f := s.frames[s.i]
	s.i++
	return f, true
}

// splitmix64 是确定性伪随机源。
type splitmix64 struct{ state uint64 }

func (r *splitmix64) next() uint64 {
	r.state += 0x9E3779B97F4A7C15
	z := r.state
	z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9
	z = (z ^ (z >> 27)) * 0x94D049BB133111EB
	return z ^ (z >> 31)
}

func (r *splitmix64) float() float64 { return float64(r.next()>>11) / (1 << 53) }

// SimSourceConfig 描述一路模拟来源。
type SimSourceConfig struct {
	UID       DeviceUID
	Type      DeviceType
	Seed      uint64
	RateHz    float64
	Duration  float64 // 秒
	StartTime int64   // 首帧时间戳（纳秒）
	// LossEvery 与 LossLen 描述周期性追踪丢失窗口（秒），LossEvery<=0 表示不丢失。
	LossEvery float64
	LossLen   float64
}

// SimulatedSource 以固定种子生成确定性的模拟帧流：
// 相同配置与种子产出的帧序列逐条一致。
type SimulatedSource struct {
	cfg   SimSourceConfig
	rng   splitmix64
	seq   uint64
	count int
	// 轨迹参数，由种子决定。
	cx, cy, cz float64
	radius     float64
	angVel     float64
	phase      float64
}

func NewSimulatedSource(cfg SimSourceConfig) *SimulatedSource {
	s := &SimulatedSource{cfg: cfg}
	s.rng = splitmix64{state: cfg.Seed}
	s.cx = (s.rng.float() - 0.5) * 0.4
	s.cy = 1.2 + s.rng.float()*0.4
	s.cz = (s.rng.float() - 0.5) * 0.4
	s.radius = 0.05 + s.rng.float()*0.2
	s.angVel = 0.5 + s.rng.float()*1.5
	s.phase = s.rng.float() * 2 * math.Pi
	if cfg.RateHz > 0 {
		s.count = int(cfg.RateHz * cfg.Duration)
	}
	return s
}

func (s *SimulatedSource) Next() (Frame, bool) {
	if s.seq >= uint64(s.count) {
		return Frame{}, false
	}
	t := float64(s.seq) / s.cfg.RateHz
	ts := s.cfg.StartTime + int64(t*1e9)

	// 圆周轨迹 + 解析速度。
	ang := s.phase + s.angVel*t
	pos := Vec3{
		X: s.cx + s.radius*math.Cos(ang),
		Y: s.cy,
		Z: s.cz + s.radius*math.Sin(ang),
	}
	vel := Vec3{
		X: -s.radius * s.angVel * math.Sin(ang),
		Y: 0,
		Z: s.radius * s.angVel * math.Cos(ang),
	}
	orient := QuatFromAxisAngle(Vec3{Y: 1}, ang)

	tracking := TrackingStateTracked
	if s.cfg.LossEvery > 0 {
		cycle := math.Mod(t, s.cfg.LossEvery)
		if cycle >= s.cfg.LossEvery-s.cfg.LossLen {
			tracking = TrackingStateNotTracked
		}
	}

	f := Frame{
		UID:      s.cfg.UID,
		Type:     s.cfg.Type,
		Tracking: tracking,
		Pose: TimedPose{
			Pose:     Pose{Position: pos, Orientation: orient},
			Velocity: Velocity{Linear: vel},
			Time:     ts,
		},
		Seq: s.seq,
	}
	s.seq++
	return f, true
}

// Collect 耗尽一个来源，返回全部帧。
func Collect(src Source) []Frame {
	var out []Frame
	for {
		f, ok := src.Next()
		if !ok {
			return out
		}
		out = append(out, f)
	}
}

// Permute 以种子确定性打乱帧顺序，用于构造乱序输入。
func Permute(frames []Frame, seed uint64) []Frame {
	out := make([]Frame, len(frames))
	copy(out, frames)
	rng := splitmix64{state: seed}
	for i := len(out) - 1; i > 0; i-- {
		j := int(rng.next() % uint64(i+1))
		out[i], out[j] = out[j], out[i]
	}
	return out
}

// Duplicate 以种子确定性地复制部分帧（追加到末尾），用于构造重复输入。
func Duplicate(frames []Frame, seed uint64, ratio float64) []Frame {
	out := make([]Frame, len(frames))
	copy(out, frames)
	rng := splitmix64{state: seed}
	for _, f := range frames {
		if rng.float() < ratio {
			out = append(out, f)
		}
	}
	return out
}
