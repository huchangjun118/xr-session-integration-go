package xr

import (
	"reflect"
	"sync"
	"testing"
)

func mkFrame(uid DeviceUID, typ DeviceType, seq uint64, t int64, x float64, tr TrackingState) Frame {
	return Frame{
		UID:      uid,
		Type:     typ,
		Tracking: tr,
		Pose: TimedPose{
			Pose:     Pose{Position: Vec3{X: x, Y: 1.5}, Orientation: QuatIdentity()},
			Velocity: Velocity{Linear: Vec3{X: 1}},
			Time:     t,
		},
		Seq: seq,
	}
}

func newTestIntegrator(t *testing.T, cfg Config, uids ...DeviceUID) *Integrator {
	t.Helper()
	it := NewIntegrator(cfg)
	for _, u := range uids {
		it.Attach(DeviceInfo{UID: u, Type: DeviceTypeHead, Name: string(u)})
	}
	return it
}

func TestIngestOutOfOrderDuplicateLate(t *testing.T) {
	cfg := DefaultConfig()
	src := NewSimulatedSource(SimSourceConfig{
		UID: "head", Type: DeviceTypeHead, Seed: 42,
		RateHz: 90, Duration: 1.0, StartTime: 1_000_000_000,
	})
	frames := Collect(src)
	if len(frames) == 0 {
		t.Fatal("empty source")
	}

	// 基准：顺序输入
	it1 := newTestIntegrator(t, cfg, "head")
	for _, f := range frames {
		it1.Ingest(f)
	}
	const now = 2_000_000_000
	want := it1.Snapshot(SpaceLocal, now)

	// 乱序 + 重复输入
	mixed := Permute(Duplicate(frames, 7, 0.3), 13)
	it2 := newTestIntegrator(t, cfg, "head")
	for _, f := range mixed {
		it2.Ingest(f)
	}
	got := it2.Snapshot(SpaceLocal, now)

	if !reflect.DeepEqual(want, got) {
		t.Fatalf("out-of-order/duplicate ingest diverged:\nwant %+v\ngot  %+v", want.Devices[0], got.Devices[0])
	}
}

func TestLateFrameDoesNotOverwrite(t *testing.T) {
	cfg := DefaultConfig()
	it := newTestIntegrator(t, cfg, "head")
	it.Ingest(mkFrame("head", DeviceTypeHead, 0, 1_000_000_000, 1.0, TrackingStateTracked))
	it.Ingest(mkFrame("head", DeviceTypeHead, 1, 2_000_000_000, 2.0, TrackingStateTracked))
	// 迟到帧：序号更旧
	it.Ingest(mkFrame("head", DeviceTypeHead, 0, 1_000_000_000, 1.0, TrackingStateTracked))
	// 迟到帧：序号新但时间戳更旧
	it.Ingest(mkFrame("head", DeviceTypeHead, 2, 500_000_000, 0.5, TrackingStateTracked))

	snap := it.Snapshot(SpaceLocal, 2_000_000_000)
	d, _ := snap.Device("head")
	if d.Pose.Time != 2_000_000_000 || d.Pose.Pose.Position.X != 2.0 {
		t.Fatalf("late frame overwrote newer pose: %+v", d.Pose)
	}
}

func TestTrackingLossAndRecovery(t *testing.T) {
	cfg := DefaultConfig() // Horizon=0.05, RecoveryBlend=0.1
	it := newTestIntegrator(t, cfg, "hand")
	// 追踪中：t=0 与 t=1s
	it.Ingest(mkFrame("hand", DeviceTypeHand, 0, 0, 0.0, TrackingStateTracked))
	it.Ingest(mkFrame("hand", DeviceTypeHand, 1, 1_000_000_000, 1.0, TrackingStateTracked))
	// 追踪丢失
	it.Ingest(mkFrame("hand", DeviceTypeHand, 2, 1_100_000_000, 0, TrackingStateNotTracked))

	snap := it.Snapshot(SpaceLocal, 1_200_000_000)
	d, _ := snap.Device("hand")
	if d.Tracking != TrackingStateNotTracked {
		t.Fatalf("tracking = %s, want not-tracked", d.Tracking)
	}
	// 无效结果必须可区分：Tracking 标志为丢失，姿态不得被当作有效零值
	if d.Pose != (TimedPose{}) {
		t.Fatalf("lost tracking should not expose a pose: %+v", d.Pose)
	}

	// 恢复：原始数据发生跳变（x 从预测位置瞬移到 10）
	it.Ingest(mkFrame("hand", DeviceTypeHand, 3, 1_300_000_000, 10.0, TrackingStateTracked))

	// 恢复瞬间：输出应延续旧预测姿态（recoveryFrom），不得跳变到新姿态
	snap = it.Snapshot(SpaceLocal, 1_300_000_000)
	d, _ = snap.Device("hand")
	if d.Tracking != TrackingStateTracked {
		t.Fatal("should be tracked after recovery frame")
	}
	// 旧采样 t=1.0, v=1 → 预测被 Horizon=0.05 截断到 x=1.05
	if got := d.Pose.Pose.Position.X; got > 2.0 {
		t.Fatalf("pose jumped on recovery: x=%v", got)
	}
	// 恢复窗口结束后：输出收敛到新姿态
	snap = it.Snapshot(SpaceLocal, 1_300_000_000+200_000_000)
	d, _ = snap.Device("hand")
	if got := d.Pose.Pose.Position.X; got < 9.9 {
		t.Fatalf("pose did not converge after recovery blend: x=%v", got)
	}
}

func TestPredictionDisabledPassthrough(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Predict.Enabled = false
	it := newTestIntegrator(t, cfg, "head")
	it.Ingest(mkFrame("head", DeviceTypeHead, 0, 1_000_000_000, 1.0, TrackingStateTracked))

	snap := it.Snapshot(SpaceLocal, 5_000_000_000)
	d, _ := snap.Device("head")
	// 关闭预测：输出必须是未经处理的原始姿态
	if d.Pose.Time != 1_000_000_000 || d.Pose.Pose.Position.X != 1.0 {
		t.Fatalf("prediction disabled should pass raw pose through: %+v", d.Pose)
	}
}

func TestPredictionHorizonAndMonotonic(t *testing.T) {
	cfg := DefaultConfig() // Horizon 0.05
	it := newTestIntegrator(t, cfg, "head")
	it.Ingest(mkFrame("head", DeviceTypeHead, 0, 1_000_000_000, 1.0, TrackingStateTracked))

	// 步长内：时间戳与所用步长一致
	snap := it.Snapshot(SpaceLocal, 1_020_000_000)
	d, _ := snap.Device("head")
	if d.Pose.Time != 1_020_000_000 {
		t.Fatalf("pose time %d != requested %d", d.Pose.Time, 1_020_000_000)
	}
	if !almostEq(d.Pose.Pose.Position.X, 1.02, 1e-9) {
		t.Fatalf("predicted x = %v, want 1.02", d.Pose.Pose.Position.X)
	}
	// 超出 Horizon：截断且时间戳反映实际步长
	snap = it.Snapshot(SpaceLocal, 2_000_000_000)
	d, _ = snap.Device("head")
	if d.Pose.Time != 1_050_000_000 {
		t.Fatalf("clamped pose time = %d, want 1050000000", d.Pose.Time)
	}
	// 读取时间早于采样时间：不得倒退
	snap = it.Snapshot(SpaceLocal, 500_000_000)
	d, _ = snap.Device("head")
	if d.Pose.Time != 1_000_000_000 || d.Pose.Pose.Position.X != 1.0 {
		t.Fatalf("prediction went backward: %+v", d.Pose)
	}
}

func TestHotplug(t *testing.T) {
	cfg := DefaultConfig()
	it := newTestIntegrator(t, cfg, "head", "ctrl")
	it.SetBinding("ctrl", "hand", "right")

	it.Ingest(mkFrame("head", DeviceTypeHead, 0, 1_000_000_000, 1.0, TrackingStateTracked))
	it.Ingest(mkFrame("ctrl", DeviceTypeController, 0, 1_000_000_000, 0.5, TrackingStateTracked))

	// 移除手柄；移除期间到达的数据被丢弃，头显不受影响
	it.Detach("ctrl")
	it.Ingest(mkFrame("ctrl", DeviceTypeController, 1, 1_100_000_000, 0.6, TrackingStateTracked))
	it.Ingest(mkFrame("head", DeviceTypeHead, 1, 1_100_000_000, 1.1, TrackingStateTracked))

	snap := it.Snapshot(SpaceLocal, 1_100_000_000)
	if _, ok := snap.Device("ctrl"); ok {
		t.Fatal("detached device should not appear in snapshot")
	}
	head, _ := snap.Device("head")
	if head.Pose.Pose.Position.X != 1.1 {
		t.Fatalf("other device disturbed by hotplug: %+v", head.Pose)
	}

	// 重新接入：身份与绑定保留
	it.Attach(DeviceInfo{UID: "ctrl", Type: DeviceTypeController, Name: "ctrl"})
	if v, ok := it.Binding("ctrl", "hand"); !ok || v != "right" {
		t.Fatalf("binding lost after re-attach: %q %v", v, ok)
	}
	// 移除期间的数据不得错位：重新接入后只接受新帧
	it.Ingest(mkFrame("ctrl", DeviceTypeController, 2, 1_200_000_000, 0.7, TrackingStateTracked))
	snap = it.Snapshot(SpaceLocal, 1_200_000_000)
	ctrl, ok := snap.Device("ctrl")
	if !ok {
		t.Fatal("re-attached device missing")
	}
	if ctrl.Pose.Pose.Position.X != 0.7 {
		t.Fatalf("stale/misrouted data after re-attach: %+v", ctrl.Pose)
	}
	head, _ = snap.Device("head")
	// 头显最新采样 t=1.1s x=1.1，快照 t=1.2s 预测被 Horizon=0.05 截断到 x=1.15
	if !almostEq(head.Pose.Pose.Position.X, 1.15, 1e-9) || head.Pose.Time != 1_150_000_000 {
		t.Fatalf("head pose corrupted: %+v", head.Pose)
	}
}

func TestMissingSourceDoesNotAffectOthers(t *testing.T) {
	cfg := DefaultConfig()
	// 只接入头显与手柄，没有手部/眼部来源
	it := newTestIntegrator(t, cfg, "head", "ctrl")
	it.Ingest(mkFrame("head", DeviceTypeHead, 0, 1_000_000_000, 1.0, TrackingStateTracked))
	it.Ingest(mkFrame("ctrl", DeviceTypeController, 0, 1_000_000_000, 0.5, TrackingStateTracked))
	snap := it.Snapshot(SpaceLocal, 1_000_000_000)
	if len(snap.Devices) != 2 {
		t.Fatalf("snapshot devices = %d, want 2", len(snap.Devices))
	}
	for _, d := range snap.Devices {
		if d.Tracking != TrackingStateTracked {
			t.Fatalf("device %s not tracked", d.Info.UID)
		}
	}
}

func TestConcurrentSnapshotConsistent(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Predict.Enabled = false // 原始姿态，便于校验一致性
	it := newTestIntegrator(t, cfg, "a", "b")

	var wg sync.WaitGroup
	stop := make(chan struct{})
	// 慢读取方：不得阻塞更新，且每个快照内部一致
	for i := 0; i < 4; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			prev := int64(-1)
			for {
				select {
				case <-stop:
					return
				default:
				}
				snap := it.Snapshot(SpaceLocal, 1_000_000_000)
				if len(snap.Devices) != 2 {
					t.Errorf("snapshot has %d devices", len(snap.Devices))
					return
				}
				// 一致性：两设备采样时间戳构成合法切片（a 先更新，故 b<=a）
				a, _ := snap.Device("a")
				b, _ := snap.Device("b")
				if a.Tracking == TrackingStateTracked && b.Tracking == TrackingStateTracked {
					if b.Pose.Time > a.Pose.Time {
						t.Errorf("inconsistent cut: b(%d) newer than a(%d)", b.Pose.Time, a.Pose.Time)
						return
					}
					if a.Pose.Time < prev {
						t.Errorf("snapshot went backward")
						return
					}
					prev = a.Pose.Time
				}
			}
		}()
	}
	// 更新方：a 总是在 b 之前更新
	for i := int64(0); i < 2000; i++ {
		ts := i * 1_000_000
		it.Ingest(mkFrame("a", DeviceTypeHead, uint64(i), ts, float64(i), TrackingStateTracked))
		it.Ingest(mkFrame("b", DeviceTypeHead, uint64(i), ts, float64(i), TrackingStateTracked))
	}
	close(stop)
	wg.Wait()
}

func TestDeterministicReplay(t *testing.T) {
	cfg := DefaultConfig()
	run := func() []Snapshot {
		it := NewIntegrator(cfg)
		it.Attach(DeviceInfo{UID: "head", Type: DeviceTypeHead})
		it.Attach(DeviceInfo{UID: "ctrl", Type: DeviceTypeController})
		it.Spaces().SetOrigin(SpaceStage, Pose{Position: Vec3{1, 0, 2}, Orientation: QuatFromAxisAngle(Vec3{Y: 1}, 0.3)})
		srcs := []Source{
			NewSimulatedSource(SimSourceConfig{UID: "head", Type: DeviceTypeHead, Seed: 1, RateHz: 90, Duration: 0.5, LossEvery: 0.3, LossLen: 0.05}),
			NewSimulatedSource(SimSourceConfig{UID: "ctrl", Type: DeviceTypeController, Seed: 2, RateHz: 60, Duration: 0.5}),
		}
		// 确定性交错：按时间戳归并
		var frames []Frame
		for _, s := range srcs {
			frames = append(frames, Collect(s)...)
		}
		sortFrames(frames)
		for _, f := range frames {
			it.Ingest(f)
		}
		var snaps []Snapshot
		for i := int64(0); i < 10; i++ {
			snaps = append(snaps, it.Snapshot(SpaceStage, i*50_000_000))
		}
		return snaps
	}
	first := run()
	second := run()
	if !reflect.DeepEqual(first, second) {
		t.Fatal("same input + seed + config produced different outputs")
	}
}

func sortFrames(frames []Frame) {
	for i := 1; i < len(frames); i++ {
		for j := i; j > 0 && frames[j].Pose.Time < frames[j-1].Pose.Time; j-- {
			frames[j], frames[j-1] = frames[j-1], frames[j]
		}
	}
}

func TestRecenterKeepsWorldPoseContinuous(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Predict.Enabled = false
	it := newTestIntegrator(t, cfg, "head")
	it.Spaces().SetOrigin(SpaceStage, Pose{Position: Vec3{2, 0, 1}, Orientation: QuatFromAxisAngle(Vec3{Y: 1}, 0.8)})
	it.Ingest(mkFrame("head", DeviceTypeHead, 0, 1_000_000_000, 1.0, TrackingStateTracked))

	before := it.Snapshot(SpaceStage, 1_000_000_000)
	// 重定位本地空间原点（重居中）
	it.RelocateSpace(SpaceLocal, Pose{Position: Vec3{3, 0, -2}, Orientation: QuatFromAxisAngle(Vec3{Y: 1}, 1.2)})
	after := it.Snapshot(SpaceStage, 1_000_000_000)

	bp, _ := before.Device("head")
	ap, _ := after.Device("head")
	if !poseAlmost(bp.Pose.Pose, ap.Pose.Pose, 1e-9) {
		t.Fatalf("world pose jumped after recenter:\nbefore %+v\nafter  %+v", bp.Pose.Pose, ap.Pose.Pose)
	}
}

func TestConfigCompatibility(t *testing.T) {
	a := DefaultConfig()
	b := DefaultConfig()
	if !a.CompatibleWith(b) {
		t.Fatal("identical configs must be compatible")
	}
	b.Predict.Horizon = 0.1
	if a.CompatibleWith(b) {
		t.Fatal("different horizon must be incompatible")
	}
	b = DefaultConfig()
	b.Version = "v2"
	if a.CompatibleWith(b) {
		t.Fatal("different version must be incompatible")
	}
}
