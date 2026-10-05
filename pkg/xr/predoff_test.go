package xr

import (
	"testing"
	"time"
)

func TestEnginePredictionOffDegradesToRaw(t *testing.T) {
	cfg := DefaultConfig()
	cfg.Space = SpaceLocal
	cfg.PredictionEnabled = false
	e := NewEngine(cfg)
	if err := e.Begin(); err != nil {
		t.Fatal(err)
	}
	if err := e.Focus(); err != nil {
		t.Fatal(err)
	}
	e.RegisterInputDevice("hmd", KindHeadset, HandNone, "sim")
	raw := Pose{Position: Vec3{X: 0.1, Y: 1.6, Z: -0.2}, Orientation: lookQuat(0.25)}
	st := e.Submit(FrameInput{DeviceKey: "hmd", Source: KindHeadset, Timestamp: 30 * time.Millisecond, Pose: raw, State: TrackValid})
	if !st.Accepted {
		t.Fatal("submit rejected")
	}
	snap, ok := e.Advance(50 * time.Millisecond)
	if !ok {
		t.Fatal("no snapshot")
	}
	ds := snapshotDevice(snap, "headset/0")
	// with prediction off, snapshot time must equal the raw sample time ...
	if snap.Time != 30*time.Millisecond {
		t.Fatalf("snapshot time = %v, want raw 30ms", snap.Time)
	}
	// ... and the pose must be the raw pose unchanged (identity local space).
	if !vecApprox(ds.Frame.Pose.Position, raw.Position, 1e-12) {
		t.Fatalf("pos = %+v, want raw %+v", ds.Frame.Pose.Position, raw.Position)
	}
}
