package xr

import (
	"testing"
	"time"
)

func mkFrame(id DeviceID, ts time.Duration, y float64, st TrackingState) PoseFrame {
	return PoseFrame{DeviceID: id, Timestamp: ts, Pose: Pose{Position: Vec3{Y: y}, Orientation: identityQuat()}, State: st}
}

func TestBufferDuplicateAndOrdering(t *testing.T) {
	b := newPoseBuffer("d", 16, 500*time.Millisecond)
	must := func(f PoseFrame, want AcceptReason, wantOk bool) {
		ok, r := b.push(f)
		if ok != wantOk || r != want {
			t.Helper()
			t.Fatalf("frame ts=%v: got ok=%v reason=%d, want ok=%v reason=%d", f.Timestamp, ok, r, wantOk, want)
		}
	}
	must(mkFrame("d", 10*time.Millisecond, 1, TrackValid), AcceptOK, true)
	must(mkFrame("d", 20*time.Millisecond, 2, TrackValid), AcceptOK, true)
	// exact duplicate rejected
	must(mkFrame("d", 10*time.Millisecond, 9, TrackValid), RejectDuplicate, false)
	// late but within window -> inserted into history slot
	must(mkFrame("d", 15*time.Millisecond, 1.5, TrackValid), AcceptOK, true)
	// too old -> rejected
	must(mkFrame("d", 20*time.Millisecond-600*time.Millisecond, 0, TrackValid), RejectTooLate, false)

	// newest must still be the 20ms frame, untouched by the old ones.
	n, _ := b.newest()
	if n.Timestamp != 20*time.Millisecond || n.Pose.Position.Y != 2 {
		t.Fatalf("newest corrupted: %v y=%v", n.Timestamp, n.Pose.Position.Y)
	}
	// at() must return the frame in effect at the queried time.
	f, ok := b.at(16 * time.Millisecond)
	if !ok || f.Timestamp != 15*time.Millisecond {
		t.Fatalf("at(16ms) = %v ok=%v", f.Timestamp, ok)
	}
}

func TestTrackingLossInvalidNotZero(t *testing.T) {
	id := DeviceID("eyes")
	b := newPoseBuffer(id, 16, time.Second)
	_, _ = b.push(mkFrame(id, 10*time.Millisecond, 0.5, TrackValid))
	_, _ = b.push(mkFrame(id, 20*time.Millisecond, 0.6, TrackValid))
	// loss frame carrying last known good pose (as engine Submit would build)
	loss := mkFrame(id, 30*time.Millisecond, 0.6, TrackInvalid)
	_, _ = b.push(loss)
	n, _ := b.newest()
	if n.State != TrackInvalid {
		t.Fatalf("state = %s, want invalid", n.State)
	}
	if n.Pose.Position.Y == 0 || n.Pose.Position.Y != 0.6 {
		t.Fatalf("loss frame pose = %v, want last known good 0.6, never zero", n.Pose.Position.Y)
	}
	if n.HasVelocity {
		t.Fatal("invalid frame must not advertise velocity")
	}
}

func TestPredictionDisabledReturnsRaw(t *testing.T) {
	cfg := DefaultConfig()
	cfg.PredictionEnabled = false
	p := newPredictor(cfg)
	id := DeviceID("hmd")
	b := newPoseBuffer(id, 16, time.Second)
	raw := mkFrame(id, 20*time.Millisecond, 1.2, TrackValid)
	_, _ = b.push(raw)
	st := &predState{}
	r := p.evaluate(b, 50*time.Millisecond, st)
	// disabled: raw newest sample, timestamp equals the sample, not the target
	if r.frame.Timestamp != raw.Timestamp || r.frame.Pose.Position.Y != 1.2 {
		t.Fatalf("disabled output = ts %v y %v", r.frame.Timestamp, r.frame.Pose.Position.Y)
	}
	if r.predicted || r.interpolated {
		t.Fatal("disabled predictor must not predict/interpolate")
	}
}

func TestPredictionOutputTimestampAndMonotonic(t *testing.T) {
	cfg := DefaultConfig() // predict ahead 20ms
	p := newPredictor(cfg)
	id := DeviceID("hmd")
	b := newPoseBuffer(id, 64, time.Second)
	for i := 1; i <= 5; i++ {
		ts := time.Duration(i) * 20 * time.Millisecond
		_, _ = b.push(mkFrame(id, ts, float64(i)*0.1, TrackValid))
	}
	st := &predState{}
	var prev Poser
	first := true
	for tick := 100; tick <= 200; tick += 20 {
		target := time.Duration(tick)*time.Millisecond + cfg.PredictAhead
		r := p.evaluate(b, target, st)
		if r.outputTime != target {
			t.Fatalf("outputTime %v != target %v", r.outputTime, target)
		}
		if first {
			first = false
			prev = Poser(r.frame.Pose.Position)
			continue
		}
		if r.frame.Pose.Position.Y < prev.Y-1e-9 {
			t.Fatalf("prediction went backwards: %v -> %v", prev.Y, r.frame.Pose.Position.Y)
		}
		prev = Poser(r.frame.Pose.Position)
	}
}

// Poser avoids shadowing imports in the table above.
type Poser = Vec3

func TestLossDuringHorizonIsInvalid(t *testing.T) {
	cfg := DefaultConfig()
	p := newPredictor(cfg)
	id := DeviceID("eyes")
	b := newPoseBuffer(id, 64, time.Second)
	_, _ = b.push(mkFrame(id, 10*time.Millisecond, 0.5, TrackValid))
	_, _ = b.push(mkFrame(id, 20*time.Millisecond, 0.55, TrackInvalid))
	r := p.evaluate(b, 40*time.Millisecond, &predState{})
	if r.frame.State != TrackInvalid {
		t.Fatalf("state = %s, want invalid during loss", r.frame.State)
	}
	if r.frame.Pose.Position.Y != 0.5 {
		t.Fatalf("invalid predicted pose y = %v, want last good 0.5", r.frame.Pose.Position.Y)
	}
}

func TestInterpolationBetweenSamples(t *testing.T) {
	cfg := DefaultConfig()
	cfg.PredictionEnabled = true
	cfg.PredictAhead = 0 // query strictly inside known samples
	p := newPredictor(cfg)
	id := DeviceID("h")
	b := newPoseBuffer(id, 16, time.Second)
	_, _ = b.push(mkFrame(id, 10*time.Millisecond, 0, TrackValid))
	_, _ = b.push(mkFrame(id, 20*time.Millisecond, 10, TrackValid))
	r := p.evaluate(b, 15*time.Millisecond, &predState{})
	if !r.interpolated {
		t.Fatal("expected interpolated result")
	}
	if !approxEq(r.frame.Pose.Position.Y, 5, 1e-9) {
		t.Fatalf("interp y = %v, want 5", r.frame.Pose.Position.Y)
	}
}

func TestPauseDoesNotPublish(t *testing.T) {
	e := newTestEngine()
	e.RegisterInputDevice("hmd", KindHeadset, HandNone, "sim")
	feed(e, "hmd", KindHeadset, HandNone, 10*time.Millisecond, Pose{Position: Vec3{Y: 1}}, TrackValid)
	_, _ = e.Advance(10 * time.Millisecond)
	if err := e.Pause(); err != nil {
		t.Fatal(err)
	}
	feed(e, "hmd", KindHeadset, HandNone, 20*time.Millisecond, Pose{Position: Vec3{Y: 2}}, TrackValid)
	if _, ok := e.Advance(20 * time.Millisecond); ok {
		t.Fatal("paused session must not publish new snapshots")
	}
	if err := e.Resume(); err != nil {
		t.Fatal(err)
	}
	snap, ok := e.Advance(20 * time.Millisecond)
	if !ok {
		t.Fatal("resume should allow publishing again")
	}
	if snap.SessionState != StateRunning {
		t.Fatalf("state after resume = %s", snap.SessionState)
	}
}
