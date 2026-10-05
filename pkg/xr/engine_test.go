package xr

import (
	"sync"
	"testing"
	"time"
)

func newTestEngine() *Engine {
	cfg := DefaultConfig()
	cfg.Space = SpaceLocal
	e := NewEngine(cfg)
	if err := e.Begin(); err != nil {
		panic(err)
	}
	if err := e.Focus(); err != nil {
		panic(err)
	}
	return e
}

func feed(e *Engine, key string, kind Kind, hand Handedness, ts time.Duration, p Pose, st TrackingState) SubmitStats {
	return e.Submit(FrameInput{
		DeviceKey: key, Source: kind, Hand: hand, Timestamp: ts,
		Pose: p, State: st,
	})
}

func TestMissingSourcesDoNotBreakOthers(t *testing.T) {
	e := newTestEngine()
	id := e.RegisterInputDevice("hmd", KindHeadset, HandNone, "sim:headset")
	// Only the headset exists; no controllers/hands/eyes.
	st := feed(e, "hmd", KindHeadset, HandNone, 10*time.Millisecond,
		Pose{Position: Vec3{Y: 1.6}, Orientation: identityQuat()}, TrackValid)
	if !st.Accepted {
		t.Fatal("headset frame rejected")
	}
	snap, ok := e.Advance(10 * time.Millisecond)
	if !ok {
		t.Fatal("snapshot not published")
	}
	if len(snap.Devices) != 1 || snap.Devices[0].Device.ID != id {
		t.Fatalf("snapshot devices = %d", len(snap.Devices))
	}
	if snap.Devices[0].Frame.State != TrackValid {
		t.Fatal("headset not valid while other sources absent")
	}
}

func TestHotplugStableIdentityBindingsAndNoMisroute(t *testing.T) {
	e := newTestEngine()
	id := e.RegisterInputDevice("ctl-r", KindController, HandRight, "sim:controller")
	e.SetBinding(id, "action", "grab")

	feed(e, "ctl-r", KindController, HandRight, 10*time.Millisecond,
		Pose{Position: Vec3{X: 0.2, Y: 1}}, TrackValid)

	// Remove at runtime.
	e.Detach(id)
	otherID := e.RegisterInputDevice("ctl-l", KindController, HandLeft, "sim:controller")
	feed(e, "ctl-l", KindController, HandLeft, 20*time.Millisecond,
		Pose{Position: Vec3{X: -0.2, Y: 1}}, TrackValid)

	// Frames arriving for the detached right controller must be dropped.
	st := feed(e, "ctl-r", KindController, HandRight, 20*time.Millisecond,
		Pose{Position: Vec3{X: 99}}, TrackValid)
	if st.Accepted || st.Online {
		t.Fatal("frame accepted while device detached")
	}

	snap, _ := e.Advance(20 * time.Millisecond)
	left := snapshotDevice(snap, otherID)
	if !left.Present || left.Frame.Pose.Position.X != -0.2 {
		t.Fatal("left controller data misrouted or missing")
	}
	right := snapshotDevice(snap, id)
	if right.Present && right.Frame.Pose.Position.X == 99 {
		t.Fatal("detached-device frame leaked into snapshot")
	}
	if right.Device.Online {
		t.Fatal("detached device shown online")
	}

	// Re-plug: stable identity, bindings intact.
	e.RegisterInputDevice("ctl-r", KindController, HandRight, "sim:controller")
	e.Attach(id)
	d, _ := e.devices.device(id)
	if !d.Online {
		t.Fatal("replug not online")
	}
	if v, ok := e.devices.binding(id, "action"); !ok || v != "grab" {
		t.Fatalf("binding lost across replug: %q ok=%v", v, ok)
	}
}

func TestOtherDevicesUnaffectedByHotplug(t *testing.T) {
	e := newTestEngine()
	hmd := e.RegisterInputDevice("hmd", KindHeadset, HandNone, "sim")
	ctl := e.RegisterInputDevice("ctl-l", KindController, HandLeft, "sim")
	feed(e, "hmd", KindHeadset, HandNone, 10*time.Millisecond, Pose{Position: Vec3{Y: 1.6}}, TrackValid)
	feed(e, "ctl-l", KindController, HandLeft, 10*time.Millisecond, Pose{Position: Vec3{X: -0.2, Y: 1}}, TrackValid)
	snap1, _ := e.Advance(10 * time.Millisecond)

	e.Detach(ctl)
	feed(e, "hmd", KindHeadset, HandNone, 20*time.Millisecond, Pose{Position: Vec3{Y: 1.61}}, TrackValid)
	snap2, _ := e.Advance(20 * time.Millisecond)

	h1 := snapshotDevice(snap1, hmd)
	h2 := snapshotDevice(snap2, hmd)
	if h1.Frame.Pose.Position.Y == h2.Frame.Pose.Position.Y {
		t.Fatal("headset did not keep updating after controller removal")
	}
	c2 := snapshotDevice(snap2, ctl)
	if c2.Device.Online {
		t.Fatal("controller should be offline")
	}
}

func TestSnapshotConsistentAcrossDevices(t *testing.T) {
	e := newTestEngine()
	e.RegisterInputDevice("hmd", KindHeadset, HandNone, "sim")
	e.RegisterInputDevice("ctl-l", KindController, HandLeft, "sim")
	feed(e, "hmd", KindHeadset, HandNone, 10*time.Millisecond, Pose{Position: Vec3{Y: 1.6}}, TrackValid)
	feed(e, "ctl-l", KindController, HandLeft, 10*time.Millisecond, Pose{Position: Vec3{X: -0.2, Y: 1}}, TrackValid)
	snap, _ := e.Advance(10 * time.Millisecond)
	base := snap.Time
	for _, ds := range snap.Devices {
		// Every device frame in this snapshot was evaluated at the same
		// output time: there must be no old/new mix.
		if ds.Frame.State == TrackValid && ds.Frame.Timestamp != base {
			t.Fatalf("device %s frame time %v != snapshot time %v", ds.Device.ID, ds.Frame.Timestamp, base)
		}
	}
}

func TestConcurrentReadersDontBlockOrTear(t *testing.T) {
	e := newTestEngine()
	keys := []struct {
		k   string
		knd Kind
		h   Handedness
	}{
		{"hmd", KindHeadset, HandNone},
		{"ctl-l", KindController, HandLeft},
		{"ctl-r", KindController, HandRight},
		{"hand-l", KindHand, HandLeft},
		{"hand-r", KindHand, HandRight},
		{"eyes", KindEyes, HandNone},
	}
	for _, d := range keys {
		e.RegisterInputDevice(d.k, d.knd, d.h, "sim")
	}

	stop := make(chan struct{})
	var wg sync.WaitGroup

	// Updater: push frames and publish snapshots.
	wg.Add(1)
	go func() {
		defer wg.Done()
		tick := 10 * time.Millisecond
		for i := 0; i < 400; i++ {
			now := time.Duration(i+1) * tick
			for _, d := range keys {
				feed(e, d.k, d.knd, d.h, now,
					Pose{Position: Vec3{Y: float64(i) * 0.001}, Orientation: identityQuat()}, TrackValid)
			}
			e.Advance(now)
		}
		close(stop)
	}()

	// Slow readers: retain and validate snapshots at leisure.
	for r := 0; r < 8; r++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			var lastIndex uint64
			for {
				select {
				case <-stop:
					return
				default:
				}
				s := e.Snapshot()
				if s.Index < lastIndex {
					t.Errorf("snapshot index went backwards")
					return
				}
				lastIndex = s.Index
				ys := map[float64]bool{}
				for _, ds := range s.Devices {
					if ds.Present {
						ys[ds.Frame.Pose.Position.Y] = true
					}
				}
				// A consistent snapshot must contain a single update value.
				if len(ys) > 1 {
					t.Errorf("torn snapshot: mixed values %v", ys)
					return
				}
				time.Sleep(time.Millisecond)
			}
		}()
	}
	wg.Wait()
}

func TestClockNeverMovesBackwards(t *testing.T) {
	e := newTestEngine()
	e.RegisterInputDevice("hmd", KindHeadset, HandNone, "sim")
	feed(e, "hmd", KindHeadset, HandNone, 10*time.Millisecond, Pose{Position: Vec3{Y: 1}}, TrackValid)
	s1, _ := e.Advance(20 * time.Millisecond)
	_, ok2 := e.Advance(10 * time.Millisecond)
	s2 := e.Snapshot()
	if ok2 {
		t.Fatal("backwards Advance must be rejected")
	}
	if s2.Index != s1.Index {
		t.Fatal("rejected Advance changed published snapshot")
	}
}

func snapshotDevice(s Snapshot, id DeviceID) DeviceSnapshot {
	for _, ds := range s.Devices {
		if ds.Device.ID == id {
			return ds
		}
	}
	return DeviceSnapshot{}
}
