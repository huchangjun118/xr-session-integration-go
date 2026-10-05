package xr

import (
	"testing"
	"time"
)

func runRecorded(seed int64, mutate func(*Config)) (Manifest, []Snapshot) {
	cfg := DefaultConfig()
	cfg.Seed = seed
	if mutate != nil {
		mutate(&cfg)
	}
	e := NewEngine(cfg)
	sim := NewSimulator(DefaultSimConfig(seed))
	rec := NewRecorder()
	var snaps []Snapshot
	// Record via custom loop so tests can inspect snapshots.
	var lastTick time.Duration
	for {
		ev, ok := sim.Next()
		if !ok {
			break
		}
		applyEvent(e, sim, ev)
		t := ev.At
		for lastTick < t {
			lastTick += DefaultSimConfig(seed).Tick
			if snap, ok := e.Advance(lastTick); ok {
				rec.Record(snap)
				snaps = append(snaps, snap)
			}
		}
	}
	return rec.Manifest(cfg), snaps
}

func applyEvent(e *Engine, s *Simulator, ev Event) {
	switch ev.Kind {
	case EvSession:
		_ = e.Session().Apply(ev.Session, ev.At)
	case EvAttach:
		if ev.DeviceKey == "" {
			for _, d := range s.sources() {
				id := e.RegisterInputDevice(d.key, d.kind, d.hand, d.source)
				e.SetBinding(id, "bind", d.key+"-binding")
			}
		} else {
			d := s.defByKey(ev.DeviceKey)
			id := e.RegisterInputDevice(d.key, d.kind, d.hand, d.source)
			e.Attach(id)
		}
	case EvDetach:
		if d, ok := e.devices.lookupKey(ev.DeviceKey); ok {
			e.Detach(d.ID)
		}
	case EvFrame:
		e.Submit(ev.Input)
	case EvFloor:
		e.SetFloorHeight(ev.Y)
	case EvStage:
		e.AdjustStageOrigin(ev.Origin)
	}
}

func TestDeterministicReplay(t *testing.T) {
	m1, _ := runRecorded(42, nil)
	m2, _ := runRecorded(42, nil)
	if m1.Hash != m2.Hash {
		t.Fatalf("same seed/config produced different hashes:\n%s\n%s", m1.Hash, m2.Hash)
	}
	if d := CompareManifolds(m1, m2); !d.Reproducible {
		t.Fatalf("expected reproducible: %s", d.Reason)
	}

	// Different seed -> different, but each replay still self-consistent.
	m3, _ := runRecorded(7, nil)
	if d := CompareManifolds(m1, m3); d.Reproducible {
		t.Fatal("different seeds must not compare equal")
	}

	// Configuration change must be explicitly reported.
	m4, _ := runRecorded(42, func(c *Config) { c.PredictAhead = 40 * time.Millisecond })
	if d := CompareManifolds(m1, m4); d.Reproducible || d.Reason != "config_mismatch" {
		t.Fatalf("want config_mismatch, got %+v", d)
	}

	// Algorithm iteration -> explicit non-reproducible verdict.
	m5 := m1
	m5.AlgorithmVersion = "future"
	if d := CompareManifolds(m5, m2); d.Reproducible || d.Reason == "" {
		t.Fatal("algorithm mismatch must be reported with a reason")
	}
}

func TestSimulatedFaultsAreAbsorbed(t *testing.T) {
	// Fault injection changes nothing deterministically: two runs with faults
	// match, and without faults produce a different (but still stable) hash.
	withFaults, _ := runRecorded(42, nil)

	cfg := DefaultConfig()
	cfg.Seed = 42
	e := NewEngine(cfg)
	sc := DefaultSimConfig(42)
	sc.InjectFaults = false
	sim := NewSimulator(sc)
	rec := NewRecorder()
	if _, err := sim.Run(e, rec); err != nil {
		t.Fatal(err)
	}
	without := rec.Manifest(cfg)
	if d := CompareManifolds(withFaults, without); d.Reproducible {
		t.Fatal("faults should change the recorded stream")
	}
	// but still deterministic with itself
	again, _ := runRecorded(42, nil)
	if withFaults.Hash != again.Hash {
		t.Fatal("fault run not reproducible")
	}
}

func TestLossWindowInvalidAndRecoveryNoJump(t *testing.T) {
	_, snaps := runRecorded(42, nil)
	var invalidCount int
	var lastValidBefore, firstValidAfter *DeviceSnapshot
	var eyesID DeviceID = "eyes/0"

	for i := range snaps {
		ds := findDevice(snaps[i], eyesID)
		if ds == nil {
			continue
		}
		insideLoss := snaps[i].Time >= 845*time.Millisecond &&
			snaps[i].Time < 1000*time.Millisecond+DefaultConfig().PredictAhead-5*time.Millisecond
		if insideLoss {
			if ds.Present && ds.Frame.State == TrackValid {
				t.Fatalf("eyes marked valid during loss window at %v", snaps[i].Time)
			}
			if ds.Present && ds.Frame.State == TrackInvalid {
				invalidCount++
				// invalid frames must never carry a zero position
				if ds.Frame.Pose.Position == (Vec3{}) {
					t.Fatalf("invalid eyes pose is zero at %v", snaps[i].Time)
				}
			}
		}
		if ds.Present && ds.Frame.State == TrackValid {
			if snaps[i].Time < 800*time.Millisecond {
				lastValidBefore = ds
			} else if snaps[i].Time >= 1000*time.Millisecond && firstValidAfter == nil {
				firstValidAfter = ds
			}
		}
	}
	if invalidCount == 0 {
		t.Fatal("expected at least one invalid snapshot during the loss window")
	}
	if lastValidBefore == nil || firstValidAfter == nil {
		t.Fatal("could not locate valid frames around the loss window")
	}
	// Recovery must not jump: the first valid pose after re-acquisition must
	// be close to the last valid pose before loss (trajectory moves slowly).
	dx := firstValidAfter.Frame.Pose.Position.X - lastValidBefore.Frame.Pose.Position.X
	dy := firstValidAfter.Frame.Pose.Position.Y - lastValidBefore.Frame.Pose.Position.Y
	dz := firstValidAfter.Frame.Pose.Position.Z - lastValidBefore.Frame.Pose.Position.Z
	const tol = 0.15
	if abs64(dx) > tol || abs64(dy) > tol || abs64(dz) > tol {
		t.Fatalf("pose jumped on recovery: d=(%.3f,%.3f,%.3f)", dx, dy, dz)
	}
}

func TestHotplugEpisodeInStream(t *testing.T) {
	_, snaps := runRecorded(42, nil)
	var offlineSeen, onlineAgain bool
	var id DeviceID = "controller/right"
	for _, s := range snaps {
		ds := findDevice(s, id)
		if ds == nil {
			continue
		}
		if s.Time > 1220*time.Millisecond && s.Time < 1500*time.Millisecond {
			if ds.Device.Online {
				t.Fatalf("right controller online during detach at %v", s.Time)
			}
			offlineSeen = true
		}
		if s.Time > 1500*time.Millisecond && ds.Device.Online {
			if v, ok := ds.Device.Bindings["bind"]; !ok || v != "ctl-r-binding" {
				t.Fatalf("binding not preserved across replug: %q", v)
			}
			onlineAgain = true
		}
	}
	if !offlineSeen || !onlineAgain {
		t.Fatalf("hot-plug episode not observed: offline=%v again=%v", offlineSeen, onlineAgain)
	}
}

func findDevice(s Snapshot, id DeviceID) *DeviceSnapshot {
	for i := range s.Devices {
		if s.Devices[i].Device.ID == id {
			return &s.Devices[i]
		}
	}
	return nil
}

func abs64(v float64) float64 {
	if v < 0 {
		return -v
	}
	return v
}
