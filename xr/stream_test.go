package xr

import (
	"reflect"
	"testing"
)

func TestSimulatedSourceDeterministic(t *testing.T) {
	cfg := SimSourceConfig{UID: "eye", Type: DeviceTypeEye, Seed: 99, RateHz: 30, Duration: 1.0, LossEvery: 0.4, LossLen: 0.1}
	a := Collect(NewSimulatedSource(cfg))
	b := Collect(NewSimulatedSource(cfg))
	if !reflect.DeepEqual(a, b) {
		t.Fatal("same seed produced different streams")
	}
	if len(a) != 30 {
		t.Fatalf("frame count = %d, want 30", len(a))
	}
	// 丢失窗口内必须出现 NotTracked 帧
	lost := 0
	for _, f := range a {
		if f.Tracking == TrackingStateNotTracked {
			lost++
		}
	}
	if lost == 0 {
		t.Fatal("expected tracking loss frames")
	}
	// 不同种子产出不同流
	c := Collect(NewSimulatedSource(SimSourceConfig{UID: "eye", Type: DeviceTypeEye, Seed: 100, RateHz: 30, Duration: 1.0}))
	if reflect.DeepEqual(a, c) {
		t.Fatal("different seeds produced identical streams")
	}
}

func TestPermuteAndDuplicateDeterministic(t *testing.T) {
	frames := Collect(NewSimulatedSource(SimSourceConfig{UID: "h", Type: DeviceTypeHead, Seed: 1, RateHz: 10, Duration: 1.0}))
	p1 := Permute(frames, 5)
	p2 := Permute(frames, 5)
	if !reflect.DeepEqual(p1, p2) {
		t.Fatal("permute not deterministic")
	}
	d1 := Duplicate(frames, 5, 0.5)
	d2 := Duplicate(frames, 5, 0.5)
	if !reflect.DeepEqual(d1, d2) {
		t.Fatal("duplicate not deterministic")
	}
	if len(d1) <= len(frames) {
		t.Fatal("duplicate did not add frames")
	}
}
