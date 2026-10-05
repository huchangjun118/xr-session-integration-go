package xr

import (
	"math"
	"testing"
)

func approxEq(a, b, eps float64) bool { return math.Abs(a-b) <= eps }

func vecApprox(a, b Vec3, eps float64) bool {
	return approxEq(a.X, b.X, eps) && approxEq(a.Y, b.Y, eps) && approxEq(a.Z, b.Z, eps)
}

func TestQuaternionRoundTrip(t *testing.T) {
	poses := []Pose{
		{Position: Vec3{1, 2, 3}, Orientation: identityQuat()},
		{Position: Vec3{-0.5, 0, 2}, Orientation: lookQuat(0.4)},
		{Position: Vec3{0, 0, 0}, Orientation: lookQuat(-0.9)},
	}
	for i, p := range poses {
		got := poseMultiply(poseInverse(p), p)
		if !vecApprox(got.Position, Vec3{}, 1e-9) {
			t.Fatalf("case %d: p^-1*p position = %+v, want zero", i, got.Position)
		}
		if !approxEq(got.Orientation.W, 1, 1e-9) {
			t.Fatalf("case %d: p^-1*p orientation = %+v, want identity", i, got.Orientation)
		}
	}
}

func TestSpaceSelfConsistent(t *testing.T) {
	physical := Pose{Position: Vec3{X: 1, Y: 2, Z: 3}, Orientation: lookQuat(0.2)}
	spaces := []ReferenceSpace{SpaceLocal, SpaceFloor, SpaceStage}
	for _, sp := range spaces {
		m := NewSpaceManager(sp)
		m.SetFloorHeight(1.5)
		m.AdjustStageOrigin(Pose{Position: Vec3{X: 0.3, Y: 1.5, Z: -0.2}, Orientation: lookQuat(0.1)})
		out := m.ToOutput(physical)
		back := m.FromOutput(out)
		if !vecApprox(back.Position, physical.Position, 1e-9) {
			t.Fatalf("space %s round-trip pos = %+v", sp, back.Position)
		}
	}

	// Explicit floor semantics: a physical point at floor level (y=1.5) must
	// appear at y=0 in floor space.
	m := NewSpaceManager(SpaceFloor)
	m.SetFloorHeight(1.5)
	got := m.ToOutput(Pose{Position: Vec3{Y: 1.5}, Orientation: identityQuat()})
	if !approxEq(got.Position.Y, 0, 1e-12) {
		t.Fatalf("floor y = %v, want 0", got.Position.Y)
	}
}

func TestStageAdjustKeepsAnchorPhysical(t *testing.T) {
	m := NewSpaceManager(SpaceStage)
	// Content authored at a fixed stage position before recenter.
	authored := Pose{Position: Vec3{X: 1, Y: 1, Z: -1}, Orientation: lookQuat(0.3)}
	baseBefore := m.Anchor("content", authored)

	outBefore, _ := m.AnchorOutput("content")
	if !vecApprox(outBefore.Position, authored.Position, 1e-9) {
		t.Fatalf("before recenter: %+v", outBefore.Position)
	}

	// Recenter the stage origin. The anchor's physical pose must not move.
	m.AdjustStageOrigin(Pose{Position: Vec3{X: 2, Y: 0, Z: 1}, Orientation: identityQuat()})

	baseAfter, ok := m.AnchorBase("content")
	if !ok {
		t.Fatal("anchor lost")
	}
	if !vecApprox(baseAfter.Position, baseBefore.Position, 1e-12) {
		t.Fatalf("anchor physically jumped: before %+v after %+v", baseBefore.Position, baseAfter.Position)
	}
	outAfter, _ := m.AnchorOutput("content")
	// Stage coordinates must shift by exactly the origin translation.
	want := vecSub(authored.Position, Vec3{X: 2, Y: 0, Z: 1})
	if !vecApprox(outAfter.Position, want, 1e-9) {
		t.Fatalf("after recenter: got %+v want %+v", outAfter.Position, want)
	}
}

func TestFloorChangeNoAnchorJump(t *testing.T) {
	m := NewSpaceManager(SpaceFloor)
	b0 := m.Anchor("a", Pose{Position: Vec3{Y: 1}, Orientation: identityQuat()})
	m.SetFloorHeight(0.7)
	b1, _ := m.AnchorBase("a")
	if !vecApprox(b0.Position, b1.Position, 1e-12) {
		t.Fatal("floor change moved physically anchored content")
	}
}
