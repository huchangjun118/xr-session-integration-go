package xr

import (
	"math"
	"testing"
)

func almostEq(a, b, eps float64) bool { return math.Abs(a-b) <= eps }

func vecAlmost(a, b Vec3, eps float64) bool {
	return almostEq(a.X, b.X, eps) && almostEq(a.Y, b.Y, eps) && almostEq(a.Z, b.Z, eps)
}

func quatAlmost(a, b Quat, eps float64) bool {
	// 四元数符号等价
	d1 := math.Abs(a.X-b.X) + math.Abs(a.Y-b.Y) + math.Abs(a.Z-b.Z) + math.Abs(a.W-b.W)
	d2 := math.Abs(a.X+b.X) + math.Abs(a.Y+b.Y) + math.Abs(a.Z+b.Z) + math.Abs(a.W+b.W)
	return math.Min(d1, d2) <= eps
}

func poseAlmost(a, b Pose, eps float64) bool {
	return vecAlmost(a.Position, b.Position, eps) && quatAlmost(a.Orientation, b.Orientation, eps)
}

func TestPoseComposeInverseRoundTrip(t *testing.T) {
	a := Pose{Position: Vec3{1, 2, 3}, Orientation: QuatFromAxisAngle(Vec3{Y: 1}, 0.7)}
	b := Pose{Position: Vec3{-0.5, 0.2, 1.1}, Orientation: QuatFromAxisAngle(Vec3{X: 1, Z: 1}, -1.2)}
	got := a.Compose(b).Compose(b.Inverse())
	if !poseAlmost(got, a, 1e-9) {
		t.Fatalf("compose/inverse round trip failed: got %+v want %+v", got, a)
	}
	inv := a.Compose(a.Inverse())
	if !poseAlmost(inv, IdentityPose(), 1e-9) {
		t.Fatalf("a * a^-1 != identity: %+v", inv)
	}
}

func TestQuatRotateMatchesAxisAngle(t *testing.T) {
	q := QuatFromAxisAngle(Vec3{Y: 1}, math.Pi/2)
	v := q.Rotate(Vec3{X: 1})
	if !vecAlmost(v, Vec3{Z: -1}, 1e-9) {
		t.Fatalf("rotate failed: %+v", v)
	}
}

func TestIntegrateConstantVelocity(t *testing.T) {
	tp := TimedPose{
		Pose:     Pose{Position: Vec3{1, 0, 0}, Orientation: QuatIdentity()},
		Velocity: Velocity{Linear: Vec3{X: 2}},
	}
	got := tp.Integrate(0.5)
	if !vecAlmost(got.Position, Vec3{X: 2}, 1e-9) {
		t.Fatalf("integrate position failed: %+v", got.Position)
	}
}

func TestSlerpEndpoints(t *testing.T) {
	a := QuatIdentity()
	b := QuatFromAxisAngle(Vec3{Z: 1}, 1.0)
	if !quatAlmost(Slerp(a, b, 0), a, 1e-9) || !quatAlmost(Slerp(a, b, 1), b, 1e-9) {
		t.Fatal("slerp endpoints mismatch")
	}
}
