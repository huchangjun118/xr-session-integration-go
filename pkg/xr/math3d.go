package xr

import "math"

// Math helpers for poses, quaternions and reference-space transforms.
//
// All angle math uses the shortest-arc convention: slerp normalizes its
// inputs and flips antipodal quaternions so interpolation never takes the
// long way around the sphere.

func identityQuat() Quat { return Quat{W: 1} }

func vecAdd(a, b Vec3) Vec3 { return Vec3{X: a.X + b.X, Y: a.Y + b.Y, Z: a.Z + b.Z} }

func vecSub(a, b Vec3) Vec3 { return Vec3{X: a.X - b.X, Y: a.Y - b.Y, Z: a.Z - b.Z} }

func vecScale(a Vec3, s float64) Vec3 { return Vec3{X: a.X * s, Y: a.Y * s, Z: a.Z * s} }

func vecLerp(a, b Vec3, t float64) Vec3 {
	return Vec3{
		X: a.X + (b.X-a.X)*t,
		Y: a.Y + (b.Y-a.Y)*t,
		Z: a.Z + (b.Z-a.Z)*t,
	}
}

func vecDot(a, b Vec3) float64 { return a.X*b.X + a.Y*b.Y + a.Z*b.Z }

func vecCross(a, b Vec3) Vec3 {
	return Vec3{
		X: a.Y*b.Z - a.Z*b.Y,
		Y: a.Z*b.X - a.X*b.Z,
		Z: a.X*b.Y - a.Y*b.X,
	}
}

func quatDot(a, b Quat) float64 { return a.X*b.X + a.Y*b.Y + a.Z*b.Z + a.W*b.W }

func quatNormalize(q Quat) Quat {
	n := math.Sqrt(quatDot(q, q))
	if n < 1e-12 {
		return identityQuat()
	}
	return Quat{X: q.X / n, Y: q.Y / n, Z: q.Z / n, W: q.W / n}
}

// quatRotate returns q applied to vector v.
func quatRotate(q Quat, v Vec3) Vec3 {
	// v' = v + 2w (u x v) + 2 (u x (u x v))
	u := Vec3{X: q.X, Y: q.Y, Z: q.Z}
	c := vecCross(u, v)
	t := vecAdd(vecScale(c, 2*q.W), vecScale(vecCross(u, c), 2))
	return vecAdd(v, t)
}

func quatMultiply(a, b Quat) Quat {
	return Quat{
		W: a.W*b.W - a.X*b.X - a.Y*b.Y - a.Z*b.Z,
		X: a.W*b.X + a.X*b.W + a.Y*b.Z - a.Z*b.Y,
		Y: a.W*b.Y - a.X*b.Z + a.Y*b.W + a.Z*b.X,
		Z: a.W*b.Z + a.X*b.Y - a.Y*b.X + a.Z*b.W,
	}
}

// quatInverse returns the inverse of a unit quaternion (its conjugate).
func quatInverse(q Quat) Quat {
	n := quatDot(q, q)
	if n < 1e-12 {
		return identityQuat()
	}
	inv := 1 / n
	return Quat{X: -q.X * inv, Y: -q.Y * inv, Z: -q.Z * inv, W: q.W * inv}
}

func quatSlerp(a, b Quat, t float64) Quat {
	a = quatNormalize(a)
	b = quatNormalize(b)
	d := quatDot(a, b)
	if d < 0 {
		// shortest arc
		b = Quat{X: -b.X, Y: -b.Y, Z: -b.Z, W: -b.W}
		d = -d
	}
	if d > 0.9995 {
		// nearly parallel: normalized linear interpolation
		r := Quat{
			X: a.X + (b.X-a.X)*t,
			Y: a.Y + (b.Y-a.Y)*t,
			Z: a.Z + (b.Z-a.Z)*t,
			W: a.W + (b.W-a.W)*t,
		}
		return quatNormalize(r)
	}
	theta0 := math.Acos(math.Min(1, math.Max(-1, d)))
	theta := theta0 * t
	sinTheta := math.Sin(theta)
	sinTheta0 := math.Sin(theta0)
	s1 := math.Cos(theta) - d*sinTheta/sinTheta0
	s2 := sinTheta / sinTheta0
	return quatNormalize(Quat{
		X: s1*a.X + s2*b.X,
		Y: s1*a.Y + s2*b.Y,
		Z: s1*a.Z + s2*b.Z,
		W: s1*a.W + s2*b.W,
	})
}

// poseMultiply returns the composed rigid transform a applied after b:
// a ∘ b.
func poseMultiply(a, b Pose) Pose {
	return Pose{
		Position:    vecAdd(a.Position, quatRotate(a.Orientation, b.Position)),
		Orientation: quatMultiply(a.Orientation, b.Orientation),
	}
}

func poseInverse(p Pose) Pose {
	qInv := quatInverse(p.Orientation)
	return Pose{
		Position:    quatRotate(qInv, vecScale(p.Position, -1)),
		Orientation: qInv,
	}
}

// poseBetween converts a pose expressed in frame a into frame b, where aInB
// is the transform of frame a's origin/axes expressed in frame b.
func poseBetween(aInB Pose, p Pose) Pose { return poseMultiply(aInB, p) }
