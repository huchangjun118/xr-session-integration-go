package xr

import "math"

// Vec3 是三维向量，单位米。
type Vec3 struct {
	X, Y, Z float64
}

// Quat 是单位四元数 (x, y, z, w)。
type Quat struct {
	X, Y, Z, W float64
}

// Pose 表示一个刚体姿态：位置 + 朝向。
type Pose struct {
	Position    Vec3
	Orientation Quat
}

// Velocity 表示线速度与角速度（角速度为轴角表示，弧度/秒）。
type Velocity struct {
	Linear  Vec3
	Angular Vec3
}

// TimedPose 是带时间戳与速度的姿态采样，时间戳为单调时钟纳秒。
type TimedPose struct {
	Pose     Pose
	Velocity Velocity
	Time     int64
}

func IdentityPose() Pose { return Pose{Orientation: QuatIdentity()} }

func (a Vec3) Add(b Vec3) Vec3      { return Vec3{a.X + b.X, a.Y + b.Y, a.Z + b.Z} }
func (a Vec3) Sub(b Vec3) Vec3      { return Vec3{a.X - b.X, a.Y - b.Y, a.Z - b.Z} }
func (a Vec3) Scale(s float64) Vec3 { return Vec3{a.X * s, a.Y * s, a.Z * s} }
func (a Vec3) Dot(b Vec3) float64   { return a.X*b.X + a.Y*b.Y + a.Z*b.Z }
func (a Vec3) Cross(b Vec3) Vec3 {
	return Vec3{a.Y*b.Z - a.Z*b.Y, a.Z*b.X - a.X*b.Z, a.X*b.Y - a.Y*b.X}
}
func (a Vec3) Len() float64 { return math.Sqrt(a.Dot(a)) }
func (a Vec3) Normalize() Vec3 {
	l := a.Len()
	if l == 0 {
		return Vec3{}
	}
	return a.Scale(1 / l)
}
func LerpVec3(a, b Vec3, t float64) Vec3 { return a.Add(b.Sub(a).Scale(t)) }

func QuatIdentity() Quat { return Quat{W: 1} }

func QuatFromAxisAngle(axis Vec3, angle float64) Quat {
	n := axis.Normalize()
	h := angle / 2
	s := math.Sin(h)
	return Quat{X: n.X * s, Y: n.Y * s, Z: n.Z * s, W: math.Cos(h)}.Normalize()
}

func (q Quat) Normalize() Quat {
	l := math.Sqrt(q.X*q.X + q.Y*q.Y + q.Z*q.Z + q.W*q.W)
	if l == 0 {
		return QuatIdentity()
	}
	return Quat{q.X / l, q.Y / l, q.Z / l, q.W / l}
}

func (q Quat) Conjugate() Quat { return Quat{-q.X, -q.Y, -q.Z, q.W} }

func (q Quat) Mul(r Quat) Quat {
	return Quat{
		X: q.W*r.X + q.X*r.W + q.Y*r.Z - q.Z*r.Y,
		Y: q.W*r.Y - q.X*r.Z + q.Y*r.W + q.Z*r.X,
		Z: q.W*r.Z + q.X*r.Y - q.Y*r.X + q.Z*r.W,
		W: q.W*r.W - q.X*r.X - q.Y*r.Y - q.Z*r.Z,
	}
}

func (q Quat) Rotate(v Vec3) Vec3 {
	u := Vec3{q.X, q.Y, q.Z}
	s := q.W
	return u.Scale(2 * u.Dot(v)).
		Add(v.Scale(s*s - u.Dot(u))).
		Add(u.Cross(v).Scale(2 * s))
}

// Slerp 球面插值；处理近零夹角与最短路径。
func Slerp(a, b Quat, t float64) Quat {
	dot := a.X*b.X + a.Y*b.Y + a.Z*b.Z + a.W*b.W
	if dot < 0 {
		b = Quat{-b.X, -b.Y, -b.Z, -b.W}
		dot = -dot
	}
	const threshold = 0.9995
	if dot > threshold {
		return Quat{
			X: a.X + t*(b.X-a.X),
			Y: a.Y + t*(b.Y-a.Y),
			Z: a.Z + t*(b.Z-a.Z),
			W: a.W + t*(b.W-a.W),
		}.Normalize()
	}
	dot = math.Min(1, math.Max(-1, dot))
	theta := math.Acos(dot)
	sin := math.Sin(theta)
	wa := math.Sin((1-t)*theta) / sin
	wb := math.Sin(t*theta) / sin
	return Quat{
		X: wa*a.X + wb*b.X,
		Y: wa*a.Y + wb*b.Y,
		Z: wa*a.Z + wb*b.Z,
		W: wa*a.W + wb*b.W,
	}.Normalize()
}

// Compose 返回先应用 b 再应用 a 的组合姿态（a * b）。
func (a Pose) Compose(b Pose) Pose {
	return Pose{
		Position:    a.Position.Add(a.Orientation.Rotate(b.Position)),
		Orientation: a.Orientation.Mul(b.Orientation).Normalize(),
	}
}

// Inverse 返回姿态的逆变换。
func (a Pose) Inverse() Pose {
	inv := a.Orientation.Conjugate().Normalize()
	return Pose{
		Position:    inv.Rotate(a.Position.Scale(-1)),
		Orientation: inv,
	}
}

// TransformPose 将姿态 p 从本姿态定义的子空间变换到父空间。
func (a Pose) TransformPose(p Pose) Pose { return a.Compose(p) }

// LerpPose 在两个姿态间插值，t ∈ [0,1]。
func LerpPose(a, b Pose, t float64) Pose {
	return Pose{
		Position:    LerpVec3(a.Position, b.Position, t),
		Orientation: Slerp(a.Orientation, b.Orientation, t),
	}
}

// Integrate 以速度外推 dt 秒后的姿态。
func (p TimedPose) Integrate(dt float64) Pose {
	pos := p.Pose.Position.Add(p.Velocity.Linear.Scale(dt))
	ang := p.Velocity.Angular
	angle := ang.Len() * dt
	var rot Quat
	if angle > 0 {
		rot = QuatFromAxisAngle(ang.Normalize(), angle)
	} else {
		rot = QuatIdentity()
	}
	return Pose{
		Position:    pos,
		Orientation: rot.Mul(p.Pose.Orientation).Normalize(),
	}
}
