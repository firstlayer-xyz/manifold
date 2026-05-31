package geom

import "math"

// float64Epsilon is std::numeric_limits<double>::epsilon(), used by the linalg
// quaternion routines for their near-degenerate guards.
const float64Epsilon = 2.220446049250313e-16

// --- Vec4 arithmetic (the quaternion + Bezier math needs these) ---

func (a Vec4) Add(b Vec4) Vec4 { return Vec4{X: a.X + b.X, Y: a.Y + b.Y, Z: a.Z + b.Z, W: a.W + b.W} }
func (a Vec4) Sub(b Vec4) Vec4 { return Vec4{X: a.X - b.X, Y: a.Y - b.Y, Z: a.Z - b.Z, W: a.W - b.W} }
func (a Vec4) Scale(s float64) Vec4 {
	return Vec4{X: a.X * s, Y: a.Y * s, Z: a.Z * s, W: a.W * s}
}
func (a Vec4) Dot(b Vec4) float64 { return a.X*b.X + a.Y*b.Y + a.Z*b.Z + a.W*b.W }
func (a Vec4) Length() float64    { return math.Sqrt(a.Dot(a)) }

// Normalize mirrors la::normalize for a vec4 (a / length(a)).
func (a Vec4) Normalize() Vec4 { return a.Scale(1 / a.Length()) }

// --- Quaternions (linalg.h). A quaternion is a Vec4 with X/Y/Z the vector part
// and W the scalar part. ---

// Qconj is the conjugate (linalg.h:1745): {-x, -y, -z, w}.
func Qconj(q Vec4) Vec4 { return Vec4{X: -q.X, Y: -q.Y, Z: -q.Z, W: q.W} }

// Qmul is the Hamilton product (linalg.h:1797).
func Qmul(a, b Vec4) Vec4 {
	return Vec4{
		X: a.X*b.W + a.W*b.X + a.Y*b.Z - a.Z*b.Y,
		Y: a.Y*b.W + a.W*b.Y + a.Z*b.X - a.X*b.Z,
		Z: a.Z*b.W + a.W*b.Z + a.X*b.Y - a.Y*b.X,
		W: a.W*b.W - a.X*b.X - a.Y*b.Y - a.Z*b.Z,
	}
}

// Qxdir is qrot(q, {1,0,0}) (linalg.h:1821) — the rotated x-axis.
func Qxdir(q Vec4) Vec3 { return qxdir(q) }

// RotationQuatFromTo is the Go port of la::rotation_quat(orig, dest)
// (linalg.h:2405): the quaternion rotating unit vector orig onto unit dest.
func RotationQuatFromTo(orig, dest Vec3) Vec4 {
	cosTheta := orig.Dot(dest)
	if cosTheta >= 1-float64Epsilon {
		return Vec4{W: 1}
	}
	if cosTheta < -1+float64Epsilon {
		axis := Vec3{X: 0, Y: 0, Z: 1}.Cross(orig)
		if axis.Dot(axis) < float64Epsilon {
			axis = Vec3{X: 1, Y: 0, Z: 0}.Cross(orig)
		}
		return RotationQuat(axis.Normalize(), math.Pi)
	}
	axis := orig.Cross(dest)
	s := math.Sqrt((1 + cosTheta) * 2)
	return Vec4{X: axis.X / s, Y: axis.Y / s, Z: axis.Z / s, W: s * 0.5}
}

// RotationQuatFromMat3 is the Go port of la::rotation_quat(mat3) (linalg.h:2424):
// the quaternion equivalent of a rotation matrix. Mat3 is column-major, so
// m[col][row]; the C++ m.x.y is column-0 row-1 = m[0][1].
func RotationQuatFromMat3(m Mat3) Vec4 {
	mxx, mxy, mxz := m[0][0], m[0][1], m[0][2]
	myx, myy, myz := m[1][0], m[1][1], m[1][2]
	mzx, mzy, mzz := m[2][0], m[2][1], m[2][2]

	q := Vec4{X: mxx - myy - mzz, Y: myy - mxx - mzz, Z: mzz - mxx - myy, W: mxx + myy + mzz}
	s := [4]Vec4{
		{X: 1, Y: mxy + myx, Z: mzx + mxz, W: myz - mzy},
		{X: mxy + myx, Y: 1, Z: myz + mzy, W: mzx - mxz},
		{X: mxz + mzx, Y: myz + mzy, Z: 1, W: mxy - myx},
		{X: myz - mzy, Y: mzx - mxz, Z: mxy - myx, W: 1},
	}
	qa := [4]float64{q.X, q.Y, q.Z, q.W}
	argmax := 0
	for i := 1; i < 4; i++ {
		if qa[i] > qa[argmax] {
			argmax = i
		}
	}
	r := Vec4{
		X: math.Sqrt(math.Max(0, 1+q.X)),
		Y: math.Sqrt(math.Max(0, 1+q.Y)),
		Z: math.Sqrt(math.Max(0, 1+q.Z)),
		W: math.Sqrt(math.Max(0, 1+q.W)),
	}.Normalize()
	sa := s[argmax]
	return Vec4{
		X: math.Copysign(r.X, sa.X),
		Y: math.Copysign(r.Y, sa.Y),
		Z: math.Copysign(r.Z, sa.Z),
		W: math.Copysign(r.W, sa.W),
	}
}
