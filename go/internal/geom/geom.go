// Package geom defines the small geometric primitive types shared across the
// Go port. They live in an internal package so packages on either side of
// the cgo boundary (manifold, reference, bridge, future tests) can use the
// same struct without forcing import cycles.
//
// The public manifold package re-exports Vec3 etc. via type aliases.
package geom

import "math"

// Vec2 is a 2D vector of doubles. Mirrors C++ vec<double, 2>.
type Vec2 struct {
	X, Y float64
}

// Sub returns a - b. Mirrors la::vec2 subtraction.
func (a Vec2) Sub(b Vec2) Vec2 { return Vec2{X: a.X - b.X, Y: a.Y - b.Y} }

// Dot returns the dot product. Mirrors la::dot(vec2, vec2).
func (a Vec2) Dot(b Vec2) float64 { return a.X*b.X + a.Y*b.Y }

// Length returns |a|. Mirrors la::length(vec2).
func (a Vec2) Length() float64 { return math.Sqrt(a.X*a.X + a.Y*a.Y) }

// Add returns a + b.
func (a Vec2) Add(b Vec2) Vec2 { return Vec2{X: a.X + b.X, Y: a.Y + b.Y} }

// Scale returns a scaled by s.
func (a Vec2) Scale(s float64) Vec2 { return Vec2{X: a.X * s, Y: a.Y * s} }

// Min returns the component-wise minimum. Mirrors la::min(vec2, vec2).
func (a Vec2) Min(b Vec2) Vec2 { return Vec2{X: math.Min(a.X, b.X), Y: math.Min(a.Y, b.Y)} }

// Max returns the component-wise maximum. Mirrors la::max(vec2, vec2).
func (a Vec2) Max(b Vec2) Vec2 { return Vec2{X: math.Max(a.X, b.X), Y: math.Max(a.Y, b.Y)} }

// Abs returns the component-wise absolute value. Mirrors la::abs(vec2).
func (a Vec2) Abs() Vec2 { return Vec2{X: math.Abs(a.X), Y: math.Abs(a.Y)} }

// Normalize returns a / |a|. For a zero vector this yields NaN components
// (IEEE 754 0/0); mirrors la::normalize. Callers that need a guarded result
// use SafeNormalize.
func (a Vec2) Normalize() Vec2 {
	// Componentwise division, matching la::normalize = a / length(a).
	length := a.Length()
	return Vec2{X: a.X / length, Y: a.Y / length}
}

// SafeNormalize is the Go port of EarClip::SafeNormalize (src/polygon.cpp:524):
// normalize, but return the zero vector if the result's x is non-finite
// (NaN or ±Inf — i.e. a zero-length input).
func (a Vec2) SafeNormalize() Vec2 {
	n := a.Normalize()
	if !math.IsNaN(n.X) && !math.IsInf(n.X, 0) {
		return n
	}
	return Vec2{}
}

// Determinant2x2 is the Go port of src/polygon.cpp:39 determinant2x2:
// a.x*b.y - a.y*b.x. Hand-written (NOT routed through a 3D cross): the
// floating-point operation order is load-bearing for the triangulator's
// epsilon bands, and the C++ comment notes la::determinant is mis-optimized.
func Determinant2x2(a, b Vec2) float64 { return a.X*b.Y - a.Y*b.X }

// Rect is a 2D axis-aligned bounding box. Mirrors C++ manifold::Rect
// (include/manifold/common.h:442). The zero value is NOT empty — use
// EmptyRect for the infinity-initialized empty rectangle.
type Rect struct {
	Min, Max Vec2
}

// EmptyRect returns the default-constructed empty rectangle
// (min = +inf, max = -inf), matching C++ Rect().
func EmptyRect() Rect {
	inf := math.Inf(1)
	return Rect{Min: Vec2{X: inf, Y: inf}, Max: Vec2{X: -inf, Y: -inf}}
}

// NewRect returns the smallest rectangle containing a and b. Mirrors
// Rect(a, b) = {min(a,b), max(a,b)}.
func NewRect(a, b Vec2) Rect { return Rect{Min: a.Min(b), Max: a.Max(b)} }

// Size returns max - min.
func (r Rect) Size() Vec2 { return r.Max.Sub(r.Min) }

// Scale returns the absolute-largest coordinate of any contained point:
// max(|min|, |max|) over both axes. Mirrors Rect::Scale.
func (r Rect) Scale() float64 {
	absMax := r.Min.Abs().Max(r.Max.Abs())
	return math.Max(absMax.X, absMax.Y)
}

// Center returns 0.5*(max+min).
func (r Rect) Center() Vec2 { return r.Max.Add(r.Min).Scale(0.5) }

// Contains reports whether p is inside r (border included). Mirrors
// Rect::Contains via la::gequal on both bounds.
func (r Rect) Contains(p Vec2) bool {
	return p.X >= r.Min.X && p.Y >= r.Min.Y && r.Max.X >= p.X && r.Max.Y >= p.Y
}

// DoesOverlap reports whether r overlaps o (equality included).
func (r Rect) DoesOverlap(o Rect) bool {
	return r.Min.X <= o.Max.X && r.Min.Y <= o.Max.Y &&
		r.Max.X >= o.Min.X && r.Max.Y >= o.Min.Y
}

// Union returns the smallest rectangle containing r and p.
func (r Rect) Union(p Vec2) Rect {
	return Rect{Min: r.Min.Min(p), Max: r.Max.Max(p)}
}

// Vec3 is a 3D vector of doubles.
type Vec3 struct {
	X, Y, Z float64
}

// Box is an axis-aligned bounding box defined by its min and max corners.
type Box struct {
	Min, Max Vec3
}

// Sub returns a - b.
func (a Vec3) Sub(b Vec3) Vec3 { return Vec3{X: a.X - b.X, Y: a.Y - b.Y, Z: a.Z - b.Z} }

// Dot returns the dot product.
func (a Vec3) Dot(b Vec3) float64 { return a.X*b.X + a.Y*b.Y + a.Z*b.Z }

// Cross returns the cross product a × b.
func (a Vec3) Cross(b Vec3) Vec3 {
	return Vec3{
		X: a.Y*b.Z - a.Z*b.Y,
		Y: a.Z*b.X - a.X*b.Z,
		Z: a.X*b.Y - a.Y*b.X,
	}
}

// Add returns a + b.
func (a Vec3) Add(b Vec3) Vec3 { return Vec3{X: a.X + b.X, Y: a.Y + b.Y, Z: a.Z + b.Z} }

// Mul returns a component-wise multiplied by b.
func (a Vec3) Mul(b Vec3) Vec3 { return Vec3{X: a.X * b.X, Y: a.Y * b.Y, Z: a.Z * b.Z} }

// Scale returns a scaled by s.
func (a Vec3) Scale(s float64) Vec3 { return Vec3{X: a.X * s, Y: a.Y * s, Z: a.Z * s} }

// Abs returns the component-wise absolute value of a.
func (a Vec3) Abs() Vec3 {
	return Vec3{X: math.Abs(a.X), Y: math.Abs(a.Y), Z: math.Abs(a.Z)}
}

// MaxComponent returns the largest component of a.
func (a Vec3) MaxComponent() float64 {
	return math.Max(a.X, math.Max(a.Y, a.Z))
}

// Length returns the Euclidean magnitude of a.
func (a Vec3) Length() float64 { return math.Sqrt(a.Dot(a)) }

// Normalize returns a / |a|. For a zero vector this returns NaN
// components (IEEE 754 0/0); callers that care must guard against |a| == 0
// themselves (matching the C++ la::normalize behavior).
func (a Vec3) Normalize() Vec3 {
	// Componentwise division, matching la::normalize = a / length(a)
	// (linalg.h:1642). x/len is not bit-identical to x*(1/len) in IEEE-754.
	length := a.Length()
	return Vec3{X: a.X / length, Y: a.Y / length, Z: a.Z / length}
}

// Degrees converts radians to degrees. Mirrors C++ manifold::degrees.
func Degrees(rad float64) float64 { return rad * (180.0 / math.Pi) }

// Radians converts degrees to radians. Mirrors C++ manifold::radians.
func Radians(deg float64) float64 { return deg * (math.Pi / 180.0) }

// SafeNormalize is the Go port of C++ SafeNormalize from src/shared.h:
// normalize, but if the result is non-finite (zero-vector or NaN
// input), return the zero vector instead.
func (a Vec3) SafeNormalize() Vec3 {
	n := a.Normalize()
	if !n.IsFinite() {
		return Vec3{}
	}
	return n
}

// AngleBetween is the Go port of C++ AngleBetween from
// src/smoothing.cpp: returns the angle (in radians) between two
// unit-vectors a and b, clamping dot to [-1, 1] before acos so a
// slightly-out-of-range dot doesn't yield NaN.
func AngleBetween(a, b Vec3) float64 {
	d := a.Dot(b)
	if d >= 1 {
		return 0
	}
	if d <= -1 {
		return math.Pi
	}
	return math.Acos(d)
}

// IsFinite reports whether all components are finite.
func (a Vec3) IsFinite() bool {
	return !math.IsNaN(a.X) && !math.IsInf(a.X, 0) &&
		!math.IsNaN(a.Y) && !math.IsInf(a.Y, 0) &&
		!math.IsNaN(a.Z) && !math.IsInf(a.Z, 0)
}

func minVec(a, b Vec3) Vec3 {
	return Vec3{X: math.Min(a.X, b.X), Y: math.Min(a.Y, b.Y), Z: math.Min(a.Z, b.Z)}
}

func maxVec(a, b Vec3) Vec3 {
	return Vec3{X: math.Max(a.X, b.X), Y: math.Max(a.Y, b.Y), Z: math.Max(a.Z, b.Z)}
}

// NewBox returns the smallest Box containing both p1 and p2.
func NewBox(p1, p2 Vec3) Box {
	return Box{Min: minVec(p1, p2), Max: maxVec(p1, p2)}
}

// Size returns max - min.
func (b Box) Size() Vec3 { return b.Max.Sub(b.Min) }

// Center returns the midpoint of the box.
func (b Box) Center() Vec3 { return b.Max.Add(b.Min).Scale(0.5) }

// Scale returns the absolute-largest coordinate value of any contained point.
func (b Box) Scale() float64 {
	return maxVec(b.Min.Abs(), b.Max.Abs()).MaxComponent()
}

// Contains reports whether p lies inside the box (boundary included).
func (b Box) Contains(p Vec3) bool {
	return p.X >= b.Min.X && p.Y >= b.Min.Y && p.Z >= b.Min.Z &&
		p.X <= b.Max.X && p.Y <= b.Max.Y && p.Z <= b.Max.Z
}

// ContainsBox reports whether other lies entirely inside b (boundary included).
func (b Box) ContainsBox(other Box) bool {
	return other.Min.X >= b.Min.X && other.Min.Y >= b.Min.Y && other.Min.Z >= b.Min.Z &&
		other.Max.X <= b.Max.X && other.Max.Y <= b.Max.Y && other.Max.Z <= b.Max.Z
}

// DoesOverlap reports whether b and other share any interior or boundary point.
func (b Box) DoesOverlap(other Box) bool {
	return b.Min.X <= other.Max.X && b.Min.Y <= other.Max.Y && b.Min.Z <= other.Max.Z &&
		b.Max.X >= other.Min.X && b.Max.Y >= other.Min.Y && b.Max.Z >= other.Min.Z
}

// Union is the Go port of C++ Box::Union(const Box&) from
// include/manifold/common.h: smallest Box containing both b and other.
func (b Box) Union(other Box) Box {
	return Box{Min: minVec(b.Min, other.Min), Max: maxVec(b.Max, other.Max)}
}

// UnionPoint is the Go port of C++ Box::Union(const vec3) from
// include/manifold/common.h: expand b in place to include point p.
// Returns the updated Box (mirroring void return in C++ is awkward
// in Go; callers do `b = b.UnionPoint(p)`).
func (b Box) UnionPoint(p Vec3) Box {
	return Box{Min: minVec(b.Min, p), Max: maxVec(b.Max, p)}
}

// DoesOverlapPoint reports whether p lies inside b (boundary included).
// Mirrors C++ Box::DoesOverlap(const vec3&).
func (b Box) DoesOverlapPoint(p Vec3) bool { return b.Contains(p) }

// Transform is the Go port of C++ Box::Transform(const mat3x4&) from
// include/manifold/common.h: applies the affine transform to b's min
// and max corners, then takes the AABB of the two transformed points.
//
// This is a conservative bound — for non-axis-aligned rotations the
// result is larger than the tight AABB of the rotated box, but it's
// what the Collider uses since its callers guarantee axis-aligned
// transforms (Collider::IsAxisAligned).
func (b Box) Transform(m Mat3x4) Box {
	minT := m.ApplyAffine(b.Min)
	maxT := m.ApplyAffine(b.Max)
	return Box{Min: minVec(minT, maxT), Max: maxVec(minT, maxT)}
}

// IsFinite reports whether all corner coordinates are finite.
func (b Box) IsFinite() bool { return b.Min.IsFinite() && b.Max.IsFinite() }

// Translate returns b shifted by v.
func (b Box) Translate(v Vec3) Box { return Box{Min: b.Min.Add(v), Max: b.Max.Add(v)} }

// MulComponent returns b scaled component-wise by s.
func (b Box) MulComponent(s Vec3) Box { return Box{Min: b.Min.Mul(s), Max: b.Max.Mul(s)} }

// Mat3 is a 3x3 matrix in column-major order: Mat3[col][row]. Chosen to
// match the column-major convention used by the C++ linalg library so the
// port can mirror C++ matrix code structurally.
// Vec4 is a 4-component vector (mirrors C++ vec4 / glm::dvec4). Used for
// halfedge tangents stored as homogeneous-weighted Bezier control points
// (xyz = geometric tangent, W = the rational weight; W < 0 marks a quad interior).
type Vec4 struct{ X, Y, Z, W float64 }

// Vec3 returns the xyz components (mirrors C++ vec3(v4)).
func (a Vec4) Vec3() Vec3 { return Vec3{X: a.X, Y: a.Y, Z: a.Z} }

// Lerp is the component-wise linear interpolation, matching linalg's lerp:
// a*(1-t) + b*t (linalg.h:471), NOT the algebraically-equal a+(b-a)*t.
func (a Vec4) Lerp(b Vec4, t float64) Vec4 {
	return Vec4{
		X: a.X*(1-t) + b.X*t,
		Y: a.Y*(1-t) + b.Y*t,
		Z: a.Z*(1-t) + b.Z*t,
		W: a.W*(1-t) + b.W*t,
	}
}

// A Vec4 also represents a quaternion (x,y,z,w), as in linalg. qxdir/qydir/qzdir
// are the rotated basis vectors (linalg.h:1820/1828/1838), used by Qrot.
func qxdir(q Vec4) Vec3 {
	return Vec3{
		X: q.W*q.W + q.X*q.X - q.Y*q.Y - q.Z*q.Z,
		Y: (q.X*q.Y + q.Z*q.W) * 2,
		Z: (q.Z*q.X - q.Y*q.W) * 2,
	}
}

func qydir(q Vec4) Vec3 {
	return Vec3{
		X: (q.X*q.Y - q.Z*q.W) * 2,
		Y: q.W*q.W - q.X*q.X + q.Y*q.Y - q.Z*q.Z,
		Z: (q.Y*q.Z + q.X*q.W) * 2,
	}
}

func qzdir(q Vec4) Vec3 {
	return Vec3{
		X: (q.Z*q.X + q.Y*q.W) * 2,
		Y: (q.Y*q.Z - q.X*q.W) * 2,
		Z: q.W*q.W - q.X*q.X - q.Y*q.Y + q.Z*q.Z,
	}
}

// Qrot rotates v by the quaternion q (la::qrot, linalg.h:1853):
// qxdir(q)*v.x + qydir(q)*v.y + qzdir(q)*v.z.
func Qrot(q Vec4, v Vec3) Vec3 {
	return qxdir(q).Scale(v.X).Add(qydir(q).Scale(v.Y)).Add(qzdir(q).Scale(v.Z))
}

// RotationQuat returns the normalized quaternion for a rotation of angle
// radians about axis (la::rotation_quat, linalg.h:1892):
// {axis * sin(angle/2), cos(angle/2)}.
func RotationQuat(axis Vec3, angle float64) Vec4 {
	a := axis.Scale(math.Sin(angle / 2))
	return Vec4{X: a.X, Y: a.Y, Z: a.Z, W: math.Cos(angle / 2)}
}

type Mat3 [3][3]float64

// Mat3x4 is a 3-row, 4-column matrix in column-major order: Mat3x4[col][row].
// The four columns are typically the three basis vectors of an affine
// transform plus a translation column.
type Mat3x4 [4][3]float64

// Mul returns a * b (matrix multiplication, column-major).
func (a Mat3) Mul(b Mat3) Mat3 {
	var r Mat3
	for col := 0; col < 3; col++ {
		for row := 0; row < 3; row++ {
			r[col][row] = a[0][row]*b[col][0] + a[1][row]*b[col][1] + a[2][row]*b[col][2]
		}
	}
	return r
}

// MulVec3 applies the matrix to a column vector: returns a * v.
// In column-major M[col][row], result[row] = sum_col M[col][row] * v[col].
func (a Mat3) MulVec3(v Vec3) Vec3 {
	return Vec3{
		X: a[0][0]*v.X + a[1][0]*v.Y + a[2][0]*v.Z,
		Y: a[0][1]*v.X + a[1][1]*v.Y + a[2][1]*v.Z,
		Z: a[0][2]*v.X + a[1][2]*v.Y + a[2][2]*v.Z,
	}
}

// MulScalar scales every element by s, mirroring linalg's `mat3 * scalar`.
func (a Mat3) MulScalar(s float64) Mat3 {
	var r Mat3
	for col := 0; col < 3; col++ {
		for row := 0; row < 3; row++ {
			r[col][row] = a[col][row] * s
		}
	}
	return r
}

// Transpose returns aᵀ.
func (a Mat3) Transpose() Mat3 {
	return Mat3{
		{a[0][0], a[1][0], a[2][0]},
		{a[0][1], a[1][1], a[2][1]},
		{a[0][2], a[1][2], a[2][2]},
	}
}

// Determinant returns det(a).
func (a Mat3) Determinant() float64 {
	return a[0][0]*(a[1][1]*a[2][2]-a[2][1]*a[1][2]) -
		a[1][0]*(a[0][1]*a[2][2]-a[2][1]*a[0][2]) +
		a[2][0]*(a[0][1]*a[1][2]-a[1][1]*a[0][2])
}

// Inverse returns a⁻¹. Behavior on singular matrices follows IEEE 754:
// division by zero yields ±Inf or NaN components rather than an error.
// Ported from la::inverse via the adjugate / determinant formula.
func (a Mat3) Inverse() Mat3 {
	det := a.Determinant()
	invDet := 1.0 / det
	return Mat3{
		{
			(a[1][1]*a[2][2] - a[2][1]*a[1][2]) * invDet,
			-(a[0][1]*a[2][2] - a[2][1]*a[0][2]) * invDet,
			(a[0][1]*a[1][2] - a[1][1]*a[0][2]) * invDet,
		},
		{
			-(a[1][0]*a[2][2] - a[2][0]*a[1][2]) * invDet,
			(a[0][0]*a[2][2] - a[2][0]*a[0][2]) * invDet,
			-(a[0][0]*a[1][2] - a[1][0]*a[0][2]) * invDet,
		},
		{
			(a[1][0]*a[2][1] - a[2][0]*a[1][1]) * invDet,
			-(a[0][0]*a[2][1] - a[2][0]*a[0][1]) * invDet,
			(a[0][0]*a[1][1] - a[1][0]*a[0][1]) * invDet,
		},
	}
}

// NormalTransform returns the matrix that maps surface normals consistently
// with the points-mapping matrix `transform`: inv(transpose(mat3(transform))).
// Ported from C++ NormalTransform in src/shared.h.
func NormalTransform(transform Mat3x4) Mat3 {
	linear := Mat3{transform[0], transform[1], transform[2]}
	return linear.Transpose().Inverse()
}

// InverseNormalTransform is the inverse of NormalTransform —
// inv(transpose(inv(mat3(transform)))) — used by SetNormals to map
// world-frame vertex normals back into the per-mesh frame. Mirrors
// C++ InverseNormalTransform in src/shared.h.
func InverseNormalTransform(transform Mat3x4) Mat3 {
	linear := Mat3{transform[0], transform[1], transform[2]}
	return linear.Inverse().Transpose().Inverse()
}

// Sind returns sin(x) for x in degrees, exact at every multiple of 90.
// Ported from C++ sind() in common.h: reduce modulo 90 with quotient
// tracking, then pick the right ±sin/±cos based on which quadrant.
func Sind(x float64) float64 {
	if math.IsNaN(x) || math.IsInf(x, 0) {
		return math.NaN()
	}
	if x < 0 {
		return -Sind(-x)
	}
	rem := math.Remainder(x, 90.0)
	quo := int(math.Round((x - rem) / 90.0))
	xr := rem * (math.Pi / 180.0)
	switch quo & 3 {
	case 0:
		return math.Sin(xr)
	case 1:
		return math.Cos(xr)
	case 2:
		return -math.Sin(xr)
	case 3:
		return -math.Cos(xr)
	}
	return 0.0
}

// Cosd returns cos(x) for x in degrees, exact at every multiple of 90.
func Cosd(x float64) float64 { return Sind(x + 90.0) }

// NoMortonCode mirrors the C++ constexpr kNoCode in src/sort.cpp.
// Used to flag verts (or faces) for removal so they sort to the end.
const NoMortonCode uint32 = 0xFFFFFFFF

// spreadBits3 mirrors the constexpr inline of the same name in
// src/collider.h. Interleaves the low 10 bits of v into every third
// bit position, producing the per-axis component of a Morton code.
func spreadBits3(v uint32) uint32 {
	v = 0xFF0000FF & (v * 0x00010001)
	v = 0x0F00F00F & (v * 0x00000101)
	v = 0xC30C30C3 & (v * 0x00000011)
	v = 0x49249249 & (v * 0x00000005)
	return v
}

// MortonCode is the Go port of Collider::MortonCode and the wrapping
// helper in src/sort.cpp: returns NoMortonCode for any NaN coordinate,
// otherwise an interleaved 30-bit Z-curve index of position within
// bBox.
func MortonCode(position Vec3, bBox Box) uint32 {
	if math.IsNaN(position.X) || math.IsNaN(position.Y) || math.IsNaN(position.Z) {
		return NoMortonCode
	}
	dx := bBox.Max.X - bBox.Min.X
	dy := bBox.Max.Y - bBox.Min.Y
	dz := bBox.Max.Z - bBox.Min.Z
	x := 1024.0 * (position.X - bBox.Min.X) / dx
	y := 1024.0 * (position.Y - bBox.Min.Y) / dy
	z := 1024.0 * (position.Z - bBox.Min.Z) / dz
	x = math.Max(0, math.Min(1023, x))
	y = math.Max(0, math.Min(1023, y))
	z = math.Max(0, math.Min(1023, z))
	xb := spreadBits3(uint32(x))
	yb := spreadBits3(uint32(y))
	zb := spreadBits3(uint32(z))
	return xb*4 + yb*2 + zb
}

// ApplyAffine applies the 3x4 affine transform to a Vec3 with implicit
// w=1, returning the transformed Vec3. Mirrors C++ `m * vec4(v, 1.0)`
// for a la::mat<double,3,4>.
//
// In column-major storage, columns 0..2 are the linear part and column 3
// is the translation. So:
//
//	result = col0*v.x + col1*v.y + col2*v.z + col3
func (a Mat3x4) ApplyAffine(v Vec3) Vec3 {
	return Vec3{
		X: a[0][0]*v.X + a[1][0]*v.Y + a[2][0]*v.Z + a[3][0],
		Y: a[0][1]*v.X + a[1][1]*v.Y + a[2][1]*v.Z + a[3][1],
		Z: a[0][2]*v.X + a[1][2]*v.Y + a[2][2]*v.Z + a[3][2],
	}
}

// Mat2x3 mirrors C++ la::mat<double, 2, 3>: a 2x3 matrix stored as
// two rows of three doubles, applied to a Vec3 to produce a Vec2.
// Used by GetAxisAlignedProjection to project a 3D point onto a
// face-aligned 2D plane.
type Mat2x3 [2][3]float64

// MulVec3 applies the 2x3 matrix to a Vec3, returning a Vec2.
func (a Mat2x3) MulVec3(v Vec3) Vec2 {
	return Vec2{
		X: a[0][0]*v.X + a[0][1]*v.Y + a[0][2]*v.Z,
		Y: a[1][0]*v.X + a[1][1]*v.Y + a[1][2]*v.Z,
	}
}

// GetAxisAlignedProjection mirrors the C++ inline of the same name in
// src/shared.h. It builds a 2x3 matrix that projects 3D points onto
// the plane perpendicular to the dominant axis of `normal`, then
// orientation-flips the first row if the dominant axis is negative.
//
// The projection is axis-aligned (rather than basis-rotated) precisely
// so it introduces no rounding error: every output coordinate is one
// of the input vec3 components, possibly negated.
func GetAxisAlignedProjection(normal Vec3) Mat2x3 {
	absN := normal.Abs()
	var p [3][2]float64
	var xyzMax float64
	switch {
	case absN.Z > absN.X && absN.Z > absN.Y:
		// projection = {{1,0,0},{0,1,0}}, will be transposed below.
		p = [3][2]float64{{1, 0}, {0, 1}, {0, 0}}
		xyzMax = normal.Z
	case absN.Y > absN.X:
		// C++ mat3x2({0,0,1}, {1,0,0}) — col0 = (0,0,1), col1 = (1,0,0).
		// Stored row-by-row as (col0[row], col1[row]).
		p = [3][2]float64{{0, 1}, {0, 0}, {1, 0}}
		xyzMax = normal.Y
	default:
		p = [3][2]float64{{0, 0}, {1, 0}, {0, 1}}
		xyzMax = normal.X
	}
	if xyzMax < 0 {
		p[0][0] *= -1
		p[1][0] *= -1
		p[2][0] *= -1
	}
	// Transpose 3x2 -> 2x3.
	return Mat2x3{
		{p[0][0], p[1][0], p[2][0]},
		{p[0][1], p[1][1], p[2][1]},
	}
}

// CCW mirrors the C++ inline `CCW` in src/utils.h. Returns 1 for CCW,
// -1 for CW, 0 for colinear within tol. tol is the absolute distance
// tolerance for declaring the three points colinear.
func CCW(p0, p1, p2 Vec2, tol float64) int {
	v1x := p1.X - p0.X
	v1y := p1.Y - p0.Y
	v2x := p2.X - p0.X
	v2y := p2.Y - p0.Y
	area := v1x*v2y - v1y*v2x
	base2 := v1x*v1x + v1y*v1y
	if d := v2x*v2x + v2y*v2y; d > base2 {
		base2 = d
	}
	if area*area*4 <= base2*tol*tol {
		return 0
	}
	if area > 0 {
		return 1
	}
	return -1
}
