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
	inv := 1.0 / a.Length()
	return Vec3{X: a.X * inv, Y: a.Y * inv, Z: a.Z * inv}
}

// Degrees converts radians to degrees. Mirrors C++ manifold::degrees.
func Degrees(rad float64) float64 { return rad * (180.0 / math.Pi) }

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

// IsFinite reports whether all corner coordinates are finite.
func (b Box) IsFinite() bool { return b.Min.IsFinite() && b.Max.IsFinite() }

// Translate returns b shifted by v.
func (b Box) Translate(v Vec3) Box { return Box{Min: b.Min.Add(v), Max: b.Max.Add(v)} }

// MulComponent returns b scaled component-wise by s.
func (b Box) MulComponent(s Vec3) Box { return Box{Min: b.Min.Mul(s), Max: b.Max.Mul(s)} }

// Mat3 is a 3x3 matrix in column-major order: Mat3[col][row]. Chosen to
// match the column-major convention used by the C++ linalg library so the
// port can mirror C++ matrix code structurally.
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
