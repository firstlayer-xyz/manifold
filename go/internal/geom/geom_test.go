package geom

import (
	"math"
	"testing"
)

func vec3Eq(a, b Vec3, eps float64) bool {
	return math.Abs(a.X-b.X) <= eps && math.Abs(a.Y-b.Y) <= eps && math.Abs(a.Z-b.Z) <= eps
}

func TestVec3_Arith(t *testing.T) {
	a := Vec3{X: 1, Y: 2, Z: 3}
	b := Vec3{X: 4, Y: -1, Z: 2}

	if got := a.Add(b); got != (Vec3{X: 5, Y: 1, Z: 5}) {
		t.Errorf("Add: got %v", got)
	}
	if got := a.Sub(b); got != (Vec3{X: -3, Y: 3, Z: 1}) {
		t.Errorf("Sub: got %v", got)
	}
	if got := a.Mul(b); got != (Vec3{X: 4, Y: -2, Z: 6}) {
		t.Errorf("Mul: got %v", got)
	}
	if got := a.Scale(2); got != (Vec3{X: 2, Y: 4, Z: 6}) {
		t.Errorf("Scale: got %v", got)
	}
	if got := a.Dot(b); got != 8 { // 1*4 + 2*-1 + 3*2 = 8
		t.Errorf("Dot: got %v, want 8", got)
	}
	// Cross of standard basis: x × y = z.
	x, y, z := Vec3{X: 1}, Vec3{Y: 1}, Vec3{Z: 1}
	if got := x.Cross(y); got != z {
		t.Errorf("x cross y: got %v, want %v", got, z)
	}
	// Cross is anti-commutative.
	if got, want := y.Cross(x), (Vec3{Z: -1}); got != want {
		t.Errorf("y cross x: got %v, want %v", got, want)
	}
}

func TestVec3_AbsMaxFinite(t *testing.T) {
	v := Vec3{X: -3, Y: 1, Z: -7}
	if got := v.Abs(); got != (Vec3{X: 3, Y: 1, Z: 7}) {
		t.Errorf("Abs: got %v", got)
	}
	if got := v.Abs().MaxComponent(); got != 7 {
		t.Errorf("MaxComponent: got %v, want 7", got)
	}
	if !v.IsFinite() {
		t.Error("finite vec reported non-finite")
	}
	if (Vec3{X: math.Inf(1)}).IsFinite() {
		t.Error("inf vec reported finite")
	}
	if (Vec3{Y: math.NaN()}).IsFinite() {
		t.Error("nan vec reported finite")
	}
}

func TestBox_SizeCenterScale(t *testing.T) {
	b := NewBox(Vec3{X: -1, Y: -2, Z: -3}, Vec3{X: 4, Y: 5, Z: 6})
	if got := b.Size(); got != (Vec3{X: 5, Y: 7, Z: 9}) {
		t.Errorf("Size: got %v", got)
	}
	if got := b.Center(); got != (Vec3{X: 1.5, Y: 1.5, Z: 1.5}) {
		t.Errorf("Center: got %v", got)
	}
	if got := b.Scale(); got != 6 {
		t.Errorf("Scale: got %v, want 6", got)
	}
}

func TestBox_ContainsOverlap(t *testing.T) {
	b := Box{Min: Vec3{}, Max: Vec3{X: 1, Y: 1, Z: 1}}
	if !b.Contains(Vec3{X: 0.5, Y: 0.5, Z: 0.5}) {
		t.Error("interior point not contained")
	}
	if !b.Contains(Vec3{}) { // boundary
		t.Error("boundary point not contained")
	}
	if b.Contains(Vec3{X: 2}) {
		t.Error("exterior point reported contained")
	}

	inner := Box{Min: Vec3{X: 0.2, Y: 0.2, Z: 0.2}, Max: Vec3{X: 0.8, Y: 0.8, Z: 0.8}}
	if !b.ContainsBox(inner) {
		t.Error("strictly inner box not contained")
	}
	if inner.ContainsBox(b) {
		t.Error("outer box wrongly contained by inner")
	}

	overlap := Box{Min: Vec3{X: 0.5}, Max: Vec3{X: 2, Y: 1, Z: 1}}
	if !b.DoesOverlap(overlap) {
		t.Error("overlapping boxes reported non-overlapping")
	}
	disjoint := Box{Min: Vec3{X: 2}, Max: Vec3{X: 3, Y: 1, Z: 1}}
	if b.DoesOverlap(disjoint) {
		t.Error("disjoint boxes reported overlapping")
	}
}

func TestBox_TranslateScaleFinite(t *testing.T) {
	b := Box{Min: Vec3{}, Max: Vec3{X: 1, Y: 1, Z: 1}}
	moved := b.Translate(Vec3{X: 10, Y: 20, Z: 30})
	if moved.Min != (Vec3{X: 10, Y: 20, Z: 30}) || moved.Max != (Vec3{X: 11, Y: 21, Z: 31}) {
		t.Errorf("Translate: got %+v", moved)
	}
	scaled := b.MulComponent(Vec3{X: 2, Y: 3, Z: 4})
	if scaled.Max != (Vec3{X: 2, Y: 3, Z: 4}) {
		t.Errorf("MulComponent: got %+v", scaled)
	}
	if !b.IsFinite() {
		t.Error("finite box reported non-finite")
	}
	infBox := Box{Min: Vec3{X: math.Inf(1)}}
	if infBox.IsFinite() {
		t.Error("inf box reported finite")
	}
}

func TestSindCosd_ExactAtMultiplesOf90(t *testing.T) {
	// The whole point of sind/cosd is that multiples of 90° come out exact.
	cases := []struct {
		deg            float64
		wantS, wantC   float64
	}{
		{0, 0, 1},
		{90, 1, 0},
		{180, 0, -1},
		{270, -1, 0},
		{360, 0, 1},
		{-90, -1, 0},
		{-180, 0, -1},
	}
	for _, c := range cases {
		if got := Sind(c.deg); got != c.wantS {
			t.Errorf("Sind(%v) = %v, want %v", c.deg, got, c.wantS)
		}
		if got := Cosd(c.deg); got != c.wantC {
			t.Errorf("Cosd(%v) = %v, want %v", c.deg, got, c.wantC)
		}
	}
}

func TestSindCosd_ArbitraryAngles(t *testing.T) {
	// At non-multiples-of-90 we only require small relative error against
	// the radian-based reference; we're not bit-identical.
	const eps = 1e-12
	for _, deg := range []float64{30, 45, 60, 17.3, -42.1, 123.456} {
		rad := deg * math.Pi / 180
		if got, want := Sind(deg), math.Sin(rad); math.Abs(got-want) > eps {
			t.Errorf("Sind(%v) = %v, want ~%v", deg, got, want)
		}
		if got, want := Cosd(deg), math.Cos(rad); math.Abs(got-want) > eps {
			t.Errorf("Cosd(%v) = %v, want ~%v", deg, got, want)
		}
	}
}

func TestMat3_Mul_Identity(t *testing.T) {
	identity := Mat3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
	a := Mat3{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}
	if got := identity.Mul(a); got != a {
		t.Errorf("identity * a should equal a, got %v", got)
	}
	if got := a.Mul(identity); got != a {
		t.Errorf("a * identity should equal a, got %v", got)
	}
}

func TestMat3_Mul_Composition(t *testing.T) {
	// Verify column-major composition: a 90° rotation about Z, applied
	// twice, should give a 180° rotation. Column-major rZ from manifold:
	//   col0=(cos, sin, 0), col1=(-sin, cos, 0), col2=(0,0,1).
	rZ90 := Mat3{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
	rZ180 := rZ90.Mul(rZ90)
	want := Mat3{{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}}
	const eps = 1e-15
	if !vec3Eq(Vec3{rZ180[0][0], rZ180[0][1], rZ180[0][2]},
		Vec3{want[0][0], want[0][1], want[0][2]}, eps) {
		t.Errorf("rZ90 ∘ rZ90 column 0 mismatch: %v vs %v", rZ180[0], want[0])
	}
}

func TestMat3_MulVec3(t *testing.T) {
	// 90° rotation about Z (column-major as defined in manifold):
	//   col0 = (cos, sin, 0) = (0, 1, 0)
	//   col1 = (-sin, cos, 0) = (-1, 0, 0)
	// Applied to (1, 0, 0) should give (0, 1, 0).
	rZ90 := Mat3{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
	if got, want := rZ90.MulVec3(Vec3{X: 1}), (Vec3{Y: 1}); !vec3Eq(got, want, 1e-15) {
		t.Errorf("rZ90 * x = %v, want %v", got, want)
	}
}

func TestMat3_Transpose(t *testing.T) {
	a := Mat3{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}}
	want := Mat3{{1, 4, 7}, {2, 5, 8}, {3, 6, 9}}
	if a.Transpose() != want {
		t.Errorf("Transpose: got %v, want %v", a.Transpose(), want)
	}
	if a.Transpose().Transpose() != a {
		t.Error("Transpose involution failed")
	}
}

func TestMat3_Determinant(t *testing.T) {
	identity := Mat3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
	if got := identity.Determinant(); got != 1 {
		t.Errorf("det(I) = %v, want 1", got)
	}
	// Reflection across the X = 0 plane: det = -1.
	mirror := Mat3{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
	if got := mirror.Determinant(); got != -1 {
		t.Errorf("det(mirror) = %v, want -1", got)
	}
	// Uniform scale by 2: det = 8.
	scale := Mat3{{2, 0, 0}, {0, 2, 0}, {0, 0, 2}}
	if got := scale.Determinant(); got != 8 {
		t.Errorf("det(2I) = %v, want 8", got)
	}
}

func TestMat3_Inverse_IdentityAndRotation(t *testing.T) {
	// Inverse of identity is identity.
	identity := Mat3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
	if identity.Inverse() != identity {
		t.Errorf("I⁻¹ = %v, want I", identity.Inverse())
	}
	// Inverse of a rotation is its transpose.
	rZ90 := Mat3{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
	inv := rZ90.Inverse()
	want := rZ90.Transpose()
	const eps = 1e-15
	for col := 0; col < 3; col++ {
		for row := 0; row < 3; row++ {
			if math.Abs(inv[col][row]-want[col][row]) > eps {
				t.Errorf("rZ90⁻¹[%d][%d] = %v, want %v",
					col, row, inv[col][row], want[col][row])
			}
		}
	}
}

func TestMat3_Inverse_NonOrthogonal(t *testing.T) {
	// A non-orthogonal invertible matrix.  A * A⁻¹ must be identity.
	a := Mat3{{2, 0, 0}, {1, 3, 0}, {0, 0, 5}}
	prod := a.Mul(a.Inverse())
	const eps = 1e-13
	for col := 0; col < 3; col++ {
		for row := 0; row < 3; row++ {
			expect := 0.0
			if col == row {
				expect = 1
			}
			if math.Abs(prod[col][row]-expect) > eps {
				t.Errorf("(A · A⁻¹)[%d][%d] = %v, want %v",
					col, row, prod[col][row], expect)
			}
		}
	}
}

func TestNormalTransform(t *testing.T) {
	// For an orthogonal transform (rotation), inv(transpose(M)) = M.
	rZ90 := Mat3{{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}}
	t34 := Mat3x4{rZ90[0], rZ90[1], rZ90[2], {0, 0, 0}}
	nt := NormalTransform(t34)
	const eps = 1e-15
	for col := 0; col < 3; col++ {
		for row := 0; row < 3; row++ {
			if math.Abs(nt[col][row]-rZ90[col][row]) > eps {
				t.Errorf("NormalTransform(rZ90)[%d][%d] = %v, want %v",
					col, row, nt[col][row], rZ90[col][row])
			}
		}
	}

	// For a non-uniform scale diag(2, 3, 4), the normal transform is
	// diag(1/2, 1/3, 1/4) (since the matrix is diagonal and symmetric).
	scale := Mat3{{2, 0, 0}, {0, 3, 0}, {0, 0, 4}}
	t34 = Mat3x4{scale[0], scale[1], scale[2], {0, 0, 0}}
	got := NormalTransform(t34)
	want := Mat3{{0.5, 0, 0}, {0, 1.0 / 3, 0}, {0, 0, 0.25}}
	for col := 0; col < 3; col++ {
		for row := 0; row < 3; row++ {
			if math.Abs(got[col][row]-want[col][row]) > 1e-15 {
				t.Errorf("NormalTransform(scale)[%d][%d] = %v, want %v",
					col, row, got[col][row], want[col][row])
			}
		}
	}
}

// TestGetAxisAlignedProjection_AxisCases covers the three branch arms
// in C++ src/shared.h: dominant Z, dominant Y, dominant X. For each,
// projecting a point picks two of its components (the ones orthogonal
// to the dominant axis).
func TestGetAxisAlignedProjection_AxisCases(t *testing.T) {
	cases := []struct {
		name   string
		normal Vec3
		probe  Vec3
		wantX  float64
		wantY  float64
	}{
		{"dom +Z", Vec3{0, 0, 1}, Vec3{1, 2, 3}, 1, 2},
		{"dom +Y", Vec3{0, 1, 0}, Vec3{1, 2, 3}, 3, 1},
		{"dom +X", Vec3{1, 0, 0}, Vec3{1, 2, 3}, 2, 3},
		{"dom -Z (orientation flip)", Vec3{0, 0, -1}, Vec3{1, 2, 3}, -1, 2},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			p := GetAxisAlignedProjection(tc.normal)
			got := p.MulVec3(tc.probe)
			if got.X != tc.wantX || got.Y != tc.wantY {
				t.Errorf("project(%v, %v) = %v, want {%v, %v}",
					tc.normal, tc.probe, got, tc.wantX, tc.wantY)
			}
		})
	}
}

// TestCCW_SignAndColinearity covers all three CCW outcomes.
func TestCCW_SignAndColinearity(t *testing.T) {
	cases := []struct {
		name       string
		p0, p1, p2 Vec2
		tol        float64
		want       int
	}{
		{"CCW triangle", Vec2{0, 0}, Vec2{1, 0}, Vec2{0, 1}, 0, 1},
		{"CW triangle", Vec2{0, 0}, Vec2{0, 1}, Vec2{1, 0}, 0, -1},
		{"colinear (exact)", Vec2{0, 0}, Vec2{1, 1}, Vec2{2, 2}, 0, 0},
		{"colinear within tol", Vec2{0, 0}, Vec2{1, 0}, Vec2{2, 1e-10}, 1e-3, 0},
		{"non-colinear above tol", Vec2{0, 0}, Vec2{1, 0}, Vec2{2, 0.1}, 1e-3, 1},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			if got := CCW(tc.p0, tc.p1, tc.p2, tc.tol); got != tc.want {
				t.Errorf("CCW(%v,%v,%v, tol=%v) = %d, want %d",
					tc.p0, tc.p1, tc.p2, tc.tol, got, tc.want)
			}
		})
	}
}
