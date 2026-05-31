package geom

import (
	"math"
	"testing"
)

func vec3Close(a, b Vec3, tol float64) bool {
	return math.Abs(a.X-b.X) < tol && math.Abs(a.Y-b.Y) < tol && math.Abs(a.Z-b.Z) < tol
}

func TestQconjQmul(t *testing.T) {
	// q * qconj(q) == identity for a unit quaternion.
	q := RotationQuat(Vec3{X: 0.3, Y: -0.5, Z: 0.8}.Normalize(), 1.1)
	id := Qmul(q, Qconj(q))
	if !vec3Close(id.Vec3(), Vec3{}, 1e-12) || math.Abs(id.W-1) > 1e-12 {
		t.Errorf("q*qconj(q) = %+v, want identity {0,0,0,1}", id)
	}
}

func TestQxdir(t *testing.T) {
	q := RotationQuat(Vec3{X: 0.2, Y: 0.7, Z: -0.3}.Normalize(), 0.9)
	if !vec3Close(Qxdir(q), Qrot(q, Vec3{X: 1}), 1e-12) {
		t.Errorf("Qxdir != Qrot(q,{1,0,0}): %+v vs %+v", Qxdir(q), Qrot(q, Vec3{X: 1}))
	}
}

func TestRotationQuatFromTo(t *testing.T) {
	cases := []struct{ orig, dest Vec3 }{
		{Vec3{X: 1}, Vec3{Y: 1}},
		{Vec3{X: 1}, Vec3{X: -1}},            // antiparallel
		{Vec3{X: 1}, Vec3{X: 1}},             // identical
		{Vec3{X: 0.3, Y: 0.4, Z: 0.5}, Vec3{X: -0.2, Y: 0.9, Z: 0.1}},
	}
	for _, c := range cases {
		orig := c.orig.Normalize()
		dest := c.dest.Normalize()
		q := RotationQuatFromTo(orig, dest)
		got := Qrot(q, orig)
		if !vec3Close(got, dest, 1e-9) {
			t.Errorf("RotationQuatFromTo(%v->%v): Qrot(q,orig)=%v, want %v", orig, dest, got, dest)
		}
	}
}

func TestRotationQuatFromMat3(t *testing.T) {
	// 90 deg about Z (column-major): rotates x->y, y->-x, z->z.
	c, s := 0.0, 1.0 // cos(90), sin(90)
	m := Mat3{{c, s, 0}, {-s, c, 0}, {0, 0, 1}}
	q := RotationQuatFromMat3(m)
	// Each basis vector should map to the matrix's corresponding column.
	for i, basis := range []Vec3{{X: 1}, {Y: 1}, {Z: 1}} {
		want := Vec3{X: m[i][0], Y: m[i][1], Z: m[i][2]}
		if got := Qrot(q, basis); !vec3Close(got, want, 1e-9) {
			t.Errorf("RotationQuatFromMat3 col %d: Qrot=%v, want %v", i, got, want)
		}
	}

	// Consistency with the axis-angle form for the same rotation.
	qAxis := RotationQuat(Vec3{Z: 1}, math.Pi/2)
	for _, basis := range []Vec3{{X: 1}, {Y: 1}, {Z: 1}} {
		if !vec3Close(Qrot(q, basis), Qrot(qAxis, basis), 1e-9) {
			t.Errorf("mat3 quat disagrees with axis-angle quat on %v", basis)
		}
	}
}
