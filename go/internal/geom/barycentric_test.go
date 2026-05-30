package geom

import (
	"math"
	"testing"
)

func TestGetBarycentric(t *testing.T) {
	tri := [3]Vec3{{X: 0, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 0, Y: 1, Z: 0}}
	tol := 1e-6
	approx := func(a, b Vec3) bool {
		return math.Abs(a.X-b.X) < 1e-9 && math.Abs(a.Y-b.Y) < 1e-9 && math.Abs(a.Z-b.Z) < 1e-9
	}
	cases := []struct {
		name string
		v    Vec3
		want Vec3
	}{
		{"vertex0", Vec3{X: 0, Y: 0, Z: 0}, Vec3{X: 1}},
		{"vertex1", Vec3{X: 1, Y: 0, Z: 0}, Vec3{Y: 1}},
		{"vertex2", Vec3{X: 0, Y: 1, Z: 0}, Vec3{Z: 1}},
		{"centroid", Vec3{X: 1.0 / 3, Y: 1.0 / 3, Z: 0}, Vec3{X: 1.0 / 3, Y: 1.0 / 3, Z: 1.0 / 3}},
		{"edge12mid", Vec3{X: 0.5, Y: 0.5, Z: 0}, Vec3{Y: 0.5, Z: 0.5}},
	}
	for _, c := range cases {
		got := GetBarycentric(c.v, tri, tol)
		if !approx(got, c.want) {
			t.Errorf("%s: GetBarycentric = %+v, want %+v", c.name, got, c.want)
		}
	}
}
