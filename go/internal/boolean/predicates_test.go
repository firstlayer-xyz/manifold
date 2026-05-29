package boolean

import (
	"math"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

func TestShadows(t *testing.T) {
	cases := []struct {
		p, q, dir float64
		want      bool
	}{
		{1, 2, 0, true},    // p < q
		{2, 1, 0, false},   // p > q
		{1, 1, -1, true},   // tie, dir < 0
		{1, 1, 1, false},   // tie, dir > 0
		{1, 1, 0, false},   // tie, dir == 0 -> 0<0 is false
		{0, 0, -0.5, true}, // tie at zero, dir < 0
	}
	for _, c := range cases {
		if got := shadows(c.p, c.q, c.dir); got != c.want {
			t.Errorf("shadows(%v,%v,%v) = %v, want %v", c.p, c.q, c.dir, got, c.want)
		}
	}
}

func TestWithSign(t *testing.T) {
	if withSign(true, 5) != 5 {
		t.Error("withSign(true, 5) != 5")
	}
	if withSign(false, 5) != -5 {
		t.Error("withSign(false, 5) != -5")
	}
}

func TestNext3Prev3(t *testing.T) {
	if Next3(0) != 1 || Next3(1) != 2 || Next3(2) != 0 {
		t.Errorf("Next3 = %d %d %d, want 1 2 0", Next3(0), Next3(1), Next3(2))
	}
	if Prev3(0) != 2 || Prev3(1) != 0 || Prev3(2) != 1 {
		t.Errorf("Prev3 = %d %d %d, want 2 0 1", Prev3(0), Prev3(1), Prev3(2))
	}
}

func TestInterpolate(t *testing.T) {
	aL := geom.Vec3{X: 0, Y: 0, Z: 0}
	aR := geom.Vec3{X: 2, Y: 4, Z: 6}
	approx := func(a, b geom.Vec2) bool {
		return math.Abs(a.X-b.X) < 1e-15 && math.Abs(a.Y-b.Y) < 1e-15
	}
	// x = 0.5 -> useL (|0.5| < |1.5|), t = 0.25: (y,z) = (1, 1.5).
	if got := interpolate(aL, aR, 0.5); !approx(got, geom.Vec2{X: 1, Y: 1.5}) {
		t.Errorf("interpolate at 0.5 = %v, want {1, 1.5}", got)
	}
	// x = 1 -> dxL==dxR in magnitude, useL false (not strictly <), uses aR
	// branch, t = 0.5: (y,z) = (2, 3).
	if got := interpolate(aL, aR, 1); !approx(got, geom.Vec2{X: 2, Y: 3}) {
		t.Errorf("interpolate at 1 = %v, want {2, 3}", got)
	}
	// Degenerate dLR.x == 0 -> non-finite lambda -> fallback to aL.(y,z).
	flat := geom.Vec3{X: 5, Y: 7, Z: 9}
	if got := interpolate(flat, geom.Vec3{X: 5, Y: 1, Z: 2}, 5); !approx(got, geom.Vec2{X: 7, Y: 9}) {
		t.Errorf("interpolate degenerate = %v, want {7, 9} (aL.yz fallback)", got)
	}
}
