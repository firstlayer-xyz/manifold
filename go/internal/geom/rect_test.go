package geom

import (
	"math"
	"testing"
)

func TestVec2Primitives(t *testing.T) {
	if d := Determinant2x2(Vec2{X: 1, Y: 0}, Vec2{X: 0, Y: 1}); d != 1 {
		t.Errorf("Determinant2x2 = %v, want 1", d)
	}
	if d := Determinant2x2(Vec2{X: 2, Y: 3}, Vec2{X: 4, Y: 5}); d != 2*5-3*4 {
		t.Errorf("Determinant2x2 = %v, want -2", d)
	}
	// Normalize of a zero vector yields NaN (mirrors la::normalize); the
	// triangulator's IsConvex relies on this.
	if n := (Vec2{}).Normalize(); !math.IsNaN(n.X) {
		t.Errorf("Normalize({0,0}).X = %v, want NaN", n.X)
	}
	// SafeNormalize of a zero vector returns {0,0} (isfinite(n.x) guard).
	if n := (Vec2{}).SafeNormalize(); n != (Vec2{}) {
		t.Errorf("SafeNormalize({0,0}) = %v, want {0,0}", n)
	}
	if n := (Vec2{X: 3, Y: 4}).SafeNormalize(); math.Abs(n.X-0.6) > 1e-12 || math.Abs(n.Y-0.8) > 1e-12 {
		t.Errorf("SafeNormalize({3,4}) = %v, want {0.6,0.8}", n)
	}
}

func TestRect(t *testing.T) {
	r := EmptyRect().Union(Vec2{X: 1, Y: 2}).Union(Vec2{X: -3, Y: 5})
	if r.Min != (Vec2{X: -3, Y: 2}) || r.Max != (Vec2{X: 1, Y: 5}) {
		t.Fatalf("Union: min=%v max=%v", r.Min, r.Max)
	}
	if !r.Contains(Vec2{X: 0, Y: 3}) {
		t.Errorf("Contains interior point failed")
	}
	if !r.Contains(Vec2{X: -3, Y: 2}) {
		t.Errorf("Contains border point failed (>= is inclusive)")
	}
	if r.Contains(Vec2{X: 2, Y: 3}) {
		t.Errorf("Contains should reject x past max")
	}
	if s := r.Scale(); s != 5 {
		t.Errorf("Scale = %v, want 5", s)
	}
	if c := r.Center(); c != (Vec2{X: -1, Y: 3.5}) {
		t.Errorf("Center = %v, want {-1,3.5}", c)
	}
	if !NewRect(Vec2{}, Vec2{X: 2, Y: 2}).DoesOverlap(NewRect(Vec2{X: 1, Y: 1}, Vec2{X: 3, Y: 3})) {
		t.Errorf("DoesOverlap should be true for overlapping rects")
	}
	if NewRect(Vec2{}, Vec2{X: 1, Y: 1}).DoesOverlap(NewRect(Vec2{X: 2, Y: 2}, Vec2{X: 3, Y: 3})) {
		t.Errorf("DoesOverlap should be false for disjoint rects")
	}
}
