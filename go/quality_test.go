package manifold

import "testing"

// TestQuality_Defaults verifies the default-segment formula matches
// the C++ Quality::GetCircularSegments output for several radii.
//
// Formula (defaults: angle=10°, length=1.0, segments=0):
//   nSegA = 360/10 = 36
//   nSegL = floor(2π·|r|)
//   nSeg = min(nSegA, nSegL) + 3, rounded down to multiple of 4, floor 4
func TestQuality_Defaults(t *testing.T) {
	defer ResetQualityToDefaults()
	cases := []struct {
		radius float64
		want   int
	}{
		// r=0: nSegL=0, +3=3, %4 → 0, max(0,4)=4
		{0, 4},
		// r=0.1: nSegL=0 (floor(0.628)), +3=3, %4 → 0, max(0,4)=4
		{0.1, 4},
		// r=1.0: nSegL = floor(2π) = 6, +3=9, 9-9%4=8
		{1.0, 8},
		// r=10: nSegL = floor(20π) = 62, min(36,62)=36, +3=39, 39-39%4=36
		{10.0, 36},
		// r=100: still capped by angle (36), +3=39, → 36
		{100.0, 36},
	}
	for _, tc := range cases {
		if got := GetCircularSegments(tc.radius); got != tc.want {
			t.Errorf("GetCircularSegments(%v) = %d, want %d", tc.radius, got, tc.want)
		}
	}
}

// TestQuality_SetCircularSegments verifies the explicit-override path
// wins over the angle/length formula.
func TestQuality_SetCircularSegments(t *testing.T) {
	defer ResetQualityToDefaults()
	SetCircularSegments(7)
	if got := GetCircularSegments(123); got != 7 {
		t.Errorf("explicit override: GetCircularSegments(123) = %d, want 7", got)
	}
	// Values < 3 (other than 0) are silently ignored.
	SetCircularSegments(2)
	if got := GetCircularSegments(1); got != 7 {
		t.Errorf("invalid override (<3): segments should remain 7, got %d", got)
	}
	// 0 clears the override.
	SetCircularSegments(0)
	if got := GetCircularSegments(1.0); got != 8 {
		t.Errorf("after clear: got %d, want 8", got)
	}
}

// TestQuality_SetMinCircular verifies that custom angle/edge-length
// values are picked up by the formula, and that <= 0 inputs are ignored.
func TestQuality_SetMinCircular(t *testing.T) {
	defer ResetQualityToDefaults()
	SetMinCircularAngle(5) // doubles segments-by-angle to 72
	if got := GetCircularSegments(100); got != 72 {
		t.Errorf("custom angle: got %d, want 72", got)
	}
	SetMinCircularAngle(-1) // ignored
	if got := GetCircularSegments(100); got != 72 {
		t.Errorf("after invalid angle: got %d, want 72", got)
	}
	SetMinCircularEdgeLength(0.5) // doubles segments-by-length
	if got := GetCircularSegments(1.0); got != 12 {
		// nSegL=floor(2π·1/0.5)=12, nSegA=72, min=12, +3=15, %4=3, 15-3=12
		t.Errorf("custom edge length: got %d, want 12", got)
	}
}
