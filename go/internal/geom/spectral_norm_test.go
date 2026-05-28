package geom

import (
	"math"
	"testing"
)

// TestSpectralNorm_Identity returns 1 for the identity matrix
// (largest singular value = 1).
func TestSpectralNorm_Identity(t *testing.T) {
	id := Mat3{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
	if got := SpectralNorm(id); math.Abs(got-1.0) > 1e-12 {
		t.Errorf("SpectralNorm(I) = %v, want 1", got)
	}
}

// TestSpectralNorm_DiagonalScale: for a diagonal scale matrix
// diag(sx, sy, sz), spectral norm = max(|sx|, |sy|, |sz|).
func TestSpectralNorm_DiagonalScale(t *testing.T) {
	cases := []struct {
		sx, sy, sz float64
		want       float64
	}{
		{1, 1, 1, 1},
		{2, 3, 4, 4},
		{0.5, 0.25, 0.1, 0.5},
		{-2, 3, -4, 4},
	}
	for _, c := range cases {
		m := Mat3{{c.sx, 0, 0}, {0, c.sy, 0}, {0, 0, c.sz}}
		got := SpectralNorm(m)
		if math.Abs(got-c.want) > 1e-12 {
			t.Errorf("SpectralNorm(diag(%v,%v,%v)) = %v, want %v",
				c.sx, c.sy, c.sz, got, c.want)
		}
	}
}

// TestSpectralNorm_Rotation: a pure rotation has spectral norm 1
// (all singular values are 1 for an orthogonal matrix).
func TestSpectralNorm_Rotation(t *testing.T) {
	// 45-degree rotation about the Z axis.
	cs := math.Cos(math.Pi / 4)
	sn := math.Sin(math.Pi / 4)
	rotZ := Mat3{{cs, sn, 0}, {-sn, cs, 0}, {0, 0, 1}}
	if got := SpectralNorm(rotZ); math.Abs(got-1.0) > 1e-12 {
		t.Errorf("SpectralNorm(rotZ45) = %v, want 1", got)
	}
}

// TestSpectralNorm_KnownExample: a 3x3 matrix with known singular
// values [3, 2, 1] (constructed as diag * rotation). Spectral norm
// must be 3.
func TestSpectralNorm_KnownExample(t *testing.T) {
	// Construct A = U · diag(3,2,1) · V^T for any orthonormal U, V.
	// Easiest: A is itself diagonal.
	a := Mat3{{3, 0, 0}, {0, 2, 0}, {0, 0, 1}}
	if got := SpectralNorm(a); math.Abs(got-3.0) > 1e-12 {
		t.Errorf("SpectralNorm = %v, want 3", got)
	}
}
