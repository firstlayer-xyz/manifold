package geom

import "math"

// SpectralNorm is the Go port of C++ manifold::SpectralNorm
// (src/svd.h:SpectralNorm) — the largest singular value of a 3x3
// matrix. Used by Impl::Transform to scale epsilon by how the
// transform amplifies numerical error.
//
// Computed as sqrt(largestEigenvalue(A^T · A)) via the Smith (1961)
// closed-form eigenvalue formula for a symmetric 3x3 matrix. This
// avoids dragging in the full Jacobi-iteration SVD3x3 used by the
// C++ code; we only need the spectral norm here, which is a single
// scalar derivable directly from A^T A.
func SpectralNorm(a Mat3) float64 {
	// B = A^T · A is symmetric 3x3 with non-negative eigenvalues.
	// Mat3 is column-major: a[c][r] = element at column c, row r.
	// (A^T)[i][j] = A[j][i], so (A^T A)[i][j] = sum_k A[k][i] A[k][j].
	var b [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			var s float64
			for k := 0; k < 3; k++ {
				s += a[k][i] * a[k][j]
			}
			b[i][j] = s
		}
	}
	return math.Sqrt(largestEigSymmetric3(b))
}

// largestEigSymmetric3 returns the largest eigenvalue of a symmetric
// 3x3 matrix m. Uses the Smith (1961) closed-form cubic-root formula
// for the characteristic polynomial of a 3x3 symmetric matrix.
func largestEigSymmetric3(m [3][3]float64) float64 {
	// p1 = sum of off-diagonal squares.
	p1 := m[0][1]*m[0][1] + m[0][2]*m[0][2] + m[1][2]*m[1][2]
	if p1 == 0 {
		// Diagonal — eigenvalues are the diagonal entries.
		return math.Max(m[0][0], math.Max(m[1][1], m[2][2]))
	}
	// q = trace / 3.
	q := (m[0][0] + m[1][1] + m[2][2]) / 3
	// p² = (||M - qI||_F)² / 6.
	d0 := m[0][0] - q
	d1 := m[1][1] - q
	d2 := m[2][2] - q
	p2 := d0*d0 + d1*d1 + d2*d2 + 2*p1
	p := math.Sqrt(p2 / 6)
	// B = (1/p) (M - qI); det(B) is in [-2, 2] for a symmetric matrix,
	// and r = det(B)/2 in [-1, 1] (clamped for numerical safety).
	invP := 1.0 / p
	var bm [3][3]float64
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			bm[i][j] = m[i][j] * invP
			if i == j {
				bm[i][j] -= q * invP
			}
		}
	}
	r := det3(bm) / 2
	if r > 1 {
		r = 1
	} else if r < -1 {
		r = -1
	}
	phi := math.Acos(r) / 3
	// Largest eigenvalue is q + 2p·cos(phi) for symmetric matrices.
	return q + 2*p*math.Cos(phi)
}

func det3(m [3][3]float64) float64 {
	return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]) -
		m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0]) +
		m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])
}
