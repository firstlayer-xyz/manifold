package geom

// Triangle-triangle and edge-edge distance primitives — Go port of
// src/tri_dist.h. The C++ originals are derived from NVIDIA
// Omniverse PhysX (BSD 3-Clause) and live verbatim alongside the
// other manifold geometry primitives. We keep the same structure.

// EdgeEdgeDist returns the closest points x (on segment p→p+a) and
// y (on segment q→q+b) between two line segments, via the standard
// minimum-distance-between-segments closed-form formula. Mirrors
// the C++ EdgeEdgeDist signature with two return values instead of
// out-parameters.
func EdgeEdgeDist(p, a, q, b Vec3) (x, y Vec3) {
	T := q.Sub(p)
	aDotA := a.Dot(a)
	bDotB := b.Dot(b)
	aDotB := a.Dot(b)
	aDotT := a.Dot(T)
	bDotT := b.Dot(T)

	// t parameterizes ray (p, a); u parameterizes ray (q, b).
	denom := aDotA*bDotB - aDotB*aDotB
	var t float64
	if denom != 0 {
		t = clamp01((aDotT*bDotB - bDotT*aDotB) / denom)
	}
	var u float64
	if bDotB != 0 {
		u = (t*aDotB - bDotT) / bDotB
		switch {
		case u < 0:
			u = 0
			if aDotA != 0 {
				t = clamp01(aDotT / aDotA)
			} else {
				t = 0
			}
		case u > 1:
			u = 1
			if aDotA != 0 {
				t = clamp01((aDotB + aDotT) / aDotA)
			} else {
				t = 0
			}
		}
	} else {
		if aDotA != 0 {
			t = clamp01(aDotT / aDotA)
		} else {
			t = 0
		}
	}
	x = p.Add(a.Scale(t))
	y = q.Add(b.Scale(u))
	return x, y
}

// DistanceTriangleTriangleSquared is the Go port of C++
// DistanceTriangleTriangleSquared (src/tri_dist.h). Returns the
// minimum squared distance between two triangles; 0 means they
// touch or intersect.
//
// Mirrors the C++ algorithm step for step — edge-edge minima
// followed by face-vertex separating-plane checks.
func DistanceTriangleTriangleSquared(p, q [3]Vec3) float64 {
	var Sv [3]Vec3
	Sv[0] = p[1].Sub(p[0])
	Sv[1] = p[2].Sub(p[1])
	Sv[2] = p[0].Sub(p[2])

	var Tv [3]Vec3
	Tv[0] = q[1].Sub(q[0])
	Tv[1] = q[2].Sub(q[1])
	Tv[2] = q[0].Sub(q[2])

	shownDisjoint := false
	mindd := maxFloat
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			cp, cq := EdgeEdgeDist(p[i], Sv[i], q[j], Tv[j])
			V := cq.Sub(cp)
			dd := V.Dot(V)
			if dd > mindd {
				continue
			}
			mindd = dd

			id := i + 2
			if id >= 3 {
				id -= 3
			}
			Z := p[id].Sub(cp)
			a := Z.Dot(V)
			id = j + 2
			if id >= 3 {
				id -= 3
			}
			Z = q[id].Sub(cq)
			b := Z.Dot(V)

			if a <= 0 && b >= 0 {
				return V.Dot(V)
			}
			if a <= 0 {
				a = 0
			} else if b > 0 {
				b = 0
			}
			if (mindd - a + b) > 0 {
				shownDisjoint = true
			}
		}
	}

	// Separating-plane check using the normal of triangle p.
	Sn := Sv[0].Cross(Sv[1])
	Snl := Sn.Dot(Sn)
	if Snl > 1e-15 {
		Tp := Vec3{
			X: p[0].Sub(q[0]).Dot(Sn),
			Y: p[0].Sub(q[1]).Dot(Sn),
			Z: p[0].Sub(q[2]).Dot(Sn),
		}
		index := -1
		switch {
		case Tp.X > 0 && Tp.Y > 0 && Tp.Z > 0:
			if Tp.X < Tp.Y {
				index = 0
			} else {
				index = 1
			}
			if (index == 0 && Tp.Z < Tp.X) || (index == 1 && Tp.Z < Tp.Y) {
				index = 2
			}
		case Tp.X < 0 && Tp.Y < 0 && Tp.Z < 0:
			if Tp.X > Tp.Y {
				index = 0
			} else {
				index = 1
			}
			if (index == 0 && Tp.Z > Tp.X) || (index == 1 && Tp.Z > Tp.Y) {
				index = 2
			}
		}
		if index >= 0 {
			shownDisjoint = true
			qIndex := q[index]
			Tpv := vec3Index(Tp, index)
			V := qIndex.Sub(p[0])
			Z := Sn.Cross(Sv[0])
			if V.Dot(Z) > 0 {
				V = qIndex.Sub(p[1])
				Z = Sn.Cross(Sv[1])
				if V.Dot(Z) > 0 {
					V = qIndex.Sub(p[2])
					Z = Sn.Cross(Sv[2])
					if V.Dot(Z) > 0 {
						cp := qIndex.Add(Sn.Scale(Tpv / Snl))
						cq := qIndex
						d := cp.Sub(cq)
						return d.Dot(d)
					}
				}
			}
		}
	}

	// Mirror separating-plane check with the normal of triangle q.
	Tn := Tv[0].Cross(Tv[1])
	Tnl := Tn.Dot(Tn)
	if Tnl > 1e-15 {
		Sp := Vec3{
			X: q[0].Sub(p[0]).Dot(Tn),
			Y: q[0].Sub(p[1]).Dot(Tn),
			Z: q[0].Sub(p[2]).Dot(Tn),
		}
		index := -1
		switch {
		case Sp.X > 0 && Sp.Y > 0 && Sp.Z > 0:
			if Sp.X < Sp.Y {
				index = 0
			} else {
				index = 1
			}
			if (index == 0 && Sp.Z < Sp.X) || (index == 1 && Sp.Z < Sp.Y) {
				index = 2
			}
		case Sp.X < 0 && Sp.Y < 0 && Sp.Z < 0:
			if Sp.X > Sp.Y {
				index = 0
			} else {
				index = 1
			}
			if (index == 0 && Sp.Z > Sp.X) || (index == 1 && Sp.Z > Sp.Y) {
				index = 2
			}
		}
		if index >= 0 {
			shownDisjoint = true
			pIndex := p[index]
			Spv := vec3Index(Sp, index)
			V := pIndex.Sub(q[0])
			Z := Tn.Cross(Tv[0])
			if V.Dot(Z) > 0 {
				V = pIndex.Sub(q[1])
				Z = Tn.Cross(Tv[1])
				if V.Dot(Z) > 0 {
					V = pIndex.Sub(q[2])
					Z = Tn.Cross(Tv[2])
					if V.Dot(Z) > 0 {
						cp := pIndex
						cq := pIndex.Add(Tn.Scale(Spv / Tnl))
						d := cp.Sub(cq)
						return d.Dot(d)
					}
				}
			}
		}
	}

	if shownDisjoint {
		return mindd
	}
	return 0
}

func clamp01(x float64) float64 {
	if x < 0 {
		return 0
	}
	if x > 1 {
		return 1
	}
	return x
}

func vec3Index(v Vec3, i int) float64 {
	switch i {
	case 0:
		return v.X
	case 1:
		return v.Y
	case 2:
		return v.Z
	}
	return 0
}

const maxFloat = 1.7976931348623157e+308
