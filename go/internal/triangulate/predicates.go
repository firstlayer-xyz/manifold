package triangulate

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// cppMin mirrors std::min(a,b) = (b<a)?b:a exactly (including its NaN
// behavior: std::min(a,NaN)==a), which differs from Go's math.Min.
func cppMin(a, b float64) float64 {
	if b < a {
		return b
	}
	return a
}

// insideEdge is the Go port of Vert::InsideEdge (src/polygon.cpp:345): returns
// true if the receiver is on the inside of the edge tail->tail.right, walking
// edges until a clear (beyond-epsilon) answer is found. toLeft chooses the
// walk direction for the receiver's side.
func (v *vert) insideEdge(tail *vert, epsilon float64, toLeft bool) bool {
	p2 := epsilon * epsilon
	step := func(n *vert) *vert {
		if toLeft {
			return n.left
		}
		return n.right
	}
	boundary := v.left
	if toLeft {
		boundary = v.right
	}

	nextL := v.left.right
	nextR := tail.right
	center := tail
	last := center

	for nextL != nextR && tail != nextR && nextL != boundary {
		edgeL := nextL.pos.Sub(center.pos)
		l2 := edgeL.Dot(edgeL)
		if l2 <= p2 {
			nextL = step(nextL)
			continue
		}
		edgeR := nextR.pos.Sub(center.pos)
		r2 := edgeR.Dot(edgeR)
		if r2 <= p2 {
			nextR = nextR.right
			continue
		}
		vecLR := nextR.pos.Sub(nextL.pos)
		lr2 := vecLR.Dot(vecLR)
		if lr2 <= p2 {
			last = center
			center = nextL
			nextL = step(nextL)
			if nextL == nextR {
				break
			}
			nextR = nextR.right
			continue
		}
		convexity := geom.CCW(nextL.pos, center.pos, nextR.pos, epsilon)
		if center != last {
			convexity += geom.CCW(last.pos, center.pos, nextL.pos, epsilon) +
				geom.CCW(nextR.pos, center.pos, last.pos, epsilon)
		}
		if convexity != 0 {
			return convexity > 0
		}
		if l2 < r2 {
			center = nextL
			nextL = step(nextL)
		} else {
			center = nextR
			nextR = nextR.right
		}
		last = center
	}
	// The whole polygon is degenerate — consider this convex.
	return true
}

// isConvex mirrors Vert::IsConvex (polygon.cpp:400): true for convex OR
// colinear ears.
func (v *vert) isConvex(epsilon float64) bool {
	return geom.CCW(v.left.pos, v.pos, v.right.pos, epsilon) >= 0
}

// isReflex mirrors Vert::IsReflex (polygon.cpp:407): walks to certainty (not
// merely !isConvex, which is true for colinear non-folded verts).
func (v *vert) isReflex(epsilon float64) bool {
	return !v.left.insideEdge(v.left.right, epsilon, true)
}

// signedDist mirrors Vert::SignedDist (polygon.cpp:440): cost of o relative to
// one closed side of the receiver's ear; walks o's neighbors when within
// epsilon of the line.
func (v *vert) signedDist(o *vert, unit geom.Vec2, epsilon float64) float64 {
	d := geom.Determinant2x2(unit, o.pos.Sub(v.pos))
	if math.Abs(d) < epsilon {
		dR := geom.Determinant2x2(unit, o.right.pos.Sub(v.pos))
		if math.Abs(dR) > epsilon {
			return dR
		}
		dL := geom.Determinant2x2(unit, o.left.pos.Sub(v.pos))
		if math.Abs(dL) > epsilon {
			return dL
		}
	}
	return d
}

// costOf mirrors Vert::Cost (polygon.cpp:453): cost of o within the receiver's
// ear; openSide is the unit vector from right to left, passed in for reuse.
func (v *vert) costOf(o *vert, openSide geom.Vec2, epsilon float64) float64 {
	cost := cppMin(v.signedDist(o, v.rightDir, epsilon),
		v.signedDist(o, v.left.rightDir, epsilon))
	openCost := geom.Determinant2x2(openSide, o.pos.Sub(v.right.pos))
	return cppMin(cost, openCost)
}

// delaunayCost mirrors Vert::DelaunayCost (polygon.cpp:464): a Delaunay-style
// improvement cost, always < -epsilon so it never affects validity.
func delaunayCost(diff geom.Vec2, scale, epsilon float64) float64 {
	return -epsilon - scale*diff.Dot(diff)
}

// earCost is the Go port of Vert::EarCost (polygon.cpp:478): the cost of this
// ear, checking it against every other vert. Increment-6 uses a brute-force
// scan over the collider verts filtered by earBox.Contains — output-identical
// to QueryTwoDTree (which also Contains-filters), since totalCost is an
// order-insensitive max. The real 2D kd-tree replaces the scan in increment 9.
func (v *vert) earCost(epsilon float64, collider []*vert) float64 {
	openSide := v.left.pos.Sub(v.right.pos)
	center := v.left.pos.Add(v.right.pos).Scale(0.5)
	scale := 4 / openSide.Dot(openSide)
	radius := openSide.Length() / 2
	openSide = openSide.Normalize()

	totalCost := v.left.rightDir.Dot(v.rightDir) - 1 - epsilon
	if geom.CCW(v.pos, v.left.pos, v.right.pos, epsilon) == 0 {
		// Clip folded ears first.
		return totalCost
	}

	earBox := geom.NewRect(
		geom.Vec2{X: center.X - radius, Y: center.Y - radius},
		geom.Vec2{X: center.X + radius, Y: center.Y + radius},
	)
	earBox = earBox.Union(v.pos)
	earBox.Min = earBox.Min.Sub(geom.Vec2{X: epsilon, Y: epsilon})
	earBox.Max = earBox.Max.Add(geom.Vec2{X: epsilon, Y: epsilon})

	lid := v.left.meshIdx
	rid := v.right.meshIdx
	for _, test := range collider {
		if !earBox.Contains(test.pos) {
			continue
		}
		if !clipped(test) && test.meshIdx != v.meshIdx &&
			test.meshIdx != lid && test.meshIdx != rid { // skip duplicated verts
			cost := v.costOf(test, openSide, epsilon)
			if cost < -epsilon {
				cost = delaunayCost(test.pos.Sub(center), scale, epsilon)
			}
			if cost > totalCost {
				totalCost = cost
			}
		}
	}
	return totalCost
}
