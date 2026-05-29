package boolean

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// mesh is the read-view of one operand Impl that the kernel cascade reads:
// vertex positions/normals, face normals, and the halfedge view. The C++ kernel
// takes Manifold::Impl& inA/inB; the Go kernel takes *mesh.
type mesh struct {
	vertPos    []geom.Vec3
	vertNormal []geom.Vec3
	faceNormal []geom.Vec3
	halfedge   halfedges
}

// b2i is the Go equivalent of C++ bool->int (1/0), used where the kernel
// subtracts two Shadows() results into a -1/0/1 overlap count.
func b2i(b bool) int {
	if b {
		return 1
	}
	return 0
}

// shadow01 is the Go port of Shadow01<expandP, forward> (boolean3.cpp:76): the
// x-shadow overlap (-1/0/1) of vertex a0 against edge b1 (b1s->b1e), plus the
// edge's interpolated (y, z) at a0.x. The C++ bool template params expandP /
// forward are passed at runtime. Returns (s01, NaN-yz) when there is no shadow.
// Float-expression order is transcribed byte-for-byte (symbolic perturbation).
func shadow01(expandP, forward bool, a0, b1, b1s, b1e int, inA, inB *mesh) (int, geom.Vec2) {
	a0x := inA.vertPos[a0].X
	b1sx := inB.vertPos[b1s].X
	b1ex := inB.vertPos[b1e].X
	a0xp := inA.vertNormal[a0].X
	b1sxp := inB.vertNormal[b1s].X
	b1exp := inB.vertNormal[b1e].X
	var s01 int
	if forward {
		s01 = b2i(shadows(a0x, b1ex, withSign(expandP, a0xp)-b1exp)) -
			b2i(shadows(a0x, b1sx, withSign(expandP, a0xp)-b1sxp))
	} else {
		s01 = b2i(shadows(b1sx, a0x, withSign(expandP, b1sxp)-a0xp)) -
			b2i(shadows(b1ex, a0x, withSign(expandP, b1exp)-a0xp))
	}
	yz01 := geom.Vec2{X: math.NaN(), Y: math.NaN()}

	if s01 != 0 {
		yz01 = interpolate(inB.vertPos[b1s], inB.vertPos[b1e], inA.vertPos[a0].X)
		b1pair := inB.halfedge.Pair(b1)
		dir := inB.faceNormal[b1/3].Y + inB.faceNormal[b1pair/3].Y
		if forward {
			if !shadows(inA.vertPos[a0].Y, yz01.X, -dir) {
				s01 = 0
			}
		} else {
			if !shadows(yz01.X, inA.vertPos[a0].Y, withSign(expandP, dir)) {
				s01 = 0
			}
		}
	}
	return s01, yz01
}

// halfedges is a read-view of the Impl's halfedge arrays (Go port of the C++
// Halfedges accessor: Start/End/Pair) over the bridge-exposed slices. End(e) is
// the start of the next halfedge in the triangle (the manifold invariant), as
// elsewhere in the Go port.
type halfedges struct {
	starts []int32
	pairs  []int32
}

func (h halfedges) Start(e int) int { return int(h.starts[e]) }
func (h halfedges) End(e int) int   { return int(h.starts[nextHalfedge(e)]) }
func (h halfedges) Pair(e int) int  { return int(h.pairs[e]) }

// nextHalfedge is the Go port of NextHalfedge (src/shared.h): the next halfedge
// within the same triangle, via Next3 ((e/3)*3 + Next3(e%3)).
func nextHalfedge(e int) int { return e - e%3 + Next3(e%3) }

// intersect is the Go port of Intersect (boolean3.cpp:34): the (x, y, z_a, z_b)
// of the crossing of the xy-projected segments aL-aR and bL-bR, in the kernel's
// bespoke smaller-magnitude interpolation form. NOT geom.Vec4.Lerp — the float
// expression order here is load-bearing and transcribed byte-for-byte.
func intersect(aL, aR, bL, bR geom.Vec3) geom.Vec4 {
	dyL := bL.Y - aL.Y
	dyR := bR.Y - aR.Y
	// C++ DEBUG_ASSERT(dyL*dyR <= 0): no-intersection guard, debug-only.
	useL := math.Abs(dyL) < math.Abs(dyR)
	dx := aR.X - aL.X
	num := dyR
	if useL {
		num = dyL
	}
	lambda := num / (dyL - dyR)
	if !isFinite(lambda) {
		lambda = 0.0
	}
	var xyzz geom.Vec4
	xBase := aR.X
	if useL {
		xBase = aL.X
	}
	xyzz.X = lambda*dx + xBase

	aDy := aR.Y - aL.Y
	bDy := bR.Y - bL.Y
	useA := math.Abs(aDy) < math.Abs(bDy)
	dyTerm := bDy
	if useA {
		dyTerm = aDy
	}
	var yBase float64
	if useL {
		yBase = bL.Y
		if useA {
			yBase = aL.Y
		}
	} else {
		yBase = bR.Y
		if useA {
			yBase = aR.Y
		}
	}
	xyzz.Y = lambda*dyTerm + yBase

	zaBase := aR.Z
	if useL {
		zaBase = aL.Z
	}
	xyzz.Z = lambda*(aR.Z-aL.Z) + zaBase

	zbBase := bR.Z
	if useL {
		zbBase = bL.Z
	}
	xyzz.W = lambda*(bR.Z-bL.Z) + zbBase
	return xyzz
}

// faceEdge is the Go port of the FaceEdge struct (boolean3.cpp:55): an edge of a
// triangle, oriented forward (start < end) with isForward recording whether the
// canonical halfedge was the forward one.
type faceEdge struct {
	edge      int
	start     int
	end       int
	isForward bool
}

// loadFaceEdges is the Go port of LoadFaceEdges (boolean3.cpp:62): the three
// edges of triangle tri, each canonicalized to start < end (taking the paired
// halfedge when the stored one runs backward).
func loadFaceEdges(h halfedges, tri int, edge *[3]faceEdge) {
	for i := 0; i < 3; i++ {
		halfedge := 3*tri + i
		start := h.Start(halfedge)
		end := h.Start(3*tri + Next3(i))
		if start < end {
			edge[i] = faceEdge{edge: halfedge, start: start, end: end, isForward: true}
		} else {
			edge[i] = faceEdge{edge: h.Pair(halfedge), start: end, end: start, isForward: false}
		}
	}
}
