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

func nanVec3() geom.Vec3 { n := math.NaN(); return geom.Vec3{X: n, Y: n, Z: n} }
func nanVec4() geom.Vec4 { n := math.NaN(); return geom.Vec4{X: n, Y: n, Z: n, W: n} }

// kernel11 is the Go port of Kernel11<expandP> (boolean3.cpp:108): the 1-vs-1
// (edge/edge) intersection — overlap count s11 and crossing xyzz11. The C++
// local bool `shadows` is `sh` here (the predicate shadows() shares the name).
type kernel11 struct {
	inP, inQ *mesh
	expandP  bool
}

func (k kernel11) call(p1, p1s, p1e, q1, q1s, q1e int) (int, geom.Vec4) {
	xyzz11 := nanVec4()
	s11 := 0

	// For pRL[k], qRL[k], k==0 is left, k==1 is right.
	kk := 0
	var pRL, qRL [2]geom.Vec3
	// Either left or right must shadow, but not both, ensuring the intersection
	// is between left and right.
	sh := false
	s11 = 0

	p0 := [2]int{p1s, p1e}
	for i := 0; i < 2; i++ {
		s01, yz01 := shadow01(k.expandP, true, p0[i], q1, q1s, q1e, k.inP, k.inQ)
		// NaN means no overlap.
		if isFinite(yz01.X) {
			sign := 1
			if i == 0 {
				sign = -1
			}
			s11 += s01 * sign
			if kk < 2 && (kk == 0 || (s01 != 0) != sh) {
				sh = s01 != 0
				pRL[kk] = k.inP.vertPos[p0[i]]
				qRL[kk] = geom.Vec3{X: pRL[kk].X, Y: yz01.X, Z: yz01.Y}
				kk++
			}
		}
	}

	q0 := [2]int{q1s, q1e}
	for i := 0; i < 2; i++ {
		s10, yz10 := shadow01(k.expandP, false, q0[i], p1, p1s, p1e, k.inQ, k.inP)
		if isFinite(yz10.X) {
			sign := 1
			if i == 0 {
				sign = -1
			}
			s11 += s10 * sign
			if kk < 2 && (kk == 0 || (s10 != 0) != sh) {
				sh = s10 != 0
				qRL[kk] = k.inQ.vertPos[q0[i]]
				pRL[kk] = geom.Vec3{X: qRL[kk].X, Y: yz10.X, Z: yz10.Y}
				kk++
			}
		}
	}

	if s11 == 0 { // No intersection
		xyzz11 = nanVec4()
	} else {
		// DEBUG_ASSERT(kk == 2)
		xyzz11 = intersect(pRL[0], pRL[1], qRL[0], qRL[1])

		p1pair := k.inP.halfedge.Pair(p1)
		dirP := k.inP.faceNormal[p1/3].Z + k.inP.faceNormal[p1pair/3].Z
		q1pair := k.inQ.halfedge.Pair(q1)
		dirQ := k.inQ.faceNormal[q1/3].Z + k.inQ.faceNormal[q1pair/3].Z
		if !shadows(xyzz11.Z, xyzz11.W, withSign(k.expandP, dirP)-dirQ) {
			s11 = 0
		}
	}
	return s11, xyzz11
}

// kernel02 is the Go port of Kernel02<expandP, forward> (boolean3.cpp:177): the
// 0-vs-2 (vertex/face) winding — overlap count s02 and z-coordinate z02.
type kernel02 struct {
	inA, inB *mesh
	expandP  bool
	forward  bool
}

func (k kernel02) call(a0, b2 int) (int, float64) {
	var edgeB [3]faceEdge
	loadFaceEdges(k.inB.halfedge, b2, &edgeB)
	return k.callEdges(a0, b2, &edgeB)
}

func (k kernel02) callEdges(a0, b2 int, edgeB *[3]faceEdge) (int, float64) {
	s02 := 0
	z02 := 0.0

	kk := 0
	var yzzRL [2]geom.Vec3
	sh := false

	for i := 0; i < 3; i++ {
		s01, yz01 := shadow01(k.expandP, k.forward, a0, edgeB[i].edge, edgeB[i].start, edgeB[i].end, k.inA, k.inB)
		if isFinite(yz01.X) {
			sign := 1
			if k.forward == edgeB[i].isForward {
				sign = -1
			}
			s02 += s01 * sign
			if kk < 2 && (kk == 0 || (s01 != 0) != sh) {
				sh = s01 != 0
				yzzRL[kk] = geom.Vec3{X: yz01.X, Y: yz01.Y, Z: yz01.Y}
				kk++
			}
		}
	}

	if s02 == 0 { // No intersection
		z02 = math.NaN()
	} else {
		// DEBUG_ASSERT(kk == 2)
		vertPosA := k.inA.vertPos[a0]
		z02 = interpolate(yzzRL[0], yzzRL[1], vertPosA.Y).Y // [1] = z
		if k.forward {
			if !shadows(vertPosA.Z, z02, -k.inB.faceNormal[b2].Z) {
				s02 = 0
			}
		} else {
			if !shadows(z02, vertPosA.Z, withSign(k.expandP, k.inB.faceNormal[b2].Z)) {
				s02 = 0
			}
		}
	}
	return s02, z02
}

// kernel12 is the Go port of Kernel12<expandP, forward> (boolean3.cpp:231): the
// 1-vs-2 (edge/face) intersection — overlap count x12 and the 3D point v12,
// composing kernel02 + kernel11. The coordinate reorders (256-301) are the
// riskiest transcription and are mapped index-for-index ([0]=X,[1]=Y,[2]=Z).
type kernel12 struct {
	inA, inB *mesh
	forward  bool
	k02      kernel02
	k11      kernel11
}

// newKernel12 wires a kernel12 from the two operands, mirroring the
// Intersect12_ instantiation (boolean3.cpp:367-373): a = edge mesh, b = face
// mesh (swapped by forward); k02{a,b}; k11{inP,inQ} (the originals, NOT a/b, so
// p1 is always from inP regardless of forward).
func newKernel12(inP, inQ *mesh, expandP, forward bool) kernel12 {
	a, b := inP, inQ
	if !forward {
		a, b = inQ, inP
	}
	return kernel12{
		inA:     a,
		inB:     b,
		forward: forward,
		k02:     kernel02{inA: a, inB: b, expandP: expandP, forward: forward},
		k11:     kernel11{inP: inP, inQ: inQ, expandP: expandP},
	}
}

func (k kernel12) call(a1, b2 int) (int, geom.Vec3) {
	x12 := 0
	v12 := nanVec3()

	// For xzyLR-[k], k==0 is left, k==1 is right.
	kk := 0
	var xzyLR0, xzyLR1 [2]geom.Vec3
	sh := false
	x12 = 0

	edgeAStart := k.inA.halfedge.Start(a1)
	edgeAEnd := k.inA.halfedge.End(a1)
	var edgeB [3]faceEdge
	loadFaceEdges(k.inB.halfedge, b2, &edgeB)

	for _, vertA := range [2]int{edgeAStart, edgeAEnd} {
		s, z := k.k02.callEdges(vertA, b2, &edgeB)
		if isFinite(z) {
			sign := -1
			if (vertA == edgeAStart) == k.forward {
				sign = 1
			}
			x12 += s * sign
			if kk < 2 && (kk == 0 || (s != 0) != sh) {
				sh = s != 0
				xzyLR0[kk] = k.inA.vertPos[vertA]
				xzyLR0[kk].Y, xzyLR0[kk].Z = xzyLR0[kk].Z, xzyLR0[kk].Y // std::swap(y, z)
				xzyLR1[kk] = xzyLR0[kk]
				xzyLR1[kk].Y = z // [1] = z
				kk++
			}
		}
	}

	for i := 0; i < 3; i++ {
		var s int
		var xyzz geom.Vec4
		if k.forward {
			s, xyzz = k.k11.call(a1, edgeAStart, edgeAEnd, edgeB[i].edge, edgeB[i].start, edgeB[i].end)
		} else {
			s, xyzz = k.k11.call(edgeB[i].edge, edgeB[i].start, edgeB[i].end, a1, edgeAStart, edgeAEnd)
		}
		if isFinite(xyzz.X) {
			sign := -1
			if edgeB[i].isForward {
				sign = 1
			}
			x12 -= s * sign
			if kk < 2 && (kk == 0 || (s != 0) != sh) {
				sh = s != 0
				xzyLR0[kk].X = xyzz.X // [0] = x
				xzyLR0[kk].Y = xyzz.Z // [1] = z
				xzyLR0[kk].Z = xyzz.Y // [2] = y
				xzyLR1[kk] = xzyLR0[kk]
				xzyLR1[kk].Y = xyzz.W // [1] = w
				if !k.forward {
					xzyLR0[kk].Y, xzyLR1[kk].Y = xzyLR1[kk].Y, xzyLR0[kk].Y // std::swap([1],[1])
				}
				kk++
			}
		}
	}

	if x12 == 0 { // No intersection
		v12 = nanVec3()
	} else {
		// DEBUG_ASSERT(kk == 2)
		xzyy := intersect(xzyLR0[0], xzyLR0[1], xzyLR1[0], xzyLR1[1])
		v12.X = xzyy.X // [0]
		v12.Y = xzyy.Z // [2]
		v12.Z = xzyy.Y // [1]
	}
	return x12, v12
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
