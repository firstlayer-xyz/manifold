package boolean

import (
	"sort"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// Halfedge mirrors the C++ Halfedge (shared.h:174): a directed half-edge with
// its start/end verts, paired halfedge, and property vert.
type Halfedge struct {
	StartVert, EndVert, PairedHalfedge, PropVert int
}

// TriRef mirrors the C++ TriRef (shared.h:288): the provenance of an output
// triangle — its mesh-instance id, original mesh id, face id, and coplanar id.
type TriRef struct {
	MeshID, OriginalID, FaceID, CoplanarID int
}

// pairUp is the Go port of PairUp (boolean_result.cpp:285): pair start verts
// with end verts to form edges. Partition the starts to the front, stable-sort
// each half by EdgePos order (pos, then collisionID), then pair edgePos[i] with
// edgePos[i+nEdges] and emit a Halfedge per pair. The C++ std::partition is
// unstable; a stable partition here is deterministic and equivalent — the
// pairing is "arbitrary for the manifoldness guarantee" and the result is
// compared semantically.
func pairUp(positions []edgePos, f func(Halfedge)) {
	nEdges := len(positions) / 2
	// Stable partition: starts first, preserving order.
	parted := make([]edgePos, 0, len(positions))
	for _, e := range positions {
		if e.isStart {
			parted = append(parted, e)
		}
	}
	for _, e := range positions {
		if !e.isStart {
			parted = append(parted, e)
		}
	}
	copy(positions, parted)

	starts, ends := positions[:nEdges], positions[nEdges:]
	sortEdgePos(starts)
	sortEdgePos(ends)
	for i := 0; i < nEdges; i++ {
		f(Halfedge{StartVert: starts[i].vert, EndVert: ends[i].vert, PairedHalfedge: -1})
	}
}

// absInt is std::abs for int.
func absInt(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// edgePos is the Go port of the EdgePos struct (boolean_result.cpp:191): a new
// vertex along a retained edge, with its position along the edge (pos, filled
// later in AppendPartialEdges), output vert index, originating crossing id
// (for deterministic tie-break), and whether it starts a span. Its ordering is
// (pos, then collisionID) — applied in PairUp.
type edgePos struct {
	pos         float64
	vert        int
	collisionID int
	isStart     bool
}

// edgePosLess mirrors EdgePos::operator< (boolean_result.cpp:197): order by pos
// (position along the edge), breaking ties by collisionID for determinism.
func edgePosLess(a, b edgePos) bool {
	return a.pos < b.pos || (a.pos == b.pos && a.collisionID < b.collisionID)
}

// sortEdgePos is std::stable_sort over EdgePos (by EdgePos::operator<).
func sortEdgePos(s []edgePos) {
	sort.SliceStable(s, func(i, j int) bool { return edgePosLess(s[i], s[j]) })
}

// addNewEdgeVerts is the Go port of AddNewEdgeVerts (boolean_result.cpp:204):
// for each edge(P)/face(Q) crossing, record the new vertex on P's edge
// (edgesP[edgeP]) and on the two new edges where Q's face meets the two P-faces
// sharing edgeP (edgesNew[keyLeft/keyRight]); the span direction comes from the
// crossing inclusion sign, duplicated |inclusion| times, with v12R remapping to
// the output vert index. forward=false mirrors P<->Q. SEQ path: the C++
// per-key mutex locking and its hashes are parallel-only, so they're dropped;
// &edgesP[k] (addressable map value) becomes append-to-map.
func addNewEdgeVerts(edgesP map[int][]edgePos, edgesNew map[[2]int][]edgePos,
	p1q2 [][2]int, i12, v12R []int, halfedgeP halfedges, forward bool, offset int) {
	for i := range p1q2 {
		edgeSlot, faceSlot := 0, 1
		if !forward {
			edgeSlot, faceSlot = 1, 0
		}
		edgeP := p1q2[i][edgeSlot]
		faceQ := p1q2[i][faceSlot]
		vert := v12R[i]
		inclusion := i12[i]

		keyRight := [2]int{halfedgeP.Pair(edgeP) / 3, faceQ}
		keyLeft := [2]int{edgeP / 3, faceQ}
		if !forward {
			keyRight[0], keyRight[1] = keyRight[1], keyRight[0]
			keyLeft[0], keyLeft[1] = keyLeft[1], keyLeft[0]
		}

		direction := inclusion < 0
		// isStart per target (C++ uses bool XOR ^; Go uses !=). The dead
		// `direction = !direction` loop tail is omitted (the targets capture
		// isStart by value).
		push := func(vec []edgePos, isStart bool) []edgePos {
			for j := 0; j < absInt(inclusion); j++ {
				vec = append(vec, edgePos{pos: 0.0, vert: vert + j, collisionID: i + offset, isStart: isStart})
			}
			return vec
		}
		edgesP[edgeP] = push(edgesP[edgeP], direction)
		edgesNew[keyRight] = push(edgesNew[keyRight], direction != !forward)
		edgesNew[keyLeft] = push(edgesNew[keyLeft], direction != forward)
	}
}

// appendPartialEdges is the Go port of AppendPartialEdges (boolean_result.cpp:304):
// each edge in edgesP is partially retained. For each, look up its original verts
// and include them by winding number (i03), remapped to the output via vP2R; place
// every vert along the edge vector (dot with edgeVec); PairUp the spans; and
// distribute the resulting halfedges to the two faces sharing the edge. forward=false
// mirrors P<->Q. SEQ path: the per-iter IsCancelled checks are parallel-cancellation
// only, so they're dropped; Vec<char> wholeHalfedgeP is a []bool; concurrent_map is
// std::map (ordered), so edgesP is iterated in sorted key order; the C++ copies
// value.second by value, so we copy the slice before sorting/appending.
func appendPartialEdges(outR *outImpl, halfedgeR []Halfedge, wholeHalfedgeP []bool,
	facePtrR []int, edgesP map[int][]edgePos, halfedgeRef []TriRef, inP *mesh,
	i03, vP2R, faceP2R []int, forward bool) {
	vertPosP := inP.vertPos
	halfedgeP := inP.halfedge

	edges := make([]int, 0, len(edgesP))
	for edgeP := range edgesP {
		edges = append(edges, edgeP)
	}
	sort.Ints(edges)

	for _, edgeP := range edges {
		edgePosP := append([]edgePos(nil), edgesP[edgeP]...)
		sortEdgePos(edgePosP)

		pairP := halfedgeP.Pair(edgeP)
		wholeHalfedgeP[edgeP] = false
		wholeHalfedgeP[pairP] = false

		vStart := halfedgeP.Start(edgeP)
		vEnd := halfedgeP.End(edgeP)
		edgeVec := vertPosP[vEnd].Sub(vertPosP[vStart])
		// Fill in the edge positions of the old points.
		for i := range edgePosP {
			edgePosP[i].pos = outR.vertPos[edgePosP[i].vert].Dot(edgeVec)
		}

		inclusion := i03[vStart]
		ep := edgePos{pos: outR.vertPos[vP2R[vStart]].Dot(edgeVec), vert: vP2R[vStart],
			collisionID: intMax, isStart: inclusion > 0}
		for j := 0; j < absInt(inclusion); j++ {
			edgePosP = append(edgePosP, ep)
			ep.vert++
		}

		inclusion = i03[vEnd]
		ep = edgePos{pos: outR.vertPos[vP2R[vEnd]].Dot(edgeVec), vert: vP2R[vEnd],
			collisionID: intMax, isStart: inclusion < 0}
		for j := 0; j < absInt(inclusion); j++ {
			edgePosP = append(edgePosP, ep)
			ep.vert++
		}

		// add halfedges to result
		faceLeftP := edgeP / 3
		faceLeft := faceP2R[faceLeftP]
		faceRightP := pairP / 3
		faceRight := faceP2R[faceRightP]
		// Negative inclusion means the halfedges are reversed, which means our
		// reference is now to the endVert instead of the startVert, which is one
		// position advanced CCW. This is only valid if this is a retained vert; it
		// will be ignored later if the vert is new.
		meshID := 1
		if forward {
			meshID = 0
		}
		forwardRef := TriRef{MeshID: meshID, OriginalID: -1, FaceID: faceLeftP, CoplanarID: -1}
		backwardRef := TriRef{MeshID: meshID, OriginalID: -1, FaceID: faceRightP, CoplanarID: -1}

		pairUp(edgePosP, func(e Halfedge) {
			forwardEdge := facePtrR[faceLeft]
			facePtrR[faceLeft]++
			backwardEdge := facePtrR[faceRight]
			facePtrR[faceRight]++

			e.PairedHalfedge = backwardEdge
			halfedgeR[forwardEdge] = e
			halfedgeRef[forwardEdge] = forwardRef

			e.StartVert, e.EndVert = e.EndVert, e.StartVert
			e.PairedHalfedge = forwardEdge
			halfedgeR[backwardEdge] = e
			halfedgeRef[backwardEdge] = backwardRef
		})
	}
}

// outImpl is the output Manifold::Impl being assembled by Boolean3::Result.
// Fields are filled as the assembly progresses; more are added as later helpers
// land (halfedge, triRef, properties).
type outImpl struct {
	vertPos            []geom.Vec3
	faceNormal         []geom.Vec3
	epsilon, tolerance float64
}

// sizeOutput is the Go port of SizeOutput (boolean_result.cpp:104): count the
// output sides per retained face (from the retained-vertex inclusions i03/i30
// and the new-vertex crossings i12/i21), drop faces with no sides, and build
//   - facePQ2R: old (P|Q) face index -> new face index (inclusive scan of the
//     keep flag), sized numTriP+numTriQ;
//   - faceEdge:  prefix sum of the kept faces' side counts (the per-face edge
//     offset into the output halfedge buffer);
//   - outR.faceNormal: the kept faces' normals (Q's negated when invertQ).
//
// SEQ path (the atomic CountNewVerts variant is the parallel optimization).
func sizeOutput(outR *outImpl, inP, inQ *mesh, i03, i30, i12, i21 []int, p1q2, p2q1 [][2]int, invertQ bool) (faceEdge, facePQ2R []int) {
	numTriP := len(inP.halfedge.starts) / 3
	numTriQ := len(inQ.halfedge.starts) / 3
	sidesPerFacePQ := make([]int, numTriP+numTriQ)
	sidesPerFaceP := sidesPerFacePQ[:numTriP]
	sidesPerFaceQ := sidesPerFacePQ[numTriP:]

	for i := 0; i < numTriP; i++ {
		countVerts(inP.halfedge, sidesPerFaceP, i03, i)
	}
	for i := 0; i < numTriQ; i++ {
		countVerts(inQ.halfedge, sidesPerFaceQ, i30, i)
	}
	for idx := range i12 {
		countNewVerts(sidesPerFaceP, sidesPerFaceQ, i12, p1q2, inP.halfedge, idx, false)
	}
	for idx := range i21 {
		countNewVerts(sidesPerFaceQ, sidesPerFaceP, i21, p2q1, inQ.halfedge, idx, true)
	}

	// facePQ2R = inclusive_scan of keepFace (sides>0 ? 1 : 0), into [1:].
	facePQ2R = make([]int, numTriP+numTriQ+1)
	acc := 0
	for i := 0; i < len(sidesPerFacePQ); i++ {
		if sidesPerFacePQ[i] > 0 {
			acc++
		}
		facePQ2R[i+1] = acc
	}
	numFaceR := facePQ2R[len(facePQ2R)-1]
	facePQ2R = facePQ2R[:numTriP+numTriQ]

	// faceNormal: kept P faces, then kept Q faces (negated if invertQ).
	outR.faceNormal = make([]geom.Vec3, numFaceR)
	w := 0
	for i := 0; i < numTriP; i++ {
		if sidesPerFacePQ[i] > 0 {
			outR.faceNormal[w] = inP.faceNormal[i]
			w++
		}
	}
	for i := 0; i < numTriQ; i++ {
		if sidesPerFacePQ[numTriP+i] > 0 {
			n := inQ.faceNormal[i]
			if invertQ {
				n = n.Scale(-1)
			}
			outR.faceNormal[w] = n
			w++
		}
	}

	// faceEdge = [0] + inclusive_scan of the kept faces' side counts (the C++
	// remove-zeros then scan).
	var nonZero []int
	for _, s := range sidesPerFacePQ {
		if s != 0 {
			nonZero = append(nonZero, s)
		}
	}
	faceEdge = make([]int, len(nonZero)+1)
	acc = 0
	for k, s := range nonZero {
		acc += s
		faceEdge[k+1] = acc
	}
	return faceEdge, facePQ2R
}

// duplicateVerts is the Go port of the DuplicateVerts functor
// (boolean_result.cpp:54): scatter source vertex `vert` into vertPosR at
// vertR[vert], repeated |inclusion[vert]| times (inclusion not in [-1,1] means
// the vertex is duplicated for multiple output components).
func duplicateVerts(vertPosR []geom.Vec3, inclusion, vertR []int, vertPosP []geom.Vec3, vert int) {
	n := absInt(inclusion[vert])
	for i := 0; i < n; i++ {
		vertPosR[vertR[vert]+i] = vertPosP[vert]
	}
}

// countVerts is the Go port of the CountVerts functor (boolean_result.cpp:68):
// count[i] += the sum over tri i's three start verts of |inclusion[startVert]|.
func countVerts(h halfedges, count, inclusion []int, i int) {
	for j := 0; j < 3; j++ {
		count[i] += absInt(inclusion[h.Start(3*i+j)])
	}
}

// countNewVerts is the Go port of the CountNewVerts functor
// (boolean_result.cpp:79), non-atomic SEQ form: for crossing idx, add
// |i12[idx]| to the face countQ[faceQ] and to both P-faces incident on edgeP
// (edgeP/3 and Pair(edgeP)/3). `inverted` selects the edge/face slots of pq
// (false: edge=pq[0],face=pq[1]; true: edge=pq[1],face=pq[0]).
func countNewVerts(countP, countQ, i12 []int, pq [][2]int, h halfedges, idx int, inverted bool) {
	edgeSlot, faceSlot := 0, 1
	if inverted {
		edgeSlot, faceSlot = 1, 0
	}
	edgeP := pq[idx][edgeSlot]
	faceQ := pq[idx][faceSlot]
	inclusion := absInt(i12[idx])
	countQ[faceQ] += inclusion
	countP[edgeP/3] += inclusion
	countP[h.Pair(edgeP)/3] += inclusion
}
