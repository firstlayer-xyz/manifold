package boolean

import (
	"math"
	"sort"

	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
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

// appendNewEdges is the Go port of AppendNewEdges (boolean_result.cpp:385): for
// each new edge (a P-face x Q-face pair in edgesNew), order its verts along the
// bounding box's longest dimension, PairUp the spans, and distribute the halfedges
// to the two faces named by the key. concurrent_map is std::map (ordered), so
// edgesNew is iterated in lexicographic (faceP, faceQ) key order; value.second is a
// reference, so the slice is sorted/positioned in place. IsCancelled is parallel-only.
func appendNewEdges(outR *outImpl, halfedgeR []Halfedge, facePtrR []int,
	edgesNew map[[2]int][]edgePos, halfedgeRef []TriRef, facePQ2R []int, numFaceP int) {
	vertPosR := outR.vertPos

	keys := make([][2]int, 0, len(edgesNew))
	for k := range edgesNew {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(i, j int) bool {
		return keys[i][0] < keys[j][0] || (keys[i][0] == keys[j][0] && keys[i][1] < keys[j][1])
	})

	for _, key := range keys {
		faceP := key[0]
		faceQ := key[1]
		positions := edgesNew[key]
		sortEdgePos(positions)

		bbox := emptyBox()
		for _, edge := range positions {
			bbox = bbox.UnionPoint(vertPosR[edge.vert])
		}
		size := bbox.Size()
		// Order the points along their longest dimension.
		i := 2
		if size.X > size.Y && size.X > size.Z {
			i = 0
		} else if size.Y > size.Z {
			i = 1
		}
		for j := range positions {
			positions[j].pos = axis(vertPosR[positions[j].vert], i)
		}

		// add halfedges to result
		faceLeft := facePQ2R[faceP]
		faceRight := facePQ2R[numFaceP+faceQ]
		forwardRef := TriRef{MeshID: 0, OriginalID: -1, FaceID: faceP, CoplanarID: -1}
		backwardRef := TriRef{MeshID: 1, OriginalID: -1, FaceID: faceQ, CoplanarID: -1}
		pairUp(positions, func(e Halfedge) {
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

// appendWholeEdges is the Go port of AppendWholeEdges (boolean_result.cpp:490) with
// its DuplicateHalfedges functor (boolean_result.cpp:437) inlined as a sequential
// for-each over inP's halfedges: each wholly-retained forward halfedge (startVert <
// endVert, nonzero winding) is emitted |i03[startVert]| times — duplicated for
// multiple output components, reversed when winding is negative — remapped via vP2R
// into the two faces sharing it. AtomicAdd is a plain post-increment in the SEQ path;
// the C++ outR parameter is unused and dropped.
func appendWholeEdges(facePtrR []int, halfedgesR []Halfedge, halfedgeRef []TriRef,
	inP *mesh, wholeHalfedgeP []bool, i03, vP2R, faceP2R []int, forward bool) {
	halfedgesP := inP.halfedge
	for idx := 0; idx < len(halfedgesP.starts); idx++ {
		if !wholeHalfedgeP[idx] {
			continue
		}
		startVert := halfedgesP.Start(idx)
		endVert := halfedgesP.Start(nextHalfedge(idx))
		if startVert >= endVert {
			continue
		}
		inclusion := i03[startVert]
		if inclusion == 0 {
			continue
		}
		if inclusion < 0 { // reverse
			startVert, endVert = endVert, startVert
		}
		startVert = vP2R[startVert]
		endVert = vP2R[endVert]
		propVert := halfedgesP.Prop(idx)
		pair := halfedgesP.Pair(idx)
		pairPropVert := halfedgesP.Prop(pair)
		faceLeftP := idx / 3
		newFace := faceP2R[faceLeftP]
		faceRightP := pair / 3
		faceRight := faceP2R[faceRightP]
		// Negative inclusion means the halfedges are reversed, which means our
		// reference is now to the endVert instead of the startVert, which is one
		// position advanced CCW.
		meshID := 1
		if forward {
			meshID = 0
		}
		forwardRef := TriRef{MeshID: meshID, OriginalID: -1, FaceID: faceLeftP, CoplanarID: -1}
		backwardRef := TriRef{MeshID: meshID, OriginalID: -1, FaceID: faceRightP, CoplanarID: -1}

		for i := 0; i < absInt(inclusion); i++ {
			forwardEdge := facePtrR[newFace]
			facePtrR[newFace]++
			backwardEdge := facePtrR[faceRight]
			facePtrR[faceRight]++

			halfedgesR[forwardEdge] = Halfedge{StartVert: startVert, EndVert: endVert, PairedHalfedge: backwardEdge, PropVert: propVert}
			halfedgesR[backwardEdge] = Halfedge{StartVert: endVert, EndVert: startVert, PairedHalfedge: forwardEdge, PropVert: pairPropVert}
			halfedgeRef[forwardEdge] = forwardRef
			halfedgeRef[backwardEdge] = backwardRef

			startVert++
			endVert++
		}
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

// Assembly is the pre-triangulation output of the Boolean assembly — everything
// Boolean3::Result computes before Face2Tri (boolean_result.cpp:776-919): the
// output vertices and per-face normals, the per-face halfedge ranges (faceEdge)
// with the arbitrary-polygon faceHalfedges and their halfedgeRef provenance, the
// output epsilon/tolerance, the total retained-vert count numVertR, and nPvNQv
// (nPv+nQv, the original-vertex count SimplifyTopology later needs). The manifold
// package feeds these into Impl.Face2Tri and the rest of the Impl tail.
type Assembly struct {
	VertPos       []geom.Vec3
	FaceNormal    []geom.Vec3
	FaceEdge      []int
	FaceHalfedges []Halfedge
	HalfedgeRef   []TriRef
	Epsilon       float64
	Tolerance     float64
	NumVertR      int
	NPvNQv        int
}

// assemble is the pure, pre-triangulation half of Boolean3::Result
// (boolean_result.cpp:776-919): convert the winding numbers to inclusion values
// (c1/c2/c3 from the op), number the output verts via exclusive_scan with AbsSum,
// duplicate the retained and new verts, accumulate the new edge verts, size the
// output, and assemble the polygonal faceHalfedges + halfedgeRef. ok is false when
// numVertR == 0 (the empty-result early return). The Impl tail (Face2Tri onward) is
// the caller's job. SEQ path throughout.
func (b *Boolean3) assemble(c1, c2, c3 int, invertQ bool) (Assembly, bool) {
	inP, inQ := b.p.mesh(), b.q.mesh()

	// Convert winding numbers to inclusion values based on operation type.
	i12 := make([]int, len(b.xv12.x12))
	i21 := make([]int, len(b.xv21.x12))
	i03 := make([]int, len(b.w03))
	i30 := make([]int, len(b.w30))
	for i, v := range b.xv12.x12 {
		i12[i] = c3 * v
	}
	for i, v := range b.xv21.x12 {
		i21[i] = c3 * v
	}
	for i, v := range b.w03 {
		i03[i] = c1 + c3*v
	}
	for i, v := range b.w30 {
		i30[i] = c2 + c3*v
	}

	absSum := func(a, b int) int { return absInt(a) + absInt(b) }

	vP2R := make([]int, len(inP.vertPos))
	parallel.ExclusiveScanFunc(parallel.Seq, i03, vP2R, 0, absSum)
	numVertR := absSum(vP2R[len(vP2R)-1], i03[len(i03)-1])
	nPv := numVertR

	vQ2R := make([]int, len(inQ.vertPos))
	parallel.ExclusiveScanFunc(parallel.Seq, i30, vQ2R, numVertR, absSum)
	numVertR = absSum(vQ2R[len(vQ2R)-1], i30[len(i30)-1])
	nQv := numVertR - nPv

	v12R := make([]int, len(b.xv12.v12))
	if len(b.xv12.v12) > 0 {
		parallel.ExclusiveScanFunc(parallel.Seq, i12, v12R, numVertR, absSum)
		numVertR = absSum(v12R[len(v12R)-1], i12[len(i12)-1])
	}

	v21R := make([]int, len(b.xv21.v12))
	if len(b.xv21.v12) > 0 {
		parallel.ExclusiveScanFunc(parallel.Seq, i21, v21R, numVertR, absSum)
		numVertR = absSum(v21R[len(v21R)-1], i21[len(i21)-1])
	}

	// Create the output Manifold
	outR := &outImpl{}

	if numVertR == 0 {
		return Assembly{}, false
	}

	outR.epsilon = math.Max(b.p.Epsilon, b.q.Epsilon)
	outR.tolerance = math.Max(b.p.Tolerance, b.q.Tolerance)

	outR.vertPos = make([]geom.Vec3, numVertR)
	// Add vertices, duplicating for inclusion numbers not in [-1, 1].
	// Retained vertices from P and Q:
	for v := range inP.vertPos {
		duplicateVerts(outR.vertPos, i03, vP2R, inP.vertPos, v)
	}
	for v := range inQ.vertPos {
		duplicateVerts(outR.vertPos, i30, vQ2R, inQ.vertPos, v)
	}
	// New vertices created from intersections:
	for v := range i12 {
		duplicateVerts(outR.vertPos, i12, v12R, b.xv12.v12, v)
	}
	for v := range i21 {
		duplicateVerts(outR.vertPos, i21, v21R, b.xv21.v12, v)
	}

	// Level 3
	// edgesP/edgesQ key on the forward halfedge index of P or Q; edgesNew keys on
	// the <P, Q> face pair.
	edgesP := map[int][]edgePos{}
	edgesQ := map[int][]edgePos{}
	edgesNew := map[[2]int][]edgePos{}

	addNewEdgeVerts(edgesP, edgesNew, b.xv12.p1q2, i12, v12R, inP.halfedge, true, 0)
	addNewEdgeVerts(edgesQ, edgesNew, b.xv21.p1q2, i21, v21R, inQ.halfedge, false, len(b.xv12.p1q2))

	// Level 4
	faceEdge, facePQ2R := sizeOutput(outR, inP, inQ, i03, i30, i12, i21, b.xv12.p1q2, b.xv21.p1q2, invertQ)

	numTriP := len(inP.halfedge.starts) / 3
	numTriQ := len(inQ.halfedge.starts) / 3

	// facePtrR is incremented for each halfedge added to a face so the next one
	// knows where to slot in (a copy of faceEdge; faceEdge is kept for Face2Tri).
	facePtrR := append([]int(nil), faceEdge...)
	// Intersected halfedges are marked false.
	wholeHalfedgeP := make([]bool, len(inP.halfedge.starts))
	wholeHalfedgeQ := make([]bool, len(inQ.halfedge.starts))
	for i := range wholeHalfedgeP {
		wholeHalfedgeP[i] = true
	}
	for i := range wholeHalfedgeQ {
		wholeHalfedgeQ[i] = true
	}
	// halfedgeRef becomes triRef once the faces are triangulated; faceHalfedges
	// holds arbitrary polygons before the triangulator.
	halfedgeRef := make([]TriRef, faceEdge[len(faceEdge)-1])
	faceHalfedges := make([]Halfedge, faceEdge[len(faceEdge)-1])

	appendPartialEdges(outR, faceHalfedges, wholeHalfedgeP, facePtrR, edgesP, halfedgeRef, inP, i03, vP2R, facePQ2R, true)
	appendPartialEdges(outR, faceHalfedges, wholeHalfedgeQ, facePtrR, edgesQ, halfedgeRef, inQ, i30, vQ2R, facePQ2R[numTriP:], false)

	appendNewEdges(outR, faceHalfedges, facePtrR, edgesNew, halfedgeRef, facePQ2R, numTriP)

	appendWholeEdges(facePtrR, faceHalfedges, halfedgeRef, inP, wholeHalfedgeP, i03, vP2R, facePQ2R[:numTriP], true)
	appendWholeEdges(facePtrR, faceHalfedges, halfedgeRef, inQ, wholeHalfedgeQ, i30, vQ2R, facePQ2R[numTriP:numTriP+numTriQ], false)

	return Assembly{
		VertPos:       outR.vertPos,
		FaceNormal:    outR.faceNormal,
		FaceEdge:      faceEdge,
		FaceHalfedges: faceHalfedges,
		HalfedgeRef:   halfedgeRef,
		Epsilon:       outR.epsilon,
		Tolerance:     outR.tolerance,
		NumVertR:      numVertR,
		NPvNQv:        nPv + nQv,
	}, true
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
