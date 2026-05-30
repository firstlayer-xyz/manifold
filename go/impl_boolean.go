package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/boolean"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// boolOperand builds the boolean.Operand the native Boolean3 reads from a const
// Impl: the kernel arrays (aliasing C++ memory — the Impl must outlive the
// Boolean3), the bbox, a fresh face collider, and epsilon/tolerance.
func boolOperand(v *Impl) boolean.Operand {
	min, max := v.BBox()
	s := v.Scalars()
	return boolean.Operand{
		VertPos:    v.Verts(),
		VertNormal: v.VertNormals(),
		FaceNormal: v.FaceNormals(),
		Starts:     v.HalfedgeStarts(),
		Pairs:      v.HalfedgePairs(),
		PropVert:   v.HalfedgeProps(),
		BBox:       geom.Box{Min: min, Max: max},
		// Persistent native collider (native-Impl-storage Phase 1): C++ reuses
		// the operand's Impl::collider_; a transformed operand carries a refitted
		// one, a finalized operand lazily builds from its (sorted) faces.
		Collider:  v.ensureCollider(),
		Epsilon:   s.Epsilon,
		Tolerance: s.Tolerance,
	}
}

// nativeBoolean3 is the native counterpart of bridge.Boolean3: it holds the
// computed boolean.Boolean3 plus its two input Impls (the C++ Boolean3 holds
// const Impl& inP_/inQ_), so Result(op) can build the output Impl and run the
// finalize tail. A single nativeBoolean3 can serve two Result calls (Split).
type nativeBoolean3 struct {
	b3       *boolean.Boolean3
	inP, inQ *Impl
}

// newNativeBoolean3 is the Go port of the Boolean3 ctor dispatch: expandP =
// (op == Add). The Operands alias the input Impls' memory, so inP/inQ must
// outlive the returned object.
func newNativeBoolean3(inP, inQ *Impl, op OpType) *nativeBoolean3 {
	b3 := boolean.NewBoolean3(boolOperand(inP), boolOperand(inQ), op == OpAdd)
	return &nativeBoolean3{b3: b3, inP: inP, inQ: inQ}
}

// Result is the Go port of Boolean3::Result (boolean_result.cpp:693): the native
// boolean.Result produces the triangulated mesh (assemble -> Face2Tri ->
// ReorderHalfedges); this builds the output Impl from it and runs the finalize
// tail (CreateProperties, UpdateReference, SimplifyTopology, RemoveUnreferenced-
// Verts, CalculateBBox, SortGeometry, IncrementMeshIDs).
func (nb *nativeBoolean3) Result(op OpType) *Manifold {
	// Input-status propagation (boolean_result.cpp:732-741): a non-NoError input
	// short-circuits with that status before the empty-input exits.
	if status := nb.inP.Scalars().Status; Error(status) != NoError {
		return propagateStatus(Error(status))
	}
	if status := nb.inQ.Scalars().Status; Error(status) != NoError {
		return propagateStatus(Error(status))
	}

	// Empty-input early exits (boolean_result.cpp:743-753).
	pEmpty := nb.inP.NumTri() == 0
	qEmpty := nb.inQ.NumTri() == 0
	if pEmpty {
		if !qEmpty && op == OpAdd {
			return nb.inQ.Copy().ToManifold()
		}
		return newImpl().ToManifold()
	} else if qEmpty {
		if op == OpIntersect {
			return newImpl().ToManifold()
		}
		return nb.inP.Copy().ToManifold()
	}
	if !nb.b3.Valid() {
		// boolean_result.cpp:770 ResultTooLarge. Status propagation lands with
		// the production Manifold.Boolean rewire; until then surface it loudly.
		panic("nativeBoolean3.Result: ResultTooLarge (INT_MAX crossing overflow)")
	}

	rm, ok := nb.b3.Result(int(op))
	if !ok {
		return newImpl().ToManifold() // numVertR == 0
	}
	invertQ := op == OpSubtract

	// Build the output Impl from the triangulated ResultMesh.
	mi := newImpl()
	mi.ResizeVerts(len(rm.VertPos))
	copy(mi.Verts(), rm.VertPos)
	mi.SetHalfedgesRaw(rm.Starts, rm.PropVert, rm.Pairs)
	mi.ResizeFaceNormals(len(rm.TriNormal))
	copy(mi.FaceNormals(), rm.TriNormal)
	setBoolTriRefs(mi, rm.TriRef)
	mi.SetEpsilonValue(rm.Epsilon)
	mi.SetToleranceValue(rm.Tolerance)

	// Finalize tail (boolean_result.cpp:951-972).
	createProperties(mi, nb.inP, nb.inQ, invertQ)
	updateReference(mi, nb.inP, nb.inQ, invertQ)
	mi.SimplifyTopology(rm.NPvNQv)
	mi.RemoveUnreferencedVerts()
	mi.CalculateBBox()
	mi.SortGeometry()
	mi.IncrementMeshIDs()
	return mi.ToManifold()
}

// setBoolTriRefs writes the assembled (boolean.TriRef) refs into outR's
// meshRelation_.triRef — the {meshID 0/1, faceID = original P/Q face} form
// Face2Tri produced; UpdateReference remaps it to the real input refs.
func setBoolTriRefs(outR *MutableImpl, refs []boolean.TriRef) {
	meshIDs := make([]int32, len(refs))
	originalIDs := make([]int32, len(refs))
	faceIDs := make([]int32, len(refs))
	coplanarIDs := make([]int32, len(refs))
	for i, r := range refs {
		meshIDs[i] = int32(r.MeshID)
		originalIDs[i] = int32(r.OriginalID)
		faceIDs[i] = int32(r.FaceID)
		coplanarIDs[i] = int32(r.CoplanarID)
	}
	outR.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)
}

// propEntry is one std::pair<ivec3,int> in the propIdx dedup bins: the (PQ,
// minProp, maxProp) provenance key and the assigned output property index.
type propEntry struct {
	key [3]int
	idx int
}

// implNumPropVert mirrors Impl::NumPropVert: NumProp==0 ? NumVert : len/NumProp.
func implNumPropVert(v *Impl, numProp int) int {
	if numProp == 0 {
		return v.NumVert()
	}
	return len(v.Properties()) / numProp
}

// baryComp indexes a barycentric vec3 by component (the C++ uvw[j]).
func baryComp(v geom.Vec3, j int) float64 {
	switch j {
	case 0:
		return v.X
	case 1:
		return v.Y
	default:
		return v.Z
	}
}

// createProperties is the Go port of CreateProperties (boolean_result.cpp:571):
// interpolate per-vertex properties at the output verts via barycentric
// coordinates within the originating input triangle, deduplicating output
// property verts by provenance. numProp = max(numPropP, numPropQ); the
// numProp==0 case is a no-op after setting numProp_.
func createProperties(outR *MutableImpl, inP, inQ *Impl, invertQ bool) {
	numPropP := inP.Scalars().NumProp
	numPropQ := inQ.Scalars().NumProp
	numProp := numPropP
	if numPropQ > numProp {
		numProp = numPropQ
	}
	outR.SetNumProp(numProp)
	if numProp == 0 {
		return
	}

	// Output mesh arrays (copies — propVert is mutated, then written back).
	starts := append([]int32(nil), outR.HalfedgeStarts()...)
	pairs := append([]int32(nil), outR.HalfedgePairs()...)
	propVert := append([]int32(nil), outR.HalfedgeProps()...)
	vertPosR := outR.Verts()
	triRefR := outR.TriRefs()
	epsilon := outR.Epsilon()
	numTri := len(starts) / 3
	numVertR := len(vertPosR)

	// Inputs.
	vertPosP, vertPosQ := inP.Verts(), inQ.Verts()
	startsP, startsQ := inP.HalfedgeStarts(), inQ.HalfedgeStarts()
	propsP, propsQ := inP.Properties(), inQ.Properties()
	hPropsP, hPropsQ := inP.HalfedgeProps(), inQ.HalfedgeProps()

	// inQ TriHasNormals lookup (impl.h:77): triRef[tri].meshID -> hasNormals.
	qHasNormals := map[int32]bool{}
	for _, r := range inQ.MeshIDTransforms() {
		qHasNormals[r.MeshID] = r.HasNormals
	}
	qTriRef := inQ.TriRefs()
	triHasNormalsQ := func(tri int) bool { return qHasNormals[qTriRef[tri].MeshID] }

	// Barycentric pass (boolean_result.cpp:584-588): uvw per output halfedge.
	bary := make([]geom.Vec3, len(starts))
	for tri := 0; tri < numTri; tri++ {
		if starts[3*tri] < 0 {
			continue
		}
		refPQ := triRefR[tri]
		triPQ := int(refPQ.FaceID)
		pq := refPQ.MeshID == 0
		vertPos, hStarts := vertPosQ, startsQ
		if pq {
			vertPos, hStarts = vertPosP, startsP
		}
		var triPos [3]geom.Vec3
		for _, j := range []int{0, 1, 2} {
			triPos[j] = vertPos[hStarts[3*triPQ+j]]
		}
		for _, i := range []int{0, 1, 2} {
			vert := int(starts[3*tri+i])
			bary[3*tri+i] = geom.GetBarycentric(vertPosR[vert], triPos, epsilon)
		}
	}

	// Dedup state.
	idMissProp := numVertR
	propIdx := make([][]propEntry, numVertR+1)
	propMissIdx := [2][]int{
		make([]int, implNumPropVert(inQ, numPropQ)),
		make([]int, implNumPropVert(inP, numPropP)),
	}
	for k := range propMissIdx {
		for i := range propMissIdx[k] {
			propMissIdx[k][i] = -1
		}
	}

	propsOut := make([]float64, 0, numVertR*numProp)
	idx := 0

	for tri := 0; tri < numTri; tri++ {
		if starts[3*tri] < 0 { // skip collapsed triangles
			continue
		}
		ref := triRefR[tri]
		pq := ref.MeshID == 0
		oldNumProp := numPropQ
		props, hProps := propsQ, hPropsQ
		if pq {
			oldNumProp, props, hProps = numPropP, propsP, hPropsP
		}
		// For Subtract, Q's tris are flipped, so Q's world-frame normals (slots
		// 0..2) flip sign. Per-source-tri (inQ may be a mixed Boolean result).
		negateNormals := !pq && invertQ && oldNumProp >= 3 && triHasNormalsQ(int(ref.FaceID))

		for _, i := range []int{0, 1, 2} {
			vert := int(starts[3*tri+i])
			uvw := bary[3*tri+i]

			key := [4]int{0, idMissProp, -1, -1}
			if pq {
				key[0] = 1
			}
			if oldNumProp > 0 {
				edge := -2
				for _, j := range []int{0, 1, 2} {
					if baryComp(uvw, j) == 1 {
						// On a retained vert; the propVert must also match.
						key[2] = int(hProps[3*int(ref.FaceID)+j])
						edge = -1
						break
					}
					if baryComp(uvw, j) == 0 {
						edge = j
					}
				}
				if edge >= 0 {
					// On an edge; both propVerts must match.
					p0 := int(hProps[3*int(ref.FaceID)+geom.Next3(edge)])
					p1 := int(hProps[3*int(ref.FaceID)+geom.Prev3(edge)])
					key[1] = vert
					key[2], key[3] = p0, p1
					if p1 < p0 {
						key[2], key[3] = p1, p0
					}
				} else if edge == -2 {
					key[1] = vert
				}
			}

			if key[1] == idMissProp && key[2] >= 0 {
				// only key.x/key.z matters
				if entry := propMissIdx[key[0]][key[2]]; entry >= 0 {
					propVert[3*tri+i] = int32(entry)
					continue
				}
				propMissIdx[key[0]][key[2]] = idx
			} else {
				bin := propIdx[key[1]]
				found := false
				for _, b := range bin {
					if b.key == [3]int{key[0], key[2], key[3]} {
						found = true
						propVert[3*tri+i] = int32(b.idx)
						break
					}
				}
				if found {
					continue
				}
				propIdx[key[1]] = append(propIdx[key[1]], propEntry{key: [3]int{key[0], key[2], key[3]}, idx: idx})
			}

			propVert[3*tri+i] = int32(idx)
			idx++
			for p := 0; p < numProp; p++ {
				if p < oldNumProp {
					op := [3]float64{}
					for _, j := range []int{0, 1, 2} {
						op[j] = props[oldNumProp*int(hProps[3*int(ref.FaceID)+j])+p]
					}
					val := uvw.Dot(geom.Vec3{X: op[0], Y: op[1], Z: op[2]})
					if negateNormals && p < 3 {
						val = -val
					}
					propsOut = append(propsOut, val)
				} else {
					propsOut = append(propsOut, 0)
				}
			}
		}
	}

	outR.SetHalfedgesRaw(starts, propVert, pairs)
	outR.SetProperties(propsOut)
}

// updateReference is the Go port of UpdateReference (boolean_result.cpp:518) and
// its MapTriRef functor (:505): remap each output tri's triRef from the
// {meshID 0/1, faceID = original P/Q face} form to the actual input triRef,
// offsetting Q's mesh IDs by the current meshIDCounter_; then copy both inputs'
// meshIDtransform entries into the output (Q offset, backSide XOR invertQ).
func updateReference(outR *MutableImpl, inP, inQ *Impl, invertQ bool) {
	offsetQ := int32(bridge.ImplReserveIDs(0)) // meshIDCounter_ (fetch_add 0)
	triRefP := inP.TriRefs()
	triRefQ := inQ.TriRefs()

	refs := outR.TriRefs()
	meshIDs := make([]int32, len(refs))
	originalIDs := make([]int32, len(refs))
	faceIDs := make([]int32, len(refs))
	coplanarIDs := make([]int32, len(refs))
	for i, ref := range refs {
		tri := int(ref.FaceID)
		pq := ref.MeshID == 0
		// C++ MapTriRef (boolean_result.cpp:510): triRef = PQ ? triRefP[tri] :
		// triRefQ[tri]. The branch must be conditional, not "index P then maybe
		// overwrite" — a Q tri's faceID is a Q-local index that can legally exceed
		// NumTriP, so eagerly evaluating triRefP[tri] would index out of range.
		var src bridge.TriRef
		if pq {
			src = triRefP[tri]
		} else {
			src = triRefQ[tri]
			src.MeshID += offsetQ
		}
		meshIDs[i] = src.MeshID
		originalIDs[i] = src.OriginalID
		faceIDs[i] = src.FaceID
		coplanarIDs[i] = src.CoplanarID
	}
	outR.SetTriRefs(meshIDs, originalIDs, faceIDs, coplanarIDs)

	for _, r := range inP.MeshIDTransforms() {
		outR.AddMeshIDTransform(int(r.MeshID), int(r.OriginalID), r.Transform, r.BackSide, r.HasNormals)
	}
	for _, r := range inQ.MeshIDTransforms() {
		// meshIDtransform[pair.first + offsetQ] = pair.second; backSide ^= invertQ.
		outR.AddMeshIDTransform(int(r.MeshID)+int(offsetQ), int(r.OriginalID), r.Transform, r.BackSide != invertQ, r.HasNormals)
	}
}
