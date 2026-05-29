package manifold

import (
	"math"

	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// validateMeshGL is the Go port of the validation cascade at the head of
// the C++ ingest constructor Manifold::Impl::Impl(const MeshGLP&)
// (src/impl.h:281-345). It returns the Error to report (via MakeEmpty) and
// proceed=false when the mesh is empty or malformed; proceed=true means the
// mesh passed every cascade check and geometry construction may continue.
//
// Faithfulness notes:
//   - The entry-time Cancelled gate (impl.h:281) is omitted: the Go port
//     models ctx as nil = never cancelled.
//   - numVert / numTri are computed exactly as C++ (integer division of the
//     raw flat buffers) before the numProp<3 guard. We additionally guard
//     numProp<=0 to avoid a divide-by-zero panic where C++ would be UB; for
//     realistic input (numProp>=3) the result is identical.
//   - isfinite rejects BOTH NaN and ±Inf, checked in the input's native
//     precision (float32 for MeshGL). It runs over the RAW flat buffers —
//     this is NOT the same as Impl.IsFinite() (which checks vertPos_ after
//     the copy and only the positions).
//   - MergeIndexOutOfBounds and VertexOutOfBounds are NOT checked here: C++
//     performs them later (prop2vert / triVert construction). Until those
//     steps are drilled the bridge body reports them.
func validateMeshGL[P float32 | float64, I uint32 | uint64](
	numProp int,
	vertProperties []P,
	triVerts []I,
	mergeFromVert, mergeToVert []I,
	runIndex []I,
	runOriginalID []uint32,
	runTransform []P,
	faceID []I,
	halfedgeTangent []P,
) (Error, bool) {
	numVert := 0
	if numProp > 0 {
		numVert = len(vertProperties) / numProp
	}
	numTri := len(triVerts) / 3

	if numVert == 0 && numTri == 0 {
		return NoError, false
	}
	if numVert < 4 || numTri < 4 {
		return NotManifold, false
	}
	if numProp < 3 {
		return MissingPositionProperties, false
	}
	if len(mergeFromVert) != len(mergeToVert) {
		return MergeVectorsDifferentLengths, false
	}
	if len(runTransform) > 0 && 12*len(runOriginalID) != len(runTransform) {
		return TransformWrongLength, false
	}
	if len(runOriginalID) > 0 && len(runIndex) > 0 &&
		len(runOriginalID)+1 != len(runIndex) && len(runOriginalID) != len(runIndex) {
		return RunIndexWrongLength, false
	}
	if len(faceID) > 0 && len(faceID) != numTri {
		return FaceIDWrongLength, false
	}

	finite := func(x P) bool { f := float64(x); return !math.IsNaN(f) && !math.IsInf(f, 0) }
	if !parallel.AllOf(parallel.AutoPolicy(len(vertProperties), 100000), len(vertProperties),
		func(i int) bool { return finite(vertProperties[i]) }) {
		return NonFiniteVertex, false
	}
	if !parallel.AllOf(parallel.AutoPolicy(len(runTransform), 100000), len(runTransform),
		func(i int) bool { return finite(runTransform[i]) }) {
		return InvalidConstruction, false
	}
	if !parallel.AllOf(parallel.AutoPolicy(len(halfedgeTangent), 100000), len(halfedgeTangent),
		func(i int) bool { return finite(halfedgeTangent[i]) }) {
		return InvalidConstruction, false
	}
	return NoError, true
}

// emptyManifold builds an empty Manifold carrying the given status —
// mirrors C++ MakeEmpty(status) followed by sealing into a Manifold.
func emptyManifold(status Error) *Manifold {
	mi := newImpl()
	defer mi.Delete()
	mi.h.MakeEmpty(int(status))
	return mi.ToManifold()
}

// newImplFromMeshGL is the Go port of the body of the C++ ingest
// constructor Manifold::Impl::Impl(const MeshGLP&) (src/impl.h:347-504) —
// everything after the validation cascade. It builds the Impl natively and
// seals it into a Manifold (including the error early-outs via MakeEmpty).
//
// useSingle == std::is_same<Precision,float>::value (true for MeshGL/
// float32, false for MeshGL64) and threads into SetEpsilon.
func newImplFromMeshGL[P float32 | float64, I uint32 | uint64](
	fullProp int,
	vertProperties []P, triVerts []I,
	mergeFromVert, mergeToVert []I,
	runIndex []I, runOriginalID []uint32, runTransform []P, runFlags []uint8,
	faceID []I, halfedgeTangent []P, tolerance P,
	useSingle bool,
) *Manifold {
	numVert := len(vertProperties) / fullProp
	numTri := len(triVerts) / 3
	extraProp := fullProp - 3

	// prop2vert: iota, then last-writer-wins from the merge vectors
	// (src/impl.h:347-360). Direct assignment, NOT union-find.
	var prop2vert []int32
	if len(mergeFromVert) > 0 {
		prop2vert = make([]int32, numVert)
		for i := range prop2vert {
			prop2vert[i] = int32(i)
		}
		for i := range mergeFromVert {
			from := int(mergeFromVert[i])
			to := int(mergeToVert[i])
			if from >= numVert || to >= numVert {
				return emptyManifold(MergeIndexOutOfBounds)
			}
			prop2vert[from] = int32(to)
		}
	}

	// needsPropMap: extra properties AND a vertex merge, so propVert
	// (triProp) is decoupled from startVert/endVert (triVert) and the
	// two-arg CreateHalfedges is required.
	needsPropMap := extraProp > 0 && len(prop2vert) > 0

	mi := newImpl()
	defer mi.Delete()

	// numProp_ / properties_ / tolerance_ / vertPos_ (src/impl.h:362-376).
	mi.h.SetNumProp(extraProp)
	mi.SetToleranceValue(float64(tolerance))
	mi.h.ResizeVerts(numVert)
	verts := mi.Verts()
	props := make([]float64, numVert*extraProp)
	for i := 0; i < numVert; i++ {
		base := fullProp * i
		verts[i] = geom.Vec3{
			X: float64(vertProperties[base+0]),
			Y: float64(vertProperties[base+1]),
			Z: float64(vertProperties[base+2]),
		}
		for j := 0; j < extraProp; j++ {
			props[i*extraProp+j] = float64(vertProperties[base+3+j])
		}
	}
	if extraProp > 0 {
		mi.h.SetProperties(props)
	}

	// halfedgeTangent_ (src/impl.h:378-382).
	if n := len(halfedgeTangent); n > 0 {
		tangents := make([]float64, n)
		for i := range halfedgeTangent {
			tangents[i] = float64(halfedgeTangent[i])
		}
		mi.h.SetHalfedgeTangents(tangents)
	}

	// Run handling: temp triRef + meshIDtransform (src/impl.h:384-432).
	ri := make([]int, len(runIndex))
	for i, x := range runIndex {
		ri[i] = int(x)
	}
	runEnd := len(triVerts)
	switch {
	case len(ri) == 0:
		ri = []int{0, runEnd}
	case len(ri) == len(runOriginalID):
		ri = append(ri, runEnd)
	case len(ri) == 1:
		ri = append(ri, runEnd)
	}
	startID := int(bridge.ImplReserveIDs(uint32(max(1, len(runOriginalID)))))
	roid := runOriginalID
	if len(roid) == 0 {
		roid = []uint32{uint32(startID)}
	}
	triRefTemp := make([]triRefData, numTri)
	for i := 0; i < len(roid); i++ {
		meshID := startID + i
		originalID := int(roid[i])
		backside := i < len(runFlags) && runFlags[i]&1 != 0
		runHasN := (i < len(runFlags) && runFlags[i]&2 != 0) && extraProp >= 3
		for tri := ri[i] / 3; tri < ri[i+1]/3; tri++ {
			fid := int32(-1)
			if len(faceID) > 0 {
				fid = int32(faceID[tri])
			}
			triRefTemp[tri] = triRefData{int32(meshID), int32(originalID), fid, int32(tri)}
		}
		if len(runTransform) == 0 {
			mi.h.AddMeshIDTransform(meshID, originalID, identityMat3x4(), false, runHasN)
		} else {
			m := runTransform[12*i:]
			tf := [4][3]float64{
				{float64(m[0]), float64(m[1]), float64(m[2])},
				{float64(m[3]), float64(m[4]), float64(m[5])},
				{float64(m[6]), float64(m[7]), float64(m[8])},
				{float64(m[9]), float64(m[10]), float64(m[11])},
			}
			mi.h.AddMeshIDTransform(meshID, originalID, tf, backside, runHasN)
		}
	}

	// triProp / triVert build + degenerate cull (src/impl.h:434-462).
	// triP[j]=raw vert (= prop index), triV[j]=prop2vert-remapped vert.
	// Common path pushes triV into triProp (single-arg form). needsPropMap
	// pushes triP into triProp and triV into triVert (two-arg form).
	triProp := make([]int32, 0, 3*numTri)
	var triVert []int32
	if needsPropMap {
		triVert = make([]int32, 0, 3*numTri)
	}
	var keptMeshID, keptOrigID, keptFaceID, keptCoplanar []int32
	for i := 0; i < numTri; i++ {
		var triP, triV [3]int32
		for j := 0; j < 3; j++ {
			vert := int(triVerts[3*i+j])
			if vert >= numVert {
				mi.h.MakeEmpty(int(VertexIndexOutOfBounds))
				return mi.ToManifold()
			}
			triP[j] = int32(vert)
			if len(prop2vert) == 0 {
				triV[j] = int32(vert)
			} else {
				triV[j] = prop2vert[vert]
			}
		}
		if triV[0] != triV[1] && triV[1] != triV[2] && triV[2] != triV[0] {
			if needsPropMap {
				triProp = append(triProp, triP[0], triP[1], triP[2])
				triVert = append(triVert, triV[0], triV[1], triV[2])
			} else {
				triProp = append(triProp, triV[0], triV[1], triV[2])
			}
			r := triRefTemp[i]
			keptMeshID = append(keptMeshID, r.meshID)
			keptOrigID = append(keptOrigID, r.originalID)
			keptFaceID = append(keptFaceID, r.faceID)
			keptCoplanar = append(keptCoplanar, r.coplanarID)
		}
	}
	mi.h.SetTriRefs(keptMeshID, keptOrigID, keptFaceID, keptCoplanar)

	mi.CreateHalfedges(triProp, triVert)
	if !mi.IsManifold() {
		mi.h.MakeEmpty(int(NotManifold))
		return mi.ToManifold()
	}

	// Finalize tail (src/impl.h:473-503), exact order.
	mi.CalculateBBox()
	mi.SetEpsilon(-1, useSingle)
	mi.CleanupTopology()
	mi.DedupePropVerts()
	mi.SetNormalsAndCoplanar()
	mi.RemoveDegenerates(0)
	mi.RemoveUnreferencedVerts()
	mi.SortGeometry()
	if !mi.IsFinite() {
		mi.h.MakeEmpty(int(NonFiniteVertex))
		return mi.ToManifold()
	}
	mi.h.SetMeshRelationOriginalID(-1)
	return mi.ToManifold()
}

type triRefData struct{ meshID, originalID, faceID, coplanarID int32 }

func identityMat3x4() [4][3]float64 {
	return [4][3]float64{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}}
}
