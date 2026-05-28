package manifold

import (
	"math"
	"math/bits"
	"sync/atomic"

	"github.com/firstlayer-xyz/manifold/go/internal/hashtable"
	"github.com/firstlayer-xyz/manifold/go/internal/parallel"
)

// LevelSet is the Go port of C++ Manifold::LevelSet (src/sdf.cpp).
// Marching-tetrahedra on a body-centered cubic grid: sample the SDF
// at every grid point, identify edges crossing the iso-surface,
// snap interior verts to the surface where possible, then emit
// triangles from each tetrahedron's iso-surface intersection.
//
// The Go port runs the SDF callback purely in Go — no cgo round
// trip per voxel sample. For a 200³ grid that's millions of saved
// cgo transitions.
//
// Parameters match C++:
//   - sdf: signed-distance function. Positive inside, negative
//     outside. Need not be a true distance, or continuous.
//   - bounds: axis-aligned box defining the grid extent.
//   - edgeLength: target max edge length; controls grid spacing.
//   - level: iso-value to extract (default 0).
//   - tolerance: ITP root-finding precision. <= 0 means "use
//     linear interpolation between grid samples" (cheap).
//   - canParallel: false forces sequential evaluation. Used by
//     callers whose SDF callback isn't goroutine-safe.
func LevelSet(
	sdf func(Vec3) float64,
	bounds Box,
	edgeLength, level, tolerance float64,
	canParallel bool,
) *Manifold {
	if tolerance <= 0 {
		tolerance = math.Inf(1)
	}

	dim := bounds.Max.Sub(bounds.Min)
	gridSize := [3]int{
		int(dim.X/edgeLength) + 1,
		int(dim.Y/edgeLength) + 1,
		int(dim.Z/edgeLength) + 1,
	}
	spacing := Vec3{
		X: dim.X / float64(gridSize[0]-1),
		Y: dim.Y / float64(gridSize[1]-1),
		Z: dim.Z / float64(gridSize[2]-1),
	}
	gridPow := computeGridPow(gridSize)
	maxIndex := encodeIndex([4]int{gridSize[0] + 2, gridSize[1] + 2, gridSize[2] + 2, 1}, gridPow)

	// Voxel grid — one SDF sample per grid point. Matches
	// src/sdf.cpp:493-498.
	voxels := make([]float64, maxIndex)
	policy := parallel.Seq
	if canParallel {
		policy = parallel.AutoPolicy(int(maxIndex))
	}
	parallel.ForEachN(policy, int(maxIndex), func(idx int) {
		gridIdx := decodeIndex(uint64(idx), gridPow)
		// Subtract the kVoxelOffset so the grid index is in
		// [-1, gridSize+1] terms.
		gridIdx[0] -= 1
		gridIdx[1] -= 1
		gridIdx[2] -= 1
		voxels[idx] = boundedSDF(gridIdx, bounds.Min, spacing, gridSize, level, sdf)
	})

	// HashTable<GridVert> sized between a dense bound and a sparse
	// bound — matches src/sdf.cpp:500-511. Level sets are sparse so
	// we don't allocate the full 2*maxIndex.
	// C++ uses std::numeric_limits<size_t>::max() which is uint64
	// max (2^64-1) on 64-bit. math.MaxInt is int max (2^63-1) which
	// is the wrong cap. Use math.MaxUint64 to match.
	tableSizeCap := uint64(math.MaxUint64)
	var denseTableSize uint64
	if maxIndex > tableSizeCap/2 {
		denseTableSize = tableSizeCap
	} else {
		denseTableSize = 2 * maxIndex
	}
	sparseTableSize := uint64(10 * math.Sqrt(float64(maxIndex)))
	if sparseTableSize > tableSizeCap {
		sparseTableSize = tableSizeCap
	}
	tableSize := denseTableSize
	if sparseTableSize < tableSize {
		tableSize = sparseTableSize
	}
	if tableSize < 1 {
		tableSize = 1
	}

	gridVerts := hashtable.New[gridVert](int(tableSize), 1)
	vertPos := make([]Vec3, gridVerts.Size()*7)

	// Outer resize loop: NearSurface may overflow the hashtable; if
	// Full(), grow and retry. Mirrors src/sdf.cpp:514-540.
	var vertIndex atomic.Int32
	for {
		vertIndex.Store(0)
		gridLimit := encodeIndex([4]int{gridSize[0], gridSize[1], gridSize[2], 1}, gridPow)
		parallel.ForEachN(policy, int(gridLimit), func(idx int) {
			nearSurface(uint64(idx), vertPos, &vertIndex, gridVerts, voxels, sdf,
				bounds.Min, gridSize, gridPow, spacing, level, tolerance)
		})

		if gridVerts.Full() {
			// Grow ratio from a sample lastVert. Mirrors C++.
			vidx := vertIndex.Load() - 1
			if vidx < 0 {
				tableSize *= 2
			} else {
				lastVert := vertPos[vidx]
				lastIndex := encodeIndex([4]int{
					int((lastVert.X - bounds.Min.X) / spacing.X),
					int((lastVert.Y - bounds.Min.Y) / spacing.Y),
					int((lastVert.Z - bounds.Min.Z) / spacing.Z),
					1,
				}, gridPow)
				ratio := float64(maxIndex) / float64(lastIndex)
				if ratio > 1000 {
					tableSize *= 2
				} else {
					tableSize = uint64(float64(tableSize) * ratio)
				}
			}
			gridVerts = hashtable.New[gridVert](int(tableSize), 1)
			vertPos = make([]Vec3, gridVerts.Size()*7)
			continue
		}

		// Success: run ComputeVerts and break.
		parallel.ForEachN(policy, gridVerts.Size(), func(idx int) {
			computeVerts(idx, vertPos, &vertIndex, gridVerts, voxels, sdf,
				bounds.Min, gridSize, gridPow, spacing, level, tolerance)
		})
		vertPos = vertPos[:vertIndex.Load()]
		break
	}

	// Build triangles. Worst case is 12 tris per gridVert entry.
	triVerts := make([]int32, gridVerts.Entries()*12*3)
	var triIndex atomic.Int32
	parallel.ForEachN(policy, gridVerts.Size(), func(idx int) {
		buildTris(idx, triVerts, &triIndex, gridVerts, gridPow)
	})
	triVerts = triVerts[:triIndex.Load()*3]

	// Build the impl and run the standard finalize tail.
	impl := newImpl()
	defer impl.Delete()
	impl.h.ResizeVerts(len(vertPos))
	copy(impl.Verts(), vertPos)
	impl.CreateHalfedges(triVerts)
	impl.CleanupTopology()
	impl.RemoveUnreferencedVerts()
	impl.InitializeOriginal()
	impl.CalculateBBox()
	impl.SetEpsilon(-1, false)
	impl.SortGeometry()
	impl.SetNormalsAndCoplanar()
	return impl.ToManifold()
}

// ============================================================
// LevelSet constants — mirror src/sdf.cpp:27-35.

const (
	lsCrossing   = -2
	lsNone       = -1
	lsKS         = 0.25
	lsKD         = 1/lsKS - 1
	lsKMaxOpposed = 3
)

var lsVoxelOffset = [4]int{1, 1, 1, 0}

// ============================================================
// Lookup tables — direct ports of src/sdf.cpp:37-74 (TetTri0,
// TetTri1) and src/sdf.cpp:77-91 (Neighbor).

var lsTetTri0 = [16][3]int{
	{-1, -1, -1},
	{0, 3, 4},
	{0, 1, 5},
	{1, 5, 3},
	{1, 4, 2},
	{1, 0, 3},
	{2, 5, 0},
	{5, 3, 2},
	{2, 3, 5},
	{0, 5, 2},
	{3, 0, 1},
	{2, 4, 1},
	{3, 5, 1},
	{5, 1, 0},
	{4, 3, 0},
	{-1, -1, -1},
}

var lsTetTri1 = [16][3]int{
	{-1, -1, -1},
	{-1, -1, -1},
	{-1, -1, -1},
	{3, 4, 1},
	{-1, -1, -1},
	{3, 2, 1},
	{0, 4, 2},
	{-1, -1, -1},
	{-1, -1, -1},
	{2, 4, 0},
	{1, 2, 3},
	{-1, -1, -1},
	{1, 4, 3},
	{-1, -1, -1},
	{-1, -1, -1},
	{-1, -1, -1},
}

var lsNeighbors = [14][4]int{
	{0, 0, 0, 1},
	{1, 0, 0, 0},
	{0, 1, 0, 0},
	{0, 0, 1, 0},
	{-1, 0, 0, 1},
	{0, -1, 0, 1},
	{0, 0, -1, 1},
	{-1, -1, -1, 1},
	{-1, 0, 0, 0},
	{0, -1, 0, 0},
	{0, 0, -1, 0},
	{0, -1, -1, 1},
	{-1, 0, -1, 1},
	{-1, -1, 0, 1},
}

// neighbor returns the ivec4 grid position of the i'th neighbor of
// `base`. Mirrors src/sdf.cpp:77-98 — the .w==2 carry adjustment
// is preserved.
func neighbor(base [4]int, i int) [4]int {
	n := lsNeighbors[i]
	out := [4]int{
		base[0] + n[0],
		base[1] + n[1],
		base[2] + n[2],
		base[3] + n[3],
	}
	if out[3] == 2 {
		out[0]++
		out[1]++
		out[2]++
		out[3] = 0
	}
	return out
}

// ============================================================
// Index encoding helpers — mirror src/sdf.cpp:100-127.

// encodeIndex packs (x, y, z, w) into a single uint64 using gridPow
// per-axis bit widths.
func encodeIndex(gridPos [4]int, gridPow [3]int) uint64 {
	return uint64(gridPos[3]) |
		uint64(gridPos[2])<<1 |
		uint64(gridPos[1])<<(1+gridPow[2]) |
		uint64(gridPos[0])<<(1+gridPow[2]+gridPow[1])
}

// decodeIndex is the inverse of encodeIndex.
func decodeIndex(idx uint64, gridPow [3]int) [4]int {
	var out [4]int
	out[3] = int(idx & 1)
	idx >>= 1
	out[2] = int(idx & ((1 << gridPow[2]) - 1))
	idx >>= gridPow[2]
	out[1] = int(idx & ((1 << gridPow[1]) - 1))
	idx >>= gridPow[1]
	out[0] = int(idx & ((1 << gridPow[0]) - 1))
	return out
}

// computeGridPow returns the per-axis bit count needed to encode
// gridSize[a]+2. floor(log2(n+2)) + 1 in integer math.
func computeGridPow(gridSize [3]int) [3]int {
	axisPow := func(n int) int {
		v := uint64(uint32(n + 2))
		return ceilLog2_(v + 1)
	}
	return [3]int{axisPow(gridSize[0]), axisPow(gridSize[1]), axisPow(gridSize[2])}
}

// ceilLog2_ matches src/hashtable.h:56 CeilLog2 — integer ceiling
// log2. ceilLog2(0)=0, ceilLog2(1)=0, ceilLog2(2)=1, ceilLog2(3)=2…
func ceilLog2_(v uint64) int {
	if v <= 1 {
		return 0
	}
	return 64 - bits.LeadingZeros64(v-1)
}

// ============================================================
// Position helpers — mirror src/sdf.cpp:129-149.

// position returns the world-space position of a grid index.
// Mirrors src/sdf.cpp:129-131. .w==1 means "on cubic grid", .w==0
// means "shifted (-0.5 in each axis) grid" — the body-centered
// cubic.
func position(gridIndex [4]int, origin Vec3, spacing Vec3) Vec3 {
	offset := -0.5
	if gridIndex[3] == 1 {
		offset = 0
	}
	return Vec3{
		X: origin.X + spacing.X*(float64(gridIndex[0])+offset),
		Y: origin.Y + spacing.Y*(float64(gridIndex[1])+offset),
		Z: origin.Z + spacing.Z*(float64(gridIndex[2])+offset),
	}
}

// bound clamps pos to the grid extent — mirrors src/sdf.cpp:133-135.
func bound(pos, origin, spacing Vec3, gridSize [3]int) Vec3 {
	max := Vec3{
		X: origin.X + spacing.X*float64(gridSize[0]-1),
		Y: origin.Y + spacing.Y*float64(gridSize[1]-1),
		Z: origin.Z + spacing.Z*float64(gridSize[2]-1),
	}
	return Vec3{
		X: math.Min(math.Max(pos.X, origin.X), max.X),
		Y: math.Min(math.Max(pos.Y, origin.Y), max.Y),
		Z: math.Min(math.Max(pos.Z, origin.Z), max.Z),
	}
}

// boundedSDF mirrors src/sdf.cpp:137-149 — returns the SDF value
// at gridIndex, but clamps boundary cells to ≤ 0 so the surface
// always closes inside the bounding box.
func boundedSDF(gridIndex [4]int, origin, spacing Vec3, gridSize [3]int,
	level float64, sdf func(Vec3) float64) float64 {
	xyz := [3]int{gridIndex[0], gridIndex[1], gridIndex[2]}
	lowerBoundDist := minInt3(xyz)
	upperBoundDist := minInt3([3]int{gridSize[0] - xyz[0], gridSize[1] - xyz[1], gridSize[2] - xyz[2]})
	boundDist := upperBoundDist - gridIndex[3]
	if lowerBoundDist < boundDist {
		boundDist = lowerBoundDist
	}
	if boundDist < 0 {
		return 0
	}
	d := sdf(position(gridIndex, origin, spacing)) - level
	if boundDist == 0 && d > 0 {
		return 0
	}
	return d
}

func minInt3(v [3]int) int {
	m := v[0]
	if v[1] < m {
		m = v[1]
	}
	if v[2] < m {
		m = v[2]
	}
	return m
}

// ============================================================
// FindSurface — ITP root finding (src/sdf.cpp:151-188).
//
// Locates the surface crossing between pos0 (SDF=d0) and pos1
// (SDF=d1) to within `tol` distance via interpolation-truncation-
// projection. Bisection worst case, super-linear average.
func findSurface(pos0 Vec3, d0 float64, pos1 Vec3, d1 float64,
	tol, level float64, sdf func(Vec3) float64) Vec3 {
	if d0 == 0 {
		return pos0
	}
	if d1 == 0 {
		return pos1
	}
	const k = 0.1
	delta := pos0.Sub(pos1)
	check := 2 * tol / math.Sqrt(delta.Dot(delta))
	frac := 1.0
	biFrac := 1.0
	for frac > check {
		t := lerpScalar(d0/(d0-d1), 0.5, k)
		r := biFrac/frac - 0.5
		var x float64
		if math.Abs(t-0.5) < r {
			x = t
		} else {
			sign := 1.0
			if t >= 0.5 {
				sign = -1.0
			}
			x = 0.5 - r*sign
		}
		mid := lerpVec3(pos0, pos1, x)
		d := sdf(mid) - level

		if (d > 0) == (d0 > 0) {
			d0 = d
			pos0 = mid
			frac *= 1 - x
		} else {
			d1 = d
			pos1 = mid
			frac *= x
		}
		biFrac /= 2
	}
	return lerpVec3(pos0, pos1, d0/(d0-d1))
}

func lerpScalar(a, b, t float64) float64 { return a + (b-a)*t }
func lerpVec3(a, b Vec3, t float64) Vec3 {
	return Vec3{
		X: a.X + (b.X-a.X)*t,
		Y: a.Y + (b.Y-a.Y)*t,
		Z: a.Z + (b.Z-a.Z)*t,
	}
}

// ============================================================
// GridVert — port of src/sdf.cpp:196-212.

type gridVert struct {
	distance  float64
	movedVert int32
	edgeVerts [7]int32
}

func newGridVert() gridVert {
	return gridVert{
		distance:  math.NaN(),
		movedVert: lsNone,
		edgeVerts: [7]int32{lsNone, lsNone, lsNone, lsNone, lsNone, lsNone, lsNone},
	}
}

func (g *gridVert) hasMoved() bool { return g.movedVert >= 0 }
func (g *gridVert) sameSide(dist float64) bool {
	return (dist > 0) == (g.distance > 0)
}
func (g *gridVert) inside() int {
	if g.distance > 0 {
		return 1
	}
	return -1
}
func (g *gridVert) neighborInside(i int) int {
	if g.edgeVerts[i] == lsNone {
		return g.inside()
	}
	return -g.inside()
}

// ============================================================
// NearSurface — port of src/sdf.cpp:214-295.
//
// Iterates a grid cell, identifies edges crossing the surface,
// and (optionally) snaps the cell's GridVert position to the
// surface when there's a closest crossing neighbor.
func nearSurface(
	index uint64,
	vertPos []Vec3, vertIndex *atomic.Int32,
	gridVerts *hashtable.HashTable[gridVert],
	voxels []float64,
	sdf func(Vec3) float64,
	origin Vec3, gridSize [3]int, gridPow [3]int, spacing Vec3,
	level, tol float64,
) {
	if gridVerts.Full() {
		return
	}

	gridIndex := decodeIndex(index, gridPow)
	if gridIndex[0] > gridSize[0] || gridIndex[1] > gridSize[1] || gridIndex[2] > gridSize[2] {
		return
	}

	g := newGridVert()
	g.distance = voxels[encodeIndex(addVoxelOffset(gridIndex), gridPow)]

	keep := false
	vMax := 0.0
	closestNeighbor := -1
	opposedVerts := 0
	for i := 0; i < 7; i++ {
		val := voxels[encodeIndex(addVoxelOffset(neighbor(gridIndex, i)), gridPow)]
		valOp := voxels[encodeIndex(addVoxelOffset(neighbor(gridIndex, i+7)), gridPow)]

		if !g.sameSide(val) {
			g.edgeVerts[i] = lsCrossing
			keep = true
			if !g.sameSide(valOp) {
				opposedVerts++
			}
			if math.Abs(val) > lsKD*math.Abs(g.distance) &&
				math.Abs(val) > math.Abs(vMax) {
				vMax = val
				closestNeighbor = i
			}
		} else if !g.sameSide(valOp) &&
			math.Abs(valOp) > lsKD*math.Abs(g.distance) &&
			math.Abs(valOp) > math.Abs(vMax) {
			vMax = valOp
			closestNeighbor = i + 7
		}
	}

	// Collapse crossing-edge verts into this GridVert when there's
	// a "close enough" neighbor and not too many opposed crossings.
	// Without this step the result is guaranteed 2-manifold; with
	// it the result is even-manifold (kissing verts), removed by
	// the post-process CleanupTopology call.
	if closestNeighbor >= 0 && opposedVerts <= lsKMaxOpposed {
		gridPos := position(gridIndex, origin, spacing)
		neighborIdx := neighbor(gridIndex, closestNeighbor)
		pos := findSurface(gridPos, g.distance,
			position(neighborIdx, origin, spacing),
			vMax, tol, level, sdf)
		// Bound the per-vert delta to avoid inverting tetrahedra.
		dx := math.Abs(pos.X - gridPos.X)
		dy := math.Abs(pos.Y - gridPos.Y)
		dz := math.Abs(pos.Z - gridPos.Z)
		if dx < lsKS*spacing.X && dy < lsKS*spacing.Y && dz < lsKS*spacing.Z {
			idx := vertIndex.Add(1) - 1
			vertPos[idx] = bound(pos, origin, spacing, gridSize)
			g.movedVert = idx
			for j := 0; j < 7; j++ {
				if g.edgeVerts[j] == lsCrossing {
					g.edgeVerts[j] = idx
				}
			}
			keep = true
		}
	} else {
		for j := 0; j < 7; j++ {
			g.edgeVerts[j] = lsNone
		}
	}

	if keep {
		gridVerts.Insert(index, g)
	}
}

func addVoxelOffset(p [4]int) [4]int {
	return [4]int{p[0] + lsVoxelOffset[0], p[1] + lsVoxelOffset[1], p[2] + lsVoxelOffset[2], p[3] + lsVoxelOffset[3]}
}

// ============================================================
// ComputeVerts — port of src/sdf.cpp:297-348.
//
// For each occupied gridVert that hasn't moved, emit one vert per
// crossing-edge neighbor. If the neighbor moved, reuse its
// movedVert.
func computeVerts(
	idx int,
	vertPos []Vec3, vertIndex *atomic.Int32,
	gridVerts *hashtable.HashTable[gridVert],
	voxels []float64,
	sdf func(Vec3) float64,
	origin Vec3, gridSize [3]int, gridPow [3]int, spacing Vec3,
	level, tol float64,
) {
	baseKey := gridVerts.KeyAt(idx)
	if baseKey == hashtable.Open {
		return
	}
	gv := gridVerts.At(idx)
	if gv.hasMoved() {
		return
	}
	gridIndex := decodeIndex(baseKey, gridPow)
	pos := position(gridIndex, origin, spacing)

	for i := 0; i < 7; i++ {
		neighborIdx := neighbor(gridIndex, i)
		// Use Slot (returns *gridVert) to avoid copying the whole
		// struct. The fields we read here — distance and movedVert
		// — were written by NearSurface, which is sequenced-before
		// ComputeVerts via parallel.ForEachN's wg.Wait. The
		// edgeVerts that ComputeVerts writes concurrently are NOT
		// read here.
		var nDist float64 = math.NaN()
		var nMoved int32 = lsNone
		if nGV, ok := gridVerts.Slot(encodeIndex(neighborIdx, gridPow)); ok {
			nDist = nGV.distance
			nMoved = nGV.movedVert
		}
		var val float64
		// C++ uses std::isfinite (rejects NaN AND ±Inf). The
		// IsNaN-only check would let an Inf-valued neighbor
		// distance leak through where C++ would substitute the
		// voxel — though in practice both equal the same value
		// (NearSurface sets distance = voxel). Mirror exactly to
		// keep the audit clean.
		if !math.IsNaN(nDist) && !math.IsInf(nDist, 0) {
			val = nDist
		} else {
			val = voxels[encodeIndex(addVoxelOffset(neighborIdx), gridPow)]
		}
		if gv.sameSide(val) {
			continue
		}
		if nMoved >= 0 {
			// Atomic store — another worker may be reading this
			// edgeVerts in BuildTris later (after wg.Wait), but
			// concurrent ComputeVerts workers MAY also be looking
			// up this slot's whole struct via plain Lookup if any
			// remain. Atomic store is the C++ benign-race
			// equivalent under the Go memory model.
			atomic.StoreInt32(&gv.edgeVerts[i], nMoved)
			continue
		}
		newIdx := vertIndex.Add(1) - 1
		p := findSurface(pos, gv.distance,
			position(neighborIdx, origin, spacing),
			val, tol, level, sdf)
		vertPos[newIdx] = bound(p, origin, spacing, gridSize)
		atomic.StoreInt32(&gv.edgeVerts[i], newIdx)
	}
}

// ============================================================
// BuildTris — port of src/sdf.cpp:350-435.
//
// Each occupied gridVert is responsible for 6 tetrahedra around
// its (1,1,1) edge. For each tetrahedron, look up the marching-
// tetrahedra triangulation table and emit 0/1/2 triangles.
func buildTris(
	idx int,
	triVerts []int32, triIndex *atomic.Int32,
	gridVerts *hashtable.HashTable[gridVert],
	gridPow [3]int,
) {
	baseKey := gridVerts.KeyAt(idx)
	if baseKey == hashtable.Open {
		return
	}
	base := *gridVerts.At(idx)
	baseIndex := decodeIndex(baseKey, gridPow)

	createTri := func(tri [3]int, edges *[6]int32) {
		if tri[0] < 0 {
			return
		}
		v := [3]int32{edges[tri[0]], edges[tri[1]], edges[tri[2]]}
		if v[0] == v[1] || v[1] == v[2] || v[2] == v[0] {
			return
		}
		ti := triIndex.Add(1) - 1
		triVerts[3*ti] = v[0]
		triVerts[3*ti+1] = v[1]
		triVerts[3*ti+2] = v[2]
	}
	createTris := func(tet [4]int, edges *[6]int32) {
		i := 0
		if tet[0] > 0 {
			i += 1
		}
		if tet[1] > 0 {
			i += 2
		}
		if tet[2] > 0 {
			i += 4
		}
		if tet[3] > 0 {
			i += 8
		}
		createTri(lsTetTri0[i], edges)
		createTri(lsTetTri1[i], edges)
	}

	// Mirrors src/sdf.cpp:380-433 verbatim.
	leadIndex := baseIndex
	if leadIndex[3] == 0 {
		leadIndex[3] = 1
	} else {
		leadIndex[0]++
		leadIndex[1]++
		leadIndex[2]++
		leadIndex[3] = 0
	}

	tet := [4]int{base.neighborInside(0), base.inside(), -2, -2}
	thisIndex := baseIndex
	thisIndex[0]++
	// On a HashTable miss, C++ operator[] returns a default
	// GridVert (distance=NaN, all edgeVerts=-1). We mirror that.
	thisVert, ok := gridVerts.Lookup(encodeIndex(thisIndex, gridPow))
	if !ok {
		thisVert = newGridVert()
	}

	tet[2] = base.neighborInside(1)
	for _, i := range [3]int{0, 1, 2} {
		prev3 := prev3Of(i)
		next3 := next3Of(i)

		thisIndex = leadIndex
		thisIndex[prev3]--
		var nextVert gridVert
		if thisIndex[prev3] < 0 {
			nextVert = newGridVert()
		} else {
			nv, found := gridVerts.Lookup(encodeIndex(thisIndex, gridPow))
			if found {
				nextVert = nv
			} else {
				nextVert = newGridVert()
			}
		}
		tet[3] = base.neighborInside(prev3 + 4)

		edges1 := [6]int32{
			base.edgeVerts[0],
			base.edgeVerts[i+1],
			nextVert.edgeVerts[next3+4],
			nextVert.edgeVerts[prev3+1],
			thisVert.edgeVerts[i+4],
			base.edgeVerts[prev3+4],
		}
		thisVert = nextVert
		createTris(tet, &edges1)

		thisIndex = baseIndex
		thisIndex[next3]++
		nv, found := gridVerts.Lookup(encodeIndex(thisIndex, gridPow))
		if found {
			nextVert = nv
		} else {
			nextVert = newGridVert()
		}
		tet[2] = tet[3]
		tet[3] = base.neighborInside(next3 + 1)

		edges2 := [6]int32{
			base.edgeVerts[0],
			edges1[5],
			thisVert.edgeVerts[i+4],
			nextVert.edgeVerts[next3+4],
			edges1[3],
			base.edgeVerts[next3+1],
		}
		thisVert = nextVert
		createTris(tet, &edges2)

		tet[2] = tet[3]
	}
}

// prev3Of and next3Of mirror C++ Prev3 / Next3 in src/utils.h.
func prev3Of(i int) int {
	switch i {
	case 0:
		return 2
	case 1:
		return 0
	case 2:
		return 1
	}
	return 0
}
func next3Of(i int) int {
	switch i {
	case 0:
		return 1
	case 1:
		return 2
	case 2:
		return 0
	}
	return 0
}
