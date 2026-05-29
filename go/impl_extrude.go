package manifold

import (
	"github.com/firstlayer-xyz/manifold/go/bridge"
	"github.com/firstlayer-xyz/manifold/go/internal/geom"
)

// extrude is the Go port of C++ Manifold::Extrude
// (src/constructors.cpp). Sweeps the input polygons along +Z by
// height, optionally with twist and scale-top. Mirrors the C++ body
// step for step:
//
//  1. Validate (empty cross-section or height <= 0 → Invalid).
//  2. Build the bottom layer of vertPos from the input polygons,
//     and a polygon-index mapping for the eventual cap triangulation.
//  3. For each layer 1..nDivisions+1, transform the cross-section by
//     a rotation+scale interpolation and push the new verts. For
//     each non-cap edge, emit two triangles (or one when this layer
//     is the cone tip).
//  4. For a cone, duplicate the tip vertex per polygon (needed for
//     correct genus accounting).
//  5. Triangulate the cross-section once and add the cap triangles —
//     bottom (flipped winding) and top (same winding, offset by the
//     final layer's vert count).
//  6. Run the standard finalize tail (CreateHalfedges,
//     InitializeOriginal, CalculateBBox, SetEpsilon, sortGeometry,
//     setNormalsAndCoplanar).
func extrude(crossSection Polygons, height float64,
	nDivisions int, twistDegrees float64, scaleTop Vec2,
) *Manifold {
	if len(crossSection) == 0 || height <= 0 {
		return invalidManifold()
	}
	if scaleTop.X < 0 {
		scaleTop.X = 0
	}
	if scaleTop.Y < 0 {
		scaleTop.Y = 0
	}

	// C++ does `++nDivisions` immediately; mirror that.
	nDivisions++

	// Step 2: bottom layer.
	var verts []geom.Vec3
	nCrossSection := 0
	for _, poly := range crossSection {
		nCrossSection += len(poly)
		for _, pv := range poly {
			verts = append(verts, geom.Vec3{X: pv.X, Y: pv.Y, Z: 0})
		}
	}
	isCone := scaleTop.X == 0 && scaleTop.Y == 0

	// Triangle indices, flat (3 per tri).
	var tris []int32

	// Step 3: sweep layers.
	for i := 1; i <= nDivisions; i++ {
		alpha := float64(i) / float64(nDivisions)
		phi := alpha * twistDegrees
		// scale = lerp(vec2(1,1), scaleTop, alpha)
		sx := 1 + alpha*(scaleTop.X-1)
		sy := 1 + alpha*(scaleTop.Y-1)
		// rotation matrix * scale: [[c, s], [-s, c]] then scale x,y
		cosP := geom.Cosd(phi)
		sinP := geom.Sind(phi)
		// transform[col][row]: applied to vec2 as (M·v).x, (M·v).y
		// In C++ la convention mat2({a,b}, {c,d}) means column 0 = (a,b),
		// column 1 = (c,d). And transform * v = col0*v.x + col1*v.y.
		// scale * rotation: ((cosP, sinP), (-sinP, cosP)) then scale rows.
		// Apply manually below.
		_ = cosP
		_ = sinP

		j := 0
		idx := 0
		for _, poly := range crossSection {
			for vert := 0; vert < len(poly); vert++ {
				offset := idx + nCrossSection*i
				thisVert := vert + offset
				lastVertOrigin := vert
				if lastVertOrigin == 0 {
					lastVertOrigin = len(poly)
				}
				lastVertOrigin--
				lastVert := lastVertOrigin + offset

				if i == nDivisions && isCone {
					// Cone-tip face. The dedicated tip-vert index will be
					// added after the layer loop; we predict it here.
					tris = append(tris,
						int32(nCrossSection*i+j),
						int32(lastVert-nCrossSection),
						int32(thisVert-nCrossSection))
				} else {
					// Transform poly[vert] by rotation*scale.
					pv := poly[vert]
					// rotation·v then scale·rotated:
					rx := cosP*pv.X + sinP*pv.Y
					ry := -sinP*pv.X + cosP*pv.Y
					verts = append(verts, geom.Vec3{
						X: sx * rx, Y: sy * ry, Z: height * alpha,
					})
					tris = append(tris,
						int32(thisVert), int32(lastVert),
						int32(thisVert-nCrossSection))
					tris = append(tris,
						int32(lastVert), int32(lastVert-nCrossSection),
						int32(thisVert-nCrossSection))
				}
			}
			j++
			idx += len(poly)
		}
	}

	// Step 4: cone tip duplicates (one per polygon).
	if isCone {
		for range crossSection {
			verts = append(verts, geom.Vec3{X: 0, Y: 0, Z: height})
		}
	}

	// Step 5: cap triangulation. We use bridge.Triangulate (still a
	// bridge call until we port Triangulate itself) on the original
	// cross-section, then map indices to the appropriate vert layer.
	polyFlat := make([][]geom.Vec2, len(crossSection))
	for i, poly := range crossSection {
		polyFlat[i] = make([]geom.Vec2, len(poly))
		copy(polyFlat[i], poly)
	}
	// C++ Extrude calls TriangulateIdx(polygonsIndexed) with no epsilon
	// (src/constructors.cpp:309), so epsilon takes its default of -1
	// (include/manifold/polygon.h:55) — NOT 0.
	top := bridge.Triangulate(polyFlat, -1)
	for t := 0; t < len(top); t += 3 {
		// Bottom: flipped winding to face -Z.
		tris = append(tris, int32(top[t+0]), int32(top[t+2]), int32(top[t+1]))
		if !isCone {
			// Top: same winding, offset to the topmost layer.
			off := int32(nCrossSection * nDivisions)
			tris = append(tris, top[t+0]+off, top[t+1]+off, top[t+2]+off)
		}
	}

	// Step 6: write to a fresh MutableImpl and finalize.
	mi := newImpl()
	defer mi.Delete()
	mi.h.ResizeVerts(len(verts))
	copy(mi.Verts(), verts)
	mi.CreateHalfedges(tris, nil)
	mi.InitializeOriginal()
	mi.CalculateBBox()
	mi.SetEpsilon(-1, false)
	mi.SortGeometry()
	mi.SetNormalsAndCoplanar()
	return mi.ToManifold()
}
