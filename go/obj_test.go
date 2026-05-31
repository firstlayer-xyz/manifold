package manifold

import (
	"bytes"
	"runtime"
	"strings"
	"testing"

	"github.com/firstlayer-xyz/manifold/go/internal/cppref"
)

// TestWriteOBJ_Tetrahedron_FormatSpotCheck verifies the OBJ output
// header and structure for a known tetrahedron — the format must match
// the C++ writer so that ReadOBJ (either side) can consume it.
func TestWriteOBJ_Tetrahedron_FormatSpotCheck(t *testing.T) {
	hRef := cppref.Tetrahedron()
	defer cppref.DeleteManifold(hRef)
	m := fromRefHandle(hRef)

	var buf bytes.Buffer
	if !m.WriteOBJ(&buf) {
		t.Fatal("WriteOBJ returned false")
	}
	out := buf.String()
	for _, want := range []string{
		"# ======= begin mesh ======",
		"# float_format = fixed",
		"# tolerance = ",
		"# epsilon = ",
		"# ======== end mesh =======",
	} {
		if !strings.Contains(out, want) {
			t.Errorf("WriteOBJ output missing %q\nfull:\n%s", want, out)
		}
	}
	// Tetrahedron has exactly 4 verts and 4 tris.
	if got := strings.Count(out, "\nv "); got != 4 {
		t.Errorf("vertex lines: got %d, want 4", got)
	}
	if got := strings.Count(out, "\nf "); got != 4 {
		t.Errorf("face lines: got %d, want 4", got)
	}
}

// TestReadOBJ_RoundTrip_Tetrahedron writes a tetrahedron, reads it
// back, and asserts the geometry survives (vertex/triangle counts,
// volume, bounding box).
func TestReadOBJ_RoundTrip_Tetrahedron(t *testing.T) {
	hRef := cppref.Tetrahedron()
	defer cppref.DeleteManifold(hRef)
	src := fromRefHandle(hRef)
	defer runtime.KeepAlive(src)

	var buf bytes.Buffer
	if !src.WriteOBJ(&buf) {
		t.Fatal("WriteOBJ failed")
	}

	dst := ReadOBJ(&buf)
	defer runtime.KeepAlive(dst)
	if dst.Status() != NoError {
		t.Fatalf("ReadOBJ status: %v", dst.Status())
	}
	if got, want := dst.NumVert(), src.NumVert(); got != want {
		t.Errorf("NumVert: got %d, want %d", got, want)
	}
	if got, want := dst.NumTri(), src.NumTri(); got != want {
		t.Errorf("NumTri: got %d, want %d", got, want)
	}
	if !floatClose(dst.Volume(), src.Volume(), 1e-15, 1e-15) {
		t.Errorf("Volume: got %v, want %v", dst.Volume(), src.Volume())
	}
	if !floatClose(dst.GetEpsilon(), src.GetEpsilon(), 1e-15, 1e-15) {
		t.Errorf("GetEpsilon round-trip: got %v, want %v",
			dst.GetEpsilon(), src.GetEpsilon())
	}
	assertSameBoundingBox(t, dst.refHandle(), src.refHandle(), 1e-15)
}

// TestReadOBJ_IgnoresOverlongLines_AndComments mirrors the C++ skip
// behavior: lines longer than 1000 chars are dropped, and unknown
// comments are ignored.
func TestReadOBJ_IgnoresOverlongLines_AndComments(t *testing.T) {
	overlong := strings.Repeat("x", 1500)
	objText := "# random comment\n" +
		"# tolerance = 0.001\n" +
		"# unknown_metadata = ignored\n" +
		"v 0 0 0\n" +
		"v 1 0 0\n" +
		"v 0 1 0\n" +
		"v 0 0 1\n" +
		"# " + overlong + "\n" + // gets dropped
		"f 1 2 3\n" +
		"f 1 3 4\n" +
		"f 1 4 2\n" +
		"f 2 4 3\n"

	m := ReadOBJ(strings.NewReader(objText))
	defer runtime.KeepAlive(m)
	if m.Status() != NoError {
		t.Fatalf("ReadOBJ status: %v", m.Status())
	}
	if got := m.NumVert(); got != 4 {
		t.Errorf("NumVert: got %d, want 4", got)
	}
	if got := m.NumTri(); got != 4 {
		t.Errorf("NumTri: got %d, want 4", got)
	}
}

// TestWriteOBJ_GoToCppRead_Differential is the cross-impl check: Go
// writes a tetrahedron to OBJ, the C++ reader parses it, and the
// resulting Manifold must match the original on counts/volume/bbox.
// Failure here means Go's OBJ output is not C++-compatible.
func TestWriteOBJ_GoToCppRead_Differential(t *testing.T) {
	hRef := cppref.Tetrahedron()
	defer cppref.DeleteManifold(hRef)
	src := fromRefHandle(hRef)

	var buf bytes.Buffer
	if !src.WriteOBJ(&buf) {
		t.Fatal("WriteOBJ failed")
	}

	hRoundTrip := cppref.ReadOBJ(buf.String())
	defer cppref.DeleteManifold(hRoundTrip)

	if got, want := cppref.NumVert(hRoundTrip), src.NumVert(); got != want {
		t.Errorf("C++-read NumVert: got %d, want %d", got, want)
	}
	if got, want := cppref.NumTri(hRoundTrip), src.NumTri(); got != want {
		t.Errorf("C++-read NumTri: got %d, want %d", got, want)
	}
	if !floatClose(cppref.Volume(hRoundTrip), src.Volume(), 1e-15, 1e-15) {
		t.Errorf("C++-read Volume: got %v, want %v",
			cppref.Volume(hRoundTrip), src.Volume())
	}
	assertSameBoundingBox(t, src.refHandle(), hRoundTrip, 1e-15)
}

// TestReadOBJ_FacesWithSlashes mirrors the C++ FACE_ELEMENT pattern,
// which strips "/<anything>" suffixes from face indices (so OBJ files
// from other tools that include texture/normal indices still parse).
func TestReadOBJ_FacesWithSlashes(t *testing.T) {
	objText := "# tolerance = 0.001\n" +
		"v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n" +
		"f 1/1/1 2/2/2 3/3/3\n" +
		"f 1//1 3//3 4//4\n" +
		"f 1 4 2\n" +
		"f 2 4 3\n"

	m := ReadOBJ(strings.NewReader(objText))
	defer runtime.KeepAlive(m)
	if m.Status() != NoError {
		t.Fatalf("ReadOBJ status: %v", m.Status())
	}
	if got := m.NumTri(); got != 4 {
		t.Errorf("NumTri: got %d, want 4", got)
	}
}
