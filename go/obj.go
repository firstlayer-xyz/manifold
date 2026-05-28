package manifold

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"regexp"
	"runtime"
	"sort"
	"strconv"
	"strings"

	"github.com/firstlayer-xyz/manifold/go/bridge"
)

// writeOBJWithEpsilon mirrors the C++ static
// WriteOBJWithEpsilon(stream, mesh, epsilon) helper in
// src/impl.cpp. Output format is the same:
//   "# ======= begin mesh ======"
//   "# float_format = fixed" | "hexfloat"
//   "# tolerance = <X>"
//   ["# epsilon = <X>" — only when epsilon != nil]
//   "v <x> <y> <z>" — one per vertex
//   "f <i> <j> <k>" — one per triangle, 1-indexed, sorted lexicographically
//   "# ======== end mesh ======="
//
// hexFloat output activates when the MANIFOLD_OBJ_HEX_FLOAT env var is
// "1", "true", "TRUE", "on", or "ON". Hex output mirrors C printf
// "%.13a"; the reader only parses the fixed-decimal form, just as the
// C++ reader does.
func writeOBJWithEpsilon(w io.Writer, mesh MeshGL64, epsilon *float64) error {
	hexFloat := useHexFloat()

	writeValue := func(v float64) error {
		if hexFloat {
			_, err := fmt.Fprint(w, formatHexFloatLikeCPrintf(v))
			return err
		}
		// std::setprecision(19) << std::fixed → 19 digits after the decimal.
		_, err := fmt.Fprintf(w, "%.19f", v)
		return err
	}

	floatFormat := "fixed"
	if hexFloat {
		floatFormat = "hexfloat"
	}
	if _, err := fmt.Fprintln(w, "# ======= begin mesh ======"); err != nil {
		return err
	}
	if _, err := fmt.Fprintf(w, "# float_format = %s\n", floatFormat); err != nil {
		return err
	}
	if _, err := fmt.Fprint(w, "# tolerance = "); err != nil {
		return err
	}
	if err := writeValue(mesh.Tolerance); err != nil {
		return err
	}
	if _, err := fmt.Fprintln(w); err != nil {
		return err
	}
	if epsilon != nil {
		if _, err := fmt.Fprint(w, "# epsilon = "); err != nil {
			return err
		}
		if err := writeValue(*epsilon); err != nil {
			return err
		}
		if _, err := fmt.Fprintln(w); err != nil {
			return err
		}
	}
	numVert := 0
	if mesh.NumProp > 0 {
		numVert = len(mesh.VertProperties) / mesh.NumProp
	}
	for i := 0; i < numVert; i++ {
		if _, err := fmt.Fprint(w, "v"); err != nil {
			return err
		}
		offset := i * mesh.NumProp
		for j := 0; j < 3; j++ {
			if _, err := fmt.Fprint(w, " "); err != nil {
				return err
			}
			if err := writeValue(mesh.VertProperties[offset+j]); err != nil {
				return err
			}
		}
		if _, err := fmt.Fprintln(w); err != nil {
			return err
		}
	}
	numTri := len(mesh.TriVerts) / 3
	triangles := make([][3]uint64, numTri)
	for i := 0; i < numTri; i++ {
		triangles[i] = [3]uint64{
			mesh.TriVerts[3*i] + 1,
			mesh.TriVerts[3*i+1] + 1,
			mesh.TriVerts[3*i+2] + 1,
		}
	}
	sort.Slice(triangles, func(i, j int) bool {
		a, b := triangles[i], triangles[j]
		if a[0] != b[0] {
			return a[0] < b[0]
		}
		if a[1] != b[1] {
			return a[1] < b[1]
		}
		return a[2] < b[2]
	})
	for _, t := range triangles {
		if _, err := fmt.Fprintf(w, "f %d %d %d\n", t[0], t[1], t[2]); err != nil {
			return err
		}
	}
	if _, err := fmt.Fprintln(w, "# ======== end mesh ======="); err != nil {
		return err
	}
	return nil
}

func useHexFloat() bool {
	v := os.Getenv("MANIFOLD_OBJ_HEX_FLOAT")
	switch v {
	case "1", "true", "TRUE", "on", "ON":
		return true
	}
	return false
}

// formatHexFloatLikeCPrintf renders v in the same hexfloat layout as
// C printf "%.13a" — single-digit exponent (no leading zeros), 13 hex
// digits after the point. Go's "%.13x" yields a two-digit exponent so
// we have to format manually.
func formatHexFloatLikeCPrintf(v float64) string {
	s := strconv.FormatFloat(v, 'x', 13, 64)
	pIdx := strings.LastIndexByte(s, 'p')
	if pIdx < 0 {
		return s
	}
	mantissa := s[:pIdx]
	expPart := s[pIdx+1:]
	if len(expPart) < 2 {
		return s
	}
	sign := expPart[0]
	digits := strings.TrimLeft(expPart[1:], "0")
	if digits == "" {
		digits = "0"
	}
	return fmt.Sprintf("%sp%c%s", mantissa, sign, digits)
}

// readOBJWithEpsilon mirrors the C++ static ReadOBJWithEpsilon helper
// in src/impl.cpp. It reads lines from r and matches them against four
// patterns:
//   "# tolerance = <X>"   → mesh.Tolerance
//   "# epsilon = <X>"     → out epsilon
//   "v <x> <y> <z>"       → append to mesh.VertProperties
//   "f <i>(/...) <j>(/...) <k>(/...)" → append to mesh.TriVerts (0-indexed)
// Lines longer than BUFFER_SIZE (1000 chars) are skipped, matching C++.
// Returns the parsed mesh and an optional epsilon (nil if absent).
func readOBJWithEpsilon(r io.Reader) (MeshGL64, *float64) {
	mesh := MeshGL64{NumProp: 3}
	var epsilon *float64
	if r == nil {
		return mesh, nil
	}
	br := bufio.NewReader(r)
	const bufSize = 1000
	for {
		line, isPrefix, err := br.ReadLine()
		if err == io.EOF {
			break
		}
		if err != nil {
			break
		}
		if isPrefix {
			// Drain the rest of the over-long line; skip it (C++ does the same).
			for isPrefix {
				_, isPrefix, err = br.ReadLine()
				if err != nil {
					break
				}
			}
			continue
		}
		if len(line) >= bufSize {
			continue
		}
		s := string(line)
		if m := objTolerancePattern.FindStringSubmatch(s); m != nil {
			v, perr := strconv.ParseFloat(m[1], 64)
			if perr == nil {
				mesh.Tolerance = v
			}
			continue
		}
		if m := objEpsilonPattern.FindStringSubmatch(s); m != nil {
			v, perr := strconv.ParseFloat(m[1], 64)
			if perr == nil {
				epsilon = &v
			}
			continue
		}
		if m := objVertexPattern.FindStringSubmatch(s); m != nil {
			for j := 0; j < 3; j++ {
				v, perr := strconv.ParseFloat(m[j+1], 64)
				if perr == nil {
					mesh.VertProperties = append(mesh.VertProperties, v)
				}
			}
			continue
		}
		if m := objFacePattern.FindStringSubmatch(s); m != nil {
			for j := 0; j < 3; j++ {
				v, perr := strconv.ParseUint(m[j+1], 10, 64)
				if perr == nil {
					mesh.TriVerts = append(mesh.TriVerts, v-1)
				}
			}
			continue
		}
	}
	return mesh, epsilon
}

// Patterns mirror the C++ regex strings in ReadOBJWithEpsilon.
var (
	objFloat            = `(-?\d+(?:\.\d*)?(?:[eE][+\-]?\d+)?)`
	objFaceElement      = `(\d+)(?:\S+)?`
	objTrailing         = `(?:\s*)`
	objSep              = `\s+`
	objTolerancePattern = regexp.MustCompile(`^# tolerance = ` + objFloat + objTrailing + `$`)
	objEpsilonPattern   = regexp.MustCompile(`^# epsilon = ` + objFloat + objTrailing + `$`)
	objVertexPattern    = regexp.MustCompile(`^v` + objSep + objFloat + objSep + objFloat + objSep + objFloat + objTrailing + `$`)
	objFacePattern      = regexp.MustCompile(`^f` + objSep + objFaceElement + objSep + objFaceElement + objSep + objFaceElement + objTrailing + `$`)
)

// ReadOBJ reads a Wavefront OBJ from r and constructs a Manifold. The
// `# tolerance = X` and optional `# epsilon = X` comments emitted by
// WriteOBJ are honored.
//
// Ported top-down from C++:
//   Manifold Manifold::ReadOBJ(std::istream& stream) {
//     if (!stream.good()) return Invalid();
//     auto [mesh, epsilon] = ReadOBJWithEpsilon(stream);
//     auto impl = std::make_shared<Impl>(mesh);
//     if (epsilon) impl->SetEpsilon(epsilon.value());
//     return Manifold(impl);
//   }
func ReadOBJ(r io.Reader) *Manifold {
	if r == nil {
		return invalidManifold()
	}
	mesh, epsilon := readOBJWithEpsilon(r)
	// std::make_shared<Impl>(mesh) — build a mutable impl from the
	// mesh, optionally adjust epsilon, then wrap as Manifold.
	m := NewManifoldFromMeshGL64(mesh)
	if epsilon != nil {
		// Mirror impl->SetEpsilon(epsilon.value()). We need a mutable
		// view: copy the impl, set epsilon, re-wrap.
		impl := bridge.GetImpl(m.h)
		defer impl.Delete()
		mut := impl.Copy()
		defer mut.Delete()
		mut.SetEpsilonMin(*epsilon)
		runtime.KeepAlive(m)
		return wrap(mut.ToManifold())
	}
	return m
}

// WriteOBJ writes m to w in Wavefront OBJ format, with header comments
// recording tolerance and epsilon so that ReadOBJ can round-trip.
//
// Ported top-down from C++:
//   bool Manifold::WriteOBJ(std::ostream& stream) const {
//     if (!stream.good()) return false;
//     stream << *this->GetCsgLeafNode().GetImpl();   // == WriteOBJWithEpsilon(stream, GetMeshGL64(-1), {epsilon})
//     return true;
//   }
func (m *Manifold) WriteOBJ(w io.Writer) bool {
	if w == nil {
		return false
	}
	mesh := m.GetMeshGL64(-1)
	eps := m.GetEpsilon()
	if err := writeOBJWithEpsilon(w, mesh, &eps); err != nil {
		return false
	}
	return true
}
