package orderedmap

import (
	"slices"
	"testing"
)

func TestOrderedMap_BasicCRUD(t *testing.T) {
	m := New[int, string]()
	if m.Len() != 0 {
		t.Errorf("empty Len = %d, want 0", m.Len())
	}
	m.Set(3, "three")
	m.Set(1, "one")
	m.Set(2, "two")
	if m.Len() != 3 {
		t.Errorf("Len after 3 Set = %d, want 3", m.Len())
	}
	if v, ok := m.Get(2); !ok || v != "two" {
		t.Errorf("Get(2) = (%q, %v), want (\"two\", true)", v, ok)
	}
	if _, ok := m.Get(99); ok {
		t.Errorf("Get(99) on absent key returned ok=true")
	}
	m.Set(2, "TWO") // overwrite
	if v, _ := m.Get(2); v != "TWO" {
		t.Errorf("after overwrite Get(2) = %q, want \"TWO\"", v)
	}
	if m.Len() != 3 {
		t.Errorf("Len after overwrite = %d, want 3 (no new entry)", m.Len())
	}
	if !m.Delete(2) {
		t.Errorf("Delete(2) returned false on present key")
	}
	if m.Delete(2) {
		t.Errorf("Delete(2) on already-deleted key returned true")
	}
	if m.Len() != 2 {
		t.Errorf("Len after Delete = %d, want 2", m.Len())
	}
}

func TestOrderedMap_SortedIteration(t *testing.T) {
	// Insert keys in random-ish order; iteration must be in sorted order.
	m := New[int, int]()
	for _, k := range []int{7, 3, 11, 1, 9, 5, 13, 2} {
		m.Set(k, k*10)
	}
	got := make([]int, 0, m.Len())
	for k, v := range m.All() {
		if v != k*10 {
			t.Errorf("All: pair (%d, %d) inconsistent (want value %d)", k, v, k*10)
		}
		got = append(got, k)
	}
	want := []int{1, 2, 3, 5, 7, 9, 11, 13}
	if !slices.Equal(got, want) {
		t.Errorf("iteration order = %v, want %v", got, want)
	}
}

func TestOrderedMap_KeysSnapshot(t *testing.T) {
	m := New[int, struct{}]()
	for _, k := range []int{5, 1, 3} {
		m.Set(k, struct{}{})
	}
	want := []int{1, 3, 5}
	if !slices.Equal(m.Keys(), want) {
		t.Errorf("Keys() = %v, want %v", m.Keys(), want)
	}
}

func TestOrderedMap_Contains(t *testing.T) {
	m := New[string, int]()
	m.Set("a", 1)
	if !m.Contains("a") {
		t.Error("Contains(\"a\") = false, want true")
	}
	if m.Contains("b") {
		t.Error("Contains(\"b\") = true, want false on absent key")
	}
}

func TestOrderedMap_Clear(t *testing.T) {
	m := New[int, int]()
	for i := 0; i < 10; i++ {
		m.Set(i, i)
	}
	m.Clear()
	if m.Len() != 0 {
		t.Errorf("Len after Clear = %d, want 0", m.Len())
	}
	if len(m.Keys()) != 0 {
		t.Errorf("Keys() after Clear has %d entries, want 0", len(m.Keys()))
	}
	// Insert after clear should work.
	m.Set(42, 42)
	if v, _ := m.Get(42); v != 42 {
		t.Errorf("Get after Clear+Set = %d, want 42", v)
	}
}

func TestOrderedMap_BreakEarly(t *testing.T) {
	m := New[int, int]()
	for i := 0; i < 5; i++ {
		m.Set(i, i)
	}
	count := 0
	for k := range m.All() {
		count++
		if k == 2 {
			break // iter.Seq2 must honor early break via false yield
		}
	}
	if count != 3 {
		t.Errorf("early break visited %d keys, want 3", count)
	}
}
