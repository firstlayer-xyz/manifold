package multimap

import "testing"

func TestMultimap_FIFOAndOrder(t *testing.T) {
	m := New[int, string]()
	if !m.Empty() {
		t.Fatal("fresh multimap should be Empty")
	}

	// FIFO among equal keys.
	m.Emplace(1, "a")
	m.Emplace(1, "b")
	if m.Empty() {
		t.Fatal("not Empty after Emplace")
	}
	if v, ok := m.FindFirst(1); !ok || v != "a" {
		t.Errorf("FindFirst(1) = %q,%v; want \"a\",true (FIFO front)", v, ok)
	}
	m.EraseFirst(1)
	if v, ok := m.FindFirst(1); !ok || v != "b" {
		t.Errorf("after EraseFirst, FindFirst(1) = %q,%v; want \"b\",true", v, ok)
	}

	// BeginValue is the first value of the SMALLEST key, regardless of insert order.
	m.Emplace(3, "x")
	m.Emplace(2, "z")
	if v := m.BeginValue(); v != "b" {
		t.Errorf("BeginValue() = %q; want \"b\" (smallest key 1's front)", v)
	}

	// Absent key.
	if _, ok := m.FindFirst(99); ok {
		t.Error("FindFirst(99) should report absent")
	}
}

func TestMultimap_EraseToEmpty(t *testing.T) {
	m := New[int, int]()
	m.Emplace(5, 50)
	m.Emplace(5, 51)
	m.EraseFirst(5)
	m.EraseFirst(5)
	if !m.Empty() {
		t.Error("multimap should be Empty after erasing all elements")
	}
}
