package astar

import (
	"errors"
	"testing"
)

// TestGuardScenarioDemo is an end-to-end walkthrough of the motivating story:
// a guard follows a planned route when a gate is open, the gate slams shut, and
// a new route is planned without rebuilding the map.
func TestGuardScenarioDemo(t *testing.T) {
	g, _ := NewGrid(7, 5, Neighbor4)
	// A wall with a gate at (3,2).
	for y := 0; y < 5; y++ {
		if y == 2 {
			continue
		}
		if err := g.SetBlocked(3, y); err != nil {
			t.Fatal(err)
		}
	}
	from, post := Cell{0, 2}, Cell{6, 2}

	path, cost, err := g.FindPath(from, post)
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("Gate open (cost %v):\n%s", cost, renderGrid(g, path))
	if path[3] != (Cell{3, 2}) {
		t.Fatalf("expected to march through the gate, got %v", path)
	}

	// The gate slams shut while the guard is en route.
	if err := g.SetBlocked(3, 2); err != nil {
		t.Fatal(err)
	}
	_, _, err = g.FindPath(from, post)
	if !errors.Is(err, ErrNoPath) {
		t.Fatalf("sealed wall: expected ErrNoPath, got %v", err)
	}

	// A side hatch opens at (3,0); the guard reroutes.
	if err := g.SetCost(3, 0, 1); err != nil {
		t.Fatal(err)
	}
	path, cost, err = g.FindPath(from, post)
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("Hatch open (cost %v):\n%s", cost, renderGrid(g, path))
	if cost != 10 {
		t.Fatalf("expected detour cost 10, got %v", cost)
	}
}
