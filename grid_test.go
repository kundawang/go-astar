package astar

// grid_test.go verifies the dynamically updatable Grid navigator: door
// toggling, mud detours, corner-cutting rules, deterministic ties, repeated
// incremental updates, error handling and parity with the original Path API.

import (
	"errors"
	"math"
	"strings"
	"testing"
)

const floatTol = 1e-12

func approx(a, b float64) bool { return math.Abs(a-b) <= floatTol }

// renderGrid draws a grid with the path marked as '#', blocked cells as 'X' and
// every other cell as '.'.  It is used in test logs for manual inspection.
func renderGrid(g *Grid, path []Cell) string {
	onPath := map[Cell]bool{}
	for _, c := range path {
		onPath[c] = true
	}
	var b strings.Builder
	for y := 0; y < g.Height(); y++ {
		for x := 0; x < g.Width(); x++ {
			switch {
			case onPath[Cell{x, y}]:
				b.WriteByte('#')
			case g.Blocked(x, y):
				b.WriteByte('X')
			default:
				b.WriteByte('.')
			}
		}
		b.WriteByte('\n')
	}
	return b.String()
}

// assertPath walks the path checking endpoints, adjacency under the mode and
// that no blocked cell is used.
func assertPath(t *testing.T, g *Grid, from, to Cell, path []Cell, diagOK bool) {
	t.Helper()
	if len(path) == 0 {
		t.Fatal("expected a non-empty path")
	}
	if path[0] != from {
		t.Fatalf("path must start at %v, got %v", from, path[0])
	}
	if path[len(path)-1] != to {
		t.Fatalf("path must end at %v, got %v", to, path[len(path)-1])
	}
	for i, c := range path {
		if g.Blocked(c.X, c.Y) {
			t.Fatalf("path step %d passes through blocked cell %v", i, c)
		}
		if i == 0 {
			continue
		}
		prev := path[i-1]
		dx, dy := c.X-prev.X, c.Y-prev.Y
		if dx < 0 {
			dx = -dx
		}
		if dy < 0 {
			dy = -dy
		}
		switch {
		case dx+dy == 1: // orthogonal step
		case dx == 1 && dy == 1 && diagOK:
		default:
			t.Fatalf("illegal step %v -> %v under mode %v", prev, c, g.Mode())
		}
	}
}

// TestGridDoorToggle closes a gate on a guard's planned route and reopens it,
// checking the path and total cost after every change.
func TestGridDoorToggle(t *testing.T) {
	// 5x5 world, guard at (2,2), gate at (2,3) directly above the post.
	g, err := NewGrid(5, 5, Neighbor4)
	if err != nil {
		t.Fatal(err)
	}
	from, gate, to := Cell{2, 2}, Cell{2, 3}, Cell{2, 3}

	// Open gate: straight through, one edge of cost 1.
	path, cost, err := g.FindPath(from, to)
	if err != nil {
		t.Fatalf("open gate: %v", err)
	}
	assertPath(t, g, from, to, path, false)
	if !approx(cost, 1) || len(path) != 2 {
		t.Fatalf("open gate: got cost %v, %d steps", cost, len(path))
	}

	// Close the gate: the guard must reroute around the single blocked cell.
	if err := g.SetBlocked(gate.X, gate.Y); err != nil {
		t.Fatal(err)
	}
	if _, _, err := g.FindPath(from, to); !errors.Is(err, ErrGoalBlocked) {
		t.Fatalf("blocked goal should report ErrGoalBlocked, got %v", err)
	}
	// Move the post to (2,4) so the gate merely sits on the route.
	to = Cell{2, 4}
	path, cost, err = g.FindPath(from, to)
	if err != nil {
		t.Fatalf("closed gate: %v", err)
	}
	t.Logf("Closed-gate detour:\n%s", renderGrid(g, path))
	assertPath(t, g, from, to, path, false)
	if !approx(cost, 4) {
		t.Fatalf("closed gate detour: expected cost 4, got %v", cost)
	}
	detour := append([]Cell(nil), path...)

	// Repeated queries while nothing changes must give byte-identical paths.
	for i := 0; i < 5; i++ {
		p, _, err := g.FindPath(from, to)
		if err != nil {
			t.Fatal(err)
		}
		if len(p) != len(detour) {
			t.Fatalf("unstable path length on query %d", i)
		}
		for j := range p {
			if p[j] != detour[j] {
				t.Fatalf("unstable path at query %d step %d: %v != %v", i, j, p[j], detour[j])
			}
		}
	}

	// Reopen the gate: the original two-edge straight route comes back.
	if err := g.SetCost(gate.X, gate.Y, 1); err != nil {
		t.Fatal(err)
	}
	path, cost, err = g.FindPath(from, to)
	if err != nil {
		t.Fatal(err)
	}
	assertPath(t, g, from, to, path, false)
	if !approx(cost, 2) || len(path) != 3 || path[1] != gate {
		t.Fatalf("reopened gate: got cost %v, path %v", cost, path)
	}
}

// TestGridMudDetour uses expensive mud cells to force a choice between the
// cheap high road and the direct mud road, then blocks and restores the high
// road.
func TestGridMud(t *testing.T) {
	const w, h = 6, 2
	g, err := NewGrid(w, h, Neighbor4)
	if err != nil {
		t.Fatal(err)
	}
	from, to := Cell{0, 1}, Cell{5, 1}
	// Bottom row between the endpoints is mud with entry cost 5.
	for x := 1; x <= 4; x++ {
		if err := g.SetCost(x, 1, 5); err != nil {
			t.Fatal(err)
		}
	}

	// High road across the plain row: seven unit edges.
	path, cost, err := g.FindPath(from, to)
	if err != nil {
		t.Fatal(err)
	}
	t.Logf("Mud detour, high road:\n%s", renderGrid(g, path))
	assertPath(t, g, from, to, path, false)
	if !approx(cost, 7) {
		t.Fatalf("high road: expected cost 7, got %v", cost)
	}

	// Block the whole high road; the only option is wading through the mud.
	updates := make([]CellUpdate, w)
	for x := 0; x < w; x++ {
		updates[x] = CellUpdate{X: x, Y: 0, Blocked: true}
	}
	if err := g.Update(updates...); err != nil {
		t.Fatal(err)
	}
	path, cost, err = g.FindPath(from, to)
	if err != nil {
		t.Fatalf("mud road: %v", err)
	}
	t.Logf("Mud detour, mud road:\n%s", renderGrid(g, path))
	assertPath(t, g, from, to, path, false)
	// Four mud entries at 5, plus the final plain entry at 1.
	if !approx(cost, 4*5+1) {
		t.Fatalf("mud road: expected cost 21, got %v", cost)
	}

	// Reopen the high road with a second batch update and expect the cheap
	// route back.
	for i := range updates {
		updates[i] = CellUpdate{X: updates[i].X, Y: 0, Cost: 1}
	}
	if err := g.Update(updates...); err != nil {
		t.Fatal(err)
	}
	_, cost, err = g.FindPath(from, to)
	if err != nil {
		t.Fatal(err)
	}
	if !approx(cost, 7) {
		t.Fatalf("restored high road: expected cost 7, got %v", cost)
	}
}

// TestGridCornerCutting verifies that diagonal moves never cross a wall
// corner and that diagonal movement cost is sqrt2 times terrain cost.
func TestGridCornerCutting(t *testing.T) {
	from, to := Cell{0, 0}, Cell{1, 1}

	// 2x2 with both orthogonal brackets blocked: the only move to the goal
	// would be a corner-cutting diagonal, which is forbidden, so no path
	// exists at all.
	g, err := NewGrid(2, 2, Neighbor8)
	if err != nil {
		t.Fatal(err)
	}
	if err := g.SetBlocked(0, 1); err != nil {
		t.Fatal(err)
	}
	if err := g.SetBlocked(1, 0); err != nil {
		t.Fatal(err)
	}
	if _, _, err := g.FindPath(from, to); !errors.Is(err, ErrNoPath) {
		t.Fatalf("two-wall corner: expected ErrNoPath, got %v", err)
	}

	// One bracket blocked: the diagonal is still forbidden, but two orthogonal
	// edges give a deterministic route of total cost 2.
	g2, err := NewGrid(2, 2, Neighbor8)
	if err != nil {
		t.Fatal(err)
	}
	if err := g2.SetBlocked(0, 1); err != nil {
		t.Fatal(err)
	}
	path, cost, err := g2.FindPath(from, to)
	if err != nil {
		t.Fatalf("one-wall corner: %v", err)
	}
	if !approx(cost, 2) {
		t.Fatalf("blocked corner: expected cost 2, got %v", cost)
	}
	want := []Cell{{0, 0}, {1, 0}, {1, 1}}
	if len(path) != len(want) {
		t.Fatalf("blocked corner: expected %v, got %v", want, path)
	}
	for i := range want {
		if path[i] != want[i] {
			t.Fatalf("blocked corner: expected %v, got %v", want, path)
		}
	}

	// Fully open 2x2: one free diagonal costs sqrt2.
	g3, err := NewGrid(2, 2, Neighbor8)
	if err != nil {
		t.Fatal(err)
	}
	path, cost, err = g3.FindPath(from, to)
	if err != nil {
		t.Fatal(err)
	}
	if len(path) != 2 || !approx(cost, math.Sqrt2) {
		t.Fatalf("free diagonal: expected 2 cells at cost %v, got %v (path %v)", math.Sqrt2, cost, path)
	}

	// 3x3 with the centre blocked: an orthogonal detour exists and every
	// diagonal step on it must still have both brackets passable.
	g4, err := NewGrid(3, 3, Neighbor8)
	if err != nil {
		t.Fatal(err)
	}
	if err := g4.SetBlocked(1, 1); err != nil {
		t.Fatal(err)
	}
	from4, to4 := Cell{0, 0}, Cell{2, 2}
	path, cost, err = g4.FindPath(from4, to4)
	if err != nil {
		t.Fatalf("centre block: %v", err)
	}
	t.Logf("Centre-block detour:\n%s", renderGrid(g4, path))
	assertPath(t, g4, from4, to4, path, true)
	if !approx(cost, 4) {
		t.Fatalf("centre block: expected cost 4, got %v", cost)
	}
	for i := 1; i < len(path); i++ {
		prev, cur := path[i-1], path[i]
		if cur.X-prev.X != 0 && cur.Y-prev.Y != 0 {
			if g4.Blocked(prev.X, cur.Y) || g4.Blocked(cur.X, prev.Y) {
				t.Fatalf("diagonal step %v->%v cuts a wall corner", prev, cur)
			}
		}
	}
}

// TestGridStableTies checks that equal-cost ties always resolve to the same
// path across repeated queries and independently built grids.
func TestGridStableTies(t *testing.T) {
	for _, mode := range []NeighborMode{Neighbor4, Neighbor8} {
		g, err := NewGrid(6, 6, mode)
		if err != nil {
			t.Fatal(err)
		}
		from, to := Cell{0, 0}, Cell{5, 5}
		first, firstCost, err := g.FindPath(from, to)
		if err != nil {
			t.Fatal(err)
		}
		for i := 0; i < 10; i++ {
			// No-op updates must not perturb the chosen path.
			if err := g.SetCost(0, 0, 1); err != nil {
				t.Fatal(err)
			}
			p, c, err := g.FindPath(from, to)
			if err != nil {
				t.Fatal(err)
			}
			if !approx(c, firstCost) || len(p) != len(first) {
				t.Fatalf("mode %v: unstable result at query %d", mode, i)
			}
			for j := range p {
				if p[j] != first[j] {
					t.Fatalf("mode %v: unstable path at query %d step %d", mode, i, j)
				}
			}
		}

		// An independently constructed, identical grid must agree as well.
		g2, err := NewGrid(6, 6, mode)
		if err != nil {
			t.Fatal(err)
		}
		other, otherCost, err := g2.FindPath(from, to)
		if err != nil {
			t.Fatal(err)
		}
		if !approx(otherCost, firstCost) || len(other) != len(first) {
			t.Fatalf("mode %v: independent grids disagree on cost", mode)
		}
		for j := range other {
			if other[j] != first[j] {
				t.Fatalf("mode %v: independent grids disagree at step %d", mode, j)
			}
		}

		want := 10.0
		if mode == Neighbor8 {
			want = 5 * math.Sqrt2
		}
		if !approx(firstCost, want) {
			t.Fatalf("mode %v: expected cost %v, got %v", mode, want, firstCost)
		}
	}
}

// TestGridIncrementalUpdates applies several successive small updates to one
// persistent grid and re-queries after each.
func TestGridIncrementalUpdates(t *testing.T) {
	g, err := NewGrid(5, 5, Neighbor4)
	if err != nil {
		t.Fatal(err)
	}
	from, to := Cell{0, 0}, Cell{4, 4}

	path, cost, err := g.FindPath(from, to)
	if err != nil {
		t.Fatal(err)
	}
	assertPath(t, g, from, to, path, false)
	if !approx(cost, 8) {
		t.Fatalf("open grid: expected 8, got %v", cost)
	}

	// Build a vertical wall at x=2 with a single door at (2,2); the Manhattan
	// optimum through the door still costs 8.
	wall := []CellUpdate{
		{X: 2, Y: 0, Blocked: true},
		{X: 2, Y: 1, Blocked: true},
		{X: 2, Y: 3, Blocked: true},
		{X: 2, Y: 4, Blocked: true},
	}
	if err := g.Update(wall...); err != nil {
		t.Fatal(err)
	}
	path, cost, err = g.FindPath(from, to)
	if err != nil {
		t.Fatalf("wall with door: %v", err)
	}
	t.Logf("Wall with door:\n%s", renderGrid(g, path))
	assertPath(t, g, from, to, path, false)
	if !approx(cost, 8) {
		t.Fatalf("door route: expected 8, got %v", cost)
	}
	foundDoor := false
	for _, c := range path {
		if c == (Cell{2, 2}) {
			foundDoor = true
		}
	}
	if !foundDoor {
		t.Fatalf("path must pass through the door (2,2): %v", path)
	}

	// Close the door: the wall spans the whole grid, no route exists.
	if err := g.SetBlocked(2, 2); err != nil {
		t.Fatal(err)
	}
	if _, _, err := g.FindPath(from, to); !errors.Is(err, ErrNoPath) {
		t.Fatalf("closed door: expected ErrNoPath, got %v", err)
	}

	// Reopen it with a terrain upgrade; route and cost are restored.
	if err := g.Update(CellUpdate{X: 2, Y: 2, Cost: 1}); err != nil {
		t.Fatal(err)
	}
	_, cost, err = g.FindPath(from, to)
	if err != nil {
		t.Fatal(err)
	}
	if !approx(cost, 8) {
		t.Fatalf("reopened door: expected 8, got %v", cost)
	}

	// Blocking and restoring the goal endpoint reports the specific error.
	if err := g.SetBlocked(to.X, to.Y); err != nil {
		t.Fatal(err)
	}
	if _, _, err := g.FindPath(from, to); !errors.Is(err, ErrGoalBlocked) {
		t.Fatalf("blocked goal: expected ErrGoalBlocked, got %v", err)
	}
	if err := g.SetCost(to.X, to.Y, 1); err != nil {
		t.Fatal(err)
	}
	if _, _, err = g.FindPath(from, to); err != nil {
		t.Fatalf("restored goal: %v", err)
	}
}

// TestGridErrors covers every documented failure mode.
func TestGridErrors(t *testing.T) {
	if _, err := NewGrid(0, 5, Neighbor4); !errors.Is(err, ErrInvalidDimensions) {
		t.Fatalf("zero width: got %v", err)
	}
	if _, err := NewGrid(3, -1, Neighbor4); !errors.Is(err, ErrInvalidDimensions) {
		t.Fatalf("negative height: got %v", err)
	}
	if _, err := NewGrid(3, 3, NeighborMode(99)); !errors.Is(err, ErrInvalidMode) {
		t.Fatalf("bad mode: got %v", err)
	}

	g, err := NewGrid(3, 3, Neighbor8)
	if err != nil {
		t.Fatal(err)
	}
	if err := g.SetCost(3, 0, 1); !errors.Is(err, ErrOutOfBounds) {
		t.Fatalf("SetCost OOB: got %v", err)
	}
	if err := g.SetCost(0, -1, 1); !errors.Is(err, ErrOutOfBounds) {
		t.Fatalf("SetCost negative: got %v", err)
	}
	if err := g.SetBlocked(9, 9); !errors.Is(err, ErrOutOfBounds) {
		t.Fatalf("SetBlocked OOB: got %v", err)
	}
	for _, bad := range []float64{-0.01, math.NaN(), math.Inf(1), math.Inf(-1)} {
		if err := g.SetCost(1, 1, bad); !errors.Is(err, ErrInvalidCost) {
			t.Fatalf("cost %v: got %v", bad, err)
		}
	}
	// Zero and sub-normal-but-finite costs are legitimate.
	if err := g.SetCost(1, 1, 0); err != nil {
		t.Fatalf("zero cost: %v", err)
	}
	if c, ok := g.Cost(1, 1); !ok || !approx(c, 0) {
		t.Fatalf("zero cost cell reports (%v, %v)", c, ok)
	}

	if _, _, err := g.FindPath(Cell{0, 0}, Cell{9, 0}); !errors.Is(err, ErrOutOfBounds) {
		t.Fatalf("goal OOB: got %v", err)
	}
	if _, _, err := g.FindPath(Cell{-1, 0}, Cell{0, 0}); !errors.Is(err, ErrOutOfBounds) {
		t.Fatalf("start OOB: got %v", err)
	}

	if err := g.SetBlocked(0, 0); err != nil {
		t.Fatal(err)
	}
	if _, _, err := g.FindPath(Cell{0, 0}, Cell{2, 2}); !errors.Is(err, ErrStartBlocked) {
		t.Fatalf("blocked start: got %v", err)
	}
	if err := g.SetBlocked(2, 2); err != nil {
		t.Fatal(err)
	}
	if _, _, err := g.FindPath(Cell{1, 1}, Cell{2, 2}); !errors.Is(err, ErrGoalBlocked) {
		t.Fatalf("blocked goal: got %v", err)
	}
	if _, _, err := g.FindPath(Cell{0, 0}, Cell{0, 0}); !errors.Is(err, ErrStartBlocked) {
		t.Fatalf("blocked self-query: got %v", err)
	}

	// A wall separating the only two regions yields ErrNoPath.
	g2, err := NewGrid(3, 3, Neighbor4)
	if err != nil {
		t.Fatal(err)
	}
	for y := 0; y < 3; y++ {
		if err := g2.SetBlocked(1, y); err != nil {
			t.Fatal(err)
		}
	}
	if _, _, err := g2.FindPath(Cell{0, 0}, Cell{2, 2}); !errors.Is(err, ErrNoPath) {
		t.Fatalf("unreachable: got %v", err)
	}

	// A self-query on a passable cell returns a zero-cost single-cell path.
	p, cost, err := g2.FindPath(Cell{0, 0}, Cell{0, 0})
	if err != nil || len(p) != 1 || p[0] != (Cell{0, 0}) || cost != 0 {
		t.Fatalf("self-query: path=%v cost=%v err=%v", p, cost, err)
	}

	// Batch updates are atomic: one bad entry rejects the whole batch.
	g3, err := NewGrid(3, 3, Neighbor4)
	if err != nil {
		t.Fatal(err)
	}
	before := g3.costs[0]
	err = g3.Update(
		CellUpdate{X: 0, Y: 0, Cost: 7},
		CellUpdate{X: 8, Y: 8, Blocked: true},
	)
	if !errors.Is(err, ErrOutOfBounds) {
		t.Fatalf("bad batch: got %v", err)
	}
	if g3.costs[0] != before || g3.Blocked(0, 0) {
		t.Fatal("failed batch update partially modified the grid")
	}
	err = g3.Update(
		CellUpdate{X: 0, Y: 0, Cost: 7},
		CellUpdate{X: 1, Y: 1, Cost: math.NaN()},
	)
	if !errors.Is(err, ErrInvalidCost) {
		t.Fatalf("bad batch cost: got %v", err)
	}
	if g3.costs[0] != before {
		t.Fatal("failed cost batch partially modified the grid")
	}

	// Matrix constructor validation.
	if _, err := NewGridFromCosts(2, 2, Neighbor4, []float64{1, 2, 3}); !errors.Is(err, ErrInvalidCosts) {
		t.Fatalf("bad matrix length: got %v", err)
	}
	if _, err := NewGridFromCosts(2, 2, Neighbor4, []float64{1, 2, 3, math.NaN()}); !errors.Is(err, ErrInvalidCost) {
		t.Fatalf("matrix NaN: got %v", err)
	}
	gm, err := NewGridFromCosts(2, 2, Neighbor4, []float64{
		1, 2,
		BlockedCost, 0,
	})
	if err != nil {
		t.Fatal(err)
	}
	if !gm.Blocked(0, 1) {
		t.Fatal("BlockedCost cell should start blocked")
	}
	if c, ok := gm.Cost(1, 1); !ok || c != 0 {
		t.Fatalf("zero-cost matrix cell: (%v,%v)", c, ok)
	}
	p, cost, err = gm.FindPath(Cell{0, 0}, Cell{1, 1})
	if err != nil {
		t.Fatal(err)
	}
	// The 4-neighbour route must pass the cost-2 cell (1,0) or (0,1).
	if !approx(cost, 2) {
		t.Fatalf("matrix route: expected cost 2, got %v (path %v)", cost, p)
	}
	// A route ending on the zero-cost cell confirms zero terrain is honored.
	_, cost, err = gm.FindPath(Cell{1, 0}, Cell{1, 1})
	if err != nil {
		t.Fatal(err)
	}
	if !approx(cost, 0) {
		t.Fatalf("path over zero-cost cell: expected 0, got %v", cost)
	}

	// Methods on a zero-value (empty) grid fail clearly.
	var empty Grid
	if _, _, err := empty.FindPath(Cell{0, 0}, Cell{0, 0}); !errors.Is(err, ErrInvalidDimensions) {
		t.Fatalf("empty FindPath: got %v", err)
	}
	if err := empty.SetCost(0, 0, 1); !errors.Is(err, ErrInvalidDimensions) {
		t.Fatalf("empty SetCost: got %v", err)
	}
	if err := empty.SetBlocked(0, 0); !errors.Is(err, ErrInvalidDimensions) {
		t.Fatalf("empty SetBlocked: got %v", err)
	}
	if err := empty.Update(CellUpdate{X: 0, Y: 0, Cost: 1}); !errors.Is(err, ErrInvalidDimensions) {
		t.Fatalf("empty Update: got %v", err)
	}
}

// TestGridParityWithPath rebuilds several of the textual Path test worlds as
// Grids and checks that both implementations return the same distances.
func TestGridParityWithPath(t *testing.T) {
	worlds := []string{
		`
.....~......
.....MM.....
.F........T.
....MMM.....
............
`,
		`
.....~......
.....MM.....
.F..MMMM..T.
....MMM.....
............
`,
		`
............
.........XXX
.F.......XTX
.........XXX
............
`,
		`
FX.X........
.X...XXXX.X.
.X.X.X....X.
...X.X.XXXXX
.XX..X.....T
`,
		`
..F..M......
.....MM.....
....MMMM..T.
....MMM.....
............
`,
		`
.....~......
.....~......
.F...X...T..
.....M......
.....M......
`,
	}
	for i, input := range worlds {
		world := ParseWorld(input)
		width, height := len(world), len(world[0])
		costs := make([]float64, width*height)
		for x := 0; x < width; x++ {
			for y := 0; y < height; y++ {
				tile := world.Tile(x, y)
				if tile == nil || tile.Kind == KindBlocker {
					costs[y*width+x] = BlockedCost
				} else {
					costs[y*width+x] = KindCosts[tile.Kind]
				}
			}
		}
		g, err := NewGridFromCosts(width, height, Neighbor4, costs)
		if err != nil {
			t.Fatalf("world %d: %v", i, err)
		}
		f, to := world.From(), world.To()
		wantPath, wantDist, wantFound := Path(f, to)
		gotPath, gotDist, gotErr := g.FindPath(Cell{f.X, f.Y}, Cell{to.X, to.Y})

		if wantFound {
			if gotErr != nil {
				t.Fatalf("world %d: Path found a route but Grid returned %v", i, gotErr)
			}
			if !approx(gotDist, wantDist) {
				t.Fatalf("world %d: Path distance %v != Grid distance %v", i, wantDist, gotDist)
			}
			if len(gotPath) != len(wantPath) {
				t.Fatalf("world %d: path length mismatch %d != %d", i, len(gotPath), len(wantPath))
			}
		} else if !errors.Is(gotErr, ErrNoPath) {
			t.Fatalf("world %d: Path found nothing but Grid returned %v", i, gotErr)
		}
	}
}

// TestGridSteadyStateAllocs ensures repeated queries on an unchanged region do
// not rebuild the map: after warm-up a query must be essentially allocation
// free (only the returned path slice may allocate).
func TestGridSteadyStateAllocs(t *testing.T) {
	g, err := NewGrid(20, 20, Neighbor8)
	if err != nil {
		t.Fatal(err)
	}
	from, to := Cell{0, 0}, Cell{19, 19}
	// Warm up reusable scratch capacity.
	if _, _, err := g.FindPath(from, to); err != nil {
		t.Fatal(err)
	}
	allocs := testing.AllocsPerRun(20, func() {
		p, _, err := g.FindPath(from, to)
		if err != nil || len(p) < 2 {
			t.Fatalf("unexpected query result: %v (path %v)", err, p)
		}
	})
	if allocs > 2 {
		t.Fatalf("steady-state query made %v allocations, expected at most 2 (the returned path)", allocs)
	}
}

// TestGridInPlaceMutation confirms an update touches only the target cell's
// backing storage and leaves unrelated cells byte-identical.
func TestGridInPlaceMutation(t *testing.T) {
	g, err := NewGrid(4, 4, Neighbor4)
	if err != nil {
		t.Fatal(err)
	}
	costsBefore := append([]float64(nil), g.costs...)
	blockedBefore := append([]bool(nil), g.blocked...)

	if err := g.SetCost(2, 2, 3.5); err != nil {
		t.Fatal(err)
	}
	for i := range costsBefore {
		if i == g.id(2, 2) {
			if g.costs[i] != 3.5 {
				t.Fatalf("updated cell cost = %v, want 3.5", g.costs[i])
			}
			continue
		}
		if g.costs[i] != costsBefore[i] || g.blocked[i] != blockedBefore[i] {
			t.Fatalf("unrelated cell %d changed by update", i)
		}
	}
	if g.blocked[g.id(2, 2)] {
		t.Fatal("cost update must leave the cell passable")
	}

	// minCost tracking survives the cheaper cost disappearing.
	g2, err := NewGridFromCosts(1, 3, Neighbor4, []float64{1, 4, 9})
	if err != nil {
		t.Fatal(err)
	}
	if err := g2.SetBlocked(0, 0); err != nil {
		t.Fatal(err)
	}
	if g2.minCost != 4 {
		t.Fatalf("minCost after blocking cheapest cell: got %v, want 4", g2.minCost)
	}
	_, cost, err := g2.FindPath(Cell{0, 1}, Cell{0, 2})
	if err != nil {
		t.Fatal(err)
	}
	if !approx(cost, 9) {
		t.Fatalf("cost after cheapest cell blocked: got %v, want 9", cost)
	}
	if err := g2.SetCost(0, 0, 2); err != nil {
		t.Fatal(err)
	}
	if g2.minCost != 2 {
		t.Fatalf("minCost after cheap cell restored: got %v, want 2", g2.minCost)
	}
}
