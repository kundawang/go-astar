package astar

// grid.go provides a fixed-size, dynamically updatable grid navigation layer
// on top of the A* algorithm.  Unlike the generic Pather/Path API which builds
// a fresh node map for every query, Grid keeps its terrain state in persistent
// flat arrays so individual cells can be updated in place and re-queried
// cheaply.  Per-query search state is retained between queries and validated
// with generation stamps, so a query never has to clear the whole grid.

import (
	"errors"
	"fmt"
	"math"
)

// NeighborMode selects which adjacent cells a Grid considers while searching.
type NeighborMode int

const (
	// Neighbor4 restricts movement to the four orthogonal neighbours.
	Neighbor4 NeighborMode = 4
	// Neighbor8 allows the four orthogonal and the four diagonal neighbours.
	Neighbor8 NeighborMode = 8
)

// BlockedCost marks a blocked cell in the cost matrix accepted by
// NewGridFromCosts.  A blocked cell can never be entered.
var BlockedCost = math.Inf(1)

// Sentinel errors returned by the Grid API.  Errors are wrapped with
// fmt.Errorf and %w, so callers can use errors.Is to match them.
var (
	// ErrInvalidDimensions is returned when a grid is constructed with a
	// non-positive width or height, or a method is called on a zero-value
	// (empty) Grid.
	ErrInvalidDimensions = errors.New("astar: grid dimensions must be positive")
	// ErrInvalidMode is returned when a NeighborMode other than Neighbor4 or
	// Neighbor8 is supplied.
	ErrInvalidMode = errors.New("astar: neighbor mode must be Neighbor4 or Neighbor8")
	// ErrInvalidCosts is returned by NewGridFromCosts when the cost slice does
	// not contain exactly width*height entries.
	ErrInvalidCosts = errors.New("astar: costs must contain exactly width*height entries")
	// ErrOutOfBounds is returned when a coordinate lies outside the grid.
	ErrOutOfBounds = errors.New("astar: coordinate is outside the grid")
	// ErrInvalidCost is returned when a cell cost is negative, NaN or infinite.
	ErrInvalidCost = errors.New("astar: cell cost must be finite and non-negative")
	// ErrStartBlocked is returned when the start cell of a path query is
	// blocked.
	ErrStartBlocked = errors.New("astar: start cell is blocked")
	// ErrGoalBlocked is returned when the goal cell of a path query is
	// blocked.
	ErrGoalBlocked = errors.New("astar: goal cell is blocked")
	// ErrNoPath is returned when no passable route exists between the start
	// and goal cells.
	ErrNoPath = errors.New("astar: no path exists between the cells")
)

// Cell identifies a grid cell by its coordinates.
type Cell struct {
	X, Y int
}

// CellUpdate is a new state for a single cell, used with Grid.Update.
type CellUpdate struct {
	X, Y int
	// Cost is the terrain entry cost of the cell.  It is ignored when
	// Blocked is true.
	Cost float64
	// Blocked marks the cell impassable when true.
	Blocked bool
}

// Fixed neighbour iteration orders.  Keeping these constant is part of what
// makes equal-cost paths deterministic.
var (
	orthOffsets = [4][2]int{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}
	diagOffsets = [4][2]int{{-1, -1}, {1, -1}, {-1, 1}, {1, 1}}
)

// Grid is a fixed-size, dynamically updatable navigation grid.
//
// The grid is allocated once; SetCost, SetBlocked and Update change individual
// cells in place without reallocating the map, so paths can be re-queried after
// small changes cheaply.  Search scratch space is retained between queries and
// reset lazily with generation stamps, so a query never clears or rebuilds the
// whole grid.
//
// Movement cost: moving orthogonally into a cell costs that cell's terrain
// (entry) cost; moving diagonally costs math.Sqrt2 times the destination cell's
// terrain cost.  In Neighbor8 mode a diagonal move is only allowed when both
// orthogonal side cells are passable, so paths never cut through a wall
// corner.
//
// The heuristic is admissible and consistent (Manhattan scaled by the minimum
// terrain cost for Neighbor4, octile distance for Neighbor8), so the first
// expanded goal holds the optimal total cost.  Equal-cost ties are broken
// deterministically, which means the same grid and query always produce the
// same path.
//
// Grid is not safe for concurrent use.
type Grid struct {
	width, height int
	mode          NeighborMode

	// Terrain state, row-major (id = y*width + x).  costs keeps the last
	// terrain cost even for blocked cells, so unblocking needs no extra data.
	costs   []float64
	blocked []bool

	// costCounts counts passable cells per terrain cost so minCost (used to
	// scale the heuristic) stays current without scanning the whole grid on
	// every cell update.
	costCounts map[float64]int
	minCost    float64

	// Reusable A* scratch space.  An entry is only valid for the current query
	// when its stamp equals searchID; searchID is bumped per FindPath call,
	// which lazily invalidates every array in O(1).
	searchID int
	seen     []int
	closed   []int
	g        []float64
	parent   []int
	open     gridHeap
	scratch  []Cell
}

// NewGrid constructs a width*height grid in the given neighbour mode.  Every
// cell starts passable with a terrain cost of 1.
func NewGrid(width, height int, mode NeighborMode) (*Grid, error) {
	if width <= 0 || height <= 0 {
		return nil, ErrInvalidDimensions
	}
	if mode != Neighbor4 && mode != Neighbor8 {
		return nil, ErrInvalidMode
	}
	n := width * height
	g := &Grid{
		width:      width,
		height:     height,
		mode:       mode,
		costs:      make([]float64, n),
		blocked:    make([]bool, n),
		costCounts: map[float64]int{1.0: n},
		minCost:    1,
		seen:       make([]int, n),
		closed:     make([]int, n),
		g:          make([]float64, n),
		parent:     make([]int, n),
	}
	for i := range g.costs {
		g.costs[i] = 1
	}
	return g, nil
}

// NewGridFromCosts constructs a grid from a row-major terrain cost matrix with
// width*height entries.  A cell holding the BlockedCost sentinel starts
// blocked; every other entry must be finite and non-negative.
func NewGridFromCosts(width, height int, mode NeighborMode, costs []float64) (*Grid, error) {
	g, err := NewGrid(width, height, mode)
	if err != nil {
		return nil, err
	}
	if len(costs) != width*height {
		return nil, ErrInvalidCosts
	}
	// Validate everything before touching the grid so a bad matrix leaves the
	// freshly constructed state untouched.
	for _, c := range costs {
		if c == BlockedCost {
			continue
		}
		if !validCost(c) {
			return nil, fmt.Errorf("%w: got %v", ErrInvalidCost, c)
		}
	}
	g.costCounts = map[float64]int{}
	g.minCost = 0
	for i, c := range costs {
		if c == BlockedCost {
			g.blocked[i] = true
			continue
		}
		g.costs[i] = c
		g.costCounts[c]++
		if len(g.costCounts) == 1 || c < g.minCost {
			g.minCost = c
		}
	}
	return g, nil
}

// Width returns the grid width.
func (g *Grid) Width() int { return g.width }

// Height returns the grid height.
func (g *Grid) Height() int { return g.height }

// Mode returns the neighbour mode the grid was constructed with.
func (g *Grid) Mode() NeighborMode { return g.mode }

func (g *Grid) id(x, y int) int { return y*g.width + x }

func (g *Grid) inBounds(x, y int) bool {
	return x >= 0 && y >= 0 && x < g.width && y < g.height
}

// Blocked reports whether the cell at (x, y) is blocked.  Out-of-bounds
// coordinates are treated as blocked.
func (g *Grid) Blocked(x, y int) bool {
	if !g.inBounds(x, y) {
		return true
	}
	return g.blocked[g.id(x, y)]
}

// Cost returns the terrain cost of the cell at (x, y) and whether it is
// passable.  Blocked or out-of-bounds cells report (0, false).
func (g *Grid) Cost(x, y int) (cost float64, passable bool) {
	if !g.inBounds(x, y) {
		return 0, false
	}
	id := g.id(x, y)
	if g.blocked[id] {
		return 0, false
	}
	return g.costs[id], true
}

// SetCost sets the terrain entry cost of the cell at (x, y) and marks it
// passable.  It returns ErrOutOfBounds for out-of-bounds coordinates and
// ErrInvalidCost for negative, NaN or infinite costs; in either error case the
// grid is left unchanged.
func (g *Grid) SetCost(x, y int, cost float64) error {
	if g.width == 0 {
		return ErrInvalidDimensions
	}
	if !g.inBounds(x, y) {
		return fmt.Errorf("%w: (%d,%d)", ErrOutOfBounds, x, y)
	}
	if !validCost(cost) {
		return fmt.Errorf("%w: got %v", ErrInvalidCost, cost)
	}
	g.applyCost(g.id(x, y), cost)
	return nil
}

// SetBlocked marks the cell at (x, y) impassable.  It returns
// ErrOutOfBounds for out-of-bounds coordinates.
func (g *Grid) SetBlocked(x, y int) error {
	if g.width == 0 {
		return ErrInvalidDimensions
	}
	if !g.inBounds(x, y) {
		return fmt.Errorf("%w: (%d,%d)", ErrOutOfBounds, x, y)
	}
	g.applyBlocked(g.id(x, y))
	return nil
}

// Update applies a batch of cell changes atomically: every update is validated
// before any cell is touched, so an out-of-bounds coordinate or invalid cost
// rejects the whole batch and leaves the grid unchanged.
func (g *Grid) Update(cells ...CellUpdate) error {
	if g.width == 0 {
		return ErrInvalidDimensions
	}
	for _, c := range cells {
		if !g.inBounds(c.X, c.Y) {
			return fmt.Errorf("%w: (%d,%d)", ErrOutOfBounds, c.X, c.Y)
		}
		if !c.Blocked && !validCost(c.Cost) {
			return fmt.Errorf("%w: got %v at (%d,%d)", ErrInvalidCost, c.Cost, c.X, c.Y)
		}
	}
	for _, c := range cells {
		id := g.id(c.X, c.Y)
		if c.Blocked {
			g.applyBlocked(id)
		} else {
			g.applyCost(id, c.Cost)
		}
	}
	return nil
}

// FindPath searches for an optimal path from the start cell to the goal cell
// and returns it in start-to-goal order together with its total movement cost.
//
// It returns ErrOutOfBounds when either coordinate is outside the grid,
// ErrStartBlocked or ErrGoalBlocked when an endpoint is blocked, and ErrNoPath
// when no passable route exists.  A query from a cell to itself returns a
// single-cell path with zero cost.
func (g *Grid) FindPath(from, to Cell) (path []Cell, totalCost float64, err error) {
	if g.width == 0 || g.height == 0 {
		return nil, 0, ErrInvalidDimensions
	}
	if !g.inBounds(from.X, from.Y) {
		return nil, 0, fmt.Errorf("%w: start (%d,%d)", ErrOutOfBounds, from.X, from.Y)
	}
	if !g.inBounds(to.X, to.Y) {
		return nil, 0, fmt.Errorf("%w: goal (%d,%d)", ErrOutOfBounds, to.X, to.Y)
	}
	fromID := g.id(from.X, from.Y)
	toID := g.id(to.X, to.Y)
	if g.blocked[fromID] {
		return nil, 0, ErrStartBlocked
	}
	if g.blocked[toID] {
		return nil, 0, ErrGoalBlocked
	}
	if fromID == toID {
		return []Cell{{from.X, from.Y}}, 0, nil
	}

	// Bump the generation stamp: all stale seen/closed entries are now ignored
	// without clearing the arrays.
	g.searchID++
	sid := g.searchID
	g.open = g.open[:0]

	g.seen[fromID] = sid
	g.g[fromID] = 0
	g.parent[fromID] = -1
	g.open.push(gridHeapItem{
		id: fromID,
		f:  g.heuristic(from.X, from.Y, to.X, to.Y),
		gv: 0,
	})

	for len(g.open) > 0 {
		current := g.open.pop()
		id := current.id
		// Skip stale duplicate entries and cells already settled optimally.
		if g.closed[id] == sid || current.gv != g.g[id] {
			continue
		}
		g.closed[id] = sid
		if id == toID {
			return g.buildPath(toID), g.g[toID], nil
		}

		x, y := id%g.width, id/g.width
		for _, off := range orthOffsets {
			nx, ny := x+off[0], y+off[1]
			if !g.inBounds(nx, ny) {
				continue
			}
			nid := g.id(nx, ny)
			if g.blocked[nid] {
				continue
			}
			g.relax(id, nid, g.costs[nid], to.X, to.Y, sid)
		}
		if g.mode == Neighbor8 {
			for _, off := range diagOffsets {
				nx, ny := x+off[0], y+off[1]
				if !g.inBounds(nx, ny) {
					continue
				}
				nid := g.id(nx, ny)
				if g.blocked[nid] {
					continue
				}
				// No corner cutting: the two orthogonal cells bracketing the
				// diagonal must both be passable.
				if g.blocked[g.id(nx, y)] || g.blocked[g.id(x, ny)] {
					continue
				}
				g.relax(id, nid, math.Sqrt2*g.costs[nid], to.X, to.Y, sid)
			}
		}
	}
	return nil, 0, ErrNoPath
}

// relax considers moving from id to nid with the given edge cost and inserts or
// re-prioritises nid in the open heap when the route is an improvement.
func (g *Grid) relax(id, nid int, edgeCost float64, tx, ty, sid int) {
	if g.closed[nid] == sid {
		return
	}
	newG := g.g[id] + edgeCost
	if g.seen[nid] == sid && newG >= g.g[nid] {
		return
	}
	g.seen[nid] = sid
	g.g[nid] = newG
	g.parent[nid] = id
	g.open.push(gridHeapItem{
		id: nid,
		f:  newG + g.heuristic(nid%g.width, nid/g.width, tx, ty),
		gv: newG,
	})
}

// heuristic returns the consistent A* heuristic between two cells.
func (g *Grid) heuristic(x, y, tx, ty int) float64 {
	dx := x - tx
	if dx < 0 {
		dx = -dx
	}
	dy := y - ty
	if dy < 0 {
		dy = -dy
	}
	m := g.minCost
	if g.mode == Neighbor8 {
		dmax, dmin := dx, dy
		if dy > dmax {
			dmax, dmin = dy, dx
		}
		// Octile distance scaled by the smallest possible edge cost.
		return m * (float64(dmax) + (math.Sqrt2-1)*float64(dmin))
	}
	return m * float64(dx+dy)
}

// buildPath reconstructs the start-to-goal path by walking parent pointers
// from the goal, reusing the scratch slice and returning an exact-sized copy.
func (g *Grid) buildPath(goalID int) []Cell {
	g.scratch = g.scratch[:0]
	for id := goalID; id != -1; id = g.parent[id] {
		g.scratch = append(g.scratch, Cell{X: id % g.width, Y: id / g.width})
	}
	out := make([]Cell, len(g.scratch))
	for i, c := range g.scratch {
		out[len(out)-1-i] = c
	}
	return out
}

func validCost(c float64) bool {
	return !math.IsNaN(c) && !math.IsInf(c, 0) && c >= 0
}

func (g *Grid) applyCost(id int, cost float64) {
	if g.blocked[id] {
		g.blocked[id] = false
	} else {
		old := g.costs[id]
		if old == cost {
			return
		}
		g.removeCostCount(old)
	}
	g.costs[id] = cost
	g.costCounts[cost]++
	// len == 1 covers the case where every cell had been blocked.
	if len(g.costCounts) == 1 || cost < g.minCost {
		g.minCost = cost
	}
}

func (g *Grid) applyBlocked(id int) {
	if g.blocked[id] {
		return
	}
	g.removeCostCount(g.costs[id])
	g.blocked[id] = true
}

func (g *Grid) removeCostCount(cost float64) {
	if g.costCounts[cost] <= 1 {
		delete(g.costCounts, cost)
		if cost == g.minCost {
			g.recomputeMinCost()
		}
		return
	}
	g.costCounts[cost]--
}

// recomputeMinCost rescans the distinct terrain costs only when the previous
// minimum disappears, which a small cell update triggers rarely.
func (g *Grid) recomputeMinCost() {
	if len(g.costCounts) == 0 {
		// Every cell is blocked; the heuristic value is irrelevant until a
		// passable cell exists again.
		g.minCost = 0
		return
	}
	min := math.Inf(1)
	for cost := range g.costCounts {
		if cost < min {
			min = cost
		}
	}
	g.minCost = min
}

// gridHeapItem is an open-set entry: cell id, A* rank f and path cost g.
type gridHeapItem struct {
	id    int
	f, gv float64
}

// gridHeap is a binary min-heap reused across queries.  Ties on f prefer the
// node with the larger g (closer to the goal), then the smaller cell id, so
// equal-cost searches are fully deterministic.
type gridHeap []gridHeapItem

func (h gridHeap) less(i, j int) bool {
	a, b := h[i], h[j]
	if a.f != b.f {
		return a.f < b.f
	}
	if a.gv != b.gv {
		return a.gv > b.gv
	}
	return a.id < b.id
}

func (h gridHeap) swap(i, j int) { h[i], h[j] = h[j], h[i] }

func (h *gridHeap) push(item gridHeapItem) {
	*h = append(*h, item)
	h.siftUp(len(*h) - 1)
}

func (h *gridHeap) pop() gridHeapItem {
	old := *h
	n := len(old)
	top := old[0]
	last := old[n-1]
	*h = old[:n-1]
	if n > 1 {
		(*h)[0] = last
		h.siftDown(0)
	}
	return top
}

func (h *gridHeap) siftUp(i int) {
	for i > 0 {
		parent := (i - 1) / 2
		if !h.less(i, parent) {
			return
		}
		h.swap(i, parent)
		i = parent
	}
}

func (h *gridHeap) siftDown(i int) {
	n := len(*h)
	for {
		left := 2*i + 1
		right := left + 1
		smallest := i
		if left < n && h.less(left, smallest) {
			smallest = left
		}
		if right < n && h.less(right, smallest) {
			smallest = right
		}
		if smallest == i {
			return
		}
		h.swap(i, smallest)
		i = smallest
	}
}
