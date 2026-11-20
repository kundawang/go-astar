go-astar
========

**A\* pathfinding implementation for Go**

[![Build Status](https://travis-ci.org/beefsack/go-astar.svg?branch=master)](https://travis-ci.org/beefsack/go-astar)

The [A\* pathfinding algorithm](http://en.wikipedia.org/wiki/A*_search_algorithm) is a pathfinding algorithm noted for its performance and accuracy and is commonly used in game development.  It can be used to find short paths for any weighted graph.

A fantastic overview of A\* can be found at [Amit Patel's Stanford website](http://theory.stanford.edu/~amitp/GameProgramming/AStarComparison.html).

Examples
--------

The following crude examples were taken directly from the automated tests.  Please see `path_test.go` for more examples.

### Key

*   `.` - Plain (movement cost 1)
*   `~` - River (movement cost 2)
*   `M` - Mountain (movement cost 3)
*   `X` - Blocker, unable to move through
*   `F` - From / start position
*   `T` - To / goal position
*   `●` - Calculated path

### Straight line

```
.....~......      .....~......
.....MM.....      .....MM.....
.F........T.  ->  .●●●●●●●●●●.
....MMM.....      ....MMM.....
............      ............
```

### Around a mountain

```
.....~......      .....~......
.....MM.....      .....MM.....
.F..MMMM..T.  ->  .●●●MMMM●●●.
....MMM.....      ...●MMM●●...
............      ...●●●●●....
```

### Blocked path

```
............      
.........XXX
.F.......XTX  ->  No path
.........XXX
............
```

### Maze

```
FX.X........      ●X.X●●●●●●..
.X...XXXX.X.      ●X●●●XXXX●X.
.X.X.X....X.  ->  ●X●X.X●●●●X.
...X.X.XXXXX      ●●●X.X●XXXXX
.XX..X.....T      .XX..X●●●●●●
```

### Mountain climber

```
..F..M......      ..●●●●●●●●●.
.....MM.....      .....MM...●.
....MMMM..T.  ->  ....MMMM..●.
....MMM.....      ....MMM.....
............      ............
```

### River swimmer

```
.....~......      .....~......
.....~......      ....●●●.....
.F...X...T..  ->  .●●●●X●●●●..
.....M......      .....M......
.....M......      .....M......
```

Usage
-----

### Import the package

```go
import "github.com/beefsack/go-astar"
```

### Implement Pather interface

An example implementation is done for the tests in `path_test.go` for the Tile type.

The `PathNeighbors` method should return a slice of the direct neighbors.

The `PathNeighborCost` method should calculate an exact movement cost for direct neighbors.

The `PathEstimatedCost` is a heuristic method for estimating the distance between arbitrary tiles.  The examples in the test files use [Manhattan distance](http://en.wikipedia.org/wiki/Taxicab_geometry) to estimate orthogonal distance between tiles.

```go
type Tile struct{}

func (t *Tile) PathNeighbors() []astar.Pather {
	return []astar.Pather{
		t.Up(),
		t.Right(),
		t.Down(),
		t.Left(),
	}
}

func (t *Tile) PathNeighborCost(to astar.Pather) float64 {
	return to.MovementCost
}

func (t *Tile) PathEstimatedCost(to astar.Pather) float64 {
	return t.ManhattanDistance(to)
}
```

### Call Path function

```go
// t1 and t2 are *Tile objects from inside the world.
path, distance, found := astar.Path(t1, t2)
if !found {
	log.Println("Could not find path")
}
// path is a slice of Pather objects which you can cast back to *Tile.
```

Updatable grid navigation (`Grid`)
----------------------------------

The generic `Pather` API builds a fresh node map for every query.  For games
and simulations where a small number of cells change between queries — a door
slamming shut, a patch of mud appearing — `Grid` provides a fixed-size
navigation grid that is updated in place and re-queried cheaply.

Features:

*   Four- or eight-neighbour search (`Neighbor4` / `Neighbor8`).
*   Per-cell terrain (entry) costs and dynamic blocking.  Diagonal moves cost
    `math.Sqrt2` times the destination cell's terrain cost.
*   In eight-neighbour mode a diagonal move is only allowed when both
    orthogonal side cells are passable, so paths never cut through a wall
    corner.
*   O(1) cell updates (`SetCost`, `SetBlocked`, or a batched atomic
    `Update`).  The grid is never rebuilt; search state is reused across
    queries with generation stamps, leaving unrelated cells untouched.
*   Deterministic equal-cost tie breaking: the same grid and query always
    produce the same path.
*   Explicit errors for blocked endpoints, unreachable goals, out-of-bounds
    coordinates, invalid (negative / NaN / infinite) costs and empty grids.

```go
// Build a 10x10 grid allowing diagonal movement.
g, err := astar.NewGrid(10, 10, astar.Neighbor8)
if err != nil {
	log.Fatal(err)
}

// Terrain costs: mud is slower, walls block movement entirely.
if err := g.SetCost(4, 4, 5); err != nil {
	log.Fatal(err)
}
if err := g.SetBlocked(5, 5); err != nil {
	log.Fatal(err)
}

path, totalCost, err := g.FindPath(astar.Cell{X: 0, Y: 0}, astar.Cell{X: 9, Y: 9})
if errors.Is(err, astar.ErrNoPath) {
	log.Println("No route exists")
} else if err != nil {
	log.Fatal(err)
}
// path is a []astar.Cell in start-to-goal order.

// A gate closes on the planned route: update a few cells and query again.
err = g.Update(
	astar.CellUpdate{X: 3, Y: 3, Blocked: true},
	astar.CellUpdate{X: 4, Y: 3, Cost: 2.5},
)
path, totalCost, err = g.FindPath(astar.Cell{X: 0, Y: 0}, astar.Cell{X: 9, Y: 9})
```

A grid can also be constructed from a row-major cost matrix, using
`astar.BlockedCost` for impassable cells:

```go
g, err := astar.NewGridFromCosts(width, height, astar.Neighbor4, costs)
```

`Grid` is not safe for concurrent use; guard it with a lock if multiple
goroutines query or update it.

Authors
-------

Michael Alexander <beefsack@gmail.com>
Robin Ranjit Chauhan <robin@pathwayi.com>
