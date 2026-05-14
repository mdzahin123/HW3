# do not modify anything in this code

import math

def get_office_grid():
    """Returns the office grid: 0 = open, 1 = obstacle."""
    return [
        [0, 0, 0, 1, 0],
        [0, 1, 0, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 1, 1, 0],
        [1, 0, 0, 0, 0]
    ]

def get_terrain_costs(grid):
    """Returns a 2D list of terrain costs: 1 for normal, 2 for slow zones."""
    costs = [[1 for _ in row] for row in grid]
    # Add some slow zones for challenge
    costs[0][1] = 2
    costs[2][3] = 2
    costs[4][2] = 2
    return costs

def test_scenarios(a_star_func):
    """Tests A* with multiple scenarios."""
    grid = get_office_grid()
    costs = get_terrain_costs(grid)
    tests = [
        ((0, 0), (4, 4), grid, costs),  # Standard
        ((0, 0), (4, 4), grid, costs),  # Reuse
        ((2, 0), (4, 4), grid, costs)   # Different start
    ]
    for start, goal, g, c in tests:
        # Simulate dynamic: add obstacle for one test
        if start == (2, 0):
            g[3][1] = 1  # Add obstacle
        path = a_star_func(g, start, goal, c)
        print(f"Path from {start} to {goal}: {path}")