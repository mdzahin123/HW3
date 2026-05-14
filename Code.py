import math
import heapq
from eastgate_helper import get_office_grid, get_terrain_costs, test_scenarios

class Node:
    """Represents a node in the search space."""
    def __init__(self, pos, g=0, h=0, parent=None):
        self.pos = pos
        self.g = g  # Cost from start
        self.h = h  # Heuristic estimate to goal
        self.f = g + h  # Total estimated cost
        self.parent = parent
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __eq__(self, other):
        return self.pos == other.pos
    
    def __hash__(self):
        return hash(self.pos)


def manhattan_heuristic(pos, goal):
    """Standard Manhattan distance heuristic."""
    return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])


def terrain_aware_heuristic(pos, goal, costs, grid):
    """
    Terrain-aware heuristic that accounts for slow zones.
    Adjusts Manhattan distance by average terrain cost to provide
    a more accurate estimate accounting for slow zones.
    """
    manhattan = abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
    
    # Calculate average cost in the grid (excluding obstacles)
    total_cost = 0
    count = 0
    for row in costs:
        for cell in row:
            total_cost += cell
            count += 1
    
    avg_cost = total_cost / count if count > 0 else 1
    
    # Adjust heuristic by average terrain cost
    return manhattan * avg_cost


def get_neighbors(pos, grid, costs):
    """
    Returns valid neighboring positions with their movement costs.
    Supports 8 directions: 4 orthogonal (cost 1) + 4 diagonal (cost √2).
    """
    row, col = pos
    rows, cols = len(grid), len(grid[0])
    neighbors = []
    
    # 8 directions: up, down, left, right, and 4 diagonals
    directions = [
        # Orthogonal (cost 1)
        ((-1, 0), 1),      # up
        ((1, 0), 1),       # down
        ((0, -1), 1),      # left
        ((0, 1), 1),       # right
        # Diagonal (cost √2 ≈ 1.414)
        ((-1, -1), math.sqrt(2)),  # up-left
        ((-1, 1), math.sqrt(2)),   # up-right
        ((1, -1), math.sqrt(2)),   # down-left
        ((1, 1), math.sqrt(2))     # down-right
    ]
    
    for (dr, dc), move_cost in directions:
        new_row, new_col = row + dr, col + dc
        
        # Check bounds
        if 0 <= new_row < rows and 0 <= new_col < cols:
            # Check if not obstacle
            if grid[new_row][new_col] == 0:
                # Diagonal moves must check for diagonal blocking
                # (can't go through corner if both adjacent cells are blocked)
                if abs(dr) == 1 and abs(dc) == 1:
                    # Check adjacent cells for diagonal movement
                    if (grid[row + dr][col] == 1 or grid[row][col + dc] == 1):
                        continue  # Block diagonal if corner is blocked
                
                # Get terrain cost
                terrain_cost = costs[new_row][new_col]
                total_cost = move_cost * terrain_cost
                neighbors.append(((new_row, new_col), total_cost))
    
    return neighbors


def reconstruct_path(node):
    """Reconstructs the path from start to goal by backtracking through parents."""
    path = []
    current = node
    while current is not None:
        path.append(current.pos)
        current = current.parent
    return path[::-1]


def a_star(grid, start, goal, costs):
    """
    A* search implementation for finding shortest path in grid.
    
    Args:
        grid: 2D list where 0=open, 1=obstacle
        start: (row, col) tuple for start position
        goal: (row, col) tuple for goal position
        costs: 2D list of terrain costs
    
    Returns:
        List of (row, col) tuples representing shortest path, or None if blocked
    """
    # Check valid start and goal
    if (grid[start[0]][start[1]] == 1 or 
        grid[goal[0]][goal[1]] == 1):
        return None
    
    # Initialize
    open_set = []
    closed_set = set()
    
    start_h = terrain_aware_heuristic(start, goal, costs, grid)
    start_node = Node(start, g=0, h=start_h)
    heapq.heappush(open_set, start_node)
    
    node_dict = {start: start_node}  # Track best node for each position
    
    while open_set:
        current = heapq.heappop(open_set)
        
        # Goal reached
        if current.pos == goal:
            return reconstruct_path(current)
        
        if current.pos in closed_set:
            continue
        
        closed_set.add(current.pos)
        
        # Explore neighbors
        neighbors = get_neighbors(current.pos, grid, costs)
        
        for neighbor_pos, move_cost in neighbors:
            if neighbor_pos in closed_set:
                continue
            
            # Calculate g cost
            new_g = current.g + move_cost
            
            # Check if we found a better path to this neighbor
            if neighbor_pos in node_dict:
                if new_g >= node_dict[neighbor_pos].g:
                    continue  # Not a better path
            
            # Calculate h cost using terrain-aware heuristic
            h = terrain_aware_heuristic(neighbor_pos, goal, costs, grid)
            neighbor_node = Node(neighbor_pos, g=new_g, h=h, parent=current)
            
            node_dict[neighbor_pos] = neighbor_node
            heapq.heappush(open_set, neighbor_node)
    
    return None  # No path found


def visualize_path(grid, costs, path, start, goal):
    """
    Visualizes the grid with the found path.
    
    Symbols:
        'S' = Start
        'G' = Goal
        '*' = Path
        '#' = Obstacle
        '~' = Slow zone (not on path)
        '.' = Open space
    """
    rows, cols = len(grid), len(grid[0])
    
    # Create visualization grid
    viz = []
    for r in range(rows):
        row = []
        for c in range(cols):
            if (r, c) == start:
                row.append('S')
            elif (r, c) == goal:
                row.append('G')
            elif path and (r, c) in path:
                row.append('*')
            elif grid[r][c] == 1:
                row.append('#')
            elif costs[r][c] > 1:
                row.append('~')
            else:
                row.append('.')
        viz.append(row)
    
    # Print grid
    print("\n" + "="*50)
    print(f"Grid Visualization (Start: {start}, Goal: {goal})")
    print("="*50)
    print("  ", end="")
    for c in range(cols):
        print(f"{c} ", end="")
    print()
    
    for r in range(rows):
        print(f"{r} ", end="")
        for cell in viz[r]:
            print(f"{cell} ", end="")
        print()
    
    print("\nLegend:")
    print("  S = Start | G = Goal | * = Path | # = Obstacle")
    print("  ~ = Slow Zone | . = Open Space")
    print("="*50)


def print_path_info(path, costs):
    """Prints detailed information about the path including total cost."""
    if not path:
        print("No path found!")
        return
    
    total_cost = 0
    print(f"\nPath found with {len(path)} steps:")
    print(f"Path: {' → '.join(str(p) for p in path)}")
    
    # Calculate costs
    for i in range(len(path) - 1):
        curr = path[i]
        next_pos = path[i + 1]
        
        # Calculate movement cost
        dr = abs(next_pos[0] - curr[0])
        dc = abs(next_pos[1] - curr[1])
        
        if dr == 1 and dc == 1:  # Diagonal
            move_cost = math.sqrt(2)
        else:  # Orthogonal
            move_cost = 1
        
        terrain_cost = costs[next_pos[0]][next_pos[1]]
        step_cost = move_cost * terrain_cost
        total_cost += step_cost
        
        print(f"  {curr} → {next_pos}: move_cost={move_cost:.3f} × terrain_cost={terrain_cost} = {step_cost:.3f}")
    
    print(f"Total Path Cost: {total_cost:.3f}")


# Main execution
if __name__ == "__main__":
    print("\n" + "="*60)
    print("A* SEARCH FOR DELIVERY ROBOT NAVIGATION")
    print("Penn State Harrisburg Eastgate Center")
    print("="*60)
    
    # Run the test scenarios
    test_scenarios(a_star)
    
    # Additional visualization examples
    print("\n" + "="*60)
    print("DETAILED VISUALIZATION WITH PATH COSTS")
    print("="*60)
    
    grid = get_office_grid()
    costs = get_terrain_costs(grid)
    
    # Test case 1: Standard case
    start1, goal1 = (0, 0), (4, 4)
    path1 = a_star(grid, start1, goal1, costs)
    visualize_path(grid, costs, path1, start1, goal1)
    print_path_info(path1, costs)
    
    # Test case 2: Different start
    start2, goal2 = (2, 0), (4, 4)
    path2 = a_star(grid, start2, goal2, costs)
    visualize_path(grid, costs, path2, start2, goal2)
    print_path_info(path2, costs)
    
    # Test case 3: With added obstacle (dynamic)
    print("\n" + "="*60)
    print("DYNAMIC OBSTACLE TEST (Added at [3,1])")
    print("="*60)
    grid_dynamic = [row[:] for row in grid]  # Deep copy
    grid_dynamic[3][1] = 1
    path3 = a_star(grid_dynamic, start2, goal2, costs)
    visualize_path(grid_dynamic, costs, path3, start2, goal2)
    print_path_info(path3, costs)