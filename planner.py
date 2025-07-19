import sys
import heapq

def read_world_file(input_file):
    grid = []
    robot_start_loc = None
    dirty_cells = set()

    with open(input_file, 'r') as f:
        # Read first line
        num_cols = int(f.readline().strip())
        # Read second line
        num_rows = int(f.readline().strip())
        # Read rest of the lines aka the grid
        for r in range(num_rows):
            line = f.readline().strip()
            row_cells = []
            for c, char in enumerate(line):
                row_cells.append(char)
                if char == '@':
                    robot_start_loc = (r, c)
                elif char == '*':
                    dirty_cells.add((r, c))
            grid.append(row_cells)

    if robot_start_loc is None:
        raise ValueError("Robot starting location doesn't exist in given file")

    return num_rows, num_cols, grid, robot_start_loc, dirty_cells

class Node:
    def __init__(self, robot_location, remaining_dirty_cells, parent=None, action=None, cost_so_far=0):
        self.location = robot_location
        self.remaining_dirty_cells = frozenset(remaining_dirty_cells) # frozenset for hashability
        self.state = (self.location, self.remaining_dirty_cells) # State is (robot_pos, frozenset_of_dirty_cells) 
        
        self.parent = parent # For path reconstruction
        self.action = action # Action taken to reach this node 
        self.cost = cost_so_far

    def find_path(self):
        path = []
        current_node = self
        while current_node and current_node.action is not None:
            path.append(current_node.action)
            current_node = current_node.parent
        path.reverse()
        return path

def get_actions(current_node, num_rows, num_cols, grid_data):
    current_r, current_c = current_node.location
    children = []

    # Define possible movements (nr, nc): (action_name, cost)
    moves = {
        (-1, 0): 'N', #North
        (1, 0): 'S', #South
        (0, -1): 'W', #West
        (0, 1): 'E' #East
    }

    # Seeing if current cell is dirty
    if current_node.location in current_node.remaining_dirty_cells:
        new_dirty_cells = current_node.remaining_dirty_cells - {current_node.location}
        clean_cost = current_node.cost + 1
        child_node = Node(current_node.location, new_dirty_cells, current_node, 'V', clean_cost)
        children.append(child_node)

    # Then, consider movement actions 
    for (nr_offset, nc_offset), action_name in moves.items():
        new_r, new_c = current_r + nr_offset, current_c + nc_offset

        # See if neighbor is valid (within grid boundaries and not blocked(#))
        if not (0 <= new_r < num_rows and 0 <= new_c < num_cols):
            continue
        if grid_data[new_r][new_c] == '#':
            continue 

        move_cost = current_node.cost + 1
        child_node = Node((new_r, new_c), current_node.remaining_dirty_cells, current_node, action_name, move_cost)
        children.append(child_node)

    return children

def dfs(num_rows, num_cols, grid_initial_data, robot_start_loc, dirty_cells_initial):
    start_node = Node(robot_start_loc, dirty_cells_initial)

    visited_states = set() # To keep track of states we've already expanded
    stack = [start_node]

    nodes_generated = 0
    nodes_expanded = 0

    nodes_generated += 1 

    while stack:
        current_node = stack.pop()

        # If this state has already been visited, prevents infinite loops in graphs
        if current_node.state in visited_states:
            continue

        # Node being expanded
        visited_states.add(current_node.state)
        nodes_expanded += 1

        # Check if all dirty cells are cleaned
        if not current_node.remaining_dirty_cells:
            return current_node.find_path(), nodes_generated, nodes_expanded
        
        # Generate children from the current node
        children = get_actions(current_node, num_rows, num_cols, grid_initial_data)
        
        for child in reversed(children): 
            # Only add to stack (generate) if this state has not been visited yet
            if child.state not in visited_states:
                stack.append(child)
                nodes_generated += 1 # Count this child as generated

    return None, nodes_generated, nodes_expanded

def ucs(num_rows, num_cols, grid_initial_data, robot_start_loc, dirty_cells_initial):
    start_node = Node(robot_start_loc, dirty_cells_initial)

    p_queue = []
    distance_map = {}

    nodes_generated = 0
    nodes_expanded = 0
    counter = 0  # manual tiebreaker

    distance_map[start_node.state] = 0
    heapq.heappush(p_queue, (0, counter, start_node))
    counter += 1
    nodes_generated += 1

    while p_queue:
        cost, _, current_node = heapq.heappop(p_queue)

        state_key = current_node.state

        if cost > distance_map.get(state_key, float('inf')):
            continue 

        nodes_expanded += 1

        if not current_node.remaining_dirty_cells:
            return current_node.find_path(), nodes_generated, nodes_expanded

        children = get_actions(current_node, num_rows, num_cols, grid_initial_data)

        for child in children:
            if child.cost <= distance_map.get(child.state, float('inf')):
                distance_map[child.state] = child.cost 
                heapq.heappush(p_queue, (child.cost, counter, child))
                counter += 1
                nodes_generated += 1 
    
    return None, nodes_generated, nodes_expanded

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Wrong number of arguments, run the program: \npython3 planner.py [uniform-cost or depth-first] [world-file]")
        sys.exit(1)

    algorithm = sys.argv[1]
    world_file = sys.argv[2]

    try:
        num_rows, num_cols, grid, robot_start_loc, dirty_cells_initial = read_world_file(world_file)

        if algorithm == 'uniform-cost':
            path, nodes_generated, nodes_expanded = ucs(num_rows, num_cols, grid, robot_start_loc, dirty_cells_initial)
            if path is None:
                print("No path found")
            else:
                for d in path:
                    print(d)
                print(nodes_generated, "nodes generated")
                print(nodes_expanded, "nodes expanded")
        elif algorithm == 'depth-first':
            path, nodes_generated, nodes_expanded = dfs(num_rows, num_cols, grid, robot_start_loc, dirty_cells_initial)
            if path is None:
                print("No path found")
            else:
                for d in path:
                    print(d)
                print(nodes_generated, "nodes generated")
                print(nodes_expanded, "nodes expanded")
        else:
            print("Invalid algorithm given, valid algorithms are 'uniform-cost' and 'depth-first'")

    except Exception as e:
        print("An unexpected error occured:", e)
        sys.exit(1)
