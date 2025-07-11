import sys

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
        raise ValueError("Robot starting location doesn't exist in given file.")

    return num_rows, num_cols, grid, robot_start_loc, dirty_cells

def robot_actions(robot_curr_loc, dirty_cells_curr, num_rows, num_cols, grid):
    actions = [] # List of (action, robot_new_loc, dirty_cells_new, cost)
    r_row, r_col = robot_curr_loc

    moves = {
        'N': (-1, 0), # North: row - 1, col
        'E': (0, 1), # East: row, col + 1
        'S': (1, 0), # South: row + 1, col
        'W': (0, -1) # West: row, col - 1
    }

    for action, (dr, dc) in moves.items():
        new_r_row, new_r_col = r_row + dr, r_col + dc
        
        if 0 <= new_r_row < num_rows and 0 <= new_r_col < num_cols:
            if grid[new_r_row][new_r_col] != '#':
                robot_new_loc = (new_r_row, new_r_col)
                actions.append((action, robot_new_loc, dirty_cells_curr, 1))

    if robot_curr_loc in dirty_cells_curr:
        dirty_cells_new = set(dirty_cells_curr)
        dirty_cells_new.remove(robot_curr_loc)
        actions.append(('V', robot_curr_loc, set(dirty_cells_new), 1))

    return actions 

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Wrong number of arguments, run the program: \npython3 planner.py [uniform-cost or depth-first] [world-file]")
        sys.exit(1)

    algorithm = sys.argv[1]
    world_file = sys.argv[2]

    try:
        num_rows, num_cols, grid, robot_start_loc, dirty_cells_initial = read_world_file(world_file)
        print("Rows:", num_rows)
        print("Columns:", num_cols)
        print("Robot Start Location:", robot_start_loc)
        print("Given Dirty Cells:", dirty_cells_initial)
        print("Grid:")
        for r in grid:
            print(r)

        initial_actions = robot_actions(robot_start_loc, dirty_cells_initial, num_rows, num_cols)

        

    except Exception as e:
        print("An unexpected error occured:", e)
        sys.exit(1)




