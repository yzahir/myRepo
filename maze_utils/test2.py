# generate_maze.py

from group2_py.maze_utils import *

def generate_and_validate_maze(n=5, m=3, filling_factor=15):
    print("Generating random maze...")
    L_random = generate_random_maze(n, m, filling_factor=filling_factor)
    print("Raw Maze:")
    print(L_random)

    try:
        validate_maze(L_random)
        print("Initial validation passed.")
    except ValueError as e:
        print(f"Initial validation failed: {e}")

    # Remove unreachable cells and trim dead ends
    L_cleaned = isolate_unreachable_cells_and_trim(L_random)

    # Final validation — should succeed now
    try:
        validate_maze(L_cleaned)
        print("Cleaned maze is valid.")
        for i in L_cleaned:
            #print(i)
            for j in i:
                print(j)
    except ValueError as e:
        print(f"Validation failed after cleaning: {e}")
        return

    # Show ASCII version
    print("ASCII Representation of Maze:")
    draw_ascii_maze(L_cleaned)

    # Plot maze
    print("Plotting Maze...")
    plot_maze(L_cleaned)


if __name__ == "__main__":
    generate_and_validate_maze()
