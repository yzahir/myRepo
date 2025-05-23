import sys
from maze_interface.srv import GetRosMaze
import rclpy
from rclpy.node import Node
from group2_py.maze_utils import *
import heapq
import numpy as np
from geometry_msgs.msg import Twist
import time
from group2_py.maze_executor import MazeExecutor


FORWARD_DURATION = 1.3 # Time = Distance / Speed = 0.254 m / 0.2 ≈ 1.27 s
#TURN_DURATION = 1.6 # Time = Angle / Angular speed = (π/2) / 1.0 ≈ 1.57 s

class MazeClient(Node):

    def __init__(self):
        super().__init__('maze_client')
        self.cli = self.create_client(GetRosMaze, 'get_ros_maze')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = GetRosMaze.Request()

    def send_request(self, maze_nr):
        self.req.maze_nr = maze_nr
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def flatten_to_grid(self, maze_data, n, m):
        return [maze_data[i * m:(i + 1) * m] for i in range(n)]

    def validate_received_maze(self, maze_msg):
        n, m = maze_msg.n, maze_msg.m
        flat = maze_msg.l
        L = self.flatten_to_grid(flat, n, m)
        #L = [flat[i * m:(i + 1) * m] for i in range(n)]
        # Mark unreachable cells and trim the maze
        L = isolate_unreachable_cells_and_trim(L)
        self.L = L
        # Validate maze - now it has to be valid
        validate_maze(L)

        # Output ASCII version
        print("ASCII Representation of Maze:")
        draw_ascii_maze(L)

        # Plot the maze
        print("Plotting Maze...")
        plot_maze(L) 
    
    def segment_to_indices(self, I, n):
        
        i = (I - 1) % n 
        j = (I - 1) // n 

        self.get_logger().info(f"Segment I is position: ({i}, {j})")
        return i, j

    def indices_to_segment(self, i, j, n):
        return (j-i)*n + 1

    def a_star(self, maze, start, goal):
        """
        maze: 2D list of integers (4-bit wall-encoded cells)
        start: (i, j)
        goal: (i, j)
        Returns a list of (i, j) positions from start to goal, or None if no path.
        """
        n, m = len(maze), len(maze[0])

        def heuristic(a, b):
            # Manhattan distance
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def neighbors(i, j):
            walls = maze[i][j]
            for di, dj, direction_bit, opposite_bit in [(-1, 0, 2, 1),  # top
                                                        (1, 0, 1, 2),   # bottom
                                                        (0, 1, 4, 8),   # right
                                                        (0, -1, 8, 4)]: # left
                ni, nj = i + di, j + dj
                if 0 <= ni < n and 0 <= nj < m:
                    if (walls & direction_bit) == 0 and (maze[ni][nj] & opposite_bit) == 0:
                        yield (ni, nj)

        open_set = []
        heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, cost, current = heapq.heappop(open_set)
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            for neighbor in neighbors(*current):
                tentative_g = cost + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                    came_from[neighbor] = current

        return None  # No path found

    def path_to_commands(self, path, initial_orientation):
        commands = []
        orientation = initial_orientation

        direction_map = {
            (0, 1): 0b0100,  # Right  → index 0
            (1, 0): 0b0001,  # Down   → index 1
            (0, -1): 0b1000, # Left   → index 2
            (-1, 0): 0b0010  # Up     → index 3
        }

        for (cur, nxt) in zip(path, path[1:]): # same as for i in range(len(path) - 1): cur = path[i]; nxt = path[i + 1]

            di, dj = nxt[0] - cur[0], nxt[1] - cur[1]
            desired = direction_map[(di, dj)]

            if desired != orientation:
                # Determine turning direction (left/right)
                if (orientation == 0b0001 and desired == 0b0100) or \
                   (orientation == 0b0100 and desired == 0b0010) or \
                   (orientation == 0b0010 and desired == 0b1000) or \
                   (orientation == 0b1000 and desired == 0b0001):
                    commands.append('turn_left')
                else:
                    commands.append('turn_right')
                orientation = desired

            commands.append('forward')

        return commands

def main(args=None):
    rclpy.init(args=args)
    client = MazeClient()
    response = client.send_request(2)  # Request for the first maze
    client.get_logger().info(f"Maze: {response.maze}")
    i_goal, j_goal = client.segment_to_indices(response.maze.end_idx, response.maze.n)
    client.get_logger().info(f"Goal position: ({i_goal}, {j_goal})")

    i_start, j_start = client.segment_to_indices(response.maze.start_idx, response.maze.n)
    client.get_logger().info(f"Start position: ({i_start}, {j_start})")
    client.get_logger().info(f"Maze dimension: {response.maze.n}x{response.maze.m}")
    client.get_logger().info(f"Maze Start Index: {response.maze.start_idx}")
    client.get_logger().info(f"Maze End Index: {response.maze.end_idx}")
    client.get_logger().info(f"Flattened Layout: {list(response.maze.l)}")

    client.validate_received_maze(response.maze)
    path = client.a_star(client.L, (i_start, j_start), (i_goal, j_goal))
    client.get_logger().info(f"Path: {path}")
    
    commands = client.path_to_commands(path, response.maze.start_orientation)

    # Initialize MazeExecutor with IMU-based turning
    executor = MazeExecutor(use_filtered_rpy=True)  # or False for raw IMU
    expected_yaw = executor.orientation_to_yaw(response.maze.start_orientation)
    executor.align_orientation(expected_yaw)  # align_orientation
    executor.execute_commands(commands)
    client.destroy_node()
    executor.destroy_node()    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
