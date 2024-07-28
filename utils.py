import numpy as np
from a_star import a_star
import matplotlib.pyplot as plt
from gridmap import OccupancyGridMap
import matplotlib.ticker as ticker
import gdspy

def set_obstacle(data_array, obstacles):
    for x, y in obstacles:
        data_array[y, x] = 1.0  # Set cell as occupied (1.0)
    return data_array

def a_star_loop(start_points, end_points, gmap, movement = '8N'):
    paths = {}
    success = 0
    total_costs = {}
    for i, start_m in enumerate(start_points):
        for j, end_m in enumerate(end_points):
            if i==j:
                # Create a copy of the grid map to modify for each routing

                path, path_idx, success, fin_cost = a_star(start_m, end_m, gmap, success, movement = movement)
            
                # Store the path and its cost
                paths[(start_m, end_m)] = path
                total_costs[(start_m, end_m)] = fin_cost
                
                if path:
                    # Mark the path as visited to avoid overlap with future paths
                    for pos in path_idx:
                        gmap.set_data_idx(pos, 1)
                else:
                    print(f"No path found between {start_m} and {end_m}")

    return paths, success, total_costs

def plot_routing(gmap, paths, colors):
    # Visualize the grid, obstacles, and the paths
    plt.figure(figsize=(10, 10))
    gmap.plot(origin='upper', alpha=0.5, min_val=0)

    plt.gca().set_facecolor('black')  # Set background color to light grey

    # Draw start and end points, paths
    for i, ((start_m, end_m), path) in enumerate(paths.items()):
        if path:
            color = colors[i % len(colors)]
            path_idx = [gmap.get_index_from_coordinates(x, y) for x, y in path]

            # Draw path
            plt.plot([p[0] for p in path], [p[1] for p in path], color=color, linewidth=4)

            # Draw start and end points
            plt.scatter([start_m[0]], [start_m[1]], color=color, edgecolors='black', s=200, label=f'Start {start_m}')
            plt.scatter([end_m[0]], [end_m[1]], color=color, edgecolors='black', s=200, label=f'End {end_m}')
        # else:
        #     print(f"No path found between start point {start_m} and end point {end_m}.")

    plt.title("A* Pathfinding")
    plt.xlabel("X coordinate")
    plt.ylabel("Y coordinate")
    plt.gca().invert_yaxis()
    # Set grid intervals
    plt.gca().xaxis.set_major_locator(ticker.MultipleLocator(1))
    plt.gca().yaxis.set_major_locator(ticker.MultipleLocator(1))

    plt.grid(True, which='both', color='gray', linestyle='--', linewidth=0.5)
    plt.show()


def create_gds_paths_with_ports_and_squares(paths, start_points, end_points, width=0.3, square_size=0.5):
    # Create a new GDSII library
    lib = gdspy.GdsLibrary()

    # Create a new cell in the library
    cell = lib.new_cell('METAL_ROUTING')

    for start, end, path in zip(start_points, end_points, paths.values()):
        # Create a path with the given width
        gdspy_path = gdspy.FlexPath([start], width)

        for point in path[1:]:
            gdspy_path.segment([point], width)

        # Add the path to the cell
        cell.add(gdspy_path)

        # Add ports as squares
        start_square = gdspy.Rectangle(
            (start[0] - square_size / 2, start[1] - square_size / 2),
            (start[0] + square_size / 2, start[1] + square_size / 2)
        )
        end_square = gdspy.Rectangle(
            (end[0] - square_size / 2, end[1] - square_size / 2),
            (end[0] + square_size / 2, end[1] + square_size / 2)
        )
        cell.add(start_square)
        cell.add(end_square)

    # Save the library in a file
    lib.write_gds('metal_routing_with_ports_and_squares.gds')

    # Optionally, view the layout using the internal viewer
    gdspy.LayoutViewer(lib)