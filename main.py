import numpy as np
from gridmap import OccupancyGridMap
from a_star import a_star
from utils import *
import time

# Start measuring time
start_time = time.time()

# Define grid size
x_size = 13
y_size = 11
cell_size = 1.0
scalar = 2

# Create an empty grid with all cells free (0.0)
data_array = np.zeros((y_size * scalar, x_size * scalar))

# Define start and end points in meters
Net = [((0,0), (4,4)), ((4,4),(4,6)), ((4,6),(2,10)), ((12,0),(9,8)), ((2,2),(3,8)), ((10,2),(8,4)), ((8,4),(8,6)), ((8,6),(10,10))]
start_points = [points[0] for points in Net]
end_points = [points[1] for points in Net]
scaled_start_points = [(x * scalar, y * scalar) for x, y in start_points]
scaled_end_points = [(x * scalar, y * scalar) for x, y in end_points]


# Define obstacles
obstacles = []
obstacles.extend(scaled_start_points)
obstacles.extend(scaled_end_points)

data_array = set_obstacle(data_array, obstacles)

# Create an occupancy grid map
gmap = OccupancyGridMap(data_array, cell_size)

# Initialize variables
colors = ['red', 'blue', 'green', 'purple', 'orange', 'cyan', 'magenta', 'yellow']

# Find paths for each start and end point combination
paths, success, total_costs = a_star_loop(scaled_start_points, scaled_end_points, gmap, '4N')

# End measuring time
end_time = time.time()

cost_sum = sum(total_costs[key] for key in total_costs)
print("Sum of total HPWL:", cost_sum)
elapsed_time = (end_time - start_time)*1000
print("Elapsed time: {:.4f} ms".format(elapsed_time))

# Visualize path with gds format
create_gds_paths_with_ports_and_squares(paths, scaled_start_points, scaled_end_points, width=0.3)

# Visualize the grid, obstacles, and the paths
plot_routing(gmap, paths, colors)
