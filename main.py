import numpy as np
from gridmap import OccupancyGridMap
from a_star import a_star
from utils import *
import time
from itertools import chain

# Start measuring time
start_time = time.time()

# Define grid size
x_size = 200
y_size = 200
cell_size = 1.0
scalar = 1

# Create an empty grid with all cells free (0.0)
data_array = np.zeros((y_size * scalar, x_size * scalar))

# Define start and end points in meters
Net = [((161, 46), (95, 84), (135, 60), (161, 82), (133, 82), (105, 62)), # A[3]
       ((91, 62), (91, 46), (97, 62)), # A[2]
       ((93, 96), (129, 134), (145, 82), (113, 82), (113, 82), (137, 118), (103, 118), (139, 82)), # A[1]
       ((99, 134), (161, 120), (139, 118), (97, 118), (165, 134)), # A[0]
       ((165, 118), (159, 82), (167, 118), (111, 82), (143, 118)), # B[3]
       ((165, 116), (159, 94), (159, 80), (143, 116), (143, 94), (111, 94), (111, 82)), # B[2]
       ((103, 132), (103, 64), (133, 132), (87, 60), (131, 62), (135, 118), (105, 118), (135, 82)), # B[1]
       ((97, 98), (91, 82), (101, 118), (87, 82), (101, 62)), # B[0]
       ((149, 62), (151, 86)), # n1
       ((145, 62), (137, 58)), # n2
       ((77, 82), (85, 58)), # n4
       ((73, 82), (97, 86)), # n5
       ((37, 82), (105, 86)), # n7
       ((53, 82), (67, 84)), # n8
       ((147, 134), (159, 122)), # n9
       ((143, 134), (135, 130)), # n10
       ((59, 130), (47, 96)), # n11
       ((25, 98), (141, 134), (147, 122)), # n12
       ((9, 98), (11, 84)), # n13
       ((31, 24), (71, 82), (93, 58)), # n16
       ((69, 26), (139, 60)), # n18
       ((15, 60), (57, 94)), # n19
       ((37, 64), (5, 78)), # n20
       ((53, 62), (53, 40), (25, 40), (25, 28)), # n21
       ((155, 28), (153, 76)), # n23
       ((171, 28), (143, 76), (143, 64)), # n24
       ((55, 12), (5, 58)), # n25
       ((33, 10), (21, 22)), # n26
       ((127, 24), (17, 10)), # n27
       ((95, 24), (95, 10)), # n28
       ((167, 44), (161, 10), (105, 26), (99, 8)), # n29
       ((143, 10), (123, 22), (107, 22), (105, 10), (101, 26)), # n30
       ((137, 10), (101, 10), (99, 28), (65, 10)), # n31
       ((157, 8), (141, 8)), # n32
       ((137, 138), (11, 136)), # n35
       ((93, 122), (81, 118), (49, 132)), # n36
       ((91, 116), (91, 100)), # n37
       ((105, 130), (83, 116)) # n38
       ]

scaled_Net_points = [
    [(x * scalar, y * scalar) for x, y in segment] 
    for segment in Net
]
all_points = list(chain.from_iterable(scaled_Net_points))

# Define obstacles
obstacles = []
obstacles.extend(all_points)
data_array = set_obstacle(data_array, obstacles)

# Create an occupancy grid map
gmap = OccupancyGridMap(data_array, cell_size)

# Initialize variables
colors = ['red', 'blue', 'green', 'purple', 'orange', 'cyan', 'magenta', 'yellow']

# Find paths for each start and end point combination
paths, success, total_costs = a_star_loop(scaled_Net_points, gmap, '4N')

# End measuring time
end_time = time.time()

cost_sum = sum(total_costs[key] for key in total_costs)
print("Sum of total HPWL:", cost_sum)
elapsed_time = (end_time - start_time)*1000
print("Elapsed time: {:.4f} ms".format(elapsed_time))
output_file = "./paths.csv"
save_paths_to_csv(paths, output_file)
print(f"Paths saved to {output_file}")
# # Visualize path with gds format
# create_gds_paths_with_ports_and_squares(paths, scaled_start_points, scaled_end_points, width=0.3)

# Visualize the grid, obstacles, and the paths
plot_routing(gmap, paths, colors)
