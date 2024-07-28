import numpy as np
import matplotlib.pyplot as plt

class OccupancyGridMap:
    def __init__(self, data_array, cell_size, occupancy_threshold=1.0):
        self.data = data_array
        self.dim_cells = data_array.shape
        self.dim_meters = (self.dim_cells[0] * cell_size, self.dim_cells[1] * cell_size)
        self.cell_size = cell_size
        self.occupancy_threshold = occupancy_threshold
        self.visited = np.zeros(self.dim_cells, dtype=np.float32)
        self.static_obstacles = np.copy(data_array)  # Separate array to track static obstacles
        self.path_visited = np.zeros(self.dim_cells, dtype=np.float32)
    
    def set_visited_empty(self):
        self.visited = np.zeros(self.dim_cells, dtype=np.float32)

    def erase_visited_idx(self, point_idx):  # start point를 위해
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[1] or y_index >= self.dim_cells[0]:
            raise Exception('Point is outside map boundary')
        self.visited[y_index][x_index] = 0.0

    def mark_visited_idx(self, point_idx):
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[1] or y_index >= self.dim_cells[0]:
            raise Exception('Point is outside map boundary')
        self.visited[y_index][x_index] = 1.0

    def mark_visited(self, point):
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)
        return self.mark_visited_idx((x_index, y_index))

    def is_visited_idx(self, point_idx):
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[1] or y_index >= self.dim_cells[0]:
            raise Exception('Point is outside map boundary')
        return self.visited[y_index][x_index] == 1.0

    def is_visited(self, point):
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)
        return self.is_visited_idx((x_index, y_index))

    def get_data_idx(self, point_idx):
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[1] or y_index >= self.dim_cells[0]:
            raise Exception('Point is outside map boundary')
        return self.data[y_index][x_index]

    def get_data(self, point):
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)
        return self.get_data_idx((x_index, y_index))

    def set_data_idx(self, point_idx, new_value):
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[1] or y_index >= self.dim_cells[0]:
            raise Exception('Point is outside map boundary')
        self.data[y_index][x_index] = new_value

    def set_data(self, point, new_value):
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)
        self.set_data_idx((x_index, y_index), new_value)

    def is_inside_idx(self, point_idx):
        x_index, y_index = point_idx
        return 0 <= x_index < self.dim_cells[1] and 0 <= y_index < self.dim_cells[0]

    def is_inside(self, point):
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)
        return self.is_inside_idx((x_index, y_index))

    def is_occupied_idx(self, point_idx):
        return self.get_data_idx(point_idx) >= self.occupancy_threshold

    def is_occupied(self, point):
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)
        return self.is_occupied_idx((x_index, y_index))

    def get_index_from_coordinates(self, x, y):
        x_index = int(round(x / self.cell_size))
        y_index = int(round(y / self.cell_size))
        return x_index, y_index

    def get_coordinates_from_index(self, x_index, y_index):
        x = x_index * self.cell_size
        y = y_index * self.cell_size
        return x, y

    def plot(self, alpha=1, min_val=0, origin='lower'):
        plt.imshow(self.static_obstacles, vmin=min_val, vmax=1, origin=origin, interpolation='none', alpha=alpha)
        plt.draw()
