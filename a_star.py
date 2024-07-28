import math
from heapq import heappush, heappop

def dist2d(point1, point2):
    x1, y1 = point1[0:2]
    x2, y2 = point2[0:2]
    dist2 = (x1 - x2) ** 2 + (y1 - y2) ** 2
    return math.sqrt(dist2)

def _get_movements_4n():
    return [(1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0)]

def _get_movements_8n():
    s2 = math.sqrt(2)
    return [(1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0), (1, 1, s2), (-1, 1, s2), (-1, -1, s2), (1, -1, s2)]

def a_star(start_m, goal_m, gmap, success, movement='8N', occupancy_cost_factor=3):
    gmap.set_visited_empty()
    start = gmap.get_index_from_coordinates(start_m[0], start_m[1])
    goal = gmap.get_index_from_coordinates(goal_m[0], goal_m[1])

    if gmap.is_occupied_idx(start):
        gmap.set_data(start, 0)

    if gmap.is_occupied_idx(goal):
        gmap.set_data(goal, 0)

    if gmap.is_visited_idx(start):
        gmap.erase_visited_idx(start)

    if gmap.is_visited_idx(goal):
        gmap.erase_visited_idx(goal)

    start_node_cost = 0
    start_node_estimated_cost_to_goal = dist2d(start, goal) + start_node_cost
    front = [(start_node_estimated_cost_to_goal, start_node_cost, start, None)]
    came_from = {}

    if movement == '4N':
        movements = _get_movements_4n()
    elif movement == '8N':
        movements = _get_movements_8n()
    else:
        raise ValueError('Unknown movement')

    while front:
        element = heappop(front)
        total_cost, cost, pos, previous = element

        if gmap.is_visited_idx(pos):
            continue

        gmap.mark_visited_idx(pos)
        came_from[pos] = previous

        if pos == goal:
            success += 1
            break

        for dx, dy, deltacost in movements:
            new_x = pos[0] + dx
            new_y = pos[1] + dy
            new_pos = (new_x, new_y)

            if not gmap.is_inside_idx(new_pos):
                continue

            if not gmap.is_visited_idx(new_pos) and not gmap.is_occupied_idx(new_pos):
                potential_function_cost = gmap.get_data_idx(new_pos) * occupancy_cost_factor
                new_cost = cost + deltacost + potential_function_cost
                new_total_cost_to_goal = new_cost + dist2d(new_pos, goal) + potential_function_cost
                heappush(front, (new_total_cost_to_goal, new_cost, new_pos, pos))

    path = []
    path_idx = []
    if pos == goal:
        while pos:
            path_idx.append(pos)
            pos_m_x, pos_m_y = gmap.get_coordinates_from_index(pos[0], pos[1])
            path.append((pos_m_x, pos_m_y))
            pos = came_from[pos]

        path.reverse()
        path_idx.reverse()

    return path, path_idx, success, cost
