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
    gmap.set_visited_empty(layer=0)  # Layer 0 초기화
    gmap.set_visited_empty(layer=1)  # Layer 1 초기화

    start = gmap.get_index_from_coordinates(start_m[0], start_m[1])
    goal = gmap.get_index_from_coordinates(goal_m[0], goal_m[1])

    if gmap.is_occupied_idx(start,0):
        gmap.set_data(start, 0, 0)
        gmap.set_data(start, 0, 1)

    if gmap.is_occupied_idx(goal,0):
        gmap.set_data(goal, 0, 0)
        gmap.set_data(goal, 0, 1)

    if gmap.is_visited_idx(start, layer=0):
        gmap.erase_visited_idx(start, layer=0)

    if gmap.is_visited_idx(goal, layer=0):
        gmap.erase_visited_idx(goal, layer=0)

    start_node_cost = 0
    start_node_estimated_cost_to_goal = dist2d(start, goal) + start_node_cost
    front = [(start_node_estimated_cost_to_goal, start_node_cost, start, None, 0, 0)]  # 마지막 숫자는 현재 layer
    came_from = {0: {}, 1: {}}  # Layer 0과 1의 경로 추적을 분리

    if movement == '4N':
        movements = _get_movements_4n()
    elif movement == '8N':
        movements = _get_movements_8n()
    else:
        raise ValueError('Unknown movement')

    while front:
        element = heappop(front)
        total_cost, cost, pos, previous, current_layer, previous_layer = element

        # 이미 방문했는지 확인
        if gmap.is_visited_idx(pos, layer=current_layer):
            continue

        # 현재 레이어에 방문 표시
        gmap.mark_visited_idx(pos, layer=current_layer)

        # 방향 전환 시 다른 레이어에도 방문 표시
        if previous:
            dx = pos[0] - previous[0]
            dy = pos[1] - previous[1]
            if dx != 0 and dy != 0:  # 방향이 바뀌는 경우
                gmap.mark_visited_idx(pos, layer=1 - current_layer)

        # 경로 추적
        came_from[current_layer][pos] = (previous, previous_layer)

        if pos == goal:
            success += 1
            break

        for dx, dy, deltacost in movements:
            new_x = pos[0] + dx
            new_y = pos[1] + dy
            new_pos = (new_x, new_y)

            if not gmap.is_inside_idx(new_pos):
                continue

            new_layer = 0 if dy == 0 else 1  # 방향에 따라 새로운 레이어 결정

            if not gmap.is_visited_idx(new_pos, layer=new_layer) and not gmap.is_occupied_idx(new_pos, new_layer):
                potential_function_cost = gmap.get_data_idx(new_pos,new_layer) * occupancy_cost_factor
                new_cost = cost + deltacost + potential_function_cost
                new_total_cost_to_goal = new_cost + dist2d(new_pos, goal) + potential_function_cost
                heappush(front, (new_total_cost_to_goal, new_cost, new_pos, pos, new_layer, current_layer))

    # 경로 재구성
    path = []
    path_idx = []
    if pos == goal:
        while pos:
            path_idx.append(pos)
            pos_m_x, pos_m_y = gmap.get_coordinates_from_index(pos[0], pos[1])
            path.append((pos_m_x, pos_m_y))

            # 이전 노드와 레이어로 이동
            pos, current_layer = came_from[current_layer].get(pos, (None, None))

        path.reverse()
        path_idx.reverse()
    
    return path, path_idx, success, cost
