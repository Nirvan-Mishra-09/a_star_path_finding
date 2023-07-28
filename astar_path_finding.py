import numpy as np
import math
import heapq
import cv2 as cv

# -------------------------------Step 1: Map using opencv----------------------------------------------
c = int(input('Enter clearance of the obstacles: '))


def rectangle1(x, y):
    return 0 <= y <= 100 <= x <= 150


def rectangle1_c(x, y):
    return 0 <= y <= 100+c and 150 + c >= x >= 100 - c


def rectangle2(x, y):
    return 100 <= x <= 150 <= y <= 250


def rectangle2_c(x, y):
    return 150-c <= y <= 250 and 150 + c >= x >= 100 - c


canvas = np.zeros((251, 601, 3), np.uint8)
# canvas.fill(255)
for y in range(251):
    for x in range(601):
        if rectangle1(x, y) or rectangle2(x, y):
            canvas[y, x] = [0, 255, 255]
        elif rectangle1_c(x,y) or rectangle2_c(x, y):
            canvas[y, x] = [0, 0, 255]

center_hex = (300, 125)
side = 75
angle_deg = 60
vertices = []

for i in range(6):
    angle = angle_deg * i
    angle_rad = np.pi / 180 * (angle + 90)

    x = int(center_hex[0] + side * np.cos(angle_rad))
    y = int(center_hex[1] + side * np.sin(angle_rad))
    vertices.append((x, y))

cv.fillPoly(canvas, [np.array(vertices)], (0, 255, 255))

side_c = 75
vertices_c = []
for i in range(6):
    angle = angle_deg * i
    angle_rad = np.pi / 180 * (angle + 90)

    x = int(center_hex[0] + side_c * np.cos(angle_rad))
    y = int(center_hex[1] + side_c * np.sin(angle_rad))
    vertices_c.append((x, y))
cv.polylines(canvas, [np.array(vertices_c)], isClosed=True, color=(0, 0, 255), thickness=c)

tri_vertices = np.array([[[460, 25]], [[460, 225]], [[510, 125]]])
tri_vertices_c = np.array([[[460, 25-c]], [[460, 225+c]], [[510, 125+c]]])
canvas = cv.fillPoly(canvas, [tri_vertices], color=(0, 255, 255))
canvas = cv.polylines(canvas, [np.array(tri_vertices_c)],isClosed=True, color=(0, 0, 255), thickness=c)

obstacle_space = []

for y in range(canvas.shape[0]):
    for x in range(canvas.shape[1]):
        if canvas[y, x].any():
            obstacle_space.append((x, y))


# -----------------------------------------Step 2: UI---------------------------------------------
coordinate = True
while coordinate:

    start_x = input('Enter starting node x coordinate: ')
    start_y = input('Enter starting node y coordinate: ')
    start_angle = input('Enter start theta (in degrees): ')
    goal_x = input('Enter goal node x coordinate: ')
    goal_y = input('Enter goal node y coordinate: ')
    goal_angle = input('Enter goal theta (in degrees): ')
    radius = int(input('Enter radius of robot: '))

    start_node = (int(start_x), int(start_y), int(start_angle))
    goal_node = (int(goal_x), int(goal_y), int(goal_angle))

    if (start_node[0], start_node[1]) in obstacle_space:
        print("The start coordinates falls into obstacles.")
    elif (goal_node[0], goal_node[1]) in obstacle_space:
        print("The goal coordinates falls into obstacles.")
    elif start_node[0] >= 601 and start_node[1] >= 251:
        print("The coordinates falls outside the map.")
    elif goal_node[0] >= 601 and goal_node[1] >= 251:
        print("The coordinates falls outside the map.")
    else:
        cv.circle(canvas, (goal_node[0], goal_node[1]), 1, (0, 0, 255), 2)
        coordinate = False

current_node = start_node


# ----------------------------Action Set--------------------------------------------------------------------------------


def forward(current_node, cost=1):
    x, y, theta = current_node
    next_node = (round(x + radius * np.cos(np.deg2rad(theta))), round(y + radius * np.sin(np.deg2rad(theta))), theta)
    if 0 <= next_node[0] <= 600 and 0 <= next_node[1] <= 250 and 0 <= next_node[2] <= 360 and (next_node[0], next_node[1]) not in obstacle_space:
        return next_node, cost
    else:
        return None, None


def above_30(current_node, cost=1):
    x, y, theta = current_node
    next_node = (round(x + radius * np.cos(np.deg2rad(theta + 30))), round(y + radius * np.sin(np.deg2rad(theta + 30))), theta - 30)
    if 0 <= next_node[0] <= 600 and 0 <= next_node[1] <= 250 and 0 <= next_node[2] <= 360 and (next_node[0], next_node[1]) not in obstacle_space:
        return next_node, cost
    else:
        return None, None


def above_60(current_node, cost=1):
    x, y, theta = current_node
    next_node = (round(x + radius * np.cos(np.deg2rad(theta + 60))), round(y + radius * np.sin(np.deg2rad(theta + 60))), theta - 60)
    if 0 <= next_node[0] <= 600 and 0 <= next_node[1] <= 250 and 0 <= next_node[2] <= 360 and (next_node[0], next_node[1]) not in obstacle_space:
        return next_node, cost
    else:
        return None, None


def down_30(current_node, cost=1):
    x, y, theta = current_node
    next_node = (round(x + radius * np.cos(np.deg2rad(theta - 30))), round(y + radius * np.sin(np.deg2rad(theta - 30))), theta + 30)
    if 0 <= next_node[0] <= 600 and 0 <= next_node[1] <= 250 and 0 <= next_node[2] <= 360 and (next_node[0], next_node[1]) not in obstacle_space:
        return next_node, cost
    else:
        return None, None


def down_60(current_node, cost=1):
    x, y, theta = current_node
    next_node = (round(x + radius * np.cos(np.deg2rad(theta - 60))), round(y + radius * np.sin(np.deg2rad(theta - 60))), theta + 60)
    if 0 <= next_node[0] <= 600 and 0 <= next_node[1] <= 250 and 0 <= next_node[2] <= 360 and (next_node[0], next_node[1]) not in obstacle_space:
        return next_node, cost
    else:
        return None, None


# -----------------------------------------------------------------------------------------------------------------------
action_set = [forward, above_30, above_60, down_30, down_60]

visited = np.empty((1201, 501, 13), dtype=object)

for i in range(1201):
    for j in range(501):
        for k in range(13):
            visited[i][j][k] = 0

open_list = []
closed_dict = {}
thresh_dist = 0.5
thresh_ang = 30
cost_to_go = math.sqrt((goal_node[0] - current_node[0]) ** 2 + (goal_node[1] - current_node[1]) ** 2)
heapq.heapify(open_list)
heapq.heappush(open_list, (cost_to_go, 0, start_node))
closed_dict[start_node] = (start_node, None, 0 + cost_to_go)
visited[int(start_node[0] / thresh_dist)][int(start_node[1] / thresh_dist)][int(start_node[2] / thresh_ang)] = 1


def astar(start, goal):
    while open_list:
        cost_2_go, cost_to_come, current = heapq.heappop(open_list)

        if current == goal:
            print('Goal Reached.')
            path = []
            while closed_dict[current][0] != closed_dict[start][0]:
                path.append(closed_dict[current][0])
                current = closed_dict[current][1]
            path.append(closed_dict[start][0])
            path.reverse()
            for point in path:
                cv.circle(canvas, (point[0], point[1]), 3,(255,0,255),1)
                flip = cv.flip(canvas, 0)
                cv.imshow('Map', flip)
                cv.waitKey(5)
            return path, closed_dict[goal][2]

        elif visited[int(current[0] / thresh_dist)][int(current[1] / thresh_dist)][int(current[2] / thresh_ang)] == 1 and closed_dict[current][2] < cost_to_come + cost_2_go:
            continue
        else:
            for moves in action_set:
                next_node, cost = moves(current)

                if cost is not None:
                    cost_to_come += cost

                if next_node is not None and (next_node[0], next_node[1]) not in obstacle_space:
                    cv.circle(canvas, (next_node[0], next_node[1]), 3, (255, 255, 0), 1)
                    flip = cv.flip(canvas, 0)
                    cv.imshow('Map', flip)
                    cv.waitKey(1)
                try:
                    if next_node is not None and visited[int(next_node[0] / thresh_dist)][int(next_node[1] / thresh_dist)][int(next_node[2] / thresh_ang)] != 1 and (next_node[0], next_node[1]) not in obstacle_space:
                        cost_2_go = math.sqrt((goal_node[0] - next_node[0]) ** 2 + (goal_node[1] - next_node[1]) ** 2)
                        heapq.heappush(open_list, (cost_2_go, cost_to_come, next_node))
                        visited[int(next_node[0] / thresh_dist)][int(next_node[1] / thresh_dist)][int(next_node[2] / thresh_ang)] = 1
                        closed_dict[next_node] = (next_node, current, cost_2_go + cost_to_come)

                    elif next_node is not None and visited[int(next_node[0] / thresh_dist)][int(next_node[1] / thresh_dist)][int(next_node[2] / thresh_ang)] == 1 and closed_dict[next_node][2] > cost_2_go + cost_to_come and (next_node[0], next_node[1]) not in obstacle_space:
                        closed_dict[next_node] = (next_node, current, cost_2_go + cost_to_come)

                except: pass
    return None, None


route, net_cost = astar(start_node, goal_node)

print(net_cost)
print(route)
