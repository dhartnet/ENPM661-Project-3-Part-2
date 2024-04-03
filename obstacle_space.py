from queue import PriorityQueue
import time

import numpy as np
import cv2

# Creates a list of the obstacles in the workspace
def obstacle_space():
    obstacle_list = []

    obstacle_list.append(((0,0),(600,200)))
    obstacle_list.append(((150,200),(175,100)))
    obstacle_list.append(((250,0),(275,100)))
    obstacle_list.append((420,120,60))

    return obstacle_list

# Populates the canvas with the obstacles
def draw_obstacles(canvas, obstacles, video_output):
    for obstacle in obstacles:       
        if len(obstacle) == 2 and obstacle != ((0,0),(600,200)):
            start_x = obstacle[0][0]
            # Invert y-value
            start_y = 200 - obstacle[0][1]
            end_x = obstacle[1][0]
            # Invert y-value
            end_y = 200 - obstacle[1][1]
            start_coordinate = (start_x, start_y)
            end_coordinate = (end_x, end_y)
            print(start_coordinate, "", end_coordinate)
            cv2.rectangle(canvas, pt1=start_coordinate, pt2=end_coordinate, color=(0, 0, 0), thickness=-1)
        elif len(obstacle) == 3:
            cv2.circle(canvas,(obstacle[0],200-obstacle[1]), obstacle[2], (0,0,0), -1)
    cv2.imshow('A*', canvas)
    video_output.write(canvas)
    cv2.waitKey(5000)
    return

# Initialize Canvas
canvas = np.ones((200, 600, 3), dtype=np.uint8) * 255  # White canvas

# Initialize VideoWriter
v_writer = cv2.VideoWriter_fourcc(*'mp4v')
fps = 60
video_output = cv2.VideoWriter('a_star_output.mp4', v_writer, fps, (1200, 500))

obstacles = obstacle_space()
draw_obstacles(canvas, obstacles, video_output)
