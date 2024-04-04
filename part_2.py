# ENPM661 Project 3 Part 2 Submission
# Rey Roque-Pérez and Dustin Hartnett

# Github Repository: https://github.com/dhartnet/ENPM661-Project-3-Part-2

from queue import PriorityQueue
import time

import numpy as np
import cv2

# PriorityQueues entries in python are not updatable
# This class is an implementation of the PriorityQueue that allows updating
# It does this by keeping a copy of all items in the queue in a dictionary
# It then uses the dictionary to search if an item is in the queue
# It also passes a new argument to the put method (priority, item)
class UpdateableQueue:
    def __init__(self):
        self.queue = PriorityQueue()
        # Maps items to their corresponding queue entries
        self.entry_finder = {}

    # Adds or updates item in PriorityQueue
    def put(self, priority, node):
        # Extracting the key (x, y, theta) from the node
        key = node[0]
        if key in self.entry_finder:
            current_priority, _ = self.entry_finder[key]
            if priority < current_priority:
                # Update the priority in the entry_finder
                self.entry_finder[key] = (priority, node)
                # Update the priority in the queue
                self.queue.put((priority, node))
                
        else:
            entry = (priority, node)
            self.entry_finder[key] = entry
            self.queue.put(entry)

    def remove(self, key):
        entry = self.entry_finder.pop(key)
        # Return the item removed from the queue
        return entry[1]  

    def get(self):
        priority, node = self.queue.get()
        # Check if the item is still in entry_finder
        key = node[0]
        if key in self.entry_finder:
            # Remove the item from the entry_finder
            del self.entry_finder[key] 
        return priority, node

    def empty(self):
        return self.queue.empty()

    def __contains__(self, key):
        return key in self.entry_finder

###---------------------------------------###
# Check to see if node in question lies within obstacle space
# Return 'False' if in free space, 'True' if in an obstacle or outside the boundaries
# These equations include the 5 space boundary around the obstacles  
def inObstacle(maybeNode, clearance, radius):

    node = tuple(maybeNode)
    xnode = node[0]
    ynode = node[1]
    vibes = False

    h = 420 # x coord of center of circle obstacle
    k = 120 # y coord of center of circle obstacle
    r = 60 # radius of circle obstacle

    # check if in map
    if xnode < clearance + radius or xnode > 600 - clearance - radius or ynode < clearance + radius or ynode > 200 - clearance - radius:
        vibes = True

    # check first obstacle (rectangle)
    elif xnode > 150 - clearance - radius and xnode < 175 + clearance + radius and ynode > 100 - clearance - radius:
        vibes = True

    # check second obstacle (rectangle)
    elif xnode > 250 - clearance - radius and xnode < 275 + clearance + radius and ynode < 100 + clearance + radius:
        vibes = True

    # check third obstacle (circle)
    elif (xnode - h)**2 + (ynode - k)**2 - (r + clearance + radius)**2 <= 0:
        vibes = True

  # return "vibes". False = node is in free space. True = node is out of map or in obstacle.
    return vibes

def inGoal(node, goal_node):
    goal_radius = 10 # cm
    x_goal = goal_node[0]
    y_goal = goal_node[1]
    x_node = node[0]
    y_node = node[1]
    return np.sqrt(np.square(x_node-x_goal) + np.square(y_node-y_goal)) < goal_radius

# Used to draw an angled square "robot" during visualization
def getRecPoints(currentNode):

  # coordinates of center of circle and orientation
  xc = currentNode[0]
  yc = currentNode[1]
  theta = currentNode[2]*30

  # square side length
  s = 5.0

  # top right
  x1 = xc + s*(np.cos(theta) - np.sin(theta))
  y1 = 200 - yc + s*(np.sin(theta) + np.cos(theta))

  # top left
  x2 = xc + s*(-np.cos(theta) - np.sin(theta))
  y2 = 200 - yc + s*(-np.sin(theta) + np.cos(theta))

  # bottom left
  x3 = xc + s*(-np.cos(theta) + np.sin(theta))
  y3 = 200 - yc + s*(-np.sin(theta) - np.cos(theta))

  # bottom right
  x4 = xc + s*(np.cos(theta) + np.sin(theta))
  y4 = 200 - yc + s*(np.sin(theta) - np.cos(theta))

  coords = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])

  return coords

# Draws start node and end node
def draw_nodes(canvas, start_node, goal_node):
    cv2.rectangle(canvas, (start_node[0] - 1, 200 - start_node[1] - 1), (start_node[0] + 1, 200 - start_node[1] + 1), color=(0, 250, 0), thickness=cv2.FILLED)
    cv2.rectangle(canvas, (goal_node[0] - 1, 200 - goal_node[1] - 1), (goal_node[0] + 1, 200 - goal_node[1] + 1), color=(0, 0, 255), thickness=cv2.FILLED)

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
            #print(start_coordinate, "", end_coordinate)
            cv2.rectangle(canvas, pt1=start_coordinate, pt2=end_coordinate, color=(0, 0, 0), thickness=-1)
        elif len(obstacle) == 3:
            cv2.circle(canvas,(obstacle[0],200-obstacle[1]), obstacle[2], (0,0,0), -1)
    canvas1 = cv2.resize(canvas, (1200, 400))            
    cv2.imshow('A*', canvas1)
    video_output.write(canvas)
    cv2.waitKey(2000)
    return

# Populates and updates the canvas with explored nodes
def draw_explored(canvas, points, video_output, u1, u2):
    count = 0
    t = 0 # start time
    r =  3.3 # wheel radius cm
    L =  28.7 # robot diameter cm
    dt = 0.1 # time step for plotting

    actions = [[0, u1], [u1, 0], [u1, u1], [0, u2], [u2, 0], [u2, u2], [u1, u2], [u2, u1]]

    # Start from the second point
    for i in range(1, len(points)): 
        point = points[i]
            
        current_node = point[0]

        for action in actions:
            x = int(current_node[0])
            y = int(current_node[1])
            theta = np.pi * current_node[2] * 30 / 180 # orientation in radians  
            t = 0
            ul = action[0] # left wheel vel
            ur = action[1] # right wheel vel
            
            while t <= 1.0:
                t = t + dt
                xs = x
                ys = y
                x += int((r/2) * (ul + ur) * np.cos(theta) * dt)
                y += int((r/2) * (ul + ur) * np.sin(theta) * dt)
                theta += (r/L) * (ur - ul) * dt
                cv2.line(canvas, (xs, 200 - ys), (x, 200 - y), (255,0,0), 1) # image, point 1, point 2, color, thickness

        count += 1
        if count % 200 == 0:
            count = 0
            canvas1 = cv2.resize(canvas, (1200, 400))            
            cv2.imshow('A*', canvas1)
            cv2.waitKey(10)
            video_output.write(canvas)
    return

# Populates and updates the canvas with path nodes
def draw_path(canvas, path, video_output, u1, u2):
    count = 0
    t = 0 # start time
    r =  3.3 # wheel radius cm
    L =  28.7 # robot diameter cm
    dt = 0.1 # time step for plotting

    actions = [[0, u1], [u1, 0], [u1, u1], [0, u2], [u2, 0], [u2, u2], [u1, u2], [u2, u1]]

    for i in range(1, len(path)-1): 

        point = path[i]
            
        current_node = point[0]

        for action in actions:
            x = int(current_node[0])
            y = int(current_node[1])
            theta = np.pi * current_node[2] * 30 / 180 # orientation in radians  
            t = 0
            ul = action[0] # left wheel vel
            ur = action[1] # right wheel vel
            
            while t <= 1.0:
                t = t + dt
                xs = x
                ys = y
                x += int((r/2) * (ul + ur) * np.cos(theta) * dt)
                y += int((r/2) * (ul + ur) * np.sin(theta) * dt)
                theta += (r/L) * (ur - ul) * dt
                cv2.line(canvas, (xs, 200 - ys), (x, 200 - y), (0,0,255), 1) # image, point 1, point 2, color, thickness

        count += 1
        if count % 1 == 0:
            count = 0
            canvas1 = cv2.resize(canvas, (1200, 400))            
            cv2.imshow('A*', canvas1)
            video_output.write(canvas1)
            video_output.write(canvas1)
            video_output.write(canvas1)
            video_output.write(canvas1)
            cv2.waitKey(int(1000 / 20))

    cv2.waitKey(int(4000))

    return

# Adds seconds at end of video
def add_blank_frames(canvas, video_output, fps, seconds):
    blank_frames = fps * seconds
    for _ in range(blank_frames):
        video_output.write(canvas)
    return

# Calls the functions for populating the canvas
def record_animation(obstacles, explored, path, start_node, goal_node):
    # Initialize VideoWriter
    v_writer = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 60
    video_output = cv2.VideoWriter('a_star_output.mp4', v_writer, fps, (1200, 500))

    canvas = np.ones((200, 600, 3), dtype=np.uint8) * 255  # White canvas
    
    draw_nodes(canvas, start_node, goal_node)
    draw_obstacles(canvas, obstacles, video_output)
    add_blank_frames(canvas, video_output, fps, 2)    
    draw_explored(canvas, explored, step, video_output)
    draw_nodes(canvas, start_node, goal_node)
    draw_path(canvas,path, step, video_output)
    cv2.waitKey(3000)
    add_blank_frames(canvas, video_output, fps, 2)
    video_output.release()
    return

# Creates a list of the obstacles in the workspace
def obstacle_space():
    obstacle_list = []

    obstacle_list.append(((0,0),(600,200)))
    obstacle_list.append(((150,200),(175,100)))
    obstacle_list.append(((250,0),(275,100)))
    obstacle_list.append((420,120,60))

    return obstacle_list

###### Move functions ######
def newNodes(nodeState, clearance, radius, u1, u2): # (node, lower wheel velocity, higher wheel velocity) speeds in RPM
  # Extract node information from nodeState value
  node = tuple(nodeState)
  xi = node[0]
  yi = node[1]
  thetai = node[2]*30 # deg

  u1 = np.pi * u1 / 30
  u2 = np.pi * u2 / 30

  # Define action set from wheel rotational velocities
  actions = [[0, u1], [u1, 0], [u1, u1], [0, u2], [u2, 0], [u2, u2], [u1, u2], [u2, u1]]

  # make empty lists to fill with values
  newNodes = [] # new node information
  c2c = [] # list of costs to come for each new node from parent node

  # define constants and counter values
  t = 0 # time (sec)
  dt = 0.1 # time step for integrating
  r =  3.3 # wheel radius cm
  L =  28.7 # robot diameter cm

  for action in actions:
    ul = action[0] # left wheel rotation velocity (rad/s)
    ur = action[1] # right wheel rotation velocity (rad/s)
    theta = np.pi * thetai / 180 # orientation in radians
    x = xi # set x value to initial node value before integration
    y = yi # set y value to initial node value before integration

    # we let each action run for one second, with a time step of 0.1 seconds for integration calculation
    while t <= 1.0:
      t = t + dt
      x = x + (r/2) * (ul + ur) * np.cos(theta) * dt
      y = y + (r/2) * (ul + ur) * np.sin(theta) * dt
      theta = theta + (r/L) * (ur - ul) * dt

    v = (r/2) * (ul + ur)
    ang = (r/L) * (ur - ul) # rad/s
    ang = round(30 * ang / np.pi, 2) # rpm
    t = 0
    c2c = np.sqrt((xi - x)**2 + (yi - y)**2) # cost to come is linear displacement, not calculating distance covered
    newX = int(round(x,0))
    newY = int(round(y,0))
    new_theta = int(((180 * theta / np.pi) % 360)//30)  # Rounded theta for comparing nodes
    #if not (newX < clearance + radius or newX > 600 - clearance - radius or newY < clearance + radius or newY > 200 - clearance - radius):
    newNodes.append(((newX, newY, new_theta), round(c2c,2), (v, ang))) # outputs node, cost to come, and associated linear and ang velocity to get to each node (to be sent to robot)

  return newNodes
###### End Move functions ######

# Calculates cost to go
def heuristic(node, goal_node, weight):
  return weight * np.sqrt(np.square(goal_node[0] - node[0]) + np.square(goal_node[1] - node[1]))

def a_star_algorithm(start, goal, weight, rpm1, rpm2, clearance, radius):
    start_node = (int(start[0]), int(start[1]), start[2])
    goal_node = (int(goal[0]), int(goal[1]), goal[2])

    # Create cost_grid and initialize cost to come for start_node
    cost_grid = [[[float('inf')] * 12 for _ in range(200)] for _ in range(600)]
    cost_grid[start_node[0]][start_node[1]][start_node[2]] = 0

    # Create grid to store parents
    parent_grid = [[[None] * 12 for _ in range(200)] for _ in range(600)]
    parent_grid[start_node[0]][start_node[1]][start_node[2]] = None

    # Create grid to store parents
    visited_grid = [[[False] * 12 for _ in range(200)] for _ in range(600)]
    visited_list = []

    # Priority queue to store open nodes
    # Cost to come, coordinate values (x,y), parent
    open_queue = UpdateableQueue()
    open_queue.put(0, (start_node,(0),(0,0)))  # (priority, node)

    while not open_queue.empty():
        _, node_tuple = open_queue.get()
        node = node_tuple[0]
        visited_grid[node[0]][node[1]][node[2]] = True
        visited_list.append(node_tuple)

        if inGoal(node, goal_node):
            return parent_grid, visited_list

        # Get neighboring nodes
        actions = newNodes(node, clearance, radius, rpm1, rpm2)
        node_cost = cost_grid[node[0]][node[1]][node[2]]

        for action in actions:
            #print(action)
            action_cost = action[1]
            move = action[0]
            if not visited_grid[move[0]][move[1]][move[2]] and not inObstacle(move, clearance, radius):
                new_cost = node_cost + action_cost
                if new_cost < cost_grid[move[0]][move[1]][move[2]]:
                    cost_grid[move[0]][move[1]][move[2]] = new_cost
                    priority = new_cost + heuristic(move, goal_node, weight)
                    open_queue.put(priority, action)                    
                    parent_grid[move[0]][move[1]][move[2]] = node_tuple

    return parent_grid, visited_list, print("Failed to find goal")

# Backtracking using path list created from visited/path dictionary
def find_path(parent_grid, visited_list, start):
    node_tuple = visited_list[-1] #((x,y,theta),(),(v,ang))
    current_node = node_tuple[0]
    path = [node_tuple]
    start_node = start
    while start_node != current_node:
        #print('\n',current_node[0],'\n',current_node[1],'\n',current_node[2],'\n')
        temp_node = parent_grid[int(current_node[0])][int(current_node[1])][current_node[2]]
        current_node = temp_node
        path.insert(0, current_node)
        current_node = current_node[0]
    return path

#### Main ###
# Get obstacle coordinates
obstacles = obstacle_space()
weight = 1 # option for weighted A*

clearance = 2 # cm
radius = 22 # cm

# Get and verify input coordinates
# xs = int(input('Enter x coordinate value for start coordinate: '))
# ys = int(input('Enter y coordinate value for start coordinate: '))
# thetas = int(input('Enter theta value for start coordinate: '))//30
# start_node = tuple((xs, ys, thetas))
# while inObstacle(start_node, clearance):
#     print('Node outside workspace or in obstacle. Choose new start location')
#     xs = int(input('Enter x coordinate value for start location: '))
#     ys = int(input('Enter y coordinate value for start location: '))
#     thetas = int(input('Enter theta value for start coordinate (must be multiple of 30 degrees): '))//30     
#     start_node = tuple((xs, ys, thetas))
# start_node = tuple((xs, ys, thetas)) # cm

# # Get and verify input coordinates
# xg = int(input('Enter x coordinate value for goal coordinate: '))
# yg = int(input('Enter y coordinate value for goal coordinate: '))
# goal_node = tuple((xg, yg)) # cm
# while inObstacle(goal_node, clearance):
#     print('Node outside workspace or in obstacle. Choose new goal location')
#     xg = int(input('Enter x coordinate value for goal location: '))
#     yg = int(input('Enter y coordinate value for goal location: '))
#     goal_node = tuple((xg, yg)) # cm
# goal_node = tuple((xg, yg)) # cm

# start = (int(start_node[0]), int(start_node[1]), start_node[2]) # cm
# goal = (int(goal_node[0]), int(goal_node[1]), goal_node[2]) # cm

#rpm1 = int(input('Enter first RPM value: '))
#rpm2 = int(input('Enter second RPM value: '))

start_node = (50, 100, 0)
goal_node = (250, 175, 0)
rpm1 = (int(5))
rpm2 = (int(10))

# Start timer
ti = time.time()

print('Exploring nodes...')
explored = a_star_algorithm(start_node, goal_node, weight, rpm1, rpm2, clearance, radius)
parent_grid = explored[0]
visited_list = explored[1]

# Needs Updating
print('Generating path...')
path = find_path(parent_grid, visited_list, start_node)

# Get time taken to find path
tf = time.time()
print('Path found in: ', tf-ti)
# print(path)

# Initialize Canvas
canvas = np.ones((200, 600, 3), dtype=np.uint8) * 255  # White canvas

# Initialize VideoWriter
v_writer = cv2.VideoWriter_fourcc(*'mp4v')
fps = 60
video_output = cv2.VideoWriter('a_star_output.mp4', v_writer, fps, (1200, 500))

draw_nodes(canvas, start_node, goal_node)

obstacles = obstacle_space()
draw_obstacles(canvas, obstacles, video_output)

draw_explored(canvas, visited_list, video_output, rpm1, rpm2)

draw_nodes(canvas, start_node, goal_node)

draw_path(canvas, path, video_output, rpm1, rpm2)

# Needs Updating
#record_animation(obstacles, visited_list, path, start, goal)
