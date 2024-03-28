import numpy as np
import matplotlib.pyplot as plt

def newNodes(nodeState, u1, u2): # (node, lower wheel velocity, higher wheel velocity) speeds in RPM
  
  # Extract node information from nodeState value
  node = tuple(nodeState)
  xi = node[0]/2
  yi = node[1]/2
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
      theta = theta + (r/L) * (ur - ul) * dt
      x = x + (r/2) * (ul + ur) * np.cos(theta) * dt
      y = y + (r/2) * (ul + ur) * np.sin(theta) * dt

    v = (r/2) * (ul + ur)
    ang = (r/L) * (ur - ul) # rad/s
    ang = round(30 * ang / np.pi, 2) # rpm
    t = 0
    c2c = np.sqrt((xi - x)**2 + (yi - y)**2) # cost to come is linear displacement, not calculating distance covered
    newNodes.append(((round(x,2), round(y,2), round(theta,2)), round(c2c,2), (v, ang))) # outputs node, cost to come, and associated linear and ang velocity to get to each node (to be sent to robot)

  return newNodes

### The below is for testing the newNode function ###
node = (0, 0, 3)

nodes = newNodes(node, 15, 30) # (node, low speed (rpm), high speed (rpm))

xpoints = np.zeros(8)
ypoints = np.zeros(8)

plt.plot(0,0, 'bo')
#hold()

for i in range (0,8):
  print(nodes[i])
  print('')
  val = nodes[i]
  coord = val[0]
  plt.plot(coord[0], coord[1], 'ro')
  #hold()

plt.grid()
plt.show()

### ------------------------------- ###
'''
# circle check for obstacle

clearance = 440 # mm

h = 4200 # x coord of center of circle obstacle
k = 1200 # y coord of center of circle obstacle
r = 600 # radius of circle obstacle

elif (xnode - h)**2 + (y - k)**2 - (r + clearance)**2 <= 0:
  vibes = True
'''
