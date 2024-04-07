from A_star import * # This imports the path data from the A_Star.py file
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class rosControlNode(Node):

    # Initialize Publisher
    def __init__(self):
        super().__init__('robot_control')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # Send commands to robot

    # We think there may be a conversion difference between truw time using the time.time() function compaored to the Gazebo simulation time that was
    # introducing an error to the commands. TO fix this we had to add some gains in places where the error accrued significantly
    
    def robotControl(self, path):
      i = 0
      # Gains for linear and angular velocity and step time used throughout the code
      cl = 0.995 # lin gain
      ca = 0.9 # ang gain
      t = 1.55
      velocity_message = Twist()

      for item in path:
        # Extract position, linear velocity and angulaw velocity from the path
        self.coord = item[0]
        self.x = self.coord[0] # x-coordinate of robot for positional awareness
        self.action = item[2]
        self.lin = self.action[0]/100 # Linear velocity at each node converted to meters/s from centimeters/s
        self.ang = self.action[1] # angular velocity at each node in rad/s
        self.ts =  time.time() #start time
        self.tc = time.time() # current time

        # due to error we had to limit some aggresive turns
        if self.ang > 0.9:
          self.ang = 0.4
        if self.ang < -0.9:
          self.ang = -0.4

        # Add gains when navigating the circle because accrued error was especially large there
        if self.x > 340 and self.x < 400:
          self.ang = self.ang * 0.7
          self.lin = self.lin * 0.275

        # Gains for after circle
        if self.x > 400:
          self.lin = self.lin * 0.925

        velocity_message.linear.x = cl*float(self.lin) # m/s
        velocity_message.angular.z = ca*float(self.ang) # rad/s
        self.cmd_vel_pub.publish(velocity_message)

        # Print functions showing linear and angular velocity at each node, as well as a time stamp and step number
        print('printing  ', self.lin, '  and  ', self.ang, '\n')
        print(self.tc, '\n')
        print('finished step ', i, '\n')
        i = i + 1

        self.ts =  time.time() #start time
        self.tc = time.time()

        # Stop the robot at the end
        while self.tc - self.ts <= t:
        
          self.tc = time.time()
        
        velocity_message.linear.x = 0.0 # m/s
        velocity_message.angular.z = 0.0 # rad/s
        self.cmd_vel_pub.publish(velocity_message)
        
      print('FINISHED', '\n')

def main(args=None):
    rclpy.init(args=args)
    node = rosControlNode()
    node.robotControl(path)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
