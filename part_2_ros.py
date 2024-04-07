from part_2 import *
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class rosControlNode(Node):

    def __init__(self):
        super().__init__('robot_control')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def robotControl(self, path):
      i = 0

      cl = 1.0 # lin gain
      ca = 0.9 # ang gain
      t = 1.55
      velocity_message = Twist()

      for item in path:

        self.coord = item[0]
        self.x = self.coord[0]
        self.action = item[2]
        self.lin = self.action[0]/100
        self.ang = self.action[1]
        self.ts =  time.time() #start time
        self.tc = time.time() # current time

        while self.tc - self.ts <= 3.0:
          
          self.tc = time.time()

        # Publish the twist message
        if self.ang > 0.9:
          self.ang = 0.4
        if self.ang < -0.9:
          self.ang = -0.4

        if self.x > 330:
          self.ang = self.ang * 0.5
          self.lin = self.lin * 0.5

        velocity_message.linear.x = cl*float(self.lin) # m/s
        velocity_message.angular.z = ca*float(self.ang) # rad/s
        self.cmd_vel_pub.publish(velocity_message)
        print('printing  ', self.lin, '  and  ', self.ang, '\n')
        print(self.tc, '\n')
        print('finished step ', i, '\n')
        i = i + 1

        self.ts =  time.time() #start time
        self.tc = time.time()

        while self.tc - self.ts <= t:
        
          self.tc = time.time()
        
        velocity_message.linear.x = 0.0 # m/s
        velocity_message.angular.z = 0.0 # rad/s
        self.cmd_vel_pub.publish(velocity_message)
        print('Pause', '\n')
        

          
      

      # self.ts =  time.time() #start time
      # self.tc = time.time() # current time

      # while self.tc - self.ts <= 1.0:
        
      #   self.tc = time.time()

      # velocity_message.linear.x = 0.0 # m/s
      # velocity_message.angular.z = 0.0 # rad/s
      # self.cmd_vel_pub.publish(velocity_message)
      print('FINISHED', '\n')

def main(args=None):
    rclpy.init(args=args)
    node = rosControlNode()
    node.robotControl(path)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()