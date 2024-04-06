from A import *
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class rosControlNode(Node):

    def __init__(self):
        super().__init__('robot_control')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def robotControl(self, path):

      print('here')
      velocity_message = Twist()

      for item in path:

        self.action = item[2]
        self.lin = self.action[0]/100
        self.ang = self.action[1]
        self.ts =  time.time() #start time
        self.tc = time.time() # current time

        while self.tc - self.ts <= 1.0:
          
          self.tc = time.time()

        # Publish the twist message
        velocity_message.linear.x = 1.6*float(self.lin) # m/s
        velocity_message.angular.z = 1.2*float(self.ang) # rad/s
        self.cmd_vel_pub.publish(velocity_message)
        print('printing  ', self.lin, '  and  ', self.ang, '\n')

        print('finished step', '\n')
      

      self.ts =  time.time() #start time
      self.tc = time.time() # current time

      while self.tc - self.ts <= 1.0:
        
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