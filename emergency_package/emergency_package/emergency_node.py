import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from std_msgs.msg import String
import time


class emergency_node(Node):
    def __init__(self):
        super().__init__('emergency_node')
        self.u_x = 0
        self.u_y = 0
        self.u_z = 0
        self.last_u_x = 0
        self.last_u_y = 0
        self.last_u_z = 0
        self.callback_period = 5
        self.last_time = time.time()
        self.current_time = time.time()
        self.robot_command_sub = self.create_subscription(RobotCmd, 'robot_command', self.command_callback, 10)
        self.emergency_pub = self.create_publisher(String, 'emergency',  5)
        self.get_logger().info('initializing finished')

    
    
    def command_callback(self, msg):
        self.last_u_x = self.u_x
        self.last_u_y = self.u_y
        self.last_u_z = self.u_z
        self.u_x = msg.accel_x
        self.u_y = msg.accel_y
        self.u_z = msg.accel_z
        self.check_positions_difference()
        self.check_time()
        self.get_logger().debug(f'robot commands recieved u_x: {self.u_x}, u_y: {self.u_y}, u_z: {self.u_z}')
    

    def check_time(self):
        self.current_time = time.time()
        if self.current_time - self.last_time > self.callback_period:
            self.publish_emergency('no position callbacks in the last 5 seconds')
        else: self.get_logger().debug('all good')


    def check_positions_difference(self):
        if abs(self.u_x) > 0.01:
            if self.last_u_x == self.u_x:
                self.publish_emergency('Motor in x Richtung zu langsam')
        if abs(self.u_y) > 0.01:
            if self.last_u_y == self.u_y:
                self.publish_emergency('Motor in y Richtung zu langsam')
        if abs(self.u_z) > 0.01:
            if self.last_u_z == self.u_z:
                self.publish_emergency('Motor in z Richtung zu langsam')
            
    
    def publish_emergency(self, message):
        msg = String()
        msg.data = message
        self.emergency_pub.publish(msg)
        self.get_logger().error(message)

def main(args=None):
        rclpy.init(args=args)
        node = emergency_node()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
