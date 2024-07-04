import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from std_msgs.msg import String, Bool
import time


class watchdog(Node):
    def __init__(self):
        super().__init__('watchdog')
        self.arm_pos_x = 0
        self.arm_pos_y = 0
        self.arm_pos_z = 0
        self.arm_last_pos_x = 0
        self.arm_last_pos_y = 0
        self.arm_last_pos_z = 0
        self.u_x = 0
        self.u_y = 0
        self.u_z = 0
        self.last_u_x = 0
        self.last_u_y = 0
        self.last_u_z = 0
        self.counter_x = 0
        self.counter_y = 0
        self.counter_z = 0
        self.callback_period = 5
        self.last_time = 0
        self.current_time = time.time()
        self.regler_node_ready = False
        self.robot_command_sub = self.create_subscription(RobotCmd, 'robot_command', self.command_callback, 10)
        self.arm_positions_sub = self.create_subscription(RobotPos, 'robot_position', self.arm_position_callback, 5)
        self.init_bool_sub = self.create_subscription(Bool, 'init_bool', self.init_bool_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.emergency_pub = self.create_publisher(String, 'emergency',  5)
        self.get_logger().info('initializing finished')

    def timer_callback(self):
        self.check_time()   

    def arm_position_callback(self, msg):
        
        self.last_time = time.time()
        self.arm_pos_x = msg.pos_x
        self.arm_pos_y = msg.pos_y
        self.arm_pos_z = msg.pos_z

    def init_bool_callback(self, msg):
        if msg.data:
            self.regler_node_ready = True
            self.get_logger().info('Zero position has been set.')  

    def command_callback(self, msg):
        self.u_x = msg.accel_x
        self.u_y = msg.accel_y
        self.u_z = msg.accel_z
        if self.regler_node_ready:
            self.check_positions_difference()
            self.check_time()
        self.get_logger().debug(f'robot commands recieved u_x: {self.u_x}, u_y: {self.u_y}, u_z: {self.u_z}')
    

    def check_time(self):
        self.current_time = time.time()
        if self.regler_node_ready:
            if self.current_time - self.last_time > self.callback_period:
                self.publish_emergency(f'no position callbacks in the last {self.callback_period} seconds \n check if a restart is necessary')
            else: self.get_logger().debug('all good')


    def check_positions_difference(self):
        warnings = []

        if abs(self.u_x) > 0.001:
            if self.arm_last_pos_x == self.arm_pos_x:
                self.counter_x += 1
                if(self.counter_x > 5):
                    warnings.append('motor x axe too slow')
            else:
                self.counter_x -= 1    

        if abs(self.u_y) > 0.001:
            if self.arm_last_pos_y == self.arm_pos_y:
                self.counter_y += 1
                if(self.counter_y > 5):
                    warnings.append('motor y axe too slow')
            else:
                self.counter_y -= 1 

        if abs(self.u_z) > 0.001:
            if self.arm_last_pos_z == self.arm_pos_z:
                self.counter_z += 1
                if(self.counter_z > 5):
                    warnings.append('motor z axe too slow')
            else:
                self.counter_z -= 1 
        if warnings:
            emergency_message = '\n'.join(warnings)
            emergency_message += '\ncheck if a restart is necessary'
            self.publish_emergency(emergency_message)
        self.arm_last_pos_x = self.arm_pos_x
        self.arm_last_pos_y = self.arm_pos_y
        self.arm_last_pos_z = self.arm_pos_z

    
    def publish_emergency(self, message):
        msg = String()
        msg.data = message
        self.emergency_pub.publish(msg)
        self.get_logger().error(message)

def main(args=None):
        rclpy.init(args=args)
        node = watchdog()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
