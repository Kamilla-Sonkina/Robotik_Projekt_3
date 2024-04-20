import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos
from ro45_portalrobot_interfaces.msg import RobotCmd
from std_msgs.msg import Float64
import time

class regelungs_node(Node):
    def __init__(self):
        super().__init__('regelungs_node')

        self.arm_positions_sub = self.create_subscription(RobotPos, 'robot_arm_position', self.arm_position_callback, 10)
        
        self.object_pos_sub_x = self.create_subscription(Float64, 'object_position_x', self.object_position_x_callback, 10)
        self.object_pos_sub_y = self.create_subscription(Float64, 'object_position_y', self.object_position_y_callback, 10)
        self.object_class_sub = self.create_subscription(Float64, 'object_class', self.object_class_callback, 10)
        self.timestamp_object = self.create_subscription(Float64, 'timestamp_object', self.timestamp_object_callback, 10)

        self.vel_sub = self.create_subscription(Float64, 'velocity', self.velocity_callback, 10)


        self.robot_command = self.create_publisher(RobotCmd, 'robot_arm_commands', 10)

       

        
        self.robot_pos = {'x': None, 'y': None, 'z': None}
        self.object_pos = {'x': None, 'y': None}
        self.velocity = None
        self.object_class = None
        self.box_unicorn = [1, 2]
        self.box_cat = [1, 2]
        self.default_pos = [1, 2, 2]
        self.safe_pos = [1, 2, 2]
        self.pick_up_z = 1
        self.transport_z = 2
        


    def arm_position_callback(self, msg):
        self.robot_pos['x'] = msg.pos_x
        self.robot_pos['y'] = msg.pos_y
        self.robot_pos['z'] = msg.pos_z
        self.calculate_target_position()

    def object_position_x_callback(self, msg):
        self.object_pos['x'] = msg.data
        self.calculate_target_position()

    def object_position_y_callback(self, msg):
        self.object_pos['y'] = msg.data
        self.calculate_target_position()    

    def velocity_callback(self, msg):
        self.velocity = msg.data
        self.calculate_target_position()

    def object_class_callback(self, msg):
        self.object_class = msg.object_class
        self.calculate_target_position()
    
    def timestamp_object_callback(self, msg):
        self.timestamp_object = msg.timestamp_object
        

    def calculate_pick_up_position(self, object_timestamp, pick_up_z):
        target_x = None
        target_y = None
        target_z = None
        
        if all(self.robot_pos.values()) and all(self.object_pos.values()) and self.velocity is not None and self.object_class:
            target_x = self.object_pos['x'] + self.velocity * (time.time() - object_timestamp)
            target_y = self.object_pos['y'] 
            target_z = pick_up_z
        
        if target_z is not None:
            self.get_logger().info('Target position: (%f, %f, %f)' % (target_x, target_y, target_z))
        else:
            self.get_logger().warn('Unable to calculate target_z position.')

        if(self.robot_pos['x'] == target_x and self.robot_pos['y'] == target_y):
            target_z = self.pick_up_z
            robot_cmd = RobotCmd()
            robot_cmd.activate_gripper = True

        if(self.robot_pos['z'] == target_z):
            self.sort()

    def sort(self, object_class, box_unicorn, box_cat, transport_z):
        target_z = transport_z
        if(object_class == 'cat'):
            target_x = box_cat[0]
            target_y = box_cat[1]

        elif(object_class == 'unicorn'):
            target_x = box_unicorn[0]
            target_y = box_unicorn[1]
        
        if(self.robot_pos['x'] == target_x and self.robot_pos['y'] == target_y):
            robot_cmd = RobotCmd()
            robot_cmd.activate_gripper = False

        target_x = self.default_pos[0]
        target_y = self.default_pos[1]
        target_z = self.default_pos[2]

            




def main(args=None):
    rclpy.init(args=args)
    node = regelungs_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '_main_':
    main()