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


        self.robot_command_pub = self.create_publisher(RobotCmd, 'robot_arm_commands', 10)

        
        self.robot_pos = {'x': None, 'y': None, 'z': None}
        self.object_pos = {'x': None, 'y': None}
        self.velocity = None
        self.object_class = None
        self.box_unicorn =  {'x': 1, 'y': 2, 'z': 3}
        self.box_cat =  {'x': 4, 'y': 5, 'z': 6}
        self.default_pos =  {'x': 7, 'y': 8, 'z': 9}
        self.safe_pos =  {'x': 10, 'y': 11, 'z': 12}
        self.pick_up_z = 13
        self.gripper_is_activated = False
        self.target_position = {'x': None, 'y': None, 'z': None}
        



    def arm_position_callback(self, msg):
        self.robot_pos['x'] = msg.pos_x
        self.robot_pos['y'] = msg.pos_y
        self.robot_pos['z'] = msg.pos_z
        self.regler()

    def object_position_x_callback(self, msg):
        self.object_pos['x'] = msg.data
       
    def object_position_y_callback(self, msg):
        self.object_pos['y'] = msg.data
          

    def velocity_callback(self, msg):
        self.velocity = msg.data
        

    def object_class_callback(self, msg):
        self.object_class = msg.data
        self.calculate_target_position()
        self.go_to_target_position
    
    def timestamp_object_callback(self, msg):
        self.timestamp_object = msg.data
        

    def calculate_target_position(self, object_class, object_pos, timestamp_object):
        
        if(self.gripper_is_activated is True):
            if(object_class == 'cat'):
                self.target_position = self.box_cat
            elif(object_class == 'unicorn'):
                self.target_position = self.box_unicorn
            else:
                self.target_position = self.default_pos
        
        if((object_class and object_pos and timestamp_object) is not None):
            self.target_position['x'] = object_pos['x'] + self.velocity * (time.time() - timestamp_object)
            self.target_position['y'] = object_pos['y']
            self.target_position['z'] = self.pick_up_z


        else: 
            self.target_position = self.default_pos
        
        return self.target_position
       

    def go_to_target_position(self, object_class):
       
        while(self.target_position is not self.robot_pos):
            self.wait(0.1)

        if(self.target_position == self.box_cat or self.target_position == self.box_unicorn):
            self.robot_command_pub.activate_gripper = False
            self.gripper_is_activated = False
            self.go_to_target_position(self.default_pos)
        else:
            self.robot_pos.activate_gripper = True
            self.gripper_is_activated = True
            self.sort(object_class)
       
    def sort(self, object_class):
        if(self.gripper_is_activated):
            if(object_class == 'cat'):
                self.go_to_target_position(self.box_cat)
                self.robot_command_pub.activate_gripper = False
                self.gripper_is_activated = False
            if(object_class ==  'unicorn'):
                self.go_to_target_position(self.box_unicorn)  
                self.robot_command_pub.activate_gripper = False
                self.gripper_is_activated = False  

    def regler(self):
        #Platzhalter
        differenz_x = self.target_position['x'] - self.robot_pos['x']    
        differenz_y = self.target_position['y'] - self.robot_pos['y']  
        differenz_z = self.target_position['z'] - self.robot_pos['z']     

        robot_cmd = RobotCmd()
        robot_cmd.vel_x = differenz_x 
        robot_cmd.vel_y = differenz_y
        robot_cmd.vel_z = differenz_z 
        self.robot_command_pub.publish(robot_cmd)

    



def main(args=None):
    rclpy.init(args=args)
    node = regelungs_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



