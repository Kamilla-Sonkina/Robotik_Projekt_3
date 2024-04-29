import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from std_msgs.msg import Float64, String
import time

class regelungs_node(Node):
    def __init__(self):
        super().__init__('regelungs_node')

        self.arm_positions_sub = self.create_subscription(RobotPos, 'robot_arm_position', self.arm_position_callback, 10)
        
        self.object_pos_sub_x = self.create_subscription(Float64, 'object_position_x', self.object_position_x_callback, 10)
        self.object_pos_sub_y = self.create_subscription(Float64, 'object_position_y', self.object_position_y_callback, 10)
        self.object_class_sub = self.create_subscription(String, 'object_class', self.object_class_callback, 10)
        self.timestamp_object = self.create_subscription(Float64, 'timestamp_object', self.timestamp_object_callback, 10)

        self.vel_sub = self.create_subscription(Float64, 'velocity', self.velocity_callback, 10)


        self.robot_command_pub = self.create_publisher(RobotCmd, 'robot_arm_commands', 10)

     
        self.robot_pos = {'x': None, 'y': None, 'z': None}
        self.zero_position = {'x': None, 'y': None, 'z': None}
        self.object_pos = {'x': None, 'y': None}
        self.velocity = None
        self.object_class = None

        self.box_unicorn =  {'x': 10, 'y': 11, 'z': 12}
        self.box_unicorn['x'] += self.zero_position['x']
        self.box_unicorn['y'] += self.zero_position['y']
        self.box_unicorn['z'] += self.zero_position['z']

        self.box_cat =  {'x': 10, 'y': 11, 'z': 12}
        self.box_cat['x'] += self.zero_position['x']
        self.box_cat['y'] += self.zero_position['y']
        self.box_cat['z'] += self.zero_position['z']

        self.default_pos =  {'x': 10, 'y': 11, 'z': 12}
        self.default_pos['x'] += self.zero_position['x']
        self.default_pos['y'] += self.zero_position['y']
        self.default_pos['z'] += self.zero_position['z']
        
        self.safe_pos =  {'x': 10, 'y': 11, 'z': 12}
        self.safe_pos['x'] += self.zero_position['x']
        self.safe_pos['y'] += self.zero_position['y']
        self.safe_pos['z'] += self.zero_position['z']

        self.pick_up_z = 13
        self.gripper_is_activated = False
        self.target_position = {'x': None, 'y': None, 'z': None}


        self.last_calculation_time = time.time()

        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_z = 0
        self.kp = 9.85199  
        self.kd = 6.447857  
        self.kn = 50

        self.velo_zaehler = 0


        while(abs(self.robot_pos['x'] - self.zero_position['x']) >= 0.1):
            self.robot_pos['x'] = self.zero_position['x']
            robot_cmd = RobotCmd()
            robot_cmd.accel_x = 0.1
            self.robot_command_pub.publish(robot_cmd)
            time.sleep(1)

        while(abs(self.robot_pos['y'] - self.zero_position['y']) >= 0.1):
            self.robot_pos['y'] = self.zero_position['y']
            robot_cmd = RobotCmd()
            robot_cmd.accel_y = -0.1
            self.robot_command_pub.publish(robot_cmd)
            time.sleep(1)

        while(abs(self.robot_pos['z'] - self.zero_position['z']) >= 0.1):
            self.robot_pos['z'] = self.zero_position['z']
            robot_cmd = RobotCmd()
            robot_cmd.accel_z = -0.1
            self.robot_command_pub.publish(robot_cmd)
            time.sleep(1)
        
        robot_cmd = RobotCmd()
        robot_cmd.activate_gripper = False
        self.robot_command_pub.publish(robot_cmd)

        self.target_position['x'] = self.default_pos['x']
        self.target_position['y'] = self.default_pos['y']
        self.target_position['z'] = self.default_pos['z']
        self.regler()
          




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
        if(self.velo_zaehler != 0):
            self.velocity = (self.velocity/self.velo_zaehler + msg.data/self.velo_zaehler) / self.velo_zaehler + 1
        else: 
            self.velocity = msg.data
        self.velo_zaehler = self.velo_zaehler + 1

    def object_class_callback(self, msg):
        self.object_class = msg.data
        self.calculate_target_position(self.object_class, self.object_pos, self.timestamp_object)
        self.go_to_target_position(self.object_class)
    
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
        
        if(object_pos is not None and timestamp_object is not None):
            self.target_position['x'] = object_pos['x'] + self.velocity * (time.time() - timestamp_object)
            self.target_position['y'] = object_pos['y']
            self.target_position['z'] = self.pick_up_z


        else: 
            self.target_position = self.default_pos
        
        return self.target_position
       

    async def go_to_target_position(self, object_class):
       
        while((abs(self.target_position - self.robot_pos)) >= 0.5):
            self.regler()
            self.go_to_target_position()

        if(abs(self.target_position - self.box_cat) >= 0.5 or abs(self.target_position - self.box_unicorn) >= 0.5):
            self.gripper_is_activated = False
            self.regler()
            self.go_to_target_position(self.default_pos)
        elif(abs(self.target_position - self.default_pos) >= 0.5):
            await(5)
        else:
            if(abs(self.robot_pos['z'] - self.pick_up_z) >= 0.5):
                self.gripper_is_activated = True
                self.regler()
                self.sort(object_class)
            else:
                self.robot_pos['z'] = self.pick_up_z
                self.regler()
                self.calculate_target_position()
       
    def sort(self, object_class):
        if(self.gripper_is_activated):
            if(object_class == 'cat'):
                self.go_to_target_position(self.box_cat)
                self.gripper_is_activated = False
                self.regler()
            if(object_class ==  'unicorn'):
                self.go_to_target_position(self.box_unicorn)  
                self.gripper_is_activated = False  
                self.regler()

    def regler(self):
       
        differenz_x = self.target_position['x'] - self.robot_pos['x']    
        differenz_y = self.target_position['y'] - self.robot_pos['y']  
        differenz_z = self.target_position['z'] - self.robot_pos['z']   

        current_time = time.time()
    
        dt = current_time - self.last_calculation_time
      
        self.last_calculation_time = current_time


        vel_x = self.compute_pd(differenz_x, self.last_error_x, dt, self.kn)
        self.last_error_x = differenz_x

      
        vel_y = self.compute_pd(differenz_y, self.last_error_y, dt, self.kn)
        self.last_error_y = differenz_y

        vel_z = self.compute_pd(differenz_z, self.last_error_z, dt, self.kn)
        self.last_error_z = differenz_z

     
        robot_cmd = RobotCmd()
        robot_cmd.accel_x = vel_x
        robot_cmd.accel_y = vel_y
        robot_cmd.accel_z = vel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        
    def compute_pd(self, error, last_error, dt):
        # Compute derivative term with low-pass filtering
        derivative = (error - last_error) / dt
        # Compute control signal with filtering
        control_signal = self.kp * error + self.kd * (self.kn / (1 + self.kn * (1 / dt))) * derivative
        return control_signal, error

    



def main(args=None):
    rclpy.init(args=args)
    node = regelungs_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
