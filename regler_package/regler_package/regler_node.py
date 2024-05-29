import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from target_pose_interfaces.msg import TargetPose
from object_interfaces.msg import ObjectData
from std_msgs.msg import Float64
import time
from enum import Enum

class State(Enum):
    Initialisierung = 0
    Idle = 1
    Moving_to_object = 2
    Ready_to_pick_up = 3
    Sorting = 4
    Over_Box = 5
    Default = 6
    Emergency = 7

    

class regelungs_node(Node):
    
    def __init__(self):
        super().__init__('regelungs_node')
        self.state = State.Initialisierung

        self.get_logger().info('Start initializing')

        self.arm_positions_sub = self.create_subscription(RobotPos, 'robot_arm_position', self.arm_position_callback, 10)
        
        self.object_data_sub = self.create_subscription(ObjectData, 'object_data', self.object_data_callback, 10)
        

        self.vel_sub = self.create_subscription(Float64, 'velocity', self.velocity_callback, 10)


        self.robot_command_pub = self.create_publisher(RobotCmd, 'robot_command', 10)

        self.target_pos_sub = self.create_subscription(TargetPose, 'target_position', self.target_position_callback, 10)
        
     
        self.robot_pos = {'x': 0, 'y': 0, 'z': 0}
        self.object_data = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.velocity = 0
        self.zero_position = {'x': 10, 'y': 10, 'z': 10}


        
        self.gripper_is_activated = False
        self.target_position = {'x': 0, 'y': 0, 'z': 0}
        self.corner = {'x': 0, 'y': 0, 'z': 0}


        self.last_calculation_time = time.time()

        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_z = 0
        self.kp = 9.85199  
        self.kd = 6.447857  
        self.first_arm_pos = False
        
        self.user_target = False
        self.velo_zaehler = 0

        self.queue = []

        self.emergency = False
        
        self.box_cat = {'x': 10, 'y': 11, 'z': 12}
        self.box_unicorn = {'x': 12, 'y': 11, 'z': 12}
        self.default_pos = {'x': 1, 'y': 1, 'z': 2}  
        self.safe_pos = {'x': 0, 'y': 1, 'z': 2}
        self.pick_up_z = 13
        self.ready_to_pick_up_z = 12

        self.move_to_zero_position()
        self.target_position['x'] = self.default_pos['x']
        self.target_position['y'] = self.default_pos['y']
        self.target_position['z'] = self.default_pos['z']

        self.box_unicorn =  self.adjust_box_position(self.box_unicorn)
        

        self.box_cat =  self.adjust_box_position(self.box_cat)

        self.default_pos =  self.adjust_box_position(self.default_pos)
        
        self.safe_pos = self.adjust_box_position(self.safe_pos)

        self.pick_up_z = self.zero_position['z'] + 13
        self.ready_to_pick_up_z = self.zero_position['z'] + 12

        self.state = State.Idle
        self.regler()
        
        


      
    def adjust_box_position(self, box):
        box['x'] += self.zero_position['x']
        box['y'] += self.zero_position['y']
        box['z'] += self.zero_position['z']
        return box

    def move_to_zero_position(self):
        robot_cmd = RobotCmd()
        robot_cmd.accel_x = 0.0
        robot_cmd.accel_y = 0.0
        robot_cmd.accel_z = -0.01
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        time.sleep(1)
        robot_cmd.accel_x = 0.0
        robot_cmd.accel_y = -0.01
        robot_cmd.accel_z = 0.0
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        time.sleep(1)
        robot_cmd.accel_x = -0.01
        robot_cmd.accel_y = 0.0
        robot_cmd.accel_z = 0.0
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        time.sleep(1)
        robot_cmd.accel_x = 0.0
        robot_cmd.accel_y = 0.0
        robot_cmd.accel_z = 0.0
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        time.sleep(5)

        self.zero_position['x'] = self.robot_pos['x']
        self.zero_position['y'] = self.robot_pos['y']
        self.zero_position['z'] = self.robot_pos['z'] 

        self.get_logger().info(f"Zero position is: x={self.zero_position['x']}, y={self.zero_position['y']}, z={self.zero_position['z']}")
        
        self.target_position['x'] = self.robot_pos['x']
        self.target_position['y'] = self.robot_pos['y']
        self.target_position['z'] = self.robot_pos['z']
        
        

        

    def target_position_callback(self, msg):
        self.user_target = True
        self.target_position['x'] = msg.target_position_x + self.zero_position['x']
        self.target_position['y'] = msg.target_position_y + self.zero_position['y']
        self.target_position['z'] = msg.target_position_z + self.zero_position['z']
        self.get_logger().info(f"target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
        
        self.go_to_target_position()
        
    


    def arm_position_callback(self, msg):
        self.robot_pos['x'] = msg.pos_x + self.zero_position['x']
        self.robot_pos['y'] = msg.pos_y + self.zero_position['y']
        self.robot_pos['z'] = msg.pos_z + self.zero_position['z']
        self.first_arm_pos = True
        self.regler()
  

    def object_data_callback(self, msg):
        self.user_target = False
        self.object_data['x'] = msg.object_pos_x + self.zero_position['x']
        self.object_data['y'] = msg.object_pos_y + self.zero_position['y']
        self.object_data['class'] = msg.object_class
        self.object_data['timestamp'] = msg.timestamp_value
        self.object_data['index'] = msg.index_value
        self.enqueue(self.object_data)
        
        if(len(self.queue) == 1):
            self.dequeue()
        

        
       

    def velocity_callback(self, msg):
        self.velocity = (self.velocity * self.velo_zaehler + msg.data) / (self.velo_zaehler + 1)
        self.velo_zaehler += 1

    
        

    def calculate_target_position(self):

        self.get_logger().info('Start calculating target position')
        
        if(self.gripper_is_activated is True):
            if(self.oldest_object['class'] == 'cat'):
                self.target_position = self.box_cat
            elif(self.oldest_object['class'] == 'unicorn'):
                self.target_position = self.box_unicorn
            else:
                self.target_position = self.default_pos
        
        elif(self.gripper_is_activated is False) and (self.oldest_object['class'] is not None):
            self.target_position['x'] = self.oldest_object['x'] + self.velocity * (time.time() - self.oldest_object['timestamp'])
            self.target_position['y'] = self.oldest_object['y']
            self.target_position['z'] = self.ready_to_pick_up_z
            self.get_logger().info(f"object is at: x={self.target_position['x']}, y={self.target_position['y']}")
        

        else: 
            if(abs(self.robot_pos - self.default_pos) >= 0.5):
                self.state = State.Default
            else:    
                self.target_position = self.default_pos

        self.go_to_target_position()
       

    def go_to_target_position(self):
        self.get_logger().info('Start going to target position')
       
        if(self.state == State.Moving_to_object or State.Sorting): 
            self.regler()
            time.sleep(1)
            self.go_to_target_position()
            

        if(self.state == State.Over_Box): 
            self.gripper_is_activated = False
            self.regler()
            self.calculate_target_position()
        elif(self.state == State.Default): 
            return None
        else:
            if(self.state == State.Ready_to_pick_up): 
                self.gripper_is_activated = True
                self.target_position['z'] = self.pick_up_z
                self.regler()
                self.sort(self.oldest_object['class'])
            elif(self.user_target == False):
                self.target_position['z'] = self.ready_to_pick_up_z
                self.regler()
                self.state = State.Ready_to_pick_up
                self.go_to_target_position()
            else: return None
       
    def sort(self, oldest_object):
        self.get_logger().info('Start sorting')
        if(self.gripper_is_activated): 
            if(oldest_object['class'] == 'cat'):
                self.target_position = self.box_cat
                self.state = State.Sorting
                self.go_to_target_position()
                self.gripper_is_activated = False
                self.oldest_object = self.dequeue
            if(oldest_object['class'] ==  'unicorn'):
                self.target_position = self.box_unicorn
                self.state = State.Sorting
                self.go_to_target_position()  
                self.gripper_is_activated = False  
                self.oldest_object = self.dequeue
                

    def regler(self):
        self.get_logger().info('Start controlling')
       
        differenz_x = self.target_position['x'] - self.robot_pos['x']    
        differenz_y = self.target_position['y'] - self.robot_pos['y']  
        differenz_z = self.target_position['z'] - self.robot_pos['z']   

        current_time = time.time()
    
        dt = current_time - self.last_calculation_time
      
        self.last_calculation_time = current_time

        
        vel_x = self.compute_pd(differenz_x, self.last_error_x, dt)
        self.last_error_x = differenz_x
        if (vel_x > 0.01):
            vel_x = 0.01
        elif(vel_x < -0.01):
            vel_x = -0.01
        print(vel_x)
        
        vel_y = self.compute_pd(differenz_y, self.last_error_y, dt)
        self.last_error_y = differenz_y
        if (vel_y > 0.01):
            vel_y = 0.01
        elif(vel_y < -0.01):
            vel_y = -0.01
        print(vel_y)
        
        vel_z = self.compute_pd(differenz_z, self.last_error_z, dt)
        self.last_error_z = differenz_z
        if (vel_z > 0.01):
            vel_z = 0.01
        elif (vel_z < -0.01):
            vel_z = -0.01
        print(vel_z)
     
        robot_cmd = RobotCmd()
        robot_cmd.accel_x = vel_x
        robot_cmd.accel_y = vel_y
        robot_cmd.accel_z = vel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        self.get_logger().info('published robot_cmd')
        self.update_State()

        
    def compute_pd(self, error, last_error, dt):
        self.get_logger().info('Start computing pd')
      
        derivative = (error - last_error) / dt
      
        control_signal = self.kp * error + self.kd * derivative

        control_signal = float(control_signal)
        
        return control_signal

    
    def enqueue(self, object_data):
        self.queue.append(object_data)
    
    def dequeue(self):
        if(len(self.queue) != 0):
            self.oldest_object = self.queue.pop(0)
            self.state = State.Moving_to_object
            self.calculate_target_position()
        
    def emergency_case(self, Fehlermeldung):
        self.state = State.Emergency
        self.get_logger().info('Fehler: ' + Fehlermeldung)
        self.target_position = self.safe_pos
        self.emergency = True
        while(self.emergency):
            self.regler()

    def update_State(self):
        if(((self.robot_pos['x'] == self.box_cat['x']) and (self.robot_pos['y'] == self.box_cat['y']))
        or ((self.robot_pos['x'] == self.box_unicorn['x']) and (self.robot_pos['y'] == self.box_unicorn['y']))):
            self.state = State.Over_Box
        elif(((self.target_position['x'] == self.box_cat['x']) and (self.target_position['y'] == self.box_cat['y']))
        or ((self.target_position['x'] == self.box_unicorn['x']) and (self.target_position['y'] == self.box_unicorn['y']))):
            self.state = State.Sorting
        if((self.robot_pos['x'] == self.default_pos['x']) and (self.robot_pos['y'] == self.default_pos['y'])):
            self.state = State.Default
        elif(((self.robot_pos['x'] == self.target_position['x']) and (self.robot_pos['y'] == self.target_position['y']))
             and self.robot_pos['z'] == self.ready_to_pick_up_z):
            self.state = State.Ready_to_pick_up
        if((self.target_position['x'] == self.robot_pos['x']) 
        and (self.target_position['y'] == self.robot_pos['y']) 
        and (self.target_position['z'] == self.robot_pos['z'])
        and self.gripper_is_activated == False):
            self.state = State.Moving_to_object
        self.get_logger().info(f'Updated state to {self.state.name}')   
        



def main(args=None):
    rclpy.init(args=args)
    node = regelungs_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



