"""
import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from target_pose_interfaces.msg import TargetPose
from object_interfaces.msg import ObjectData
from std_msgs.msg import Float64
import time
from enum import Enum
from builtin_interfaces.msg import Time

class State(Enum):
    Initialisierung = 0
    Idle = 1
    Moving_to_object = 2
    Ready_to_pick_up = 3
    Sorting = 4
    Over_Box = 5
    Default = 6
    Emergency = 7
    Picked_up = 8 

    

class regelungs_node(Node):
    
    def __init__(self):
        super().__init__('regelungs_node')
        self.state = State.Initialisierung

        
        self.get_logger().info('Start initializing')
        self.object_data_sub = self.create_subscription(ObjectData, 'object_data', self.object_data_callback, 10)
        self.target_pos_sub = self.create_subscription(TargetPose, 'target_position', self.target_position_callback, 10)
        self.vel_sub = self.create_subscription(Float64, 'velocity', self.velocity_callback, 10)
        self.arm_positions_sub = self.create_subscription(RobotPos, 'robot_position', self.arm_position_callback, 10)
        
        self.robot_command_pub = self.create_publisher(RobotCmd, 'robot_command', 10)
        self.robot_pos = {'x': 0, 'y': 0, 'z': 0}
        self.object_data = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.velocity = 0
        self.zero_position = {'x': 10, 'y': 10, 'z': 10}
        self.gripper_is_activated = False
        self.target_position = {'x': None, 'y': None, 'z': None}
        self.corner = {'x': 0, 'y': 0, 'z': 0}
        self.last_calculation_time = time.time_ns()

        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_z = 0
        #self.kp = 9.85199 
        #self.kd_x = 6.447857
        self.kp_x = 6.0
        self.kd_x = 5.7 
        self.kp_y = 3 
        self.kd_y = 8
        self.kp_z = 3 
        self.kd_z = 8
        self.first_arm_pos = 0
        
        self.user_target = False
        self.velo_zaehler = 0

        self.queue = []

        self.emergency = False
        
        self.box_cat = {'x': 0.0854, 'y': 0.0, 'z': 0.0562} #box 1
        self.box_unicorn = {'x': 0.0012, 'y': 0.0, 'z': 0.0562} #box 2
        self.default_pos = {'x': 0.0854, 'y': 0.0556, 'z': 0.0562}  
        self.safe_pos = {'x': 0, 'y': 0, 'z': 0}
        self.pick_up_z = 0.0791  #0.0068
        self.ready_to_pick_up_z = 0.0714 #0.0067
        self.transport_z= 0.0562
        self.last_msg_time = time.time()
        self.move_to_zero_position()
        time.sleep(15)
        
        self.controlling_tolerance = 0.005
        self.safe_mode = False
        self.current_time = 1

        self.state = State.Idle
        #self.regler()
        self.get_logger().info('initializing finished')

        self.controll_u_x = 0
        self.controll_u_y = 0
        self.controll_u_z = 0
        


      
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
        self.controll_u_x = robot_cmd.accel_x
        self.controll_u_y = robot_cmd.accel_y
        self.controll_u_z = robot_cmd.accel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        time.sleep(1)
        robot_cmd.accel_x = 0.0
        robot_cmd.accel_y = -0.01
        robot_cmd.accel_z = 0.0
        self.controll_u_x = robot_cmd.accel_x
        self.controll_u_y = robot_cmd.accel_y
        self.controll_u_z = robot_cmd.accel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        time.sleep(1)
        robot_cmd.accel_x = -0.01
        robot_cmd.accel_y = 0.0
        robot_cmd.accel_z = 0.0
        self.controll_u_x = robot_cmd.accel_x
        self.controll_u_y = robot_cmd.accel_y
        self.controll_u_z = robot_cmd.accel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        time.sleep(1)
        robot_cmd.accel_x = 0.0
        robot_cmd.accel_y = 0.0
        robot_cmd.accel_z = 0.0
        self.controll_u_x = robot_cmd.accel_x
        self.controll_u_y = robot_cmd.accel_y
        self.controll_u_z = robot_cmd.accel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        

        
        
        
        
        

        

    def target_position_callback(self, msg):
        self.user_target = True
        self.target_position['x'] = msg.target_position_x + self.zero_position['x']
        self.target_position['y'] = msg.target_position_y + self.zero_position['y']
        self.target_position['z'] = msg.target_position_z + self.zero_position['z']
        self.get_logger().info(f"target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
        
        self.go_to_target_position()
        
    


    def arm_position_callback(self, msg):
        self.last_calculation_time = self.current_time
        self.current_time = time.time_ns() / 10e9 #self.get_clock().now().to_msg().sec
        print(self.current_time)
        print(self.last_calculation_time)
        self.first_arm_pos += 1
        if (self.first_arm_pos == 1):
            self.robot_pos['x'] = round(-msg.pos_x, 3)
            self.robot_pos['y'] = round(msg.pos_y, 3)
            self.robot_pos['z'] = round(msg.pos_z, 3)
            self.zero_position['x'] = self.robot_pos['x']
            self.zero_position['y'] = self.robot_pos['y']
            self.zero_position['z'] = self.robot_pos['z']
            self.box_unicorn =  self.adjust_box_position(self.box_unicorn)
            self.box_cat =  self.adjust_box_position(self.box_cat)
            self.default_pos =  self.adjust_box_position(self.default_pos)
            self.safe_pos = self.adjust_box_position(self.safe_pos)
            self.pick_up_z = self.zero_position['z'] + self.pick_up_z
            self.ready_to_pick_up_z = self.zero_position['z'] + self.ready_to_pick_up_z
            self.get_logger().info(f"zero position is: x={self.zero_position['x']}, y={self.zero_position['y']}, z={self.zero_position['z']}")
            
        
        else:
            self.robot_pos['x'] = round((-msg.pos_x + self.zero_position['x']), 3)
            self.robot_pos['y'] = round((msg.pos_y + self.zero_position['y']), 3)
            self.robot_pos['z'] = round((msg.pos_z + self.zero_position['z']), 3)
        
        #if current_time - self.last_msg_time >= 1.0:
        self.get_logger().info(f"robot position is: x={self.robot_pos['x']}, y={self.robot_pos['y']}, z={self.robot_pos['z']}")
        self.last_msg_time = self.current_time
        self.calculate_target_position()
        

    def object_data_callback(self, msg):
        self.user_target = False
        self.object_data['x'] = msg.object_pos_x + self.zero_position['x']
        self.object_data['y'] = msg.object_pos_y + self.zero_position['y']
        self.object_data['class'] = msg.object_class
        self.object_data['timestamp'] = msg.timestamp_value
        self.object_data['index'] = msg.index_value
        if not any(obj['index'] == self.object_data['index'] for obj in self.queue):
            self.enqueue(self.object_data)
        
        if(len(self.queue) == 1):
            self.dequeue()
        

        
       

    def velocity_callback(self, msg):
        self.velocity = (self.velocity * self.velo_zaehler + msg.data) / (self.velo_zaehler + 1)
        self.velo_zaehler += 1

    
        

    def calculate_target_position(self):

        #self.get_logger().info('Start calculating target position')
        if(self.state == State.Idle):
            return
        if(self.gripper_is_activated is True):
            if(self.oldest_object['class'] == 'cat' or 'unicorn'):
                self.sort(self.oldest_object)
            else:
                self.target_position = self.default_pos
        
        elif(self.gripper_is_activated is False) and (self.oldest_object['class'] is not None):
            self.target_position['x'] = self.oldest_object['x'] + self.velocity * (time.time() - self.oldest_object['timestamp'])
            self.target_position['y'] = self.oldest_object['y']
            self.target_position['z'] = self.ready_to_pick_up_z
            self.get_logger().info(f"object is at: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
            self.go_to_target_position()
        elif(self.user_target == True):
            self.get_logger().info(f"target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
            self.go_to_target_position()

        else: 
            self.update_State()
            if(self.state == State.Default):
                return
            else:    
                self.target_position = self.default_pos
        self.get_logger().info(f"target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
        self.go_to_target_position()
       

    def go_to_target_position(self):
        self.get_logger().info(f"going from robot position x:{self.robot_pos['x']}, y: {self.robot_pos['y']}, z: {self.robot_pos['z']}")
        self.get_logger().info(f"Start going to target position x:{self.target_position['x']}, y: {self.target_position['y']}, z: {self.target_position['z']}")
       
        if(self.state == State.Moving_to_object or State.Sorting): 
            self.regler()
            return
            #self.go_to_target_position()
            

        if(self.state == State.Over_Box): 
            self.gripper_is_activated = False
            self.regler()
            
        elif(self.state == State.Default): 
            return
        else:
            if(self.state == State.Ready_to_pick_up): 
                print('picking up')
                
                self.target_position['z'] = self.pick_up_z
                self.regler()
                if(self.gripper_is_activated == True):
                    self.sort(self.oldest_object)
            elif(self.user_target == False):
                self.target_position['z'] = self.ready_to_pick_up_z
                self.state = State.Moving_to_object
                self.go_to_target_position()
            else: 
                self.state = State.Moving_to_object
                self.regler()
       
    def sort(self, oldest_object):
        self.get_logger().info('Start sorting')
        self.target_position['x'] = self.target_position['x']
        self.target_position['y'] = self.target_position['y']
        self.target_position['z'] = self.transport_z
        
        if self.gripper_is_activated: 
            if oldest_object['class'] == 'cat':
                self.target_position = self.box_cat
                self.state = State.Sorting
                self.gripper_is_activated = False
                robot_cmd = RobotCmd()
                robot_cmd.activate_gripper = self.gripper_is_activated
                self.robot_command_pub.publish(robot_cmd)
                self.oldest_object = self.dequeue()
            elif oldest_object['class'] == 'unicorn':
                self.target_position = self.box_unicorn
                self.state = State.Sorting
                self.gripper_is_activated = False 
                robot_cmd = RobotCmd()
                robot_cmd.activate_gripper = self.gripper_is_activated
                self.robot_command_pub.publish(robot_cmd) 
                self.oldest_object = self.dequeue()
            else: 
                self.target_position = self.default_pos
                

    def regler(self):
        #self.get_logger().info('Start controlling')
        print(self.current_time)
        print(self.last_calculation_time)
       
        differenz_x = self.target_position['x'] - self.robot_pos['x']    
        differenz_y = self.target_position['y'] - self.robot_pos['y']  
        differenz_z = self.target_position['z'] - self.robot_pos['z']   
        print(differenz_x)
        #print(differenz_y)
        #print(differenz_z)

        
    
        dt = (self.current_time - self.last_calculation_time) 
      
        

        
        u_x = self.compute_pd(differenz_x, self.last_error_x, dt, self.kp_x, self.kd_x)
        self.last_error_x = differenz_x
        self.controll_u_x = u_x
        #print(u_x)
        #self.get_logger().info(f"differenz x:{differenz_x}, u_x: {u_x}, last error {self.last_error_x}, kd: {self.kd_x}, kp: {self.kp_x}")
        #self.get_logger().info(f"robot x:{self.robot_pos['x']},")
        

        u_y = self.compute_pd(differenz_y, self.last_error_y, dt, self.kp_y, self.kd_y)
        self.last_error_y = differenz_y
        self.controll_u_y = u_y
        #print(u_y)
        
        u_z = self.compute_pd(differenz_z, self.last_error_z, dt, self.kp_z, self.kd_z)
        self.last_error_z = differenz_z
        self.controll_u_z = u_z
        #print(u_z)
     
        robot_cmd = RobotCmd()
        robot_cmd.accel_x = u_x
        robot_cmd.accel_y = u_y
        robot_cmd.accel_z = u_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        self.get_logger().info('published robot_cmd')
        self.update_State()


        
    def compute_pd(self, error, last_error, dt, kp,kd):
        
        
        derivative = (error - last_error) / dt
        
        control_signal = kp * error + kd * derivative * error
        
        control_signal = float(control_signal)
        
        if (self.safe_mode or self.emergency):
            control_signal = 0.1 * control_signal
        
       
        control_signal = max(min(control_signal, 0.3), -0.3)
        
        return control_signal


    
    def enqueue(self, object_data):
        self.queue.append(object_data)
    
    def dequeue(self):
        if(len(self.queue) != 0):
            self.oldest_object = self.queue.pop(0)
            self.state = State.Moving_to_object
            self.calculate_target_position()
        else: self.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}

    def emergency_case(self, Fehlermeldung):
        self.state = State.Emergency
        self.get_logger().info('Fehler: ' + Fehlermeldung)
        self.move_to_zero_position()
        self.emergency = True
        self.safe_mode = True
        
        
            

    def update_State(self):
        self.get_logger().info('updating state')
        if(self.emergency == True):
            self.state = State.Emergency
            self.get_logger().info(f'Updated state to {self.state.name}')  
            return
        elif(((abs(self.robot_pos['x'] - self.box_cat['x'])< self.controlling_tolerance) 
            and (abs(self.robot_pos['y'] - self.box_cat['y'])< self.controlling_tolerance))
        or ((abs(self.robot_pos['x'] - self.box_unicorn['x'])< self.controlling_tolerance) 
            and (abs(self.robot_pos['y'] - self.box_unicorn['y']))< self.controlling_tolerance)
            and self.robot_pos['x'] != 0 and self.user_target == False):
            self.state = State.Over_Box

            self.gripper_is_activated = False
            self.get_logger().info(f'Updated state to {self.state.name}')  
             
            robot_cmd = RobotCmd()
            robot_cmd.activate_gripper = self.gripper_is_activated
            self.robot_command_pub.publish(robot_cmd)
            return
        elif(self.gripper_is_activated == True 
             and ((abs(self.box_cat['x'] - self.target_position['x']) < self.controlling_tolerance)
             and (abs(self.box_cat['y'] - self.target_position['y']) < self.controlling_tolerance) )
             or ((abs(self.box_unicorn['x'] - self.target_position['x']) < self.controlling_tolerance)
             and (abs(self.box_unicorn['y'] - self.target_position['y']) < self.controlling_tolerance))
             and self.user_target == False):
            self.state = State.Sorting
            
            self.get_logger().info(f'Updated state to {self.state.name}')  
            return
        
        elif(abs((self.target_position['x'] - self.robot_pos['x']) < self.controlling_tolerance)
            and (abs(self.target_position['y'] - self.robot_pos['y']) < self.controlling_tolerance)
            and (abs(self.ready_to_pick_up_z - self.robot_pos['z']) < self.controlling_tolerance) 
            and self.gripper_is_activated == False
            and self.user_target == False):
            self.gripper_is_activated = True
            self.state = State.Ready_to_pick_up
            self.get_logger().info(f'Updated state to {self.state.name}')  
            robot_cmd = RobotCmd()
            robot_cmd.activate_gripper = self.gripper_is_activated
            self.robot_command_pub.publish(robot_cmd)
            return
        elif(abs((self.target_position['x'] - self.robot_pos['x']) < self.controlling_tolerance)
            and (abs(self.target_position['y'] - self.robot_pos['y']) < self.controlling_tolerance)
            and (abs(self.pick_up_z - self.robot_pos['z']) < self.controlling_tolerance) 
            and self.gripper_is_activated == True
            and self.user_target == False):
            self.state = State.Picked_up
            self.get_logger().info(f'Updated state to {self.state.name}') 
            self.sort()
        
        elif(abs((self.target_position['x'] - self.robot_pos['x']) >= self.controlling_tolerance)
            or (abs(self.target_position['y'] - self.robot_pos['y']) >= self.controlling_tolerance)
            or (abs(self.target_position['z'] - self.robot_pos['z'])>= self.controlling_tolerance)
            and (self.gripper_is_activated == False or self.user_target == True)):
            self.state = State.Moving_to_object
            self.get_logger().info(f'Updated state to {self.state.name}')  
        elif(abs((self.target_position['x'] - self.robot_pos['x']) < self.controlling_tolerance)
            and (abs(self.target_position['y'] - self.robot_pos['y']) < self.controlling_tolerance)
            and (abs(self.target_position['z'] - self.robot_pos['z']) < self.controlling_tolerance)
            and (self.oldest_object['index'] is None or self.user_target == False)):
            self.state = State.Idle
            self.get_logger().info(f'Updated state to {self.state.name}')  
            return
        elif((abs(self.robot_pos['x'] - self.default_pos['x']) < self.controlling_tolerance)
             and (abs(self.robot_pos['y'] - self.default_pos['y']) < self.controlling_tolerance)
             and (abs(self.robot_pos['z'] - self.default_pos['z']) < self.controlling_tolerance)):
            self.state = State.Default
            self.get_logger().info(f'Updated state to {self.state.name}')  
            return
        
            
        
            
        



def main(args=None):
    rclpy.init(args=args)
    node = regelungs_node()
    rclpy.spin(node)
    rclpy.shutdown()
                
if __name__ == '__main__':
    main()


"""

import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from target_pose_interfaces.msg import TargetPose
from object_interfaces.msg import ObjectData
from std_msgs.msg import Float64
import time
from enum import Enum
from builtin_interfaces.msg import Time
from statemachine import StateMachine, State

class StateMachine(StateMachine):
    initializing = State('Initializing', initial=True)
    idle = State('Idle')
    moving_to_object = State('Moving to object')
    ready_to_pick_up = State('Ready to pick up')
    sorting = State('Sorting')
    over_box = State('Over Box')
    default = State('Default')
    emergency = State('Emergency')
    picked_up = State('Picked up')

    start_idle = initializing.to(initializing)
    idling = initializing.to(idle) | sorting.to(idle) | moving_to_object.to(idle) | idle.to(idle)
    move_to_object = idle.to(moving_to_object) | ready_to_pick_up.to(moving_to_object) | sorting.to(moving_to_object) | moving_to_object.to(moving_to_object)
    ready_to_pick_up_state = moving_to_object.to(ready_to_pick_up) | ready_to_pick_up.to(ready_to_pick_up)
    sort_object = ready_to_pick_up.to(sorting) | picked_up.to(sorting) | sorting.to(sorting) 
    over_box_state = sorting.to(over_box) | over_box.to(over_box) 
    default_state = sorting.to(default) | over_box.to(default) | default.to(default)
    emergency_state = idle.to(emergency) | moving_to_object.to(emergency) | ready_to_pick_up.to(emergency) | sorting.to(emergency) | over_box.to(emergency) | default.to(emergency) | picked_up.to(emergency) | emergency.to(emergency)
    picked_up_state = ready_to_pick_up.to(picked_up) | picked_up.to(picked_up)
    

class regelungs_node(Node):
    
    def __init__(self):
        super().__init__('regelungs_node')
        self.state_machine = StateMachine()

        
        self.get_logger().info('Start initializing')
        self.object_data_sub = self.create_subscription(ObjectData, 'object_data', self.object_data_callback, 10)
        self.target_pos_sub = self.create_subscription(TargetPose, 'target_position', self.target_position_callback, 10)
        self.vel_sub = self.create_subscription(Float64, 'velocity', self.velocity_callback, 10)
        self.arm_positions_sub = self.create_subscription(RobotPos, 'robot_position', self.arm_position_callback, 10)
        
        self.robot_command_pub = self.create_publisher(RobotCmd, 'robot_command', 10)
        self.robot_pos = {'x': 0, 'y': 0, 'z': 0}
        self.object_data = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.velocity = 0
        self.zero_position = {'x': 10, 'y': 10, 'z': 10}
        self.gripper_is_activated = False
        self.target_position = {'x': 0, 'y': 0, 'z': 0}
        self.corner = {'x': 0, 'y': 0, 'z': 0}
        self.last_calculation_time = time.time_ns()

        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_z = 0
        #self.kp = 9.85199 
        #self.kd_x = 6.447857
        self.kp_x = 6.0
        self.kd_x = 5.7 
        self.kp_y = 3 
        self.kd_y = 8
        self.kp_z = 3 
        self.kd_z = 8
        self.first_arm_pos = 0
        
        self.user_target = False
        self.velo_zaehler = 0

        self.queue = []

        self.emergency = False
        
        self.box_cat = {'x': 0.0854, 'y': 0.0, 'z': 0.0562} #box 1
        self.box_unicorn = {'x': 0.0012, 'y': 0.0, 'z': 0.0562} #box 2
        self.default_pos = {'x': 0.0854, 'y': 0.0556, 'z': 0.0562}  
        self.safe_pos = {'x': 0, 'y': 0, 'z': 0}
        self.pick_up_z = 0.0791  #0.0068
        self.ready_to_pick_up_z = 0.0714 #0.0067
        self.transport_z= 0.0562
        self.last_msg_time = time.time()
        self.move_to_zero_position()
        #time.sleep(15)
        
        self.controlling_tolerance = 0.005
        self.safe_mode = False
        self.current_time = 1

        
        self.get_logger().info('ready to receive data')
        

        self.controll_u_x = 0
        self.controll_u_y = 0
        self.controll_u_z = 0
        
        self.get_logger().info(f"state: {self.state_machine.current_state.id}")
        self.img_path = "/home/markus/Desktop/diagramm.png"
        self.state_machine._graph().write(self.img_path)

      
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
        self.controll_u_x = robot_cmd.accel_x
        self.controll_u_y = robot_cmd.accel_y
        self.controll_u_z = robot_cmd.accel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        time.sleep(1)
        robot_cmd.accel_x = 0.0
        robot_cmd.accel_y = -0.01
        robot_cmd.accel_z = 0.0
        self.controll_u_x = robot_cmd.accel_x
        self.controll_u_y = robot_cmd.accel_y
        self.controll_u_z = robot_cmd.accel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        time.sleep(1)
        robot_cmd.accel_x = -0.01
        robot_cmd.accel_y = 0.0
        robot_cmd.accel_z = 0.0
        self.controll_u_x = robot_cmd.accel_x
        self.controll_u_y = robot_cmd.accel_y
        self.controll_u_z = robot_cmd.accel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        time.sleep(1)
        robot_cmd.accel_x = 0.0
        robot_cmd.accel_y = 0.0
        robot_cmd.accel_z = 0.0
        self.controll_u_x = robot_cmd.accel_x
        self.controll_u_y = robot_cmd.accel_y
        self.controll_u_z = robot_cmd.accel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        

        
        
        
        
        

        

    def target_position_callback(self, msg):
        self.user_target = True
        self.target_position['x'] = msg.target_position_x + self.zero_position['x']
        self.target_position['y'] = msg.target_position_y + self.zero_position['y']
        self.target_position['z'] = msg.target_position_z + self.zero_position['z']
        self.get_logger().info(f"target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
        
        self.go_to_target_position()
        
    


    def arm_position_callback(self, msg):
        self.last_calculation_time = self.current_time
        self.current_time = time.time_ns() / 10e9 #self.get_clock().now().to_msg().sec
        print(self.current_time)
        print(self.last_calculation_time)
        
        self.get_logger().info(f"state: {self.state_machine.current_state.id}")
        if (self.state_machine.current_state == StateMachine.initializing):
            self.robot_pos['x'] = round(-msg.pos_x, 3)
            self.robot_pos['y'] = round(msg.pos_y, 3)
            self.robot_pos['z'] = round(msg.pos_z, 3)
            self.zero_position['x'] = self.robot_pos['x']
            self.zero_position['y'] = self.robot_pos['y']
            self.zero_position['z'] = self.robot_pos['z']
            self.box_unicorn =  self.adjust_box_position(self.box_unicorn)
            self.box_cat =  self.adjust_box_position(self.box_cat)
            self.default_pos =  self.adjust_box_position(self.default_pos)
            self.safe_pos = self.adjust_box_position(self.safe_pos)
            self.pick_up_z = self.zero_position['z'] + self.pick_up_z
            self.ready_to_pick_up_z = self.zero_position['z'] + self.ready_to_pick_up_z
            self.get_logger().info(f"zero position is: x={self.zero_position['x']}, y={self.zero_position['y']}, z={self.zero_position['z']}")
            self.state_machine.idling()
            self.get_logger().info('initializing finished')
            return
        
        else:
            self.robot_pos['x'] = round((-msg.pos_x + self.zero_position['x']), 3)
            self.robot_pos['y'] = round((msg.pos_y + self.zero_position['y']), 3)
            self.robot_pos['z'] = round((msg.pos_z + self.zero_position['z']), 3)
        
        #if current_time - self.last_msg_time >= 1.0:
        self.get_logger().info(f"robot position is: x={self.robot_pos['x']}, y={self.robot_pos['y']}, z={self.robot_pos['z']}")
        self.last_msg_time = self.current_time
        self.calculate_target_position()
        

    def object_data_callback(self, msg):
        self.user_target = False
        self.object_data['x'] = msg.object_pos_x + self.zero_position['x']
        self.object_data['y'] = msg.object_pos_y + self.zero_position['y']
        self.object_data['class'] = msg.object_class
        self.object_data['timestamp'] = msg.timestamp_value
        self.object_data['index'] = msg.index_value
        if not any(obj['index'] == self.object_data['index'] for obj in self.queue):
            self.enqueue(self.object_data)
        
        if(len(self.queue) == 1):
            self.dequeue()
        

        
       

    def velocity_callback(self, msg):
        self.velocity = (self.velocity * self.velo_zaehler + msg.data) / (self.velo_zaehler + 1)
        self.velo_zaehler += 1

    def calculate_target_position(self):

        #self.get_logger().info('Start calculating target position')
        if(self.user_target == True):
            self.get_logger().info(f"user target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
            
        elif(self.robot_pos['z'] == self.ready_to_pick_up_z):
            self.target_position['z'] = self.pick_up_z 
            
        elif(self.gripper_is_activated is True):
            if(self.oldest_object['class'] == 'cat' or 'unicorn' and self.state_machine.current_state != StateMachine.sorting):
                self.sort(self.oldest_object)
            else:
                return
                
        
        elif(self.oldest_object['class'] == 'cat' or 'unicorn' and self.state_machine.current_state == StateMachine.moving_to_object):
            self.target_position['x'] = self.oldest_object['x'] + self.velocity * (time.time() - self.oldest_object['timestamp'])
            self.target_position['y'] = self.oldest_object['y']
            self.target_position['z'] = self.ready_to_pick_up_z
            self.get_logger().info(f"object is at: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
       
        else: 
            self.update_state()
            if self.state_machine.current_state == StateMachine.idle:
                return
            else:    
                self.target_position = self.default_pos
        self.get_logger().info(f"target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
        self.go_to_target_position()

        

    def go_to_target_position(self):
        self.get_logger().info(f"going from robot position x:{self.robot_pos['x']}, y: {self.robot_pos['y']}, z: {self.robot_pos['z']}")
        self.get_logger().info(f"Start going to target position x:{self.target_position['x']}, y: {self.target_position['y']}, z: {self.target_position['z']}")
       
        if(self.user_target == True):
            self.regler()
            return

        if self.state_machine.current_state == StateMachine.moving_to_object or self.state_machine.current_state == StateMachine.sorting:
            self.regler()
            return
           
        if self.state_machine.current_state == StateMachine.over_box:
            self.gripper_is_activated = False
            self.regler()
            return
        
        else:
            if(self.state_machine.current_state == StateMachine.ready_to_pick_up): 
                print('picking up')
                
                self.target_position['z'] = self.pick_up_z
                self.regler()
                if(self.gripper_is_activated == True):
                    self.sort(self.oldest_object)
            
            else: 
                self.state = StateMachine.moving_to_object
                self.regler()
       
    def sort(self, oldest_object):
        self.state = StateMachine.sort_object
        self.get_logger().info('Start sorting')
        self.target_position['x'] = self.target_position['x']
        self.target_position['y'] = self.target_position['y']
        self.target_position['z'] = self.transport_z
        
        if self.gripper_is_activated: 
            if oldest_object['class'] == 'cat':
                self.target_position = self.box_cat
                
            elif oldest_object['class'] == 'unicorn':
                self.target_position = self.box_unicorn
                
            else: 
                self.target_position = self.default_pos
            if(self.state == StateMachine.over_box):
                self.gripper_is_activated = False 
                robot_cmd = RobotCmd()
                robot_cmd.activate_gripper = self.gripper_is_activated
                self.robot_command_pub.publish(robot_cmd) 
                self.oldest_object = self.dequeue()
        self.regler()        

    def regler(self):
        #self.get_logger().info('Start controlling')
        print(self.current_time)
        print(self.last_calculation_time)
       
        differenz_x = self.target_position['x'] - self.robot_pos['x']    
        differenz_y = self.target_position['y'] - self.robot_pos['y']  
        differenz_z = self.target_position['z'] - self.robot_pos['z']   
        print(differenz_x)
        #print(differenz_y)
        #print(differenz_z)

        
    
        dt = (self.current_time - self.last_calculation_time) 
      
        

        
        u_x = self.compute_pd(differenz_x, self.last_error_x, dt, self.kp_x, self.kd_x)
        self.last_error_x = differenz_x
        self.controll_u_x = u_x
        #print(u_x)
        self.get_logger().info(f"differenz x:{differenz_x}, u_x: {u_x}, last error {self.last_error_x}, kd: {self.kd_x}, kp: {self.kp_x}")
        self.get_logger().info(f"robot x:{self.robot_pos['x']},")
        

        u_y = self.compute_pd(differenz_y, self.last_error_y, dt, self.kp_y, self.kd_y)
        self.last_error_y = differenz_y
        self.controll_u_y = u_y
        #print(u_y)
        
        u_z = self.compute_pd(differenz_z, self.last_error_z, dt, self.kp_z, self.kd_z)
        self.last_error_z = differenz_z
        self.controll_u_z = u_z
        #print(u_z)
     
        robot_cmd = RobotCmd()
        robot_cmd.accel_x = u_x
        robot_cmd.accel_y = u_y
        robot_cmd.accel_z = u_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        self.get_logger().info('published robot_cmd')
        self.update_state()


        
    def compute_pd(self, error, last_error, dt, kp,kd):
        
        
        derivative = (error - last_error) / dt
        
        control_signal = kp * error + kd * derivative * error
        
        control_signal = float(control_signal)
        
        if (self.safe_mode or self.emergency):
            control_signal = 0.1 * control_signal
        
       
        control_signal = max(min(control_signal, 0.3), -0.3)
        
        return control_signal


    
    def enqueue(self, object_data):
        self.queue.append(object_data)
    
    def dequeue(self):
        if(len(self.queue) != 0):
            self.oldest_object = self.queue.pop(0)
            self.state_machine.move_to_object()
            self.calculate_target_position()
        else: self.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}

    def emergency_case(self, Fehlermeldung):
        self.state = State.Emergency
        self.get_logger().info('Fehler: ' + Fehlermeldung)
        self.move_to_zero_position()
        self.emergency = True
        self.safe_mode = True
        
        
            
    def update_state(self):
        self.get_logger().info('updating state')
        if self.emergency:
            self.state_machine.emergency_state()
            self.get_logger().info('emergency')
            return
        elif ((abs(self.robot_pos['x'] - self.box_cat['x']) < self.controlling_tolerance and abs(self.robot_pos['y'] - self.box_cat['y']) < self.controlling_tolerance) or 
              (abs(self.robot_pos['x'] - self.box_unicorn['x']) < self.controlling_tolerance and abs(self.robot_pos['y'] - self.box_unicorn['y']) < self.controlling_tolerance) and 
              self.gripper_is_activated):
            self.state_machine.over_box_state()
            self.gripper_is_activated = False 
            robot_cmd = RobotCmd()
            robot_cmd.activate_gripper = self.gripper_is_activated
            self.robot_command_pub.publish(robot_cmd)
            self.get_logger().info(f'Over Box; released object') 
            return
        elif (self.gripper_is_activated and 
              ((abs(self.box_cat['x'] - self.target_position['x']) > self.controlling_tolerance or abs(self.box_cat['y'] - self.target_position['y']) > self.controlling_tolerance) or 
               (abs(self.box_unicorn['x'] - self.target_position['x']) > self.controlling_tolerance or abs(self.box_unicorn['y'] - self.target_position['y']) > self.controlling_tolerance)) and 
               not self.user_target):
            self.state_machine.sort_object()
            self.get_logger().info(f'Updated state to sorting')  
            return
        elif (abs(self.target_position['x'] - self.robot_pos['x']) < self.controlling_tolerance and 
              abs(self.target_position['y'] - self.robot_pos['y']) < self.controlling_tolerance and 
              abs(self.ready_to_pick_up_z - self.robot_pos['z']) < self.controlling_tolerance and 
              not self.gripper_is_activated and 
              not self.user_target):
            self.state_machine.ready_to_pick_up_state()
            self.gripper_is_activated = True
            robot_cmd = RobotCmd()
            robot_cmd.activate_gripper = self.gripper_is_activated
            self.robot_command_pub.publish(robot_cmd)
            self.get_logger().info(f'ready to pick up')  
            return
        elif (abs(self.target_position['x'] - self.robot_pos['x']) < self.controlling_tolerance and 
              abs(self.target_position['y'] - self.robot_pos['y']) < self.controlling_tolerance and 
              abs(self.pick_up_z - self.robot_pos['z']) < self.controlling_tolerance and 
              self.gripper_is_activated and 
              not self.user_target):
            self.state_machine.picked_up_state()
            self.get_logger().info('object picked up going to sort')
            self.sort()
            return
        elif (abs(self.target_position['x'] - self.robot_pos['x']) >= self.controlling_tolerance or 
              abs(self.target_position['y'] - self.robot_pos['y']) >= self.controlling_tolerance or 
              abs(self.target_position['z'] - self.robot_pos['z']) >= self.controlling_tolerance and 
              ( self.gripper_is_activated == False or self.user_target)):
            self.state_machine.move_to_object()
            self.get_logger().info('moving to object')
        elif (abs(self.robot_pos['x'] - self.default_pos['x']) < self.controlling_tolerance and 
              abs(self.robot_pos['y'] - self.default_pos['y']) < self.controlling_tolerance and 
              abs(self.robot_pos['z'] - self.default_pos['z']) < self.controlling_tolerance):
            self.state_machine.default_state()
            self.get_logger().info(f'default')  
            return
        elif (abs(self.target_position['x'] - self.robot_pos['x']) < self.controlling_tolerance and 
              abs(self.target_position['y'] - self.robot_pos['y']) < self.controlling_tolerance and 
              abs(self.target_position['z'] - self.robot_pos['z']) < self.controlling_tolerance and 
              (self.oldest_object['index'] is None or not self.user_target)):
            self.state_machine.idling()
            self.get_logger().info(f'idle')  
            return
        else:
            self.emergency_case('unknown State')

            

def main(args=None):
    rclpy.init(args=args)
    node = regelungs_node()
    rclpy.spin(node)
    rclpy.shutdown()
                
if __name__ == '__main__':
    main()



