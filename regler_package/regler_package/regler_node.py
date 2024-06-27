import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from target_pose_interfaces.msg import TargetPose
from object_interfaces.msg import ObjectData
from std_msgs.msg import Float64
import time
from builtin_interfaces.msg import Time


class StateMachine:
    def __init__(self, node):
        self.node = node
        self.states = {
            'initializing': Initializing(node),
            'idle': Idle(node),
            'moving_to_object': MovingToObject(node),
            'ready_to_pick_up': ReadyToPickUp(node),
            'picked_up': PickedUp(node),
            'sorting': Sorting(node),
            'over_box': OverBox(node),
            'default': Default(node),
            'emergency': Emergency(node)
        }
        self.current_state = self.states['initializing']

    def transition_to(self, state_name):
        if state_name in self.states:
            self.current_state = self.states[state_name]
            self.node.get_logger().info(f'Transitioned to {state_name}')

    def update_state(self):
        if self.current_state:
            self.current_state.update_state()

class CustomState:
    def __init__(self, node):
        self.node = node

    def update_state(self):
        raise NotImplementedError("Each state must implement the update_state method.")

class Initializing(CustomState):
    def update_state(self):
        if self.node.zero_position['x'] is not None:
            self.node.state_machine.transition_to('idle')

class Idle(CustomState):
    def update_state(self):
        if ((abs(self.node.target_position['x'] - self.node.robot_pos['x']) >= self.node.controlling_tolerance or 
             abs(self.node.target_position['y'] - self.node.robot_pos['y']) >= self.node.controlling_tolerance or 
             abs(self.node.target_position['z'] - self.node.robot_pos['z']) >= self.node.controlling_tolerance) and 
             ((not self.node.gripper_is_activated and self.node.oldest_object['sorted'] == False) or self.node.user_target)):
            self.node.state_machine.transition_to('moving_to_object')
        if (self.node.target_position['x'] == self.node.default_position['x'] 
            and self.node.target_position['y'] == self.node.default_position['y']
            and self.node.target_position['z'] == self.node.default_position['z']):
            self.node.state_machine.transition_to('default')
        if (time.time_ns() - self.node.last_calculation_time) > self.node.callback_period:
            self.node.state_machine.transition_to('emergency')
            self.node.emergency_case('No position callbacks in the last 5 seconds') 

class MovingToObject(CustomState):
    def update_state(self):
        if (abs(self.node.target_position['x'] - self.node.robot_pos['x']) < self.node.controlling_tolerance and 
            abs(self.node.target_position['y'] - self.node.robot_pos['y']) < self.node.controlling_tolerance and 
            (self.node.robot_pos['z'] >= (self.node.ready_to_pick_up_z - self.node.controlling_tolerance) and 
             self.node.robot_pos['z'] < (self.node.pick_up_z + self.node.controlling_tolerance)) and 
            not self.node.user_target):
            self.node.state_machine.transition_to('ready_to_pick_up')
        if (time.time_ns() - self.node.last_calculation_time) > self.node.callback_period:
            self.node.state_machine.transition_to('emergency')
            self.node.emergency_case('No position callbacks in the last 5 seconds')    

class ReadyToPickUp(CustomState):
    def update_state(self):
        if (abs(self.node.target_position['x'] - self.node.robot_pos['x']) < self.node.controlling_tolerance and 
            abs(self.node.target_position['y'] - self.node.robot_pos['y']) < self.node.controlling_tolerance and 
            (self.node.pick_up_z - self.node.robot_pos['z']) <= (self.node.controlling_tolerance) and 
            self.node.gripper_is_activated and 
            not self.node.user_target):
            self.node.state_machine.transition_to('picked_up')
            self.node.target_position['z'] = self.node.transport_z
        if (time.time_ns() - self.node.last_calculation_time) > self.node.callback_period:
            self.node.state_machine.transition_to('emergency')
            self.node.emergency_case('No position callbacks in the last 5 seconds') 

class PickedUp(CustomState):
    def update_state(self):
        if (abs(self.node.target_position['x'] - self.node.robot_pos['x']) < self.node.controlling_tolerance and 
            abs(self.node.target_position['y'] - self.node.robot_pos['y']) < self.node.controlling_tolerance and 
            (self.node.transport_z + self.node.controlling_tolerance) > self.node.robot_pos['z'] and 
            self.node.gripper_is_activated and 
            not self.node.user_target):
            self.node.state_machine.transition_to('sorting')
        if (time.time_ns() - self.node.last_calculation_time) > self.node.callback_period:
            self.node.state_machine.transition_to('emergency')
            self.node.emergency_case('No position callbacks in the last 5 seconds')


class Sorting(CustomState):
    def update_state(self):
        if ((abs(self.node.robot_pos['x'] - self.node.box_cat['x']) < self.node.controlling_tolerance and 
             abs(self.node.robot_pos['y'] - self.node.box_cat['y']) < self.node.controlling_tolerance 
             and self.node.oldest_object['class'] == 'cat') or 
            (abs(self.node.robot_pos['x'] - self.node.box_unicorn['x']) < self.node.controlling_tolerance and 
             abs(self.node.robot_pos['y'] - self.node.box_unicorn['y']) < self.node.controlling_tolerance
             and self.node.oldest_object['class'] == 'unicorn') and 
            self.node.gripper_is_activated):
            self.node.state_machine.transition_to('over_box')
            
        if (time.time_ns() - self.node.last_calculation_time) > self.node.callback_period:
            self.node.state_machine.transition_to('emergency')
            self.node.emergency_case('No position callbacks in the last 5 seconds')

class OverBox(CustomState):
    def update_state(self):
        if self.node.gripper_is_activated == False:
            self.node.state_machine.transition_to('idle')
        
        if (time.time_ns() - self.node.last_calculation_time) > self.node.callback_period:
            self.node.state_machine.transition_to('emergency')
            self.node.emergency_case('No position callbacks in the last 5 seconds')


class Default(CustomState):
    def update_state(self):
        if ((abs(self.node.target_position['x'] - self.node.robot_pos['x']) >= self.node.controlling_tolerance or 
             abs(self.node.target_position['y'] - self.node.robot_pos['y']) >= self.node.controlling_tolerance or 
             abs(self.node.target_position['z'] - self.node.robot_pos['z']) >= self.node.controlling_tolerance) and 
             ((not self.node.gripper_is_activated and self.node.oldest_object['sorted'] == False) or self.node.user_target)):
            self.node.state_machine.transition_to('moving_to_object')
        
        if (time.time_ns() - self.node.last_calculation_time) > self.node.callback_period:
            self.node.state_machine.transition_to('emergency')
            self.node.emergency_case('No position callbacks in the last 5 seconds')

class Emergency(CustomState):
    def update_state(self):
        if abs(self.node.robot_pos['x']) < self.node.controlling_tolerance:
            self.node.state_machine.transition_to('idle')        

class regelungs_node(Node):
    
    def __init__(self):
        super().__init__('regelungs_node')
        self.state_machine = StateMachine(self)

        
        self.get_logger().info('Start initializing')
        self.object_data_sub = self.create_subscription(ObjectData, 'object_data', self.object_data_callback, 10)
        self.target_pos_sub = self.create_subscription(TargetPose, 'target_position', self.target_position_callback, 10)
        self.vel_sub = self.create_subscription(Float64, 'velocity', self.velocity_callback, 10)
        self.arm_positions_sub = self.create_subscription(RobotPos, 'robot_position', self.arm_position_callback, 5)
        self.timer_period = 5.0 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.robot_command_pub = self.create_publisher(RobotCmd, 'robot_command', 10)
        self.robot_pos = {'x': 0, 'y': 0, 'z': 0}
       
        self.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None, 'sorted' : False}
        self.velocity = 0
        self.zero_position = {'x': None, 'y': None, 'z': None}
        self.default_position = {'x': 0.15, 'y': 0.06, 'z': 0.06}
        self.gripper_is_activated = False
        self.target_position = {'x': None, 'y': None, 'z': None}
        
        self.last_calculation_time = time.time_ns()
        self.object_data = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_z = 0
        #self.kp = 9.85199 
        #self.kd_x = 6.447857
        self.kp_x = 0.98
        self.kd_x = 0.49
        self.kp_y = 0.98
        self.kd_y = 0.49
        self.kp_z = 0.9 #0.985199
        self.kd_z = 0.51
        self.n = 100
        self.first_arm_pos = 0
        
        self.user_target = False
        self.velo_zaehler = 0

        self.queue = []

        self.box_cat = {'x': 0.0854, 'y': 0.001, 'z': 0.0562} 
        self.box_unicorn = {'x': 0.0012, 'y': 0.001, 'z': 0.0562} 
        
          
        
        self.pick_up_z = 0.079  
        self.ready_to_pick_up_z = 0.0714 
        self.transport_z= 0.055
        self.last_msg_time = time.time()
        self.move_to_zero_position()
        time.sleep(25)
        self.opposit_corner = {'x': 0.2, 'y': 0.1, 'z': self.pick_up_z}
        self.controlling_tolerance = 0.005
        self.current_time = 1

        
        self.get_logger().info('ready to receive data')
        

        self.controll_u_x = 0
        self.controll_u_y = 0
        self.controll_u_z = 0
        self.callback_period = 5e90
        self.black_list_objects = []
        self.velocity_in_coordinates = 0.0066
        self.get_logger().debug(f"state: {self.state_machine.current_state}")
        

      
    def adjust_box_position(self, box):
        box['x'] -= self.zero_position['x']
        box['y'] -= self.zero_position['y']
        box['z'] -= self.zero_position['z']
        return box

    """
    move_to_zero_position let the robot go to the corner where switches are placed. It gives a start point for the robot and 
    defines the coordinate zero point
    """

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
        

        
    def timer_callback(self):
        if((time.time_ns()- self.last_calculation_time) > self.callback_period
        and self.state_machine.current_state != self.state_machine.states['initializing']):
            self.emergency_case('no position callbacks in the last 5 seconds')   
        
        
        

        

    def target_position_callback(self, msg):
        self.user_target = True
        self.target_position['x'] = msg.target_position_x #- self.zero_position['x']
        self.target_position['y'] = msg.target_position_y #- self.zero_position['y']
        self.target_position['z'] = msg.target_position_z #- self.zero_position['z']
        self.get_logger().info(f"target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
        
        self.go_to_target_position()
        
    


    def arm_position_callback(self, msg):
        self.last_calculation_time = self.current_time
        self.current_time = time.time_ns() / 10e9 

        self.get_logger().debug(f"last time: {self.last_calculation_time}, new time: {self.current_time}", throttle_duration_sec = 1)
        
        self.get_logger().debug(f"State: {self.state_machine.current_state}", throttle_duration_sec = 1)
        if (self.state_machine.current_state == self.state_machine.states['initializing']):
            self.robot_pos['x'] = round(-msg.pos_x, 3)
            self.robot_pos['y'] = round(msg.pos_y, 3)
            self.robot_pos['z'] = round(msg.pos_z, 3)
            self.zero_position['x'] = self.robot_pos['x']
            self.zero_position['y'] = self.robot_pos['y']
            self.zero_position['z'] = self.robot_pos['z']
            self.get_logger().debug(f"zero position is: x={self.zero_position['x']}, y={self.zero_position['y']}, z={self.zero_position['z']}")
            self.state_machine.transition_to('idle')
            self.get_logger().info('initializing finished')
            return
        
        else:
           
            self.robot_pos['x'] = round((-msg.pos_x - self.zero_position['x']), 3)
            self.robot_pos['y'] = round((msg.pos_y - self.zero_position['y']), 3)
            self.robot_pos['z'] = round((msg.pos_z - self.zero_position['z']), 3)
                                        
        self.get_logger().debug(f"robot position is: x={self.robot_pos['x']}, y={self.robot_pos['y']}, z={self.robot_pos['z']}", throttle_duration_sec = 1)
        self.last_msg_time = self.current_time
        self.calculate_target_position()
        

    def object_data_callback(self, msg):
        self.user_target = False
        self.object_data['x'] = (-0.00009823 * msg.object_pos_x)-(0.00009554*(1017-msg.object_pos_y)) + 0.27329 
        self.object_data['y'] = (0.00000110 * msg.object_pos_x)-(0.00000725*(1017-msg.object_pos_y)) + 0.04866   
        self.object_data['class'] = msg.object_class
        self.object_data['timestamp'] = msg.timestamp_value
        self.object_data['index'] = msg.index_value
        if(self.object_data['index'] == self.oldest_object['index']):
            self.get_logger().debug(f"updatet oldest object from x: {self.oldest_object['x']} timestamp: {self.oldest_object['timestamp']}")
            self.oldest_object['x'] = self.object_data['x']
            self.oldest_object['timestamp'] = self.object_data['timestamp']
            self.get_logger().debug(f"updatet oldest object to x: {self.oldest_object['x']} timestamp: {self.oldest_object['timestamp']}")

        if (not any(obj['index'] == self.object_data['index'] for obj in self.queue) 
            and self.object_data['index'] not in self.black_list_objects
            and self.oldest_object['index'] != self.object_data['index']):
            self.enqueue(self.object_data)
            self.get_logger().debug(f'received object date enqueuing now')
        
        if(self.state_machine.current_state == self.state_machine.states['idle'] 
           or self.state_machine.current_state == self.state_machine.states['default']):
            self.get_logger().debug(f'robot is in idle or default dequeuing object now')
            self.dequeue()
        self.get_logger().debug(f'oldest object is: {self.oldest_object}')
   
        
    def velocity_callback(self, msg):
        if self.velo_zaehler == 5:
            self.velocity = self.msg.data * self.velocity_in_coordinates
        if self.velo_zaehler > 6:
            self.velocity = (self.velocity * self.velo_zaehler + round((msg.data),3) * self.velocity_in_coordinates) / (self.velo_zaehler + 1)
            self.velo_zaehler += 1
        self.get_logger().debug(f'raw input velocity is: {msg.data}')
        self.get_logger().debug(f'transformed input velocity is: {msg.data*self.velocity_in_coordinates}')
        self.get_logger().debug(f'calculated velocity is: {self.velocity}')

    def calculate_target_position(self):
        
        if(self.state_machine.current_state == self.state_machine.states['over_box']):
            self.sort(self.oldest_object)
            return
        if(self.target_position is None and self.oldest_object is None):
            return
        if(self.oldest_object is not None):
            self.get_logger().debug('Start calculating target position')
            if(self.user_target == True):
                self.get_logger().debug(f"user target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
                
            elif (self.state_machine.current_state == self.state_machine.states['ready_to_pick_up'] 
                and self.target_position['z'] != self.pick_up_z):
                self.target_position['z'] = self.pick_up_z 
                self.get_logger().debug(f'pick up pose is set')
            elif(self.gripper_is_activated is True):
                
                if (self.oldest_object['class'] == 'cat' or self.oldest_object['class'] == 'unicorn'):
                    self.get_logger().debug(f'going in sort part')
                    self.sort(self.oldest_object)
                
                else:
                    return
            elif(self.oldest_object['class'] != 'cat' and self.oldest_object['class'] != 'unicorn'):      
                self.dequeue()
                return
            
            elif((self.oldest_object['class'] == 'cat' or self.oldest_object['class'] == 'unicorn') 
                and self.state_machine.current_state == self.state_machine.states['moving_to_object']):
                self.target_position['x'] = (self.oldest_object['x'] - self.velocity * (time.time() - self.oldest_object['timestamp'])) 
                self.target_position['y'] = self.oldest_object['y'] 
                self.target_position['z'] = self.ready_to_pick_up_z 
                
                self.get_logger().debug(f"object is at: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
        
        else: 
            self.update_state()
            if self.state_machine.current_state == self.state_machine.states['idle']:
                return
            
        self.target_position['x'] = min(self.target_position['x'], self.opposit_corner['x'])
        self.target_position['y'] = min(self.target_position['y'], self.opposit_corner['y'])
        self.target_position['z'] = min(self.target_position['z'], self.opposit_corner['z'])
        self.get_logger().debug(f"target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
        self.go_to_target_position()

        

    def go_to_target_position(self):
        self.get_logger().debug(f"going from robot position x:{self.robot_pos['x']}, y: {self.robot_pos['y']}, z: {self.robot_pos['z']}")
        self.get_logger().debug(f"Start going to target position x:{self.target_position['x']}, y: {self.target_position['y']}, z: {self.target_position['z']}")
       
        if(self.user_target == True or self.state_machine.current_state == self.state_machine.states['moving_to_object']):
            self.regler()
            return

        elif(self.state_machine.current_state == self.state_machine.states['sorting']
             or self.state_machine.current_state == self.state_machine.states['over_box']
             or self.state_machine.current_state == self.state_machine.states['picked_up']
             or self.gripper_is_activated):
            self.sort(self.oldest_object)
        
        elif(self.state_machine.current_state == self.state_machine.states['ready_to_pick_up']): 
            self.get_logger().debug(f'picking up')
            self.target_position['x'] = (self.oldest_object['x'] - self.velocity * (time.time() - self.oldest_object['timestamp'])) 
            self.target_position['z'] = self.pick_up_z
            self.gripper_is_activated = True
            self.sort(self.oldest_object)
            
        else: 
            self.regler()
       
    def sort(self, oldest_object):
        self.get_logger().debug('Start sorting')
        if(self.oldest_object is not None):
        
            if (self.state_machine.current_state == self.state_machine.states['sorting']):
                if oldest_object['class'] == 'cat':
                    self.target_position['x'] = self.box_cat['x']
                    self.target_position['y'] = self.box_cat['y']
                    self.target_position['z'] = self.box_cat['z']
                    
                elif oldest_object['class'] == 'unicorn':
                    self.target_position['x'] = self.box_unicorn['x']
                    self.target_position['y'] = self.box_unicorn['y']
                    self.target_position['z'] = self.box_unicorn['z']
                    
                
            elif self.state_machine.current_state == self.state_machine.states['over_box']:
                robot_cmd = RobotCmd()
                self.gripper_is_activated = False
                robot_cmd.activate_gripper = self.gripper_is_activated
                self.robot_command_pub.publish(robot_cmd) 
                self.get_logger().debug(f'deactivating gripper now gripper is: {self.gripper_is_activated}')
                self.update_state()
                self.target_position['x'] = self.default_position['x']
                self.target_position['y'] = self.default_position['y']
                self.target_position['z'] = self.default_position['z']
                
                self.dequeue()
            elif (self.state_machine.current_state == self.state_machine.states['picked_up']):
                
                self.target_position['z'] = self.transport_z
            
            self.regler()        

    def regler(self):
        self.get_logger().debug(f'target position is: {self.target_position}', throttle_duration_sec = 1)
        self.get_logger().debug('Start controlling')
        
       
        differenz_x = self.target_position['x'] - self.robot_pos['x']    
        differenz_y = self.target_position['y'] - self.robot_pos['y']  
        differenz_z = self.target_position['z'] - self.robot_pos['z']   
        
        dt = (self.current_time - self.last_calculation_time) 
      
        u_x = self.compute_pd(differenz_x, self.last_error_x, dt, self.kp_x, self.kd_x)
        self.last_error_x = differenz_x
        self.controll_u_x = u_x
        self.get_logger().debug(f"differenz x:{differenz_x}, u_x: {u_x}, last error {self.last_error_x}, kd: {self.kd_x}, kp: {self.kp_x}, robot x:{self.robot_pos['x']},")
        
        u_y = self.compute_pd(differenz_y, self.last_error_y, dt, self.kp_y, self.kd_y)
        self.last_error_y = differenz_y
        self.controll_u_y = u_y 
        self.get_logger().debug(f"differenz y:{differenz_y}, u_y: {u_y}, last error {self.last_error_y}, kd: {self.kd_y}, kp: {self.kp_y}, robot y:{self.robot_pos['y']},")
        
        u_z = self.compute_pd(differenz_z, self.last_error_z, dt, self.kp_z, self.kd_z)
        self.last_error_z = differenz_z
        self.controll_u_z = u_z
        self.get_logger().debug(f"differenz z:{differenz_z}, u_z: {u_z}, last error {self.last_error_z}, kd: {self.kd_z}, kp: {self.kp_z}, robot z:{self.robot_pos['z']},")
     
        robot_cmd = RobotCmd()
        robot_cmd.accel_x = u_x
        robot_cmd.accel_y = u_y
        robot_cmd.accel_z = u_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        self.get_logger().debug('published robot_cmd')
        self.update_state()


        
    def compute_pd(self, error, last_error, dt, kp,kd):
        
        
        derivative = (error - last_error) / dt
        
        control_signal = kp * error + kd * derivative
        
        control_signal = float(control_signal)
        
        
        return control_signal

    """
    def regler(self):
        self.get_logger().debug(f'target position is: {self.target_position}', throttle_duration_sec=1)
        self.get_logger().debug('Start controlling')
        
        differenz_x = self.target_position['x'] - self.robot_pos['x']    
        differenz_y = self.target_position['y'] - self.robot_pos['y']  
        differenz_z = self.target_position['z'] - self.robot_pos['z']   
        
        dt = (self.current_time - self.last_calculation_time)
        
        u_x, self.last_derivative_x = self.compute_pd(differenz_x, self.last_error_x, self.last_derivative_x, dt, self.kp_x, self.kd_x)
        self.last_error_x = differenz_x
        self.controll_u_x = u_x
        self.get_logger().debug(f"differenz x:{differenz_x}, u_x: {u_x}, last error {self.last_error_x}, kd: {self.kd_x}, kp: {self.kp_x}, robot x:{self.robot_pos['x']},")
        
        u_y, self.last_derivative_y = self.compute_pd(differenz_y, self.last_error_y, self.last_derivative_y, dt, self.kp_y, self.kd_y)
        self.last_error_y = differenz_y
        self.controll_u_y = u_y 
        self.get_logger().debug(f"differenz y:{differenz_y}, u_y: {u_y}, last error {self.last_error_y}, kd: {self.kd_y}, kp: {self.kp_y}, robot y:{self.robot_pos['y']},")
        
        u_z, self.last_derivative_z = self.compute_pd(differenz_z, self.last_error_z, self.last_derivative_z, dt, self.kp_z, self.kd_z)
        self.last_error_z = differenz_z
        self.controll_u_z = u_z
        self.get_logger().debug(f"differenz z:{differenz_z}, u_z: {u_z}, last error {self.last_error_z}, kd: {self.kd_z}, kp: {self.kp_z}, robot z:{self.robot_pos['z']},")
     
        robot_cmd = RobotCmd()
        robot_cmd.accel_x = u_x
        robot_cmd.accel_y = u_y
        robot_cmd.accel_z = u_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        self.get_logger().debug('published robot_cmd')
        self.update_state()

    def compute_pd(self, error, last_error, last_derivative, dt, kp, kd):
        derivative = (error - last_error) / dt
        
        filtered_derivative = self.alpha * derivative + (1 - self.alpha) * last_derivative
        
        control_signal = kp * error + kd * filtered_derivative
        control_signal = float(control_signal)
        
        return control_signal, filtered_derivative 
        """   
    
    def enqueue(self, object_data):
        self.queue.append(object_data)
        self.get_logger().debug(f'appended object')
    
    def dequeue(self):
        self.oldest_object['sorted'] = True
        self.black_list_objects.append(self.oldest_object['index']) 
        if len(self.queue) != 0:
            popped_object = self.queue.pop(0)

            
        
            if isinstance(popped_object, dict):
                self.oldest_object['x'] = popped_object.get('x', None)
                self.oldest_object['y'] = popped_object.get('y', None)
                self.oldest_object['class'] = popped_object.get('class', None)
                self.oldest_object['timestamp'] = popped_object.get('timestamp', None)
                self.oldest_object['index'] = popped_object.get('index', None)
                self.oldest_object['sorted'] = False
                
                self.get_logger().debug(f"Popped object: {popped_object}")
                self.state_machine.transition_to('moving_to_object')
                self.calculate_target_position()
            else:
                self.get_logger().error('Popped object is not a dictionary')
        else:
            self.get_logger().info(f'no objects in queue')
        
    def emergency_case(self, Fehlermeldung):
        self.state_machine.transition_to('emergency')
        self.get_logger().warn('Fehler: ' + Fehlermeldung)
        self.move_to_zero_position()
        time.sleep(30)
        if(self.robot_pos == self.zero_position):
            self.state_machine.transition_to('idle')
        
        
            
    def update_state(self):
        self.get_logger().debug(f'updating state to: {self.state_machine.current_state}')
        self.state_machine.update_state()
        
            

def main(args=None):
    rclpy.init(args=args)
    node = regelungs_node()
    rclpy.spin(node)
    rclpy.shutdown()
                
if __name__ == '__main__':
    main()

