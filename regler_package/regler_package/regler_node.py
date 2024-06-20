import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from target_pose_interfaces.msg import TargetPose
from object_interfaces.msg import ObjectData
from std_msgs.msg import Float64
import time
from builtin_interfaces.msg import Time
from statemachine import StateMachine as BaseStateMachine, State

class StateMachine:
    def __init__(self, node):
        self.node = node
        self.states = {
            'initializing': Initializing(node),
            'idle': Idle(node),
            'moving_to_object': MovingToObject(node),
            'ready_to_pick_up': ReadyToPickUp(node),
            'picked_up': PickedUp(node),
            'ready_to_sort': ReadyToSort(node),
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
             ((not self.node.gripper_is_activated) or self.node.user_target)):
            self.node.state_machine.transition_to('moving_to_object')
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
            abs(self.node.pick_up_z - self.node.robot_pos['z']) <= self.node.controlling_tolerance and 
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
            (self.node.transport_z - self.node.controlling_tolerance) > self.node.robot_pos['z'] and 
            self.node.gripper_is_activated and 
            not self.node.user_target):
            self.node.state_machine.transition_to('ready_to_sort')
        if (time.time_ns() - self.node.last_calculation_time) > self.node.callback_period:
            self.node.state_machine.transition_to('emergency')
            self.node.emergency_case('No position callbacks in the last 5 seconds')

class ReadyToSort(CustomState):
    def update_state(self):
        if self.node.robot_pos['z'] < self.node.transport_z:
           self.node.state_machine.transition_to('sorting') 

class Sorting(CustomState):
    def update_state(self):
        if ((abs(self.node.robot_pos['x'] - self.node.box_cat['x']) < self.node.controlling_tolerance and 
             abs(self.node.robot_pos['y'] - self.node.box_cat['y']) < self.node.controlling_tolerance) or 
            (abs(self.node.robot_pos['x'] - self.node.box_unicorn['x']) < self.node.controlling_tolerance and 
             abs(self.node.robot_pos['y'] - self.node.box_unicorn['y']) < self.node.controlling_tolerance) and 
            self.node.gripper_is_activated):
            self.node.state_machine.transition_to('over_box')
        if (time.time_ns() - self.node.last_calculation_time) > self.node.callback_period:
            self.node.state_machine.transition_to('emergency')
            self.node.emergency_case('No position callbacks in the last 5 seconds')

class OverBox(CustomState):
    def update_state(self):
        if not self.node.gripper_is_activated:
            self.node.state_machine.transition_to('idle')
        if (time.time_ns() - self.node.last_calculation_time) > self.node.callback_period:
            self.node.state_machine.transition_to('emergency')
            self.node.emergency_case('No position callbacks in the last 5 seconds')

class Default(CustomState):
    def update_state(self):
        if (abs(self.node.robot_pos['x'] - self.node.default_pos['x']) < self.node.controlling_tolerance and 
            abs(self.node.robot_pos['y'] - self.node.default_pos['y']) < self.node.controlling_tolerance and 
            abs(self.node.robot_pos['z'] - self.node.default_pos['z']) < self.node.controlling_tolerance):
            self.node.state_machine.transition_to('default')
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
        self.object_data = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.velocity = 0
        self.zero_position = {'x': None, 'y': None, 'z': None}
        self.gripper_is_activated = False
        self.target_position = {'x': None, 'y': None, 'z': None}
        self.corner = {'x': 0, 'y': 0, 'z': 0}
        self.last_calculation_time = time.time_ns()

        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_z = 0
        #self.kp = 9.85199 
        #self.kd_x = 6.447857
        self.kp_x = 0.985199
        self.kd_x = 0.49
        self.kp_y = 0.3
        self.kd_y = 0.4
        self.kp_z = 0.5 
        self.kd_z = 0.5
        self.n = 100
        self.first_arm_pos = 0
        
        self.user_target = False
        self.velo_zaehler = 0

        self.queue = []

        self.emergency = False
        
        self.box_cat = {'x': 0.0854, 'y': 0.0, 'z': 0.0562} #box 1  ; 0.0562
        self.box_unicorn = {'x': 0.0012, 'y': 0.0, 'z': 0.0562} #box 2
        #self.default_pos = {'x': 0.0854, 'y': 0.0556, 'z': 0.0562}
        self.default_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}  
        self.safe_pos = {'x': 0, 'y': 0, 'z': 0}
        self.pick_up_z = 0.0791  
        self.ready_to_pick_up_z = 0.0714 
        self.transport_z= 0.0562
        self.last_msg_time = time.time()
        self.move_to_zero_position()
        #time.sleep(25)
        self.opposit_corner = {'x': 0.2, 'y': 0.1, 'z': self.pick_up_z}
        self.controlling_tolerance = 0.005
        self.safe_mode = False
        self.current_time = 1

        
        self.get_logger().info('ready to receive data')
        

        self.controll_u_x = 0
        self.controll_u_y = 0
        self.controll_u_z = 0
        self.callback_period = 5e90
        
        self.get_logger().info(f"state: {self.state_machine.current_state}")
        

      
    def adjust_box_position(self, box):
        box['x'] -= self.zero_position['x']
        box['y'] -= self.zero_position['y']
        box['z'] -= self.zero_position['z']
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
        

        
    def timer_callback(self):
        if((time.time_ns()- self.last_calculation_time) > 5e90 #5e9 
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
        print(self.current_time)
        print(self.last_calculation_time)
        
        self.get_logger().info(f"State: {self.state_machine.current_state}")
        if (self.state_machine.current_state == self.state_machine.states['initializing']):
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
            self.state_machine.transition_to('idle')
            self.get_logger().info('initializing finished')
            return
        
        else:
           
            
            self.robot_pos['x'] = round((-msg.pos_x - self.zero_position['x']), 3)
            self.robot_pos['y'] = round((msg.pos_y - self.zero_position['y']), 3)
            self.robot_pos['z'] = round((msg.pos_z - self.zero_position['z']), 3)
        
        
        
        self.get_logger().debug(f"robot position is: x={self.robot_pos['x']}, y={self.robot_pos['y']}, z={self.robot_pos['z']}")
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
        
        if(self.state_machine.current_state == self.state_machine.states['idle'] 
           or self.state_machine.current_state == self.state_machine.states['default']):
            self.dequeue()
        
   
        
       

    def velocity_callback(self, msg):
        self.velocity = (self.velocity * self.velo_zaehler + msg.data) / (self.velo_zaehler + 1)
        self.velo_zaehler += 1

    def calculate_target_position(self):
        if(self.target_position['x'] == None and self.oldest_object['class'] == None):
            return
        #self.get_logger().info('Start calculating target position')
        if(self.user_target == True):
            self.get_logger().info(f"user target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
            
        elif (self.state_machine.current_state == self.state_machine.states['ready_to_pick_up'] 
            and self.target_position['z'] != self.pick_up_z):
            self.target_position['z'] = self.pick_up_z 
            print('pick up pose is set')
        elif(self.gripper_is_activated is True):
            print('going in sort part')
            if self.oldest_object['class'] in ['cat', 'unicorn'] and self.state_machine.current_state != self.state_machine.states['sorting']:
                self.sort(self.oldest_object)
            else:
                return
                
        
        elif((self.oldest_object['class'] == 'cat' or self.oldest_object['class'] == 'unicorn') 
             and self.state_machine.current_state == self.state_machine.states['moving_to_object']):
            self.target_position['x'] = self.oldest_object['x'] + self.velocity * (time.time() - self.oldest_object['timestamp'])
            self.target_position['y'] = self.oldest_object['y']
            self.target_position['z'] = self.ready_to_pick_up_z
            self.get_logger().info(f"object is at: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
       
        else: 
            self.update_state()
            if self.state_machine.current_state == self.state_machine.states['idle']:
                return
            else:    
                self.target_position = self.default_pos

        self.get_logger().info(f"target position is: x={self.target_position['x']}, y={self.target_position['y']}, z={self.target_position['z']}")
        self.go_to_target_position()

        

    def go_to_target_position(self):
        self.get_logger().info(f"going from robot position x:{self.robot_pos['x']}, y: {self.robot_pos['y']}, z: {self.robot_pos['z']}")
        self.get_logger().info(f"Start going to target position x:{self.target_position['x']}, y: {self.target_position['y']}, z: {self.target_position['z']}")
       
        if(self.user_target == True or self.state_machine.current_state == self.state_machine.states['moving_to_object']):
            self.regler()
            return

        elif(self.state_machine.current_state == self.state_machine.states['ready_to_sort'] 
             or self.state_machine.current_state == self.state_machine.states['sorting']
             or self.state_machine.current_state == self.state_machine.states['over_box']
             or self.state_machine.current_state == self.state_machine.states['picked_up']
             or self.gripper_is_activated):
            self.sort(self.oldest_object)
           
        
        
        
        elif(self.state_machine.current_state == self.state_machine.states['ready_to_pick_up']): 
            print('picking up')
            
            self.target_position['z'] = self.pick_up_z
            self.gripper_is_activated = True
            self.sort(self.oldest_object)
            
        
        else: 
            #self.state = StateMachine.moving_to_object
            self.regler()
       
    def sort(self, oldest_object):
        #self.state_machine.sort_object()
        self.get_logger().info('Start sorting')
        
        

        

        if (self.state_machine.current_state == self.state_machine.states['ready_to_sort']):
            if oldest_object['class'] == 'cat':
                self.target_position = self.box_cat
                
            elif oldest_object['class'] == 'unicorn':
                self.target_position = self.box_unicorn
                
            else: 
                self.target_position = self.default_pos
            if self.state_machine.current_state == self.state_machine.states['over_box']:
                 
                self.oldest_object = self.dequeue()
        elif (self.state_machine.current_state == self.state_machine.states['picked_up']):
            
            self.target_position['z'] = self.transport_z
            
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
        #derivative = self.n * derivative + (1 - self.n) * derivative
        
        control_signal = kp * error + kd * derivative
        
        control_signal = float(control_signal)
        
        
           
        
       
        #control_signal = max(min(control_signal, 0.3), -0.3)
        
        return control_signal


    
    def enqueue(self, object_data):
        self.queue.append(object_data)
    
    def dequeue(self):
        if(len(self.queue) != 0):
            self.oldest_object = self.queue.pop(0)
            self.state_machine.transition_to('moving_to_object')
            self.calculate_target_position()
        
    def emergency_case(self, Fehlermeldung):
        self.state_machine.transition_to('emergency')
        self.get_logger().info('Fehler: ' + Fehlermeldung)
        self.move_to_zero_position()
        time.sleep(30)
        #self.emergency = True
        #self.safe_mode = True
        if(self.robot_pos == self.zero_position):
            self.state_machine.transition_to('idle')
        
        
            
    def update_state(self):
        self.get_logger().info('updating state')
        self.state_machine.update_state()
        
            

def main(args=None):
    rclpy.init(args=args)
    node = regelungs_node()
    rclpy.spin(node)
    rclpy.shutdown()
                
if __name__ == '__main__':
    main()




