import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from object_interfaces.msg import ObjectData
from std_msgs.msg import Float64, Float32
import time
from threading import Event



    



class regelungs_node(Node):
    def __init__(self):
        super().__init__('regelungs_node')

        self.arm_positions_sub = self.create_subscription(RobotPos, 'robot_arm_position', self.arm_position_callback, 10)
        
        self.object_data_sub = self.create_subscription(ObjectData, 'object_data', self.object_data_callback, 10)
        

        self.vel_sub = self.create_subscription(Float64, 'velocity', self.velocity_callback, 10)


        self.robot_command_pub = self.create_publisher(RobotCmd, 'robot_arm_commands', 10)

     
        self.robot_pos = {'x': 0, 'y': 0, 'z': 0}
        self.object_data = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.velocity = 0
        self.zero_position = {'x': 10, 'y': 10, 'z': 10}


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
        

        self.velo_zaehler = 0

        self.queue = []

       
        self.position_updated_event = Event()

        self.position_updated_event.wait()

        self.move_to_zero_position()
        self.target_position['x'] = self.default_pos['x']
        self.target_position['y'] = self.default_pos['y']
        self.target_position['z'] = self.default_pos['z']
        self.regler()

    async def move_to_zero_position(self):
        while abs(self.robot_pos['x'] - self.zero_position['x']) >= 0.1:
            self.zero_position['x'] = self.robot_pos['x']
            robot_cmd = RobotCmd()
            robot_cmd.accel_x = 0.1
            robot_cmd.accel_y = 0
            robot_cmd.accel_z = 0
            robot_cmd.activate_gripper = False
            self.robot_command_pub.publish(robot_cmd)
            await(500)

        while abs(self.robot_pos['y'] - self.zero_position['y']) >= 0.1:
            self.zero_position['y'] = self.robot_pos['y']
            robot_cmd = RobotCmd()
            robot_cmd.accel_x = 0
            robot_cmd.accel_y = -0.1
            robot_cmd.accel_z = 0
            robot_cmd.activate_gripper = False
            self.robot_command_pub.publish(robot_cmd)
            await(500)

        while abs(self.robot_pos['z'] - self.zero_position['z']) >= 0.1:
            self.zero_position['z'] = self.robot_pos['z']
            robot_cmd = RobotCmd()
            robot_cmd.accel_x = 0
            robot_cmd.accel_y = 0
            robot_cmd.accel_z = -0.1
            robot_cmd.activate_gripper = False
            self.robot_command_pub.publish(robot_cmd)
            await(500)
        
        

    

    def arm_position_callback(self, msg):
        self.robot_pos['x'] = msg.pos_x
        self.robot_pos['y'] = msg.pos_y
        self.robot_pos['z'] = msg.pos_z
        self.position_updated_event.set() 
        self.regler()

    def object_data_callback(self, msg):
        self.object_data[0][0] = msg.object_pos_x
        self.object_data[0][1] = msg.object_pos_y
        self.object_data[1][0] = msg.object_class
        self.object_data[2][0] = msg.timestamp_value
        self.object_data[3][0] = msg.index_value

        self.enqueue(self.object_data)
       
    

    def velocity_callback(self, msg):
        self.velocity = (self.velocity * self.velo_zaehler + msg.data) / (self.velo_zaehler + 1)
        self.velo_zaehler += 1

    
        

    def calculate_target_position(self):
        
        if(self.gripper_is_activated is True):
            if(self.oldest_object['class'] == 'cat'):
                self.target_position = self.box_cat
            elif(self.oldest_object['class'] == 'unicorn'):
                self.target_position = self.box_unicorn
            else:
                self.target_position = self.default_pos
        
        if( len(self.queue) != 0):
            self.target_position['x'] = self.oldest_object['class'] + self.velocity * (time.time() - self.oldest_object['timestamp'])
            self.target_position['y'] = self.oldest_object['class']
            self.target_position['z'] = self.pick_up_z


        else: 
            self.target_position = self.default_pos
        
        return self.target_position
       

    async def go_to_target_position(self):
       
        while((abs(self.target_position - self.robot_pos)) >= 0.5): # oder while(MOVING_STATE == True):
            self.regler()
            self.go_to_target_position()

        if(abs(self.target_position - self.box_cat) >= 0.5 or abs(self.target_position - self.box_unicorn) >= 0.5): # oder if(OVER_BOX):
            self.gripper_is_activated = False
            self.regler()
            self.go_to_target_position(self.default_pos)
        elif(abs(self.target_position - self.default_pos) >= 0.5): # oder elif(DEFAULT_STATE):
            await(5)
        else:
            if(abs(self.robot_pos['z'] - self.pick_up_z) >= 0.5): # oder if(PICK_UP_READY_STATE)
                self.gripper_is_activated = True
                self.regler()
                self.sort(self.oldest_object['class'])
            else:
                self.robot_pos['z'] = self.pick_up_z
                self.regler()
                self.calculate_target_position()
       
    def sort(self, oldest_object):
        if(self.gripper_is_activated): 
            if(oldest_object['class'] == 'cat'):
                self.go_to_target_position(self.box_cat)
                self.gripper_is_activated = False
                self.oldest_object = self.dequeue
            if(oldest_object['class'] ==  'unicorn'):
                self.go_to_target_position(self.box_unicorn)  
                self.gripper_is_activated = False  
                self.oldest_object = self.dequeue
                

    def regler(self):
       
        differenz_x = self.target_position['x'] - self.robot_pos['x']    
        differenz_y = self.target_position['y'] - self.robot_pos['y']  
        differenz_z = self.target_position['z'] - self.robot_pos['z']   

        current_time = time.time()
    
        dt = current_time - self.last_calculation_time
      
        self.last_calculation_time = current_time

        
        vel_x = self.compute_pd(differenz_x, self.last_error_x, dt)
        self.last_error_x = differenz_x
      
        
        vel_y = self.compute_pd(differenz_y, self.last_error_y, dt)
        self.last_error_y = differenz_y

        
        vel_z = self.compute_pd(differenz_z, self.last_error_z, dt)
        self.last_error_z = differenz_z
        
     
        robot_cmd = RobotCmd()
        robot_cmd.accel_x = vel_x
        robot_cmd.accel_y = vel_y
        robot_cmd.accel_z = vel_z
        robot_cmd.activate_gripper = self.gripper_is_activated
        self.robot_command_pub.publish(robot_cmd)
        
    def compute_pd(self, error, last_error, dt):
      
        derivative = (error - last_error) / dt
      
        control_signal = self.kp * error + self.kd * derivative

        control_signal = float(control_signal)
        
        return control_signal

    
    def enqueue(self, object_data):
        self.queue.append(object_data)
    
    def dequeue(self):
        if(len(self.queue) != 0):
            self.queue.pop(0)


def main(args=None):
    rclpy.init(args=args)
    node = regelungs_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



