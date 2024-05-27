import rclpy
from rclpy.node import Node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from object_interfaces.msg import ObjectData
from std_msgs.msg import Float64
import time
import asyncio
from enum import Enum

class State(Enum):
    INITIALISIERUNG = 0
    IDLE = 1
    ZIEL = 2
    PICK_UP = 3
    LOS = 4
    DEFAULT = 5
    EMERGENCY = 6

class RegelungsNode(Node):
    def __init__(self):
        super().__init__('regelungs_node')

        self.state = State.INITIALISIERUNG
        self.arm_positions_sub = self.create_subscription(RobotPos, 'robot_arm_position', self.arm_position_callback, 10)
        self.object_data_sub = self.create_subscription(ObjectData, 'object_data', self.object_data_callback, 10)
        self.vel_sub = self.create_subscription(Float64, 'velocity', self.velocity_callback, 10)
        self.robot_command_pub = self.create_publisher(RobotCmd, 'robot_arm_commands', 10)

        self.robot_pos = {'x': 0, 'y': 0, 'z': 0}
        self.object_data = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.velocity = 0
        self.zero_position = {'x': 10, 'y': 10, 'z': 10}

        self.box_unicorn = self._adjust_box_position({'x': 10, 'y': 11, 'z': 12})
        self.box_cat = self._adjust_box_position({'x': 10, 'y': 11, 'z': 12})
        self.default_pos = self._adjust_box_position({'x': 10, 'y': 11, 'z': 12})
        self.safe_pos = self._adjust_box_position({'x': 10, 'y': 11, 'z': 12})
        self.pick_up_z = self.zero_position['z'] + 13

        self.gripper_is_activated = False
        self.target_position = self.default_pos
        self.corner = {'x': -10000, 'y': -10000, 'z': -1000}

        self.last_calculation_time = time.time()
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_z = 0
        self.kp = 9.85199
        self.kd = 6.447857
        self.velo_zaehler = 0
        self.queue = []
        self.emergency = False

        self.move_to_zero_position()

        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.go_to_target_position())

    def _adjust_box_position(self, box):
        box['x'] += self.zero_position['x']
        box['y'] += self.zero_position['y']
        box['z'] += self.zero_position['z']
        return box

    def arm_position_callback(self, msg):
        self.robot_pos['x'] = msg.pos_x
        self.robot_pos['y'] = msg.pos_y
        self.robot_pos['z'] = msg.pos_z

    def object_data_callback(self, msg):
        self.object_data['x'] = msg.object_pos_x
        self.object_data['y'] = msg.object_pos_y
        self.object_data['class'] = msg.object_class
        self.object_data['timestamp'] = msg.timestamp_value
        self.object_data['index'] = msg.index_value
        self.enqueue(self.object_data)

    def velocity_callback(self, msg):
        self.velocity = (self.velocity * self.velo_zaehler + msg.data) / (self.velo_zaehler + 1)
        self.velo_zaehler += 1

    def move_to_zero_position(self):
        self.target_position = self.corner
        while abs(self.robot_pos['x'] - self.zero_position['x']) > 0:
            self.zero_position['x'] = self.robot_pos['x']
            self.regler()
        self.target_position = self.default_pos
        self.state = State.IDLE

    async def go_to_target_position(self):
        while True:
            if self.emergency:
                self.state = State.EMERGENCY

            if self.state == State.INITIALISIERUNG:
                self.initialisierung()
                self.state = State.IDLE

            elif self.state == State.IDLE:
                if self.queue:
                    self.calculate_target_position()
                    self.state = State.ZIEL

            elif self.state == State.ZIEL:
                self.regler()
                if self._is_at_target_position():
                    self.state = State.PICK_UP

            elif self.state == State.PICK_UP:
                self.gripper_is_activated = True
                self.regler()
                self.sort(self.oldest_object['class'])
                self.calculate_target_position()
                self.state = State.LOS

            elif self.state == State.LOS:
                self.regler()
                if self._is_at_target_position():
                    self.gripper_is_activated = False
                    self.state = State.DEFAULT

            elif self.state == State.DEFAULT:
                self.target_position = self.default_pos
                self.regler()
                if self._is_at_target_position():
                    self.state = State.IDLE

            elif self.state == State.EMERGENCY:
                self.emergency_state()
                if not self.emergency:
                    self.state = State.IDLE

            await asyncio.sleep(0.1)

    def initialisierung(self):
        self.target_position['x'] = self.robot_pos['x'] - 1
        self.target_position['y'] = self.robot_pos['y'] - 1
        self.target_position['z'] = self.robot_pos['z'] - 1
        self.regler()
        time.sleep(1)

    def idle_state(self):
        pass

    def ziel_state(self):
        pass

    def pick_up_state(self):
        pass

    def los_state(self):
        pass

    def default_state(self):
        pass

    def emergency_state(self):
        self.target_position = self.safe_pos
        self.regler()
        while self.emergency:
            self.regler()
            time.sleep(0.1)

    def _is_at_target_position(self):
        return abs(self.target_position['x'] - self.robot_pos['x']) < 0.5 and \
               abs(self.target_position['y'] - self.robot_pos['y']) < 0.5 and \
               abs(self.target_position['z'] - self.robot_pos['z']) < 0.5

    def calculate_target_position(self):
        if self.gripper_is_activated:
            if self.oldest_object['class'] == 'cat':
                self.target_position = self.box_cat
            elif self.oldest_object['class'] == 'unicorn':
                self.target_position = self.box_unicorn
            else:
                self.target_position = self.default_pos

        if len(self.queue) != 0:
            self.target_position = {
                'x': self.oldest_object['x'] + self.velocity * (time.time() - self.oldest_object['timestamp']),
                'y': self.oldest_object['y'],
                'z': self.pick_up_z
            }
        else:
            self.target_position = self.default_pos
        return self.target_position

    def sort(self, oldest_object):
        if self.gripper_is_activated:
            if oldest_object == 'cat':
                self.target_position = self.box_cat
                self.gripper_is_activated = False
                self.oldest_object = self.dequeue()
            elif oldest_object == 'unicorn':
                self.target_position = self.box_unicorn
                self.gripper_is_activated = False
                self.oldest_object = self.dequeue()

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
        return float(control_signal)

    def enqueue(self, object_data):
        self.queue.append(object_data)

    def dequeue(self):
        if len(self.queue) != 0:
            return self.queue.pop(0)
        return None

    def emergency_case(self, Fehlermeldung):
        self.get_logger().info('Fehler: ' + Fehlermeldung)
        self.emergency = True
        self.state = State.EMERGENCY

def main(args=None):
    rclpy.init(args=args)
    node = RegelungsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
