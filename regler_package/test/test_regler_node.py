import asyncio
import unittest

from unittest.mock import MagicMock, patch

import pytest
import rclpy
from rclpy.node import Node
from regler_package.regler_node import regelungs_node
from ro45_portalrobot_interfaces.msg import RobotPos, RobotCmd
from object_interfaces.msg import ObjectData
from std_msgs.msg import Float64
import time

class TestRegelungsNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = regelungs_node()
        self.node.robot_command_pub.publish = MagicMock()
        self.regler_subscriber = self.node.create_subscription(RobotCmd, 'acceleration', self.acceleration_callback, 10)    
        self.object_data_pub = self.node.create_publisher(ObjectData, 'object_data', 10)
        self.acceleration_x = 0
        self.acceleration_y = 0
        self.acceleration_z = 0
        self.target_object = {'x': 10, 'y': 10, 'class': 'cat', 'timestamp': time.time, 'index': 1}

    def tearDown(self):
        self.node.destroy_node()

    """
    Given a newly turned on pick-up-robot and it's initialised the node name should be 'regelungsnode'
    and all subscriptions and publisher should be not None
    """
    def test_init(self):
        self.assertEqual(self.node.get_name(), 'regelungs_node')
        self.assertIsNotNone(self.node.arm_positions_sub)
        self.assertIsNotNone(self.node.object_data_sub)
        self.assertIsNotNone(self.node.vel_sub)
        self.assertIsNotNone(self.node.robot_command_pub)

    """
    Given a newly turned on pick-up-robot in the initialising prozess the method move_to_zero
    should determine the zero position and set afterwards the target position to default position
    """

    def test_move_to_zero_position(self):
        self.node.robot_pos = {'x': 0, 'y': 0, 'z': 0}
        self.node.move_to_zero_position()
        self.assertEqual(self.node.target_position, self.node.default_pos)
        self.assertNotEqual(self.node.zero_position, {'x': 0, 'y': 0, 'z': 0})

    """
    given a pick-up-robot and the controller is called it needs to compute pd.
    Pd should be computed by the conrolling fuormular
    """
    def test_compute_pd(self):
        error = 5.0
        last_error = 3.0
        dt = 0.1
        kp = self.node.kp
        kd = self.node.kd

        expected_control_signal = kp * error + kd * (error - last_error) / dt
        control_signal = self.node.compute_pd(error, last_error, dt)

        self.assertAlmostEqual(control_signal, expected_control_signal)

    """
    Given a pick-up-robot and an object and the method sort is called
    it should check wheter the gripper is activated or not. If the gripper is activated
    it should set its target pos to the box of the objects class
    """

    def test_sort(self):
        self.node.oldest_object['class'] = 'cat'
        self.node.robot_pos = self.node.box_cat
        self.node.sort(self.node.oldest_object)
        self.assertEqual(self.node.target_position, self.node.box_cat)
        self.assertFalse(self.node.gripper_is_activated)

        self.node.oldest_object['class'] = 'unicorn'
        self.node.robot_pos = self.node.box_unicorn
        self.node.sort(self.node.oldest_object)
        self.assertEqual(self.node.target_position, self.node.box_unicorn)
        self.assertFalse(self.node.gripper_is_activated)

    """
    Given a pick-up-robot an a new object is detected it should enqueue the object. The queue should 
    have the length one.
    It should  dequeue it and make it to the oldest object.
    """

    def test_enqueue_dequeue(self):
        object_data = {'x': 1, 'y': 2, 'class': 'cat', 'timestamp': 123, 'index': 0}
        self.node.enqueue(object_data)
        self.assertEqual(len(self.node.queue), 1)
        self.node.dequeue()
        self.assertEqual(self.node.oldest_object, object_data)
        self.assertEqual(len(self.node.queue), 0)

    """
    Given a pick-up-robot and the method calculate target position is called
    it should calculate the target position. If the gripper is activated
    the target position for the cat object should be box_cat
    and for the unicorn the target position should be box_unicorn. 
    If the gripper is deactivated and there is an oldest object
    the target position should be the position of the object.
    when there is no oldest object the target position should be the default position
    """

    def test_calculate_target_position(self):
        self.node.gripper_is_activated = True
        self.node.oldest_object['timestamp'] = time.time()
        self.node.velocity = 0.1
        self.node.oldest_object['x'] = 10
        self.node.oldest_object['y'] = 10

        self.node.oldest_object['class'] = 'cat'
        self.node.calculate_target_position()
        self.assertEqual(self.node.target_position, self.node.box_cat)

        self.controll_target_position = {'x': None, 'y': None, 'z': None}
        self.node.gripper_is_activated = False
        self.node.calculate_target_position()
        self.controll_target_position['x'] = self.node.oldest_object['x'] + self.node.velocity * (time.time() - self.node.oldest_object['timestamp'])
        self.controll_target_position['y'] = self.node.oldest_object['y']
        self.controll_target_position['z'] = self.node.pick_up_z
        self.assertAlmostEqual(self.controll_target_position['x'], self.node.target_position['x'], places = 5)
        self.assertEqual(self.controll_target_position['y'], self.node.target_position['y'])
        self.assertEqual(self.controll_target_position['z'], self.node.target_position['z'])

        self.node.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.node.calculate_target_position()
        self.assertEqual(self.node.target_position, self.node.default_pos)

    
    @pytest.mark.asyncio
    async def test_regler(self):
    

        
        self.node.robot_pos = {'x': 1.0, 'y': 2.0, 'z': 3.0}
        self.node.target_position = {'x': 4.0, 'y': 5.0, 'z': 6.0}
        self.node.last_error_x = 0
        self.node.last_error_y = 0
        self.node.last_error_z = 0
        
        self.node.last_calculation_time = time.time()

        await self.node.regler()
        
        
        
        expected_differenz_x = self.node.target_position['x'] - self.node.robot_pos['x']
        expected_differenz_y = self.node.target_position['y'] - self.node.robot_pos['y']
        expected_differenz_z = self.node.target_position['z'] - self.node.robot_pos['z']
        expected_dt = 1.0

        expected_vel_x = self.node.compute_pd(expected_differenz_x, self.node.last_error_x, expected_dt)
        expected_vel_y = self.node.compute_pd(expected_differenz_y, self.node.last_error_y, expected_dt)
        expected_vel_z = self.node.compute_pd(expected_differenz_z, self.node.last_error_z, expected_dt)

        while(self.acceleration_x == 0):
            await asyncio.sleep(0.005)


        self.assertAlmostEqual(self.acceleration_x, expected_vel_x)
        self.assertAlmostEqual(self.acceleration_y, expected_vel_y)
        self.assertAlmostEqual(self.acceleration_z, expected_vel_z)

    def acceleration_callback(self, msg):
        self.acceleration_x = msg.accel_x
        self.acceleration_y = msg.accel_y
        self.acceleration_z = msg.accel_z        

    @pytest.mark.asyncio
    async def test_go_to_target_position(self):
        self.node.target_position = {'x': 5.0, 'y': 5.0, 'z': 5.0}
        self.node.robot_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        with patch.object(self.node, 'regler') as mock_regler:
            await self.node.go_to_target_position()
            self.assertGreaterEqual(self.node.robot_pos['x'], self.node.target_position['x'])
            self.assertGreaterEqual(self.node.robot_pos['y'], self.node.target_position['y'])
            self.assertGreaterEqual(self.node.robot_pos['z'], self.node.target_position['z'])
            mock_regler.assert_called()
        
    
    async def test_go_to_target_position(self):
        await self.node.go_to_target_position()
        excepted_pose = self.node.safe_pos
        self.node.emergency_case('Test')
        self.assertEqual(self.node.target_position, excepted_pose)

    def test_full_automation(self):
        self.target_object = ObjectData()
        self.target_object.object_pos_x = 10
        self.target_object.object_pos_y = 10
        self.target_object.object_class = 'cat'
        self.target_object.timestamp_value = time.time
        self.target_object.index_value = 1
        self.object_data_pub.publish(self.target_object)
        self.assertEqual(self.target_object['x'], self.node.target_position['x'])
        self.assertEqual(self.target_object['y'], self.node.target_position['y'])
        self.node.robot_pos['z'] = self.node.ready_to_pick_up_z
        
        self.assertEqual(self.node.target_position['x'], self.node.box_cat['x'])
        self.assertEqual(self.node.target_position['y'], self.node.box_cat['y'])
        self.assertEqual(self.node.target_position['z'], self.node.box_cat['z'])
        self.node.robot_pos['x'] = self.node.target_position['x']
        self.node.robot_pos['y'] = self.node.target_position['y']
        self.node.robot_pos['z'] = self.node.target_position['z']


        
        self.assertEqual(self.node.target_position['x'], self.target_object['x'])
        self.assertEqual(self.node.target_position['y'], self.target_object['y'])

if __name__ == '__main__':
    unittest.main()
