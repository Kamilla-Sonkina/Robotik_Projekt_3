import unittest
from unittest.mock import MagicMock
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

    def tearDown(self):
        self.node.destroy_node()

    def test_init(self):
        self.assertEqual(self.node.get_name(), 'regelungs_node')
        self.assertIsNotNone(self.node.arm_positions_sub)
        self.assertIsNotNone(self.node.object_data_sub)
        self.assertIsNotNone(self.node.vel_sub)
        self.assertIsNotNone(self.node.robot_command_pub)

    def test_move_to_zero_position(self):
        self.node.robot_pos = {'x': 0, 'y': 0, 'z': 0}
        self.node.move_to_zero_position()
        self.assertEqual(self.node.target_position, self.node.default_pos)
        self.assertNotEqual(self.node.zero_position, {'x': 0, 'y': 0, 'z': 0})

    def test_compute_pd(self):
        error = 5.0
        last_error = 3.0
        dt = 0.1
        kp = self.node.kp
        kd = self.node.kd

        expected_control_signal = kp * error + kd * (error - last_error) / dt
        control_signal = self.node.compute_pd(error, last_error, dt)

        self.assertAlmostEqual(control_signal, expected_control_signal)

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

    def test_enqueue_dequeue(self):
        object_data = {'x': 1, 'y': 2, 'class': 'cat', 'timestamp': 123, 'index': 0}
        self.node.enqueue(object_data)
        self.assertEqual(len(self.node.queue), 1)
        self.node.dequeue()
        self.assertEqual(len(self.node.queue), 0)

    def test_calculate_target_position(self):
        self.node.gripper_is_activated = True
        self.node.oldest_object['timestamp'] = time.time()
        self.node.velocity = 0.1
        self.node.oldest_object['x'] = 10
        self.node.oldest_object['y'] = 10

        self.node.oldest_object['class'] = 'cat'
        self.node.calculate_target_position()
        self.assertEqual(self.node.target_position, self.node.box_cat)

        self.node.gripper_is_activated = False
        self.node.queue.append({'x': 1, 'y': 2, 'class': 'unicorn', 'timestamp': 123, 'index': 0})
        self.node.oldest_object = self.node.queue[0]
        self.node.calculate_target_position()
        self.assertNotEqual(self.node.target_position, self.node.default_pos)

    def test_regler(self):
        self.node.target_position = {'x': 1, 'y': 2, 'z': 3}
        self.node.robot_pos = {'x': 0, 'y': 0, 'z': 0}
        self.node.regler()
        self.node.robot_command_pub.publish.assert_called()

if __name__ == '__main__':
    unittest.main()
