import unittest
import pytest
from unittest.mock import MagicMock, patch
import rclpy
from regler_package.regler_node import regelungs_node, State
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
        """Test initialization of the node."""
        self.assertEqual(self.node.get_name(), 'regelungs_node')
        self.assertIsNotNone(self.node.arm_positions_sub)
        self.assertIsNotNone(self.node.object_data_sub)
        self.assertIsNotNone(self.node.vel_sub)
        self.assertIsNotNone(self.node.robot_command_pub)

    def test_move_to_zero_position(self):
        """Test the move_to_zero_position method."""
        self.node.robot_pos = {'x': 1, 'y': 1, 'z': 1}
        self.node.move_to_zero_position()
        self.assertEqual(self.node.target_position, self.node.default_pos)
        self.assertNotEqual(self.node.zero_position, {'x': 1, 'y': 1, 'z': 1})

    def test_compute_pd(self):
        """Test the compute_pd method."""
        error = 6.0
        last_error = 2.0
        dt = 0.2
        kp = self.node.kp_x
        kd = self.node.kd_x

        expected_control_signal = kp * error + kd * (error - last_error) / dt
        control_signal = self.node.compute_pd(error, last_error, dt, kp, kd)

        self.assertAlmostEqual(control_signal, expected_control_signal)

    def test_sort(self):
        """Test the sort method."""
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
        """Test the enqueue and dequeue methods."""
        object_data = {'x': 2, 'y': 3, 'class': 'cat', 'timestamp': 456, 'index': 1}
        self.node.enqueue(object_data)
        self.assertEqual(len(self.node.queue), 1)
        self.node.dequeue()
        self.assertEqual(self.node.oldest_object, object_data)
        self.assertEqual(len(self.node.queue), 0)

    def test_calculate_target_position(self):
        """Test the calculate_target_position method."""
        self.node.gripper_is_activated = True
        self.node.oldest_object['timestamp'] = time.time()
        self.node.velocity = 0.2
        self.node.oldest_object['x'] = 12
        self.node.oldest_object['y'] = 15

        self.node.oldest_object['class'] = 'cat'
        self.node.calculate_target_position()
        self.assertEqual(self.node.target_position, self.node.box_cat)

        self.node.gripper_is_activated = False
        self.node.calculate_target_position()
        expected_target_x = self.node.oldest_object['x'] + self.node.velocity * (time.time() - self.node.oldest_object['timestamp'])
        expected_target_y = self.node.oldest_object['y']
        expected_target_z = self.node.ready_to_pick_up_z
        self.assertAlmostEqual(self.node.target_position['x'], expected_target_x, places=5)
        self.assertEqual(self.node.target_position['y'], expected_target_y)
        self.assertEqual(self.node.target_position['z'], expected_target_z)

        self.node.oldest_object = {'x': None, 'y': None, 'class': None, 'timestamp': None, 'index': None}
        self.node.calculate_target_position()
        self.assertEqual(self.node.target_position, self.node.default_pos)

    def test_regler(self):
        """Test the regler method."""
        self.node.robot_pos = {'x': 1.0, 'y': 2.0, 'z': 3.0}
        self.node.target_position = {'x': 4.0, 'y': 5.0, 'z': 6.0}
        self.node.last_error_x = 0
        self.node.last_error_y = 0
        self.node.last_error_z = 0
        self.node.last_calculation_time = time.time()

        expected_differenz_x = self.node.target_position['x'] - self.node.robot_pos['x']
        expected_differenz_y = self.node.target_position['y'] - self.node.robot_pos['y']
        expected_differenz_z = self.node.target_position['z'] - self.node.robot_pos['z']
        expected_dt = 1.0

        expected_vel_x = self.node.compute_pd(expected_differenz_x, self.node.last_error_x, expected_dt, self.node.kp_x, self.node.kd_x)
        expected_vel_y = self.node.compute_pd(expected_differenz_y, self.node.last_error_y, expected_dt, self.node.kp_y, self.node.kd_y)
        expected_vel_z = self.node.compute_pd(expected_differenz_z, self.node.last_error_z, expected_dt, self.node.kp_z, self.node.kd_z)

        self.node.regler()

        self.assertAlmostEqual(self.node.controll_u_x, expected_vel_x, places=5)
        self.assertAlmostEqual(self.node.controll_u_y, expected_vel_y, places=5)
        self.assertAlmostEqual(self.node.controll_u_z, expected_vel_z, places=5)

    def test_go_to_target_position(self):
        """Test the go_to_target_position method."""
        self.node.state = State.Moving_to_object
        self.node.target_position = {'x': 1.0, 'y': 2.0, 'z': 3.0}
        self.node.robot_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        # Patch the regler method to monitor its calls
        with patch.object(self.node, 'regler', wraps=self.node.regler) as mock_regler:
            self.node.go_to_target_position()
            # Assert that regler was called during go_to_target_position
            assert mock_regler.called

        # Test the Over_Box state
        self.node.state = State.Over_Box
        self.node.go_to_target_position()
        self.assertFalse(self.node.gripper_is_activated)

        # Test the Ready_to_pick_up state
        self.node.state = State.Ready_to_pick_up
        self.node.go_to_target_position()
        self.assertTrue(self.node.gripper_is_activated)

        # Test the Default state
        self.node.state = State.Default
        self.node.go_to_target_position()
        assert not mock_regler.called

    def test_update_state(self):
        """Test the update_state method."""
        self.node.robot_pos = {'x': 10, 'y': 11, 'z': 12}
        self.node.box_cat = {'x': 10, 'y': 11, 'z': 12}
        self.node.update_State()
        self.assertEqual(self.node.state, State.Over_Box)

        self.node.gripper_is_activated = True
        self.node.target_position = {'x': 10, 'y': 11, 'z': self.node.pick_up_z}
        self.node.update_State()
        self.assertEqual(self.node.state, State.Sorting)

        self.node.gripper_is_activated = False
        self.node.target_position = {'x': 10, 'y': 11, 'z': self.node.ready_to_pick_up_z}
        self.node.update_State()
        self.assertEqual(self.node.state, State.Ready_to_pick_up)

        self.node.robot_pos = self.node.default_pos
        self.node.update_State()
        self.assertEqual(self.node.state, State.Default)

if __name__ == '__main__':
    pytest.main()
