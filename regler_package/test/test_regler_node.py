
import unittest
from unittest.mock import MagicMock, patch
import rclpy
from rclpy.node import Node
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
        #self.regler_subscriber = self.node.create_subscription(RobotCmd, 'robot_command', self.acceleration_callback, 10)    
        self.object_data_pub = self.node.create_publisher(ObjectData, 'object_data', 10)
        self.acceleration_x = 0
        self.acceleration_y = 0
        self.acceleration_z = 0

    def tearDown(self):
        self.node.destroy_node()

    """def acceleration_callback(self, msg):
        self.acceleration_x = msg.accel_x 
        self.node.get_logger().info(self.acceleration_x) 
        self.acceleration_y = msg.accel_y
        self.node.get_logger().info(self.acceleration_y) 
        self.acceleration_z = msg.accel_z
        self.node.get_logger().info(self.acceleration_z) 

    def test_init(self):
        self.assertEqual(self.node.get_name(), 'regelungs_node')
        self.assertIsNotNone(self.node.arm_positions_sub)
        self.assertIsNotNone(self.node.object_data_sub)
        self.assertIsNotNone(self.node.vel_sub)
        self.assertIsNotNone(self.node.robot_command_pub)

    def test_move_to_zero_position(self):
        self.node.robot_pos = {'x': 10, 'y': 10, 'z': 10}
        self.node.move_to_zero_position()
        self.assertNotEqual(self.node.zero_position, {'x': 0, 'y': 0, 'z': 0})

    def test_compute_pd(self):
        error = 1.0
        last_error = 0.5
        dt = 1.0
        kp = 4.0
        kd = 6.447857
        expected_pd = kp * error + kd * (error - last_error) / dt
        expected_pd = max(min(expected_pd, 0.3), -0.3)
        self.assertAlmostEqual(self.node.compute_pd(error, last_error, dt, kp, kd), expected_pd)
"""
    def test_sort(self):
        self.node.gripper_is_activated = True
        self.node.oldest_object = {'x': 0, 'y': 0, 'class': 'cat', 'timestamp': time.time(), 'index':0}
        
        self.node.sort(self.node.oldest_object)
        self.assertEqual(self.node.target_position, self.node.box_cat)
        
        self.node.oldest_object = {'x': 0, 'y': 0, 'class': 'unicorn', 'timestamp': time.time(), 'index':0}
        
        self.node.sort(self.node.oldest_object)
        self.assertEqual(self.node.target_position, self.node.box_unicorn)

    """ def test_enqueue_dequeue(self):
        object_data = {'x': 1, 'y': 2, 'class': 'cat', 'timestamp': 123, 'index': 0}
        self.node.enqueue(object_data)
        self.assertEqual(len(self.node.queue), 1)
        self.node.dequeue()
        self.assertEqual(self.node.oldest_object, object_data)
        self.assertEqual(len(self.node.queue), 0)"""

    """def test_calculate_target_position(self):
        self.node.state = State.Moving_to_object
        self.node.gripper_is_activated = True
        self.node.oldest_object = {'x': 0, 'y': 0, 'class': 'cat', 'timestamp': time.time(), 'index':0}
        self.node.calculate_target_position()
        self.assertEqual(self.node.target_position, self.node.box_cat)
        
        self.node.gripper_is_activated = False
        self.node.oldest_object = {'x': 1.0, 'y': 2.0, 'timestamp': time.time(), 'class': 'unicorn'}
        self.node.calculate_target_position()
        expected_x = self.node.oldest_object['x'] + self.node.velocity * (time.time() - self.node.oldest_object['timestamp'])
        self.assertAlmostEqual(self.node.target_position['x'], expected_x)"""

    """@patch('time.time', return_value=1000000.0)
    def test_regler(self, mock_time):
        self.node.robot_pos = {'x': 1.0, 'y': 2.0, 'z': 3.0}
        self.node.target_position = {'x': 1.01, 'y': 1.99, 'z': 3.1}
        self.node.last_error_x = 0
        self.node.last_error_y = 0
        self.node.last_error_z = 0
    
        self.node.last_calculation_time = time.time()
        
    
        self.node.regler()
    
        expected_differenz_x = self.node.target_position['x'] - self.node.robot_pos['x']
        expected_differenz_y = self.node.target_position['y'] - self.node.robot_pos['y']
        expected_differenz_z = self.node.target_position['z'] - self.node.robot_pos['z']
        expected_dt = 1.0
    
        expected_vel_x = self.node.compute_pd(expected_differenz_x, self.node.last_error_x, expected_dt, self.node.kp_x, self.node.kd_x)
        expected_vel_y = self.node.compute_pd(expected_differenz_y, self.node.last_error_y, expected_dt, self.node.kp_y, self.node.kd_y)
        expected_vel_z = self.node.compute_pd(expected_differenz_z, self.node.last_error_z, expected_dt, self.node.kp_z, self.node.kd_z)

        expected_vel_x = max(min(expected_vel_x, 0.3), -0.3)
        expected_vel_y = max(min(expected_vel_y, 0.3), -0.3)
        expected_vel_z = max(min(expected_vel_z, 0.3), -0.3)

        self.assertAlmostEqual(self.node.controll_u_x, expected_vel_x)
        self.assertAlmostEqual(self.node.controll_u_y, expected_vel_y)
        self.assertAlmostEqual(self.node.controll_u_z, expected_vel_z)

        

    def test_go_to_target_position(self):
        self.node.target_position = {'x': 5.0, 'y': 5.0, 'z': 5.0}
        self.node.robot_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}

        with patch.object(self.node, 'regler') as mock_regler:
            self.node.go_to_target_position()
            mock_regler.assert_called()

    def test_emergency_case(self):
        expected_pose = self.node.safe_pos
        self.node.emergency_case('Test')
        self.assertEqual(self.node.target_position, expected_pose)
        self.assertTrue(self.node.emergency)
        self.assertTrue(self.node.safe_mode)
        self.assertEqual(self.node.state, State.Emergency)

    def test_full_automation(self):
        # This would be a full integration test involving the whole flow
        pass"""

if __name__ == '__main__':
    unittest.main()

