import unittest
from unittest.mock import MagicMock

class TestRobotController(unittest.TestCase):
    
    def setUp(self):
        self.controller = RobotController()
        self.controller.get_logger = MagicMock()
        self.controller.robot_command_pub = MagicMock()
        self.controller.update_state = MagicMock()
        
        # Initialize the state variables
        self.controller.pt2_state_x = [0, 0]
        self.controller.pt2_state_y = [0, 0]
        self.controller.pt2_state_z = [0, 0]
        self.controller.last_error_x = 0
        self.controller.last_error_y = 0
        self.controller.last_error_z = 0
        self.controller.last_derivative_x = 0
        self.controller.last_derivative_y = 0
        self.controller.last_derivative_z = 0
        self.controller.kp_x = 1.0
        self.controller.kd_x = 0.1
        self.controller.kp_y = 1.0
        self.controller.kd_y = 0.1
        self.controller.kp_z = 1.0
        self.controller.kd_z = 0.1
        self.controller.N = 10
        self.controller.target_position = {'x': 1, 'y': 1, 'z': 1}
        self.controller.robot_pos = {'x': 0.5, 'y': 0.5, 'z': 0.5}
        self.controller.current_time = 1.0
        self.controller.last_calculation_time = 0.0

    def test_compute_pd(self):
        error = 1.0
        last_error = 0.5
        last_derivative = 0.1
        dt = 0.1
        kp = 1.0
        kd = 0.1
        N = 10
        control_signal, filtered_derivative = self.controller.compute_pd(error, last_error, last_derivative, dt, kp, kd, N)
        
        expected_control_signal = kp * error + kd * ((error - last_error) / dt)
        self.assertAlmostEqual(control_signal, expected_control_signal, places=5)
        self.assertTrue(filtered_derivative < (error - last_error) / dt)

    def test_apply_pt2_filter(self):
        control_signal = 1.0
        dt = 0.1
        axis = 'x'
        filtered_signal = self.controller.apply_pt2_filter(control_signal, dt, axis)
        
        self.assertIsNotNone(filtered_signal)
        self.assertTrue(isinstance(filtered_signal, float))
        
    def test_regler(self):
        self.controller.regler()
        
        # Check that control signals are within bounds
        self.assertTrue(-0.3 <= self.controller.controll_u_x <= 0.3)
        self.assertTrue(-0.3 <= self.controller.controll_u_y <= 0.3)
        self.assertTrue(-0.3 <= self.controller.controll_u_z <= 0.3)
        
        # Check that the robot command is published
        self.controller.robot_command_pub.publish.assert_called_once()
        
        # Check that state is updated
        self.controller.update_state.assert_called_once()

if __name__ == '__main__':
    unittest.main()
