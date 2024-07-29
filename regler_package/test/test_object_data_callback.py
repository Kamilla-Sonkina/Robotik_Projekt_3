import unittest
from unittest.mock import Mock, MagicMock
import rclpy
from regler_package.regler_node import regelungs_node
import pytest


class TestObjectDataCallback(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = regelungs_node()
        self.node.get_logger = Mock()
        self.node.enqueue = Mock()
        self.node.dequeue = Mock()
        self.node.state_machine = MagicMock()
        self.node.state_machine.states = {'idle': 'idle', 'default': 'default'}
        self.node.queue = []
        self.node.oldest_object = {'x': 0.0, 'timestamp': 0.0, 'index': -1}  # Initialize with default values
        self.node.user_target = False
        self.node.object_data = {}
        self.node.black_list_objects = set()
        self.node.last_object_pose = 0.0
        self.node.last_object_timestamp = 0.0
        self.node.object_velocity = 0.0

    def tearDown(self):
        self.node.destroy_node()


    def test_object_data_callback_queue_is_nonEmpty_expectedOutput_is_queueLengthIncreased(self):
        msg = self._create_msg(0.1, 0.1, 'class1', 0.1, 1)
        self.node.queue.append({'x': 0.178, 'y': 0.1, 'class': 'class2', 'timestamp': 0.2, 'index': 2})
        self.node.state_machine.current_state = 'idle'
        self.node.oldest_object = {'x': 0.0, 'timestamp': 0.0, 'index': -1}  # Ensure oldest_object is initialized
        self.node.object_data_callback(msg)
        expected_object = {'x': 0.1789652, 'y': 0.0999777, 'class': 'class1', 'timestamp': 0.1, 'index': 1}
        assert len(self.node.queue) == 1, f"Expected queue length to be 1, but got {len(self.node.queue)}"
        


    def test_object_data_callback_state_is_idle_expectedOutput_is_objectDequeued(self):
        msg = self._create_msg(0.1, 0.1, 'class1', 0.1, 1)
        self.node.state_machine.current_state = 'idle'
        self.node.oldest_object = {'x': 0.5, 'timestamp': 0.2, 'index': 1}
        self.node.object_data_callback(msg)
        assert self.node.dequeue.called

    def test_object_data_callback_state_is_nonIdle_expectedOutput_is_objectNotDequeued(self):
        msg = self._create_msg(0.1, 0.1, 'class1', 0.1, 1)
        self.node.state_machine.current_state = 'active'
        self.node.oldest_object = {'x': 0.5, 'timestamp': 0.2, 'index': 1}
        self.node.object_data_callback(msg)
        assert not self.node.dequeue.called

    def test_object_data_callback_userTarget_is_true_expectedOutput_is_userTargetSetToFalse(self):
        msg = self._create_msg(0.1, 0.1, 'class1', 0.1, 1)
        self.node.user_target = True
        self.node.oldest_object = {'x': 0.5, 'timestamp': 0.2, 'index': 1}
        self.node.object_data_callback(msg)
        assert self.node.user_target is False

    def test_object_data_callback_objectData_is_variousValues_expectedOutput_is_objectDataUpdatedCorrectly(self):
        msg = self._create_msg(0.2, 0.2, 'class2', 0.2, 2)
        self.node.object_data_callback(msg)
        assert self.node.object_data['x'] == pytest.approx(0.178952, rel=1e-2)
        assert self.node.object_data['y'] == pytest.approx(0.0999554, rel=1e-2)
        assert self.node.object_data['class'] == 'class2'
        assert self.node.object_data['timestamp'] == 0.2
        assert self.node.object_data['index'] == 2

    def test_object_data_callback_blackList_is_currentObject_expectedOutput_is_objectNotEnqueued(self):
        msg = self._create_msg(0.1, 0.1, 'class1', 0.1, 1)
        self.node.black_list_objects.add(1)
        self.node.object_data_callback(msg)
        assert not any(obj['index'] == 1 for obj in self.node.queue)

    def test_object_data_callback_lastObjectPose_is_withinTolerance_expectedOutput_is_velocityCalculatedCorrectly(self):
        msg = self._create_msg(0.1, 0.1, 'class1', 0.1, 1)
        self.node.last_object_pose = 0.178 + 0.0001
        self.node.oldest_object = {'x': 0.178, 'timestamp': 0.2, 'index': 1}
        self.node.object_data_callback(msg)
        assert self.node.object_velocity == pytest.approx((0.178 - 0.178) / (0.1 - 0.2), abs=1e-5)

    def _create_msg(self, x, y, cls, ts, idx):
        msg = MagicMock()
        msg.object_pos_x = x
        msg.object_pos_y = y
        msg.object_class = cls
        msg.timestamp_value = ts
        msg.index_value = idx
        return msg

if __name__ == '__main__':
    unittest.main()
