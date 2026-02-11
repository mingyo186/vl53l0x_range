# Copyright 2025 The vl53l0x_range Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Launch test for VL53L0X time-of-flight range sensor node."""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Range
from std_srvs.srv import Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch VL53L0X node in fake_mode for testing."""
    node = launch_ros.actions.Node(
        package='vl53l0x_range',
        executable='vl53l0x_node.py',
        name='vl53l0x_range_node',
        parameters=[{
            'fake_mode': True,
            'publish_rate': 50.0,
        }],
    )
    return launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ]), {'sensor_node': node}


class TestVL53L0XTopics(unittest.TestCase):
    """Verify range topic publishes valid data."""

    def test_range_topic_published(self):
        """Range data topic should receive messages."""
        topic_list = [('range/data', Range)]
        with WaitForTopics(topic_list, timeout=10.0) as wait:
            self.assertEqual(wait.topics_received(), {'range/data'})

    def test_range_data_valid(self):
        """Range message should have valid frame_id and radiation_type."""
        topic_list = [('range/data', Range)]
        with WaitForTopics(
            topic_list, timeout=10.0, messages_received_buffer_length=5
        ) as wait:
            msgs = wait.received_messages('range/data')
            self.assertGreater(len(msgs), 0)
            for msg in msgs:
                self.assertEqual(msg.header.frame_id, 'tof_link')
                self.assertEqual(msg.radiation_type, Range.INFRARED)
                self.assertGreater(msg.range, 0.0)
                self.assertLess(msg.range, 3.0)


class TestVL53L0XServices(unittest.TestCase):
    """Verify calibrate and reset services."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_vl53l0x_services')

    def tearDown(self):
        self.node.destroy_node()

    def test_calibrate_service(self):
        """Calibrate should return success in fake mode."""
        client = self.node.create_client(Trigger, 'range/calibrate')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().success)
        self.assertIn('fake', future.result().message.lower())
        self.node.destroy_client(client)

    def test_reset_service(self):
        """Reset should return success."""
        client = self.node.create_client(Trigger, 'range/reset')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().success)
        self.assertIn('reset complete', future.result().message.lower())
        self.node.destroy_client(client)


class TestVL53L0XParameters(unittest.TestCase):
    """Verify runtime parameter changes."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_vl53l0x_params')

    def tearDown(self):
        self.node.destroy_node()

    def test_change_publish_rate(self):
        """Publish_rate should be changeable at runtime."""
        client = self.node.create_client(
            SetParameters, 'vl53l0x_range_node/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        request = SetParameters.Request()
        request.parameters = [
            Parameter('publish_rate', value=20.0).to_parameter_msg(),
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().results[0].successful)
        self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_code(self, proc_info):
        """Node should exit cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
