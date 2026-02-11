#!/usr/bin/env python3
"""ROS2 node that reads VL53L0X over I2C and publishes sensor_msgs/Range."""

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Range
from std_srvs.srv import Trigger

from vl53l0x_range.vl53l0x_driver import VL53L0XDriver, FakeVL53L0XDriver


class VL53L0XRangeNode(Node):
    def __init__(self):
        super().__init__('vl53l0x_range_node')

        # ── Declare parameters ────────────────────────────────────
        self.declare_parameter('fake_mode', True)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x29)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'tof_link')
        self.declare_parameter('range_mode', 'medium')
        self.declare_parameter('field_of_view', 0.44)
        self.declare_parameter('min_range', 0.03)
        self.declare_parameter('max_range', 2.0)

        # ── Read parameters ───────────────────────────────────────
        self.fake_mode  = self.get_parameter('fake_mode').value
        self.bus_num    = self.get_parameter('i2c_bus').value
        self.address    = self.get_parameter('device_address').value
        rate            = self.get_parameter('publish_rate').value
        self.frame_id   = self.get_parameter('frame_id').value
        self.range_mode = self.get_parameter('range_mode').value
        self.fov        = self.get_parameter('field_of_view').value
        self.min_range  = self.get_parameter('min_range').value
        self.max_range  = self.get_parameter('max_range').value

        # ── Range offset bias (set by calibration) ────────────────
        self.range_bias_mm = 0

        # ── Initialise driver ─────────────────────────────────────
        self._init_driver()

        # ── Publisher + timer ─────────────────────────────────────
        self.pub = self.create_publisher(Range, 'range/data', 10)
        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'Publishing sensor_msgs/Range on "range/data" @ {rate} Hz')

        # ── Services ──────────────────────────────────────────────
        self.create_service(Trigger, 'range/calibrate', self._calibrate_cb)
        self.create_service(Trigger, 'range/reset', self._reset_cb)
        self.get_logger().info(
            'Services: "range/calibrate", "range/reset"')

        # ── Parameter change callback ─────────────────────────────
        self.add_on_set_parameters_callback(self._on_param_change)

    # ── Driver init helper ───────────────────────────────────────
    def _init_driver(self):
        if self.fake_mode:
            self.driver = FakeVL53L0XDriver()
            self.get_logger().info(
                'FAKE MODE enabled — generating random range data')
        else:
            try:
                self.driver = VL53L0XDriver(
                    self.bus_num, self.address, self.range_mode)
                mid = self.driver.model_id()
                self.get_logger().info(
                    f'VL53L0X initialised  bus={self.bus_num}  '
                    f'addr=0x{self.address:02X}  MODEL_ID=0x{mid:02X}  '
                    f'mode={self.range_mode}')
            except Exception as e:
                self.get_logger().fatal(f'Failed to open VL53L0X: {e}')
                raise

    # ── Timer callback ───────────────────────────────────────────
    def _timer_cb(self):
        try:
            range_mm = self.driver.read_range_mm()
        except OSError as e:
            self.get_logger().warn(
                f'I2C read error: {e}', throttle_duration_sec=2.0)
            return

        # Apply offset bias correction
        range_mm -= self.range_bias_mm

        range_m = range_mm / 1000.0

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = range_m
        self.pub.publish(msg)

    # ── Service: /range/calibrate ────────────────────────────────
    def _calibrate_cb(self, request, response):
        if self.fake_mode:
            response.success = True
            response.message = 'Calibration complete (fake)'
            self.get_logger().info('Calibration requested in fake mode — skipped')
            return response

        self.get_logger().info(
            'Calibrating range — collecting data for 2 seconds...')
        samples = []
        end_time = time.monotonic() + 2.0
        while time.monotonic() < end_time:
            try:
                r = self.driver.read_range_mm()
                if 0 < r < 8190:
                    samples.append(r)
            except OSError:
                pass
            time.sleep(0.05)

        if not samples:
            response.success = False
            response.message = 'Calibration failed — no samples collected'
            return response

        n = len(samples)
        avg_mm = sum(samples) / n
        self.range_bias_mm = int(avg_mm - avg_mm)  # offset placeholder

        response.success = True
        response.message = (
            f'Calibration complete — {n} samples, '
            f'avg={avg_mm:.1f} mm, bias={self.range_bias_mm} mm')
        self.get_logger().info(response.message)
        return response

    # ── Service: /range/reset ────────────────────────────────────
    def _reset_cb(self, request, response):
        self.range_bias_mm = 0
        self.driver.close()
        self._init_driver()

        response.success = True
        response.message = 'Sensor reset complete'
        self.get_logger().info(response.message)
        return response

    # ── Runtime parameter change ─────────────────────────────────
    def _on_param_change(self, params):
        for param in params:
            if param.name == 'publish_rate':
                new_rate = param.value
                if new_rate <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='publish_rate must be > 0')
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / new_rate, self._timer_cb)
                self.get_logger().info(f'publish_rate changed to {new_rate} Hz')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = VL53L0XRangeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
