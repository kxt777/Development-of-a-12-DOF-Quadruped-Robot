"""Passive USB reader and complete sensor_msgs/Imu publisher."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import serial

from .protocol import ImuParser, SampleAssembler


class DmImuNode(Node):
    def __init__(self):
        super().__init__("dog_imu")
        for name, default in {
            "port": "/dev/ttyACM1", "baudrate": 921600, "slave_id": 1,
            "frame_id": "dog_imu_link", "imu_topic": "/dog_imu/data",
            "status_topic": "/dog_imu/status", "poll_period": 0.005,
            "max_sample_age": 0.1, "max_sample_skew": 0.03,
            "crc_mode": "ccitt",
        }.items():
            self.declare_parameter(name, default)
        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.slave_id = int(self.get_parameter("slave_id").value)
        self.crc_mode = str(self.get_parameter("crc_mode").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.max_age = float(self.get_parameter("max_sample_age").value)
        self.max_skew = float(self.get_parameter("max_sample_skew").value)
        period = float(self.get_parameter("poll_period").value)
        if not self.port or self.baudrate <= 0 or not (math.isfinite(period) and period > 0):
            raise ValueError("port, baudrate and poll_period must be valid")
        self.parser = ImuParser(self.slave_id, self.crc_mode)
        self.assembler = SampleAssembler(self.max_age, self.max_skew)
        self.serial_handle = None
        self._retry_at = 0.0
        self._last_sample_at = None
        self._published = 0
        self._last_status_count = 0
        self._last_status_at = time.monotonic()
        self._last_error = "waiting for USB connection"
        self.pub = self.create_publisher(
            Imu, str(self.get_parameter("imu_topic").value), qos_profile_sensor_data)
        self.status_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter("status_topic").value), 10)
        self.timer = self.create_timer(period, self.poll)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info(
            f"DM-IMU passive reader: {self.port} @ {self.baudrate}, frame={self.frame_id}")

    def poll(self):
        now = time.monotonic()
        if self.serial_handle is None:
            if now < self._retry_at:
                return
            try:
                self.serial_handle = serial.Serial(self.port, self.baudrate, timeout=0.0, exclusive=True)
                self.serial_handle.reset_input_buffer()
                self.parser = ImuParser(self.slave_id, self.crc_mode)
                self.assembler = SampleAssembler(self.max_age, self.max_skew)
                self._last_sample_at = None
                self._last_error = "waiting for complete accel/gyro/orientation frames"
            except (serial.SerialException, OSError) as exc:
                self._last_error = str(exc)
                self._retry_at = now + 1.0
                if self.serial_handle is not None:
                    self.serial_handle.close()
                    self.serial_handle = None
                return
        try:
            waiting = self.serial_handle.in_waiting
            if not waiting:
                return
            data = self.serial_handle.read(min(waiting, 65536))
            now = time.monotonic()
            stamp = self.get_clock().now().to_msg()
            for frame in self.parser.append(data):
                sample = self.assembler.add(frame, now)
                if sample is None:
                    continue
                msg = Imu()
                msg.header.stamp = stamp
                msg.header.frame_id = self.frame_id
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = sample.acceleration
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = sample.angular_velocity
                msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = sample.orientation_xyzw
                # Unknown covariance is zero, not an invented noise estimate.
                self.pub.publish(msg)
                self._last_sample_at = now
                self._published += 1
                self._last_error = ""
        except (serial.SerialException, OSError) as exc:
            self._last_error = str(exc)
            self.serial_handle.close()
            self.serial_handle = None
            self._last_sample_at = None
            self._retry_at = now + 1.0

    def publish_status(self):
        now = time.monotonic()
        status = DiagnosticStatus()
        status.name = "dm_imu/usb"
        status.hardware_id = self.port
        if self.serial_handle is None:
            status.level = DiagnosticStatus.ERROR
            status.message = self._last_error
        elif self._last_sample_at is None or now - self._last_sample_at > self.max_age:
            status.level = DiagnosticStatus.WARN
            status.message = "No fresh complete IMU sample; check enabled outputs, slave ID and CRC"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Receiving complete IMU samples (host receipt timestamps)"
        rate = (self._published - self._last_status_count) / max(now - self._last_status_at, 1e-9)
        values = {
            "crc_mode": self.crc_mode,
            "published_samples": self._published, "sample_hz": round(rate, 2),
            "crc_valid_measurement_frames": self.parser.frames_ok,
            "crc_errors": self.parser.crc_errors, "format_errors": self.parser.format_errors,
            "other_devices": self.parser.other_devices,
            "received_types": ",".join(f"0x{kind:02x}" for kind in sorted(self.assembler.latest)),
            "sample_age_s": "none" if self._last_sample_at is None else round(now - self._last_sample_at, 4),
        }
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.status_pub.publish(msg)
        self._last_status_count = self._published
        self._last_status_at = now

    def destroy_node(self):
        if self.serial_handle is not None:
            self.serial_handle.close()
            self.serial_handle = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DmImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
