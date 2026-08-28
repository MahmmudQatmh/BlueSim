#!/usr/bin/env python3

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class BlueSimDroneExperiment(Node):

    def __init__(self):
        super().__init__('bluesim_drone_experiment')

        # ============================================================
        # ROS 2 interfaces
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_drone_vel',
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/camera_Drone',
            self.camera_callback,
            10
        )

        # ============================================================
        # Video configuration
        # ============================================================

        self.video_path = Path(
            '/media/mahmmudqatmh/Mahmmud_Qatmh/'
            'BlueSim/ros2_ws/BlueSim_Drone_Record.mp4'
        )

        self.video_writer = None
        self.video_frames = 0

        # Drone camera publisher is configured for 5 Hz.
        self.video_fps = 5.0

        # ============================================================
        # Experiment flags
        # ============================================================

        self.recording_started = False
        self.mission_started = False
        self.mission_finished = False
        self.shutdown_requested = False

        # ============================================================
        # Command magnitudes
        # ============================================================

        # Current Unreal interface treats these as control values,
        # not guaranteed physical m/s or rad/s.

        self.altitude_command = 1.0
        self.forward_command = 1.0
        self.yaw_command = 1.0

        # ============================================================
        # Experiment durations
        # ============================================================

        # Tune these experimentally.

        self.altitude_duration = 5.0
        self.yaw_right_duration = 2.0
        self.forward_first_duration = 10.0
        self.yaw_left_duration = 2.0
        self.forward_second_duration = 10.0

        # ============================================================
        # State machine
        # ============================================================

        self.state = 'WAIT_FOR_CAMERA'
        self.state_start_time = time.monotonic()

        # ============================================================
        # Control timer
        # ============================================================

        # Publish commands continuously at 20 Hz.
        #
        # This is independent of the camera rate.
        #
        # Camera = 5 Hz
        # Commands = 20 Hz

        self.command_timer = self.create_timer(
            0.05,
            self.control_loop
        )

        # ============================================================
        # Logging
        # ============================================================

        self.get_logger().info(
            '=================================================='
        )

        self.get_logger().info(
            'BlueSim Drone Experiment'
        )

        self.get_logger().info(
            '=================================================='
        )

        self.get_logger().info(
            'Waiting for first valid camera frame...'
        )

        self.get_logger().info(
            'Camera topic : /camera_Drone'
        )

        self.get_logger().info(
            'Command topic: /cmd_drone_vel'
        )

        self.get_logger().info(
            f'Video output : {self.video_path}'
        )

        self.get_logger().info(
            'Camera rate  : 5 Hz'
        )

        self.get_logger().info(
            'Command rate : 20 Hz'
        )

    # ================================================================
    # Publish Drone command
    # ================================================================

    def publish_command(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        linear_z: float = 0.0,
        angular_z: float = 0.0
    ):

        if self.shutdown_requested:
            return

        msg = Twist()

        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.linear.z = float(linear_z)

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        self.cmd_pub.publish(msg)

    # ================================================================
    # Start mission
    # ================================================================

    def start_mission(self):

        if self.mission_started or self.mission_finished:
            return

        self.mission_started = True

        self.state = 'ALTITUDE'
        self.state_start_time = time.monotonic()

        self.get_logger().info(
            'Camera recording is ready.'
        )

        self.get_logger().info(
            'Starting Drone movement.'
        )

        self.get_logger().info(
            '>>> INCREASE ALTITUDE'
        )

    # ================================================================
    # Change experiment state
    # ================================================================

    def change_state(self, new_state: str):

        if self.mission_finished:
            return

        self.state = new_state
        self.state_start_time = time.monotonic()

        self.get_logger().info(
            f'>>> {new_state}'
        )

    # ================================================================
    # Main control loop
    # ================================================================

    def control_loop(self):

        if self.mission_finished:
            return

        # ------------------------------------------------------------
        # Wait for camera / video
        # ------------------------------------------------------------

        if not self.mission_started:

            self.publish_command()

            return

        elapsed = time.monotonic() - self.state_start_time

        # ------------------------------------------------------------
        # 1. INCREASE ALTITUDE
        #
        # linear.z > 0 corresponds to positive DroneAltitude.
        # ------------------------------------------------------------

        if self.state == 'ALTITUDE':

            self.publish_command(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=self.altitude_command,
                angular_z=0.0
            )

            if elapsed >= self.altitude_duration:

                self.change_state('YAW_RIGHT')

        # ------------------------------------------------------------
        # 2. YAW RIGHT
        #
        # With the Unreal Z-axis rotation convention used by your
        # DroneYaw implementation:
        #
        # positive angular.z -> positive yaw.
        # ------------------------------------------------------------

        elif self.state == 'YAW_RIGHT':

            self.publish_command(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=0.0,
                angular_z=self.yaw_command
            )

            if elapsed >= self.yaw_right_duration:

                self.change_state('FORWARD_1')

        # ------------------------------------------------------------
        # 3. MOVE FORWARD
        # ------------------------------------------------------------

        elif self.state == 'FORWARD_1':

            self.publish_command(
                linear_x=0.0,
                linear_y=-self.forward_command,
                linear_z=0.0,
                angular_z=0.0
            )

            if elapsed >= self.forward_first_duration:

                self.change_state('YAW_LEFT')

        # ------------------------------------------------------------
        # 4. YAW LEFT
        # ------------------------------------------------------------

        elif self.state == 'YAW_LEFT':

            self.publish_command(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=0.0,
                angular_z=-self.yaw_command
            )

            if elapsed >= self.yaw_left_duration:

                self.change_state('FORWARD_2')

        # ------------------------------------------------------------
        # 5. MOVE FORWARD AGAIN
        # ------------------------------------------------------------

        elif self.state == 'FORWARD_2':

            self.publish_command(
                linear_x=0.0,
                linear_y=-self.forward_command,
                linear_z=0.0,
                angular_z=0.0
            )

            if elapsed >= self.forward_second_duration:

                self.change_state('STOP')

        # ------------------------------------------------------------
        # 6. STOP
        # ------------------------------------------------------------

        elif self.state == 'STOP':

            self.finish_experiment()

    # ================================================================
    # Camera callback
    # ================================================================

    def camera_callback(self, msg: Image):

        # Never process frames after experiment completion.
        if self.mission_finished:
            return

        # ------------------------------------------------------------
        # Validate image
        # ------------------------------------------------------------

        if (
            msg.width <= 0
            or msg.height <= 0
            or len(msg.data) == 0
        ):
            return

        try:

            # ========================================================
            # BGR8
            # ========================================================

            if msg.encoding == 'bgr8':

                expected_size = (
                    msg.height *
                    msg.width *
                    3
                )

                raw = np.frombuffer(
                    msg.data,
                    dtype=np.uint8
                )

                if raw.size < expected_size:
                    self.get_logger().warning(
                        'Drone camera data is smaller than expected.'
                    )
                    return

                frame = raw[
                    :expected_size
                ].reshape(
                    msg.height,
                    msg.width,
                    3
                )

            # ========================================================
            # RGB8
            # ========================================================

            elif msg.encoding == 'rgb8':

                expected_size = (
                    msg.height *
                    msg.width *
                    3
                )

                raw = np.frombuffer(
                    msg.data,
                    dtype=np.uint8
                )

                if raw.size < expected_size:
                    self.get_logger().warning(
                        'Drone camera data is smaller than expected.'
                    )
                    return

                frame = raw[
                    :expected_size
                ].reshape(
                    msg.height,
                    msg.width,
                    3
                )

                frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_RGB2BGR
                )

            # ========================================================
            # MONO8
            # ========================================================

            elif msg.encoding == 'mono8':

                expected_size = (
                    msg.height *
                    msg.width
                )

                raw = np.frombuffer(
                    msg.data,
                    dtype=np.uint8
                )

                if raw.size < expected_size:
                    self.get_logger().warning(
                        'Drone camera data is smaller than expected.'
                    )
                    return

                gray = raw[
                    :expected_size
                ].reshape(
                    msg.height,
                    msg.width
                )

                frame = cv2.cvtColor(
                    gray,
                    cv2.COLOR_GRAY2BGR
                )

            else:

                self.get_logger().warning(
                    f'Unsupported Drone image encoding: '
                    f'{msg.encoding}'
                )

                return

            # ========================================================
            # Create video writer ONCE
            # ========================================================

            if self.video_writer is None:

                # Prevent creation after experiment completion.
                if self.mission_finished:
                    return

                height, width = frame.shape[:2]

                self.video_writer = cv2.VideoWriter(
                    str(self.video_path),
                    cv2.VideoWriter_fourcc(*'mp4v'),
                    self.video_fps,
                    (width, height)
                )

                if not self.video_writer.isOpened():

                    self.get_logger().error(
                        f'Could not open video output: '
                        f'{self.video_path}'
                    )

                    self.video_writer = None
                    return

                self.recording_started = True

                self.get_logger().info(
                    f'Video recording started: '
                    f'{width}x{height} @ '
                    f'{self.video_fps:.1f} FPS'
                )

                # ----------------------------------------------------
                # Only now start the Drone.
                # ----------------------------------------------------

                self.start_mission()

            # ========================================================
            # Write frame
            # ========================================================

            if (
                self.video_writer is not None
                and not self.mission_finished
            ):

                self.video_writer.write(frame)
                self.video_frames += 1

        except Exception as exc:

            self.get_logger().error(
                f'Drone camera processing failed: {exc}'
            )

    # ================================================================
    # Finish experiment
    # ================================================================

    def finish_experiment(self):

        if self.mission_finished:
            return

        # Set this FIRST so camera callbacks cannot recreate the video.
        self.mission_finished = True

        # ------------------------------------------------------------
        # Stop Drone
        # ------------------------------------------------------------

        self.publish_command()

        self.get_logger().info(
            'Drone stopped.'
        )

        # ------------------------------------------------------------
        # Stop command timer
        # ------------------------------------------------------------

        if self.command_timer is not None:
            self.command_timer.cancel()

        # ------------------------------------------------------------
        # Stop receiving camera frames
        # ------------------------------------------------------------

        if self.camera_sub is not None:

            self.destroy_subscription(
                self.camera_sub
            )

            self.camera_sub = None

        # ------------------------------------------------------------
        # Finalize video
        # ------------------------------------------------------------

        if self.video_writer is not None:

            self.video_writer.release()
            self.video_writer = None

            self.get_logger().info(
                f'Video saved to: '
                f'{self.video_path}'
            )

            self.get_logger().info(
                f'Frames recorded: '
                f'{self.video_frames}'
            )

        else:

            self.get_logger().warning(
                'Experiment finished but no camera frames '
                'were recorded.'
            )

        self.get_logger().info(
            'Drone experiment completed.'
        )

        # ------------------------------------------------------------
        # Clean ROS shutdown.
        # ------------------------------------------------------------

        self.shutdown_timer = self.create_timer(
            0.1,
            self.shutdown_callback
        )

    # ================================================================
    # Shutdown
    # ================================================================

    def shutdown_callback(self):

        if self.shutdown_requested:
            return

        self.shutdown_requested = True

        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()

        if rclpy.ok():
            rclpy.shutdown()

    # ================================================================
    # Manual Ctrl+C cleanup
    # ================================================================

    def destroy_node(self):

        # Safety stop if interrupted during flight.

        if not self.mission_finished:

            try:
                self.publish_command()
            except Exception:
                pass

        # Finalize video if necessary.

        if self.video_writer is not None:

            self.video_writer.release()
            self.video_writer = None

            print(
                f'Video saved to: {self.video_path}'
            )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = BlueSimDroneExperiment()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Drone experiment interrupted by user.'
        )

    finally:

        node.destroy_node()


if __name__ == '__main__':
    main()