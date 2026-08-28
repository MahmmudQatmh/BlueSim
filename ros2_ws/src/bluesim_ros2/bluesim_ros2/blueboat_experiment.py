#!/usr/bin/env python3

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class BlueBoatExperiment(Node):

    def __init__(self):
        super().__init__('blueboat_experiment')

        # ============================================================
        # ROS 2 interfaces
        # ============================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_BlueBoat_vel',
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            'camera_BlueBoat',
            self.camera_callback,
            10
        )

        # ============================================================
        # Output video
        # ============================================================

        self.video_path = Path(
            '/media/mahmmudqatmh/Mahmmud_Qatmh/'
            'BlueSim/ros2_ws/BlueSim_BlueBoat_Record.mp4'
        )

        self.video_writer = None
        self.video_frames = 0

        # Must correspond to the Unreal camera publication rate.
        self.video_fps = 5.0

        # ============================================================
        # Experiment state
        # ============================================================

        self.recording_started = False
        self.mission_started = False
        self.mission_finished = False
        self.shutdown_requested = False

        # ============================================================
        # Commands
        # ============================================================

        self.forward_command = 1.0
        self.turn_command = 1.0

        # Timing-based experiment.
        #
        # These are NOT physical metres/degrees.
        # They are durations for the current force/torque model.

        self.turn_left_duration = 5.0
        self.forward_first_duration = 20.0

        self.turn_right_duration = 5.0
        self.forward_second_duration = 20.0

        # ============================================================
        # Mission state
        # ============================================================

        self.state = 'WAIT_FOR_CAMERA'
        self.state_start_time = time.monotonic()

        # ============================================================
        # Control timer
        # ============================================================

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
            'BlueSim BlueBoat Experiment'
        )
        self.get_logger().info(
            '=================================================='
        )
        self.get_logger().info(
            'Waiting for first valid camera frame...'
        )
        self.get_logger().info(
            'Camera topic : camera_BlueBoat'
        )
        self.get_logger().info(
            'Command topic: /cmd_BlueBoat_vel'
        )
        self.get_logger().info(
            f'Video output : {self.video_path}'
        )

    # ================================================================
    # Publish command
    # ================================================================

    def publish_command(
        self,
        linear_x: float,
        angular_z: float
    ):

        if self.shutdown_requested:
            return

        msg = Twist()

        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

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
        self.state = 'TURN_LEFT'
        self.state_start_time = time.monotonic()

        self.get_logger().info(
            'Camera recording is ready.'
        )

        self.get_logger().info(
            'Starting BlueBoat movement.'
        )

        self.get_logger().info(
            '>>> TURN LEFT'
        )

    # ================================================================
    # State transition
    # ================================================================

    def change_state(self, new_state):

        if self.mission_finished:
            return

        self.state = new_state
        self.state_start_time = time.monotonic()

        self.get_logger().info(
            f'>>> {new_state}'
        )

    # ================================================================
    # Mission control
    # ================================================================

    def control_loop(self):

        if self.mission_finished:
            return

        # ------------------------------------------------------------
        # Wait for camera
        # ------------------------------------------------------------

        if not self.mission_started:

            self.publish_command(
                linear_x=0.0,
                angular_z=0.0
            )

            return

        elapsed = time.monotonic() - self.state_start_time

        # ------------------------------------------------------------
        # TURN LEFT
        #
        # Unreal's current convention:
        # negative angular.z = left
        # ------------------------------------------------------------

        if self.state == 'TURN_LEFT':

            self.publish_command(
                linear_x=0.0,
                angular_z=-self.turn_command
            )

            if elapsed >= self.turn_left_duration:

                self.change_state('FORWARD_1')

        # ------------------------------------------------------------
        # FORWARD 1
        # ------------------------------------------------------------

        elif self.state == 'FORWARD_1':

            self.publish_command(
                linear_x=self.forward_command,
                angular_z=0.0
            )

            if elapsed >= self.forward_first_duration:

                self.change_state('TURN_RIGHT')

        # ------------------------------------------------------------
        # TURN RIGHT
        # ------------------------------------------------------------

        elif self.state == 'TURN_RIGHT':

            self.publish_command(
                linear_x=0.0,
                angular_z=self.turn_command
            )

            if elapsed >= self.turn_right_duration:

                self.change_state('FORWARD_2')

        # ------------------------------------------------------------
        # FORWARD 2
        # ------------------------------------------------------------

        elif self.state == 'FORWARD_2':

            self.publish_command(
                linear_x=self.forward_command,
                angular_z=0.0
            )

            if elapsed >= self.forward_second_duration:

                self.change_state('STOP')

        # ------------------------------------------------------------
        # STOP
        # ------------------------------------------------------------

        elif self.state == 'STOP':

            self.finish_experiment()

    # ================================================================
    # Camera callback
    # ================================================================

    def camera_callback(self, msg: Image):

        # ============================================================
        # CRITICAL:
        # Never process camera frames after experiment completion.
        # ============================================================

        if self.mission_finished:
            return

        # ------------------------------------------------------------
        # Ignore empty images
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
                        'Camera data is smaller than expected.'
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
                        'Camera data is smaller than expected.'
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
                        'Camera data is smaller than expected.'
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
                    f'Unsupported image encoding: '
                    f'{msg.encoding}'
                )

                return

            # ========================================================
            # Create video writer once
            # ========================================================

            if self.video_writer is None:

                # Never recreate writer after mission completion.
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
                # Movement begins only AFTER the video writer exists.
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
                f'Camera processing failed: {exc}'
            )

    # ================================================================
    # Finish experiment
    # ================================================================

    def finish_experiment(self):

        if self.mission_finished:
            return

        # ============================================================
        # IMPORTANT:
        # Set this FIRST.
        #
        # Any later camera callback immediately returns.
        # ============================================================

        self.mission_finished = True

        # ------------------------------------------------------------
        # Stop command
        # ------------------------------------------------------------

        self.publish_command(
            linear_x=0.0,
            angular_z=0.0
        )

        self.get_logger().info(
            'BlueBoat stopped.'
        )

        # ------------------------------------------------------------
        # Stop control timer
        # ------------------------------------------------------------

        if self.command_timer is not None:
            self.command_timer.cancel()

        # ------------------------------------------------------------
        # Stop receiving camera data
        #
        # This prevents another callback from attempting to create
        # another VideoWriter after the current writer is released.
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
                'No valid camera frames were recorded.'
            )

        self.get_logger().info(
            'BlueBoat experiment completed.'
        )

        # ------------------------------------------------------------
        # Schedule clean ROS shutdown.
        # ------------------------------------------------------------

        self.shutdown_timer = self.create_timer(
            0.1,
            self.shutdown_callback
        )

    # ================================================================
    # ROS shutdown
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
    # Manual interruption / cleanup
    # ================================================================

    def destroy_node(self):

        # ------------------------------------------------------------
        # Safety stop
        # ------------------------------------------------------------

        if not self.mission_finished:

            try:

                self.publish_command(
                    linear_x=0.0,
                    angular_z=0.0
                )

            except Exception:
                pass

        # ------------------------------------------------------------
        # Finalize video if Ctrl+C happens during the experiment.
        # ------------------------------------------------------------

        if self.video_writer is not None:

            self.video_writer.release()
            self.video_writer = None

            print(
                f'Video saved to: {self.video_path}'
            )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = BlueBoatExperiment()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'Experiment interrupted by user.'
        )

        if not node.mission_finished:
            node.mission_finished = True

    finally:

        node.destroy_node()


if __name__ == '__main__':
    main()