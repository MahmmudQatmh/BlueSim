#!/usr/bin/env python3

"""
===============================================================================
BlueSim - Rover ROS 2 Command and Camera Recording Example
===============================================================================

PURPOSE
-------
This script is an example/template for controlling the simulated Rover through
ROS 2 while simultaneously recording the Rover onboard camera.

The important Rover movement definitions are documented at the beginning of
this file so that a new user can understand how to operate the simulated Rover
without first reading the Unreal Engine Blueprint implementation.

A new user should normally only need to modify:

    MISSION = [
        ("forward", 10.0),
        ("turn_right", 5.0),
        ("forward", 10.0),
        ("stop", 1.0),
    ]

The rest of this file implements:

    - ROS 2 publisher
    - ROS 2 camera subscriber
    - Camera image conversion
    - Video recording
    - Experiment execution
    - Automatic shutdown

===============================================================================
1. ROVER ROS 2 INTERFACE
===============================================================================

COMMAND TOPIC
-------------
    /cmd_rover_vel

MESSAGE TYPE
------------
    geometry_msgs/msg/Twist


CAMERA TOPIC
------------
    /camera_Rover

MESSAGE TYPE
------------
    sensor_msgs/msg/Image


===============================================================================
2. ROVER MOVEMENT CONVENTION
===============================================================================

IMPORTANT
---------
The current Unreal Rover implementation does NOT use the standard ROS
linear.x / angular.z interpretation directly.

The current BlueSim Rover Blueprint uses the following mapping:


-----------------------------------------------------------------------------
FORWARD / BACKWARD
-----------------------------------------------------------------------------

Current Unreal Rover convention:

    ROS linear.x = -1.0
         Move FORWARD

    ROS linear.x = +1.0
         Move BACKWARD

    ROS linear.x = 0.0
         No forward/backward command


Why?

The current Unreal Rover movement system uses the local Y axis:

    Local Y = -1
         Forward

    Local Y = +1
         Backward

Therefore this ROS interface currently uses:

    linear.x = -1
         Forward

    linear.x = +1
         Backward


-----------------------------------------------------------------------------
LEFT / RIGHT STEERING
-----------------------------------------------------------------------------

Current Unreal Rover steering convention:

    ROS angular.z = -1.0
         Turn RIGHT

    ROS angular.z = +1.0
         Turn LEFT

    ROS angular.z = 0.0
         No steering command


IMPORTANT
---------
The sign convention is specific to the current BlueSim Rover implementation.

Do not assume that every ROS robot will use the same sign convention.


===============================================================================
3. AVAILABLE HIGH-LEVEL MOVEMENT COMMANDS
===============================================================================

The experiment section uses readable high-level commands.

Available commands:

    "forward"
        Move forward.

    "backward"
        Move backward.

    "turn_right"
        Turn right.

    "turn_left"
        Turn left.

    "stop"
        Stop the commanded movement.


COMMAND FORMAT
--------------

Each command uses:

    ("command_name", duration_in_seconds)


Examples:

    ("forward", 10.0)
         Move forward for 10 seconds.

    ("backward", 5.0)
         Move backward for 5 seconds.

    ("turn_right", 5.0)
         Turn right for 5 seconds.

    ("turn_left", 5.0)
         Turn left for 5 seconds.


===============================================================================
4. ROVER MOVEMENT MODEL
===============================================================================

The current Rover uses Unreal Engine physics.

The ROS commands are therefore control values rather than guaranteed physical
velocities.

For example:

    linear.x = -1.0

means:

    "Apply the configured forward Rover command"

It does NOT necessarily mean:

    "Move at exactly 1 metre/second."


Likewise:

    angular.z = -1.0

means:

    "Apply the configured right-turn Rover command"

It does NOT necessarily mean:

    "Rotate at exactly 1 radian/second."


The actual distance travelled and angle turned depend on:

    - Unreal Engine physics
    - Rover mass
    - Rover Drive Force
    - Rover steering torque
    - Friction
    - Damping
    - Ground interaction
    - Command duration


===============================================================================
5. CAMERA INTERFACE
===============================================================================

The Rover camera is available through:

    /camera_Rover

ROS 2 message type:

    sensor_msgs/msg/Image


CURRENT CAMERA CONFIGURATION
----------------------------

    Width       : 640
    Height      : 360
    Encoding    : bgr8
    Step        : 1920


CAMERA PIPELINE
---------------

    Unreal Rover Camera
            ↓
    SceneCaptureComponent2D
            ↓
    Render Target
            ↓
    rclUE Camera Publisher
            ↓
    /camera_Rover
            ↓
    sensor_msgs/msg/Image
            ↓
    Python / rclpy
            ↓
    NumPy
            ↓
    OpenCV
            ↓
    MP4 Video


===============================================================================
6. VIDEO RECORDING
===============================================================================

The script waits for the first valid Rover camera frame before starting the
movement experiment.

This guarantees that camera recording is ready before the Rover starts moving.

The sequence is:

    Start script
         ↓
    Wait for valid camera frame
         ↓
    Create VideoWriter
         ↓
    Start Rover movement
         ↓
    Execute MISSION
         ↓
    Stop Rover
         ↓
    Finalize video
         ↓
    Save video
         ↓
    Terminate


VIDEO DIRECTORY
---------------

    BlueSim/ros2_ws/Recorded Videos/


VIDEO FILE
----------

    BlueSim_Rover_Record.mp4


===============================================================================
7. COMMAND PUBLICATION RATE
===============================================================================

COMMAND RATE
------------

    20 Hz

A command is therefore published approximately every:

    0.05 seconds


VIDEO FPS
---------

    5 FPS

This matches the current Rover camera recording configuration.

The actual camera publication rate can be checked with:

    ros2 topic hz /camera_Rover


===============================================================================
8. EXPERIMENT DEFINITION
===============================================================================

The mission is defined using the MISSION list below.

Current example:

    MISSION = [
        ("forward", 10.0),
        ("turn_right", 5.0),
        ("forward", 10.0),
        ("stop", 1.0),
    ]


This means:

    1. Move forward for 10 seconds.
    2. Turn right for 5 seconds.
    3. Move forward for 10 seconds.
    4. Stop.


To create another experiment, simply modify MISSION.

For example:

    MISSION = [
        ("forward", 20.0),
        ("turn_left", 4.0),
        ("forward", 15.0),
        ("backward", 5.0),
        ("stop", 1.0),
    ]


No ROS 2 publisher implementation needs to be changed.


===============================================================================
9. SAFETY / STOP BEHAVIOR
===============================================================================

When the experiment finishes, the script sends:

    linear.x  = 0.0
    angular.z = 0.0

This requests the Rover to stop applying commanded movement.

If Ctrl+C is pressed during the experiment, the script also attempts to send
a zero command before shutting down.


===============================================================================
"""

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


# =============================================================================
# ROVER CONTROL DEFINITIONS
# =============================================================================

# -----------------------------------------------------------------------------
# ROS 2 topics
# -----------------------------------------------------------------------------

COMMAND_TOPIC = "/cmd_rover_vel"
CAMERA_TOPIC = "/camera_Rover"


# -----------------------------------------------------------------------------
# Rover movement command values
# -----------------------------------------------------------------------------
#
# IMPORTANT:
#
# These values follow the CURRENT BlueSim Unreal Rover implementation.
#
# Forward:
#     linear.x = -1.0
#
# Backward:
#     linear.x = +1.0
#
# Right:
#     angular.z = -1.0
#
# Left:
#     angular.z = +1.0
#
# These are control values, not guaranteed physical units.

FORWARD_COMMAND = -1.0
BACKWARD_COMMAND = 1.0

TURN_RIGHT_COMMAND = -1.0
TURN_LEFT_COMMAND = 1.0

ZERO_COMMAND = 0.0


# -----------------------------------------------------------------------------
# ROS 2 command publication rate
# -----------------------------------------------------------------------------

COMMAND_RATE_HZ = 20.0


# -----------------------------------------------------------------------------
# Camera / video configuration
# -----------------------------------------------------------------------------

VIDEO_FPS = 5.0

EXPECTED_CAMERA_WIDTH = 640
EXPECTED_CAMERA_HEIGHT = 360
EXPECTED_CAMERA_ENCODING = "bgr8"


# -----------------------------------------------------------------------------
# Video output
# -----------------------------------------------------------------------------
#
# The Python file is located at:
#
# BlueSim/
# └── ros2_ws/
#     └── src/
#         └── bluesim_ros2/
#             └── bluesim_ros2/
#                 └── rover_command.py
#
# parents[3] points to:
#
# BlueSim/ros2_ws/
#
# Therefore the video is automatically saved to:
#
# BlueSim/ros2_ws/Recorded Videos/
# -----------------------------------------------------------------------------

ROS2_WORKSPACE = Path(__file__).resolve().parents[3]

VIDEO_DIRECTORY = ROS2_WORKSPACE / "Recorded Videos"

VIDEO_FILENAME = "BlueSim_Rover_Record.mp4"

VIDEO_PATH = VIDEO_DIRECTORY / VIDEO_FILENAME


# =============================================================================
# WRITE YOUR EXPERIMENT HERE
# =============================================================================
#
# Available commands:
#
#     "forward"
#     "backward"
#     "turn_right"
#     "turn_left"
#     "stop"
#
# Command format:
#
#     ("command", duration_in_seconds)
#
# =============================================================================

MISSION = [

    # 1. Move forward.
    ("forward", 10.0),

    # 2. Turn right.
    ("turn_right", 5.0),

    # 3. Move forward again.
    ("forward", 10.0),

    # 4. Stop.
    ("stop", 1.0),
]


# =============================================================================
# ROVER ROS 2 NODE
# =============================================================================


class BlueSimRoverCommand(Node):

    def __init__(self):

        super().__init__("bluesim_rover_command")

        # =====================================================================
        # ROS 2 COMMAND PUBLISHER
        # =====================================================================

        self.command_publisher = self.create_publisher(
            Twist,
            COMMAND_TOPIC,
            10,
        )

        # =====================================================================
        # ROS 2 CAMERA SUBSCRIBER
        # =====================================================================

        self.camera_subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.camera_callback,
            10,
        )

        # =====================================================================
        # VIDEO
        # =====================================================================

        self.video_writer = None
        self.video_frame_count = 0

        self.recording_started = False

        # =====================================================================
        # EXPERIMENT STATE
        # =====================================================================

        self.mission_started = False
        self.mission_finished = False
        self.shutdown_requested = False

        self.current_mission_index = 0
        self.current_command = None
        self.current_command_duration = 0.0
        self.command_start_time = None

        # =====================================================================
        # CONTROL TIMER
        # =====================================================================

        self.control_timer = self.create_timer(
            1.0 / COMMAND_RATE_HZ,
            self.control_loop,
        )

        # =====================================================================
        # LOGGING
        # =====================================================================

        self.get_logger().info(
            "=================================================="
        )

        self.get_logger().info(
            "BlueSim Rover Command"
        )

        self.get_logger().info(
            "=================================================="
        )

        self.get_logger().info(
            f"Command topic : {COMMAND_TOPIC}"
        )

        self.get_logger().info(
            f"Camera topic  : {CAMERA_TOPIC}"
        )

        self.get_logger().info(
            f"Video output  : {VIDEO_PATH}"
        )

        self.get_logger().info(
            f"Command rate  : {COMMAND_RATE_HZ:.1f} Hz"
        )

        self.get_logger().info(
            f"Video FPS     : {VIDEO_FPS:.1f}"
        )

        self.get_logger().info(
            "Waiting for first valid camera frame..."
        )

    # =========================================================================
    # COMMAND PUBLISHER
    # =========================================================================

    def publish_command(
        self,
        linear_x: float = 0.0,
        angular_z: float = 0.0,
    ):

        if self.mission_finished:
            return

        msg = Twist()

        # ---------------------------------------------------------------------
        # Linear velocity command
        # ---------------------------------------------------------------------

        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # ---------------------------------------------------------------------
        # Angular velocity command
        # ---------------------------------------------------------------------

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        self.command_publisher.publish(msg)

    # =========================================================================
    # HIGH-LEVEL ROVER COMMAND DEFINITIONS
    # =========================================================================
    #
    # These functions translate the readable MISSION commands into ROS 2
    # Twist values according to the current BlueSim Rover convention.
    # =========================================================================

    def execute_command(self, command_name: str):

        if command_name == "forward":

            self.publish_command(
                linear_x=FORWARD_COMMAND,
                angular_z=ZERO_COMMAND,
            )

        elif command_name == "backward":

            self.publish_command(
                linear_x=BACKWARD_COMMAND,
                angular_z=ZERO_COMMAND,
            )

        elif command_name == "turn_right":

            self.publish_command(
                linear_x=ZERO_COMMAND,
                angular_z=TURN_RIGHT_COMMAND,
            )

        elif command_name == "turn_left":

            self.publish_command(
                linear_x=ZERO_COMMAND,
                angular_z=TURN_LEFT_COMMAND,
            )

        elif command_name == "stop":

            self.publish_command(
                linear_x=ZERO_COMMAND,
                angular_z=ZERO_COMMAND,
            )

        else:

            self.get_logger().error(
                f"Unknown Rover command: '{command_name}'"
            )

            self.finish_experiment()

    # =========================================================================
    # START MISSION
    # =========================================================================

    def start_mission(self):

        if self.mission_started:
            return

        if self.mission_finished:
            return

        if not self.validate_mission():

            self.get_logger().error(
                "Mission definition is invalid."
            )

            self.finish_experiment()

            return

        self.mission_started = True

        self.current_mission_index = 0

        self.current_command = None

        self.start_next_command()

    # =========================================================================
    # VALIDATE MISSION
    # =========================================================================

    def validate_mission(self):

        valid_commands = {
            "forward",
            "backward",
            "turn_right",
            "turn_left",
            "stop",
        }

        for item in MISSION:

            if not isinstance(item, tuple):

                self.get_logger().error(
                    "Each MISSION entry must be a tuple:"
                    " ('command', duration)"
                )

                return False

            if len(item) != 2:

                self.get_logger().error(
                    "Each MISSION entry must contain exactly "
                    "two values: command and duration."
                )

                return False

            command_name, duration = item

            if command_name not in valid_commands:

                self.get_logger().error(
                    f"Unknown MISSION command: {command_name}"
                )

                return False

            if duration <= 0.0:

                self.get_logger().error(
                    f"Invalid duration for '{command_name}': {duration}"
                )

                return False

        return True

    # =========================================================================
    # START NEXT COMMAND
    # =========================================================================

    def start_next_command(self):

        # ---------------------------------------------------------------------
        # Mission finished.
        # ---------------------------------------------------------------------

        if self.current_mission_index >= len(MISSION):

            self.finish_experiment()

            return

        # ---------------------------------------------------------------------
        # Get next command.
        # ---------------------------------------------------------------------

        command_name, duration = MISSION[
            self.current_mission_index
        ]

        self.current_command = command_name
        self.current_command_duration = float(duration)

        self.command_start_time = time.monotonic()

        self.get_logger().info(
            f">>> {command_name.upper()} "
            f"for {duration:.1f} seconds"
        )

        self.execute_command(command_name)

    # =========================================================================
    # CONTROL LOOP
    # =========================================================================

    def control_loop(self):

        # ---------------------------------------------------------------------
        # Mission has not started yet.
        # Camera recording must be ready first.
        # ---------------------------------------------------------------------

        if not self.mission_started:

            self.publish_command(
                linear_x=ZERO_COMMAND,
                angular_z=ZERO_COMMAND,
            )

            return

        # ---------------------------------------------------------------------
        # Mission finished.
        # ---------------------------------------------------------------------

        if self.mission_finished:
            return

        # ---------------------------------------------------------------------
        # Safety check.
        # ---------------------------------------------------------------------

        if self.command_start_time is None:
            return

        # ---------------------------------------------------------------------
        # Calculate elapsed command time.
        # ---------------------------------------------------------------------

        elapsed = (
            time.monotonic()
            - self.command_start_time
        )

        # ---------------------------------------------------------------------
        # Current command duration completed.
        # ---------------------------------------------------------------------

        if elapsed >= self.current_command_duration:

            self.current_mission_index += 1

            self.start_next_command()

            return

        # ---------------------------------------------------------------------
        # Re-publish current command continuously.
        # ---------------------------------------------------------------------

        if self.current_command is not None:

            self.execute_command(
                self.current_command
            )

    # =========================================================================
    # CAMERA CALLBACK
    # =========================================================================

    def camera_callback(self, msg: Image):

        # Never process frames after experiment completion.
        if self.mission_finished:
            return

        # =====================================================================
        # Ignore invalid/empty images
        # =====================================================================

        if (
            msg.width <= 0
            or msg.height <= 0
            or len(msg.data) == 0
        ):

            return

        try:

            # =================================================================
            # BGR8
            # =================================================================

            if msg.encoding == "bgr8":

                expected_size = (
                    msg.width
                    * msg.height
                    * 3
                )

                raw = np.frombuffer(
                    msg.data,
                    dtype=np.uint8,
                )

                if raw.size < expected_size:

                    self.get_logger().warning(
                        "Rover camera data is smaller than expected."
                    )

                    return

                frame = raw[
                    :expected_size
                ].reshape(
                    msg.height,
                    msg.width,
                    3,
                )

            # =================================================================
            # RGB8
            # =================================================================

            elif msg.encoding == "rgb8":

                expected_size = (
                    msg.width
                    * msg.height
                    * 3
                )

                raw = np.frombuffer(
                    msg.data,
                    dtype=np.uint8,
                )

                if raw.size < expected_size:

                    self.get_logger().warning(
                        "Rover camera data is smaller than expected."
                    )

                    return

                frame = raw[
                    :expected_size
                ].reshape(
                    msg.height,
                    msg.width,
                    3,
                )

                frame = cv2.cvtColor(
                    frame,
                    cv2.COLOR_RGB2BGR,
                )

            # =================================================================
            # MONO8
            # =================================================================

            elif msg.encoding == "mono8":

                expected_size = (
                    msg.width
                    * msg.height
                )

                raw = np.frombuffer(
                    msg.data,
                    dtype=np.uint8,
                )

                if raw.size < expected_size:

                    self.get_logger().warning(
                        "Rover camera data is smaller than expected."
                    )

                    return

                gray = raw[
                    :expected_size
                ].reshape(
                    msg.height,
                    msg.width,
                )

                frame = cv2.cvtColor(
                    gray,
                    cv2.COLOR_GRAY2BGR,
                )

            # =================================================================
            # Unsupported encoding
            # =================================================================

            else:

                self.get_logger().warning(
                    f"Unsupported Rover camera encoding: "
                    f"{msg.encoding}"
                )

                return

            # =================================================================
            # Create VideoWriter ONCE
            # =================================================================

            if self.video_writer is None:

                # Do not create a writer after completion.
                if self.mission_finished:
                    return

                VIDEO_DIRECTORY.mkdir(
                    parents=True,
                    exist_ok=True,
                )

                height, width = frame.shape[:2]

                self.video_writer = cv2.VideoWriter(
                    str(VIDEO_PATH),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    VIDEO_FPS,
                    (width, height),
                )

                if not self.video_writer.isOpened():

                    self.get_logger().error(
                        f"Could not open video file: "
                        f"{VIDEO_PATH}"
                    )

                    self.video_writer = None

                    return

                self.recording_started = True

                self.get_logger().info(
                    f"Video recording started: "
                    f"{width}x{height} @ "
                    f"{VIDEO_FPS:.1f} FPS"
                )

                # Start Rover movement only after recording is ready.
                self.start_mission()

            # =================================================================
            # Write current frame
            # =================================================================

            if (
                self.video_writer is not None
                and not self.mission_finished
            ):

                self.video_writer.write(frame)

                self.video_frame_count += 1

        except Exception as exc:

            self.get_logger().error(
                f"Rover camera processing failed: {exc}"
            )

    # =========================================================================
    # FINISH EXPERIMENT
    # =========================================================================

    def finish_experiment(self):

        if self.mission_finished:
            return

        # ---------------------------------------------------------------------
        # Mark finished FIRST.
        #
        # This prevents new camera frames from recreating the video writer.
        # ---------------------------------------------------------------------

        self.mission_finished = True

        # ---------------------------------------------------------------------
        # Stop Rover.
        # ---------------------------------------------------------------------

        try:

            msg = Twist()

            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

            self.command_publisher.publish(msg)

        except Exception:
            pass

        self.get_logger().info(
            "Rover stopped."
        )

        # ---------------------------------------------------------------------
        # Stop camera subscription.
        # ---------------------------------------------------------------------

        if self.camera_subscription is not None:

            self.destroy_subscription(
                self.camera_subscription
            )

            self.camera_subscription = None

        # ---------------------------------------------------------------------
        # Stop command timer.
        # ---------------------------------------------------------------------

        if self.control_timer is not None:

            self.control_timer.cancel()

        # ---------------------------------------------------------------------
        # Finalize video.
        # ---------------------------------------------------------------------

        if self.video_writer is not None:

            self.video_writer.release()

            self.video_writer = None

            self.get_logger().info(
                f"Video saved to: "
                f"{VIDEO_PATH}"
            )

            self.get_logger().info(
                f"Frames recorded: "
                f"{self.video_frame_count}"
            )

        else:

            self.get_logger().warning(
                "No valid Rover camera frames were recorded."
            )

        self.get_logger().info(
            "Rover experiment completed."
        )

        # ---------------------------------------------------------------------
        # Clean ROS shutdown.
        # ---------------------------------------------------------------------

        self.shutdown_timer = self.create_timer(
            0.1,
            self.shutdown_callback,
        )

    # =========================================================================
    # ROS SHUTDOWN
    # =========================================================================

    def shutdown_callback(self):

        if self.shutdown_requested:
            return

        self.shutdown_requested = True

        if self.shutdown_timer is not None:

            self.shutdown_timer.cancel()

        if rclpy.ok():

            rclpy.shutdown()

    # =========================================================================
    # CLEANUP
    # =========================================================================

    def destroy_node(self):

        # ---------------------------------------------------------------------
        # Safety stop if Ctrl+C occurs before normal completion.
        # ---------------------------------------------------------------------

        if not self.mission_finished:

            try:

                msg = Twist()

                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.linear.z = 0.0

                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = 0.0

                self.command_publisher.publish(msg)

            except Exception:
                pass

        # ---------------------------------------------------------------------
        # Finalize video if Ctrl+C occurs.
        # ---------------------------------------------------------------------

        if self.video_writer is not None:

            self.video_writer.release()

            self.video_writer = None

            print(
                f"Video saved to: {VIDEO_PATH}"
            )

        super().destroy_node()


# =============================================================================
# MAIN
# =============================================================================


def main(args=None):

    rclpy.init(args=args)

    node = BlueSimRoverCommand()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Rover experiment interrupted by user."
        )

    finally:

        node.destroy_node()


if __name__ == "__main__":
    main()