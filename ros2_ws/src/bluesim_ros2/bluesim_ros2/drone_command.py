#!/usr/bin/env python3

"""
===============================================================================
BlueSim - Drone ROS 2 Command and Camera Recording Example
===============================================================================

PURPOSE
-------
This script is an example/template for controlling the simulated Drone through
ROS 2 while simultaneously recording the Drone onboard camera.

The important Drone movement definitions are documented at the beginning of
this file so that a new user can understand how to operate the simulated Drone
without first reading the Unreal Engine Blueprint implementation.

A new user should normally only need to modify:

    MISSION = [
        ("increase_altitude", 5.0),
        ("yaw_right", 2.0),
        ("forward", 10.0),
        ("yaw_left", 2.0),
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
1. DRONE ROS 2 INTERFACE
===============================================================================

COMMAND TOPIC
-------------
    /cmd_drone_vel

MESSAGE TYPE
------------
    geometry_msgs/msg/Twist


CAMERA TOPIC
------------
    /camera_Drone

MESSAGE TYPE
------------
    sensor_msgs/msg/Image


===============================================================================
2. DRONE MOVEMENT CONVENTION
===============================================================================

The current Unreal Drone implementation uses the following ROS 2 command
mapping.

IMPORTANT:
These values are currently command values used by the Unreal Blueprint
control system. They should NOT automatically be interpreted as physical
metres/second or radians/second.


-----------------------------------------------------------------------------
ALTITUDE
-----------------------------------------------------------------------------

ROS:

    linear.z = +1.0
         Increase altitude / move upward

    linear.z = -1.0
         Decrease altitude / move downward

    linear.z = 0.0
         No vertical command


-----------------------------------------------------------------------------
FORWARD / BACKWARD
-----------------------------------------------------------------------------

The current Unreal Drone forward direction is implemented through the local
Y axis.

For the current Unreal implementation:

    Local Y = -1
         Forward

    Local Y = +1
         Backward


The ROS 2 interface used by the current Drone implementation therefore sends:

    linear.y = -1.0
         Forward

    linear.y = +1.0
         Backward

    linear.y = 0.0
         No forward/backward command


IMPORTANT:
The Drone forward command currently uses ROS linear.y rather than the more
common ROS linear.x convention because this matches the existing Unreal Drone
movement implementation.


-----------------------------------------------------------------------------
YAW
-----------------------------------------------------------------------------

For the current Unreal Drone:

    angular.z = +1.0
         Yaw RIGHT

    angular.z = -1.0
         Yaw LEFT

    angular.z = 0.0
         No yaw command


-----------------------------------------------------------------------------
LATERAL MOVEMENT
-----------------------------------------------------------------------------

The Drone has lateral movement available in the Unreal control system.

However, the current experiment implemented in this template does not use
lateral movement.

Therefore, this file currently defines and uses:

    - Altitude
    - Forward
    - Backward
    - Yaw left
    - Yaw right
    - Stop

Lateral movement can be added later using the appropriate ROS Twist field
once the Drone ROS/Unreal lateral-axis mapping is standardized.


===============================================================================
3. AVAILABLE HIGH-LEVEL MOVEMENT COMMANDS
===============================================================================

The experiment section uses readable high-level commands.

Available commands:

    "increase_altitude"
        Move upward.

    "decrease_altitude"
        Move downward.

    "forward"
        Move forward.

    "backward"
        Move backward.

    "yaw_right"
        Rotate/yaw right.

    "yaw_left"
        Rotate/yaw left.

    "stop"
        Stop all commanded movement.


COMMAND FORMAT
--------------

Each command uses:

    ("command_name", duration_in_seconds)


Examples:

    ("forward", 10.0)
         Move forward for 10 seconds.

    ("yaw_right", 2.0)
         Yaw right for 2 seconds.

    ("increase_altitude", 5.0)
         Increase altitude for 5 seconds.


===============================================================================
4. CAMERA INTERFACE
===============================================================================

The Drone camera is available through:

    /camera_Drone

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

    Unreal Drone Camera
            ↓
    SceneCaptureComponent2D
            ↓
    Render Target
            ↓
    rclUE Camera Publisher
            ↓
    /camera_Drone
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
5. VIDEO RECORDING
===============================================================================

The script waits for the first valid camera frame before starting movement.

This guarantees that the camera recording begins before the Drone starts its
mission.

The sequence is:

    Start script
         ↓
    Wait for valid camera frame
         ↓
    Create VideoWriter
         ↓
    Start Drone movement
         ↓
    Execute MISSION
         ↓
    Stop Drone
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

    BlueSim_Drone_Record.mp4


===============================================================================
6. COMMAND PUBLICATION RATE
===============================================================================

COMMAND RATE
------------

    20 Hz

That means a command is continuously published approximately every:

    0.05 seconds


VIDEO FPS
---------

    5 FPS

The current Drone camera publisher is configured for 5 Hz, so this template
uses 5 FPS for the video file.

The actual camera publication rate can be checked with:

    ros2 topic hz /camera_Drone


===============================================================================
7. EXPERIMENT DEFINITION
===============================================================================

The mission is defined using the MISSION list below.

Example:

    MISSION = [
        ("increase_altitude", 5.0),
        ("yaw_right", 2.0),
        ("forward", 10.0),
        ("yaw_left", 2.0),
        ("forward", 10.0),
        ("stop", 1.0),
    ]


This means:

    1. Increase altitude for 5 seconds.
    2. Yaw right for 2 seconds.
    3. Move forward for 10 seconds.
    4. Yaw left for 2 seconds.
    5. Move forward for 10 seconds.
    6. Stop.


To create a new experiment, simply modify MISSION.

For example:

    MISSION = [
        ("increase_altitude", 5.0),
        ("forward", 20.0),
        ("yaw_right", 3.0),
        ("forward", 15.0),
        ("stop", 1.0),
    ]


No ROS 2 publisher implementation needs to be changed.


===============================================================================
8. IMPORTANT PHYSICAL-UNIT NOTE
===============================================================================

The current Drone Unreal implementation converts the ROS command values into
its own movement system.

Therefore:

    linear.y = -1.0

does NOT necessarily mean:

    -1 metre/second

and:

    angular.z = +1.0

does NOT necessarily mean:

    +1 radian/second.

The distance travelled, altitude gained, and angle turned depend on the
current Unreal Engine Drone movement implementation and the duration of each
command.


===============================================================================
9. SAFETY / STOP BEHAVIOR
===============================================================================

When the mission finishes, the script sends:

    linear.x = 0
    linear.y = 0
    linear.z = 0

    angular.x = 0
    angular.y = 0
    angular.z = 0

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
# DRONE CONTROL DEFINITIONS
# =============================================================================

# -----------------------------------------------------------------------------
# ROS 2 topics
# -----------------------------------------------------------------------------

COMMAND_TOPIC = "/cmd_drone_vel"
CAMERA_TOPIC = "/camera_Drone"


# -----------------------------------------------------------------------------
# ROS 2 movement command values
# -----------------------------------------------------------------------------
#
# These values correspond to the current Unreal Drone implementation.
#
# Altitude:
#     +1  increase altitude
#     -1  decrease altitude
#
# Forward/backward:
#     linear.y = -1  forward
#     linear.y = +1  backward
#
# Yaw:
#     +1  right
#     -1  left
#
# These are control values, not guaranteed physical units.

INCREASE_ALTITUDE_COMMAND = 1.0
DECREASE_ALTITUDE_COMMAND = -1.0

FORWARD_COMMAND = -1.0
BACKWARD_COMMAND = 1.0

YAW_RIGHT_COMMAND = 1.0
YAW_LEFT_COMMAND = -1.0

ZERO_COMMAND = 0.0


# -----------------------------------------------------------------------------
# Command publication rate
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
# Automatically determine the ROS 2 workspace:
#
# BlueSim/
# └── ros2_ws/
#     └── src/
#         └── bluesim_ros2/
#             └── bluesim_ros2/
#                 └── drone_command.py
#
# parents[3] = ros2_ws
# -----------------------------------------------------------------------------

ROS2_WORKSPACE = Path(__file__).resolve().parents[3]

VIDEO_DIRECTORY = ROS2_WORKSPACE / "Recorded Videos"

VIDEO_FILENAME = "BlueSim_Drone_Record.mp4"

VIDEO_PATH = VIDEO_DIRECTORY / VIDEO_FILENAME


# =============================================================================
# WRITE YOUR EXPERIMENT HERE
# =============================================================================
#
# Available commands:
#
#     "increase_altitude"
#     "decrease_altitude"
#     "forward"
#     "backward"
#     "yaw_right"
#     "yaw_left"
#     "stop"
#
# Command format:
#
#     ("command", duration_in_seconds)
#
# =============================================================================

MISSION = [

    # 1. Increase altitude.
    ("increase_altitude", 5.0),

    # 2. Yaw right.
    ("yaw_right", 2.0),

    # 3. Move forward.
    ("forward", 10.0),

    # 4. Yaw left.
    ("yaw_left", 2.0),

    # 5. Move forward again.
    ("forward", 10.0),

    # 6. Stop.
    ("stop", 1.0),
]


# =============================================================================
# DRONE ROS 2 NODE
# =============================================================================


class BlueSimDroneCommand(Node):

    def __init__(self):

        super().__init__("bluesim_drone_command")

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
            "BlueSim Drone Command"
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
        linear_y: float = 0.0,
        linear_z: float = 0.0,
        angular_z: float = 0.0,
    ):

        if self.mission_finished:
            return

        msg = Twist()

        # ---------------------------------------------------------------------
        # Linear components
        # ---------------------------------------------------------------------

        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.linear.z = float(linear_z)

        # ---------------------------------------------------------------------
        # Angular components
        # ---------------------------------------------------------------------

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        self.command_publisher.publish(msg)

    # =========================================================================
    # HIGH-LEVEL DRONE COMMAND DEFINITIONS
    # =========================================================================
    #
    # These functions translate the readable MISSION commands into ROS 2
    # Twist values.
    # =========================================================================

    def execute_command(self, command_name: str):

        if command_name == "increase_altitude":

            self.publish_command(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=INCREASE_ALTITUDE_COMMAND,
                angular_z=0.0,
            )

        elif command_name == "decrease_altitude":

            self.publish_command(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=DECREASE_ALTITUDE_COMMAND,
                angular_z=0.0,
            )

        elif command_name == "forward":

            self.publish_command(
                linear_x=0.0,
                linear_y=FORWARD_COMMAND,
                linear_z=0.0,
                angular_z=0.0,
            )

        elif command_name == "backward":

            self.publish_command(
                linear_x=0.0,
                linear_y=BACKWARD_COMMAND,
                linear_z=0.0,
                angular_z=0.0,
            )

        elif command_name == "yaw_right":

            self.publish_command(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=0.0,
                angular_z=YAW_RIGHT_COMMAND,
            )

        elif command_name == "yaw_left":

            self.publish_command(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=0.0,
                angular_z=YAW_LEFT_COMMAND,
            )

        elif command_name == "stop":

            self.publish_command(
                linear_x=0.0,
                linear_y=0.0,
                linear_z=0.0,
                angular_z=0.0,
            )

        else:

            self.get_logger().error(
                f"Unknown Drone command: '{command_name}'"
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
            "increase_altitude",
            "decrease_altitude",
            "forward",
            "backward",
            "yaw_right",
            "yaw_left",
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
        # Mission hasn't started because camera recording isn't ready yet.
        # ---------------------------------------------------------------------

        if not self.mission_started:

            self.publish_command()

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
        # Calculate elapsed time.
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
                        "Drone camera data is smaller than expected."
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
                        "Drone camera data is smaller than expected."
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
                        "Drone camera data is smaller than expected."
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
                    f"Unsupported Drone camera encoding: "
                    f"{msg.encoding}"
                )

                return

            # =================================================================
            # Create VideoWriter ONCE
            # =================================================================

            if self.video_writer is None:

                # Do not recreate a writer after completion.
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

                # Start the Drone only after the video writer is ready.
                self.start_mission()

            # =================================================================
            # Write frame
            # =================================================================

            if (
                self.video_writer is not None
                and not self.mission_finished
            ):

                self.video_writer.write(frame)

                self.video_frame_count += 1

        except Exception as exc:

            self.get_logger().error(
                f"Drone camera processing failed: {exc}"
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
        # This prevents another camera callback from recreating the writer.
        # ---------------------------------------------------------------------

        self.mission_finished = True

        # ---------------------------------------------------------------------
        # Stop Drone.
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
            "Drone stopped."
        )

        # ---------------------------------------------------------------------
        # Stop command timer.
        # ---------------------------------------------------------------------

        if self.control_timer is not None:

            self.control_timer.cancel()

        # ---------------------------------------------------------------------
        # Stop camera subscription.
        # ---------------------------------------------------------------------

        if self.camera_subscription is not None:

            self.destroy_subscription(
                self.camera_subscription
            )

            self.camera_subscription = None

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
                "No valid Drone camera frames were recorded."
            )

        self.get_logger().info(
            "Drone experiment completed."
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

    node = BlueSimDroneCommand()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Drone experiment interrupted by user."
        )

    finally:

        node.destroy_node()


if __name__ == "__main__":
    main()