#!/usr/bin/env python3

"""
===============================================================================
BlueSim - BlueBoat ROS 2 Command and Camera Recording Example
===============================================================================

PURPOSE
-------
This script is an example/template for controlling the simulated BlueBoat
through ROS 2 while simultaneously recording the BlueBoat onboard camera.

The important information needed to create a BlueBoat experiment is defined
at the beginning of this file.

A new user should normally only need to modify:

    MISSION = [
        ("turn_right", 5.0),
        ("forward", 10.0),
        ("backward", 5.0),
        ("turn_left", 5.0),
        ("stop", 1.0),
    ]

The rest of the file implements the ROS 2 communication, camera reception,
video recording, and experiment state machine.

===============================================================================
1. BLUEBOAT ROS 2 INTERFACE
===============================================================================

COMMAND TOPIC
-------------
    /cmd_BlueBoat_vel

MESSAGE TYPE
------------
    geometry_msgs/msg/Twist

The relevant fields are:

    msg.linear.x
        Forward / backward command

    msg.angular.z
        Left / right turning command


===============================================================================
2. BLUEBOAT MOVEMENT CONVENTION
===============================================================================

The current BlueBoat Unreal Engine control system uses the following
ROS 2 command convention:

    linear.x = +1.0
        → Move FORWARD

    linear.x = -1.0
        → Move BACKWARD

    linear.x = 0.0
        → No forward/backward driving force


    angular.z = +1.0
        → Turn RIGHT

    angular.z = -1.0
        → Turn LEFT

    angular.z = 0.0
        → No turning command


IMPORTANT
---------
The BlueBoat currently uses Unreal Engine force/torque physics.

Therefore these values should NOT currently be interpreted as:

    linear.x  = metres/second
    angular.z = radians/second

Instead, they are command values that are converted inside the Unreal
Blueprint into force and torque.

For example:

    linear.x = 1.0
        → positive BoatThrottle
        → forward force

    angular.z = 1.0
        → positive BoatSteering
        → right turn

The exact distance travelled and angle turned therefore depend on the
current Unreal physics configuration and command duration.


===============================================================================
3. AVAILABLE HIGH-LEVEL MOVEMENT COMMANDS
===============================================================================

The experiment section can use these predefined commands:

    "forward"
        Move forward.

    "backward"
        Move backward.

    "turn_left"
        Turn left.

    "turn_right"
        Turn right.

    "stop"
        Stop applying movement commands.

Each command has this format:

    ("command_name", duration_in_seconds)

Examples:

    ("forward", 10.0)
        → Move forward for 10 seconds.

    ("backward", 5.0)
        → Move backward for 5 seconds.

    ("turn_right", 4.0)
        → Turn right for 4 seconds.


===============================================================================
4. CAMERA INTERFACE
===============================================================================

CAMERA TOPIC
------------
    /camera_BlueBoat

MESSAGE TYPE
------------
    sensor_msgs/msg/Image

CURRENT CAMERA CONFIGURATION
----------------------------
    Width       : 640
    Height      : 360
    Encoding    : bgr8
    Step        : 1920

The camera image is received through ROS 2 and converted into a NumPy array
for OpenCV processing and video recording.

Camera pipeline:

    Unreal Camera
          ↓
    SceneCaptureComponent2D
          ↓
    Render Target
          ↓
    rclUE Camera Publisher
          ↓
    /camera_BlueBoat
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

The experiment starts recording before robot movement begins.

The script waits for the first valid camera frame.

Then:

    First valid frame
          ↓
    Open video file
          ↓
    Start movement
          ↓
    Execute experiment
          ↓
    Stop robot
          ↓
    Release VideoWriter
          ↓
    Save video
          ↓
    Terminate


VIDEO OUTPUT DIRECTORY
----------------------
The video is saved inside:

    BlueSim/ros2_ws/Recorded Videos/

OUTPUT FILE:

    BlueSim_BlueBoat_Record.mp4


===============================================================================
6. CONTROL RATE
===============================================================================

COMMAND PUBLISH RATE
--------------------
    20 Hz

Commands are repeatedly published because continuous publication is used
instead of publishing only once.

    20 Hz = one command approximately every 0.05 seconds.


VIDEO FPS
---------
    20 FPS

This is the nominal FPS used when creating the MP4 file.

The actual ROS 2 camera publication rate can be checked independently with:

    ros2 topic hz /camera_BlueBoat


===============================================================================
7. EXPERIMENT DESIGN
===============================================================================

The experiment is defined below in the MISSION list.

Example:

    MISSION = [
        ("turn_right", 5.0),
        ("forward", 10.0),
        ("backward", 5.0),
        ("turn_left", 5.0),
        ("stop", 1.0),
    ]

This means:

    1. Turn right for 5 seconds.
    2. Move forward for 10 seconds.
    3. Move backward for 5 seconds.
    4. Turn left for 5 seconds.
    5. Stop.

To create another experiment, simply change the MISSION list.

For example:

    MISSION = [
        ("forward", 20.0),
        ("turn_right", 4.0),
        ("forward", 10.0),
        ("stop", 1.0),
    ]

No ROS publisher code needs to be changed.


===============================================================================
8. IMPORTANT SAFETY / STOP BEHAVIOR
===============================================================================

When the experiment finishes, the script sends:

    linear.x  = 0.0
    angular.z = 0.0

before finalizing the video.

If the user presses Ctrl+C during the experiment, the script also attempts
to send a zero command before shutting down.

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
# BLUEBOAT CONTROL DEFINITIONS
# =============================================================================

# -----------------------------------------------------------------------------
# ROS 2 topics
# -----------------------------------------------------------------------------

COMMAND_TOPIC = "/cmd_BlueBoat_vel"
CAMERA_TOPIC = "/camera_BlueBoat"


# -----------------------------------------------------------------------------
# ROS 2 message types
# -----------------------------------------------------------------------------

# Command:
#     geometry_msgs/msg/Twist
#
# Camera:
#     sensor_msgs/msg/Image
#
# These are represented by Twist and Image imports below.


# -----------------------------------------------------------------------------
# BlueBoat movement command definitions
# -----------------------------------------------------------------------------
#
# DO NOT confuse these values with physical velocity units.
#
# The current Unreal implementation interprets these values through the
# BoatThrottle and BoatSteering logic.

FORWARD_COMMAND = 1.0
BACKWARD_COMMAND = -1.0

TURN_RIGHT_COMMAND = 1.0
TURN_LEFT_COMMAND = -1.0

ZERO_COMMAND = 0.0


# -----------------------------------------------------------------------------
# ROS command publication rate
# -----------------------------------------------------------------------------

COMMAND_RATE_HZ = 20.0


# -----------------------------------------------------------------------------
# Camera / video definition
# -----------------------------------------------------------------------------

VIDEO_FPS = 20.0

EXPECTED_CAMERA_WIDTH = 640
EXPECTED_CAMERA_HEIGHT = 360
EXPECTED_CAMERA_ENCODING = "bgr8"


# -----------------------------------------------------------------------------
# Video output
# -----------------------------------------------------------------------------
#
# The script automatically finds:
#
#     BlueSim/ros2_ws/
#
# relative to this Python file.
#
# Therefore no user-specific absolute path is required.

ROS2_WORKSPACE = Path(__file__).resolve().parents[3]

VIDEO_DIRECTORY = ROS2_WORKSPACE / "Recorded Videos"

VIDEO_FILENAME = "BlueSim_BlueBoat_Record.mp4"

VIDEO_PATH = VIDEO_DIRECTORY / VIDEO_FILENAME


# =============================================================================
# WRITE YOUR EXPERIMENT HERE
# =============================================================================
#
# COMMAND FORMAT:
#
#     ("command", duration_seconds)
#
# AVAILABLE COMMANDS:
#
#     "forward"
#     "backward"
#     "turn_left"
#     "turn_right"
#     "stop"
#
#
# The experiment below is only an example.
#
# A user can change the movement sequence without changing the ROS 2 code.
#
# =============================================================================

MISSION = [

    # 1. Rotate the BlueBoat to the right.
    ("turn_right", 5.0),

    # 2. Move forward.
    ("forward", 10.0),

    # 3. Move backward.
    ("backward", 5.0),

    # 4. Rotate back to the left.
    ("turn_left", 5.0),

    # 5. Stop.
    ("stop", 1.0),
]


# =============================================================================
# BLUEBOAT ROS 2 NODE
# =============================================================================


class BlueSimBlueBoatCommand(Node):

    def __init__(self):

        super().__init__("bluesim_blueboat_command")

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
            "BlueSim BlueBoat Command"
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
        linear_x: float,
        angular_z: float,
    ):

        if self.mission_finished:
            return

        msg = Twist()

        # ---------------------------------------------------------------------
        # Linear component
        # ---------------------------------------------------------------------

        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        # ---------------------------------------------------------------------
        # Angular component
        # ---------------------------------------------------------------------

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        self.command_publisher.publish(msg)

    # =========================================================================
    # HIGH-LEVEL MOVEMENT COMMAND DEFINITIONS
    # =========================================================================
    #
    # These functions make the experiment readable.
    #
    # The user can simply write:
    #
    #     ("forward", 10.0)
    #
    # instead of manually constructing Twist messages.
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
                f"Unknown BlueBoat command: '{command_name}'"
            )

            self.mission_finished = True

    # =========================================================================
    # START MISSION
    # =========================================================================

    def start_mission(self):

        if self.mission_started:
            return

        if self.mission_finished:
            return

        if not self.MISSION_is_valid():

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

    def MISSION_is_valid(self):

        valid_commands = {
            "forward",
            "backward",
            "turn_left",
            "turn_right",
            "stop",
        }

        for item in MISSION:

            if not isinstance(item, tuple):

                self.get_logger().error(
                    "Every MISSION entry must be a tuple:"
                    " ('command', duration)"
                )

                return False

            if len(item) != 2:

                self.get_logger().error(
                    "Every MISSION entry must contain exactly "
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
        # Check whether the mission is complete.
        # ---------------------------------------------------------------------

        if self.current_mission_index >= len(MISSION):

            self.current_command = "stop"

            self.current_command_duration = 0.0

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
        # ---------------------------------------------------------------------

        if not self.mission_started:

            self.publish_command(
                linear_x=ZERO_COMMAND,
                angular_z=ZERO_COMMAND,
            )

            return

        # ---------------------------------------------------------------------
        # Mission has already finished.
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
        # Command duration completed.
        # ---------------------------------------------------------------------

        if elapsed >= self.current_command_duration:

            self.current_mission_index += 1

            self.start_next_command()

            return

        # ---------------------------------------------------------------------
        # Continuously publish the current command.
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
                        "Camera data is smaller than expected."
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
                        "Camera data is smaller than expected."
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
                        "Camera data is smaller than expected."
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
                    f"Unsupported camera encoding: "
                    f"{msg.encoding}"
                )

                return

            # =================================================================
            # Create VideoWriter using the first valid frame
            # =================================================================

            if self.video_writer is None:

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
                        f"Could not open video file:\n"
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

                # -------------------------------------------------------------
                # Start movement only after the video writer is ready.
                # -------------------------------------------------------------

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
                f"Camera processing failed: {exc}"
            )

    # =========================================================================
    # FINISH EXPERIMENT
    # =========================================================================

    def finish_experiment(self):

        if self.mission_finished:
            return

        # ---------------------------------------------------------------------
        # Set this FIRST.
        #
        # Any camera callback arriving after this immediately returns.
        # ---------------------------------------------------------------------

        self.mission_finished = True

        # ---------------------------------------------------------------------
        # Stop BlueBoat.
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
            "BlueBoat stopped."
        )

        # ---------------------------------------------------------------------
        # Stop control timer.
        # ---------------------------------------------------------------------

        if self.control_timer is not None:

            self.control_timer.cancel()

        # ---------------------------------------------------------------------
        # Stop camera subscription.
        #
        # This prevents the VideoWriter from being reopened after release.
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
                f"Video saved to:\n{VIDEO_PATH}"
            )

            self.get_logger().info(
                f"Frames recorded: "
                f"{self.video_frame_count}"
            )

        else:

            self.get_logger().warning(
                "No valid camera frames were recorded."
            )

        self.get_logger().info(
            "BlueBoat experiment completed."
        )

        # ---------------------------------------------------------------------
        # Request clean ROS shutdown.
        # ---------------------------------------------------------------------

        self.shutdown_timer = self.create_timer(
            0.1,
            self.shutdown_callback,
        )

    # =========================================================================
    # CLEAN ROS SHUTDOWN
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
        # Safety stop if the experiment was interrupted.
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
        # Finalize video if Ctrl+C occurs before normal completion.
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

    node = BlueSimBlueBoatCommand()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "BlueBoat experiment interrupted by user."
        )

    finally:

        node.destroy_node()


if __name__ == "__main__":
    main()