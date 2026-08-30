#!/usr/bin/env python3

"""
===============================================================================
BlueSim - Multi-Robot ROS 2 Experiment
===============================================================================

PURPOSE
-------
This script controls the BlueSim BlueBoat, Drone, and Rover simultaneously
through ROS 2 while recording their three onboard cameras into ONE video.

The experiment is intentionally defined in a simple high-level form near the
beginning of this file.

A new user should normally only need to modify the three MISSION definitions.

===============================================================================
1. ROBOTS
===============================================================================

    BlueBoat
    Drone
    Rover


===============================================================================
2. ROS 2 COMMAND TOPICS
===============================================================================

BLUEBOAT:

    /cmd_BlueBoat_vel

    geometry_msgs/msg/Twist


DRONE:

    /cmd_drone_vel

    geometry_msgs/msg/Twist


ROVER:

    /cmd_rover_vel

    geometry_msgs/msg/Twist


===============================================================================
3. ROS 2 CAMERA TOPICS
===============================================================================

BLUEBOAT:

    /camera_BlueBoat


DRONE:

    /camera_Drone


ROVER:

    /camera_Rover


Camera message type:

    sensor_msgs/msg/Image


===============================================================================
4. BLUEBOAT MOVEMENT DEFINITIONS
===============================================================================

Current BlueSim BlueBoat convention:

    linear.x = +1.0
        → Forward

    linear.x = -1.0
        → Backward

    angular.z = +1.0
        → Turn RIGHT

    angular.z = -1.0
        → Turn LEFT

    angular.z = 0.0
        → No turning


===============================================================================
5. DRONE MOVEMENT DEFINITIONS
===============================================================================

Current BlueSim Drone convention:

    linear.z = +1.0
        → Elevate / move UP

    linear.z = -1.0
        → Move DOWN


Forward/backward:

    linear.y = -1.0
        → Forward

    linear.y = +1.0
        → Backward


Yaw:

    angular.z = +1.0
        → Yaw RIGHT

    angular.z = -1.0
        → Yaw LEFT


NOTE:
The Drone currently uses linear.y for forward/backward because this matches
the current Unreal Drone movement implementation.


===============================================================================
6. ROVER MOVEMENT DEFINITIONS
===============================================================================

Current BlueSim Rover convention:

    linear.x = -1.0
        → Forward

    linear.x = +1.0
        → Backward


Steering:

    angular.z = -1.0
        → Turn RIGHT

    angular.z = +1.0
        → Turn LEFT

    angular.z = 0.0
        → No steering


IMPORTANT:
These values are command values used by the current Unreal physics system.

They are NOT guaranteed to represent:

    metres/second
    radians/second

Actual distance and rotation depend on the Unreal physics configuration and
the duration of each command.


===============================================================================
7. HIGH-LEVEL COMMANDS
===============================================================================

BLUEBOAT:

    "forward"
    "backward"
    "turn_left"
    "turn_right"
    "stop"


DRONE:

    "increase_altitude"
    "decrease_altitude"
    "forward"
    "backward"
    "yaw_left"
    "yaw_right"
    "stop"


ROVER:

    "forward"
    "backward"
    "turn_left"
    "turn_right"
    "stop"


Command format:

    ("command_name", duration_seconds)


Example:

    ("forward", 10.0)

means:

    Move forward for 10 seconds.


===============================================================================
8. EXPERIMENT
===============================================================================

All three missions start simultaneously.

BLUEBOAT:

    Turn LEFT  for 2 seconds
    Forward    for 10 seconds
    Turn RIGHT for 4 seconds
    Forward    for 10 seconds
    Stop       for 1 second


DRONE:

    Elevate    for 5 seconds
    Forward    for 10 seconds
    Yaw LEFT   for 4 seconds
    Forward    for 10 seconds
    Stop       for 1 second


ROVER:

    Turn RIGHT for 2 seconds
    Forward    for 10 seconds
    Turn LEFT  for 5 seconds
    Forward    for 10 seconds
    Stop       for 1 second


To create a different experiment, modify the MISSION lists below.


===============================================================================
9. VIDEO RECORDING
===============================================================================

The script waits until all three cameras have produced at least one valid
image.

Then:

    All cameras valid
          ↓
    Start video recording
          ↓
    2-second pre-roll
          ↓
    Start all three robot missions simultaneously
          ↓
    Record all three cameras
          ↓
    All missions complete
          ↓
    1-second post-roll
          ↓
    Stop all robots
          ↓
    Save video
          ↓
    Terminate


VIDEO FORMAT:

    MP4

VIDEO LAYOUT:

    ┌──────────────┬──────────────┐
    │   BlueBoat   │    Drone     │
    │    Camera    │    Camera    │
    ├──────────────┼──────────────┤
    │    Rover     │   Experiment │
    │    Camera    │    Status    │
    └──────────────┴──────────────┘


===============================================================================
10. VIDEO OUTPUT
===============================================================================

The video is saved to:

    BlueSim/ros2_ws/Recorded Videos/

Filename:

    BlueSim_MultiRobot_Record.mp4


===============================================================================
11. CONTROL RATE
===============================================================================

Robot command publication:

    20 Hz

Video:

    5 FPS

The video uses the latest frame received from each camera.

This is important because the three cameras may not publish at exactly the
same frequency.


===============================================================================
"""

import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


# =============================================================================
# GENERAL DEFINITIONS
# =============================================================================

COMMAND_RATE_HZ = 20.0

VIDEO_FPS = 5.0

PRE_ROLL_DURATION = 2.0

POST_ROLL_DURATION = 1.0


# =============================================================================
# ROS 2 TOPICS
# =============================================================================

BLUEBOAT_COMMAND_TOPIC = "/cmd_BlueBoat_vel"
DRONE_COMMAND_TOPIC = "/cmd_drone_vel"
ROVER_COMMAND_TOPIC = "/cmd_rover_vel"

BLUEBOAT_CAMERA_TOPIC = "/camera_BlueBoat"
DRONE_CAMERA_TOPIC = "/camera_Drone"
ROVER_CAMERA_TOPIC = "/camera_Rover"


# =============================================================================
# MOVEMENT COMMAND DEFINITIONS
# =============================================================================

# -----------------------------------------------------------------------------
# BlueBoat
# -----------------------------------------------------------------------------

BLUEBOAT_FORWARD = 1.0
BLUEBOAT_BACKWARD = -1.0

BLUEBOAT_RIGHT = 1.0
BLUEBOAT_LEFT = -1.0


# -----------------------------------------------------------------------------
# Drone
# -----------------------------------------------------------------------------

DRONE_UP = 1.0
DRONE_DOWN = -1.0

DRONE_FORWARD = -1.0
DRONE_BACKWARD = 1.0

DRONE_RIGHT = 1.0
DRONE_LEFT = -1.0


# -----------------------------------------------------------------------------
# Rover
# -----------------------------------------------------------------------------

ROVER_FORWARD = -1.0
ROVER_BACKWARD = 1.0

ROVER_RIGHT = -1.0
ROVER_LEFT = 1.0


# =============================================================================
# VIDEO OUTPUT
# =============================================================================

ROS2_WORKSPACE = Path(__file__).resolve().parents[3]

VIDEO_DIRECTORY = ROS2_WORKSPACE / "Recorded Videos"

VIDEO_FILENAME = "BlueSim_MultiRobot_Record.mp4"

VIDEO_PATH = VIDEO_DIRECTORY / VIDEO_FILENAME


# =============================================================================
# WRITE YOUR THREE ROBOT EXPERIMENTS HERE
# =============================================================================

# -----------------------------------------------------------------------------
# BLUEBOAT MISSION
# -----------------------------------------------------------------------------

BLUEBOAT_MISSION = [

    # Turn left.
    ("turn_left", 2.0),

    # Move forward.
    ("forward", 10.0),

    # Turn right.
    ("turn_right", 4.0),

    # Move forward again.
    ("forward", 10.0),

    # Stop.
    ("stop", 1.0),
]


# -----------------------------------------------------------------------------
# DRONE MISSION
# -----------------------------------------------------------------------------

DRONE_MISSION = [

    # Take off / elevate.
    ("increase_altitude", 5.0),

    # Move forward.
    ("forward", 10.0),

    # Yaw left.
    ("yaw_left", 4.0),

    # Move forward again.
    ("forward", 10.0),

    # Stop.
    ("stop", 1.0),
]


# -----------------------------------------------------------------------------
# ROVER MISSION
# -----------------------------------------------------------------------------

ROVER_MISSION = [

    # Turn right.
    ("turn_right", 2.0),

    # Move forward.
    ("forward", 10.0),

    # Turn left.
    ("turn_left", 5.0),

    # Move forward again.
    ("forward", 10.0),

    # Stop.
    ("stop", 1.0),
]


# =============================================================================
# HELPER CLASS
# =============================================================================


class TimedMission:

    def __init__(self, mission):

        self.mission = mission

        self.total_duration = sum(
            duration
            for _, duration in mission
        )

    def get_command(
        self,
        elapsed: float,
    ):

        current_time = 0.0

        for command, duration in self.mission:

            if elapsed < current_time + duration:

                return command

            current_time += duration

        return "stop"


# =============================================================================
# MULTI-ROBOT ROS 2 NODE
# =============================================================================


class BlueSimMultiRobotExperiment(Node):

    def __init__(self):

        super().__init__(
            "bluesim_multi_robot_experiment"
        )

        # =====================================================================
        # ROS 2 COMMAND PUBLISHERS
        # =====================================================================

        self.blueboat_publisher = self.create_publisher(
            Twist,
            BLUEBOAT_COMMAND_TOPIC,
            10,
        )

        self.drone_publisher = self.create_publisher(
            Twist,
            DRONE_COMMAND_TOPIC,
            10,
        )

        self.rover_publisher = self.create_publisher(
            Twist,
            ROVER_COMMAND_TOPIC,
            10,
        )

        # =====================================================================
        # CAMERA SUBSCRIBERS
        # =====================================================================

        self.blueboat_camera_subscription = (
            self.create_subscription(
                Image,
                BLUEBOAT_CAMERA_TOPIC,
                self.blueboat_camera_callback,
                10,
            )
        )

        self.drone_camera_subscription = (
            self.create_subscription(
                Image,
                DRONE_CAMERA_TOPIC,
                self.drone_camera_callback,
                10,
            )
        )

        self.rover_camera_subscription = (
            self.create_subscription(
                Image,
                ROVER_CAMERA_TOPIC,
                self.rover_camera_callback,
                10,
            )
        )

        # =====================================================================
        # LATEST CAMERA FRAMES
        # =====================================================================

        self.blueboat_frame: Optional[np.ndarray] = None

        self.drone_frame: Optional[np.ndarray] = None

        self.rover_frame: Optional[np.ndarray] = None

        # =====================================================================
        # VIDEO
        # =====================================================================

        self.video_writer = None

        self.video_frame_count = 0

        self.recording_started = False

        # =====================================================================
        # EXPERIMENT STATE
        # =====================================================================

        self.experiment_started = False

        self.experiment_finished = False

        self.shutdown_requested = False

        self.pre_roll_start_time = None

        self.mission_start_time = None

        self.post_roll_start_time = None

        # =====================================================================
        # MISSIONS
        # =====================================================================

        self.blueboat_mission = TimedMission(
            BLUEBOAT_MISSION
        )

        self.drone_mission = TimedMission(
            DRONE_MISSION
        )

        self.rover_mission = TimedMission(
            ROVER_MISSION
        )

        self.total_mission_duration = max(
            self.blueboat_mission.total_duration,
            self.drone_mission.total_duration,
            self.rover_mission.total_duration,
        )

        # =====================================================================
        # LAST COMMANDS
        # =====================================================================

        self.last_blueboat_command = "WAITING"

        self.last_drone_command = "WAITING"

        self.last_rover_command = "WAITING"

        # =====================================================================
        # CONTROL TIMER
        # =====================================================================

        self.control_timer = self.create_timer(
            1.0 / COMMAND_RATE_HZ,
            self.control_loop,
        )

        # =====================================================================
        # VIDEO TIMER
        # =====================================================================

        self.video_timer = self.create_timer(
            1.0 / VIDEO_FPS,
            self.video_loop,
        )

        # =====================================================================
        # LOGGING
        # =====================================================================

        self.get_logger().info(
            "=================================================="
        )

        self.get_logger().info(
            "BlueSim Multi-Robot Experiment"
        )

        self.get_logger().info(
            "=================================================="
        )

        self.get_logger().info(
            f"BlueBoat command : "
            f"{BLUEBOAT_COMMAND_TOPIC}"
        )

        self.get_logger().info(
            f"Drone command    : "
            f"{DRONE_COMMAND_TOPIC}"
        )

        self.get_logger().info(
            f"Rover command    : "
            f"{ROVER_COMMAND_TOPIC}"
        )

        self.get_logger().info(
            f"BlueBoat camera  : "
            f"{BLUEBOAT_CAMERA_TOPIC}"
        )

        self.get_logger().info(
            f"Drone camera     : "
            f"{DRONE_CAMERA_TOPIC}"
        )

        self.get_logger().info(
            f"Rover camera     : "
            f"{ROVER_CAMERA_TOPIC}"
        )

        self.get_logger().info(
            f"Video output     : "
            f"{VIDEO_PATH}"
        )

        self.get_logger().info(
            "Waiting for valid frames from all "
            "three cameras..."
        )

    # =========================================================================
    # TWIST HELPERS
    # =========================================================================

    @staticmethod
    def create_twist(
        linear_x=0.0,
        linear_y=0.0,
        linear_z=0.0,
        angular_z=0.0,
    ):

        msg = Twist()

        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.linear.z = float(linear_z)

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        return msg

    # =========================================================================
    # CAMERA DECODING
    # =========================================================================

    def decode_image(
        self,
        msg: Image,
    ) -> Optional[np.ndarray]:

        if (
            msg.width <= 0
            or msg.height <= 0
            or len(msg.data) == 0
        ):

            return None

        try:

            if msg.encoding == "bgr8":

                channels = 3

                row_size = msg.width * channels

                raw = np.frombuffer(
                    msg.data,
                    dtype=np.uint8,
                )

                required_size = (
                    msg.height * msg.step
                )

                if raw.size < required_size:

                    return None

                image = raw[
                    :required_size
                ].reshape(
                    msg.height,
                    msg.step,
                )

                image = image[
                    :,
                    :row_size,
                ].reshape(
                    msg.height,
                    msg.width,
                    channels,
                )

                return image.copy()

            elif msg.encoding == "rgb8":

                channels = 3

                row_size = msg.width * channels

                raw = np.frombuffer(
                    msg.data,
                    dtype=np.uint8,
                )

                required_size = (
                    msg.height * msg.step
                )

                if raw.size < required_size:

                    return None

                image = raw[
                    :required_size
                ].reshape(
                    msg.height,
                    msg.step,
                )

                image = image[
                    :,
                    :row_size,
                ].reshape(
                    msg.height,
                    msg.width,
                    channels,
                )

                image = cv2.cvtColor(
                    image,
                    cv2.COLOR_RGB2BGR,
                )

                return image

            elif msg.encoding == "mono8":

                raw = np.frombuffer(
                    msg.data,
                    dtype=np.uint8,
                )

                required_size = (
                    msg.height * msg.step
                )

                if raw.size < required_size:

                    return None

                image = raw[
                    :required_size
                ].reshape(
                    msg.height,
                    msg.step,
                )

                image = image[
                    :,
                    :msg.width,
                ]

                image = cv2.cvtColor(
                    image,
                    cv2.COLOR_GRAY2BGR,
                )

                return image

            else:

                self.get_logger().warning(
                    f"Unsupported camera encoding: "
                    f"{msg.encoding}"
                )

                return None

        except Exception as exc:

            self.get_logger().error(
                f"Camera decoding failed: {exc}"
            )

            return None

    # =========================================================================
    # CAMERA CALLBACKS
    # =========================================================================

    def blueboat_camera_callback(
        self,
        msg: Image,
    ):

        frame = self.decode_image(msg)

        if frame is not None:

            self.blueboat_frame = frame

    # -------------------------------------------------------------------------

    def drone_camera_callback(
        self,
        msg: Image,
    ):

        frame = self.decode_image(msg)

        if frame is not None:

            self.drone_frame = frame

    # -------------------------------------------------------------------------

    def rover_camera_callback(
        self,
        msg: Image,
    ):

        frame = self.decode_image(msg)

        if frame is not None:

            self.rover_frame = frame

    # =========================================================================
    # CAMERA READINESS
    # =========================================================================

    def all_cameras_ready(self):

        return (
            self.blueboat_frame is not None
            and self.drone_frame is not None
            and self.rover_frame is not None
        )

    # =========================================================================
    # START RECORDING
    # =========================================================================

    def start_recording(self):

        if self.recording_started:
            return

        if not self.all_cameras_ready():
            return

        VIDEO_DIRECTORY.mkdir(
            parents=True,
            exist_ok=True,
        )

        # =====================================================================
        # Normalize all cameras to 640x360.
        # =====================================================================

        target_width = 640
        target_height = 360

        # =====================================================================
        # Composite video:
        #
        # BlueBoat | Drone
        # Rover    | Status
        #
        # Final:
        #
        # 1280 x 720
        # =====================================================================

        video_width = target_width * 2
        video_height = target_height * 2

        self.video_writer = cv2.VideoWriter(
            str(VIDEO_PATH),
            cv2.VideoWriter_fourcc(*"mp4v"),
            VIDEO_FPS,
            (
                video_width,
                video_height,
            ),
        )

        if not self.video_writer.isOpened():

            self.get_logger().error(
                f"Could not open video file:\n"
                f"{VIDEO_PATH}"
            )

            self.video_writer = None

            return

        self.recording_started = True

        self.pre_roll_start_time = time.monotonic()

        self.get_logger().info(
            "Video recording started."
        )

        self.get_logger().info(
            f"Video resolution: "
            f"{video_width}x{video_height}"
        )

        self.get_logger().info(
            f"Video FPS: "
            f"{VIDEO_FPS:.1f}"
        )

        self.get_logger().info(
            f"Pre-roll: "
            f"{PRE_ROLL_DURATION:.1f} seconds"
        )

    # =========================================================================
    # COMMAND PUBLISHING
    # =========================================================================

    def publish_blueboat_command(
        self,
        command: str,
    ):

        if command == "forward":

            msg = self.create_twist(
                linear_x=BLUEBOAT_FORWARD
            )

        elif command == "backward":

            msg = self.create_twist(
                linear_x=BLUEBOAT_BACKWARD
            )

        elif command == "turn_left":

            msg = self.create_twist(
                angular_z=BLUEBOAT_LEFT
            )

        elif command == "turn_right":

            msg = self.create_twist(
                angular_z=BLUEBOAT_RIGHT
            )

        else:

            msg = self.create_twist()

        self.blueboat_publisher.publish(msg)

    # =========================================================================

    def publish_drone_command(
        self,
        command: str,
    ):

        if command == "increase_altitude":

            msg = self.create_twist(
                linear_z=DRONE_UP
            )

        elif command == "decrease_altitude":

            msg = self.create_twist(
                linear_z=DRONE_DOWN
            )

        elif command == "forward":

            msg = self.create_twist(
                linear_y=DRONE_FORWARD
            )

        elif command == "backward":

            msg = self.create_twist(
                linear_y=DRONE_BACKWARD
            )

        elif command == "yaw_right":

            msg = self.create_twist(
                angular_z=DRONE_RIGHT
            )

        elif command == "yaw_left":

            msg = self.create_twist(
                angular_z=DRONE_LEFT
            )

        else:

            msg = self.create_twist()

        self.drone_publisher.publish(msg)

    # =========================================================================

    def publish_rover_command(
        self,
        command: str,
    ):

        if command == "forward":

            msg = self.create_twist(
                linear_x=ROVER_FORWARD
            )

        elif command == "backward":

            msg = self.create_twist(
                linear_x=ROVER_BACKWARD
            )

        elif command == "turn_right":

            msg = self.create_twist(
                angular_z=ROVER_RIGHT
            )

        elif command == "turn_left":

            msg = self.create_twist(
                angular_z=ROVER_LEFT
            )

        else:

            msg = self.create_twist()

        self.rover_publisher.publish(msg)

    # =========================================================================
    # ROBOT STOP
    # =========================================================================

    def stop_all_robots(self):

        self.publish_blueboat_command("stop")

        self.publish_drone_command("stop")

        self.publish_rover_command("stop")

    # =========================================================================
    # CONTROL LOOP
    # =========================================================================

    def control_loop(self):

        # ---------------------------------------------------------------------
        # Wait until cameras are ready.
        # ---------------------------------------------------------------------

        if not self.recording_started:

            self.stop_all_robots()

            return

        # ---------------------------------------------------------------------
        # Pre-roll period.
        # ---------------------------------------------------------------------

        if not self.experiment_started:

            elapsed = (
                time.monotonic()
                - self.pre_roll_start_time
            )

            self.stop_all_robots()

            if elapsed >= PRE_ROLL_DURATION:

                self.experiment_started = True

                self.mission_start_time = time.monotonic()

                self.get_logger().info(
                    "=================================================="
                )

                self.get_logger().info(
                    "Starting all three robot missions."
                )

                self.get_logger().info(
                    "BlueBoat: "
                    "LEFT → FORWARD → RIGHT → FORWARD"
                )

                self.get_logger().info(
                    "Drone: "
                    "ELEVATE → FORWARD → YAW LEFT → FORWARD"
                )

                self.get_logger().info(
                    "Rover: "
                    "RIGHT → FORWARD → LEFT → FORWARD"
                )

                self.get_logger().info(
                    "=================================================="
                )

            return

        # ---------------------------------------------------------------------
        # Experiment already finished.
        # ---------------------------------------------------------------------

        if self.experiment_finished:

            self.stop_all_robots()

            return

        # ---------------------------------------------------------------------
        # Current mission time.
        # ---------------------------------------------------------------------

        elapsed = (
            time.monotonic()
            - self.mission_start_time
        )

        # ---------------------------------------------------------------------
        # Get current command for each robot.
        # ---------------------------------------------------------------------

        blueboat_command = (
            self.blueboat_mission.get_command(
                elapsed
            )
        )

        drone_command = (
            self.drone_mission.get_command(
                elapsed
            )
        )

        rover_command = (
            self.rover_mission.get_command(
                elapsed
            )
        )

        # ---------------------------------------------------------------------
        # Publish commands.
        # ---------------------------------------------------------------------

        self.publish_blueboat_command(
            blueboat_command
        )

        self.publish_drone_command(
            drone_command
        )

        self.publish_rover_command(
            rover_command
        )

        # ---------------------------------------------------------------------
        # Store commands for video status display.
        # ---------------------------------------------------------------------

        self.last_blueboat_command = (
            blueboat_command
        )

        self.last_drone_command = (
            drone_command
        )

        self.last_rover_command = (
            rover_command
        )

        # ---------------------------------------------------------------------
        # Check whether all missions are complete.
        # ---------------------------------------------------------------------

        if elapsed >= self.total_mission_duration:

            self.experiment_finished = True

            self.post_roll_start_time = (
                time.monotonic()
            )

            self.get_logger().info(
                "All robot missions completed."
            )

    # =========================================================================
    # VIDEO LOOP
    # =========================================================================

    def video_loop(self):

        # ---------------------------------------------------------------------
        # Start recording once all cameras are ready.
        # ---------------------------------------------------------------------

        if not self.recording_started:

            if self.all_cameras_ready():

                self.start_recording()

            return

        # ---------------------------------------------------------------------
        # Need all frames.
        # ---------------------------------------------------------------------

        if not self.all_cameras_ready():

            return

        # ---------------------------------------------------------------------
        # Prepare frames.
        # ---------------------------------------------------------------------

        blueboat = cv2.resize(
            self.blueboat_frame,
            (640, 360),
            interpolation=cv2.INTER_AREA,
        )

        drone = cv2.resize(
            self.drone_frame,
            (640, 360),
            interpolation=cv2.INTER_AREA,
        )

        rover = cv2.resize(
            self.rover_frame,
            (640, 360),
            interpolation=cv2.INTER_AREA,
        )

        # ---------------------------------------------------------------------
        # Add labels.
        # ---------------------------------------------------------------------

        cv2.putText(
            blueboat,
            "BlueBoat",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            drone,
            "Drone",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            rover,
            "Rover",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # ---------------------------------------------------------------------
        # Status panel.
        # ---------------------------------------------------------------------

        status = np.zeros(
            (360, 640, 3),
            dtype=np.uint8,
        )

        cv2.putText(
            status,
            "BlueSim",
            (30, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.4,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )

        cv2.putText(
            status,
            "Multi-Robot Experiment",
            (30, 95),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # ---------------------------------------------------------------------
        # Time information.
        # ---------------------------------------------------------------------

        if (
            self.experiment_started
            and not self.experiment_finished
        ):

            elapsed = (
                time.monotonic()
                - self.mission_start_time
            )

            phase = "RUNNING"

        elif not self.experiment_started:

            elapsed = 0.0

            phase = "PRE-ROLL"

        else:

            elapsed = (
                self.total_mission_duration
            )

            phase = "COMPLETED"

        cv2.putText(
            status,
            f"Time: {elapsed:.1f} s",
            (30, 145),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            status,
            f"Phase: {phase}",
            (30, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # ---------------------------------------------------------------------
        # Current commands.
        # ---------------------------------------------------------------------

        cv2.putText(
            status,
            f"BlueBoat: {self.last_blueboat_command}",
            (30, 225),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            status,
            f"Drone: {self.last_drone_command}",
            (30, 260),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            status,
            f"Rover: {self.last_rover_command}",
            (30, 295),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.putText(
            status,
            "ROS 2 / rclUE",
            (30, 335),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        # ---------------------------------------------------------------------
        # Build composite frame.
        # ---------------------------------------------------------------------

        top_row = np.hstack(
            (
                blueboat,
                drone,
            )
        )

        bottom_row = np.hstack(
            (
                rover,
                status,
            )
        )

        composite = np.vstack(
            (
                top_row,
                bottom_row,
            )
        )

        # ---------------------------------------------------------------------
        # Write frame.
        # ---------------------------------------------------------------------

        self.video_writer.write(
            composite
        )

        self.video_frame_count += 1

        # ---------------------------------------------------------------------
        # Finish after post-roll.
        # ---------------------------------------------------------------------

        if self.experiment_finished:

            post_roll_elapsed = (
                time.monotonic()
                - self.post_roll_start_time
            )

            if (
                post_roll_elapsed
                >= POST_ROLL_DURATION
            ):

                self.finish_experiment()

    # =========================================================================
    # FINISH EXPERIMENT
    # =========================================================================

    def finish_experiment(self):

        if self.experiment_finished is False:

            self.experiment_finished = True

        # ---------------------------------------------------------------------
        # Stop all robots.
        # ---------------------------------------------------------------------

        self.stop_all_robots()

        self.get_logger().info(
            "All robots stopped."
        )

        # ---------------------------------------------------------------------
        # Stop control timer.
        # ---------------------------------------------------------------------

        if self.control_timer is not None:

            self.control_timer.cancel()

        # ---------------------------------------------------------------------
        # Stop video timer.
        # ---------------------------------------------------------------------

        if self.video_timer is not None:

            self.video_timer.cancel()

        # ---------------------------------------------------------------------
        # Release video writer.
        # ---------------------------------------------------------------------

        if self.video_writer is not None:

            self.video_writer.release()

            self.video_writer = None

            self.get_logger().info(
                f"Video saved to:\n"
                f"{VIDEO_PATH}"
            )

            self.get_logger().info(
                f"Frames recorded: "
                f"{self.video_frame_count}"
            )

        else:

            self.get_logger().warning(
                "Video writer was not initialized."
            )

        self.get_logger().info(
            "Multi-robot experiment completed."
        )

        # ---------------------------------------------------------------------
        # Clean ROS shutdown.
        # ---------------------------------------------------------------------

        self.shutdown_timer = self.create_timer(
            0.1,
            self.shutdown_callback,
        )

    # =========================================================================
    # SHUTDOWN
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
        # Safety stop.
        # ---------------------------------------------------------------------

        try:

            self.stop_all_robots()

        except Exception:

            pass

        # ---------------------------------------------------------------------
        # Finalize video if interrupted.
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

    node = BlueSimMultiRobotExperiment()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Multi-robot experiment interrupted by user."
        )

    finally:

        node.destroy_node()


if __name__ == "__main__":

    main()