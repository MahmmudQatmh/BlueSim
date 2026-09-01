#!/usr/bin/env python3

"""
===============================================================================
BlueSim - Multi-Robot ROS 2 Experiment
===============================================================================

PURPOSE
-------
This script controls the BlueSim BlueBoat, Drone, and Rover simultaneously
through ROS 2 while recording their three onboard cameras into ONE video.

The experiment is synchronized with the Unreal Engine Level Sequencer.

IMPORTANT:
The Unreal cinematic sequence is configured to run for 800 seconds.

This script therefore:

    1. Starts the camera recording.
    2. Keeps ALL robots stopped.
    3. Waits for the 800-second Unreal cinematic sequence to finish.
    4. Starts all three robot missions simultaneously.
    5. Continues recording the three onboard cameras.
    6. Stops all robots after the missions are complete.
    7. Finalizes and saves the onboard-camera video.
    8. Terminates automatically.

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


===============================================================================
8. EXPERIMENT
===============================================================================

All three missions start simultaneously AFTER the Unreal Level Sequencer
has completed.

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


===============================================================================
9. LEVEL SEQUENCER SYNCHRONIZATION
===============================================================================

The Unreal Level Sequence is currently:

    Start = 0
    End   = 800 seconds

This script waits:

    800 seconds

before starting any robot movement.

During those 800 seconds:

    BlueBoat = STOP
    Drone    = STOP
    Rover    = STOP


Timeline:

    0 s
     │
     ▼
    Start camera recording
     │
     ▼
    Keep all robots stopped
     │
     │
     │    Unreal Level Sequencer
     │    is running
     │
     │
    800 s
     │
     ▼
    Sequencer completed
     │
     ▼
    Start all robot missions simultaneously
     │
     ▼
    Record onboard cameras
     │
     ▼
    Missions complete
     │
     ▼
    Stop all robots
     │
     ▼
    Save video
     │
     ▼
    Terminate


IMPORTANT:
The 800-second wait is a synchronization time between this ROS 2 node and
the Unreal Level Sequencer.

If the Level Sequence duration is changed later, update:

    SEQUENCER_DURATION_SECONDS

below.


===============================================================================
10. VIDEO RECORDING
===============================================================================

The Python node receives all three onboard camera streams through ROS 2.

The video layout is:

    ┌──────────────┬──────────────┐
    │   BlueBoat   │    Drone     │
    │    Camera    │    Camera    │
    ├──────────────┼──────────────┤
    │    Rover     │   Experiment │
    │    Camera    │    Status    │
    └──────────────┴──────────────┘


The onboard-camera video resolution is:

    1280 × 720


This is produced from four 640 × 360 panels.


===============================================================================
11. VIDEO OUTPUT
===============================================================================

Directory:

    BlueSim/ros2_ws/Recorded Videos/


Filename:

    BlueSim_MultiRobot_Record.mp4


===============================================================================
12. CONTROL RATE
===============================================================================

Robot command publication:

    20 Hz


Video:

    5 FPS


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


# =============================================================================
# LEVEL SEQUENCER SYNCHRONIZATION
# =============================================================================
#
# IMPORTANT:
#
# This MUST match the End time of LS_BlueSim_Showcase.
#
# Current Unreal Sequencer:
#
#     Start = 0
#     End   = 800
#
# Therefore:
#

SEQUENCER_DURATION_SECONDS = 27.0

#
# If you later change the Sequencer End time to 600:
#
#     SEQUENCER_DURATION_SECONDS = 600.0
#
# If you later change it to 900:
#
#     SEQUENCER_DURATION_SECONDS = 900.0
#
# =============================================================================


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

        # Time at which the onboard recording started.
        self.recording_start_time = None

        # Time at which the robot missions begin.
        self.mission_start_time = None

        # Time at which the robot missions complete.
        self.mission_finished_time = None

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
            f"Sequencer wait   : "
            f"{SEQUENCER_DURATION_SECONDS:.1f} seconds"
        )

        self.get_logger().info(
            f"Mission duration : "
            f"{self.total_mission_duration:.1f} seconds"
        )

        self.get_logger().info(
            "Waiting for valid frames from all "
            "three cameras..."
        )

    # =========================================================================
    # TWIST HELPER
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

                row_size = (
                    msg.width * channels
                )

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

                row_size = (
                    msg.width * channels
                )

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
        # All camera panels are normalized to 640x360.
        # =====================================================================

        target_width = 640
        target_height = 360

        # =====================================================================
        # Composite:
        #
        # BlueBoat | Drone
        # Rover    | Status
        #
        # Final = 1280x720
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

        self.recording_start_time = time.monotonic()

        self.get_logger().info(
            "Onboard camera recording started."
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
            "All robots will remain STOPPED "
            "until the Unreal Sequencer completes."
        )

        self.get_logger().info(
            f"Waiting {SEQUENCER_DURATION_SECONDS:.1f} "
            f"seconds for the Level Sequencer..."
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
    # STOP ALL ROBOTS
    # =========================================================================

    def stop_all_robots(self):

        self.publish_blueboat_command("stop")

        self.publish_drone_command("stop")

        self.publish_rover_command("stop")

        self.last_blueboat_command = "STOP"
        self.last_drone_command = "STOP"
        self.last_rover_command = "STOP"

    # =========================================================================
    # CONTROL LOOP
    # =========================================================================

    def control_loop(self):

        # ---------------------------------------------------------------------
        # Cameras/video not ready yet.
        # ---------------------------------------------------------------------

        if not self.recording_started:

            self.stop_all_robots()

            return

        # ---------------------------------------------------------------------
        # IMPORTANT:
        #
        # The camera recording has started.
        # The robots remain stopped for the COMPLETE Level Sequencer period.
        # ---------------------------------------------------------------------

        if not self.experiment_started:

            elapsed = (
                time.monotonic()
                - self.recording_start_time
            )

            self.stop_all_robots()

            remaining = (
                SEQUENCER_DURATION_SECONDS
                - elapsed
            )

            # -------------------------------------------------------------
            # Print a useful status message approximately every 10 seconds.
            # -------------------------------------------------------------

            if (
                remaining > 0.0
                and (
                    int(elapsed) % 10 == 0
                )
            ):

                # Avoid flooding the terminal by only logging when the
                # integer second changes.
                if not hasattr(
                    self,
                    "_last_wait_log_second",
                ):

                    self._last_wait_log_second = -1

                current_second = int(elapsed)

                if (
                    current_second
                    != self._last_wait_log_second
                    and current_second % 10 == 0
                ):

                    self._last_wait_log_second = (
                        current_second
                    )

                    self.get_logger().info(
                        f"Waiting for Sequencer: "
                        f"{remaining:.0f} s remaining."
                    )

            # -------------------------------------------------------------
            # Sequencer completed.
            # -------------------------------------------------------------

            if elapsed >= SEQUENCER_DURATION_SECONDS:

                self.experiment_started = True

                self.mission_start_time = (
                    time.monotonic()
                )

                self.get_logger().info(
                    "=================================================="
                )

                self.get_logger().info(
                    "UNREAL SEQUENCER COMPLETED."
                )

                self.get_logger().info(
                    "Starting all three robot missions "
                    "SIMULTANEOUSLY."
                )

                self.get_logger().info(
                    "BlueBoat: "
                    "LEFT → FORWARD → RIGHT → FORWARD"
                )

                self.get_logger().info(
                    "Drone: "
                    "ELEVATE → FORWARD → "
                    "YAW LEFT → FORWARD"
                )

                self.get_logger().info(
                    "Rover: "
                    "RIGHT → FORWARD → "
                    "LEFT → FORWARD"
                )

                self.get_logger().info(
                    "=================================================="
                )

            return

        # ---------------------------------------------------------------------
        # Mission finished.
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
        # Current command for each robot.
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
        #
        # Because all three are evaluated from the SAME elapsed time,
        # they start together.
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
        # Save command names for the video status panel.
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
        # Check mission completion.
        # ---------------------------------------------------------------------

        if elapsed >= self.total_mission_duration:

            self.experiment_finished = True

            self.mission_finished_time = (
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
        # Start recording as soon as all cameras are available.
        # ---------------------------------------------------------------------

        if not self.recording_started:

            if self.all_cameras_ready():

                self.start_recording()

            return

        # ---------------------------------------------------------------------
        # All three camera frames are required for the composite video.
        # ---------------------------------------------------------------------

        if not self.all_cameras_ready():

            return

        # ---------------------------------------------------------------------
        # Resize cameras to 640x360.
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
        # Camera labels.
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
        # Status / time.
        # ---------------------------------------------------------------------

        if not self.experiment_started:

            elapsed = 0.0

            if self.recording_start_time is not None:

                elapsed = (
                    time.monotonic()
                    - self.recording_start_time
                )

            remaining = max(
                0.0,
                SEQUENCER_DURATION_SECONDS
                - elapsed,
            )

            phase = "SEQUENCER"

            cv2.putText(
                status,
                f"Sequencer: {remaining:.0f}s",
                (30, 145),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        elif (
            self.experiment_started
            and not self.experiment_finished
        ):

            elapsed = (
                time.monotonic()
                - self.mission_start_time
            )

            phase = "RUNNING"

            cv2.putText(
                status,
                f"Mission: {elapsed:.1f}s",
                (30, 145),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        else:

            phase = "COMPLETED"

            cv2.putText(
                status,
                "Mission completed",
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
        # Build 2x2 video.
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
        # Write video frame.
        # ---------------------------------------------------------------------

        if self.video_writer is not None:

            self.video_writer.write(
                composite
            )

            self.video_frame_count += 1

    # =========================================================================
    # FINISH EXPERIMENT
    # =========================================================================

    def finish_experiment(self):

        # Prevent repeated finalization.
        if self.shutdown_requested:

            return

        # ---------------------------------------------------------------------
        # Stop all robots FIRST.
        # ---------------------------------------------------------------------

        self.stop_all_robots()

        self.get_logger().info(
            "All robots stopped."
        )

        # ---------------------------------------------------------------------
        # Cancel control timer.
        # ---------------------------------------------------------------------

        if self.control_timer is not None:

            self.control_timer.cancel()

        # ---------------------------------------------------------------------
        # Cancel video timer.
        # ---------------------------------------------------------------------

        if self.video_timer is not None:

            self.video_timer.cancel()

        # ---------------------------------------------------------------------
        # Release video.
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
        # Request clean ROS shutdown.
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
        # Always attempt to stop all robots.
        # ---------------------------------------------------------------------

        try:

            self.stop_all_robots()

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
