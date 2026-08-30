# BlueSim ROS 2 Interface

ROS 2 interface and Python utilities for the **BlueSim** Unreal Engine simulator.

BlueSim integrates **Unreal Engine 5.4.4** with **ROS 2 Humble** through the **rclUE** plugin. The integration allows simulated robots to:

- Publish sensor data to ROS 2.
- Receive ROS 2 commands.
- Be controlled from Python ROS 2 nodes.
- Stream simulated camera images.
- Record camera data.
- Run external perception, planning, control, and autonomy algorithms.
- Provide a foundation for future multi-robot experiments.

The current simulator contains three robots:

- **BlueBoat**
- **Drone**
- **Rover**

---

# 1. Overview

The BlueSim ROS 2 architecture is divided into three major layers:

```text
┌─────────────────────────────────────────────────────────────┐
│                     BLUE SIM                                │
│                 Unreal Engine 5.4.4                         │
│                                                             │
│   ┌───────────┐      ┌──────────────┐      ┌───────────┐    │
│   │ BlueBoat  │      │    Drone     │      │   Rover   │    │
│   └─────┬─────┘      └──────┬───────┘      └─────┬─────┘    │
│         │                   │                    │          │
│         └───────────────────┼────────────────────┘          │
│                             │                               │
│                           rclUE                             │
│                             │                               │
└─────────────────────────────┼───────────────────────────────┘
                              │
                         ROS 2 / DDS
                              │
┌─────────────────────────────┼──────────────────────────────┐
│                       ROS 2 HUMBLE                         │
│                           ros2_ws                          │
│                             │                              │
│                           rclpy                            │
│                             │                              │
│              ┌──────────────┴──────────────┐               │
│              │                             │               │
│        Sensor Processing             Robot Control         │
│              │                             │               │
│       OpenCV / NumPy / ML           Twist / Autonomy       │
│              │                             │               │
│              └──────────────┬──────────────┘               │
│                             │                              │
└─────────────────────────────┼──────────────────────────────┘
                              │
                       ROS 2 commands/data
                              │
                              ▼
                       BlueSim robots
```

The fundamental concept is:

> **Unreal Engine performs simulation, physics, rendering, and sensor generation. rclUE provides the ROS 2 interface inside Unreal. ROS 2 provides communication. Python provides perception, control, autonomy, and experimentation.**

---

# 2. High-Level Data Flow

There are two main communication directions.

## 2.1 Sensor data: Unreal → ROS 2 → Python

```text
Unreal Robot
     │
     ▼
Camera / Sensor
     │
     ▼
rclUE Publisher
     │
     ▼
ROS 2 Topic
     │
     ▼
ROS 2 Humble
     │
     ▼
Python rclpy Subscriber
     │
     ├──────────────► OpenCV
     │
     ├──────────────► NumPy
     │
     ├──────────────► Machine Learning
     │
     └──────────────► Data Recording
```

---

## 2.2 Commands: Python → ROS 2 → Unreal

```text
Python Autonomy / Control
            │
            ▼
       rclpy Publisher
            │
            ▼
     geometry_msgs/Twist
            │
            ▼
        ROS 2 / DDS
            │
            ▼
       rclUE Subscriber
            │
            ▼
      Unreal Blueprint
            │
            ▼
      Robot Command Variables
            │
            ▼
     Robot Physics / Control
            │
            ▼
       Simulated Robot
```

---

# 3. rclUE

## 3.1 What is rclUE?

`rclUE` is the ROS 2 integration plugin used inside Unreal Engine.

It provides Unreal Engine wrappers around ROS 2 functionality, including:

- ROS 2 nodes
- Publishers
- Subscribers
- ROS 2 message wrappers
- Services
- Actions
- QoS configuration

Relevant rclUE classes used by BlueSim include:

```text
UROS2NodeComponent
UROS2Publisher
UROS2Subscriber
UROS2GenericMsg
UROS2ImgMsg
UROS2TwistMsg
```

The basic relationship is:

```text
                    UROS2NodeComponent
                           │
             ┌─────────────┴──────────────┐
             │                            │
             ▼                            ▼
      UROS2Publisher                UROS2Subscriber
             │                            │
             ▼                            ▼
      UROS2GenericMsg              UROS2GenericMsg
             │                            │
       ┌─────┴─────┐                ┌─────┴─────┐
       │           │                │           │
       ▼           ▼                ▼           ▼
 UROS2ImgMsg   Other Msgs      UROS2TwistMsg   Other Msgs
```

---

# 4. ROS 2 Node in Unreal

Each ROS-enabled robot has an `UROS2NodeComponent`.

The Blueprint contains a ROS 2 node component, for example:

```text
ROS2Node
```

The node is initialized during `BeginPlay`.

Conceptually:

```text
Event BeginPlay
       │
       ▼
ROS2Node
       │
       ▼
Init
       │
       ▼
rclc_node_init_default()
       │
       ▼
ROS 2 Node Initialized
```

The node provides the foundation for the publishers and subscribers attached to the robot.

---

# 5. UROS2NodeComponent

The main Unreal-side ROS 2 node implementation is:

```text
BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Public/ROS2NodeComponent.h
BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Private/ROS2NodeComponent.cpp
```

The node component is responsible for:

- Initializing the ROS 2 node.
- Maintaining the ROS 2 context.
- Registering publishers.
- Registering subscribers.
- Maintaining ROS 2 communication entities.
- Processing incoming subscriptions.

The initialization internally uses the ROS 2 C/rclc APIs.

For example, the implementation eventually performs:

```cpp
rclc_node_init_default(
    &node,
    node_name,
    node_namespace,
    &Support->Get()
);
```

---

# 6. ROS 2 Publisher Architecture

A publisher is created through the Unreal Blueprint and rclUE.

General architecture:

```text
Blueprint
    │
    ▼
ROS2Node
    │
    ▼
Add Publisher
    │
    ▼
UROS2Publisher
    │
    ▼
rcl_publisher_init()
    │
    ▼
ROS 2 Middleware
    │
    ▼
ROS 2 Topic
```

The publisher requires:

- Topic name
- Publisher class
- ROS message class
- QoS
- Publication frequency

For example:

```text
Topic Name:
camera_Rover

Publisher Class:
BlueSimCameraPublisher

Msg Class:
ROS2ImgMsg

Publication Frequency:
20 Hz

QoS:
Default
```

---

# 7. UROS2Publisher

The main rclUE publisher implementation is:

```text
BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Public/ROS2Publisher.h
BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Private/ROS2Publisher.cpp
```

`UROS2Publisher` provides the generic ROS 2 publishing functionality.

Important properties include:

```text
PublicationFrequencyHz
UpdateDelegate
TopicName
MsgClass
QoS
```

The generic execution flow is:

```text
Timer
  │
  ▼
UpdateAndPublish()
  │
  ▼
UpdateDelegate
  │
  ▼
UpdateMessage()
  │
  ▼
Publish()
  │
  ▼
rcl_publish()
  │
  ▼
ROS 2
```

The publisher internally creates an Unreal timer based on:

```text
Period = 1 / PublicationFrequencyHz
```

For example:

```text
1 Hz  → approximately once every second

20 Hz → approximately every 50 ms
```

---

# 8. ROS 2 Message Wrappers

rclUE provides wrappers around ROS 2 message types.

For camera images:

```text
UROS2ImgMsg
       │
       ▼
sensor_msgs/msg/Image
```

For robot commands:

```text
UROS2TwistMsg
       │
       ▼
geometry_msgs/msg/Twist
```

These are wrappers around the actual ROS 2 C message structures.

For example, the image wrapper internally contains:

```cpp
sensor_msgs__msg__Image image_msg;
```

and the Twist wrapper contains:

```cpp
geometry_msgs__msg__Twist twist_msg;
```

Therefore the data eventually sent through ROS 2 is a real ROS 2 message, not a custom BlueSim-only format.

---

# 9. Camera Architecture

Each robot has a camera setup similar to:

```text
Robot
 │
 └── Camera
       │
       └── SceneCaptureComponent2D
                    │
                    ▼
                Render Target
```

Examples:

```text
BlueBoat
 └── BoatCamera
       └── BoatCameraCapture
              └── RT_BoatCamera

Drone
 └── DroneCamera
       └── DroneCameraCapture
              └── RT_DroneCamera

Rover
 └── RoverCamera
       └── RoverCameraCapture
              └── RT_RoverCamera
```

The camera uses:

```text
Projection Type = Perspective
Field of View   = 90°
Resolution      = 640 × 360
```

The Scene Capture Components use their corresponding render targets.

---

# 10. Camera Pipeline

The complete camera pipeline is:

```text
┌──────────────────────────────┐
│ SceneCaptureComponent2D      │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ Unreal Render Target         │
│ RT_*Camera                   │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ BlueSimCameraPublisher       │
│                              │
│ CaptureScene()               │
│ ReadPixels()                 │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ Unreal FColor Pixel Array    │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ FROSImg                      │
│                              │
│ Header                       │
│ Height                       │
│ Width                        │
│ Encoding                     │
│ Step                         │
│ Data                         │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ UROS2ImgMsg                  │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ sensor_msgs/msg/Image        │
└──────────────┬───────────────┘
               │
               ▼
             ROS 2
```

---

# 11. BlueSimCameraPublisher

The standard rclUE publisher knows how to publish a ROS 2 message, but it does not know how to obtain pixels from an Unreal `SceneCaptureComponent2D`.

Therefore BlueSim adds a custom publisher:

```text
BlueSim/unreal/BlueSim/Source/BlueSim/BlueSimCameraPublisher.h
BlueSim/unreal/BlueSim/Source/BlueSim/BlueSimCameraPublisher.cpp
```

The class extends:

```cpp
UROS2Publisher
```

Conceptually:

```text
             UROS2Publisher
                    ▲
                    │
                    │ inherits
                    │
       BlueSimCameraPublisher
```

This allows the custom publisher to reuse all the standard rclUE ROS 2 publisher functionality while adding Unreal camera functionality.

---

# 12. BlueSimCameraPublisher.h

The header defines the custom camera publisher.

The important concept is that the publisher receives Unreal references to:

```text
Render Target
Capture Component
Flip Vertical
```

This makes the publisher reusable.

For example:

```text
BlueBoat

Render Target:
RT_BoatCamera

Capture Component:
BoatCameraCapture
```

```text
Drone

Render Target:
RT_DroneCamera

Capture Component:
DroneCameraCapture
```

```text
Rover

Render Target:
RT_RoverCamera

Capture Component:
RoverCameraCapture
```

The class overrides the rclUE publisher callback responsible for updating the outgoing message.

Conceptually:

```cpp
virtual void UpdateMessage(
    UROS2GenericMsg* InMessage
) override;
```

---

# 13. BlueSimCameraPublisher.cpp

The implementation performs the conversion from Unreal rendering data to a ROS 2 image.

The process is:

```text
Scene Capture
      │
      ▼
Render Target
      │
      ▼
FTextureRenderTargetResource
      │
      ▼
ReadPixels()
      │
      ▼
TArray<FColor>
      │
      ▼
BGR pixel data
      │
      ▼
FROSImg
      │
      ▼
UROS2ImgMsg
      │
      ▼
rcl_publish()
```

The render-target resource is obtained from the Unreal render target:

```cpp
FTextureRenderTargetResource* RenderTargetResource =
    RenderTarget->GameThread_GetRenderTargetResource();
```

Pixels are then obtained using:

```cpp
TArray<FColor> Pixels;

FReadSurfaceDataFlags ReadFlags(RCM_UNorm);

RenderTargetResource->ReadPixels(
    Pixels,
    ReadFlags
);
```

The pixel information is then copied into the ROS image structure.

---

# 14. Camera Message Construction

The camera publisher produces an image message containing:

```text
Header
Height
Width
Encoding
IsBigendian
Step
Data
```

A typical BlueSim camera frame is:

```yaml
height: 360
width: 640
encoding: bgr8
step: 1920
```

The image data is stored in:

```text
data
```

The `data` field contains the actual camera pixels.

The current camera frame ID is configured per camera, for example:

```text
boat_camera
```

---

# 15. Why `bgr8` is Used

The camera data is published using:

```text
encoding = bgr8
```

This is convenient for OpenCV because OpenCV commonly works with BGR channel ordering.

The Python pipeline therefore becomes:

```text
ROS Image
    │
    ▼
msg.data
    │
    ▼
NumPy array
    │
    ▼
Height × Width × 3
    │
    ▼
OpenCV
```

For the current camera resolution:

```text
640 × 360 × 3
```

---

# 16. Camera Frame Validation

The camera message should contain real image data.

A valid message looks conceptually like:

```text
width     = 640
height    = 360
encoding  = bgr8
step      = 1920
data      = real pixel bytes
```

A message containing:

```text
width     = 0
height    = 0
encoding  = ""
step      = 0
data      = []
```

represents an empty camera message and cannot be used for image processing.

The Python camera nodes therefore validate the incoming image before processing it.

---

# 17. ROS 2 Subscriber Architecture

The subscriber performs the reverse communication path.

General architecture:

```text
ROS 2 Topic
     │
     ▼
rclUE Subscriber
     │
     ▼
UROS2Subscriber
     │
     ▼
UROS2GenericMsg
     │
     ▼
Specific ROS message wrapper
     │
     ▼
Blueprint Callback
     │
     ▼
Robot Control Variables
     │
     ▼
Robot Physics
```

---

# 18. Twist Subscriber

For robot movement, the standard ROS 2 message used is:

```text
geometry_msgs/msg/Twist
```

The rclUE wrapper is:

```text
ROS2TwistMsg
```

The subscriber is configured with:

```text
Topic Name:
<robot command topic>

Msg Class:
ROS2TwistMsg

QoS:
Default
```

---

# 19. Blueprint Subscriber Callback

The Blueprint callback follows this structure:

```text
    OnCmdVelReceived
            │
            ▼
    Cast To ROS2TwistMsg
            │
            ▼
         Get Msg
            │
            ▼
      Break ROSTwist
       /          \
      /            \
  Linear          Angular
      │              │
Break Vector      Break Vector
      │              │
      X              Z
      │              │
      ▼              ▼
Drive Command    Steering Command
```

The important ROS convention used by the robot control interface is:

```text
linear.x
    │
    └── forward/backward command

angular.z
    │
    └── steering/yaw command
```

---

# 20. Why Commands Are Stored in Variables

The ROS callback does not need to directly perform the entire robot physics operation.

Instead:

```text
ROS message
    │
    ▼
Command Variable
    │
    ▼
Physics Update
```

This separates:

- input source
- robot control
- physics

The same robot can therefore use:

```text
Keyboard
```

or:

```text
ROS 2
```

without duplicating the physics system.

---

# 21. Control Architecture

The preferred architecture is:

```text
               ┌─────────────────┐
               │    Keyboard     │
               └────────┬────────┘
                        │
                        ▼
                 Command Variables
                        ▲
                        │
               ┌────────┴────────┐
               │                 │
               │      ROS 2      │
               │                 │
               └─────────────────┘
                        │
                        ▼
                  Event Tick
                        │
                        ▼
               Robot Physics
                        │
                        ▼
                    Robot
```

This separation is particularly important for the Rover.

---

# 22. BlueBoat Control

The BlueBoat receives a `Twist` command.

Conceptually:

```text
ROS 2
 │
 │ /cmd_BlueBoat_vel
 ▼
ROS2TwistMsg
 │
 ▼
Break ROSTwist
 │
 ├── Linear.X
 │       │
 │       ▼
 │   BoatThrottle
 │
 └── Angular.Z
         │
         ▼
     BoatSteering
```

The internal commands are then used by the BlueBoat movement system.

The movement architecture uses:

```text
BoatThrottle
     │
     ▼
Add Force

BoatSteering
     │
     ▼
Add Torque
```

---

# 23. Rover Control

The Rover receives:

```text
/cmd_rover_vel
```

with:

```text
geometry_msgs/msg/Twist
```

The command mapping is:

```text
ROS linear.x > 0
        │
        ▼
     Forward

ROS linear.x < 0
        │
        ▼
    Backward
```

For steering:

```text
ROS angular.z
       │
       ▼
Sign conversion
       │
       ▼
Rover Steering Command
```

This conversion is necessary because the Rover's internal Unreal steering convention is different from the ROS angular convention.

---

# 24. Rover Physics Architecture

The preferred Rover architecture is:

```text
Keyboard / ROS
       │
       ▼
Command Variables
       │
       ▼
Event Tick
       │
       ▼
┌──────┴────────────────────────┐
│                               │
▼                               ▼
Drive Force                Steering Torque
│                               │
▼                               ▼
Add Force                  Add Torque
│                               │
└──────────────┬────────────────┘
               ▼
          RoverPhysics
               │
               ▼
             Rover
```

A lateral friction model can additionally apply a force opposing sideways velocity:

```text
Rover Physics Velocity
       │
       ▼
World → Local Velocity
       │
       ▼
Local Lateral Velocity
       │
       ▼
Opposing Force
       │
       ▼
Add Force
       │
       ▼
RoverPhysics
```

---

# 25. Event Tick and Command Variables

A key principle of the final control architecture is:

> **Inputs change command variables. Event Tick applies physics.**

For example:

```text
Keyboard Forward
       │
       ▼
RoverDriveCommand = +1
```

or:

```text
ROS linear.x = +1
       │
       ▼
RoverDriveCommand = +1
```

The physics layer then reads that value:

```text
Event Tick
     │
     ▼
RoverDriveCommand
     │
     ▼
Drive Force
     │
     ▼
Add Force
```

The physics layer therefore does not care whether the command came from a keyboard or ROS 2.

---

# 26. Python ROS 2 Workspace

The Python ROS 2 workspace is:

```text
BlueSim/ros2_ws
```

The ROS 2 package is:

```text
BlueSim/ros2_ws/src/bluesim_ros2
```

The package contains Python ROS 2 utilities for BlueSim.

Current structure:

```text
BlueSim/
└── ros2_ws/
    └── src/
        └── bluesim_ros2/
            ├── package.xml
            ├── CMakeLists.txt
            ├── README.md
            │
            └── bluesim_ros2/
                ├── camera_monitor.py
                ├── boat_command.py
                ├── blueboat_autonomy.py
                ├── blueboat_experiment.py
                └── rover_command.py
```

---

# 27. ROS 2 Python Dependencies

The package uses ROS 2 Humble and Python 3.

Important Python ROS 2 dependencies include:

```text
rclpy
sensor_msgs
geometry_msgs
```

Additional processing is performed using:

```text
NumPy
OpenCV
```

---

# 28. Building the ROS 2 Workspace

Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

Build:

```bash
cd BlueSim/ros2_ws

colcon build --symlink-install
```

Source the workspace:

```bash
source install/setup.bash
```

Verify the package:

```bash
ros2 pkg list | grep bluesim
```

Expected:

```text
bluesim_ros2
```

---

# 29. ROS 2 Environment

The current development environment uses:

```text
ROS 2 Distribution : Humble
Python             : 3.10
ROS Version        : 2
```

Check:

```bash
echo "ROS_DISTRO=$ROS_DISTRO"
```

```bash
which ros2
```

```bash
python3 --version
```

Check `rclpy`:

```bash
python3 -c "import rclpy; print('rclpy OK:', rclpy.__file__)"
```

---

# 30. Current ROS 2 Topics

The simulator currently exposes command and camera topics for the three robots.

Current topics:

```text
/camera_BlueBoat
/camera_Drone
/camera_Rover

/cmd_BlueBoat_vel
/cmd_drone_vel
/cmd_rover_vel
```

Camera messages use:

```text
sensor_msgs/msg/Image
```

Command messages use:

```text
geometry_msgs/msg/Twist
```

The exact topic names are configured in the Unreal Blueprints and Python ROS 2 nodes.

---

# 31. Inspecting ROS 2 Topics

List topics:

```bash
ros2 topic list
```

List topics together with message types:

```bash
ros2 topic list -t
```

Inspect a topic:

```bash
ros2 topic info /cmd_rover_vel -v
```

Inspect a camera topic:

```bash
ros2 topic info /camera_Rover -v
```

---

# 32. Measuring Camera Frequency

Camera publication frequency can be checked with:

```bash
ros2 topic hz /camera_Rover
```

Example:

```text
average rate: ...
min: ...
max: ...
std dev: ...
```

The measured frequency is the actual publication frequency seen by ROS 2.

This is useful because the Unreal publisher frequency is a target/requested frequency, while the actual frequency can be affected by rendering and image readback performance.

---

# 33. Inspecting Camera Image Metadata

Width:

```bash
ros2 topic echo /camera_Rover --once --field width
```

Height:

```bash
ros2 topic echo /camera_Rover --once --field height
```

Encoding:

```bash
ros2 topic echo /camera_Rover --once --field encoding
```

Step:

```bash
ros2 topic echo /camera_Rover --once --field step
```

Header:

```bash
ros2 topic echo /camera_Rover --once --field header
```

The image data itself can be inspected with:

```bash
ros2 topic echo /camera_Rover --once --field data
```

For large image messages this produces a very large terminal output, so inspecting individual metadata fields is usually preferable.

---

# 34. Python Camera Subscriber

A Python camera node uses:

```python
import rclpy
from sensor_msgs.msg import Image
```

Subscription:

```python
self.create_subscription(
    Image,
    '/camera_Rover',
    self.camera_callback,
    10
)
```

The callback receives:

```python
def camera_callback(self, msg: Image):
    ...
```

The most important fields are:

```python
msg.width
msg.height
msg.encoding
msg.step
msg.data
```

---

# 35. Converting ROS Image to NumPy

For:

```text
encoding = bgr8
```

the image data can be converted into a NumPy array:

```text
ROS Image
    │
    ▼
msg.data
    │
    ▼
np.frombuffer(...)
    │
    ▼
height × width × 3
    │
    ▼
OpenCV image
```

For the current resolution:

```text
640 × 360 × 3
```

---

# 36. Camera Video Recording

BlueSim supports recording the simulated camera through Python.

The recording architecture is:

```text
Unreal Camera
      │
      ▼
rclUE Camera Publisher
      │
      ▼
/camera_*
      │
      ▼
Python ROS 2 Subscriber
      │
      ▼
sensor_msgs/msg/Image
      │
      ▼
NumPy
      │
      ▼
OpenCV
      │
      ▼
cv2.VideoWriter
      │
      ▼
MP4
```

The recording system can wait for the first valid camera frame before starting robot movement.

This provides:

```text
Start experiment
      │
      ▼
Wait for valid camera frame
      │
      ▼
Start video recording
      │
      ▼
Start robot movement
      │
      ▼
Mission completed
      │
      ▼
Stop robot
      │
      ▼
Finalize video
      │
      ▼
Terminate Python node
```

---

# 37. Example BlueBoat Experiment

An experiment can combine:

- robot control
- camera recording
- state-machine based movement

Example:

```text
              Start
                │
                ▼
        Wait for camera
                │
                ▼
        Start recording
                │
                ▼
           Turn Left
                │
                ▼
            Forward
                │
                ▼
          Turn Right
                │
                ▼
            Forward
                │
                ▼
              Stop
                │
                ▼
        Save camera video
                │
                ▼
             Finish
```

---

# 38. Example Rover Experiment

A Rover experiment can similarly use:

```text
Start
  │
  ▼
Wait for camera
  │
  ▼
Start recording
  │
  ▼
Forward
  │
  ▼
Turn Right
  │
  ▼
Forward
  │
  ▼
Stop
  │
  ▼
Save video
  │
  ▼
Terminate
```

---

# 39. ROS 2 Command Example

A Rover forward command can be published manually:

```bash
ros2 topic pub --rate 10 /cmd_rover_vel geometry_msgs/msg/Twist \
"{linear: {x: 1.0}, angular: {z: 0.0}}"
```

This corresponds to:

```text
linear.x = +1
angular.z = 0
```

and therefore requests forward motion.

---

# 40. Rover Right Turn Example

The current Rover steering convention requires:

```text
angular.z < 0
```

for a right turn.

Example:

```bash
ros2 topic pub --rate 10 /cmd_rover_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: -1.0}}"
```

---

# 41. Stop Command

A zero command can be published with:

```bash
ros2 topic pub --rate 10 /cmd_rover_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

This produces:

```text
Drive command = 0
Steering command = 0
```

---

# 42. Python Robot Control

Python autonomy nodes use:

```python
from geometry_msgs.msg import Twist
```

and publish:

```python
msg = Twist()

msg.linear.x = 1.0
msg.angular.z = 0.0

publisher.publish(msg)
```

The overall flow is:

```text
Python Algorithm
      │
      ▼
Twist Message
      │
      ▼
ROS 2 Publisher
      │
      ▼
Robot Command Topic
      │
      ▼
rclUE Subscriber
      │
      ▼
Blueprint
      │
      ▼
Robot Physics
```

---

# 43. Manual Control and ROS Control

The BlueSim robots are designed so that manual keyboard control and ROS control can coexist.

The preferred architecture is:

```text
                         ┌───────────────┐
                         │   Keyboard    │
                         └───────┬───────┘
                                 │
                                 ▼
                       Keyboard Commands
                                 │
                                 │
                                 ▼
                              Select
                                 ▲
                                 │
                                 │
                         ROS Commands
                                 ▲
                                 │
                         ┌───────┴───────┐
                         │     ROS 2     │
                         └───────────────┘
                                 │
                                 ▼
                           Event Tick
                                 │
                                 ▼
                         Robot Physics
                                 │
                                 ▼
                              Robot
```

A control-state variable can be used to select which command source currently controls the robot.

---

# 44. Why This Architecture Matters

The ROS 2 integration intentionally separates:

## Simulation

```text
Unreal Engine
```

from:

## Communication

```text
rclUE
ROS 2
DDS
```

from:

## Algorithms

```text
Python
OpenCV
NumPy
Machine Learning
Planning
Control
```

This means an external algorithm does not need to know that the robot is implemented in Unreal Engine.

For example:

```text
Camera
   │
   ▼
/camera_Rover
   │
   ▼
Python
   │
   ▼
YOLO / OpenCV / ML
   │
   ▼
Decision
   │
   ▼
/cmd_rover_vel
   │
   ▼
Rover
```

---

# 45. BlueSim as a Robotics Research Platform

The ROS 2 interface makes BlueSim suitable for:

- Computer vision
- Object detection
- Object tracking
- Visual navigation
- Autonomous control
- Sensor fusion
- Path planning
- Reinforcement learning
- Multi-robot systems
- Cooperative control
- Search and rescue experiments
- Environment monitoring

The important design goal is that the same algorithms can eventually be used with:

```text
Simulation
```

and:

```text
Real Robot
```

provided that the ROS 2 interfaces are kept compatible.

---

# 46. Current Project Structure

Relevant project structure:

```text
BlueSim/
│
├── unreal/
│   └── BlueSim/
│       │
│       ├── Source/
│       │   └── BlueSim/
│       │       ├── BlueSim.Build.cs
│       │       ├── BlueSimCameraPublisher.h
│       │       └── BlueSimCameraPublisher.cpp
│       │
│       ├── Content/
│       │   ├── BlueBoat/
│       │   ├── Drone/
│       │   ├── Rover/
│       │   └── Sensors/
│       │
│       └── Plugins/
│           └── rclUE/
│
└── ros2_ws/
    │
    ├── build/
    ├── install/
    ├── log/
    │
    └── src/
        │
        └── bluesim_ros2/
            ├── package.xml
            ├── CMakeLists.txt
            ├── README.md
            │
            └── bluesim_ros2/
                ├── camera_monitor.py
                ├── boat_command.py
                ├── blueboat_autonomy.py
                ├── blueboat_experiment.py
                └── rover_command.py
```

---

# 47. Important Unreal Files

The standard rclUE ROS publisher implementation:

```text
BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Public/ROS2Publisher.h

BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Private/ROS2Publisher.cpp
```

The standard rclUE ROS node:

```text
BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Public/ROS2NodeComponent.h

BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Private/ROS2NodeComponent.cpp
```

The image message wrapper:

```text
BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Public/Msgs/ROS2Img.h

BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Private/Msgs/ROS2Img.cpp
```

The Twist message wrapper:

```text
BlueSim/unreal/BlueSim/Plugins/rclUE/Source/rclUE/Public/Msgs/ROS2Twist.h
```

The BlueSim-specific camera publisher:

```text
BlueSim/unreal/BlueSim/Source/BlueSim/BlueSimCameraPublisher.h

BlueSim/unreal/BlueSim/Source/BlueSim/BlueSimCameraPublisher.cpp
```

---

# 48. Important rclUE Classes

The most relevant classes are:

| Class | Purpose |
|---|---|
| `UROS2NodeComponent` | ROS 2 node and communication management |
| `UROS2Publisher` | Generic ROS 2 publisher |
| `UROS2Subscriber` | Generic ROS 2 subscriber |
| `UROS2GenericMsg` | Base class for ROS 2 message wrappers |
| `UROS2ImgMsg` | Wrapper for `sensor_msgs/msg/Image` |
| `UROS2TwistMsg` | Wrapper for `geometry_msgs/msg/Twist` |
| `BlueSimCameraPublisher` | BlueSim-specific Unreal camera publisher |

---

# 49. Publisher Flow in Detail

```text
              Unreal Blueprint
                     │
                     ▼
              ROS2PublisherTest
                     │
                     │ configuration
                     │
        ┌────────────┼─────────────┐
        │            │             │
        ▼            ▼             ▼
   Topic Name     Msg Class       QoS
        │            │             │
        └────────────┼─────────────┘
                     │
                     ▼
                 rclUE Node
                     │
                     ▼
              UROS2Publisher
                     │
                     ▼
            rcl_publisher_init()
                     │
                     ▼
                ROS 2 DDS
```

---

# 50. Camera Publisher Flow in Detail

```text
SceneCaptureComponent2D
             │
             ▼
      RT_BoatCamera /
      RT_DroneCamera /
      RT_RoverCamera
             │
             ▼
   BlueSimCameraPublisher
             │
             ├── CaptureScene()
             │
             ├── ReadPixels()
             │
             ▼
        TArray<FColor>
             │
             ▼
           FROSImg
             │
             ▼
        UROS2ImgMsg
             │
             ▼
  sensor_msgs/msg/Image
             │
             ▼
       rcl_publish()
             │
             ▼
            ROS 2
```

---

# 51. Subscriber Flow in Detail

```text
Python
  │
  ▼
Twist
  │
  ▼
ROS 2 Publisher
  │
  ▼
ROS 2 DDS
  │
  ▼
rclUE Subscriber
  │
  ▼
UROS2Subscriber
  │
  ▼
UROS2TwistMsg
  │
  ▼
Blueprint Callback
  │
  ▼
Get Msg
  │
  ▼
Break ROSTwist
  │
  ├──────────────┐
  │              │
  ▼              ▼
Linear          Angular
  │              │
  ▼              ▼
X              Z
  │              │
  ▼              ▼
Drive         Steering
Command       Command
```

---

# 52. Full BlueSim ROS 2 Architecture

```text
                              BlueSim
                       Unreal Engine 5.4.4
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
    BlueBoat                Drone                 Rover
        │                     │                     │
        └─────────────────────┼─────────────────────┘
                              │
                             rclUE
                              │
                ┌─────────────┴─────────────┐
                │                           │
                ▼                           ▼
           Publishers                  Subscribers
                │                           ▲
                │                           │
                ▼                           │
          Camera / Sensors              Commands
                │                           ▲
                │                           │
                └───────────┬───────────────┘
                            │
                         ROS 2 / DDS
                            │
                            ▼
                       ROS 2 Humble
                            │
                           rclpy
                            │
                ┌───────────┴───────────┐
                │                       │
                ▼                       ▼
            Perception              Control
                │                       │
                ▼                       ▼
         OpenCV / NumPy           Twist Commands
                │                       │
                └───────────┬───────────┘
                            │
                            ▼
                          ROS 2
                            │
                            ▼
                           rclUE
                            │
                            ▼
                    Unreal Robot Physics
```

---

# 53. Future Sensor Integration

The architecture is not limited to cameras.

The same principle can be used for additional sensors such as:

```text
Camera
LiDAR
Depth Camera
IMU
GPS
Sonar
Radar
Odometry
Point Clouds
```

For example:

```text
LiDAR
   │
   ▼
Unreal Sensor
   │
   ▼
rclUE Publisher
   │
   ▼
sensor_msgs/msg/LaserScan
       OR
sensor_msgs/msg/PointCloud2
   │
   ▼
ROS 2
   │
   ▼
Python
```

This allows perception algorithms to operate on simulated sensor data in the same way they operate on real robot data.

---

# 54. Future Multi-Robot Architecture

The long-term architecture can use robot-specific namespaces.

Recommended future organization:

```text
/blueboat/
/drone/
/rover/
```

For example:

```text
/blueboat/camera/image_raw
/blueboat/cmd_vel

/drone/camera/image_raw
/drone/cmd_vel

/rover/camera/image_raw
/rover/cmd_vel
```

This makes multi-robot experiments easier to manage and prevents topic-name conflicts.

---

# 55. Future Cooperative Robotics

The ROS 2 architecture allows different robots to exchange information through ROS 2.

For example:

```text
                 ┌─────────────┐
                 │   Camera    │
                 │   BlueBoat  │
                 └──────┬──────┘
                        │
                        ▼
                       ROS 2
                        │
             ┌──────────┴──────────┐
             │                     │
             ▼                     ▼
           Drone                Rover
             │                     │
             └──────────┬──────────┘
                        │
                        ▼
                Cooperative Control
```

This creates a foundation for:

- Cooperative perception
- Search and rescue
- Environmental monitoring
- UAV/USV collaboration
- Multi-robot navigation
- Distributed control

---

# 56. Performance Considerations

The current camera implementation uses a render-target readback process:

```text
GPU Render Target
       │
       ▼
ReadPixels()
       │
       ▼
CPU
       │
       ▼
ROS 2
```

The important point is that synchronous GPU-to-CPU image readback can be expensive.

The current implementation is therefore suitable for establishing the ROS 2 camera interface and experimentation, but higher-frequency multi-camera operation may require optimization.

A possible future architecture is:

```text
Current:

SceneCapture
     │
     ▼
RenderTarget
     │
     ▼
ReadPixels()
     │
     ▼
CPU
     │
     ▼
ROS 2


Future:

SceneCapture
     │
     ▼
RenderTarget
     │
     ▼
Asynchronous GPU Readback
     │
     ▼
CPU
     │
     ▼
ROS 2
```

This can reduce the impact of camera publishing on Unreal rendering performance.

---

# 57. Debugging Checklist

When a camera topic does not work, check:

```text
1. Is Unreal running?
2. Is the SceneCaptureComponent2D active?
3. Is the correct Render Target assigned?
4. Is the camera publisher component initialized?
5. Is the publisher added to the ROS 2 node?
6. Is the topic visible with ros2 topic list?
7. Is the message type sensor_msgs/msg/Image?
8. Is the publisher actually publishing?
9. Does ros2 topic hz show a rate?
10. Does the Image contain non-zero width/height?
11. Is data non-empty?
```

Useful commands:

```bash
ros2 topic list -t
```

```bash
ros2 topic info /camera_Rover -v
```

```bash
ros2 topic hz /camera_Rover
```

```bash
ros2 topic echo /camera_Rover --once --field width
```

```bash
ros2 topic echo /camera_Rover --once --field height
```

```bash
ros2 topic echo /camera_Rover --once --field encoding
```

---

# 58. Debugging Robot Commands

When a ROS command does not move a robot, check:

```text
1. Is the ROS command topic present?
2. Is the message type geometry_msgs/msg/Twist?
3. Is the Python publisher running?
4. Does ros2 topic echo show the command?
5. Is the Unreal subscriber present?
6. Is ROS2TwistMsg configured?
7. Is the callback connected?
8. Is the callback extracting Linear.X / Angular.Z?
9. Are the command variables being updated?
10. Is Event Tick applying the command to physics?
```

Useful commands:

```bash
ros2 topic list -t
```

```bash
ros2 topic info /cmd_rover_vel -v
```

```bash
ros2 topic echo /cmd_rover_vel
```

---

# 59. Main Design Principle

The most important architectural principle in BlueSim is:

```text
INPUT
  │
  ▼
COMMAND VARIABLE
  │
  ▼
CONTROL / PHYSICS
  │
  ▼
ROBOT
```

For sensors:

```text
ROBOT SENSOR
     │
     ▼
SIMULATED DATA
     │
     ▼
rclUE
     │
     ▼
ROS 2
     │
     ▼
PYTHON / ALGORITHM
```

This separation keeps the simulator modular and makes it easier to add new sensors, robots, and algorithms.

---

# 60. Summary

The BlueSim ROS 2 system provides the following pipeline:

```text
                    UNREAL ENGINE
                         │
                         ▼
                        rclUE
                         │
            ┌────────────┴────────────┐
            │                         │
        PUBLISHER                 SUBSCRIBER
            │                         ▲
            │                         │
            ▼                         │
       Sensor Data               Robot Commands
            │                         │
            ▼                         │
      sensor_msgs/...          geometry_msgs/Twist
            │                         │
            └───────────┬─────────────┘
                        │
                       ROS 2
                        │
                        ▼
                      rclpy
                        │
            ┌───────────┴───────────┐
            │                       │
        Perception               Control
            │                       │
       OpenCV / ML             Autonomy / Planning
            │                       │
            └───────────┬───────────┘
                        │
                        ▼
                      ROS 2
                        │
                        ▼
                       rclUE
                        │
                        ▼
                 Unreal Robot
```

The resulting architecture allows BlueSim to be used as a ROS 2-enabled robotics simulation platform for perception, autonomy, control, and multi-robot research.

---

# 61. Quick Start

Build the ROS 2 workspace:

```bash
cd BlueSim/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

Check the package:

```bash
ros2 pkg list | grep bluesim
```

Start BlueSim in Unreal Engine and play the simulation.

Then inspect the ROS 2 system:

```bash
ros2 node list
```

```bash
ros2 topic list -t
```

Check a camera:

```bash
ros2 topic hz /camera_Rover
```

Check a command topic:

```bash
ros2 topic info /cmd_rover_vel -v
```

The BlueSim ROS 2 interface is then ready for Python-based perception, control, autonomy, and experimentation.