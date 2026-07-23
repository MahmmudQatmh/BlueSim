# 🌊 BlueSim — Autonomous Maritime Vehicle Simulator

[![Unreal Engine](https://img.shields.io/badge/UnrealEngine-5.4-blue?logo=unrealengine)](https://www.unrealengine.com/)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20/%20Jazzy-brightgreen?logo=ros)](https://docs.ros.org/)
[![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-red)](https://ardupilot.org/)
[![Status](https://img.shields.io/badge/Roadmap-Phase_A_(In_Progress)-orange)](#-project-roadmap)

> High-fidelity simulation platform built in **Unreal Engine 5** designed to emulate the **BlueRobotics BlueBoat** in realistic ocean environments, integrated with **ROS 2** and **ArduPilot SITL** for multi-agent cooperative control research.

---

## 📌 Context & Objectives

This project is developed as part of research activities at the **University of Turku** during Summer 2026. 

The primary objective is to bridge physical hardware experimentation with software-in-the-loop simulation. Alongside deploying ROS 2 on real hardware (configuring the Raspberry Pi onboard the BlueBoat), **BlueSim** serves as a digital twin for safe testing, sensor validation, and algorithm development.

### Key Goals
* 🌊 **Physics-accurate Ocean Simulation**: Model realistic buoyancy, hydrodynamic drag, and wave interaction for the BlueBoat USV.
* 🛰️ **Hardware-In-The-Loop / SITL**: Connect seamlessly to ArduPilot SITL and QGroundControl for realistic navigation behavior.
* 🤖 **ROS 2 Bridge**: Stream synthetic sensor outputs (cameras, odometry, IMU) and accept actuation controls (`cmd_vel`).
* 🚁 **Multi-Robot Cooperative Control**: Scale the simulation environment to support collaborative operations between the BlueBoat, autonomous drones, and rovers.

---

## 🚀 Project Architecture

The targeted execution pipeline establishes a closed-loop system between simulation physics, autopilot software, ROS 2 nodes, and Ground Control Stations:


```text
┌─────────────────┐       SITL        ┌─────────────┐
│  ArduPilot      │ ◄───────────────► │   BlueSim   │
│  (Navigation)   │                   │ (UE5 Ocean) │
└────────┬────────┘                   └──────┬──────┘
         │                                   │ Sensor Data /
         │ Telemetry                         │ Camera Feeds
         ▼                                   ▼
┌─────────────────┐                   ┌─────────────┐
│ QGroundControl  │                   │    ROS 2    │
│  (GCS Interface)│                   │ (Middleware)│
└─────────────────┘                   └──────┬──────┘
                                             │
                                             ▼
                                  ┌────────────────────┐
                                  │  Multi-Robot /     │
                                  │  Coop Control      │
                                  └────────────────────┘
```



---

## 🗺️ Project Roadmap & Progress

The development of BlueSim is structured across **seven planned phases** over a 2–4 week sprint.

### Current Status: ⚙️ Phase A (In Progress)

- [ ] **Phase A — Build the Ocean Environment** *(Current)*
  - [x] Configure Unreal Engine 5 project settings & renderer
  - [ ] Enable Water Plugin, Landmass, and Modeling Tools
  - [ ] Set up realistic Ocean, Sky Atmosphere, and Directional Sun Lighting
  - [ ] Save base scene under `Content/Maps/Ocean.umap`

- [ ] **Phase B — Floating Boat Physics**
  - [ ] Implement initial buoyancy body (cube verification)
  - [ ] Import BlueBoat 3D mesh model & physics asset
  - [ ] Validate hull collision, surface hydrodynamic forces, and manual movement

- [ ] **Phase C — Synthetic Sensor Integration**
  - [ ] Add onboard RGB camera sensors to the BlueBoat
  - [ ] Configure render targets and frame rate synchronization inside UE5

- [ ] **Phase D — ROS 2 Middleware Integration**
  - [ ] Establish ROS 2 bridge node
  - [ ] Publish synthetic sensor topics (`/camera/image`, `/odom`, `/tf`)
  - [ ] Subscribe to control commands (`/cmd_vel`)

- [ ] **Phase E — ArduPilot SITL Integration**
  - [ ] Establish socket connection between UE5 and ArduPilot SITL
  - [ ] Connect telemetry streams to QGroundControl
  - [ ] Validate autonomous waypoint navigation on water

- [ ] **Phase F — Multi-Robot System Expansion**
  - [ ] Introduce secondary robotic agents (Aerial Drone & Ground Rover) into `Ocean.umap`
  - [ ] Isolate individual ROS 2 namespaces and sensor streams per vehicle

- [ ] **Phase G — Cooperative Control & Field Testing**
  - [ ] Implement multi-agent algorithms (Formation Control, Search & Rescue, Sensor Sharing)
  - [ ] Cross-validate simulation performance against physical BlueBoat field deployment data

---

## 🛠️ Prerequisites & Setup

### Requirements
* **OS**: Ubuntu 22.04 LTS (or compatible Linux distro)
* **GPU**: Vulkan 1.2+ compatible dedicated graphics card (NVIDIA RTX recommended)
* **Engine**: Unreal Engine 5.4+
* **Middleware**: ROS 2 (Humble / Jazzy)

### Quick Start
1. **Clone the Repository:**
   ```bash
   git clone [https://github.com/MahmmudQatmh/BlueSim.git](https://github.com/MahmmudQatmh/BlueSim.git)
   cd BlueSim