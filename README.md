# XRCE Link Lab — Phase 2 (micro-ROS → Agent → ROS 2 Demo)

This repository contains a **minimal, reproducible** Phase 2 demo that connects a micro-ROS client (running on host) to the **Micro XRCE-DDS Agent** and exposes data on the **ROS 2 Graph** (Humble). It is intentionally lightweight and **does not vendor third‑party sources**; dependencies are fetched at build time using standard micro-ROS/ROS 2 tooling.

---

## Table of Contents
- [What you get](#what-you-get)
- [Prerequisites](#prerequisites)
- [Repository layout](#repository-layout)
- [Quickstart (5–10 min)](#quickstart-510-min)
- [Run the demo](#run-the-demo)
- [Verify + collect metrics](#verify--collect-metrics)
- [Troubleshooting](#troubleshooting)
- [Why this matters](#why-this-matters)
- [License](#license)

---

## What you get

- A **host micro-ROS client** that publishes `std_msgs/Int32` over **rmw_microxrcedds** via UDP to the Micro XRCE-DDS Agent.
- A small **ROS 2 package** (`xrce_link_demo`) with:
  - `relay` — remaps the demo topic to `/mcu/heartbeat`
  - `processor` — 5‑sample moving average on `/mcu/heartbeat` → `/mcu/heartbeat_avg5`
  - `jitter` — reports inter‑arrival **avg/std/p90** for `/mcu/heartbeat` (for reviewers who love numbers)
- A **repeatable build** that **does not commit** third‑party code: we keep a `ros2.repos` file and let `vcs import` fetch the exact sources.

---

## Prerequisites

- Ubuntu **22.04** with **ROS 2 Humble** installed (`/opt/ros/humble` present).
- Packages:
  ```bash
  sudo apt update
  sudo apt install -y python3-vcstool python3-colcon-common-extensions
  ```

> No GitHub Actions/CI needed for Phase 2. This repo deliberately ships **no workflows**.

---

## Repository layout

```
.
├─ phase2/                      # Everything specific to this Phase 2 demo
│  ├─ microros_ws/              # Minimal micro-ROS workspace (no third-party code)
│  │  ├─ firmware/
│  │  │  └─ dev_ws/
│  │  │     ├─ ros2.repos      # Pinned sources (eProsima, micro-ROS, ROS 2 common_interfaces)
│  │  │     └─ BOOTSTRAP.sh    # One-button setup & build
│  │  └─ .gitignore            # Ignores build/, install/, log/ etc.
│  └─ phase2_ros2_ws/           # Minimal ROS 2 ws containing xrce_link_demo
│     └─ src/xrce_link_demo/    # relay/processor/jitter + launch
└─ README.md                    # (this file)
```

> The repo **does not** include any `build/`, `install/`, `log/` folders or third‑party trees. You can upload this layout using only the GitHub web UI.

---

## Quickstart (5–10 min)

> **Open 4 terminals** (A/B/C/D). Copy–paste exactly. If any step prints errors, see troubleshooting at the end.

### 1) Terminal **A** — XRCE Agent
```bash
unset RMW_IMPLEMENTATION; unset ROS_DOMAIN_ID
source /opt/ros/humble/setup.bash
micro-ros-agent udp4 --port 8888 -v6
```

### 2) Terminal **B** — micro‑ROS client (host)
```bash
# Prepare tree
mkdir -p ~/phase2/ && cd ~/phase2/

# === micro-ROS workspace (host client) ===
mkdir -p microros_ws/firmware/dev_ws/src
cd microros_ws

# Bring in Phase 2 helper files from this repo (clone or download zip beforehand)
# e.g., if this repository is cloned at ~/XRCE-Phase2:
# cp -r ~/XRCE-Phase2/phase2/microros_ws/* ./

# Bootstrap (fetch sources and build)
bash firmware/dev_ws/BOOTSTRAP.sh

# Activate
source firmware/dev_ws/install/setup.bash

# Run the demo publisher with rmw_microxrcedds
export RMW_IMPLEMENTATION=rmw_microxrcedds
export MICRO_ROS_AGENT_IP=127.0.0.1
export MICRO_ROS_AGENT_PORT=8888
ros2 run micro_ros_demos_rclc int32_publisher || \
ros2 run micro_ros_demos_rclc int32_publisher_node
```

### 3) Terminal **C** — ROS 2 helper nodes
```bash
# Build the tiny ROS 2 ws in this repo
cd ~/phase2/phase2_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Launch relay + processor
ros2 launch xrce_link_demo demo.launch.py
```

### 4) Terminal **D** — Verify topics
```bash
source /opt/ros/humble/setup.bash

# Upstream (micro-ROS demo topic; some versions use the type name as topic)
ros2 topic echo /std_msgs_msg_Int32 --qos-profile sensor_data

# Relayed topic (semantically named)
ros2 topic echo /mcu/heartbeat
ros2 topic hz /mcu/heartbeat

# Optional: jitter stats (avg/std/p90 every 50 msgs)
source ~/phase2/phase2_ros2_ws/install/setup.bash
ros2 run xrce_link_demo jitter
```

Expected:
- Agent (A) prints `session established`, `write`, etc.
- Publisher (B) prints `Sent: N` increasing.
- D shows data on `/std_msgs_msg_Int32` and `/mcu/heartbeat` and `hz ≈ 1.0 Hz` (default demo rate).
- `jitter` prints e.g. `avg ~1000.0 ms, std ~0.1 ms, p90 ~1000.1 ms`.

---

## Troubleshooting

**No data on `/std_msgs_msg_Int32`**  
- Ensure Terminal B exported:
  ```bash
  export RMW_IMPLEMENTATION=rmw_microxrcedds
  export MICRO_ROS_AGENT_IP=127.0.0.1
  export MICRO_ROS_AGENT_PORT=8888
  ```
- Use QoS compatible echo: `--qos-profile sensor_data`.

**Agent shows nothing when publisher runs**  
- Forgot to run Agent first, or wrong IP/port/domain id.
- Clear env: `unset RMW_IMPLEMENTATION; unset ROS_DOMAIN_ID` in all shells.

**Runtime error: cannot load `libstd_msgs__rosidl_typesupport_microxrcedds_c.so`**  
- Rebuild **std_msgs** with micro-XRCE typesupport as shared library:
  ```bash
  cd ~/phase2/microros_ws/firmware/dev_ws
  rm -rf build/std_msgs install/std_msgs log
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  colcon build --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF -DBUILD_SHARED_LIBS=ON \
                 -DROSIDL_TYPESUPPORTS=rosidl_typesupport_microxrcedds_c \
    --packages-select std_msgs
  ```

**Topic name differs (`/int32` vs `/std_msgs_msg_Int32`)**  
- Demo versions vary. The relay node normalizes to `/mcu/heartbeat`.

---

## Why this matters

Most ROS 2 tutorials assume nodes run on a PC. In the real world, many MCU targets (nRF52/STM32/ESP32) talk XRCE-DDS and need an Agent to reach the DDS graph. This demo provides a **clean, automatable template** showing:
- how to bring a **micro-ROS client** online (even without physical hardware),
- how to bridge it into the **ROS 2 graph** via Agent,
- how to verify **throughput/stability** (Hz + jitter) — the first metrics reviewers ask for.

This Phase 2 stands alone for reproducibility while keeping Phase 1 CI green.

---

## License

Apache-2.0 for original work here. Third‑party projects fetched via `ros2.repos` retain their respective licenses.
