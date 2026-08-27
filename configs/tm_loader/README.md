# 📜 TM Robot Deployment Loader Script Guide: lite_ld.sh Edition

This technical guide provides comprehensive operational documentation for the Lightweight Single-Target Loader script (`lite_ld.sh`). This script optimizes workspace management for Techman Robot series configurations within TM ROS 2 Jazzy or higher environments.

---

## 1. 🧩 Functional Overview

`lite_ld.sh` is a precise, lightweight deployment loader designed to extract a single robot profile and a single specific ROS package from remote repositories.

### Key Capabilities
*   **Targeted Extraction**: Downloads only the requested robot model folder instead of cloning full repositories.
*   **Low Footprint**: Employs Git Sparse-Checkout and Shallow Clone (`--depth 1`) to minimize disk overhead and bandwidth consumption.
*   **Modular Architecture**: Tailored for developers adopting a **Progressive Manual Deployment Paradigm**, ensuring workspaces remain uncluttered.

---

## 2. ⚙️ Prerequisites & Environment Setup

Ensure your local development platform satisfies the minimum operational baselines before executing the tool.

### System Requirements
*   **TM ROS2 Jazzy APPs**: TM ROS2 version **Jazzy or higher** is strictly required to support the modern TM Robots Series file system structure syntax.
*   **Git Engine**: Version **2.25 or higher** is required for native sparse-checkout engine mechanics.

### Permissions Assignment
Before initial execution, grant execute privileges to the wrapper script:
```bash
chmod +x lite_ld.sh
```

### 🔖 Path Workspace Configuration: user_workspace.txt

Set your environment root path before execution.

```text
# ==============================================================================
#    User Workspace Configuration File
# ==============================================================================
# INSTRUCTION: Set the root paths to match your local development environment.
# IMPORTANT: Do NOT leave a trailing slash '/' at the end of the path.

# ---- Default Example ----
WS_ROOT=$HOME/tm2_ws
```
> 📄 Reference: [`user_workspace.txt`](https://github.com/TechmanRobotInc/tm2_ros2/tree/jazzy/configs/tm_loader/user_workspace.txt)

---

## 3. ⌨️ Command Syntax & Arguments

### ⚡ lite_ld.sh <sub><sup>(Lightweight Single-Target Loader)</sup></sub>
```bash
./lite_ld.sh [MODEL] [PACKAGE (optional)] [-f (optional)]
```

#### Arguments Breakdown
*   **`[MODEL]` (Required)**
    *   Specifies the exact Techman Robot profile to extract.
    *   *Examples*: `tm5s`, `tm12s`, `tm14s`.
*   **`[PACKAGE]` (Optional)**
    *   Specifies the target ROS 2 package suite. Defaults automatically to **`tm_description`** if omitted.
    *   **`tm_description`**: Contains 3D meshes, URDF models, and Xacro kinematic profiles.
    *   **`tm_moveit`**: Contains MoveIt 2 motion planning setups and collision matrix configurations.
    *   **`tm_gazebo`**: Contains modern Gazebo Sim physics simulation environment setups.
*   **`-f` (Optional)**
    *   Activates the **Forced Overwrite Flag**.
    *   Bypasses the interactive terminal choice prompt (`Overwrite? (y/n)`) and **instantly deletes** the colliding local target package if it already exists.
> 🗂️ The Loader Script [`tm_loader` tool](https://github.com/TechmanRobotInc/tm2_ros2/tree/jazzy/configs/tm_loader/)

---

## 4. 🎬 Practical Execution Examples

*   **Scenario 1: Deploy the core 3D kinematics description profile for tm5s**
    ```bash
    ./lite_ld.sh tm5s tm_description
    ```
*   **Scenario 2: Unattended Forced Update tm12s (Robot 3D Description Pack: Automatically to **`tm_description`** if omitted)**
    ```bash
    ./lite_ld.sh tm12s -f
    ```
*   **Scenario 3: Deploy Specific Package tm12s (MoveIt 2 Configurations)**
    ```bash
    ./lite_ld.sh tm12s tm_moveit
    ```
*   **Scenario 4: Deploy Specific Package tm12s (Fetch Gazebo Sim & Wipe Legacy Files)**
    ```bash
    ./lite_ld.sh tm12s tm_gazebo
    ```

---

## 5. 🌿 Post-Deployment Workspace Topology

After successfully executing the initialization scripts within your workspace (Example: `~/tm2_ws/tm2_ros2` or `~/user_ws/tm2_ros2`), the structural tree populates as modeled below:
*(Note: If your local setup utilizes an alternative directory path such as `~/user_ws/`, adjust the execution path accordingly.)*

When executing `lite_ld.sh`, the script dynamically maps the model's lineage (e.g., classifying `tm5s` under the `cobot_s` directory tree). 

```text
📁 tm2_ws/ (or catkin_ws/ or user_ws/)   # User's workspace root directory (from user_workspace.txt)
┗━ 📁 tm2_ros2/                          # TM ROS 2 standard source code directory (Acts as the 'src' space)
   ┣━ 📁 tm_description/                 # Robot Model Description Module (PACKAGE: tm_description)
   ┃  ┗━ 📁 cobot_s/                     # Auto-resolved series parent folder for TM S-Series
   ┃     ┣━ 📁 tm12s_description/        # Default TM12S 3D meshes, URDF models, and Xacro files
   ┃     ┗━ 📁 tm5s_description/         # Extracted via: ./lite_ld.sh tm12s
   ┣━ 📁 tm_moveit/                      # Motion Planning Module (PACKAGE: tm_moveit)
   ┃  ┗━ 📁 cobot_s/                     # Auto-anchored baseline location for TM S-Series
   ┃     ┗━ 📁 tm12s_moveit/             # Extracted via: ./lite_ld.sh tm12s tm_moveit
   ┗━ 📁 tm_gazebo/                      # Physics Simulation Module (PACKAGE: tm_gazebo)
      ┗━ 📁 cobot_s/                     # Auto-resolved series parent folder for TM S-Series
         ┗━ 📁 tm12s_gazebo/             # Extracted via: ./lite_ld.sh tm5s tm_gazebo -f
```

---

## 6. 📈 Progressive Manual Deployment Workflow

This paradigm maximizes hardware resource efficiency by gradually expanding your local development footprint only when features are requested.

```bash
# Step 1: Deploy the core 3D kinematics description profile for tm7s (Automatically to **`tm_description`** if omitted)
./lite_ld.sh tm7s

# Step 2: Dynamically append MoveIt 2 motion planning to the tm7s workspace
./lite_ld.sh tm7s tm_moveit

# Step 3: Expand the tm7s workspace to include Gazebo Sim physics environments
./lite_ld.sh tm7s tm_gazebo
```
---

## 7. 🔨 Compilation Workflow & Workspace Activation

Once your subset of packages has finished loading via `lite_ld.sh`, update dependencies and compile via standard ROS 2 `colcon` parameters.

#### Step 1: Install System Dependencies
Automatically pull down hardware communication libraries and external workspace dependencies:
```bash
cd ~/tm2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### Step 2: Build the Workspace
Compile the newly loaded Techman Robot functional packages using optimized symlink installation rules:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Step 3: Source the Environment
Register the compiled Techman executables, launch scripts, and 3D mesh pipelines into your active terminal shell session:
```bash
source install/setup.bash
```

---

## 8. 🩺 Troubleshooting & Diagnostics

##### Error: `fatal: Protocol error: bad line length character` or Network Drops(Timeout)
*   **Root Cause**: Local firewall restrictions, corporate proxy barriers, or transient GitHub server outages disrupting the sparse-checkout engine.
*   **Remediation**: Force your local Git layer to expand network buffers using HTTP configurations:
    ```bash
    git config --global http.postBuffer 524288000
    git config --global core.compression 0
    ```
