#!/bin/bash
###########################################################################################
# Script Name: deploy.sh                                                                  #
# Description: Pure deployment wrapper for lite_ld.sh with dynamic package swapping       #
# Version:     1.0.0                                                                      #
# Usage:       ./deploy.sh [model_name]                                                   #
# Example:     ./deploy.sh tm5s                                                           #
###########################################################################################


# ==============================================================================
# 0. Core Executable Options & Dynamic Target Package Swapping
# ==============================================================================
COBOT_MODEL="${1:-tm12s}"

# The core loader binary targeted for deployment
LOADER_BIN="lite_ld.sh"

# ✨ FLEXIBLE PACKAGE SWITCH: Change between "tm_description", "tm_gazebo", or "tm_moveit"
# Modify this variable line to switch your target deployment subsystem seamlessly
TARGET_PKG="tm_description"

# Construct tokenized command array explicitly for lite_ld.sh
# ./lite_ld.sh [MODEL] tm_description -f 
LOADER_CMD=("./${LOADER_BIN}" "$COBOT_MODEL" "$TARGET_PKG" "-f")

# ==============================================================================
# 1. Environment & Version Initialization
# ==============================================================================
if [ -f "version.txt" ]; then
    VERSION=$(cat version.txt)
else
    VERSION="Unknown (version.txt missing)"
fi

echo "============================================================================"
echo "📦 tm_loader version is : $VERSION"

# ==============================================================================
# 2. Path Resolutions & Workspace Parsing
# ==============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/user_workspace.txt"

# Default fallback environment setups
WS_ROOT="$HOME/tm2_ws"

# Source custom setup definitions if configuration files are accessible
if [ -f "$CONFIG_FILE" ]; then
    while IFS='=' read -r key value || [ -n "$key" ]; do
        key=$(echo "$key" | tr -d '\r' | xargs)
        value=$(echo "$value" | tr -d '\r' | xargs)
        [[ "$key" =~ ^#.*$ ]] && continue
        [[ -z "$key" ]] && continue
        value="${value/\$HOME/$HOME}"
        if [ "$key" == "WS_ROOT" ]; then WS_ROOT="${value%/}"; fi
    done < "$CONFIG_FILE"
fi

# Assign fully verified workspace root derived from dynamic configurations
WORKSPACE_ROOT="$WS_ROOT"

echo "📂 Script Location   : $SCRIPT_DIR"
echo "📂 Workspace Root    : $WORKSPACE_ROOT"
echo "🤖 Target Model      : $COBOT_MODEL"
# echo "📦 Target Package    : $TARGET_PKG"
# echo "⚙️  Selected Binary  : ./${LOADER_BIN}"
# echo "📜 Dispatched Cmd    : ${LOADER_CMD[*]}"
echo "============================================================================"

# ==============================================================================
# 3. Dynamic ROS 2 Environment Lookup
# ==============================================================================
ros_sourced=false

# Search through all folders inside /opt/ros/
for dist in /opt/ros/*; do
    if [ -f "$dist/setup.bash" ]; then
        source "$dist/setup.bash"
        ros_distro_name=$(basename "$dist")
        echo "✅ Loaded ROS 2 ($ros_distro_name) Environment automatically."
        ros_sourced=true
        break 
    fi
done

# If no active ROS 2 installation path is verified, terminate execution safely
if [ "$ros_sourced" = false ]; then
    echo "❌ Error: No valid system ROS 2 installation found under /opt/ros/*!"
    exit 1
fi

# ==============================================================================
# 4. Model Deployment Execution
# ==============================================================================
# Navigate back to the loader directory and execute deployment
cd "$SCRIPT_DIR" || exit 1

# Verify if the chosen loader binary actually exists in the local environment
if [ ! -f "./${LOADER_BIN}" ]; then
    echo "============================================================================"
    echo "❌ Error: Configured loader file './${LOADER_BIN}' not found!"
    echo "   Please check if the filename changed or if it exists in this directory."
    echo "============================================================================"
    exit 1
fi

chmod +x "./${LOADER_BIN}"

# Execute the robustly tokenized command array directly
if ! "${LOADER_CMD[@]}"; then
    echo "============================================================================"
    echo "❌ Error: Deployment failed via ./${LOADER_BIN}! Aborting compilation."
    echo "============================================================================"
    exit 1
fi

# Allow file system cache buffers to sync completely
sleep 3s

# ==============================================================================
# 5. Workspace Clean & Rebuild (Colcon)
# ==============================================================================
echo "============================================================================"
echo "🛠️  Cleaning Workspace Cache and Rebuilding ROS 2 Workspace..."
echo "============================================================================"
cd "$WORKSPACE_ROOT" || { echo -e "❌ Error: Cannot access Workspace Root directory: $WORKSPACE_ROOT"; exit 1; }

# Erase old compilation artifacts to prevent cross-branch code caching issues
rm -rf build/ install/ log/

# Run colcon compilation with optimized Release arguments and multi-core speedup
# Note: Added '--parallel-workers' to utilize all CPU cores for faster build.
colcon build \
    --parallel-workers "$(nproc)" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# [💡 Tip for Development]: If you don't want to clear cache every time, comment out the `rm -rf` above 
# and use `--packages-select tm_description` to build only the target package:
# colcon build --packages-select tm_description --cmake-args -DCMAKE_BUILD_TYPE=Release

# ==============================================================================
# 6. Post-Build Environment Sourcing
# ==============================================================================
if [ -f "./install/setup.bash" ]; then
    source ./install/setup.bash
    echo "============================================================================"
    echo "✅ Success: Workspace rebuilt and environmental paths loaded successfully!"
    echo "============================================================================"
else
    echo "============================================================================"
    echo "⚠️  Warning: Compilation might have failed. './install/setup.bash' not found."
    echo "============================================================================"
    exit 1
fi
