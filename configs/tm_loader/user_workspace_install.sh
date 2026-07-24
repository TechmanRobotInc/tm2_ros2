#!/bin/bash
###########################################################################################
# Script Name: user_workspace_install.sh                                                  #
# Description: Advanced Workspace Configuration & Setup                                   #
# Version:     1.0.0                                                                      #
###########################################################################################
# ANSI Terminal Color Escape Codes
RED='\033[1;31m'
PURPLE='\033[1;35m'  # Purple Color for reminders
NC='\033[0m'        # No Color (Reset)

# Configuration file name
CONFIG_FILE="./user_workspace.txt"

# 1. Initialize fallback path if config does not exist
WS_ROOT="$HOME/tm2_ws"

# 2. Check and parse configuration file
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ Error: Configuration file '$CONFIG_FILE' not found."
    echo "💡 Using default fallback path."
else
    while IFS='=' read -r key value || [ -n "$key" ]; do
        key=$(echo "$key" | tr -d '\r' | xargs)
        value=$(echo "$value" | tr -d '\r' | xargs)

        [[ "$key" =~ ^#.*$ ]] && continue
        [[ -z "$key" ]] && continue

        # Safely replace $HOME variable without using eval
        value="${value/\$HOME/$HOME}"

        if [ "$key" == "WS_ROOT" ]; then
            WS_ROOT="${value%/}"
        fi
    done < "$CONFIG_FILE"
fi

SRC_DIR="${WS_ROOT}"
# Please select 'tm2_ros2' for S Series, or 'tmr_ros2' for legacy series.
REPO_URL="https://github.com/TechmanRobotInc/tm2_ros2.git" 
REPO_NAME="tm2_ros2"

# 3. Auto-verify workspace structure
if [ ! -d "$SRC_DIR" ]; then
    echo "📁 Target path does not exist. Creating ROS 2 workspace: $SRC_DIR"
    mkdir -p "$SRC_DIR"
fi

# 4. Check and clone Techman Robot repository
cd "$SRC_DIR" || { echo "❌ Error: Failed to access directory $SRC_DIR"; exit 1; }

if [ ! -d "$REPO_NAME" ]; then
    echo "📥 Cloning Techman Robot ROS 2 Jazzy branch directly into current directory..."
    if git clone "$REPO_URL" -b jazzy; then
        echo "⏳ Syncing file system..."
        sync # safe and fast.
        echo "✅ Workspace initialization and git clone completed!"
    else
        echo "❌ Error: Git clone failed. Please check your network connection."
        exit 1
    fi
else
    echo "✨ Techman Robot repository already exists. Skipping clone."
fi

# 5. Display the finalized path configuration to the user
echo "============================================================================"
echo "🚀 Workspace root path successfully loaded!"
echo "📂 Workspace Root: $WS_ROOT"
echo "📂 Source Directory: $SRC_DIR"
echo -e "${PURPLE}🔔 [Reminder: If deployed successfully, remember to rebuild your workspace.]${NC}"
# echo -e "${PURPLE}   1. To clear old build files, run 'rm -rf build/ install/ log/' inside the Workspace Root.${NC}"
# echo -e "${PURPLE}   2. To recompile, run 'colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'.${NC}"
echo "============================================================================"

