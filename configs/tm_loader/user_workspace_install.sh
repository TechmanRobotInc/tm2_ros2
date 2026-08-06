#!/bin/bash
###########################################################################################
# Script Name: user_workspace_install.sh                                                  #
# Description: Advanced Workspace Configuration & Setup                                   #
# Version:     1.1.0                                                                      #
###########################################################################################
# ANSI Terminal Color Escape Codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[1;31m'
PURPLE='\033[1;35m'
NC='\033[0m'

# Configuration file name
CONFIG_FILE="./user_workspace.txt"

# ==============================================================================
# 1. Initialize Fallback Path if Config Does Not Exist
# ==============================================================================
SRC_DIR="tm2_ros2"
WS_ROOT="$HOME/tm2_ws"

if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${YELLOW}⚠️  Warning: Configuration file '$CONFIG_FILE' not found. Using fallbacks.${NC}"
else
    while IFS='=' read -r key value || [ -n "$key" ]; do
        key=$(echo "$key" | tr -d '\r' | xargs)
        value=$(echo "$value" | tr -d '\r' | xargs)
        [[ "$key" =~ ^#.*$ ]] && continue
        [[ -z "$key" ]] && continue
        value="${value/\$HOME/$HOME}"
        if [ "$key" == "WS_ROOT" ]; then WS_ROOT="${value%/}"; fi
        if [ "$key" == "SRC_DIR" ]; then SRC_DIR="${value%/}"; fi
    done < "$CONFIG_FILE"
fi

TARGET_SRC_PATH="${WS_ROOT}/${SRC_DIR}"
# Please select 'tm2_ros2' for S Series, or 'tmr_ros2' for legacy series.
REPO_URL="https://github.com/TechmanRobotInc/tm2_ros2.git" 
REPO_NAME="tm2_ros2"

# ==============================================================================
# 2. Setup and Verify ROS 2 Core Source Workspace Layout
# ==============================================================================
if [ ! -d "$TARGET_SRC_PATH" ]; then
    echo "📁 Creating ROS 2 Source Space: $TARGET_SRC_PATH"
    mkdir -p "$TARGET_SRC_PATH"
fi

# ==============================================================================
# 3. Deploy Techman Robot Repository (tm2_ros2)
# ==============================================================================
cd "$TARGET_SRC_PATH" || { echo -e "${RED}❌ Error: Cannot access $TARGET_SRC_PATH${NC}"; exit 1; }

if [ ! -d ".git" ]; then
    echo "📥 Cloning Techman Robot ROS 2 Jazzy core apps..."
    if git clone "$REPO_URL" -b jazzy .; then
        sync
        echo -e "${GREEN}✅ Core drivers successfully deployed.${NC}"
    else
        echo -e "${RED}❌ Error: Git clone failed. Please check your connection.${NC}"
        exit 1
    fi
else
    echo "✨ Techman Robot repository already exists in $TARGET_SRC_PATH. Skipping clone."
fi

# ==============================================================================
# 4. Deploy Standalone Cobot Packages Inherited from Main Config Launcher
# ==============================================================================
echo "⚙️  Deploying and Migrating Standalone Cobot Repositories..."

sync_standalone_repo() {
    local repo_url="$1"
    local branch="$2"
    local target_dir="$3" 
    
    # Create the nested target profile directory
    mkdir -p "$target_dir"

    # Isolate runtime artifacts inside unique temporary spaces
    local tmp_clone_dir="/tmp/tm_migration_$(date +%s%N)"
    
    echo "📥 Fetching standalone packages from [$repo_url]..."
    if git clone "$repo_url" -b "$branch" --depth 1 "$tmp_clone_dir" 2>/dev/null; then
        echo "🚚 Extracting and merging profiles into nested structure..."
        
        # Recursively transfer standalone directories without copying parent .git objects
        cp -r "$tmp_clone_dir"/* "$target_dir/" 2>/dev/null
        
        # Flush volatile file cache safely
        rm -rf "$tmp_clone_dir"
        sync
        echo -e "${GREEN}   -> Successfully migrated standalone package into: $target_dir${NC}"
    else
        echo -e "${RED}   ❌ Error: Failed to fetch standalone repo $repo_url${NC}"
        rm -rf "$tmp_clone_dir"
    fi
}

if [ -n "$REPO_tm_description" ] && [ -n "$DIR_tm_description" ]; then
    sync_standalone_repo "$REPO_tm_description" "$BRANCH_tm_description" "$DIR_tm_description"
fi

if [ -n "$REPO_tm_moveit" ] && [ -n "$DIR_tm_moveit" ]; then
    sync_standalone_repo "$REPO_tm_moveit" "$BRANCH_tm_moveit" "$DIR_tm_moveit"
fi

if [ -n "$REPO_tm_gazebo" ] && [ -n "$DIR_tm_gazebo" ]; then
    sync_standalone_repo "$REPO_tm_gazebo" "$BRANCH_tm_gazebo" "$DIR_tm_gazebo"
fi

# ==============================================================================
# 5. Finalized Summary Display
# ==============================================================================
echo "============================================================================"
# echo -e "${GREEN}🚀 Automated Workspace Deployment Pipeline Completed Successfully!${NC}"
echo "📂 Workspace Root : $WS_ROOT"
echo "📂 Source Folder  : $TARGET_SRC_PATH"
echo -e "${PURPLE}🔔 [Reminder: If deployed successfully, remember to rebuild your workspace.]${NC}"
# echo -e "${PURPLE}   1. Switch to workspace root: cd $WS_ROOT${NC}"
# echo -e "${PURPLE}   2. To clear old build files, run 'rm -rf build/ install/ log/' inside the Workspace Root.${NC}"
# echo -e "${PURPLE}   3. To recompile, run 'colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'.${NC}"
# echo -e "${PURPLE}   4. Source environment      : source install/setup.bash${NC}"
echo "============================================================================"
