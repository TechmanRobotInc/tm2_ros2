#!/bin/bash
###########################################################################################
# Script Name: user_workspace_install.sh
# 
# SPDX-FileCopyrightText: Copyright (c) 2026, TECHMAN ROBOT INC.
# SPDX-FileContributor: yh.wang
# SPDX-License-Identifier: BSD-3-Clause
# 
# Description: Advanced Workspace Configuration & Setup
# Version:     1.2.0
###########################################################################################

# ANSI Terminal Color Escape Codes
RED='\033[1;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[1;34m'
PURPLE='\033[1;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color (Reset)

# Configuration file name
CONFIG_FILE="./user_workspace.txt"

# ==============================================================================
# 0. Global Feature Switches
# ==============================================================================
DEPLOY_STANDALONE=false

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

# ==============================================================================
# 2. Safety Fallback Variable Declarations (Ensures values exist if called standalone)
# ==============================================================================
ROS_DISTRO_BRANCH="jazzy" 

REPO_URL="https://github.com/TechmanRobotInc/tm2_ros2.git"
REPO_NAME="tm2_ros2"

[ -z "$REPO_tm_description" ] && REPO_tm_description="https://github.com/TechmanRobotInc/TM_Cobots_ROS2_Description"
[ -z "$REPO_tm_moveit" ]      && REPO_tm_moveit="https://github.com/TechmanRobotInc/TM_Cobots_ROS2_Moveit2"
[ -z "$REPO_tm_gazebo" ]      && REPO_tm_gazebo="https://github.com/TechmanRobotInc/TM_Cobots_ROS2_GZ_Simulation"

[ -z "$BRANCH_tm_description" ] && BRANCH_tm_description="main"
[ -z "$BRANCH_tm_moveit" ]      && BRANCH_tm_moveit="main"
[ -z "$BRANCH_tm_gazebo" ]      && BRANCH_tm_gazebo="main"

[ -z "$DIR_tm_description" ] && DIR_tm_description="$TARGET_SRC_PATH/tm_description/cobot_s"
[ -z "$DIR_tm_moveit" ]      && DIR_tm_moveit="$TARGET_SRC_PATH/tm_moveit/cobot_s"
[ -z "$DIR_tm_gazebo" ]      && DIR_tm_gazebo="$TARGET_SRC_PATH/tm_gazebo/cobot_s"

# ==============================================================================
# 3. Interactive Path Check & Backup Workflow
# ==============================================================================
CURRENT_WS_NAME=$(basename "$WS_ROOT")
BACKUP_SUFFIX="$(date +%Y%m%d_%H%M%S).bak"

handle_existing_directory() {
    local target_to_check="$1"
    local display_name="$2"

    if [ -d "$target_to_check" ] && [ "$(ls -A "$target_to_check" 2>/dev/null)" ]; then
        echo -e "${YELLOW}⚠️  Warning: $display_name directory already exists and is not empty: $target_to_check${NC}"
        read -p "❓ Do you want to BACKUP this directory before proceeding? (y/n): " chk_backup
        if [[ "$chk_backup" =~ ^[Yy]$ ]]; then
            local backup_path="${target_to_check}_${BACKUP_SUFFIX}"
            echo -e "${CYAN}📦 Backing up directory to: $backup_path ...${NC}"
            mv "$target_to_check" "$backup_path"
            echo -e "${GREEN}✅ Backup completed successfully.${NC}"
        else
            echo -e "${RED}⚠️  Proceeding without backup. Overwriting will clear existing data!${NC}"
            read -p "👉 Choose an option: [1] Continue & Overwrite, [2] Exit Script (1/2): " chk_overwrite
            if [ "$chk_overwrite" == "1" ]; then
                echo -e "${CYAN}🧹 Clearing existing path: $target_to_check ...${NC}"
                rm -rf "$target_to_check"
            else
                echo -e "${RED}❌ Deployment terminated by user.${NC}"
                exit 0
            fi
        fi
    fi
}

if [ "$CURRENT_WS_NAME" == "tm2_ws" ]; then
    handle_existing_directory "$WS_ROOT" "Workspace Root (tm2_ws)"
else
    handle_existing_directory "$TARGET_SRC_PATH" "Source Root Workspace Layout"
fi

mkdir -p "$TARGET_SRC_PATH"

# ==============================================================================
# 4. Deploy Techman Robot Repository (tm2_ros2)
# ==============================================================================
(
    cd "$TARGET_SRC_PATH" || exit 1
    if [ ! -d ".git" ]; then
        echo -e "${CYAN}📥 Cloning Techman Robot ROS 2 ($ROS_DISTRO_BRANCH) core apps...${NC}"
        if git clone "$REPO_URL" -b "$ROS_DISTRO_BRANCH" .; then
            sync
            echo -e "${GREEN}✅ Core drivers successfully deployed.${NC}"
        else
            echo -e "${RED}❌ Error: Git clone failed. Please check your connection.${NC}"
            exit 1
        fi
    else
        echo -e "${YELLOW}✨ Techman Robot repository already exists in $TARGET_SRC_PATH. Skipping clone.${NC}"
    fi
) || { echo -e "${RED}❌ Error: Core deployment failed.${NC}"; exit 1; }

# ==============================================================================
# 5a. Deploy Standalone Cobot Packages Inherited from Main Config Launcher
# ==============================================================================
sync_standalone_repo() {
    local repo_url="$1"
    local branch="$2"
    local target_dir="$3" 
    
    local leaf_dir="$(basename "$target_dir")"
    local workspace_src_base="$(dirname "$(dirname "$target_dir")")"

    if [ -d "$target_dir" ] && [ "$(ls -A "$target_dir" 2>/dev/null)" ]; then
        echo -e "${YELLOW}⚠️  [$target_dir] already exists and is verified. Skipping clone...${NC}"
        return 0
    fi

    local tmp_clone_dir="/tmp/tm_migration_$(date +%s%N)"
    
    echo "📥 Fetching standalone packages from [$repo_url] (Branch: $branch)..."
    if git clone "$repo_url" -b "$branch" --depth 1 "$tmp_clone_dir" 2>/dev/null; then
        echo "🚚 Extracting and merging profiles via Precision Asymmetric Routing Strategy..."
        local repo_name=$(basename "$repo_url")
        local pkg_name=""
        if [[ "$repo_name" == *"Description"* ]]; then pkg_name="tm_description"; fi
        if [[ "$repo_name" == *"Moveit2"* ]];     then pkg_name="tm_moveit"; fi
        if [[ "$repo_name" == *"Simulation"* ]]; then pkg_name="tm_gazebo"; fi

        local real_payload_path="${tmp_clone_dir}/${pkg_name}/${leaf_dir}"

        if [ -d "$real_payload_path" ]; then
            mkdir -p "${workspace_src_base}/${pkg_name}"
            mv "$real_payload_path" "$target_dir"
            sync
            echo -e "${GREEN}   -> Successfully migrated standalone package into perfect layout: $target_dir${NC}"
        else
            echo -e "${RED}   ❌ Error: Remote internal layout mismatches expected pattern for $pkg_name/${leaf_dir}${NC}"
            mkdir -p "$target_dir"
            cp -r "$tmp_clone_dir"/* "$target_dir/" 2>/dev/null
            sync
        fi
        rm -rf "$tmp_clone_dir"
    else
        echo -e "${RED}   ❌ Error: Failed to fetch standalone repo $repo_url${NC}"
        rm -rf "$tmp_clone_dir"
    fi
}

# ==============================================================================
# 5b. Deploy Standalone Cobot Packages Trigger Sequence
# ==============================================================================
if [ "$DEPLOY_STANDALONE" = true ]; then
    echo -e "${CYAN}⚙️  Deploying and Migrating Standalone Cobot Repositories...${NC}"

    if [ -n "$REPO_tm_description" ] && [ -n "$DIR_tm_description" ]; then
        sync_standalone_repo "$REPO_tm_description" "$BRANCH_tm_description" "$DIR_tm_description"
    fi

    if [ -n "$REPO_tm_moveit" ] && [ -n "$DIR_tm_moveit" ]; then
        sync_standalone_repo "$REPO_tm_moveit" "$BRANCH_tm_moveit" "$DIR_tm_moveit"
    fi

    if [ -n "$REPO_tm_gazebo" ] && [ -n "$DIR_tm_gazebo" ]; then
        sync_standalone_repo "$REPO_tm_gazebo" "$BRANCH_tm_gazebo" "$DIR_tm_gazebo"
    fi
else
    # echo -e "${YELLOW}⚙️  Skipping Standalone Cobot Repositories (Core Only Mode Powered by Switch)...${NC}"
    echo "============================================================================"
fi

# ==============================================================================
# 6. Finalized Summary Display
# ==============================================================================
# echo -e "${GREEN}🚀 Automated Workspace Deployment Pipeline Completed Successfully!${NC}"
echo "📂 Workspace Root : $WS_ROOT"
echo "📂 Source Folder  : $TARGET_SRC_PATH"
echo -e "${PURPLE}🔔 [Reminder: If deployed successfully, remember to rebuild your workspace.]${NC}"
# echo -e "${CYAN}   1. Switch to workspace root:${NC} cd $WS_ROOT"
# echo -e "${CYAN}   2. Clear old build files   :${NC} rm -rf build/ install/ log/"
# echo -e "${CYAN}   3. Install dependencies    :${NC} rosdep install --from-paths src --ignore-src -r -y"
# echo -e "${CYAN}   4. Recompile with release  :${NC} colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
# echo -e "${CYAN}   5. Source overlay env      :${NC} source install/setup.bash"
echo "============================================================================"
