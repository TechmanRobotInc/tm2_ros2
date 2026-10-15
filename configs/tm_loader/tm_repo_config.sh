#!/bin/bash
###########################################################################################
# Script Name: tm_repo_config.sh
# 
# SPDX-FileCopyrightText: Copyright (c) 2026, TECHMAN ROBOT INC.
# SPDX-FileContributor: yh.wang
# SPDX-License-Identifier: BSD-3-Clause
# 
# Description: Repository & Path Configuration Launcher
# Version:     1.2.0
###########################################################################################

# ANSI Terminal Color Escape Codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[1;31m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color (Reset)

# ==============================================================================
# Environment Initializer
# ==============================================================================
echo "============================================================================"
echo "🤖 Techman Robot ROS 2 Environment Initializer"
echo "============================================================================"

# ==============================================================================
# Remote Git Repository URLs (TM ROS2 Jazzy or higher Supported)
# ==============================================================================
# Global Parameter Declarations
export REPO_tm_description="https://github.com/TechmanRobotInc/TM_Cobots_ROS2_Description"
export REPO_tm_moveit="https://github.com/TechmanRobotInc/TM_Cobots_ROS2_Moveit2"
export REPO_tm_gazebo="https://github.com/TechmanRobotInc/TM_Cobots_ROS2_GZ_Simulation"

export BRANCH_tm_description="main"
export BRANCH_tm_moveit="main"
export BRANCH_tm_gazebo="main"

# Parse configuration settings from the central workspace profile
if [ -f "./user_workspace.txt" ]; then
    echo -e "📂 Loading workspace configurations from ${CYAN}user_workspace.txt${NC}..."
    sed -i 's/\r$//' ./user_workspace.txt
    
    # Read variables cleanly to export them safely to the script environment
    while IFS='=' read -r key value || [ -n "$key" ]; do
        [[ "$key" =~ ^#.*$ ]] && continue
        [[ -z "$key" ]] && continue
        key=$(echo "$key" | xargs)
        value=$(echo "$value" | tr -d '\r' | xargs)
        value="${value/\$HOME/$HOME}"
        
        if [ "$key" == "WS_ROOT" ]; then export WS_ROOT="${value%/}"; fi
        if [ "$key" == "SRC_DIR" ]; then export SRC_DIR="${value%/}"; fi
    done < ./user_workspace.txt
else
    echo -e "${RED}❌ Error: 'user_workspace.txt' was not found in the current directory.${NC}"
    exit 1
fi

# Dynamically construct target system operational paths for nested components
export DIR_tm_description="$WS_ROOT/$SRC_DIR/tm_description/cobot_s"
export DIR_tm_moveit="$WS_ROOT/$SRC_DIR/tm_moveit/cobot_s"
export DIR_tm_gazebo="$WS_ROOT/$SRC_DIR/tm_gazebo/cobot_s"

# ==============================================================================
# Execution Mode Selection (Manual Toggle)
# ==============================================================================
# 💡 MANUAL CONTROL: Set to "1" for Environment Check, or "2" for Auto-Installation
Default_CHOICE="1"

echo "----------------------------------------------------------------------------"
if [ "$Default_CHOICE" == "2" ]; then
    echo -e "⚙️  Action: Launching automated workspace installation workflow..."
    source ./user_workspace_install.sh || { 
        echo -e "${RED}❌ Error: Failed to load or execute 'user_workspace_install.sh'.${NC}"; 
        exit 1; 
    }
else
    # Automatically falls back here since Default_CHOICE is set to "1"
    echo -e "🔍 Action: Launching standard environment verification workflow (Default)..."
    source ./user_workspace_check.sh || { 
        echo -e "${RED}❌ Error: Failed to load or execute 'user_workspace_check.sh'.${NC}"; 
        exit 1; 
    }
fi

