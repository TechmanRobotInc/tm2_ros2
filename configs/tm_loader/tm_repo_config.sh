#!/bin/bash
###########################################################################################
# Script Name: tm_repo_config.sh                                                          #
# Description: Repository & Path Configuration Launcher                                   #
# Version:     1.0.0                                                                      #
###########################################################################################

# ==============================================================================
# Environment Initializer
# ==============================================================================
echo "============================================================================"
echo "🤖 Techman Robot ROS 2 Environment Initializer"
echo "============================================================================"

# Hardcoded execution mode selection: "1" = Check, "2" = Install
Default_CHOICE="1"

if [ "$Default_CHOICE" == "2" ]; then
    echo "⚙️  Action: Launching automated workspace installation workflow..."
    source ./user_workspace_install.sh 2>/dev/null || { 
        echo "❌ Error: 'user_workspace_install.sh' was not found in the current directory."; 
        exit 1; 
    }
else
    # Automatically falls back here since Default_CHOICE is hardcoded to "1"
    echo "🔍 Action: Launching standard environment verification workflow (Default)..."
    source ./user_workspace_check.sh 2>/dev/null || { 
        echo "❌ Error: 'user_workspace_check.sh' was not found in the current directory."; 
        exit 1; 
    }
fi

# ==============================================================================
# Remote Git Repository URLs (TM ROS2 Jazzy Jazzy or higher Supported)
# ==============================================================================
REPO_tm_description="https://github.com/TechmanRobotInc/TM_Cobots_ROS2_Description"
REPO_tm_moveit="https://github.com/TechmanRobotInc/TM_Cobots_ROS2_Moveit2"
REPO_tm_gazebo="https://github.com/TechmanRobotInc/TM_Cobots_ROS2_GZ_Simulation"

# ==============================================================================
# Remote Branch Configurations
# ==============================================================================
BRANCH_tm_description="main"
BRANCH_tm_moveit="main"
BRANCH_tm_gazebo="main"

# ==============================================================================
# Local destination parent directories (dynamically based on $WS_ROOT)
# ==============================================================================
DIR_tm_description="$WS_ROOT/tm_description/cobot_s"
DIR_tm_moveit="$WS_ROOT/tm_moveit/cobot_s"
DIR_tm_gazebo="$WS_ROOT/tm_gazebo/cobot_s"

