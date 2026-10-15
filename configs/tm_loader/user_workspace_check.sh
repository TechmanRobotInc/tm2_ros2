#!/bin/bash
###########################################################################################
# Script Name: user_workspace_check.sh
# 
# SPDX-FileCopyrightText: Copyright (c) 2026, TECHMAN ROBOT INC.
# SPDX-FileContributor: yh.wang
# SPDX-License-Identifier: BSD-3-Clause
# 
# Description: Techman Robot Deployment Loader Script - Verify Workspace Configuration
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
# 1. Initialize Fallback Environments
# ==============================================================================
SRC_DIR="tm2_ros2"
WS_ROOT="$HOME/tm2_ws"

if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${YELLOW}⚠️  Warning: Configuration file '$CONFIG_FILE' not found. Using fallbacks.${NC}"
else
    # 2. Parse the configuration file securely
    while IFS='=' read -r key value || [ -n "$key" ]; do
        # Clean up whitespace and carriage returns (\r)
        key=$(echo "$key" | tr -d '\r' | xargs)
        value=$(echo "$value" | tr -d '\r' | xargs)

        # Skip comments and empty lines
        [[ "$key" =~ ^#.*$ ]] && continue
        [[ -z "$key" ]] && continue

        # Securely expand variables without using unsafe 'eval'
        value="${value/\$HOME/$HOME}"

        if [ "$key" == "WS_ROOT" ]; then WS_ROOT="${value%/}"; fi
        if [ "$key" == "SRC_DIR" ]; then SRC_DIR="${value%/}"; fi
    done < "$CONFIG_FILE"
fi

# Resolve the complete physical path where the package layout sits
TARGET_SRC_PATH="${WS_ROOT}/${SRC_DIR}"

# ==============================================================================
# 3. Path Validation and Directory Layout Verification
# ==============================================================================
echo "============================================================================"
echo "🔍 Validating Techman Robot Workspace Directories..."
echo "============================================================================"

# Ensure the root workspace exists
if [ ! -d "$WS_ROOT" ]; then
    echo -e "${RED}⚠️  Workspace root missing. Creating base directory: $WS_ROOT${NC}"
    mkdir -p "$WS_ROOT"
fi

# Ensure the source tree directory exists
if [ ! -d "$TARGET_SRC_PATH" ]; then
    echo -e "${YELLOW}⚠️  Source space folder missing. Creating layout: $TARGET_SRC_PATH${NC}"
    mkdir -p "$TARGET_SRC_PATH"
fi

# Functional check for nested cobot profiles (e.g., cobot_s layout validation)
[ -z "$DIR_tm_description" ] && DIR_tm_description="${TARGET_SRC_PATH}/tm_description/cobot_s"

if [ -d "$DIR_tm_description" ] && [ "$(ls -A "$DIR_tm_description" 2>/dev/null)" ]; then
    echo -e "${GREEN}✅ Profile layout structure verified: $DIR_tm_description${NC}"
else
    echo -e "ℹ️  Note: Subdirectory tree 'tm_description/cobot_s' is empty and ready for migration packages."
fi

echo "============================================================================"
# echo -e "${GREEN}🚀 Workspace Configuration Checked and Verified!${NC}"
echo "📂 Workspace Root : $WS_ROOT"
echo "📂 Source Folder  : $TARGET_SRC_PATH"
echo -e "${PURPLE}🔔 [Reminder: If deployed successfully, remember to rebuild your workspace.]${NC}"
# echo -e "${CYAN}   1. Switch to workspace root:${NC} cd $WS_ROOT"
# echo -e "${CYAN}   2. Clear old build files   :${NC} rm -rf build/ install/ log/"
# echo -e "${CYAN}   3. Install dependencies    :${NC} rosdep install --from-paths src --ignore-src -r -y"
# echo -e "${CYAN}   4. Recompile with release  :${NC} colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
# echo -e "${CYAN}   5. Source overlay env      :${NC} source install/setup.bash"
echo "============================================================================"
