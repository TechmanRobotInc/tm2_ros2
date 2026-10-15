#!/bin/bash
###########################################################################################
# Script Name: print_ver.sh
# 
# SPDX-FileCopyrightText: Copyright (c) 2026, TECHMAN ROBOT INC.
# SPDX-FileContributor: yh.wang
# SPDX-License-Identifier: BSD-3-Clause
# 
# Description: Print Techman Robot ROS 2 Application Versions
# Version:     1.0.0
# Usage:       ./print_ver.sh [MODEL] [PACKAGE/all (optional)]
###########################################################################################
# ANSI Terminal Color Escape Codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[1;31m'
BLUE='\033[1;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color (Reset)

# ==============================================================================
# How to Check TM Apps Version.
# ==============================================================================
# cd your_workspace(WS_ROOT), build, source your workspace.
# WS_ROOT$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# WS_ROOT$ source ./install/setup.bash
# WS_ROOT$ cd tm2_ros2/configs/tm_loader
# WS_ROOT/tm2_ros2/configs/tm_loader$ chmod +x ./print_ver.sh
# WS_ROOT/tm2_ros2/configs/tm_loader$ ./print_ver.sh tm12s all
# ==============================================================================

# ==============================================================================
# 0. Help Instruction Menu Trigger (-h / --help)
# ==============================================================================
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo -e "${BLUE}============================================================================${NC}"
    echo -e "🤖 ${CYAN}Techman Robot ROS 2 Version Printer Tool${NC}"
    echo -e "${BLUE}============================================================================${NC}"
    echo -e "${YELLOW}Usage:${NC}"
    echo -e "  $0 [MODEL] [PACKAGE/all (optional)]"
    echo ""
    echo -e "${YELLOW}Allowed Packages for Filtering:${NC}"
    echo -e "  tm_driver, tm_image, tm_inspect, tm_inspecp, tm_description, tm_moveit_config, tm_gazebo"
    echo ""
    echo -e "${YELLOW}Display Modes & Examples:${NC}"
    echo -e "  ${GREEN}1. Advanced Only Mode (Default)${NC}"
    echo -e "     $ $0 tm5s"
    echo -e "     -> Prints only the Advanced App block for the specified model."
    echo ""
    echo -e "  ${GREEN}2. Full Diagnostic Mode (Explicit 'all')${NC}"
    echo -e "     $ $0 tm5s all"
    echo -e "     -> Prints full setup banners, tm_loader version, Basic apps, and Advanced apps."
    echo ""
    echo -e "  ${GREEN}3. Precise Package Filter Mode${NC}"
    echo -e "     $ $0 tm5s tm_description"
    echo -e "     -> Strips all boxes and prints only the single targeted package row."
    echo ""
    echo -e "  ${GREEN}4. Quick Workspace Status Mode${NC}"
    echo -e "     $ $0 all"
    echo -e "     -> Auto-redirects to full diagnostic mode using the default model (tm12s)."
    echo -e "${BLUE}============================================================================${NC}"
    exit 0
fi

# ==============================================================================
# 1. Load Workspace Configuration from TXT File
# ==============================================================================
CONFIG_FILE="./user_workspace.txt"
WS_ROOT=""

if [ -f "$CONFIG_FILE" ]; then
    while IFS='=' read -r key value; do
        [[ "$key" =~ ^[[:space:]]*# ]] && continue
        [[ -z "$key" ]] && continue
        key=$(echo "$key" | xargs)
        value=$(echo "$value" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' -e 's/^"//' -e 's/"$//' -e "s/^'//" -e "s/'$//")
        eval "export $key=\"$value\""
    done < "$CONFIG_FILE"
fi

TARGET_WS_DIR="${WS_ROOT:-$(pwd)}"

if [ ! -f "${TARGET_WS_DIR}/install/setup.bash" ]; then
    echo -e "${YELLOW}[WARN] The workspace at '${TARGET_WS_DIR}' has not been built yet.${NC}"
    exit 1
fi

source "${TARGET_WS_DIR}/install/setup.bash"

# ==============================================================================
# 2. Extract and Validate Arguments (With Position Auto-Correction)
# ==============================================================================
# Automatically adjust positions if the user types "all" as the first argument
if [ "$1" == "all" ] || [ "$1" == "ALL" ]; then
    COBOT_MODEL="tm12s"
    TARGET_FILTER="all"
else
    COBOT_MODEL="${1:-tm12s}"
    TARGET_FILTER="${2:-default}" 
fi

FILTER_LOWER=$(echo "$TARGET_FILTER" | tr '[:upper:]' '[:lower:]')
VALID_PACKAGES=("default" "all" "tm_driver" "tm_image" "tm_mod_urdf" "tm_inspect" "demo" "image_sub" "tm_description" "tm_moveit_config" "tm_gazebo")

if [[ ! " ${VALID_PACKAGES[@]} " =~ " ${FILTER_LOWER} " ]]; then
    echo -e "${RED}[ERROR] Invalid package filter '$TARGET_FILTER'.${NC}"
    echo "Allowed options: ${VALID_PACKAGES[*]:1}"
    exit 1
fi

# ==============================================================================
# 3. Parse Robot Model String
# ==============================================================================
MODEL_LOWER=$(echo "$COBOT_MODEL" | tr '[:upper:]' '[:lower:]')

if [[ "$MODEL_LOWER" =~ tm[0-9]+s ]]; then
    SERIES_DIR="cobot_s"
    RESOLVED_MODEL_NAME=$(echo "$MODEL_LOWER" | grep -oE '^tm[0-9]+s(c)?')
else
    SERIES_DIR="cobot"
    CLEANED_NAME=$(echo "$MODEL_LOWER" | sed -E 's/^tm([0-9]+)x(.*)/tm\1\2/')
    RESOLVED_MODEL_NAME="${CLEANED_NAME//-/_}"
fi

# ============================================================================
# 4. Display Initializer & Tool Banners (ONLY if filter is explicitly "all")
# ============================================================================
if [ "$FILTER_LOWER" == "all" ]; then
    echo "============================================================================"
    echo "🤖 Techman Robot ROS 2 Environment Initializer"
    echo "🤖 Configuration File: $CONFIG_FILE"
    echo "🤖 Target Workspace : ${TARGET_WS_DIR}"
    echo "🤖 Input Model       : $COBOT_MODEL"
    echo "🤖 Baseline Target  : $RESOLVED_MODEL_NAME ($SERIES_DIR)"
    echo "🤖 Package Filter    : $TARGET_FILTER"
    echo "============================================================================"

    if [ -f "version.txt" ]; then
        VERSION=$(cat version.txt | xargs)
    else
        VERSION="Unknown (version.txt missing)"
    fi

    echo "============================================================================"
    echo -e "📦 tm_loader version is : ${GREEN}$VERSION${NC}"
    echo "============================================================================"
fi

# ============================================================================
# Helper Functions for Package Parsing
# ============================================================================
get_pkg_version() {
    local target_pkg="$1"
    local xml_line
    xml_line=$(ros2 pkg xml "$target_pkg" 2>/dev/null | grep version)
    if [ -n "$xml_line" ]; then
        echo "$xml_line" | sed -E 's/.*<version>(.*)<\/version>.*/\1/' | xargs
    else
        echo "NOT_INSTALLED"
    fi
}

print_row() {
    local display_name="$1"
    local actual_pkg="$2"
    local version_str
    version_str=$(get_pkg_version "$actual_pkg")
    
    if [ "$version_str" == "NOT_INSTALLED" ]; then
        printf "  %-40s: ${RED}[NOT INSTALLED]${NC}\n" "$display_name"
    else
        printf "  %-40s: ${GREEN}v%s${NC}\n" "$display_name" "$version_str"
    fi
}

print_advanced_row() {
    local suffix="$1"          
    local preferred_pkg="$2"   
    local alternative_pkg="${RESOLVED_MODEL_NAME}_${suffix}"
    
    if ros2 pkg xml "$preferred_pkg" &>/dev/null; then
        print_row "${RESOLVED_MODEL_NAME}_${suffix} ($preferred_pkg)" "$preferred_pkg"
    elif ros2 pkg xml "$alternative_pkg" &>/dev/null; then
        print_row "$alternative_pkg" "$alternative_pkg"
    else
        printf "  %-40s: ${RED}[NOT INSTALLED]${NC}\n" "${RESOLVED_MODEL_NAME}_${suffix}"
    fi
}

# ============================================================================
# 5. Execute Output Filtering Logic
# ============================================================================

# --- Section A: Basic App Stack ---
if [[ "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "tm_driver" || "$FILTER_LOWER" == "tm_image" || "$FILTER_LOWER" == "tm_inspect" || "$FILTER_LOWER" == "tm_mod_urdf" || "$FILTER_LOWER" == "demo" || "$FILTER_LOWER" == "image_sub" || "$FILTER_LOWER" == "tm_msgs" ]]; then
    if [ "$FILTER_LOWER" == "all" ]; then
        echo "============================================================================"
        echo "🤖 Techman Robot ROS 2 Basic App Versions"
        echo "============================================================================"
    fi
    
    # Standard Core Apps
    [[ "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "tm_driver" ]]  && print_row "tm_driver" "tm_driver"
    [[ "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "tm_image" ]]   && print_row "tm_image" "tm_image"
    [[ "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "tm_inspect" ]] && print_row "tm_inspect" "tm_inspect"
    
    # Custom Utility Apps (Mod / Demo Targets)
    [[ "$FILTER_LOWER" == "tm_mod_urdf" ]] && print_row "tm_mod_urdf" "tm_mod_urdf"
    [[ "$FILTER_LOWER" == "demo" ]]        && print_row "demo" "demo"
    
    # Handle Vision / Communication Messaging Fallback
    if [[ "$FILTER_LOWER" == "all" ]]; then
        print_row "tm_msgs" "tm_msgs"
        if ros2 pkg xml image_sub &>/dev/null; then
            print_row "image_sub" "image_sub"
        fi
    elif [[ "$FILTER_LOWER" == "image_sub" ]]; then
        print_row "image_sub" "image_sub"
    elif [[ "$FILTER_LOWER" == "tm_msgs" ]]; then
        print_row "tm_msgs" "tm_msgs"
    fi
fi

# --- Section B: Advanced App Stack ---
if [[ "$FILTER_LOWER" == "default" || "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "tm_description" || "$FILTER_LOWER" == "tm_moveit_config" || "$FILTER_LOWER" == "tm_gazebo" ]]; then
    test_pkg="${RESOLVED_MODEL_NAME}_description"
    if ! ros2 pkg xml "tm_description" &>/dev/null && ! ros2 pkg xml "$test_pkg" &>/dev/null; then
        if [[ "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "default" ]]; then
            echo "============================================================================"
            echo "🤖 Techman Robot ROS 2 Advanced App Versions"
            echo "============================================================================"
            echo -e "  ${RED}[WARN] Model '${COBOT_MODEL}' specific packages are not found in this workspace.${NC}"
            echo -e "  ${BLUE}[HINT] Available robot description models installed on your system:${NC}"
            ros2 pkg list 2>/dev/null | grep "_description" | sed 's/_description//' | grep -v "moveit" | sed 's/^/         - /'
            echo "============================================================================"
        fi
    else
        if [[ "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "default" ]]; then
            echo "============================================================================"
            echo "🤖 Techman Robot ROS 2 Advanced App Versions"
            echo "============================================================================"
        fi
        
        [[ "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "default" || "$FILTER_LOWER" == "tm_description" ]] && \
            print_advanced_row "description" "tm_description"
            
        [[ "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "default" || "$FILTER_LOWER" == "tm_moveit_config" ]] && \
            print_advanced_row "moveit_config" "tm_moveit_config"
            
        [[ "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "default" || "$FILTER_LOWER" == "tm_gazebo" ]] && \
            print_advanced_row "gazebo" "tm_gazebo"
            
        if [[ "$FILTER_LOWER" == "all" || "$FILTER_LOWER" == "default" ]]; then
            echo "============================================================================"
        fi
    fi
fi
