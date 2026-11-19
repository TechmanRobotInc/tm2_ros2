#!/bin/bash
###########################################################################################
# Script Name: print_ver.sh
# 
# SPDX-FileCopyrightText: Copyright (c) 2026, TECHMAN ROBOT INC.
# SPDX-FileContributor: yh.wang
# SPDX-License-Identifier: BSD-3-Clause
# 
# Description: Print Techman Robot ROS 2 Application Versions with Sub-tree Layout
# Version:     1.1.0
# Usage:       ./print_ver.sh [MODEL] [PACKAGE/all (optional)]
###########################################################################################
# ANSI Terminal Color Escape Codes
RED='\033[1;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[1;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color (Reset)

VALID_PACKAGES=("default" "all" "tm_driver" "tm_image" "tm_mod_urdf" "tm_inspect" "tm_msgs" "demo" "image_sub" "tm_description" "tm_moveit_config" "tm_gazebo")

# ==============================================================================
# 0. Help Instruction Menu Trigger (-h / --help)
# ==============================================================================
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo -e "${BLUE}============================================================================${NC}"
    echo -e "[Apps]   ${CYAN}Techman Robot ROS 2 Version Printer Tool${NC}"
    echo -e "${BLUE}============================================================================${NC}"
    echo -e "${YELLOW}Usage:${NC}"
    echo -e "  $0 [MODEL] [PACKAGE/all (optional)]"
    echo -e "  $0 [PACKAGE] (Will use default model: tm12s)"
    echo ""
    echo -e "${YELLOW}Allowed Packages for Filtering:${NC}"
    echo -e "  ${VALID_PACKAGES[*]}"
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
    echo -e "     $ $0 tm_driver  (Auto-corrects model to tm12s)"
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
    echo -e "${YELLOW}[WARN]   The workspace at '${TARGET_WS_DIR}' has not been built yet.${NC}"
    exit 1
fi

source "${TARGET_WS_DIR}/install/setup.bash"

# ==============================================================================
# 2. Extract and Validate Arguments (With Position Auto-Correction)
# ==============================================================================
DEFAULT_MODEL="tm12s"

if [ "$1" == "all" ] || [ "$1" == "ALL" ]; then
    export COBOT_MODEL="$DEFAULT_MODEL"
    export TARGET_FILTER="all"
elif [ $# -eq 1 ]; then
    ARG_LOWER=$(echo "$1" | tr '[:upper:]' '[:lower:]')
    if [[ " ${VALID_PACKAGES[@]} " =~ " ${ARG_LOWER} " ]]; then
        export COBOT_MODEL="$DEFAULT_MODEL"
        export TARGET_FILTER="$1"
    else
        export COBOT_MODEL="$1"
        export TARGET_FILTER="default"
    fi
else
    export COBOT_MODEL="${1:-$DEFAULT_MODEL}"
    export TARGET_FILTER="${2:-default}" 
fi

export FILTER_LOWER=$(echo "$TARGET_FILTER" | tr '[:upper:]' '[:lower:]')

if [[ ! " ${VALID_PACKAGES[@]} " =~ " ${FILTER_LOWER} " ]]; then
    echo -e "${RED}[ERROR]  Invalid package filter '$TARGET_FILTER'.${NC}"
    echo "Allowed options: ${VALID_PACKAGES[*]}"
    return 1 2>/dev/null || exit 1
fi

# ==============================================================================
# 3. Parse Robot Model String (Official Compatibility Matrix)
# ==============================================================================
MODEL_LOWER=$(echo "$COBOT_MODEL" | tr '[:upper:]' '[:lower:]')
MODEL_CLEANED="${MODEL_LOWER//-/_}"

# Define valid Xacro Model names based on official hardware configuration matrix
VALID_XACRO_MODELS=(
    # TM AI Cobot S Series
    "tm3s" "tm3sx"
    "tm5s" "tm5sx" "tm5sft" "tm5sxft"
    "tm6s" "tm6sx" "tm6sft" "tm6sxft"
    "tm7s" "tm7sx" "tm7sft" "tm7sxft"
    "tm12s" "tm12sx" "tm12sft" "tm12sxft"
    "tm14s" "tm14sx" "tm14sft" "tm14sxft"
    "tm16s" "tm16sx" "tm16sft" "tm16sxft"
    "tm20s" "tm20sx" "tm20sft" "tm20sxft"
    "tm25s" "tm25sx" "tm25sft" "tm25sxft"
    "tm30s" "tm30sx" "tm30sft" "tm30sxft"
    "tm30scft" "tm30scxft"
    "tm45s" "tm45sx"
    # TM AI Cobot Series
    "tm5_700" "tm5x_700"
    "tm5_900" "tm5x_900"
    "tm12" "tm12x"
    "tm14" "tm14x"
    "tm16" "tm16x"
    "tm20" "tm20x"
)

# Strict frontend verification filter against ghost/invalid models
IS_VALID_MODEL=false
for valid_model in "${VALID_XACRO_MODELS[@]}"; do
    if [ "$MODEL_CLEANED" == "$valid_model" ]; then
        IS_VALID_MODEL=true
        break
    fi
done

if [ "$IS_VALID_MODEL" = false ]; then
    echo -e "${RED}[ERROR]  Invalid or unsupported Techman Robot model '${COBOT_MODEL}'.${NC}"
    echo -e "${YELLOW}[HINT]   Please check the model name (e.g., tm5-900, tm12sxft, tm30scft).${NC}"
    return 1 2>/dev/null || exit 1
fi

# Deconstruct valid configurations for structural package pooling separation
if [[ "$MODEL_CLEANED" =~ tm[0-9]+s ]]; then
    export SERIES_DIR="cobot_s"
    export RESOLVED_MODEL_NAME="$MODEL_CLEANED"
    
    # Route specialized TM30SC setups back to baseline TM30S meshes
    if [[ "$MODEL_CLEANED" =~ tm30sc ]]; then
        export DESCRIPTION_MODEL_NAME="tm30s"
    else
        export DESCRIPTION_MODEL_NAME=$(echo "$MODEL_CLEANED" | grep -oE '^tm[0-9]+s')
    fi
else
    export SERIES_DIR="cobot"
    export RESOLVED_MODEL_NAME="$MODEL_CLEANED"
    
    # Standardize Legacy X-variants back to shared description package limits
    if [[ "$MODEL_CLEANED" =~ x_ ]]; then
        export DESCRIPTION_MODEL_NAME="${MODEL_CLEANED//x_/_.}"
    else
        export DESCRIPTION_MODEL_NAME="$MODEL_CLEANED"
    fi
fi

# ============================================================================
# 4. Display Initializer & Tool Banners (ONLY if filter is explicitly "all")
# ============================================================================
if [ "$FILTER_LOWER" == "all" ]; then
    if [ -f "version.txt" ]; then
        VERSION=$(xargs < version.txt)
    else
        VERSION="Unknown (version.txt missing)"
    fi

    echo "============================================================================"
    echo -e "[Script] tm_loader version is : ${GREEN}$VERSION${NC}"
    echo "============================================================================"
    echo "⚙️  [INIT]   Techman Robot ROS 2 Environment Initializer"
    echo "📄 [CONF]   Configuration File: $CONFIG_FILE"
    echo "📂 [PATH]   Target Workspace : ${TARGET_WS_DIR}"
    echo "⚙️  [SPEC]   Input Model       : $COBOT_MODEL"
    echo "📂 [Series] Baseline Target  : $RESOLVED_MODEL_NAME ($SERIES_DIR)"
    # echo "🧹 [FLTR]   Package Filter    : $TARGET_FILTER"
fi

# ==============================================================================
# Print Local Workspace & Remote GitHub Git Version
# ==============================================================================
if [ "$FILTER_LOWER" == "all" ]; then
    GIT_REPO_DIR="${TARGET_WS_DIR}/${SRC_DIR:-tm2_ros2}"

    if [ -d "$GIT_REPO_DIR/.git" ]; then
        CURRENT_DIR=$(pwd)
        cd "$GIT_REPO_DIR" || exit
        
        LOCAL_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
        LOCAL_HASH=$(git rev-parse --short HEAD 2>/dev/null)
        LOCAL_MSG=$(git log -1 --format="%s" 2>/dev/null)
        LOCAL_VERSION=$(git describe --tags --abbrev=0 2>/dev/null || echo "$LOCAL_MSG" | awk '{print $1}')
        
        if [ -n "$(git status --porcelain)" ]; then
            LOCAL_STATUS="Modified"
            LOCAL_COLOR="$RED"
        else
            LOCAL_STATUS="Clean"
            LOCAL_COLOR="$GREEN"
        fi

        REMOTE_NAME=$(git config branch."${LOCAL_BRANCH}".remote 2>/dev/null || echo "origin")
        git fetch "$REMOTE_NAME" "${LOCAL_BRANCH}" &>/dev/null
        
        if [ $? -eq 0 ]; then
            REMOTE_HASH=$(git rev-parse --short "$REMOTE_NAME"/"${LOCAL_BRANCH}" 2>/dev/null)
            REMOTE_MSG=$(git log -1 --format="%s" "$REMOTE_NAME"/"${LOCAL_BRANCH}" 2>/dev/null)
            REMOTE_VERSION=$(git describe --tags --abbrev=0 "$REMOTE_NAME"/"${LOCAL_BRANCH}" 2>/dev/null || echo "$REMOTE_MSG" | awk '{print $1}')
            
            if [ "$LOCAL_HASH" == "$REMOTE_HASH" ]; then
                REMOTE_STATUS="Up-to-Date"
                REMOTE_COLOR="$GREEN"
            else
                REMOTE_STATUS="Update Available"
                REMOTE_COLOR="$YELLOW"
            fi   
        else
            REMOTE_HASH="Unknown"
            REMOTE_STATUS="Connection Error"
            REMOTE_COLOR='\033[1;35m'
            REMOTE_MSG="Network offline or remote connection failed."
            REMOTE_VERSION="Unknown"
        fi
        
        if [ "$LOCAL_HASH" == "$REMOTE_HASH" ]; then
            COMPARE_BANNER="${BLUE}[Local == On GitHub]${NC} Techman Robot ROS 2 Jazzy Core tm2_ros2 Version"
        else
            COMPARE_BANNER="${YELLOW}[Local != On GitHub]${NC} Techman Robot ROS 2 Jazzy Core tm2_ros2 Version"
        fi    

        cd "$CURRENT_DIR" || exit

        echo "============================================================================"
        echo -e "${COMPARE_BANNER} COMPARE"
        echo "============================================================================"        
        
        # Output Section: Local Workspace
        # echo "============================================================================"
        # echo "⚖️ Techman Robot ROS 2 Jazzy Core tm2_ros2 Version - User Workspace"
        # echo "============================================================================"
        # printf "  %-40s: %s (%s, \033[%bm%s\033[0m)\n" "Workspace Git Status" "${LOCAL_BRANCH}" "${LOCAL_HASH}" "${LOCAL_COLOR}" "${LOCAL_STATUS}"
        # printf "  %-40s: %s\n" "Last Commit Message" "${LOCAL_MSG}"
        printf "  %-25s : %-10s (%b%s%b) | %-15s : %b%s%b\n" \
               "User Workspace Git Status" "${LOCAL_BRANCH}" "${LOCAL_COLOR}" "${LOCAL_HASH}" "${NC}" \
               "local version" "${GREEN}" "${LOCAL_VERSION}" "${NC}"        
        # Output Section: Remote GitHub
        # echo "----------------------------------------------------------------------------"
        # echo "⚖️ TM ROS 2 Jazzy tm2_ros2 Core Version - On GitHub"
        # echo "----------------------------------------------------------------------------"
        # printf "  %-40s: %s (%s, \033[%bm%s\033[0m)\n" "GitHub Git Status" "${LOCAL_BRANCH}" "${REMOTE_HASH}" "${REMOTE_COLOR}" "${REMOTE_STATUS}"
        # printf "  %-40s: %s\n" "Latest Commit Message" "${REMOTE_MSG}"
        printf "  %-25s : %-10s (%b%s%b) | %-15s : %b%s%b\n" \
               "Newest GitHub Git Status" "${LOCAL_BRANCH}" "${REMOTE_COLOR}" "${REMOTE_HASH}" "${NC}" \
               "remote version" "${GREEN}" "${REMOTE_VERSION}" "${NC}"
    else
        echo "============================================================================"
        echo -e "🔍 ${YELLOW}[WARN] Git repository not found at '${GIT_REPO_DIR}'. Skipping Git status sync.${NC}"
        echo "============================================================================"
    fi
fi

# ============================================================================
# 5: Execute Output Filtering Logic for Basic and Advanced ROS Apps
# ============================================================================

if [ -z "$FILTER_LOWER" ] || [ -z "$RESOLVED_MODEL_NAME" ]; then
    echo -e "\033[1;31m[ERROR] Required parameters not found. Execute step 2->4 first.\033[0m"
    exit 1
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
    local version_str="$3"
    [ -z "$version_str" ] && version_str=$(get_pkg_version "$actual_pkg")
    
    if [ "$version_str" == "NOT_INSTALLED" ]; then
        printf "  %-60s : ${RED}[NOT INSTALLED]${NC}\n" "$display_name"
    else
        printf "  %-60s : ${GREEN}v%s${NC}\n" "$display_name" "$version_str"
    fi
}

print_advanced_row() {
    local suffix="$1"          
    local preferred_pkg="$2"   
    local alternative_pkg="${RESOLVED_MODEL_NAME}_${suffix}"
    local version_str
    
    version_str=$(get_pkg_version "$preferred_pkg")
    if [ "$version_str" != "NOT_INSTALLED" ]; then
        print_row "${RESOLVED_MODEL_NAME}_${suffix} ($preferred_pkg)" "$preferred_pkg" "$version_str"
        return
    fi
    
    version_str=$(get_pkg_version "$alternative_pkg")
    if [ "$version_str" != "NOT_INSTALLED" ]; then
        print_row "$alternative_pkg" "$alternative_pkg" "$version_str"
    else
        printf "  %-40s: ${RED}[NOT INSTALLED]${NC}\n" "${RESOLVED_MODEL_NAME}_${suffix}"
    fi
}

# ============================================================================
# 5. Execute Output Filtering Logic
# ============================================================================
# --- Section A: Basic App Stack ---
if [[ "$FILTER_LOWER" =~ ^(all|tm_driver|tm_image|tm_inspect|tm_mod_urdf|demo|image_sub|tm_msgs)$ ]]; then
    if [ "$FILTER_LOWER" == "all" ]; then
        echo "============================================================================"
        echo -e "🤖 ${YELLOW}[Local]${NC} Techman Robot ROS 2 Basic App Versions"
        echo "============================================================================"
    fi
    
    [[ "$FILTER_LOWER" =~ ^(all|tm_driver)$ ]]  && print_row "tm_driver" "tm_driver"
    [[ "$FILTER_LOWER" =~ ^(all|tm_image)$ ]]   && print_row "tm_image" "tm_image"
    [[ "$FILTER_LOWER" =~ ^(all|tm_inspect)$ ]] && print_row "tm_inspect" "tm_inspect"
    [[ "$FILTER_LOWER" == "tm_mod_urdf" ]]       && print_row "tm_mod_urdf" "tm_mod_urdf"
    [[ "$FILTER_LOWER" == "demo" ]]              && print_row "demo" "demo"
    
    if [[ "$FILTER_LOWER" == "all" ]]; then
        print_row "tm_msgs" "tm_msgs"
        img_sub_v=$(get_pkg_version "image_sub")
        if [ "$img_sub_v" != "NOT_INSTALLED" ]; then
            print_row "image_sub" "image_sub" "$img_sub_v"
        fi
    elif [[ "$FILTER_LOWER" == "image_sub" ]]; then
        print_row "image_sub" "image_sub"
    elif [[ "$FILTER_LOWER" == "tm_msgs" ]]; then
        print_row "tm_msgs" "tm_msgs"
    fi
fi

# --- Section B: Advanced App Stack ---
if [[ "$FILTER_LOWER" =~ ^(default|all|tm_description|tm_moveit_config|tm_gazebo)$ ]]; then
    test_pkg="${DESCRIPTION_MODEL_NAME}_description"
    desc_v1=$(get_pkg_version "tm_description")
    desc_v2=$(get_pkg_version "$test_pkg")
    
    # Setup tree branch label tags with an elegant 6-space indentation
    SUB_TREE_TAG=""
    if [ "$RESOLVED_MODEL_NAME" != "$DESCRIPTION_MODEL_NAME" ]; then
        SUB_TREE_TAG="      ${CYAN}└── [Target Model: ${RESOLVED_MODEL_NAME}]${NC}"
    fi

    if [[ "$FILTER_LOWER" =~ ^(all|default)$ ]]; then
        echo "============================================================================"
        echo -e "🤖 ${YELLOW}[Local]${NC} Techman Robot ROS 2 Advanced App Versions"
        echo "============================================================================"
    fi

    if [ "$desc_v1" == "NOT_INSTALLED" ] && [ "$desc_v2" == "NOT_INSTALLED" ]; then
        if [[ "$FILTER_LOWER" =~ ^(all|default)$ ]]; then
            echo -e "  [WARN]   Model '${COBOT_MODEL}' specific packages are not found in this workspace."
            echo -e "  [HINT]   Available robot description models installed on your system:"
            ros2 pkg list 2>/dev/null | grep "_description" | sed 's/_description//' | grep -v "moveit" | sed 's/^/           - /'
            echo "============================================================================"
        fi
    else
        # ====================================================================
        # Fixed 60-character column layout ensuring true vertical alignment
        # ====================================================================
        
        # 1. Display Description Package Module
        if [[ "$FILTER_LOWER" =~ ^(all|default|tm_description)$ ]]; then
            if [ "$desc_v1" != "NOT_INSTALLED" ]; then
                printf "  %-60s : " "${DESCRIPTION_MODEL_NAME}_description"
                echo -e "${GREEN}v$desc_v1${NC}"
            else
                printf "  %-60s : " "${test_pkg}"
                echo -e "${GREEN}v$desc_v2${NC}"
            fi
            [ -n "$SUB_TREE_TAG" ] && echo -e "$SUB_TREE_TAG"
        fi
        
        # 2. Display MoveIt Configuration Module
        if [[ "$FILTER_LOWER" =~ ^(all|default|tm_moveit_config)$ ]]; then
            preferred_pkg="tm_moveit_config"
            alternative_pkg="${RESOLVED_MODEL_NAME}_moveit_config"
            version_str=$(get_pkg_version "$preferred_pkg")
            
            if [ "$version_str" != "NOT_INSTALLED" ]; then
                printf "  %-60s : " "${RESOLVED_MODEL_NAME}_moveit_config ($preferred_pkg)"
                echo -e "${GREEN}v$version_str${NC}"
            else
                version_alt=$(get_pkg_version "$alternative_pkg")
                printf "  %-60s : " "${alternative_pkg}"
                if [ "$version_alt" == "NOT_INSTALLED" ]; then
                    echo -e "${RED}[NOT INSTALLED]${NC}"
                else
                    echo -e "${GREEN}v$version_alt${NC}"
                fi
            fi
        fi
        
        # 3. Display Gazebo Simulation Module
        if [[ "$FILTER_LOWER" =~ ^(all|default|tm_gazebo)$ ]]; then
            gazebo_pkg="${DESCRIPTION_MODEL_NAME}_gazebo"
            gazebo_v=$(get_pkg_version "$gazebo_pkg")
            
            printf "  %-60s : " "${DESCRIPTION_MODEL_NAME}_gazebo"
            if [ "$gazebo_v" == "NOT_INSTALLED" ]; then
                echo -e "${RED}[NOT INSTALLED]${NC}"
            else
                echo -e "${GREEN}v$gazebo_v${NC}"
            fi
            [ -n "$SUB_TREE_TAG" ] && echo -e "$SUB_TREE_TAG"
        fi
            
        if [[ "$FILTER_LOWER" =~ ^(all|default)$ ]]; then
            echo "============================================================================"
        fi
    fi
fi













