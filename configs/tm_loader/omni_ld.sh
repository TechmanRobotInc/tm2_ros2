#!/bin/bash
###########################################################################################
# Script Name: omni_ld.sh
# 
# SPDX-FileCopyrightText: Copyright (c) 2026, TECHMAN ROBOT INC.
# SPDX-FileContributor: yh.wang
# SPDX-License-Identifier: BSD-3-Clause
# 
# Description: Techman Robot Deployment Loader Script - Omni Cascade Loader
# Version:     1.0.0
# Usage:       ./omni_ld.sh [MODEL1] [MODEL2...] [MODE (optional)] [-f (optional)]
###########################################################################################

# Set strict mode for robust error handling
set -o pipefail

if [ -f "version.txt" ]; then
    VERSION=$(cat version.txt)
else
    VERSION="Unknown (version.txt missing)"
fi

echo "📦 tm_loader version is : $VERSION"

# ANSI Terminal Color Escape Codes
RED='\033[1;31m'
GREEN='\033[1;32m'
NC='\033[0m' # No Color (Reset)

# ==============================================================================
# 1. Initialize Global Array and Signal Traps for Robust Cleanup
# ==============================================================================
TRACKED_TEMPS=()

cleanup() {
    # Iterates through all captured temporary directories and purges them safely.
    for temp_dir in "${TRACKED_TEMPS[@]}"; do
        if [ -d "$temp_dir" ]; then
            rm -rf "$temp_dir" 2>/dev/null
        fi
    done
}
# Trap signals to ensure cleanup triggers on exit, Ctrl+C, or termination
trap cleanup EXIT INT TERM

# ==============================================================================
# 2. Load Configuration Files
# ==============================================================================
# Source the companion environment script to map repository URLs and target folders.
source ./tm_repo_config.sh 2>/dev/null || { echo -e "${RED}❌ Error: tm_repo_config.sh not found.${NC}"; exit 1; }

# Initialize engine operational variables
MODELS=()
MODE="p1"
FORCE_ALL=false

# ==============================================================================
# 3. Parse Arguments (With Intelligent Package-to-Mode Auto-Mapping)
# ==============================================================================
for arg in "$@"; do
    case $arg in
        p1|tm_description) MODE="p1" ;;
        p2|tm_moveit)      MODE="p2" ;;
        p3|all|tm_gazebo)  MODE="p3" ;;
        -f|--force)        FORCE_ALL=true ;;
        -*)                echo -e "${RED}❌ Error: Unknown flag '$arg'${NC}"; exit 1 ;;
        *)                 MODELS+=("$arg") ;;
    esac
done

# ==============================================================================
# 4. Check Required Arguments & Validate Model Syntax (Strict Guardian)
# ==============================================================================
if [ ${#MODELS[@]} -eq 0 ]; then
    echo "💡 Usage: ./omni_ld.sh [MODEL1] [MODEL2...] [MODE / ROS_PACKAGE] [-f]"
    echo "  Modes & Package Syntax Mappings:"
    echo "    p1 / tm_description : Fetches Description Suite only"
    echo "    p2 / tm_moveit      : Cascades Description + MoveIt Configs"
    echo "    p3 / tm_gazebo      : Cascades Full-Stack (Description + MoveIt + Gazebo)"
    echo "  Flags:"
    echo "    -f / --force        : Bypasses interactive prompts with safe timestamped backups"
    exit 1
fi

# Full-Enclosure Regex: Custom-tailored for Techman Robot repository naming legacy
for model_check in "${MODELS[@]}"; do
    model_lower=$(echo "$model_check" | tr '[:upper:]' '[:lower:]')
    
    # 💡 Intelligent Filter Strategy:
    # 1. Immediately reject incomplete tm5 or tm7 standalone text (e.g., tm5, tm7, tm5_)
    if [[ "$model_lower" =~ ^tm[57]$ || "$model_lower" =~ ^tm[57]_$ ]]; then
        echo -e "${RED}❌ Validation Error: '$model_check' is not a recognized Techman Robot profile name.${NC}"
        echo "   -> For TM5/TM7, you must specify reach extension. Examples: tm5_700, tm5-900"
        exit 1
    fi

    # 2. General strict regex validation:
    #    - Matches S/SC/X/FT variants (e.g., tm5s, tm12s, tm30sc, tm12sxft)
    #    - Matches Large-Payload baseline models (e.g., tm12, tm14, tm20)
    #    - Matches Reach-specific models (e.g., tm5_700, tm5_900)
    if [[ ! "$model_lower" =~ ^(tm[0-9]+(s|sc|x|ft)+|tm[0-9]+|tm[0-9]+[-_][0-9]+(x|ft)*)$ ]]; then
        echo -e "${RED}❌ Validation Error: '$model_check' is not a recognized Techman Robot profile name.${NC}"
        echo "   -> Valid examples: tm5s, tm12, tm14, tm20, tm5_700, tm12sxft"
        exit 1
    fi
done

# ==============================================================================
# 5. Core Download Function
# ==============================================================================
do_download() {
    local model_input=$1
    local pkg=$2
    local suffix=$3
    
    local model=$(echo "$model_input" | tr '[:upper:]' '[:lower:]')
    local repo_var="REPO_$pkg"
    local dir_var="DIR_$pkg"
    local branch_var="BRANCH_$pkg"

    local repo_url=${!repo_var}
    local dest_parent=${!dir_var}

    local repo_branch=${!branch_var:-main}
    if [ "$repo_branch" = "master" ] || [ "$repo_branch" = "jazzy" ]; then
        repo_branch="main"
    fi

    if [ -z "$repo_url" ] || [ -z "$dest_parent" ]; then
        echo -e "${RED}  ❌ Error: Misconfigured package '$pkg' in config file.${NC}"
        return 1
    fi

    # --------------------------------------------------------------------------
    # Dynamic Series Classification & Baseline Parsing
    # --------------------------------------------------------------------------
    local SERIES_DIR="cobot"
    local RESOLVED_MODEL_NAME=""

    if [[ "$model" == *s* ]]; then
        SERIES_DIR="cobot_s"
        if [[ "$model" == *sc* ]]; then
            RESOLVED_MODEL_NAME=$(echo "$model" | sed -E 's/(.*sc).*/\1/')
        else
            RESOLVED_MODEL_NAME=$(echo "$model" | sed -E 's/(.*s).*/\1/')
        fi
    else
        SERIES_DIR="cobot"
        local CLEANED_NAME=$(echo "$model" | sed -E 's/^tm([0-9]+)x(.*)/tm\1\2/')
        RESOLVED_MODEL_NAME="${CLEANED_NAME//-/_}"
    fi

    # Adjust local parent directory dynamically to match series
    local DEST_PARENT_BASE="${dest_parent%/cobot*}"
    dest_parent="${DEST_PARENT_BASE}/${SERIES_DIR}"

    # --------------------------------------------------------------------------
    # Standard Strategy for tm_description, tm_moveit and tm_gazebo
    # --------------------------------------------------------------------------
    local folder_name="${RESOLVED_MODEL_NAME}${suffix}"
    local remote_path="$pkg/${SERIES_DIR}/$folder_name"
    local local_target="$dest_parent/$folder_name"

    if [[ -d "$local_target" && "$FORCE_ALL" = false ]]; then
        if [[ -t 0 ]]; then
            echo -n "⚠️  [Local Changes?] $folder_name already exists. Overwrite? (y/n): "
            read -r CONFIRM < /dev/tty
            if [[ ! "$CONFIRM" =~ ^[Yy]$ ]]; then
                echo "  ⏭️  [Skipping] Kept local modifications for $pkg."
                return 0
            fi
        else
            echo "  ⏭️  [Skipping] Non-interactive shell detected. Kept $folder_name."
            return 0
        fi
    fi

    echo "  📥 Pulling remote package: [$pkg] (Branch: $repo_branch)..."
    
    local ORIGINAL_DIR=$(pwd)

    mkdir -p "$dest_parent" && cd "$dest_parent" || { echo -e "${RED}  ❌ Error: Cannot access $dest_parent${NC}"; return 1; }
    
    local ABS_DEST_PARENT=$(pwd)
    # Generate unique, bulletproof temporary subdirectory name
    local UNIQUE_TEMP_NAME="temp_dl_${$}_${RANDOM}"
    local CURRENT_TEMP="$ABS_DEST_PARENT/$UNIQUE_TEMP_NAME"
    TRACKED_TEMPS+=("$CURRENT_TEMP")
    
    mkdir -p "$UNIQUE_TEMP_NAME" && cd "$UNIQUE_TEMP_NAME" || { echo -e "${RED}  ❌ Error: Cannot create workspace directory${NC}"; cd "$ORIGINAL_DIR"; return 1; }
    
    git init -q -b main
    git remote add origin "$repo_url"
    git config core.sparseCheckout true
    echo "$remote_path/*" >> .git/info/sparse-checkout
    git pull origin "$repo_branch" --depth 1 -q

    if [ -d "$remote_path" ]; then
        if [ -d "$local_target" ]; then
            local BACKUP_DIR="${local_target}_bak_$(date +%Y%m%d_%H%M%S)"
            mv "$local_target" "$BACKUP_DIR"
            echo "  📦 Note: Existing folder backed up to $(basename "$BACKUP_DIR")"
        fi
        
        cp -r "$remote_path" "$ABS_DEST_PARENT/"
        echo "  ✅ $folder_name downloaded successfully."
    else
        echo -e "${RED}  ❌ Failure: Remote path '$remote_path' not found in the repository.${NC}"
        cd "$ORIGINAL_DIR"
        rm -rf "$CURRENT_TEMP" 2>/dev/null
        return 1
    fi

    cd "$ABS_DEST_PARENT" && rm -rf "$UNIQUE_TEMP_NAME"
    cd "$ORIGINAL_DIR"
    
    if [ ${#TRACKED_TEMPS[@]} -gt 0 ]; then
        unset 'TRACKED_TEMPS[${#TRACKED_TEMPS[@]}-1]'
    fi
}

# ==============================================================================
# 6. Execute Batch Processing with Terminal Logs
# ==============================================================================
for M in "${MODELS[@]}"; do
    echo "===================================================="
    echo "🚀 Configuring Environment for Model: [ $M ] (Mode: $MODE)"
    echo "===================================================="
    
    # Phase 1: Description Setup (tmx_description Series)
    do_download "$M" "tm_description" "_description" || continue

    # Phase 2: MoveIt Setup (tmx_moveit Series)
    if [[ "$MODE" =~ p2|p3 ]]; then
        do_download "$M" "tm_moveit" "_moveit"
    fi

    # Phase 3: Gazebo Physics Setup (tmx_gazebo Series)
    if [[ "$MODE" == "p3" ]]; then
        do_download "$M" "tm_gazebo" "_gazebo"
    fi
done

echo -e "\n${GREEN}============================================================================"
echo "✨ All tasks executed!"
echo -e "============================================================================${NC}"
