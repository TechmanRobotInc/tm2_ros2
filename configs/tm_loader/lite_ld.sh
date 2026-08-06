#!/bin/bash
###########################################################################################
# Script Name: lite_ld.sh                                                                 #
# Description: Techman Robot Deployment Loader Script - Lite Download Wrapper             #
# Version:     1.1.0                                                                      #
# Usage:       ./lite_ld.sh [MODEL] [PACKAGE (optional)] [-f (optional)]                  #
###########################################################################################

# ANSI Terminal Color Escape Codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[1;31m'
NC='\033[0m' # No Color (Reset)

# ==============================================================================
# 1. Load Configuration File
# ==============================================================================
CONFIG_FILE="./tm_repo_config.sh"

if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}[ERROR] '$CONFIG_FILE' not found. Please ensure it exists in the current directory.${NC}"
    exit 1
fi
source "$CONFIG_FILE"

# ==============================================================================
# 2. Initialize Variables
# ==============================================================================
MODEL=""
PACKAGE=""
FORCE_OVERWRITE=false
BRANCH="main"

# ==============================================================================
# 3. Parse Arguments (Supports flexible order and detects invalid options)
# ==============================================================================
while [[ $# -gt 0 ]]; do
    case "$1" in
        -f)
            FORCE_OVERWRITE=true
            shift
            ;;
        -*)
            echo -e "${RED}[ERROR] Unknown option: $1${NC}"
            exit 1
            ;;
        *)
            if [ -z "$MODEL" ]; then
                MODEL="$1"
            elif [ -z "$PACKAGE" ]; then
                PACKAGE="$1"
            else
                echo -e "${RED}[ERROR] Too many arguments provided: $1${NC}"
                exit 1
            fi
            shift
            ;;
    esac
done

# Apply default package if omitted
PACKAGE=${PACKAGE:-"tm_description"}

# ==============================================================================
# 4. Validate Required Arguments and Display Usage
# ==============================================================================
if [ -z "$MODEL" ]; then
    echo "[USAGE] ./lite_ld.sh [MODEL] [PACKAGE (optional)] [-f]"
    echo "  Example 1 (Default):  ./lite_ld.sh tm5s              -> Downloads tm_description"
    echo "  Example 2 (Specific): ./lite_ld.sh tm5s tm_gazebo    -> Downloads tm_gazebo"
    echo "  Example 3 (Force):    ./lite_ld.sh tm5s -f           -> Forces overwrite"
    echo "  Example 4 (MoveIt):   ./lite_ld.sh tm12s tm_moveit   -> Downloads full tm12s_moveit bundle"
    exit 1
fi

# ==============================================================================
# 5. Dynamically Resolve Repository URL and Destination Parent Directory
# ==============================================================================
REPO_VAR="REPO_$PACKAGE"
DIR_VAR="DIR_$PACKAGE"
REPO_URL=${!REPO_VAR}
DEST_PARENT=${!DIR_VAR}

if [ -z "$REPO_URL" ] || [ -z "$DEST_PARENT" ]; then
    echo -e "${RED}[ERROR] Unsupported or misconfigured package '$PACKAGE'. Please verify '$CONFIG_FILE'.${NC}"
    exit 1
fi

# ==============================================================================
# 6. Determine Remote Subdirectory Suffix and Series Path Based on Model & Package
# ==============================================================================
case "$PACKAGE" in
    "tm_description") SUFFIX="_description" ;;
    "tm_gazebo")      SUFFIX="_gazebo" ;;
    "tm_moveit")      SUFFIX="_moveit" ;;
    *)                SUFFIX="" ;;
esac

# Convert input to lowercase to prevent case-sensitivity mismatches
MODEL_LOWER=$(echo "$MODEL" | tr '[:upper:]' '[:lower:]')

# 1. Classify series and parse baseline model name
if [[ "$MODEL_LOWER" == *s* ]]; then
    # [Case A: S and SC Series with Variants] (e.g., tm5s, tm5sft, tm12sx, tm12sxft)
    SERIES_DIR="cobot_s"
    
    # Extract baseline model by stripping variant suffixes
    if [[ "$MODEL_LOWER" == *sc* ]]; then
        RESOLVED_MODEL_NAME=$(echo "$MODEL_LOWER" | sed -E 's/(.*sc).*/\1/')
    else
        RESOLVED_MODEL_NAME=$(echo "$MODEL_LOWER" | sed -E 's/(.*s).*/\1/')
    fi
else
    # [Case B & C: Regular Cobot Series with Variants] (e.g., tm12x, tm5x-900, tm5-900)
    SERIES_DIR="cobot"
    
    # Strip 'x' variant flags inside or at the end of the string
    CLEANED_NAME=$(echo "$MODEL_LOWER" | sed -E 's/^tm([0-9]+)x(.*)/tm\1\2/')
    
    # Convert hyphens to underscores for standard compliant naming (e.g., tm5-900 -> tm5_900)
    RESOLVED_MODEL_NAME="${CLEANED_NAME//-/_}"
fi

# 2. Update remote download paths and local target directory names
REMOTE_SUB_PATH="$PACKAGE/${SERIES_DIR}/${RESOLVED_MODEL_NAME}${SUFFIX}"
TARGET_DIR="${RESOLVED_MODEL_NAME}${SUFFIX}"

# 3. Correct local parent directory dynamically for ALL package types
if [ -n "$DEST_PARENT" ]; then
    DEST_PARENT_BASE="${DEST_PARENT%/cobot*}"
    DEST_PARENT="${DEST_PARENT_BASE}/${SERIES_DIR}"
fi

echo -e "${GREEN}[INFO] Input Model: $MODEL -> Resolved Baseline Target: $RESOLVED_MODEL_NAME ($PACKAGE)${NC}"
echo -e "[INFO] Destination parent directory: $DEST_PARENT"
echo -e "[INFO] Target deployment directory: $TARGET_DIR"
if [ "$FORCE_OVERWRITE" = true ]; then
    echo -e "${YELLOW}[INFO] Force overwrite is ENABLED.${NC}"
fi

# ==============================================================================
# 7. Safe Directory Navigation and Automated Cleanup Setup
# ==============================================================================
mkdir -p "$DEST_PARENT" && cd "$DEST_PARENT" || exit 1

# Capture absolute path of the destination parent folder for bulletproof cleanup
ABS_DEST_PARENT="$(pwd)"
TEMP_DIR="temp_dl_${$}_${RANDOM}"

# Trap guarantees deletion of the temporary folder using absolute paths upon exit
trap 'rm -rf "$ABS_DEST_PARENT/$TEMP_DIR" 2>/dev/null' EXIT

mkdir -p "$TEMP_DIR" && cd "$TEMP_DIR" || exit 1

# ==============================================================================
# 8. Configure Git Sparse Checkout & Fetch Package (Modern Approach)
# ==============================================================================
git init -q -b "$BRANCH"
git remote add origin "$REPO_URL"

git sparse-checkout init --cone
git sparse-checkout set "$REMOTE_SUB_PATH"

echo "[INFO] Fetching files from remote repository (Shallow Clone)..."
git pull origin "$BRANCH" --depth 1 -q

# ==============================================================================
# 9. Verify, Handle Overwrites, and Deploy the Downloaded Directory
# ==============================================================================
if [ -d "$REMOTE_SUB_PATH" ]; then
    # Check if target directory already exists in the destination folder
    if [ -d "../$TARGET_DIR" ]; then
        if [ "$FORCE_OVERWRITE" = true ]; then
            echo "[INFO] Force deleting existing directory '$TARGET_DIR'..."
            rm -rf "../$TARGET_DIR"
        else
            # Redirected stdin from /dev/tty to ensure read works in piped/automated environments
            read -p "[PROMPT] Target directory '$TARGET_DIR' already exists. Overwrite? (y/n): " ANSWER < /dev/tty
            case "$ANSWER" in
                [yY][eE][sS]|[yY])
                    echo "[INFO] Overwriting existing directory..."
                    rm -rf "../$TARGET_DIR"
                    ;;
                *)
                    echo "[INFO] Operation canceled by user. Keeping original files."
                    exit 0
                    ;;
            esac
        fi
    fi

    # --------------------------------------------------------------------------
    # FLAT DEPLOYMENT SEQUENCE (Resolves deep path layout structures)
    # --------------------------------------------------------------------------
    echo "[INFO] Deploying extracted profile into target environment..."
    
    # Safely migrate the deeply nested structure back up into the user layout
    mv "$REMOTE_SUB_PATH" "../$TARGET_DIR"
    sync

    echo -e "${GREEN}[SUCCESS] $TARGET_DIR is deployed and ready for use.${NC}"
else
    echo -e "${RED}[ERROR] Remote path '$REMOTE_SUB_PATH' not found in repository. Deployment failed.${NC}"
    exit 1
fi
