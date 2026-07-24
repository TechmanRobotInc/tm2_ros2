#!/bin/bash
###########################################################################################
# Script Name: user_workspace_check.sh                                                    #
# Description: Techman Robot Deployment Loader Script - Check User Workspace Configuration#
# Version:     1.0.0                                                                      #
###########################################################################################
# ANSI Terminal Color Escape Codes
RED='\033[1;31m'
PURPLE='\033[1;35m'  # Purple Color for reminders
NC='\033[0m'        # No Color (Reset)

# Configuration file name
CONFIG_FILE="./user_workspace.txt"

# 1. Initialize with a default fallback path
WS_ROOT="$HOME/tm2_ws/tm2_ros2"

# 2. Check if the text configuration file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}⚠️  Warning: '$CONFIG_FILE' not found. Using default fallback path.${NC}"
else
    # 3. Parse the file line by line to extract WS_ROOT
    while IFS='=' read -r key value || [ -n "$key" ]; do
        # Clean up whitespace and carriage returns (\r)
        key=$(echo "$key" | tr -d '\r' | xargs)
        value=$(echo "$value" | tr -d '\r' | xargs)

        # Strictly skip comments and empty lines
        [[ "$key" =~ ^#.*$ ]] && continue
        [[ -z "$key" ]] && continue

        # Dynamically evaluate system variables like $HOME
        value=$(eval echo "$value")

        if [ "$key" == "WS_ROOT" ]; then
            # Remove any accidental trailing slash '/'
            WS_ROOT="${value%/}"
        fi
    done < "$CONFIG_FILE"
fi

# 4. Auto-verify and create the custom directory structure if missing
if [ ! -d "$WS_ROOT" ]; then
    echo "📁 Target directory does not exist. Creating path: $WS_ROOT"
    mkdir -p "$WS_ROOT"
fi

echo "============================================================================"
echo "🚀 Workspace Path Successfully Loaded!"
echo "📂 Target Location: $WS_ROOT"
echo -e "${PURPLE}🔔 [Reminder: If deployed successfully, remember to rebuild your workspace.]${NC}"
# echo -e "${PURPLE}   1. To clear old build files, run 'rm -rf build/ install/ log/' inside the Workspace Root.${NC}"
# echo -e "${PURPLE}   2. To recompile, run 'colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'.${NC}"
echo "============================================================================"
