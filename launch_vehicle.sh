#!/bin/bash

# SmartDrive Vehicle Launch Dashboard
# This script launches an improved monitoring dashboard for all ROS2 nodes

set -euo pipefail

# Set ROS Domain ID
export ROS_DOMAIN_ID=42

# Colors for terminal output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Function to get network IP addresses
get_network_ips() {
    # Get all IPv4 addresses excluding localhost
    ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | head -5
}

# Parse command line arguments
AUTO_START=false
PORT=5000

while [[ $# -gt 0 ]]; do
    case $1 in
        --auto-start)
            AUTO_START=true
            shift
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --auto-start     Automatically start all nodes on launch"
            echo "  --port PORT      Web server port (default: 5000)"
            echo "  --help           Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check if virtual environment exists
if [ ! -d "$SCRIPT_DIR/dashboard" ]; then
    echo -e "${YELLOW}Virtual environment not found. Creating...${NC}"
    python3 -m venv "$SCRIPT_DIR/dashboard"
    source "$SCRIPT_DIR/dashboard/bin/activate"
    pip install --upgrade pip
    pip install -r "$SCRIPT_DIR/requirements.txt"
else
    source "$SCRIPT_DIR/dashboard/bin/activate"
fi

# Print startup message
echo -e "${BLUE}╔════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║     SmartDrive Vehicle Launch Dashboard    ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════╝${NC}"
echo ""

# Build command arguments
ARGS=""
if [ "$AUTO_START" = true ]; then
    ARGS="$ARGS --auto-start"
fi
ARGS="$ARGS --port $PORT --no-browser"

# Enable max utilization on ORIN (will prompt for password)
echo -e "${YELLOW}Enabling Jetson performance mode...${NC}"
sudo /usr/bin/jetson_clocks --fan 2>/dev/null || true

# Get network IPs
echo ""
echo -e "${GREEN}Starting dashboard on port ${PORT}...${NC}"
echo ""
echo -e "${CYAN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║              Dashboard Access URLs                     ║${NC}"
echo -e "${CYAN}╠════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║  Local:    ${MAGENTA}http://localhost:${PORT}${NC}                      "
echo -e "${CYAN}║                                                        ║${NC}"
echo -e "${CYAN}║  Network Access (use any of these from other devices): ║${NC}"

# Display all network IPs
for ip in $(get_network_ips); do
    printf "${CYAN}║  ${GREEN}➜ ${MAGENTA}http://%-45s${CYAN}║${NC}\n" "${ip}:${PORT} "
done

echo -e "${CYAN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop all nodes and exit${NC}"
echo ""

# Run the dashboard
python "$SCRIPT_DIR/launch_dashboard.py" $ARGS