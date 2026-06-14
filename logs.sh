#!/bin/bash
#
# TerraSLAM Log Viewer
# Usage: ./logs.sh [component] [options]
#
# Components: all, publisher, relay, slam, supervisor
# Options: 
#   -f, --follow     Follow logs in real-time (tail -f)
#   -e, --errors     Show only error logs
#   -n NUM           Show last NUM lines (default: 50)
#   -h, --help       Show this help
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Defaults
COMPONENT="all"
FOLLOW=false
ERRORS_ONLY=false
LINES=50
CONTAINER="TerraSLAM"
SOCKET="/tmp/supervisor.sock"
SUPERVISORCTL="/usr/bin/supervisorctl -s unix://${SOCKET}"

# Helper functions
print_header() {
    echo -e "\n${CYAN}════════════════════════════════════════${NC}"
    echo -e "${CYAN}  $1${NC}"
    echo -e "${CYAN}════════════════════════════════════════${NC}"
}

print_section() {
    echo -e "\n${BLUE}▶ $1${NC}"
}

error_msg() {
    echo -e "${RED}✗ $1${NC}" >&2
}

success_msg() {
    echo -e "${GREEN}✓ $1${NC}"
}

warn_msg() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        publisher|relay|slam|supervisor|all)
            COMPONENT="$1"
            shift
            ;;
        -f|--follow)
            FOLLOW=true
            shift
            ;;
        -e|--errors)
            ERRORS_ONLY=true
            shift
            ;;
        -n)
            LINES="$2"
            shift 2
            ;;
        -h|--help)
            echo "TerraSLAM Log Viewer"
            echo ""
            echo "Usage: $0 [component] [options]"
            echo ""
            echo "Components:"
            echo "  all        - Show logs for all components (default)"
            echo "  publisher  - Image publisher only"
            echo "  relay      - TerraSLAM Relay only"
            echo "  slam       - ORB-SLAM3 core only"
            echo "  supervisor - Supervisor daemon logs"
            echo ""
            echo "Options:"
            echo "  -f, --follow     Follow logs in real-time (tail -f)"
            echo "  -e, --errors     Show only error logs (*.err.log)"
            echo "  -n NUM           Show last NUM lines (default: 50)"
            echo "  -h, --help       Show this help"
            echo ""
            echo "Examples:"
            echo "  $0                          # Show all logs (last 50 lines)"
            echo "  $0 slam -f                  # Follow SLAM logs in real-time"
            echo "  $0 relay -e                 # Show only relay errors"
            echo "  $0 publisher -n 100 -e      # Last 100 lines of publisher errors"
            exit 0
            ;;
        *)
            error_msg "Unknown option: $1"
            echo "Use -h for help"
            exit 1
            ;;
    esac
done

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    error_msg "Container '$CONTAINER' is not running!"
    echo "Start it with: docker compose up -d"
    exit 1
fi

# Check if supervisord socket exists
if ! docker exec "$CONTAINER" test -e "${SOCKET}" 2>/dev/null; then
    warn_msg "Supervisor socket not found at ${SOCKET}"
    warn_msg "Trying alternative paths..."
    # Try to find the socket
    SOCKET=$(docker exec "$CONTAINER" find /tmp /var/run -name "*.sock" 2>/dev/null | grep -i supervisor | head -1 || echo "")
    if [ -z "$SOCKET" ]; then
        error_msg "Could not find supervisor socket. Is supervisord running?"
        echo "Check with: docker exec $CONTAINER ps aux | grep supervisor"
        exit 1
    fi
    SUPERVISORCTL="/usr/bin/supervisorctl -s unix://${SOCKET}"
    success_msg "Found socket at: $SOCKET"
fi

# Function to show logs for a component
show_component_logs() {
    local name="$1"
    local display_name="$2"
    local out_log="/var/log/supervisor/${name}.out.log"
    local err_log="/var/log/supervisor/${name}.err.log"
    
    print_header "$display_name"
    
    # Check supervisor status
    print_section "Supervisor Status"
    docker exec "$CONTAINER" $SUPERVISORCTL status "$name" 2>&1 || warn_msg "Process not managed by supervisor"
    
    # Show output logs
    if [ "$ERRORS_ONLY" = false ]; then
        print_section "Output Log ($out_log)"
        if [ "$FOLLOW" = true ]; then
            docker exec "$CONTAINER" tail -f "$out_log" 2>&1
        else
            docker exec "$CONTAINER" tail -n "$LINES" "$out_log" 2>&1 || warn_msg "No output log found"
        fi
    fi
    
    # Show error logs
    if [ "$ERRORS_ONLY" = true ] || [ "$FOLLOW" = false ]; then
        print_section "Error Log ($err_log)"
        if [ "$FOLLOW" = true ] && [ "$ERRORS_ONLY" = true ]; then
            docker exec "$CONTAINER" tail -f "$err_log" 2>&1
        else
            local err_content
            err_content=$(docker exec "$CONTAINER" tail -n "$LINES" "$err_log" 2>&1 || echo "")
            if [ -n "$err_content" ] && [ "$err_content" != "" ]; then
                echo -e "${RED}$err_content${NC}"
            else
                success_msg "No errors found"
            fi
        fi
    fi
}

# Function to show ROS2 info for a component
show_ros2_info() {
    local component="$1"
    local display_name="$2"
    
    print_section "ROS2 Topics Related to $display_name"
    docker exec "$CONTAINER" /bin/bash -l -c "source /opt/ros/humble/setup.bash && ros2 topic list" 2>&1 | grep -E \
        "$(case $component in
            publisher) echo "image_raw|camera" ;;
            relay) echo "relay|gps|status" ;;
            slam) echo "slam|pose|map|keypoint|frame" ;;
            *) echo ".*" ;;
        esac)" || warn_msg "No matching topics found"
    
    # Show topic rates for key topics
    if [ "$FOLLOW" = false ] && [ "$ERRORS_ONLY" = false ]; then
        case $component in
            publisher)
                print_section "Image Publisher Rate"
                docker exec "$CONTAINER" /bin/bash -l -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /camera/image_raw" 2>&1 | head -5 || warn_msg "Could not measure rate"
                ;;
            slam)
                print_section "SLAM Pose Rate"
                docker exec "$CONTAINER" /bin/bash -l -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic hz /slam/pose" 2>&1 | head -5 || warn_msg "SLAM pose topic not active"
                ;;
        esac
    fi
}

# Main logic
case $COMPONENT in
    all)
        print_header "TerraSLAM System Logs"
        echo -e "${CYAN}Container: ${CONTAINER}${NC}"
        echo -e "${CYAN}Time: $(date)${NC}"
        echo -e "${CYAN}Mode: ${FOLLOW:+Follow }${ERRORS_ONLY:+Errors Only }${LINES:+Last $LINES lines}${NC}"
        
        # Show supervisor overview
        print_section "All Supervisor Processes"
        docker exec "$CONTAINER" $SUPERVISORCTL status 2>&1
        
        # Show logs for each component
        show_component_logs "image_publisher" "📷 Image Publisher"
        show_ros2_info "publisher" "Image Publisher"
        
        show_component_logs "relay" "🔗 TerraSLAM Relay"
        show_ros2_info "relay" "Relay"
        
        show_component_logs "slam_core" "🗺️ ORB-SLAM3 Core"
        show_ros2_info "slam" "SLAM"
        
        # Show supervisor daemon logs
        print_header "🔧 Supervisor Daemon Logs"
        print_section "Supervisord Log"
        docker exec "$CONTAINER" tail -n "$LINES" /var/log/supervisor/supervisord.log 2>&1 || warn_msg "No supervisord log found"
        ;;
        
    publisher)
        show_component_logs "image_publisher" "📷 Image Publisher"
        show_ros2_info "publisher" "Image Publisher"
        ;;
        
    relay)
        show_component_logs "relay" "🔗 TerraSLAM Relay"
        show_ros2_info "relay" "Relay"
        ;;
        
    slam)
        show_component_logs "slam_core" "🗺️ ORB-SLAM3 Core"
        show_ros2_info "slam" "SLAM"
        ;;
        
    supervisor)
        print_header "🔧 Supervisor Daemon"
        print_section "Supervisor Status (All)"
        docker exec "$CONTAINER" $SUPERVISORCTL status 2>&1
        
        print_section "Supervisord Log"
        if [ "$FOLLOW" = true ]; then
            docker exec "$CONTAINER" tail -f /var/log/supervisor/supervisord.log 2>&1
        else
            docker exec "$CONTAINER" tail -n "$LINES" /var/log/supervisor/supervisord.log 2>&1
        fi
        ;;
esac

echo -e "\n${CYAN}════════════════════════════════════════${NC}"
if [ "$FOLLOW" = false ]; then
    echo -e "${GREEN}Log view complete.${NC}"
    echo -e "Tip: Use ${YELLOW}-f${NC} to follow logs in real-time"
    echo -e "Tip: Use ${YELLOW}-e${NC} to show only errors"
fi
echo -e "${CYAN}════════════════════════════════════════${NC}"
