#!/bin/bash
#
# TerraSLAM Component Controller
# Usage: ./control.sh <component> <action> [options]
#
# Components: slam, relay, publisher, publisher:folder, publisher:realsense, gps_bridge, all
# Actions: start, stop, restart, status, logs, kill, mode, path
#

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Configuration
CONTAINER="TerraSLAM"
SOCKET="/tmp/supervisor.sock"
SUPERVISORCTL="/usr/bin/supervisorctl -s unix://${SOCKET}"
SUPERVISOR_CONF_PATH="/etc/supervisor/conf.d/terraslam.conf"
FOLDER_CHANGE_LOG="/tmp/terraslam_folder_changes.log"

# Component mappings (component -> supervisor program name)
declare -A COMPONENTS
COMPONENTS=(
    ["slam"]="slam_core"
    ["relay"]="relay"
    ["publisher"]="image_publisher"
    ["publisher:folder"]="image_publisher_folder"
    ["publisher:realsense"]="image_publisher_realsense"
    ["gps_bridge"]="gps_bridge"
)

# Log file name mapping (program name -> actual log filename)
declare -A LOG_NAMES
LOG_NAMES=(
    ["slam_core"]="slam_core"
    ["relay"]="relay"
    ["image_publisher_folder"]="publisher_folder"
    ["image_publisher_realsense"]="publisher_realsense"
    ["gps_bridge"]="gps_bridge"
)

# Process patterns for killing orphaned processes
declare -A PROCESS_PATTERNS
PROCESS_PATTERNS=(
    ["slam"]="orb_slam3|slam_core|rgbd|mono"
    ["relay"]="relay.py"
    ["publisher"]="image_publish.py"
    ["publisher:folder"]="image_publish.py.*folder"
    ["publisher:realsense"]="realsense.py"
    ["gps_bridge"]="gps_bridge_node.py"
)

# Publisher mode tracking
PUBLISHER_MODE_FILE="/tmp/terraslam_publisher_mode"
DEFAULT_PUBLISHER_MODE="folder"

# ============================================================================
# Helper Functions
# ============================================================================

print_header() {
    echo -e "\n${CYAN}════════════════════════════════════════${NC}"
    echo -e "${CYAN}  $1${NC}"
    echo -e "${CYAN}════════════════════════════════════════${NC}"
}

success_msg() { echo -e "${GREEN}✓ $1${NC}"; }
error_msg() { echo -e "${RED}✗ $1${NC}" >&2; }
warn_msg() { echo -e "${YELLOW}⚠ $1${NC}"; }
info_msg() { echo -e "${BLUE}▶ $1${NC}"; }
section_msg() { echo -e "\n${MAGENTA}--- $1 ---${NC}"; }

check_container() {
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
        error_msg "Container '$CONTAINER' is not running!"
        echo "Start it with: docker compose up -d"
        exit 1
    fi
}

show_usage() {
    cat << EOF
TerraSLAM Component Controller

Usage: $0 <component> <action> [options]

Components:
  slam                    - ORB-SLAM3 Core
  relay                   - TerraSLAM Relay
  publisher               - Image Publisher (active mode)
  publisher:folder        - Publisher in folder mode
  publisher:realsense     - Publisher in RealSense mode
  gps_bridge              - GPS to Controller Bridge (UART/I2C/USB)
  all                     - All components

Actions:
  start                   - Start the component
  stop                    - Stop the component
  restart                 - Restart the component
  status                  - Show component status
  logs                    - Show recent logs (last 50 lines)
  kill                    - Force kill all related processes
  mode [folder|realsense] - Switch publisher mode
  path [new_path]         - View/set VIDEO_FOLDER for publisher:folder

Examples:
  $0 slam start
  $0 publisher:realsense start
  $0 publisher mode realsense
  $0 publisher:folder path                    # Show current folder
  $0 publisher:folder path /data/videos/      # Change folder and restart
  $0 gps_bridge status
  $0 gps_bridge logs
  $0 all restart

Note: ROS 2 sends all logs (INFO/WARN/ERROR) to stderr.
      In log output, look under "ROS 2 Logs (stderr)" for messages.
EOF
}

get_supervisor_name() {
    local comp="$1"
    echo "${COMPONENTS[$comp]:-$comp}"
}

get_publisher_mode() {
    local mode
    mode=$(docker exec "$CONTAINER" cat "$PUBLISHER_MODE_FILE" 2>/dev/null || echo "$DEFAULT_PUBLISHER_MODE")
    echo "$mode"
}

set_publisher_mode() {
    local new_mode="$1"
    docker exec "$CONTAINER" /bin/bash -c "echo '$new_mode' > $PUBLISHER_MODE_FILE"
}

run_supervisorctl() {
    local action="$1"
    local component="$2"
    docker exec "$CONTAINER" $SUPERVISORCTL "$action" "$component" 2>&1
}

kill_processes() {
    local comp="$1"
    local pattern="${PROCESS_PATTERNS[$comp]}"
    
    if [ -z "$pattern" ]; then
        warn_msg "No process pattern defined for $comp"
        return
    fi
    
    info_msg "Finding processes matching: $pattern"
    
    local pids
    pids=$(docker exec "$CONTAINER" /bin/bash -c "
        ps aux | grep -E '$pattern' | grep -v grep | grep -v 'grep -E' | awk '{print \$2}'
    " 2>/dev/null || echo "")
    
    if [ -n "$pids" ]; then
        info_msg "Found PIDs: $pids"
        docker exec "$CONTAINER" /bin/bash -c "kill -9 $pids" 2>/dev/null || true
        sleep 2
        
        local remaining
        remaining=$(docker exec "$CONTAINER" /bin/bash -c "
            ps aux | grep -E '$pattern' | grep -v grep | wc -l
        " 2>/dev/null || echo "0")
        
        if [ "$remaining" -eq 0 ]; then
            success_msg "All $comp processes killed!"
        else
            warn_msg "$remaining process(es) still running, trying force kill..."
            docker exec "$CONTAINER" /bin/bash -c "
                ps aux | grep -E '$pattern' | grep -v grep | awk '{print \$2}' | xargs kill -9
            " 2>/dev/null || true
        fi
    else
        info_msg "No running processes found for $comp"
    fi
}

filter_ros2_logs() {
    grep -E "^\[INFO\]|^\[WARN\]|^\[ERROR\]|^\[DEBUG\]|rclpy|ros2|image_publisher" || true
}

# ============================================================================
# Publisher Folder Path Functions
# ============================================================================

get_video_folder() {
    docker exec "$CONTAINER" /bin/bash -c "
        grep -E '^environment=' '$SUPERVISOR_CONF_PATH' 2>/dev/null | \
        grep -oP 'VIDEO_FOLDER=\"\K[^\"]+' | head -1 || echo ''
    " 2>/dev/null
}

set_video_folder() {
    local new_path="$1"
    
    if ! docker exec "$CONTAINER" test -d "$new_path"; then
        error_msg "Directory '$new_path' does not exist inside container!"
        return 1
    fi
    
    local escaped_new="${new_path//\//\\/}"
    
    docker exec "$CONTAINER" /bin/bash -c "
        cp '$SUPERVISOR_CONF_PATH' '${SUPERVISOR_CONF_PATH}.bak' &&
        sed -i '/\\[program:image_publisher_folder\\]/,/^\\[/ {
            /^environment=/ s/VIDEO_FOLDER=\"[^\"]*\"/VIDEO_FOLDER=\"$escaped_new\"/
        }' '$SUPERVISOR_CONF_PATH'
    " || {
        error_msg "Failed to update config! Restoring backup..."
        docker exec "$CONTAINER" cp "${SUPERVISOR_CONF_PATH}.bak" "$SUPERVISOR_CONF_PATH" 2>/dev/null || true
        return 1
    }
    
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    docker exec "$CONTAINER" /bin/bash -c "
        echo '[$timestamp] VIDEO_FOLDER: \"$new_path\"' >> '$FOLDER_CHANGE_LOG'
    "
    
    success_msg "Config updated: VIDEO_FOLDER → $new_path"
}

handle_publisher_folder_path() {
    local new_path="${1:-}"
    
    print_header "Publisher Folder Path"
    
    local current_path
    current_path=$(get_video_folder)
    
    if [ -z "$current_path" ]; then
        warn_msg "VIDEO_FOLDER not found in config!"
        echo "Check: $SUPERVISOR_CONF_PATH inside container"
        return 1
    fi
    
    echo -e "${CYAN}Current path:${NC} ${GREEN}$current_path${NC}"
    
    if [ -n "$new_path" ]; then
        echo ""
        info_msg "Changing to: $new_path"
        
        local prog_name="${COMPONENTS[publisher:folder]}"
        run_supervisorctl "stop" "$prog_name" 2>/dev/null || true
        sleep 1
        
        if set_video_folder "$new_path"; then
            docker exec "$CONTAINER" $SUPERVISORCTL reread >/dev/null 2>&1 || true
            docker exec "$CONTAINER" $SUPERVISORCTL update "$prog_name" >/dev/null 2>&1 || \
            docker exec "$CONTAINER" $SUPERVISORCTL start "$prog_name"
            
            success_msg "Publisher restarted with new folder!"
            
            sleep 2
            if run_supervisorctl "status" "$prog_name" 2>/dev/null | grep -q "RUNNING"; then
                success_msg "✓ Process running"
            else
                warn_msg "⚠ Process may have failed to start — check logs with: $0 publisher:folder logs"
            fi
        else
            error_msg "Failed to update path. Process stopped. Run 'start' manually after fixing."
            return 1
        fi
    else
        echo ""
        echo "To change path:"
        echo "  $0 publisher:folder path /new/path/to/videos/"
        echo ""
        echo "Tips:"
        echo "  • Path must exist inside container"
        echo "  • Use absolute paths"
        echo "  • No trailing spaces in path"
    fi
}

# ============================================================================
# Publisher Mode Functions
# ============================================================================

show_publisher_mode() {
    local current_mode
    current_mode=$(get_publisher_mode)
    
    print_header "Publisher Mode"
    echo -e "${CYAN}Current mode:${NC} ${GREEN}$current_mode${NC}"
    echo ""
    echo "Available modes:"
    echo "  folder      - Publish pre-recorded images from directory"
    echo "  realsense   - Publish live frames from Intel RealSense D415i"
    echo ""
    echo "Switch mode:"
    echo "  $0 publisher mode folder"
    echo "  $0 publisher mode realsense"
}

switch_publisher_mode() {
    local new_mode="$1"
    
    if [ "$new_mode" != "folder" ] && [ "$new_mode" != "realsense" ]; then
        error_msg "Invalid mode: $new_mode"
        echo "Valid modes: folder, realsense"
        return 1
    fi
    
    local current_mode
    current_mode=$(get_publisher_mode)
    
    if [ "$current_mode" = "$new_mode" ]; then
        info_msg "Publisher already in $new_mode mode"
        return 0
    fi
    
    print_header "Switching Publisher Mode: $current_mode → $new_mode"
    
    local current_prog="${COMPONENTS[publisher:$current_mode]}"
    info_msg "Stopping $current_mode mode ($current_prog)..."
    run_supervisorctl "stop" "$current_prog" || true
    sleep 2
    
    kill_processes "publisher:$current_mode"
    sleep 1
    
    set_publisher_mode "$new_mode"
    info_msg "Mode updated to: $new_mode"
    
    local new_prog="${COMPONENTS[publisher:$new_mode]}"
    info_msg "Starting $new_mode mode ($new_prog)..."
    run_supervisorctl "start" "$new_prog"
    
    success_msg "Publisher switched to $new_mode mode!"
}

# ============================================================================
# Status & Logging Functions
# ============================================================================

show_status() {
    local comp="$1"
    
    if [ "$comp" = "publisher" ] && [ "${2:-}" = "mode" ]; then
        show_publisher_mode
        return
    fi
    
    if [ "$comp" = "publisher" ] && [ "${2:-}" = "path" ]; then
        handle_publisher_folder_path
        return
    fi
    
    local supervisor_name
    if [[ "$comp" == publisher:* ]]; then
        supervisor_name=$(get_supervisor_name "$comp")
        local mode="${comp#publisher:}"
        print_header "Publisher ($mode mode) Status"
    elif [ "$comp" = "publisher" ]; then
        local active_mode
        active_mode=$(get_publisher_mode)
        supervisor_name="${COMPONENTS[publisher:$active_mode]}"
        print_header "Publisher Status (active: $active_mode)"
    else
        supervisor_name=$(get_supervisor_name "$comp")
        print_header "$(echo "$comp" | tr '[:lower:]' '[:upper:]') Status"
    fi
    
    run_supervisorctl "status" "$supervisor_name"
    
    local pattern="${PROCESS_PATTERNS[$comp]:-${PROCESS_PATTERNS[publisher]}}"
    local count
    count=$(docker exec "$CONTAINER" /bin/bash -c "
        ps aux | grep -E '$pattern' | grep -v grep | wc -l
    " 2>/dev/null || echo "0")
    
    if [ "$count" -gt 0 ]; then
        warn_msg "⚠ $count orphaned process(es) still running!"
        echo "Run: ./control.sh ${comp%%:*} kill"
    else
        success_msg "✓ No orphaned processes"
    fi
    
    if [[ "$comp" == *publisher* ]]; then
        echo ""
        section_msg "ROS2 Topics"
        docker exec "$CONTAINER" /bin/bash -l -c "
            source /opt/ros/humble/setup.bash && 
            ros2 topic list
        " 2>&1 | grep -E "camera|image" || warn_msg "No camera topics found"
    fi
}

show_logs() {
    local comp="$1"
    
    _show_single_logs() {
        local component="$1"
        local supervisor_name
        local log_name
        
        if [[ "$component" == publisher:* ]]; then
            supervisor_name=$(get_supervisor_name "$component")
            local mode="${component#publisher:}"
            print_header "Publisher ($mode mode) Logs"
        elif [ "$component" = "publisher" ]; then
            local mode=$(get_publisher_mode)
            supervisor_name="${COMPONENTS[publisher:$mode]}"
            print_header "Publisher Logs (active: $mode)"
        else
            supervisor_name=$(get_supervisor_name "$component")
            print_header "$(echo "$component" | tr '[:lower:]' '[:upper:]') Logs"
        fi
        
        log_name="${LOG_NAMES[$supervisor_name]:-$supervisor_name}"
        
        echo ""
        info_msg "ROS 2 Logs (stderr - INFO/WARN/ERROR/DEBUG)"
        echo -e "${CYAN}────────────────────────────────────────${NC}"
        docker exec "$CONTAINER" tail -60 "/var/log/supervisor/${log_name}.err.log" 2>&1 || warn_msg "No ROS 2 logs found"
        
        echo ""
        info_msg "Standard Output (stdout)"
        echo -e "${CYAN}────────────────────────────────────────${NC}"
        local stdout_content
        stdout_content=$(docker exec "$CONTAINER" tail -20 "/var/log/supervisor/${log_name}.out.log" 2>&1 | grep -v "^$" || true)
        if [ -n "$stdout_content" ]; then
            echo "$stdout_content"
        else
            echo -e "${YELLOW}(empty - ROS 2 uses stderr for logs)${NC}"
        fi
        echo ""
    }
    
    if [ "$comp" = "all" ]; then
        print_header "All Components Logs"
        for key in slam relay publisher gps_bridge; do
            if [[ "$key" == publisher:* ]]; then continue; fi
            _show_single_logs "$key"
            echo ""
        done
    else
        _show_single_logs "$comp"
    fi
}

# ============================================================================
# Component Actions
# ============================================================================

start_component() {
    local comp="$1"
    local supervisor_name
    
    if [ "$comp" = "publisher" ]; then
        local mode=$(get_publisher_mode)
        supervisor_name="${COMPONENTS[publisher:$mode]}"
        info_msg "Starting publisher in $mode mode ($supervisor_name)..."
    elif [[ "$comp" == publisher:* ]]; then
        supervisor_name=$(get_supervisor_name "$comp")
        local mode="${comp#publisher:}"
        info_msg "Starting publisher in $mode mode ($supervisor_name)..."
        set_publisher_mode "$mode"
    else
        supervisor_name=$(get_supervisor_name "$comp")
        info_msg "Starting $comp ($supervisor_name)..."
    fi
    
    kill_processes "${comp%%:*}"
    sleep 1
    
    run_supervisorctl "start" "$supervisor_name"
    success_msg "${comp%%:*} started!"
}

stop_component() {
    local comp="$1"
    local supervisor_name
    
    if [ "$comp" = "publisher" ]; then
        local mode=$(get_publisher_mode)
        supervisor_name="${COMPONENTS[publisher:$mode]}"
        info_msg "Stopping publisher ($mode mode)..."
    elif [[ "$comp" == publisher:* ]]; then
        supervisor_name=$(get_supervisor_name "$comp")
        local mode="${comp#publisher:}"
        info_msg "Stopping publisher ($mode mode)..."
    else
        supervisor_name=$(get_supervisor_name "$comp")
        info_msg "Stopping $comp ($supervisor_name)..."
    fi
    
    run_supervisorctl "stop" "$supervisor_name"
    sleep 2
    
    kill_processes "${comp%%:*}"
    success_msg "${comp%%:*} stopped!"
}

restart_component() {
    local comp="$1"
    
    if [ "$comp" = "all" ]; then
        print_header "Restarting All Components"
        stop_component "all"
        sleep 3
        start_component "all"
    else
        info_msg "Restarting $comp..."
        stop_component "$comp"
        sleep 2
        start_component "$comp"
    fi
}

handle_all() {
    local action="$1"
    
    case "$action" in
        start)
            print_header "Starting All Components"
            for key in slam relay publisher gps_bridge; do
                start_component "$key"
                sleep 2
            done
            success_msg "All components started!"
            ;;
        stop)
            print_header "Stopping All Components"
            for key in slam relay publisher gps_bridge; do
                stop_component "$key"
            done
            success_msg "All components stopped!"
            ;;
        restart)
            print_header "Restarting All Components"
            handle_all "stop"
            sleep 3
            handle_all "start"
            ;;
        status)
            print_header "All Components Status"
            for key in slam relay publisher gps_bridge; do
                show_status "$key"
                echo ""
            done
            ;;
        logs)
            show_logs "all"
            ;;
        kill)
            print_header "Killing All Processes"
            for key in slam relay publisher gps_bridge; do
                kill_processes "$key"
            done
            success_msg "All processes killed!"
            ;;
    esac
}

# ============================================================================
# Main Logic
# ============================================================================

main() {
    if [ $# -lt 1 ]; then
        show_usage
        exit 1
    fi
    
    local component="$1"
    local action="${2:-status}"
    local extra_arg="${3:-}"
    
    check_container
    
    if [ "$component" = "all" ]; then
        handle_all "$action"
        return
    fi
    
    if [ -z "${COMPONENTS[$component]}" ] && [ "$component" != "publisher" ]; then
        error_msg "Unknown component: $component"
        echo "Valid: slam, relay, publisher, publisher:folder, publisher:realsense, gps_bridge, all"
        exit 1
    fi
    
    case "$action" in
        start)
            start_component "$component"
            ;;
        stop)
            stop_component "$component"
            ;;
        restart)
            restart_component "$component"
            ;;
        status)
            show_status "$component" "$extra_arg"
            ;;
        logs)
            show_logs "$component"
            ;;
        kill)
            kill_processes "$component"
            ;;
        mode)
            if [ "$component" != "publisher" ]; then
                error_msg "Mode switching only available for 'publisher' component"
                exit 1
            fi
            if [ -z "$extra_arg" ]; then
                show_publisher_mode
            else
                switch_publisher_mode "$extra_arg"
            fi
            ;;
        path)
            if [ "$component" != "publisher:folder" ]; then
                error_msg "'path' action only available for 'publisher:folder' component"
                exit 1
            fi
            handle_publisher_folder_path "$extra_arg"
            ;;
        *)
            error_msg "Unknown action: $action"
            echo "Valid: start, stop, restart, status, logs, kill, mode, path"
            exit 1
            ;;
    esac
}

main "$@"
