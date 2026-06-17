#!/bin/bash
# ============================================================
#  OmniBlue Startup Script  (v3 — with SLAM)
#  Launches:
#    - SSH to Pi -> OmniBlue ROS2 node
#    - SSH to Pi -> RPLiDAR A2 driver
#    - Static TF bridges (Path A — glue broken TF chain)
#    - Gazebo (optional)
#    - cmd_vel Bridge
#    - Teleop (WASD)
#    - SLAM Toolbox (live 2D mapping)
#    - RViz2 (LaserScan + Map + TF)
#    - Rosbridge WebSocket
# ============================================================

CONFIG_FILE="$HOME/.omniblue_config"
LAUNCH_DIR="$HOME/Desktop/Omniblue Launch"
DDS_PROFILE="$LAUNCH_DIR/fastdds.xml"
RVIZ_CONFIG="$LAUNCH_DIR/omniblue.rviz"
SLAM_CONFIG="$LAUNCH_DIR/slam_toolbox_omniblue.yaml"

# ---------- Load last config or set defaults ----------
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
else
    LAST_IP="192.168.0.12"
    LAST_GAZEBO="y"
    LAST_LIDAR="y"
    LAST_SLAM="y"
fi

LAST_IP="${LAST_IP:-192.168.0.12}"
LAST_GAZEBO="${LAST_GAZEBO:-y}"
LAST_LIDAR="${LAST_LIDAR:-y}"
LAST_SLAM="${LAST_SLAM:-y}"

# ---------- Banner ----------
clear
echo "============================================"
echo "       OMNIBLUE STARTUP SCRIPT  v3"
echo "============================================"
echo ""

# ---------- IP Selection ----------
echo "Last IP used: $LAST_IP"
echo ""
echo "  1) Use last IP ($LAST_IP)"
echo "  2) Enter IP manually"
echo ""
read -p "Choice [1/2] (default: 1): " IP_CHOICE
IP_CHOICE="${IP_CHOICE:-1}"

if [ "$IP_CHOICE" == "2" ]; then
    read -p "Enter Pi IP address: " PI_IP
    PI_IP="${PI_IP:-$LAST_IP}"
else
    PI_IP="$LAST_IP"
fi

echo ""

# ---------- Gazebo Selection ----------
echo "Launch Gazebo simulation?"
echo "  1) Yes (last setting: $LAST_GAZEBO)"
echo "  2) No  (real robot only)"
echo ""
read -p "Choice [1/2] (default: 1): " GAZEBO_CHOICE
GAZEBO_CHOICE="${GAZEBO_CHOICE:-1}"

if [ "$GAZEBO_CHOICE" == "2" ]; then
    USE_GAZEBO="n"
else
    USE_GAZEBO="y"
fi

echo ""

# ---------- LiDAR Selection ----------
echo "Launch RPLiDAR on the Pi?"
echo "  1) Yes (last setting: $LAST_LIDAR)"
echo "  2) No  (skip LiDAR — sim only or already running)"
echo ""
read -p "Choice [1/2] (default: 1): " LIDAR_CHOICE
LIDAR_CHOICE="${LIDAR_CHOICE:-1}"

if [ "$LIDAR_CHOICE" == "2" ]; then
    USE_LIDAR="n"
else
    USE_LIDAR="y"
fi

echo ""

# ---------- SLAM Selection ----------
echo "Launch SLAM Toolbox (live mapping)?"
echo "  1) Yes (last setting: $LAST_SLAM)"
echo "  2) No  (skip SLAM)"
echo ""
read -p "Choice [1/2] (default: 1): " SLAM_CHOICE
SLAM_CHOICE="${SLAM_CHOICE:-1}"

if [ "$SLAM_CHOICE" == "2" ]; then
    USE_SLAM="n"
else
    USE_SLAM="y"
fi

echo ""
echo "============================================"
echo " Starting with:"
echo "   Pi IP   : $PI_IP"
echo "   Gazebo  : $USE_GAZEBO"
echo "   LiDAR   : $USE_LIDAR"
echo "   SLAM    : $USE_SLAM"
echo "============================================"
echo ""
sleep 1

# ---------- Save config for next time ----------
cat > "$CONFIG_FILE" <<EOF
LAST_IP="$PI_IP"
LAST_GAZEBO="$USE_GAZEBO"
LAST_LIDAR="$USE_LIDAR"
LAST_SLAM="$USE_SLAM"
EOF

# ---------- ROS environment ----------
FOXY="source /opt/ros/foxy/setup.bash"
WS="source \$HOME/omniblue_ws/install/setup.bash"
DOMAIN="export ROS_DOMAIN_ID=0"
DDS_EXPORTS="export FASTRTPS_DEFAULT_PROFILES_FILE=\"$DDS_PROFILE\" && $DOMAIN"

PI_DDS="export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/omniblue_config/fastdds.xml && export ROS_DOMAIN_ID=0"
PI_ENV="source /opt/ros/humble/setup.bash && source \$HOME/ros2_ws/install/setup.bash && $PI_DDS"

# ---------- Terminal helper ----------
open_terminal() {
    local TITLE="$1"
    local CMD="$2"
    gnome-terminal --tab --title="$TITLE" -- bash -c "$CMD; exec bash" &
    sleep 1
}

# ---------- 1. SSH into Pi -> OmniBlue node ----------
PI_CMD="ssh -t ubuntu@$PI_IP '$PI_ENV && ros2 run omniblue omniblue_node'"
open_terminal "Pi - OmniBlue Node" "$PI_CMD"

# ---------- 2. SSH into Pi -> RPLiDAR A2 ----------
if [ "$USE_LIDAR" == "y" ]; then
    LIDAR_CMD="ssh -t ubuntu@$PI_IP '$PI_DDS && source /opt/ros/humble/setup.bash && ros2 launch rplidar_ros rplidar_a2m8_launch.py'"
    open_terminal "Pi - RPLiDAR A2" "$LIDAR_CMD"
    sleep 2
fi
# ---------- 2c. Robot State Publisher (URDF -> TF) ----------
# Loads the omniblue URDF and publishes base_footprint -> base_link -> lidar
# as static TFs. Previously this came from Gazebo; now it runs standalone
# so the TF tree exists whether or not Gazebo is launched.
URDF_FILE="$HOME/omniblue_ws/src/omniblue_description/urdf/omniblue.urdf.xacro"
RSP_CMD="$FOXY && $WS && $DDS_EXPORTS && ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(xacro $URDF_FILE robot_name:=omniblue)\""
open_terminal "Robot State Publisher" "$RSP_CMD"
sleep 2


# ---------- 3. Static TF Bridges (Path A) ----------
# Glue together the broken TF chain so SLAM can connect odom to the laser frame.
# - omniblue/base_footprint -> base_footprint  (bridges odom side to URDF side)
# - lidar -> laser                              (bridges URDF side to scan frame_id)
STATIC_TF_CMD="$FOXY && $DDS_EXPORTS && \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 omniblue/base_footprint base_footprint & \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 lidar laser & \
wait"
open_terminal "Static TF Bridges" "$STATIC_TF_CMD"
sleep 1

# ---------- 4. Gazebo (optional) ----------
if [ "$USE_GAZEBO" == "y" ]; then
    GAZEBO_CMD="$FOXY && $WS && $DDS_EXPORTS && ros2 launch omniblue_gazebo one_omniblue.launch.py"
    open_terminal "Gazebo Simulation" "$GAZEBO_CMD"
    sleep 4
fi

# ---------- 5. cmd_vel Bridge ----------
BRIDGE_CMD="$FOXY && $WS && $DDS_EXPORTS && python3 -c \"
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Bridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/omniblue/cmd_vel', self.cb, 10)
    def cb(self, msg):
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(Bridge())
\""
open_terminal "cmd_vel Bridge" "$BRIDGE_CMD"

# ---------- 6. Teleop ----------
TELEOP_CMD="$FOXY && $WS && $DDS_EXPORTS && ros2 run omniblue_teleop omniblue_teleop_key"
open_terminal "Teleop WASD" "$TELEOP_CMD"

# ---------- 7. SLAM Toolbox ----------
if [ "$USE_SLAM" == "y" ]; then
    if [ -f "$SLAM_CONFIG" ]; then
        SLAM_CMD="$FOXY && $DDS_EXPORTS && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=\"$SLAM_CONFIG\" use_sim_time:=false"
    else
        echo "WARNING: $SLAM_CONFIG not found, launching SLAM Toolbox with defaults"
        SLAM_CMD="$FOXY && $DDS_EXPORTS && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false"
    fi
    open_terminal "SLAM Toolbox" "$SLAM_CMD"
    sleep 2
fi

# ---------- 8. RViz2 ----------
if [ -f "$RVIZ_CONFIG" ]; then
    RVIZ_CMD="$FOXY && $DDS_EXPORTS && rviz2 -d \"$RVIZ_CONFIG\""
else
    echo "WARNING: $RVIZ_CONFIG not found, launching RViz2 with empty config"
    RVIZ_CMD="$FOXY && $DDS_EXPORTS && rviz2"
fi
open_terminal "RViz2" "$RVIZ_CMD"

# ---------- 9. Rosbridge WebSocket ----------
ROSBRIDGE_CMD="source /opt/ros/foxy/setup.bash && $DDS_EXPORTS && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
open_terminal "Rosbridge WS" "$ROSBRIDGE_CMD"

echo ""
echo "All terminals launched! Check each tab."
echo "  - 'Teleop WASD' to drive the robot"
echo "  - 'SLAM Toolbox' is building the map"
echo "  - 'RViz2' shows LaserScan + Map + TF tree"
echo "  - 'Pi - RPLiDAR A2' is the LiDAR driver, leave it running"
echo ""
