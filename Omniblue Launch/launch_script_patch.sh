#!/bin/bash
# ============================================================
#  PATCH: Path A SLAM additions for omniblue_start.sh
#  Paste these three sections into your existing script.
#  See the comments for where each goes.
# ============================================================


# ============================================================
#  SECTION 1 — Add near the top, after RVIZ_CONFIG="..." line
# ============================================================
SLAM_CONFIG="$LAUNCH_DIR/slam_toolbox_omniblue.yaml"


# ============================================================
#  SECTION 2 — Add as a new step, AFTER the RPLiDAR section
#  and BEFORE Gazebo. This bridges the broken TF chain so
#  SLAM can connect odom to the laser frame.
# ============================================================

# ---------- 2b. Static TF bridges (Path A) ----------
# The Pi publishes:
#   omniblue/odom -> omniblue/base_footprint    (from omniblue_node)
#   base_footprint -> base_link -> lidar        (from robot_state_publisher/URDF)
# The RPLiDAR publishes scans with frame_id "laser" — but the URDF declares
# the child frame as "lidar". So we add two identity transforms to glue
# everything together for Path A. These go away in Path B.
STATIC_TF_CMD="$FOXY && $DDS_EXPORTS && \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 omniblue/base_footprint base_footprint & \
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 lidar laser & \
wait"
open_terminal "Static TF Bridges" "$STATIC_TF_CMD"
sleep 1


# ============================================================
#  SECTION 3 — Add AFTER teleop, BEFORE RViz2.
#  SLAM Toolbox subscribes to /scan and the TF tree, publishes /map.
# ============================================================

# ---------- 5b. SLAM Toolbox ----------
if [ -f "$SLAM_CONFIG" ]; then
    SLAM_CMD="$FOXY && $DDS_EXPORTS && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=\"$SLAM_CONFIG\" use_sim_time:=false"
else
    echo "WARNING: $SLAM_CONFIG not found, launching SLAM Toolbox with defaults"
    SLAM_CMD="$FOXY && $DDS_EXPORTS && ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false"
fi
open_terminal "SLAM Toolbox" "$SLAM_CMD"
sleep 2
