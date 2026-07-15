#!/bin/bash
set -e

XACRO_TOP="$(ros2 pkg prefix gz_launch)/share/gz_launch/config/robot_gazebo.urdf.xacro"
XACRO_ARM="$(ros2 pkg prefix s622_moveit_descriptions)/share/s622_moveit_descriptions/urdf/s622_moveit_descriptions.urdf.xacro"
XACRO_CAM="$(ros2 pkg prefix s622_moveit_descriptions)/share/s622_moveit_descriptions/urdf/camera/camera_standalone.urdf.xacro"

echo "=== Test 1: 空前缀 + 相机 + world (M1.7 兼容) ==="
xacro "$XACRO_TOP" prefix:="" include_camera:=true attach_world:=true \
      > /tmp/m21_test_empty.urdf
check_urdf /tmp/m21_test_empty.urdf && echo "  [OK] check_urdf passed"

echo "=== Test 2: 左前缀 + 无相机 + 无 world (双臂 left) ==="
xacro "$XACRO_TOP" \
      prefix:="left_" include_camera:=false attach_world:=false \
      > /tmp/m21_test_left.urdf
check_urdf /tmp/m21_test_left.urdf && echo "  [OK] check_urdf passed"

echo "=== Test 3: 右前缀 (双臂 right) ==="
xacro "$XACRO_TOP" \
      prefix:="right_" include_camera:=false attach_world:=false \
      > /tmp/m21_test_right.urdf
check_urdf /tmp/m21_test_right.urdf && echo "  [OK] check_urdf passed"

echo "=== Test 4: 独立相机 (M2.7 预备) ==="
xacro "$XACRO_CAM" camera_xyz:="0 0 0.75" camera_rpy:="0 1.5708 0" \
      > /tmp/m21_test_camera.urdf
check_urdf /tmp/m21_test_camera.urdf && echo "  [OK] check_urdf passed"

echo ""
echo "=== 边界检查 ==="
# 检查前缀化是否完整,没漏
echo "Test 2 (left prefix):"
BARE_LINKS=$(grep -oP '<link name="\K(base_link|shoulder_link|upperarm_link|forearm_link|wrist[1-3]_link|grasp_frame|finger[12]|calibration_marker|aruco_marker_link)"' /tmp/m21_test_left.urdf | grep -v '^left_' || true)
if [[ -n "$BARE_LINKS" ]]; then
    echo "  [FAIL] 发现未前缀化 link:"
    echo "$BARE_LINKS"
    exit 1
else
    echo "  [OK] 所有 arm link 已前缀化"
fi

BARE_JOINTS=$(grep -oP '<joint name="\K(j[1-6]|finger[12]_joint|wrist3_to_grasp_frame|wrist3_to_calibration_marker|wrist3_to_aruco_marker)"' /tmp/m21_test_left.urdf | grep -v '^left_' || true)
if [[ -n "$BARE_JOINTS" ]]; then
    echo "  [FAIL] 发现未前缀化 joint:"
    echo "$BARE_JOINTS"
    exit 1
else
    echo "  [OK] 所有 arm joint 已前缀化"
fi

echo ""
echo "=== 全部通过 ==="