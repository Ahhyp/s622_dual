import re
import sys
import shutil

# 需要加 prefix 的 link (world 特殊处理,不加 prefix)
LINKS = [
    "base_link", "shoulder_link", "upperarm_link", "forearm_link",
    "wrist1_link", "wrist2_link", "wrist3_link",
    "finger1", "finger2",
    "grasp_frame",
    "calibration_marker", "aruco_marker_link",
]

# 需要加 prefix 的 joint (world_joint 特殊处理)
JOINTS = [
    "j1", "j2", "j3", "j4", "j5", "j6",
    "finger1_joint", "finger2_joint",
    "wrist3_to_grasp_frame",
    "wrist3_to_calibration_marker",
    "wrist3_to_aruco_marker",
    "calibration_to_aruco_marker",
]


def prefix_link(content, name):
    """替换 link 相关的 name/parent link/child link/gazebo reference。"""
    # <link name="X"> (跨行也行)
    content = re.sub(
        r'(<link\s+name\s*=\s*")' + re.escape(name) + r'(")',
        r'\g<1>${prefix}' + name + r'\g<2>',
        content, flags=re.DOTALL
    )
    # <parent link="X"> (跨行也行)
    content = re.sub(
        r'(<parent\s+link\s*=\s*")' + re.escape(name) + r'(")',
        r'\g<1>${prefix}' + name + r'\g<2>',
        content, flags=re.DOTALL
    )
    # <child link="X"> (跨行也行)
    content = re.sub(
        r'(<child\s+link\s*=\s*")' + re.escape(name) + r'(")',
        r'\g<1>${prefix}' + name + r'\g<2>',
        content, flags=re.DOTALL
    )
    # <gazebo reference="X">
    content = re.sub(
        r'(<gazebo\s+reference\s*=\s*")' + re.escape(name) + r'(")',
        r'\g<1>${prefix}' + name + r'\g<2>',
        content
    )
    return content


def prefix_joint(content, name):
    """替换 joint 的 name 和 mimic joint 引用。"""
    # <joint name="X">
    content = re.sub(
        r'(<joint\s+name\s*=\s*")' + re.escape(name) + r'(")',
        r'\g<1>${prefix}' + name + r'\g<2>',
        content, flags=re.DOTALL
    )
    # <mimic joint="X">
    content = re.sub(
        r'(<mimic\s+joint\s*=\s*")' + re.escape(name) + r'(")',
        r'\g<1>${prefix}' + name + r'\g<2>',
        content
    )
    # <gazebo reference="X"> (可能引用 joint)
    content = re.sub(
        r'(<gazebo\s+reference\s*=\s*")' + re.escape(name) + r'(")',
        r'\g<1>${prefix}' + name + r'\g<2>',
        content
    )
    return content


def add_xacro_header(content):
    """加 xmlns:xacro 和 arg 声明。"""
    # 加 xmlns:xacro
    if 'xmlns:xacro' not in content:
        content = re.sub(
            r'(<robot\s+)',
            r'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" ',
            content, count=1
        )
    
    # 在 <robot ...> 之后插入 arg 声明
    header = (
        '\n  <xacro:arg name="prefix" default=""/>\n'
        '  <xacro:arg name="attach_world" default="true"/>\n'
        '  <xacro:property name="prefix" value="$(arg prefix)"/>\n'
    )
    content = re.sub(r'(<robot[^>]*>)', r'\1' + header, content, count=1)
    return content


def wrap_world(content):
    """把 world link 和 world_joint 用 xacro:if attach_world 包起来。"""
    # 匹配 <link name="world"/> (自闭合 或 有 body)
    world_link_re = re.compile(
        r'<link\s+name\s*=\s*"world"\s*/>',
        re.DOTALL
    )
    world_joint_re = re.compile(
        r'<joint\s+name\s*=\s*"world_joint"[^>]*>.*?</joint>',
        re.DOTALL
    )
    
    world_link_match = world_link_re.search(content)
    world_joint_match = world_joint_re.search(content)
    
    if not (world_link_match and world_joint_match):
        print("WARN: 未找到 world/world_joint,跳过条件化")
        return content
    
    # world_joint 的 child link="base_link" 需要被参数化
    # 因为 world_joint 内部有 <child link="base_link"/> 引用会被前面的 prefix_link 处理
    # 所以先记录原文本 (此时 world_joint 里的 child link 已被前缀化了)
    world_link_text = world_link_match.group(0)
    world_joint_text = world_joint_match.group(0)
    
    # 删除原位置
    content = content.replace(world_link_text, '')
    content = content.replace(world_joint_text, '')
    
    # 在 xacro:property 之后插入 xacro:if 块
    wrapped = (
        '\n  <xacro:if value="$(arg attach_world)">\n'
        f'    {world_link_text}\n'
        f'    {world_joint_text}\n'
        '  </xacro:if>\n'
    )
    content = re.sub(
        r'(<xacro:property\s+name\s*=\s*"prefix"[^/]*/>)',
        r'\1' + wrapped,
        content, count=1
    )
    return content


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input.urdf> <output.urdf.xacro>")
        sys.exit(1)
    
    input_path, output_path = sys.argv[1], sys.argv[2]
    shutil.copy(input_path, input_path + '.before_prefix')
    
    with open(input_path) as f:
        content = f.read()
    
    # 1. 加 xacro 头
    content = add_xacro_header(content)
    
    # 2. 参数化 links
    for name in LINKS:
        content = prefix_link(content, name)
    
    # 3. 参数化 joints
    for name in JOINTS:
        content = prefix_joint(content, name)
    
    # 4. 条件化 world/world_joint
    content = wrap_world(content)
    
    with open(output_path, 'w') as f:
        f.write(content)
    
    print(f"[OK] Output: {output_path}")
    print(f"[OK] Backup: {input_path}.before_prefix")


if __name__ == '__main__':
    main()