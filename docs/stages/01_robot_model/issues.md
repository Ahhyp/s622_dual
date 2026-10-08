# 阶段 1：遇到的问题与解决方案

## 问题 1：conda 环境下 colcon build 失败

**日期：** 2026-05-21

**现象：**
```
ModuleNotFoundError: No module named 'catkin_pkg'
CMake Error at ament_package_xml.cmake:95
```

**原因：**
- CMake 调用 `package_xml_2_cmake.py` 解析 `package.xml`
- 该脚本需要 `catkin_pkg` 模块
- conda base Python 缺少该模块

**解决：**
在 yolov8 环境中安装缺失的构建依赖：
```bash
conda activate yolov8
pip install catkin_pkg empy colcon-common-extensions
```

---

## 问题 2：CMake 找到 conda base Python 而非 yolov8 的 Python

**日期：** 2026-05-21

**现象：**
安装 catkin_pkg 后仍然报错，日志显示：
```
execute_process(/home/yep/miniconda3/bin/python3 ...)
```
而非 yolov8 环境下的 Python（`/home/yep/miniconda3/envs/yolov8/bin/python3`）。

**原因：**
CMake 查找 Python 时优先找到了 conda base 的 Python，而非已激活环境中的 Python。

**解决：**
构建时显式指定 Python 路径：
```bash
colcon build --merge-install --symlink-install \
    --cmake-args "-DPython3_EXECUTABLE=$(which python3)"
```

---

## 问题 3：empy 4.x 与 ROS 2 Humble rosidl_adapter 不兼容

**日期：** 2026-05-22

**现象：**
```
AttributeError: module 'em' has no attribute 'BUFFERED_OPT'
AttributeError: 'NoneType' object has no attribute 'shutdown'
```
编译 `fairino_msgs` 时，`rosidl_adapter` 调用 `em` 模板引擎失败。

**原因：**
pip 安装的 `empy` 4.2.1 API 与 ROS 2 Humble 需要的 3.x 不兼容。`BUFFERED_OPT` 在 4.x 中被移除。

**解决：**
```bash
pip install 'empy<4'
```
