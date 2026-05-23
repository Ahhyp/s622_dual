# 阶段 2：遇到的问题与解决方案

## 问题 1：nanoflann FetchContent 导致 "uninstall" 目标冲突

**日期：** 2026-05-22

**现象：**
```
add_custom_target cannot create target "uninstall" because another target
with the same name already exists.
```

**原因：**
ament_cmake 已创建了 `uninstall` 自定义目标，而 `FetchContent_MakeAvailable(nanoflann)` 也尝试创建同名目标，导致冲突。

**解决：**
不要通过 FetchContent 下载 nanoflann。按原项目 CLAUDE.md 的说明，直接安装系统包：
```bash
sudo apt install libnanoflann-dev
```
CMakeLists.txt 中的 `find_path` 会在 `/usr/include` 找到 nanoflann，完全跳过 FetchContent 分支。

---

## 问题 2：path_shortcut.h 使用 Python 三引号语法

**日期：** 2026-05-22

**现象：**
```
error: empty character constant
error: missing terminating ' character
error: extended character 、 is not valid in an identifier
```

**原因：**
原项目 `path_shortcut.h` 第 14-16 行使用了 Python 的 `'''` 三引号包裹中文注释文本，C++ 编译器无法识别此语法。

**解决：**
将 `'''` 改为 C++ 单行注释 `//`：
```cpp
// 这个 PathOptimizer 类是整个路径后处理的核心...
```

---

（以下为自写阶段的记录）
