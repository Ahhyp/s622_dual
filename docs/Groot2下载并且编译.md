# Groot2 安装方法

Groot2 不走 apt，是单文件 AppImage。社区版免费够用。

## 一、下载

官网：[https://www.behaviortree.dev/groot](https://www.behaviortree.dev/groot)

进去选 **Community Edition**，下载 Linux 版本的 AppImage（大概 100MB 左右）。

如果官网下载慢，也可以直接到 GitHub release 页面找：
[https://github.com/BehaviorTree/Groot2/releases](https://github.com/BehaviorTree/Groot2/releases)

## 二、安装（其实就是给执行权限）

```bash
# 假设下载到了 ~/Downloads
cd ~/Downloads
chmod +x Groot2-*.AppImage

# 直接跑
./Groot2-*.AppImage
```

## 三、WSL 用户特别说明

你是 WSL2（看你之前路径是 `DESKTOP-I7Q0R33`），Groot2 是 GUI 程序，分两种情况：

### 情况 A：Windows 11 + WSLg（最省事）

直接在 WSL 里跑就行，窗口会自动转发到 Windows 桌面：

```bash
./Groot2-*.AppImage
```

如果弹出 sandbox 相关错误，加参数：

```bash
./Groot2-*.AppImage --no-sandbox
```

### 情况 B：Windows 10 或没装 WSLg

WSL 里跑 AppImage 比较麻烦，**推荐直接装到 Windows 侧**：

1. 下载 Windows 版本的 Groot2（官网同一页有 `.exe` 安装包）
2. 在 Windows 上安装
3. 通过 TCP 连接到 WSL 里的 BT executor

Groot2 和 BT executor 是通过 **网络通信**（默认端口 1667），不需要在同一系统。WSL2 的网络配置稍微注意一下：

* WSL2 的 ROS2 节点监听 1667 端口
* Windows 侧 Groot2 用 `localhost:1667` 或 WSL 的 IP 连接（一般 localhost 就行，因为 WSL2 端口转发）

## 四、放到方便启动的位置

AppImage 是单文件，建议放到固定目录：

```bash
mkdir -p ~/apps
mv ~/Downloads/Groot2-*.AppImage ~/apps/Groot2.AppImage
chmod +x ~/apps/Groot2.AppImage

# 加 alias，敲 groot2 就启动
echo "alias groot2='~/apps/Groot2.AppImage'" >> ~/.bashrc
source ~/.bashrc
```

之后任何终端敲 `groot2` 就能开。

## 五、第一次启动需要做什么

1. 打开后会让你选 **License**，选 **Community Edition**，免费且功能够用

2. 主界面有两个模式：

   * **Editor**：拖拽编辑 XML，新建/打开 BT 文件
   * **Monitor**：连接到正在运行的 BT executor，实时观察

3. 跑你的 BT executor 时，在 C++ 代码里加一行：

   ```cpp
   BT::Groot2Publisher publisher(tree, 1667);
   ```

   然后在 Groot2 选 Monitor 模式，连 `localhost:1667`，能看到行为树实时高亮。

## 六、可选：装桌面图标（仅 native Linux 有用）

WSL 里没意义，跳过。native Linux 想要桌面图标：

```bash
cat > ~/.local/share/applications/groot2.desktop << EOF
[Desktop Entry]
Name=Groot2
Exec=$HOME/apps/Groot2.AppImage
Icon=applications-development
Type=Application
Categories=Development;
EOF
```

---

装好之后告诉我能不能正常打开，然后我们就能在第一版 mock 跑起来时用它实时监控行为树了。

# 验证
终端都是 纯 ROS2
第一个终端:
```bash
ros2 run btcpp_ros2_samples sleep_server
```
第二个:
```bash
ros2 launch btcpp_ros2_samples sample_bt_executor.launch.xml
```
第三个
```bash
ros2 action send_goal /behavior_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: 'SleepActionSample}"

# 然后开启 groot2
groot2
```
groot2 里面 调整到 monitor 模式， 然后端口换成 1667， 就可以看到东西了。
