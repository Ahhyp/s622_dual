# C43 抓取关系迁移与双臂接近验证（2026-11-11）

本轮接续 `C43_ESCAPE_DIAGNOSIS_20261111.md`：把此前“样件与支撑 X+50 mm”的成功相对几何转为机器人位姿候选，样件和支撑恢复原 world 位置。冻结的 q_grasp12 和正式场景未改。

**结果：完整双臂模型中，右臂停在 home，左臂新位姿连续两轮完成约50 mm抬升和约20 s保持；双臂分开抓取点并倾斜15°的张爪接近完成一次。尚未验证双臂同时夹持或交接。** 两轮左臂夹持均有约5.42 mm的横向摆动峰值，且一指短时到达开限位，不能据此宣称所有方向均已稳定。

## 环境与输入

本轮实际 CLI 再次成功：`docker inspect s622_yep --format '{{json .Mounts}}'` 显示 `/home/yep/s622_dual` 读写挂载到 `/workspace`；`docker exec -w /workspace s622_yep pwd` 输出 `/workspace`。未发生 socket 权限拒绝。

使用完整双臂模型，源为 `log/c43_position_ab_20261111/w50_fixed_contact_r1/robot.urdf`，SRDF 为 `src/s622_moveit_config/config/s622_dual_arm.srdf`。离线副本 `log/c43_candidates_20261111/dual_robot_offline.urdf` 仅把绝对 mesh 路径加 `file://`，满足 MoveIt 资源加载器；Gazebo 输入保持原路径。初次未加 URI 时加载器拒绝空几何，未用该失败结果做碰撞判断。最终加载器确认 9 个 mesh 引用均解析成功。

样件仍为 120×50×50 mm、0.35 kg；支撑圆柱半径 40 mm、高 250 mm，样件中心 z=275 mm。保留原 STL、原生效的接触配置，无 weld。仿真采用独立 ROS_DOMAIN_ID=146 和每轮独立 IGN_PARTITION；原有其他仿真不动。

## IK 与碰撞筛选

新增 `src/dual_arm_pose_optimization/src/c43_candidate_probe.cpp`。解析 IK 后用实际 URDF 的 FK 过滤，位置残差 <0.1 mm、旋转残差 <0.0001 rad，并检查关节限位。两臂所有候选组合使用完整机器人 FCL、原 SRDF 允许碰撞矩阵及既有底座/地面豁免，不新增左右臂间豁免。手指保持张开 ±30.5 mm。

冻结目标实际 TCP world X 约为左 +49.692 mm、右 -49.692 mm。左臂目标沿 world X 减 50 mm 后，TCP X 约 -0.308 mm，保留原朝向和 z≈300.004 mm。

| 候选 | 筛选结果 |
|---|---|
| 左臂居中，右臂 home | 存在 2 组无碰撞端点组合 |
| 左臂居中，右臂旧抓取位 | 腕部互碰，拒绝 |
| 两臂都居中、原朝向 | 手指及腕部互碰，拒绝 |
| 双臂 X=±15/25/35 mm、无倾角 | 所测组合均碰撞 |
| X=±25 mm、各绕 world Y 向外倾 15° | 存在 12 组无碰撞端点组合；选中组合两臂最小距离约25.228 mm |

选中解按距冻结 q12 的平方距离排序；不代表已做全局最优抓取优化。其它倾角端点见 `candidate_scan.csv` 和 `candidates.json`，`first_rejected_pairs` 是被拒分支的首个碰撞记录，不是选中解的碰撞。

接近顺序与实跑保持一致，线性关节插值做离散 FCL 检查：

- 左臂方案：home→左预抓取→每 5 mm 下降至抓取；311 个检查状态，两臂间最小266.878 mm、机器人对环境最小5.402 mm。反向下降表用于抬升。
- 双臂方案：右预抓取→左预抓取→左下降→右下降；622 个检查状态，两臂间最小24.123 mm、机器人对环境最小5.401 mm。

这是离散路径采样结果，不能当作连续碰撞检测证明。完整机器人加载了18个 mesh 碰撞形状；环境采用样件、支撑及4×4 m、上表面z=0的保守平板，并非逐个复刻 Gazebo 桌腿。环境距离覆盖整个机器人，最小值也受底座/平板影响。张爪检查不涵盖夹紧、持物或交接运动。

候选关节、TCP 和路径表保存在 `left_center_path.json`、`dual_split25_tilt15_path.json`；左臂路径采样保存在 `path_samples.csv`。

## 实际命令与日志

日志根目录：`/home/yep/s622_dual/log/c43_candidates_20261111/`。以下命令在宿主机执行，实验目录必须尚不存在。

离线构建与筛选（需已有上述离线 URDF）：

```bash
docker exec -w /workspace s622_yep bash -lc 'source /opt/ros/humble/setup.bash; source install/setup.bash; cmake --build build/dual_arm_pose_optimization --target c43_candidate_probe -j2 > log/c43_candidates_20261111/build.log 2>&1 && build/dual_arm_pose_optimization/c43_candidate_probe log/c43_candidates_20261111/dual_robot_offline.urdf src/s622_moveit_config/config/s622_dual_arm.srdf log/c43_candidates_20261111 > log/c43_candidates_20261111/probe.log 2>&1'
```

左臂新位姿、完整双臂模型、右臂 home、每指原生恒定驱动力5 N：

```bash
docker exec -w /workspace s622_yep bash -lc 'source /opt/ros/humble/setup.bash; source install/setup.bash; C43_LAB_DUAL=1 C43_CANDIDATE_JSON=/workspace/log/c43_candidates_20261111/left_center_path.json C43_CLAMP_N=5 C43_CAPTURE_NATIVE_POSES=1 C43_HOLD_LIFT_S=20 python3 src/gz_launch/scripts/c43_force_lab.py /workspace/log/c43_candidates_20261111/left_center_dual_robot_r2 force .0249'
```

第一轮 `left_center_dual_robot_r1` 在发送轨迹前因候选 JSON 整数零没有转换为 ROS 浮点值而停止；已在轨迹封装中统一 float 转换。该轮无抓取结果，不计入物理对照。

第二次完整复测使用同一命令，将输出目录改为 `left_center_dual_robot_r3`。

双臂张爪接近：

```bash
docker exec -w /workspace s622_yep bash -lc 'source /opt/ros/humble/setup.bash; source install/setup.bash; C43_LAB_DUAL=1 C43_CANDIDATE_JSON=/workspace/log/c43_candidates_20261111/dual_split25_tilt15_path.json python3 src/gz_launch/scripts/c43_force_lab.py /workspace/log/c43_candidates_20261111/dual_approach_r1 dual_dry .0249'
```

实测关节状态复查（读取全部12个臂关节及4个实际手指关节，不假定手指始终在指令位置）：

```bash
docker exec -w /workspace s622_yep bash -lc 'source /opt/ros/humble/setup.bash; source install/setup.bash; build/dual_arm_pose_optimization/c43_candidate_probe log/c43_candidates_20261111/dual_robot_offline.urdf src/s622_moveit_config/config/s622_dual_arm.srdf log/c43_candidates_20261111/dual_approach_r1 log/c43_candidates_20261111/dual_approach_r1/telemetry.csv > log/c43_candidates_20261111/dual_approach_r1/actual_path_collision_audit.log 2>&1'
```

每轮保存 `launch.log`、`probe.log`、`conditions.json`、生成后的模型/世界、脚本快照、原始接触消息、关节/位姿 telemetry。恒力轮另有两指 FT 原始流、原生位姿和 `instrumented_summary.json`。

## 本轮观测

| 完整双臂模型中的左臂恒力轮 | 腕部升高 mm | 样件升高 mm | 竖直跟随差 mm | 保持采样跨度 s | 样件末Y偏移 mm | 保持最大绝对Y偏移 mm |
|---|---:|---:|---:|---:|---:|---:|
| left_center_dual_robot_r2 | 50.001 | 49.975 | 0.026 | 19.97 | -3.267 | 5.422 |
| left_center_dual_robot_r3 | 50.001 | 49.977 | 0.024 | 19.90 | -3.174 | 5.423 |

保持指令为20 s墙钟时间，表中为实际仿真时钟采样跨度。两轮保持阶段分别4000/3986条完整接触消息，全都同时包含左侧两指；接近阶段没有样件/手指接触。右臂最大home关节误差分别约5.08e-6/4.89e-6 rad。

原生位姿与Contacts按完全相同仿真时间戳审计：两轮抬升/保持分别1415/1424帧，只有最初2帧仍触支撑，此后脱离。末俯仰分别约-0.0099°/-0.0142°；两指STL顶点深入样件盒的最大见证均不超过0.0082 mm。未复现原端部抓取的毫米级深重叠与明显翻转。FT轴向外载估计仍在约5 N，动态估计不等同精确指面接触力。

横向摆动不能忽略：峰值处finger1 q≈+0.0305 m、finger2 q≈-0.01966 m，总开口仍约50.16 mm。一指到了开限位，另一指随样件移动；不能把该阶段都解释成完全不受限位影响的自由恒力夹持。按q1≥0.03049 m计，保持阶段分别66/1998、237/1991个关节采样靠近开限位。相等反向恒力没有显式约束夹爪中心，这是横向共同移动的一种机制解释，尚未用中心约束对照单独验证。不能把横摆归因于右臂存在。

双臂张爪轮：两侧末关节最大误差均约5.56e-11 rad；样件仍在支撑上，接近及5 s候选保持阶段的样件接触消息均没有机器人碰撞对。对全部3831个实测状态做FCL复查，0个碰撞状态，最小两臂间距23.971 mm、机器人对环境5.401 mm。该数据仅验证张爪接近；没有夹紧或持物。

## 验证、结论与下一步

新增C++目标构建成功，相关Python通过py_compile，git diff --check通过。各轮脚本正常结束并清理各自进程组。结束时原有launch PID42896、Gazebo PID42900/299501仍在，记录见 `remaining_simulations.json`；本轮隔离Gazebo已结束。

`comparison.json` 汇总所有轮次，保留启动失败而不混入物理成功次数；可在容器运行 `python3 src/gz_launch/scripts/c43_candidate_transfer_analyze.py log/c43_candidates_20261111` 重建。各恒力轮的 `native_escape_audit.json` 来自同目录脚本 `c43_native_escape_audit.py RUN_DIR`。

本轮把临时平移场景的成功几何迁移到了真实可达的左臂候选，并验证了完整模型中的抬升；同时找到了有接近空间的双臂端点。下一步应先加入夹爪中心约束并验证横摆/单指限位，再按双臂倾斜姿态核对力轴与重力补偿，验证双臂夹紧、共同持物、释放交接全过程。当前结果不属于双臂恒力夹持或W7交接验收。
