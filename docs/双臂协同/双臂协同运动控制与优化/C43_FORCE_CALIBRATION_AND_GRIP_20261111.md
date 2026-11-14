# C43 外载标定与位置预紧/恒力驱动对照（2026-11-11）

## 结论

1. **Gazebo 关节 FT 测量链能正确响应已知外载。** 两个手指在抓取姿态下，各完成 world X/Y/Z 的 +2、-2、+5 N 共 9 个工况；finger1 另有 home 姿态 9 点，合计 27 个加载工况。力的幅值、方向、坐标变换及撤载回零均有原始数据。
2. **位置命令并不意味着没有夹紧作用。** 位置组接触后，去重力的轴向关节反力估计常达到约 150 N，与模型 effort 上限相符；继续改变位置目标没有展示稳定改善。该量不能直接等同于真实指面法向力，尤其在动态阶段未去除惯性贡献。
3. **每指 2 N / 5 N 的恒定驱动已实现，静态轴向反力估计也与给定值吻合，但稳定夹持未实现。** 抬升过程中总关节开口塌缩，触发诊断停止；5 N 重复亦如此。不是传感器不发布，也不是把 2/5 N 指令当成已经实现稳定夹持。
4. 本轮没有证明加力能修复当前抓取。应继续核对物体与手指的接触包络、抬升脱离路径、接触求解/模型行为；本轮尚未把这些几何与物理因素进一步区分。

## 环境与隔离

- /home/yep/s622_dual → s622_yep:/workspace，Docker inspect 与容器 pwd 本轮直接成功。
- ROS_DOMAIN_ID=146；每轮独立 IGN_PARTITION=c43-force-<run>；无 GUI。
- 使用上轮真正单臂模型作为固定输入：log/c43_position_ab_20261111/single_w50_fixed_r2/robot.urdf 与 world.sdf。无右臂、一个 ros2_control 硬件组件、无 weld。
- 每轮保存生成 URDF/SDF、world、conditions、launch/probe 日志。FT 传感器先声明在 ros2_control 中，再注入转换后的 SDF，避免 URDF→SDF 丢失 joint sensor。
- ForceTorque、ApplyLinkWrench 均放在 world。仅 force 模式加入两个 ApplyJointForce；该模式不加载手指 JTC，避免位置/力驱动竞争。手臂仍由相同 JTC 和相同路径驱动。
- 0.12×0.05×0.05 m / 0.35 kg 样件；q_grasp12 和已有 pre-grasp/下降、反向 +50 mm 抬升路径未改。位置模式使用手指 JTC；力模式是 native constant drive，不是已验收的闭环接触力控制器。

## 已知外载标定

通过 /world/c43_s4_world/wrench/persistent 向 LINK 实体 s622_dual_arm::left_finger1 或 left_finger2 施加 world 坐标系外力，各档之间使用 /wrench/clear 清除，防止累加。每点保存加载前、加载后、撤载后各约 0.5 s 的新鲜 FT 样本，通常约 100 条，记录均值、标准差、仿真时间、手指位移及 link 姿态。

| 对象/姿态 | 外载点数 | 与 RᵀF 预期的最大力向量误差 | 最大撤载回零差 |
|---|---:|---:|---:|
| finger1 / home | 9 | 约 1.0e-9 N | 2.52e-6 N |
| finger1 / grasp | 9 | 约 8.1e-10 N | 5.44e-14 N |
| finger2 / grasp | 9 | 约 8.1e-10 N | 3.47e-5 N |

这些精度是**理想 Gazebo 仿真链路的一致性结果**，不是实物传感器标定精度。

抓取姿态近似坐标映射：

```
F_child ≈ [-F_world_y, -F_world_z, +F_world_x]
```

精确预期使用当时手指 link 的四元数，传感器 frame=child、measure_direction=child_to_parent。拟合文件里的矩阵使用行向量约定：child_row = world_row @ gain。

两个工况发生行程变化，明确保留：finger1/home 的 world Z +5 N 使手指移动约 +10.5 mm；finger2/grasp 的 world Y +5 N 使手指移动约 -10.5 mm。加载后的测量窗口仍有与外载相符的读数，但不能把全套点描述为固定行程下的标定。

已知外载只标定了**力通道**；力矩通道有原始记录，但没有施加独立已知力矩来标定全部力矩轴。

此前 force.x≈-0.0864948 N、torque.z≈0.00306797077 N·m，分别吻合手指质量 0.008826 kg 的重力及质心 y=0.03547 m 的力矩。现在外载加载/撤除试验进一步说明：恒定基线不等于传感器失效。

## 对照方法与停止条件

- position：从 ±0.0305 m 用 8 s 合到 ±0.0249 或 ±0.0248 m，继续保持目标，稳定 3 s，再抬升 50 mm / 8 s，观察 5 s。
- force：先保持张开完成同样的左臂 homing；使用关节速度反馈以约 0.7 mm/s 低速靠近。完整顶层 Contacts 消息出现双指接触后，在 4 档、每档 1 s 内提升到每指 2 或 5 N；稳定 3 s，再执行同一路径。
- 两种模式接触建立历史不同，因此是两套驱动配置的实测比较，不是完全相同闭合过程下的单变量力值因果实验。
- native C++ 发布器在同一进程中更新两个 /cmd_force，避免两次 CLI 调用造成的不对称。日志 pair_publish_us 是两次 Publish 调用的耗时（通常十几到几十微秒），**不是物理执行延迟或硬实时性能测量**。
- force 的最终诊断停止条件：|q1|+|q2| < 48 mm 持续 30 ms 仿真时间。该量是**关节位置绝对值之和，不是直接测得的指面内距**；它用于避免物体失去包络后手指继续合空，不是正式夹持验收阈值或硬实时保护。
- 报告中的“关节轴向反力估计” = 原生 FT 力向量减去按 link 姿态预测的手指重力，再投影到关节轴，并按两个手指的方向归一化。动态阶段含惯性等贡献，不能不加区分地称为指面接触法向力。JSON 中 estimated_external_normal_stats 是该估计的字段名。

## 最终协议实测

position 行为最终值取完整抬升后 5 s 观察末；force 被中止行的值只到停止时刻，**没有完成 50 mm 腕部抬升**，不可当作完整抬升后的最终值。

| run | 策略 | 静态轴向反力估计（每指） | 腕部实际抬升 mm | 样件峰值抬升 mm | 样件末值 mm | 结果 |
|---|---|---:|---:|---:|---:|---|
| position_p0249_r1 | ±.0249 | 约150 N | 50.001 | 45.388 | 43.924 | 有滑移，未完整跟随 |
| position_p0249_r2 | ±.0249 | 原始记录见日志 | 50.000 | 21.001 | -0.003 | 回落，重复不稳定 |
| position_p0248_r1 | ±.0248 | 均值约145.5/146.0 N，峰值150 N | 50.001 | 38.470 | 38.161 | 更深目标未展示改善 |
| force_2N_sync_gap_r1 | 每指2 N | 约2.000/2.000 N | 5.912 | 2.597 | 2.533 | 开口塌缩，诊断停止 |
| force_5N_sync_gap_r1 | 每指5 N | 约5.000/5.000 N | 7.197 | 3.219 | 3.219 | 开口塌缩，诊断停止 |
| force_5N_sync_gap_r2 | 每指5 N | 原始记录见日志 | 18.478 | 8.993 | 8.993 | 同类停止，未建立稳定夹持 |

position 的“完成脚本”不等于夹持成功；comparison.json 的 completed 字段只表示程序到达了抬升观察末。

±.0249 第一轮抬升前实际 q≈[.0249170,-.0249175]；±.0248 的实际 q≈[.0248399,-.0249557]。后者一侧并未完全达到更深目标。因此不能把改变命令值简单解释为实际预紧量同比增加。

force 最终轮次停止时的最小关节总开口分别约 38.36、35.42、35.20 mm。物体仍为 50 mm 宽，说明当前几何下有效包络已经异常；仅凭这些数值尚不能把“手指滑离端边”和“接触求解/穿透问题”分开定性。

## 排除的协议调试记录

- force_2N_r1、force_5N_r1：分别用 CLI 更新两指，时间差大，且直接施力自由闭合过快；未形成有效抬升对照。
- force_2N_sync_r1、force_5N_sync_r1：已实现同步/低速靠近，但使用了单根手指的固定中心阈值。5 N 轮次两指总开口仍约50.14 mm时即停止，证明该条件会误判整体偏移。改为总开口条件后，以上两轮不用于最终结果。
- 所有调试记录保留；comparison.json 中标记 superseded，不混入最终协议统计。最终协议位置组3轮、恒力组3轮，数量有限，不签发成功率结论。

## 文件和复现

日志根目录：/home/yep/s622_dual/log/c43_force_20261111（容器 /workspace/log/c43_force_20261111）。

- calibration_audit.json / calibration_points.json：27 点标定与运动边界。
- calibration_r1/、calibration_finger2_r1/：calibration.json、calibration_fit.json、ft_raw.txt、telemetry.csv、world/robot、启动/探针日志。
- comparison.json / comparison.csv：包含最终协议与 superseded 状态的全部结果。
- 每轮 instrumented_summary.json、wrench_telemetry.jsonl、两个 ft_finger*_raw.txt、contacts_raw.txt、contacts.jsonl、telemetry.csv、conditions.json、world/robot。
- 后续轮次保存脚本快照；所有轮次均保存了实际仿真输入和原始数据。

```bash
# 用新的目录名；脚本拒绝覆盖已有输出。
# finger1 标定（home + grasp）
docker exec -w /workspace s622_yep bash -lc \
  'source /opt/ros/humble/setup.bash; source install/setup.bash; python3 src/gz_launch/scripts/c43_force_lab.py /workspace/log/c43_force_20261111/NEW_CALIBRATION'

# finger2 抓取姿态标定
docker exec -w /workspace -e C43_FT_FINGER=2 -e C43_FT_GRASP_ONLY=1 s622_yep bash -lc \
  'source /opt/ros/humble/setup.bash; source install/setup.bash; python3 src/gz_launch/scripts/c43_force_lab.py /workspace/log/c43_force_20261111/NEW_CALIBRATION_2'

# 位置预紧
docker exec -w /workspace s622_yep bash -lc \
  'source /opt/ros/humble/setup.bash; source install/setup.bash; python3 src/gz_launch/scripts/c43_force_lab.py /workspace/log/c43_force_20261111/NEW_POSITION position .0249'

# 恒力驱动；自动使用已安装 ignition-transport11 / msgs8 编译 native publisher。
docker exec -w /workspace -e C43_CLAMP_N=5 s622_yep bash -lc \
  'source /opt/ros/humble/setup.bash; source install/setup.bash; python3 src/gz_launch/scripts/c43_force_lab.py /workspace/log/c43_force_20261111/NEW_FORCE force .0249'

docker exec -w /workspace s622_yep python3 \
  src/gz_launch/scripts/c43_force_lab_analyze.py log/c43_force_20261111
```

默认种子是上轮真正单臂模型目录，可用 C43_LAB_SEED 指向包含 robot.urdf/world.sdf 的归档副本。

新增诊断源码：c43_force_lab.py、c43_ft_load_probe.py、c43_instrumented_grip.py、c43_force_pair.cpp、c43_force_lab_analyze.py。调整了此前新增的单臂诊断 launcher，使 force 模式不启动手指位置控制器。没有修改正式执行器、冻结位姿或用户原有 FT/xacro 改动。

Python 编译、C++ 编译/实际发布、git diff --check 通过。结束时确认没有 c43-force-* 实验进程残留，原有 42896/42900 和旧 Gazebo 299501 保留。外载逐档清除，力驱动结束发送零值；每轮隔离进程组随后清理。
