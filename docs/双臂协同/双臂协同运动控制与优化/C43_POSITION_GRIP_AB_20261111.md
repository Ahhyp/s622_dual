# C43 位置夹持对照实测（2026-11-11）

## 结论与证据边界

**同一套 C43 左夹爪在真正单臂模型中也会滑落。位置命令确实能把样件短暂带起，但本轮没有得到 50 mm 完整跟随并保持的稳定结果。不能把问题概括为“双臂没有 effort 所以不能夹”。**

真正单臂模型删除了右侧全部 link、joint 和 right_GazeboSimSystem，并以裁剪后的 URDF 提供 robot_description；实际只加载一个硬件组件。两次 0.35 kg / 50 mm 宽样件，峰值分别抬高 46.184 / 25.006 mm，随后回落。没有 weld、ApplyJointForce、执行器或 QP 参与。

首次接触后 cancel 再保持实测停位，在 50 / 30 mm 宽度下均不能建立持续对向接触，抬升近零。保持稍向内的位置目标可以建立双侧接触并短暂带动物体，因此“首次接触成功”不等于“已经建立足够的夹持预紧”。法向力及实际摩擦能力尚未测定，不能据此断言力控一定解决问题。

本轮原始日志保留了一个重要混杂因素：右爪虽张开且零手指接触，部分被带高的样件仍撞到了静止右腕。这三轮只能作为受干扰记录，不能用于纯左爪能力或质量因果判断。该干扰来自本轮“只抬左臂、右臂静止”的对照方式，**不是已证实的历史双臂同步搬运根因**。

## 实验条件

- 宿主机 /home/yep/s622_dual；容器 s622_yep；ROS 2 Humble / Gazebo Fortress。
- ROS_DOMAIN_ID=145；每轮独立 IGN_PARTITION=c43-ab-<run>，依次运行，原有仿真保留。
- 从当前 C43 xacro 生成诊断 URDF/SDF，明确删除 DetachableJoint 和 ApplyJointForce，运行前断言生成 SDF 不含这两个插件。
- 手臂、手指都是 position 命令；控制参数沿用当前配置，position_proportional_gain=20。
- 关闭正式 homing/executor；诊断节点直接驱动 JTC。双臂模型先 right pre-grasp，再 left pre-grasp，再依次下降。单臂模型只有左臂相同的 pre-grasp/下降段。
- 左臂抬升反向使用已有 11 点下降路径，world Z +50 mm / 8 s；随后观察 5 s。实际腕部升高均约 50 mm。
- 样件长 120 mm、高 50 mm、中心 z=275 mm、摩擦等其余条件保持。宽度 50→30 mm 是**只改宽度的控制变量实验**，不是历史 30×30 mm、不同姿态长条的完整复现。
- 质量主组 0.35 kg；额外轻载组 0.03 kg，同时按尺寸、质量重新计算惯量。
- nominal：50 mm 宽、目标 ±0.0251 m。fixed：50 mm 宽目标 ±0.0249；30 mm 宽目标 ±0.0149，目标持续保留。
- cancel_hold：相同目标，首次手指接触后 cancel，等待 0.35 s，再明确下发实测手指位置并保持，无额外挤压。
- 两种宽度的接触组名义闭合速度相同（0.7 mm/s/指）：50 mm 约 8 s；30 mm 约 22.286 s。

## 全部完成轮次

“最终”相对抬升前稳定位置；峰值统计 lift/hold_lift 阶段。负值表示回落、倾斜后略低于初始中心，并非成功向下搬运。

| run | 模型 | 宽 mm | 质量 kg | 策略 | 峰值升高 mm | 5 s 观察末升高 mm | 审计 |
|---|---|---:|---:|---|---:|---:|---|
| w50_fixed_nominal_r1c | 双臂、仅左运动 | 50 | .35 | fixed ±.0251 | 0.000 | 0.000 | 无手指接触 |
| w50_fixed_contact_r1 | 双臂、仅左运动 | 50 | .35 | fixed ±.0249 | 15.942 | -0.071 | 无右臂接触；失去夹持后回落 |
| w50_cancel_hold_r1 | 双臂、仅左运动 | 50 | .35 | cancel_hold | 0.000 | 0.000 | 无右臂接触；finger2 短暂接触后离开 |
| w30_fixed_contact_r1 | 双臂、仅左运动 | 30 | .35 | fixed ±.0149 | 8.137 | -0.174 | 无右臂接触；回落 |
| w30_cancel_hold_r1 | 双臂、仅左运动 | 30 | .35 | cancel_hold | 0.001 | 0.000 | 无右臂接触；无持续双侧接触 |
| w50_fixed_contact_r2 | 双臂、仅左运动 | 50 | .35 | fixed ±.0249 | 35.108 | 34.932 | **撞静止右腕，受干扰** |
| w50_fixed_light_r1 | 双臂、仅左运动 | 50 | .03 | fixed ±.0249 | 37.064 | 36.052 | **撞静止右腕，受干扰** |
| w30_fixed_light_r1 | 双臂、仅左运动 | 30 | .03 | fixed ±.0149 | 33.042 | -0.008 | **撞静止右腕，受干扰** |
| single_w50_fixed_r1b | 真正单臂、单硬件 | 50 | .35 | fixed ±.0249 | 46.184 | -1.947 | 回落，无其他机器人接触 |
| single_w50_fixed_r2 | 真正单臂、单硬件 | 50 | .35 | fixed ±.0249 | 25.006 | -0.005 | 回落，无其他机器人接触 |
| single_w50_fixed_light_r1 | 真正单臂、单硬件 | 50 | .03 | fixed ±.0249 | 42.323 | 37.990 | 观察末仍持物，但约 12.01 mm 竖直滑移 |

11 个完整轮次中有 3 个受右腕碰撞干扰，8 个可用于无其他机器人接触的观测。轻载真正单臂只有一次，不作成功率或统计显著性结论。所有有效轮次 homing 阶段均未捕获手指-样件接触；全碰撞名字审计也保存于 comparison.json。

尚有 3 次启动/采集调试记录，不计入实验：w50_fixed_nominal_r1（诊断属性名与 rclpy Node 冲突）；w50_fixed_nominal_r1b（订阅了未实际发布的自定义 topic，中止）；single_w50_fixed_r1（诊断启动器缺插件路径，中止）。没有抹去或覆盖它们。

首个有效 nominal 轮次的节点已返回 0、清理已完成，但外层 shell 随后因运行期脚本修订报语法错误；该轮依据落盘原始数据判断，不以外层退出码判断。后续固定版本的执行与语法检查通过。

## 接触采集校验

Fortress 实际发布的原生话题为：

`/world/c43_s4_world/model/c43_sample/link/base_link/sensor/sample_contact/contact`

它没有使用本轮 SDF 指定的 /c43/position_ab/contact；因此改用实际话题，并保存发布者信息/有内容的原始 Contacts 消息。样件与支撑的接触流同时证明订阅有效，不能仅凭没有手指事件就判断零接触。

诊断解析器按空行分隔的**完整顶层 Contacts 消息**聚合其中各 contact 块，兼容两种 collision 顺序，分别记录两指和非手指机器人碰撞。旧 c43_contact_capture.py 的 same_message 是按 contact 块统计，历史 false 不能排除两个不同 contact 块处于同一顶层消息。

50 mm fixed .0249 的干净轮次，settle 阶段 595 条消息全部同时包含两指；lift 中也有 865 条双指消息，随后接触丢失。50 mm cancel_hold 只有 7 条顶层消息包含两指；保持阶段仅有 finger1。30 mm 对应为 fixed settle 597 条双指、cancel_hold close 5 条短暂双指。数据支持区分“接触后停止”和“有持续向内位置需求”的不同物理状态。

## FT 恒定读数的重新解释

此前读取到 force.x≈-0.0864948 N、torque.z≈0.00306797077 N·m。当前模型 left_finger1 的质量为 0.008826 kg，质心 y=0.03547 m：

此前原生样本归档：log/c43_resume_20261111T072912Z/existing_ft_samples.txt；本轮未施加 FT 标定载荷。

```
mg = 0.008826 × 9.8 = 0.0864948 N
mg × COM_y = 0.003067970556 N·m
```

数值与读数高度吻合，强烈提示当时主要测到手指自重及其力矩；这是一项模型与读数匹配的推断，尚不是外载标定。不能把此恒定值解释为夹持法向力，也不能单凭它证明传感器不响应外载。

## 复现与文件

所有日志在 /home/yep/s622_dual/log/c43_position_ab_20261111/，容器对应 /workspace/log/c43_position_ab_20261111/。

```bash
# 双臂模型，右爪打开，左爪位置保持；请使用新的 run 目录
docker exec -w /workspace s622_yep bash \
  src/gz_launch/scripts/run_c43_position_ab.sh \
  /workspace/log/c43_position_ab_20261111/NEW_DUAL_RUN 0.05 fixed 0.0249 0.35

# 真正单臂、单硬件对照
docker exec -w /workspace -e C43_AB_SINGLE=1 s622_yep bash \
  src/gz_launch/scripts/run_c43_position_ab.sh \
  /workspace/log/c43_position_ab_20261111/NEW_SINGLE_RUN 0.05 fixed 0.0249 0.35

# 从落盘原始数据重新汇总
docker exec -w /workspace s622_yep python3 \
  src/gz_launch/scripts/c43_position_ab_analyze.py log/c43_position_ab_20261111
```

每轮包含 world.sdf、robot.urdf、robot.sdf、conditions.json、launch.log、probe.log、summary.json、telemetry.csv、contacts_raw.txt、contacts.jsonl；后续轮次还保存了 probe_source.py、runner_source.sh 和话题发布者信息。总体为 comparison.json / comparison.csv。

新增源码仅为诊断：c43_position_grip_probe.py、run_c43_position_ab.sh、c43_position_single.launch.py、c43_position_ab_analyze.py。未修改冻结 q_grasp12、正式启动器、正式执行器或用户已有 FT 改动。

## 校验与后续

- Python 编译、bash 语法、git diff --check 均通过。
- 解析器独立校验通过：同一顶层消息两指、反向碰撞顺序、非手指机器人接触、时间戳。
- 从实际 world 核验质量，从实际 robot.urdf 核验硬件组件数，从 SDF 核验无 weld/施力插件，从原始接触消息审计右臂/其他 link。
- 本轮各隔离进程均已清理，原有 PID 42896/42900 和旧 Gazebo 299501 保留。

下一步应先在真正单臂场景中标定已知外载与传感器轴向、去掉重力基线，测出保持/滑移时的反力；随后比较有界的位置预紧和恒力夹持。需要单独验证接触几何、保持期间滑移与重复性，不能从一次峰值抬高推断稳定夹持，也不能直接把历史 60 N 配方移植到 C43。
