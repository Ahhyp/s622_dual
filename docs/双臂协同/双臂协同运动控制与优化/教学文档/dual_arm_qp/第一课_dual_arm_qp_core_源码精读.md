# 第一课：`dual_arm_qp_core` 源码精读

> S622 双臂协同控制学习笔记。以 `public` 分支源码为准，核对日期：2026-10-16。  
> 本课的四个阶段**都属于第一课**：A. 认识类和 C++；B. 理解 QP 数学；C. 顺着 `solve()` 读代码；D. 连接 OSQP 与 ROS 2 控制器。

## 本课的核心目标

读完之后，你应该能回答：

1. 为什么 QP 接收 12 个关节角，却优化 24 个变量？
2. `DualArmQpParams`、`DualArmQpResult` 和 `DualArmQp` 分别负责什么？
3. 任务误差如何变成 `P`、`q`、`A`、`l`、`u`？
4. `solve()` 返回的 `dq` 如何变成 12 轴 position command？
5. 为什么 Core 中存在物体任务，不代表控制器已经启用了物体跟踪？

学习主线：

```text
q_state（12轴）
   ↓
DualArmKinematics（FK、相对误差和Jacobian）
   ↓
DualArmQp::solve()（建立 P, q, A, l, u）
   ↓
QpSolver / OSQP（数值优化）
   ↓
dq（12轴每周期关节增量）
   ↓
dual_arm_qp_controller::update()（q_cmd += dq）
   ↓
12个 position command interfaces
```

### 应同时打开的源码

- [dual_arm_qp.hpp](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/include/dual_arm_qp_core/dual_arm_qp.hpp)
- [dual_arm_qp.cpp](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/src/dual_arm_qp.cpp)
- [qp_types.hpp](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/include/dual_arm_qp_core/qp_types.hpp)
- [osqp_solver.cpp](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_core/src/osqp_solver.cpp)
- [dual_arm_qp_controller.cpp](https://github.com/Ahhyp/s622_dual/blob/public/src/dual_arm_qp_controller/src/dual_arm_qp_controller.cpp)

---

# 阶段 A：认识类和 C++ 接口

## A1. 先读头文件，不要立即陷进算法

`dual_arm_qp.hpp` 给出了程序向外承诺提供的功能。阅读时先区分三个结构：

| 名称 | 作用 | 类比 |
|---|---|---|
| `DualArmQpParams` | 保存权重、增益、限位、额外约束 | 调节旋钮 |
| `DualArmQp` | 根据当前状态建立并求解优化问题 | 决策机器 |
| `DualArmQpResult` | 返回关节增量、求解状态、误差指标 | 本轮决策报告 |

先看 Eigen 的类型：

```cpp
using Vec12 = Eigen::Matrix<double, 12, 1>;
using Vec6  = Eigen::Matrix<double, 6, 1>;
```

`double` 是数据类型，后两个数字分别是**行数、列数**。`Vec12` 可装 12 个关节的数据，`Vec6` 可装 6 维位置/姿态任务的数据。`Q12`、`Iso3` 来自 `dual_arm_kinematics`，具体约定应在阅读运动学包时核对。

常见语法：

- `const Q12& q`：以常量引用接收关节向量，避免不必要的复制。
- `Vec12::Zero()`：生成全零的 12 维向量。
- `.block<6,12>(0,0)`：从第 0 行、第 0 列开始，选取 6 行 12 列的小矩阵。
- `private:`：类的内部成员，外部代码不能直接访问。
- `solver = nullptr`：允许调用时不手动指定求解器。
- `std::shared_ptr<QpSolver>`：求解器的共享智能指针，可替换实现。

## A2. 参数不是越多越好，先按功能分类

| 参数 | 代码中的意图 |
|---|---|
| `dt` | 每轮控制的时间间隔，默认 `0.008` 秒 |
| `kp_rel` | 相对位姿误差反馈增益 |
| `w_rel` | 相对任务权重 |
| `w_slack` | 相对任务 slack 的惩罚权重 |
| `w_center` | 喜欢某个关节增量方向的程度 |
| `w_reg` | 对过大增量的正则化 |
| `kp_obj`、`w_obj` | 物体跟踪参数，`w_obj=0` 默认关闭物体任务 |
| `w_slack_obj` | 物体任务 slack 惩罚 |
| `w_null_margin` | 倾向远离关节限位 |
| `w_null_manip` | 倾向更好的相对运动可操作性 |
| `w_null_clear` | 倾向更大的简化双臂间距；**不是完整避碰安全** |
| `enable_singularity` 等 | 在相对任务状态较差时平滑退化 |
| `q_min`、`q_max`、`v_max` | 关节角度及速度硬约束 |
| `C_ineq`、`d_ineq` | 额外线性不等式约束 |

注意：**Core 里声明一个参数，并不意味着 ROS 2 控制器里已经读取了同名 YAML 参数。**

## A3. `DualArmQpResult` 是算法层返回值

最关键的成员：

```cpp
Vec12 dq = Vec12::Zero();
Vec6 slack = Vec6::Zero();
Vec6 slack_object = Vec6::Zero();
bool converged = false;
int iterations = 0;
double solve_time_ms = 0.0;
```

其中 `dq` 是**一个控制周期内的关节增量**，不是每秒速度。`converged` 表示求解器是否报告成功，不等价于真实物理系统安全。

## A4. `solve()` 的三种函数重载

头文件约第 105—115 行声明了三个 `solve()`：

1. `solve(q, relative_target)`；
2. `solve(q, relative_target, object_target)`；
3. `solve(q, relative_target, object_target, object_twist_ff)`。

源码约第 83—92 行显示：前两个重载最后都会转发到第三个完整版。这叫**函数重载**：调用参数个数不同，但复用同一套核心求解实现。

不要忽略：第一种重载会调用 `kin_.worldTobject(q)` 取得当前物体位姿。即使 `w_obj=0`，完整版当前仍计算物体误差和 `objectJacobianBody()`。因此不能将“物体任务关掉”理解为“绝不触碰物体中点相关计算”。

### A 阶段检验

请尝试向别人解释：`DualArmQpParams` 和 `DualArmQpResult` 哪个是输入配置，哪个是输出报告？为什么 `solver_` 不直接写死为 OSQP 类型？

---

# 阶段 B：QP 数学与 Eigen 矩阵

## B1. 机器人有 12 个关节，为什么 QP 却有 24 个变量？

源码明确规定：

$$
z=\begin{bmatrix}
\Delta q_{12}\\
s_{\mathrm{rel},6}\\
s_{\mathrm{obj},6}
\end{bmatrix}.
$$

前 12 个是本轮关节增量。中间 6 个是相对任务松弛量（slack）。最后 6 个是物体任务松弛量。**slack 不是新的机械关节，而是优化器为了处理任务冲突引入的数学变量。**

```cpp
constexpr int kNumVars = 24;
constexpr int kSRel0 = 12;
constexpr int kSObj0 = 18;
```

`kSRel0` 是相对 slack 在变量向量中的起点，`kSObj0` 是物体 slack 的起点；索引从零开始。

## B2. 相对任务的矩阵

假设 `J_rel` 是 6×12：

$$
A_{\mathrm{rel}}=
\begin{bmatrix}
J_{\mathrm{rel}}&-I_6&0
\end{bmatrix}
\in\mathbb{R}^{6\times24}.
$$

于是：

$$
A_{\mathrm{rel}}z=J_{\mathrm{rel}}\Delta q-s_{\mathrm{rel}}.
$$

再定义：

$$
\alpha_{\mathrm{rel}}=k_{p,\mathrm{rel}}dt,\qquad
b_{\mathrm{rel}}=-\alpha_{\mathrm{rel}}e_{\mathrm{rel}}.
$$

代码优化的相对残差是：

$$
r_{\mathrm{rel}}=A_{\mathrm{rel}}z+b_{\mathrm{rel}}.
$$

这里的误差方向由实际 `relativeError()` 的定义决定。不要脱离代码的 frame/正负号约定直接照搬其他资料的公式。

物体任务类似：

$$
A_{\mathrm{obj}}=[J_{\mathrm{obj}},0,-I_6],
$$

并加入物体跟踪反馈和前馈速度。当前旧 `objectJacobianBody()` 在 π 附近曾被审计指出问题，因此**“可以拼进 QP”不代表 C4.3 可以不经修复就启用它**。

## B3. QP 的 `P`、`q_cost` 从哪里来？

一般的二次规划写成：

$$
\min_z\frac12 z^\top Pz+q_{\mathrm{cost}}^\top z,
\quad l\le Az\le u.
$$

对一个平方残差：

$$
\frac w2\|Bz+b\|^2
=\frac w2z^\top B^\top Bz+w\,b^\top Bz+\text{常数}.
$$

所以：

$$
P\mathrel{+}=wB^\top B,\qquad
q_{\mathrm{cost}}\mathrel{+}=wB^\top b.
$$

这对应真实代码的结构：

```cpp
P_ = params_.w_rel *
     (A_task_rel_.transpose() * A_task_rel_);
q_cost_ = params_.w_rel *
     (A_task_rel_.transpose() * bias_rel);
```

重要：标准 QP 前有 `1/2`，所以从数学推导到代码应检查系数是否一致；头文件注释是概括式表述，判断真实行为应以矩阵构造为准。

## B4. 正则项在做什么？

Core 还加入：

$$
\frac{w_{\mathrm{center}}}{2}\|\Delta q-\Delta q_{\mathrm{center}}\|^2
+\frac{w_{\mathrm{reg}}}{2}\|\Delta q\|^2
+\frac{w_{\mathrm{slack}}}{2}\|s_{\mathrm{rel}}\|^2.
$$

它们分别表达：

- 尽量按偏好的次级方向运动；
- 不要产生不必要的巨大关节增量；
- 尽量不要用 slack 放弃相对任务。

这些项主要体现在 `P_` 的对角块以及 `q_cost_` 的前 12 个元素。

## B5. 硬约束为什么要同时检查速度和角度？

单关节的两组要求：

$$
-v_{\max}dt\le\Delta q\le v_{\max}dt
$$

以及：

$$
q_{\min}-q\le\Delta q\le q_{\max}-q.
$$

取交集：

$$
\ell=\max(-v_{\max}dt,q_{\min}-q),\qquad
u=\min(v_{\max}dt,q_{\max}-q).
$$

若单步最大转 `0.02 rad`，但再转 `0.01 rad` 就碰到关节上限，那么本轮正方向最多只能转 `0.01 rad`。

源码将这些边界放入 `l_scratch_`、`u_scratch_`，约束矩阵前 12 行写入 `[I12,0]`。额外的 `C_ineq` 作为后续的行加入。

注意：代码对已经越界的关节存在特殊的 `lo > hi` 处理，应作为安全边界审查点，而不是理解成“数学必然可行”。

### B 阶段检验

请自己推导：为什么 `A_task_rel_` 必须是 6×24？为什么 slack 对应的位置填 `-I6`？

---

# 阶段 C：逐段读 `dual_arm_qp.cpp::solve()`

| 源码行 | 作用 | 应该观察什么 |
|---|---|---|
| 83–92 | `solve()` 重载转发 | 谁提供物体目标？ |
| 93–106 | 当前误差和 Jacobian | frame 是哪个？ |
| 107–139 | 奇异值及大误差平滑退化 | 改了哪些权重与速度界？ |
| 140–162 | 构造相对任务和物体任务 | `w_obj` 是否大于零？ |
| 163–206 | 计算零空间次级方向 | 近似零空间如何投影？ |
| 207–210 | 加入正则和 slack 惩罚 | 对应哪个 `P` 的块？ |
| 211–241 | 关节 box 约束及额外不等式 | 速度限位怎样乘以 `dt`？ |
| 242–251 | 生成 `QpProblem` 并求解 | `last_problem_` 内容是什么？ |
| 252–275 | 解析结果、clamp、warm start | 求解失败由谁处理？ |

## C1. 奇异退化不是 C4.3 的 π 问题修复

`DualArmQp` 有 `smoothstep01()` 和 `sigmaMinRel()`，可以在相对 Jacobian 的最小奇异值较小时平滑改变阻尼、slack 权重和速度上限。这主要是**运动控制中的病态/奇异处理**。

而我们之前讨论的 π 问题是**对称旋转中点的定义在 180° 相对旋转时不唯一**。两个问题名字相似、成因不同，不可以指望调大阻尼自动修好错误的物体 Jacobian。

## C2. 代码中的“零空间”怎么来？

次级目标可能来自：

- 离关节极限远一些；
- 提高相对 Jacobian 的最小奇异值；
- 提高简单的双臂间距指标。

代码先把多个改善方向相加，再使用带阻尼的投影矩阵：

$$
N=I-J^\top(JJ^\top+\lambda^2I)^{-1}J.
$$

最后把结果放到 `dq_center_scratch_`，作为 QP 的偏好方向。这里是**带阻尼的近似零空间投影**，不等于在所有情况下实现严格层级 QP。

还有一个很关键的源码事实：`clearanceProxy()` 注释明确标识它只是 comfort objective，**不是安全碰撞检测**。

## C3. 求解过程和输出

QP 被装进：

```cpp
last_problem_.P = P_;
last_problem_.q = q_cost_;
last_problem_.A = A_scratch_;
last_problem_.l = l_scratch_;
last_problem_.u = u_scratch_;
```

随后：

```cpp
const QpSolution solution = solver_->solve(last_problem_);
```

程序把解的前 12 个元素取为 `dq`，另外两块分别取为两种 slack。对于只有 box 约束的情况，后处理还会将 `dq` 限制在上下界内；存在一般不等式时则避免此 clamp，以免破坏其他约束。

Core 会存储 warm start 初值供下轮使用。这里**存上次答案作初值**和**底层 OSQP 工作区长期复用**是两回事。

### C 阶段检验

找出 `objectJacobianBody(q)` 的调用位置。即使 `object_on` 为 `false`，它还会执行吗？这对 π 问题意味着什么？

---

# 阶段 D：求解器及 ROS 2 控制器

## D1. `qp_types.hpp` 是算法和求解器之间的契约

`QpProblem` 提供 `P`、`q`、`A`、`l`、`u`、`x0`。`QpSolution` 提供解向量、收敛标志、迭代次数、求解耗时等。

`QpSolver` 是抽象基类：

```cpp
class QpSolver {
public:
    virtual ~QpSolver() = default;
    virtual QpSolution solve(const QpProblem& problem) = 0;
    virtual std::string name() const = 0;
};
```

`virtual` 使不同求解器可以实现统一接口；`=0` 是纯虚函数声明。因此 Core 不必知道底层究竟是 OSQP 还是其他 backend。

## D2. OSQP 后端实际做了什么？

目前读取的 `osqp_solver.cpp` 按顺序：

1. 验证 QP 问题；
2. 将 Eigen 矩阵转换为 CSC 稀疏形式（`P` 只取上三角）；
3. 为 OSQP 分配数据；
4. 调用 `osqp_setup()`；
5. 若传入初值，则调用 `osqp_warm_start_x()`；
6. 调用 `osqp_solve()`；
7. 取结果、清理 workspace、释放内存。

**源码发现：**当前 `OsqpSolver::solve()` 每轮都 `osqp_setup()`、`osqp_cleanup()`。所以虽然 QP 变量维数固定且使用 warm start，仍不能称为“已实现跨周期持久 OSQP workspace”。

## D3. 控制器怎么把 `dq` 送给机械臂？

在已读取的 `dual_arm_qp_controller.cpp` 里：

- `on_init()` 声明参数，并确定 12 个 position state / command interface；
- `on_configure()` 建立运动学和 QP 对象；
- `on_activate()` 将当前左右夹爪相对位姿记录为目标；
- `update()` 周期性读取关节状态、做健康检查、调用 `qp_->solve()`、检查求解是否收敛。

实际关键步骤：

```cpp
const auto result = qp_->solve(q_state_, target);
if (!result.converged) {
    ++solve_fail_streak_;
    writeCommand(q_cmd_);
    return return_type::OK;
}
q_cmd_ += result.dq;
clampToLimits(&q_cmd_);
writeCommand(q_cmd_);
```

这说明算法层求 `dq`，执行层累加为 `q_cmd`，最终写的是 **position command interface**。

一个容易忽略的边界：控制器当前调用的是 `solve(q_state_, target)`，只传相对位姿目标；在读到的该版本控制器 `on_init()` / `on_configure()` 中，**并没有真正启用 Core 的 `w_obj`、物体目标入口**。这也是为什么 C4.3 规划要单列 adapter 工作。

控制器还有 D7 健康监控：如果任一臂状态异常，进入双臂 hold，停止正常轨迹推进。这个软件逻辑不是设备级急停，更不能单凭它证明真实夹持力安全。

## D4. 设计目标与当前源码状态

| 设计/注释中出现的功能 | 现有已读取源码中的状态 |
|---|---|
| 12 轴 QP | Core 和 Controller 都存在 |
| 相对位姿任务 | Core 可计算，Controller 在 `update()` 调用 |
| 物体轨迹任务 | Core 中实现；Controller 目前没有对应目标输入 |
| 24 维优化变量 | Core 固定使用 |
| 零空间优化 | Core 有实现；Controller 当前未显式配置开启相关权重 |
| 奇异处理 | Core 中有开关；Controller 当前未显式读取相应开关 |
| OSQP warm start | 可传初值；workspace 每次求解仍重建 |
| 简化 clearance proxy | 仅是次级偏好，不是完整碰撞安全证明 |
| 求解失败保护 | Controller 不累加未收敛结果 |
| 动态物体 Jacobian 的 π 问题 | 仍需依照 C4.3 S1 的单独验证和修复 |

---

# 本课结业小测

不看讲义，试着回答：

1. `Vec12`、`Vec6` 和 `Iso3` 各代表什么？
2. 为什么 12 关节需要 24 个 QP 变量？
3. `dq` 与 `qdot` 有什么不同？
4. 把 `w/2 * ||Bz+b||²` 写为 `P`、`q_cost` 时，为什么会出现 `B.transpose()*B`？
5. 为什么要同时有速度限位与关节位置限位？
6. 为什么 `clearanceProxy()` 不能代替 FCL？
7. 为什么不能把 warm start 理解为 OSQP workspace 持续存在？
8. Core 有 `objectJacobianBody()`，为什么当前 Controller 还不能直接完成 C4.3 的物体轨迹任务？

**过关标准：**你能自己画出 `state → J/e → P,q,A,l,u → solver → dq → q_cmd`，并在源文件中指出每一步大致位置。

---

## 下一课预告

第二课建议研究 `dual_arm_qp_core` 的 **QP solver 抽象和测试体系**，也可以先切到 `dual_arm_kinematics` 把 `relativeError()`、`relativeJacobian()` 逐个推导清楚。原则是：学完一个核心链路再换主题。

> 注：这份讲义是阅读当前公开源码后编写的代码导读，未将代码编译或 Gazebo 验证结果冒充为本次运行实测。
