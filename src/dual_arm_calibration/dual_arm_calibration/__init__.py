"""M2.6 双基座标定 Monte Carlo 仿真包。

纯 Python 数值仿真（不依赖 Gazebo 渲染）：
  - base_alignment: Kabsch/SVD 刚性对齐求解 {}^{B_L}T_{B_R}
  - monte_carlo: 噪声注入（contact / TCP / FK）→ 采样数→P95 曲线 + 空间覆盖对比
"""
