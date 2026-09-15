# MuJoCo Flex 总体建模与配置

**本篇范围**：Flex 的统一对象语义、建模入口、两套形变范式、接触与求解数据流、配置速查。  
**不涵盖**：各维度的完整方程推导与源码行级映射（见专篇）。  
**延伸阅读**：[一维 Flex 理论和计算](mujoco一维Flex理论和计算.md) · [二三维 Flex 理论和计算](mujoco 二三维Flex理论和计算.md)

写法参考 `接触/` 目录：先物理/数学语义，再对应 MuJoCo 元素与参数。

---

## 0) Flex 是什么

MuJoCo 中的 **flex** 是“跨多个刚体参考系定义的可变形几何元集合”。

核心结构：

- **顶点（vertex）**：绑定到 body，随 body 运动；
- **元素（element）**：连接若干顶点的 **无质量可拉伸单元**；
- **边（edge）**：元素内部的边，用于长度度量、被动力与约束。

按 `dim` 划分基本单元类型：

| dim | 单元 | 顶点数/元素 |
|-----|------|------------|
| 1 | capsule（胶囊） | 2 |
| 2 | triangle（三角形） | 3 |
| 3 | tetrahedron（四面体） | 4 |

所有维度均可设 `radius`，使 1D/2D 元素具有体积性，并影响碰撞与渲染。

与普通 **geom** 的关键区别：flex 元会随顶点 body 独立运动而变形；碰撞接触力不再只作用于两个 body，而是按权重分配到元素涉及的多个顶点 body。

官方表述（[doc/modeling.rst](../../doc/modeling.rst)）：

> A flex is a collection of MuJoCo bodies that are connected with **massless stretchable elements**.

质量集中在 **顶点 body** 上（`flexcomp` 自动创建点质量体），不在边/元素上。

---

## 1) 与 geom / composite / cable 的边界

### 1.1 与 geom

| | geom | flex |
|---|------|------|
| 形状 | 刚体附着，不变形 | 顶点独立运动，实时变形 |
| 碰撞 | 标准刚体碰撞 | flex 专用碰撞管线，多 body 力分配 |
| 适用 | 刚性物体 | 绳、膜、软体等 |

可将 **所有顶点绑定同一 body** 构造 rigid flex，复用 flex 碰撞处理非凸 mesh（与 convexified mesh geom 不同）。

### 1.2 与 composite

**composite** 是较早的软体建模宏（tendon + 约束组合）；**flex** 是 MuJoCo 3.0+ 的统一可变形体框架，大尺度 flex 更高效。两者用途部分重叠，flex 可能逐步替代 composite，但目前各有适用场景。

### 1.3 与 cable（1D 选型）

| 对象 | 伸长性 | 主要力学 | 适用场景 |
|------|--------|---------|---------|
| **1D flex** | 可伸长 | 边弹簧/阻尼 + 可选边长约束 | 橡皮筋、可伸长细线、受拉显著的结构 |
| **cable** 插件 | 不可伸长 | 弯曲 + 扭转 | 电缆、皮带、弹簧钢丝 |

- `rope`/`loop` composite 已弃用；不可伸长杆用 `cable`，可伸长细长体用 **1D flex**。
- 若“本应不可伸长却被拉长很多”，考虑改用 `cable` 或加强 `equality/flex` / `edge stiffness`。

1D 细节见 [一维专篇 §0](mujoco一维Flex理论和计算.md)。

---

## 2) 建模入口：`flex` vs `flexcomp`

| 入口 | 层级 | 用途 |
|------|------|------|
| `deformable/flex` | 低层运行时表达 | 手工指定 `body` / `vertex` / `element` / `radius` |
| `worldbody/flexcomp` | 高层建模宏 | 按规则自动生成点、拓扑、body、低层 flex |

**flexcomp 自动生成**：

- 一组点质量 body（通常每点 3 个 slide DOF，或径向 1 DOF）；
- `deformable/flex` 低层定义；
- 可选 `equality/flex`（锁定所有边到初始长度）。

**实践建议**：

- 初次建模优先 **flexcomp**（快、稳、少出错）；
- 需精确控制拓扑或顶点-body 绑定关系时，直接写 **deformable/flex**。

---

## 3) 统一拓扑语义

### 3.1 低层 `deformable/flex` 关键属性

- `dim`：1 / 2 / 3，决定元素类型。
- `body`：每个顶点所属 body（单个 body 时退化为 rigid flex）。
- `vertex`：顶点在各自 body 局部坐标系下的位置。
- `element`：每个元素 `(dim+1)` 个顶点索引。
- `radius`：1D 必须为正；2D/3D 可选，影响碰撞体积与渲染。

### 3.2 高层 `flexcomp` 常用属性

- `type`：`grid` / `circle` / `mesh` / `gmsh` / `direct` 等。
- `count` / `spacing`：规则网格生成（如 1D 点列、3D 体网格）。
- `mass` / `inertiabox`：总质量均分到各非 pinned 顶点 body。
- `pin`：固定某些点到父 body（不生成独立子 body）。
- `dof`：`full`（3 slide）或 `radial`（1 slide）。

1D 常用：`type="grid" dim="1"` 生成点链；`type="circle" dim="1"` 生成闭环。

---

## 4) 两套形变范式（核心）

MuJoCo 用 **同一套 flex 拓扑**，提供两种保形思路（[doc/modeling.rst](../../doc/modeling.rst) “Deformation model”）：

### 4.1 Edge-based（边模型，lumped stiffness）

- 通过 `flex/edge` 的 `stiffness` / `damping` 与 `equality/flex` 控制边长行为。
- 边力为标量弹簧-阻尼：$f = k(l_0 - l) - c\dot l$，经边 Jacobian 映射到 `qfrc`。
- 官方称为 **"lumped" stiffness model**——变形模态（如剪切、体积）耦合被平均到边长标量中。

### 4.2 Continuum（连续体，StVK PL-FEM 等价）

- 通过 `flex/elasticity` 指定 `young`、`poisson`、`damping`（及 2D 的 `thickness`）。
- **Saint Venant-Kirchhoff** 超弹性，分片线性有限元离散（piecewise linear FEM）。
- 编译期预计算单元刚度度量；运行时用边长平方梯度组装力。
- 实现为 **coordinate-free 离散几何形式**，不显式写形函数 $N_i(x)$，但等价于线性有限元。

### 4.3 dim 路由表（运行时主路径）

| dim | 单元 | 形变主路径 | 主要 XML 参数 | 运行时核心 |
|-----|------|-----------|--------------|-----------|
| **1** | capsule | **edge 弹簧/阻尼 + equality/flex** | `edge stiffness/damping` | `flexedge_J` + `engine_passive.c` 边力块 |
| **2** | triangle | **elasticity (StVK)** + 可选 bending | `young/poisson/damping/thickness` | `ComputeStiffness` + `GradSquaredLengths` |
| **3** | tetrahedron | **elasticity (StVK)** | `young/poisson/damping` | 同上 |

**重要**：`edge stiffness` **仅 dim=1** 可用；2D/3D 设 edge stiffness 会编译报错，需用 `elasticity` 或插件。  
**重要**：1D 运行时 **`dim==1` 跳过 FEM 弹性块**（`engine_passive.c`）；`young/poisson` 对 1D 不触发 `ComputeStiffness`。

### 4.4 为何没有形函数？

MuJoCo flex **不采用**经典 FEM 流程 $u^h(x)=\sum N_i(x)u_i$ → 应变 → 刚度矩阵 $K$。

原因概括：

1. **配置变量在顶点 body 的关节上**，单元内部无独立连续位移场；质量 lumped 在顶点。
2. **1D 仅关心轴向伸长**，标量边长 $l_e=\|x_j-x_i\|$ 足够，无需单元内插值。
3. **2D/3D 采用离散几何 FEM 等价形式**：边长平方梯度 + 预计算 9×9 度量张量（Weischedel / Kharevych 风格），而非显式 $N_i(\xi)$ 与高斯积分。

维度专篇展开：

- 1D：边长标量驱动 → [一维专篇 §2](mujoco一维Flex理论和计算.md)
- 2D/3D：StVK + 度量张量 → [二三维专篇 §2–3](mujoco 二三维Flex理论和计算.md)

---

## 5) 统一接触与求解数据流

从仿真管线看，flex 的力来自三类：

1. **contact forces**：flex 元参与碰撞检测与接触求解；
2. **constraint forces**：如 `equality/flex` 边长约束、关节限位等；
3. **passive forces**：`edge stiffness/damping`（1D）或 `elasticity`（2D/3D）产生的被动力。

关键点：

- flex 接触不是“两刚体点接触”：接触力按 **权重** 分配到元素涉及的顶点 body（`mj_elemBodyWeight`、`mj_contactJacobian`）。
- 系统层面仍回到广义力 `qfrc`，中间经过 **元素 → 顶点 → body** 映射。
- 约束统一写入 `efc_J`（Jacobian）、`efc_pos`（残差）、求解得 `efc_force` → `qfrc_constraint`。

前向动力学汇总：

$$
M(q)\ddot q = \tau_{\text{smooth}} + J^\top\lambda
$$

其中 $\tau_{\text{smooth}}$ 含 passive、bias、applied、actuator 等；$\tau_{\text{constraint}}=J^\top\lambda$ 含接触与等式约束。

对应实现：`src/engine/engine_forward.c`、`engine_core_constraint.c`、`engine_solver.c`。

---

## 6) 配置速查

### 6.1 形变相关子元素

| 子元素 | 作用 | 适用 dim |
|--------|------|---------|
| `flex/edge` `stiffness` | 边向弹簧被动力 | **仅 1** |
| `flex/edge` `damping` | 边向阻尼 | 全部（语义不同：1D 边阻尼；2D/3D 常与 equality 配合） |
| `flex/edge` `equality="true"` | 编译期添加 `equality/flex` | 全部 |
| `flex/elasticity` | StVK 材料参数 | **2D/3D 主路径**（1D 运行时不用） |
| `equality/flex` | 软等式约束固定边长 | 全部 |

### 6.2 `flex/elasticity` 主要参数

- `young`：杨氏模量（pressure 单位）。
- `poisson`：泊松比 $[0, 0.5)$。
- `damping`：Rayleigh 阻尼系数（时间单位），与刚度配合得阻尼矩阵。
- `thickness`：2D shell 厚度，缩放拉伸刚度。

### 6.3 `flexcomp/contact`

- `condim`、`solref`、`solimp`：接触求解参数；
- `selfcollide`：自碰撞剪枝策略（大 flex 必需）。

### 6.4 建模组合建议

- **1D 可伸长线**：`edge stiffness/damping`；近不可伸时可加 `edge equality="true"`。
- **2D 膜/壳**：`elasticity young/poisson/thickness`；大 flex 关闭或剪枝自碰撞。
- **3D 软体**：`elasticity` + 适当 `timestep`；可参考 `doc/modeling.rst` 3D grid 示例。

---

## 7) 官方锚点与示例索引

| 资源 | 内容 |
|------|------|
| [doc/modeling.rst](../../doc/modeling.rst) | Deformable objects 总体设计、两类形变模型 |
| [doc/XMLreference.rst](../../doc/XMLreference.rst) | `flexcomp`、`deformable/flex`、`edge`、`elasticity`、`equality/flex` |
| [doc/computation/index.rst](../../doc/computation/index.rst) | 系统方程、`efc_J`、约束求解 |
| [model/flex/pulley.xml](../../model/flex/pulley.xml) | 1D 圆环 + `edge equality` |
| `doc/modeling.rst` 3D grid 示例 | 3D tetra + `elasticity` |

### 关键源码（文件级）

| 主题 | 文件 |
|------|------|
| 边长与 `flexedge_J` | `src/engine/engine_core_smooth.c` (`mj_flex`) |
| 1D 边弹簧 / 2D/3D 弹性 | `src/engine/engine_passive.c` |
| 边长等式约束 | `src/engine/engine_core_constraint.c` (`mjEQ_FLEX`) |
| flexcomp 编译 | `src/user/user_flexcomp.cc` |
| 2D/3D 刚度预计算 | `src/user/user_mesh.cc` |
| flex 碰撞 | `src/engine/engine_collision_driver.c` |

---

## 8) 延伸阅读

- **[一维 Flex 理论和计算](mujoco一维Flex理论和计算.md)**：边弹簧/阻尼、`equality/flex`、质量分配、完整方程推导、1D 示例与调参。
- **[二三维 Flex 理论和计算](mujoco 二三维Flex理论和计算.md)**：StVK、`ComputeStiffness`、`GradSquaredLengths`、2D 弯曲、Rayleigh 阻尼、插件边界。
