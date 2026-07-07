# MuJoCo 接触约束理论

## MuJoCo 官网对接触的说明（Computation）

本节基于 MuJoCo 官方文档 [Computation](https://mujoco.readthedocs.io/en/stable/computation/index.html) 中 `Soft contact model`、`Constraint model/Contact` 与 `Constraint solver` 对接触的描述。

### 1) 软接触建模立场

- MuJoCo 采用 soft contact（软接触）立场，不将经典 LCP 中的严格互补性作为必须条件。
- 在无摩擦情形下，凸优化形式与 LCP 的 KKT 条件等价；在有摩擦情形下，允许小幅互补违反（例如法向力与法向速度可同时为正）。
- 官方强调这类偏离在软材料接触中更符合物理直觉，并且有利于数值稳定与参数辨识。
- 模型在连续时间（力与加速度）中定义，而非依赖离散时间速度步进近似。

### 2) 接触在约束系统中的表示

- 接触是约束模型中的一类约束（与 equality、friction loss、limit 并列）。
- 每个活动接触会向系统级约束写入一个或多个标量约束行，对应 `mjData.efc_J`（Jacobian）、`mjData.efc_pos`（残差）和 `mjData.efc_force`（约束力）。
- MuJoCo 使用点接触：接触坐标系以接触点为中心，`x` 轴是法向，`y/z` 为切平面方向。
- 接触距离 `dist` 的语义为：正值分离、零值接触、负值穿透。

### 3) `condim` 的接触维度语义

`condim` 决定接触可生成的力/力矩分量维度：

- `condim=1`：仅法向（无摩擦）。
- `condim=3`：法向 + 两个切向摩擦分量。
- `condim=4`：在 `condim=3` 基础上增加绕法向的扭转摩擦。
- `condim=6`：进一步增加两方向滚动摩擦，可抑制持续滚动。

官方同时说明 `condim` 不取 `2` 或 `5`，因为切向与滚动方向按成对维度处理。

### 4) 摩擦锥：elliptic 与 pyramidal

设单接触维度为 \(n=\texttt{condim}\)，摩擦系数向量为 \(\mu\)。官方给出：

$$
\text{elliptic cone}:\quad
\mathcal{K}=\left\{f\in\mathbb{R}^n:\ f_1\ge0,\ f_1^2\ge\sum_{i=2}^n \frac{f_i^2}{\mu_{i-1}^2}\right\}
$$

$$
\text{pyramidal cone}:\quad
\mathcal{K}=\left\{f\in\mathbb{R}^{2(n-1)}:\ f\ge0\right\}
$$

- `elliptic` 对应二阶锥约束；`pyramidal` 对应非负变量锥（元素级不等式）。
- 对于 pyramidal，维度从 \(n\) 展开为 \(2(n-1)\)，本质是在椭圆锥边界上用棱边基向量做近似。

### 5) 接触 Jacobian 的几何构造

- 对单接触先构造 \(S\in\mathbb{R}^{6\times n_v}\)：将关节速度映射为接触点相对空间速度（在接触系表达）。
- 再由接触基矩阵 \(E\) 将接触力分量映射到空间力/力矩，接触 Jacobian 为：
  $$
  J_c=E^T S
  $$
- 该块 Jacobian 最终插入系统级 \(J\)（即 `efc_J`）参与整体求解。

### 6) 求解视角（官网表述）

- 前向动力学的接触力由一个凸优化问题定义：pyramidal 情况是带盒约束的 QP，elliptic 情况包含二阶锥约束。
- 逆动力学在该软约束框架下具有唯一解，并可分解为按约束/接触的独立子问题（包含解析处理）。
- MuJoCo 提供 `fwdinv` 检查（`mjModel.opt.enableflags`）用于比较前向与逆向动力学结果一致性，以监控数值求解收敛质量。

本文整理 MuJoCo的接触理论，mujoco中在接触实例化阶段把单个接触变成系统级约束行（`efc`），其中实现时分为3类分支：
- 无摩擦接触（frictionless），无论全局option设置为pyramidal还是elliptic，若设置dim=1，就只考虑法向接触，无摩擦
- 棱锥摩擦锥（pyramidal cone），若设置dim>1，且设置cone="pyramidal"，则为此情况
- 椭圆摩擦锥（elliptic cone），若设置dim>1，且设置cone="elliptic"，则为此情况
