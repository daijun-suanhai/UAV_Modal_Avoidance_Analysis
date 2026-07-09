# 无人机模态与避频分析 — 第三方库选型 & 输入数据规划

## Context

为 V60 阿黛尔(AdeleX-10) 4+1 垂起固定翼无人机开发模态分析与避频分析系统。系统需支持：M-01 悬臂梁验证、M-02 板壳验证、M-03 旋翼支架避频（核心）、M-04 挂载状态对比。采用"优先第三方库"策略。

---

## 1. 第三方库全景

### 1.1 立即可用的库（环境实测通过）

| 库名 | 版本 | 用途 | 状态 |
|------|------|------|------|
| **CalculiX** | 2.21 | FEM 求解器 — 自动 K/M 组装 + Lanczos 特征值求解 | ✅ `apt install calculix-ccx` |
| **Gmsh** | 4.13.1 | 1D/2D/3D 网格生成，Python API | ✅ 可用 |
| **meshio** | 5.3.5 | 多格式网格读写，Gmsh ↔ CalculiX 桥接 | ✅ 可用 |
| **PyVista** | 0.45.2 | 3D 振型可视化、变形图渲染 | ✅ 可用 |
| **matplotlib** | 3.10.3 | Campbell 图、频率对比图、报告图 | ✅ 可用 |
| **numpy** | 2.4.4 | 基础数值计算 | ✅ 可用 |
| **scipy** | 1.15.3 | 数据处理、验证计算 | ✅ 可用 |
| **pandas** | — | 输入参数表格化管理、结果整理 | ✅ 可用 |

> **核心理念变更**: 不再手动组装 K/M 矩阵。用户通过 Python API 描述物理模型（节点、材料、截面、质量），
> 生成 CalculiX `.inp` 文件，CalculiX 自动完成 K/M 组装和特征值求解。
> 详见 [drone_fem/data/inp_template.md](drone_fem/data/inp_template.md)。

### 1.2 高性能特征值求解 — 自定义 PETSc/SLEPc 环境

你在 `~/.local/modalx/` 下有一套**含 Intel MKL PARDISO 的自定义 PETSc/SLEPc**：

| 组件 | 版本 | 路径 | 特性 |
|------|------|------|------|
| **PETSc** | 3.23.3 | `~/.local/modalx/petsc/3.23.3/lnx-x64-gcc-ompi-nogpu-rel-sh/` | MKL BLAS, **PARDISO**, SuperLU_DIST, SuiteSparse, hypre |
| **SLEPc** | 3.23.3 | `~/.local/modalx/slepc/3.23.3/lnx-x64-gcc-ompi-nogpu-rel-sh/` | 与上述 PETSc 配套 |
| **MKL** | 2025.3 | `/opt/intel/oneapi/mkl/2025.3/lib/` | PARDISO 稀疏直接求解器 |

**实测验证**：
- `nm -D libpetsc.so | grep pardiso` → 包含 `MatGetFactor_aij_mkl_pardiso`、`pardisoinit` 等符号 ✅
- 链接了 `libmkl_intel_lp64.so.2`、`libmkl_core.so.2` ✅

**⚠️ 需要一步编译**：当前 pip 安装的 `petsc4py 3.24.6` 和 `slepc4py 3.24.1` 是链接到另一个 PETSc（无 PARDISO）的。要使用 PARDISO，需重新编译 Python 绑定：

```bash
# 加载自定义 PETSc/SLEPc 环境
source ~/.local/modalx/petsc/3.23.3/lnx-x64-gcc-ompi-nogpu-rel-sh/env_petsc.sh
source ~/.local/modalx/slepc/3.23.3/lnx-x64-gcc-ompi-nogpu-rel-sh/env_slepc.sh

# 设置 PETSC_DIR（petsc4py 编译时需要）
export PETSC_DIR=$PETSC_PREFIX
export PETSC_ARCH=linux-real-opt

# 从源码编译 petsc4py 和 slepc4py (版本需匹配 PETSc 3.23.3)
pip install --no-binary :all: petsc4py==3.23.0
pip install --no-binary :all: slepc4py==3.23.0
```

> **备注**：此步骤约需 2-5 分钟编译时间。在此之前，可先用 `scipy.sparse.linalg.eigsh` 进行小-中规模特征值求解，开发不受阻塞。

### 1.3 不使用但值得说明的

| 库名 | 原因 |
|------|------|
| FEniCS/DOLFIN | 仅有 fenics-ufl/ffc 元包，无 dolfin 求解器，不可用 |
| scikit-fem | 缺少内置梁单元和壳单元，需手写矩阵公式 — 已改用 CalculiX |

---

## 2. 核心库选择与理由

### 2.1 选型决策矩阵

| 需求 | 候选方案 | 选中 | 理由 |
|------|---------|------|------|
| **FEM 求解** | scikit-fem / FEniCSx / CalculiX | **CalculiX** | 内置 B32梁/S4壳/MASS集中质量，自动 K/M 组装 + 特征值求解，用户不写矩阵 |
| **Python 集成** | subprocess / 直接调用 | **subprocess ccx** | Python 生成 .inp → ccx 求解 → 解析 .dat/.frd |
| **网格生成** | Gmsh / CalculiX 内置 | **Gmsh** | 支持 1D/2D/3D、Python API 完善、可参数化建模 |
| **3D 可视化** | PyVista / matplotlib 3D | **PyVista** | 专业 FEM 后处理，振型动画 |
| **Campbell 图** | matplotlib / plotly | **matplotlib** | 静态报告为主，更成熟 |

### 2.2 为什么选 CalculiX 而非 scikit-fem

- **scikit-fem**：轻量、Python 原生，但无内置梁/壳单元——需手写 Timoshenko 梁公式(12×12)、膜+板弯曲组合壳、手动组装 K/M。这等于"自己写 FEM 求解器"，工作量大且易出错。
- **CalculiX**：**零矩阵编码**。用户只需描述物理模型（节点、材料、截面、集中质量、约束），CalculiX 自动完成所有内部计算。B32 (Timoshenko 梁)、S4 (MITC4 壳)、MASS (集中质量)、SPRING2 (弹簧) 全部内置，已验证 30+ 年的工业级求解器。

### 2.3 双轨特征值求解策略

| 规模 | 求解器 | 适用场景 |
|------|--------|---------|
| **小-中** (DOF < 20,000) | `scipy.sparse.linalg.eigsh` | M-01 梁、M-02 板、M-03 简化骨架、快速迭代开发 |
| **中-大** (DOF > 20,000) | SLEPc **Krylov-Schur + PARDISO shift-and-invert** | M-03/M-04 全机壳+梁混合模型 |

**为什么 PARDISO 很重要**：对于广义特征值问题 \(K\phi = \omega^2 M\phi\)，当需要求**低频段**（最小特征值）时，shift-and-invert 变换需要反复求解 \((K - \sigma M)^{-1}\)。PARDISO 将 \(K - \sigma M\) 做一次 LU 分解后缓存，后续每次反代入极快。没有 PARDISO 时，SLEPc 只能用 Krylov-Schur 不做 shift-and-invert，收敛到最小特征值会**极慢**，尤其对于无人机这种低频密集的结构模态问题。

当前主要使用 CalculiX 内置的 Lanczos 求解器。若未来全机模型 DOF 极大，可改为调用 SLEPc+PARDISO 的 shift-and-invert 模式获取极低频密集模态。

---

## 3. 软件架构

**核心理念**: 用户只描述物理模型，不写矩阵公式。CalculiX 自动完成 K/M 组装和特征值求解。

```
drone_fem/
├── __init__.py
├── materials.py                  # 材料数据库 (ABS, PLA+, 碳纤维, 钢)
├── preprocess/
│   ├── __init__.py
│   └── inp_generator.py          # Python → CalculiX .inp 生成器
│       InpGenerator              #   描述节点、材料、截面、集中质量、约束
│       BeamSet, ShellSet         #   梁/壳单元组数据结构
├── solver/
│   ├── __init__.py
│   └── calculix.py               # subprocess ccx + .dat/.frd 解析
│       run_calculix() → ModalResult
│         .frequencies  (Hz)
│         .mode_shapes  (振型位移)
├── postprocess/
│   └── __init__.py               # Campbell 图、避频裕度、可视化 (待开发)
└── data/
    └── inp_template.md           # CalculiX .inp 模板说明

m01_validate.py                   # M-01 悬臂梁验证 ✅ (项目根目录)
```

数据流：

```
节点坐标 + 材料 + 截面 + 集中质量 + 约束
        │
        ▼
[preprocess/inp_generator.py]  →  model.inp
        │
        ▼
[CalculiX ccx]                  →  model.dat (频率) + model.frd (振型)
  (自动 K/M 组装 + Lanczos 特征值求解)
        │
        ▼
[solver/calculix.py]            →  ModalResult
        │
        ▼
[postprocess/]  +  电机 RPM/桨叶数  →  Campbell 图 + 避频裕度报告
```

---

## 4. 输入数据规格

### 4.1 材料参数

```yaml
# materials.yaml - 由材料数据手册和文献获取
materials:
  ABS:
    E: 2.0e9        # Pa (各向同性近似，FDM 有折减)
    nu: 0.35
    rho: 1050.0     # kg/m³
    damping_ratio: 0.02   # (FDM 打印件阻尼偏高)
  PLA_plus:
    E: 3.5e9
    nu: 0.35
    rho: 1240.0
    damping_ratio: 0.02
  carbon_tube:
    E: 60.0e9       # 50-70 GPa，取中值
    nu: 0.30
    rho: 1600.0
    damping_ratio: 0.005  # (碳纤维阻尼低)
```

### 4.2 几何参数（V60 具体参数从手册提取）

```yaml
# v60_geometry.yaml
fuselage:
  type: shell
  material: PLA_plus
  thickness: 0.001       # 1 mm
  mesh_size: 0.005       # 目标网格尺寸 5mm

wing_left:
  type: shell
  material: PLA_plus
  thickness: 0.001
  span: 0.35             # m (估算)
  chord: 0.12

carbon_tubes:
  longitudinal:           # 纵向碳管×2
    type: beam
    section: square_hollow
    outer_width: 0.006     # 6mm
    wall_thickness: 0.001  # 1mm
    length: 0.410          # 410mm
    material: carbon_tube
    clamp_position: 0.150  # Beamfix 夹紧位置距前端 150mm
  spar:                    # 翼梁碳管×1
    type: beam
    section: square_hollow
    outer_width: 0.006
    wall_thickness: 0.001
    length: 0.500
    material: carbon_tube

point_masses:              # 集中质量列表
  - name: battery
    mass: 0.300             # 300g
    cg: [0.0, 0.0, -0.03]  # 相对机身参考点
    moi: [0.001, 0.001, 0.0005]  # 转动惯量
    connection: spring      # 绑带固定 → 弹簧单元
    attached_to: fuselage_cg
  - name: lift_motor_1
    mass: 0.025
    cg: [0.08, -0.20, 0.0]
    connection: rbe3        # RBE3 分配到电机座 4 螺栓孔
    attached_to: tube_L_front
  # ... 其他 3 个升力电机、1 个推进电机、飞控、FPV 等
```

### 4.3 电机激励参数

```yaml
# motor_excitation.yaml
lift_motor:            # EMax 2004 1600kv
  kv: 1600
  voltage_nominal: 22.2  # 6S
  num_blades: 2
  rpm_hover: [4000, 8000]  # 悬停转速范围
  rpm_max: 35500           # 空载理论值
  harmonics: [1, 2, 4]     # 1P, 2P(BPF), 4P
  
push_motor:            # EMax 2807 1300kv
  kv: 1300
  voltage_nominal: 22.2
  num_blades: 2
  rpm_cruise: [4000, 8000]
  harmonics: [1, 2, 4]
```

### 4.4 连接/边界条件

```yaml
# connections.yaml
connections:
  - type: shared_nodes     # 刚性共节点
    parts: [spar_tube, fuselage_center]  # 翼梁穿过机身
  - type: shared_nodes
    parts: [longitudinal_tube_L, fuselage_beamfix]
  - type: shared_nodes
    parts: [longitudinal_tube_R, fuselage_beamfix]
  - type: shared_nodes
    parts: [wing_left_root, spar_tube_left_end]
  - type: shared_nodes
    parts: [wing_right_root, spar_tube_right_end]
  - type: shared_nodes
    parts: [stabilizer_root, longitudinal_tube_rear]
  - type: rbe3
    master: lift_motor_1_cg
    slaves: [motor_mount_L_front_bolt_1, ..., motor_mount_L_front_bolt_4]
  - type: spring           # 电池绑带
    parts: [battery_cg, fuselage_bay]
    stiffness: [1000, 1000, 5000]  # x, y, z N/m
```

### 4.5 网格文件

每个组件由 Gmsh 生成独立的 `.msh` 文件：
- `fuselage.msh` — 壳网格（Tri3/Tri6）
- `wing_L.msh`, `wing_R.msh` — 壳网格
- `stabilizer.msh` — 壳网格
- `spar.msh` — 线网格（Line2/Line3）
- `tube_L.msh`, `tube_R.msh` — 线网格

---

## 5. 输入数据来源

| 数据类别 | 来源 | 获取方式 |
|---------|------|---------|
| **材料参数** (E, ν, ρ) | 材料数据手册、文献、3D 打印厂商数据表 | 已知值写入 `materials.yaml` |
| **几何尺寸** | V60 手册（`资料/V60 阿黛尔4+1垂起FPV adelex-10手册.pdf`） | 从手册图纸/规格表提取 → `v60_geometry.yaml` |
| **质量分布** | 手册组件清单 + 实测 | 手册给定估算值 → `point_masses` 配置 |
| **碳管截面/长度** | 手册装配图 | 提取 6×6×410/500 mm 等参数 |
| **壁厚** | 手册（FDM 打印件 ~1mm） | 写入材料段 |
| **电机参数** (KV/桨叶/转速) | 手册 + EMax 厂商规格 | 写入 `motor_excitation.yaml` |
| **连接拓扑** | 手册装配关系 + M_K.md 第3.3节表格 | 手动编写 `connections.yaml` |
| **网格** | Gmsh 参数化建模 | 按 `v60_geometry.yaml` 参数生成 |
| **边界条件** | 飞行工况（自由-自由模态分析） | 无需约束，或施加惯性释放 |
| **验证基准参考值** | 材料力学教材、振动力学教材 | M-01/M-02 解析解用于回归测试 |

**关键判断**：输入数据的**骨架**（连接拓扑、激励路径、物理关系）已在 M_K.md 中完整记录。当前最需要的不是"从哪找到这些数"，而是**把已有参数化为结构化配置文件**，让代码能读取。

---

## 6. 开发路线图

### Phase 1: 基础框架 + M-01 悬臂梁 (P0) ✅
- 搭建 `drone_fem/` 包结构
- 实现 **InpGenerator** (B32梁/S4壳/C3D8实体) — Python → CalculiX .inp
- 实现 **run_calculix + 解析** — subprocess ccx + 解析 .dat
- M-01: COMET-FEniCS 基准 (3D C3D8) vs FEniCS SLEPc 参考解 ✅ 误差 < 1.2%
  - 参考: https://comet-fenics.readthedocs.io/en/latest/demo/modal_analysis_dynamics/cantilever_modal.html

### Phase 2: M-02 板壳 (P0) ✅
- NAFEMS FV52 标准基准: 10×10×1m 简支方板 (C3D8 3D实体)
- InpGenerator → CalculiX C3D8 → vs NAFEMS 参考解 ✅ 误差 < 3%
  - 参考: NAFEMS TNSB Rev.3, Test FV52, 1990

### Phase 3: M-03 旋翼支架避频 (P0)
- 用 Gmsh 生成 V60 碳管骨架 (梁单元) + 电机节点 (集中质量)
- InpGenerator: B32 + MASS + 连接关系 (共享节点)
- 实现 **电机激励模型** (`postprocess/` 或独立脚本)
- 计算固有频率，生成 Campbell 图，避频裕度分析

### Phase 4: 全机模型 + M-04 (P1)
- 加入 S4 壳单元建模机身/机翼/尾翼
- 多工况（空载/满载/Droid挂载）模态对比
- 振型可视化（PyVista 读取 .frd）

### Phase 5: 完善 & 高性能
- 编译 petsc4py/slepc4py 到自定义 PETSc (启用 PARDISO)
- 可选: SLEPc+PARDISO 后端用于大规模全机模型
- 频响分析（FRF）、参数化研究报告生成

---

## 7. 验证方法

| Benchmark | 验证方式 | 判定标准 |
|-----------|---------|---------|
| M-01 | CalculiX C3D8 vs FEniCS SLEPc 3D参考解 (COMET) | ✅ 6阶 < 1.2% |
| M-02 | CalculiX C3D8 vs NAFEMS FV52 参考解 | ✅ < 2.8% |
| M-03 | 物理合理性检查 + 与参考资料频率范围对比 | 模态频率在预期范围内，避频裕度 ≥ 15% |
| M-04 | 空载/满载频率偏移趋势合理（增质→降频） | 定性正确 + 数值合理 |
