# CalculiX .inp 模板

> 由 `drone_fem/preprocess/inp_generator.py` 自动生成。
> 用户通过 Python API 描述物理模型，InpGenerator 将其转换为以下格式。

---

## 完整模板（带 V60 无人机注释）

```inp
** ============================================================
** <模型名称>
** 由 drone_fem InpGenerator 自动生成
**
** 节点数: <N>  |  梁单元组: <B>  |  壳单元组: <S>
** ============================================================

** ──────────────────────────────────────────────────────────
** 1. 节点坐标 (*NODE)
**    格式: node_id, x, y, z
**    节点 ID 从 1 开始，必须连续
** ──────────────────────────────────────────────────────────
*NODE
1, 0.00000000, 0.00000000, 0.00000000
2, 0.01250000, 0.00000000, 0.00000000
3, 0.02500000, 0.00000000, 0.00000000
...

** ──────────────────────────────────────────────────────────
** 2. 梁单元 (*ELEMENT, TYPE=B32)
**    B32 = 3节点 Timoshenko 梁
**    格式: elem_id, node_左端, node_中点, node_右端
**    用于: 碳管骨架 (翼梁 6×6×500, 纵向管 6×6×410)
** ──────────────────────────────────────────────────────────
*ELEMENT, TYPE=B32, ELSET=CarbonTubes
1, 1, 2, 3
2, 3, 4, 5
...

** ──────────────────────────────────────────────────────────
** 3. 壳单元 (*ELEMENT, TYPE=S4)
**    S4 = 4节点壳 (MITC4)
**    格式: elem_id, n1, n2, n3, n4  (逆时针或顺时针)
**    用于: 3D打印件 (机身/机翼/平尾, 壁厚~1mm)
** ──────────────────────────────────────────────────────────
*ELEMENT, TYPE=S4, ELSET=Fuselage
1, 10, 11, 14, 13
2, 11, 12, 15, 14
...

** ──────────────────────────────────────────────────────────
** 4. 弹簧单元 (*ELEMENT, TYPE=SPRING2)  [可选]
**    两节点弹簧，模拟绑带等柔性连接
**    用于: 电池绑带固定 (弱刚度连接)
** ──────────────────────────────────────────────────────────
*ELEMENT, TYPE=SPRING2, ELSET=Straps
101, 50, 51
*SPRING, ELSET=Straps
1.000e+03           ← 刚度 k (N/m)，每个弹簧一行

** ──────────────────────────────────────────────────────────
** 5. 梁截面 (*BEAM SECTION)
**    定义梁单元的截面几何
**    格式:
**      *BEAM SECTION, ELSET=<组名>, MATERIAL=<材料名>, SECTION=<类型>
**      <参数1>, <参数2>, ...        ← 截面尺寸 (m)
**      <nx>, <ny>, <nz>             ← 截面 y 轴方向向量
**
**    支持的 SECTION 类型:
**      RECT  — 实心矩形:         a, b           (宽, 高)
**      BOX   — 空心矩形:         a, b, ta, tb   (外边宽, 外边高, 壁厚_a, 壁厚_b)
**      PIPE  — 圆管:             r, t           (半径, 壁厚)
** ──────────────────────────────────────────────────────────
*BEAM SECTION, ELSET=CarbonTubes, MATERIAL=CarbonFiber, SECTION=BOX
0.006000, 0.006000, 0.001000, 0.001000    ← 6×6mm 方管, 壁厚 1mm
0.000000, 1.000000, 0.000000              ← 截面 y 轴沿全局 Y

** ──────────────────────────────────────────────────────────
** 6. 壳截面 (*SHELL SECTION)
**    定义壳的厚度
** ──────────────────────────────────────────────────────────
*SHELL SECTION, ELSET=Fuselage, MATERIAL=PLAplus
0.001000             ← 厚度 1mm (FDM 打印件)

** ──────────────────────────────────────────────────────────
** 7. 材料 (*MATERIAL)
**    每个材料包含:
**      *ELASTIC  — 杨氏模量 E (Pa), 泊松比 ν
**      *DENSITY  — 密度 ρ (kg/m³)
** ──────────────────────────────────────────────────────────
*MATERIAL, NAME=CarbonFiber
*ELASTIC
6.000000e+10, 3.000000e-01       ← E=60GPa, ν=0.30
*DENSITY
1.600000e+03                      ← ρ=1600 kg/m³

*MATERIAL, NAME=PLAplus
*ELASTIC
3.500000e+09, 3.500000e-01       ← E=3.5GPa, ν=0.35
*DENSITY
1.240000e+03                      ← ρ=1240 kg/m³

** ──────────────────────────────────────────────────────────
** 8. 集中质量 (*MASS)  [可选]
**    将质量集中到指定节点
**    格式: node_id, mass (kg)
**    用于: 电机 (25g×4+45g), 电池 (300g), 飞控 (50g), FPV (60g)
** ──────────────────────────────────────────────────────────
*MASS, ELSET=PointMasses
201, 0.025000        ← 升力电机 25g
202, 0.025000
203, 0.300000        ← 电池 300g
...

** ──────────────────────────────────────────────────────────
** 9. 节点组 (*NSET)  [可选]
**    定义节点集合，方便施加边界条件
**    每行最多 16 个节点 ID
** ──────────────────────────────────────────────────────────
*NSET, NSET=FixedNodes
1

*NSET, NSET=MotorNodes
201, 202, 203, 204, 205

** ──────────────────────────────────────────────────────────
** 10. 边界条件 (*BOUNDARY)
**     约束节点的自由度
**     DOF: 1=ux, 2=uy, 3=uz, 4=rx, 5=ry, 6=rz
**     格式: node_id, dof_start, dof_end
**
**     固定端: 1, 1, 6     ← 约束全部 6 个 DOF
**     简支:   1, 1, 3     ← 只约束平移
** ──────────────────────────────────────────────────────────
*BOUNDARY
1, 1, 6              ← 固定端: 节点1的 DOF 1-6

** ──────────────────────────────────────────────────────────
** 11. 频率分析步 (*STEP + *FREQUENCY)
**     Lanczos 特征值求解:  K φ = ω² M φ
**
**     *FREQUENCY 后的数字 = 需求的模态数
**     *NODE FILE, U   → 输出节点位移 (振型) 到 .frd
** ──────────────────────────────────────────────────────────
*STEP
*FREQUENCY
10                   ← 求前 10 阶模态
*NODE FILE
U                    ← 输出位移 (振型)
*END STEP
```

---

## 各模块与物理输入的对应关系

| `.inp` 关键字 | 物理含义 | V60 中的实际值 | `InpGenerator` API |
|--------------|---------|--------------|-------------------|
| `*NODE` | 节点坐标 | 碳管两端+中点, 壳网格节点 | `gen.add_node(x,y,z)` |
| `*ELEMENT, TYPE=B32` | Timoshenko 梁 | 翼梁(1根)+纵向管(2根) | `BeamSet` → `gen.add_beam_elements()` |
| `*ELEMENT, TYPE=S4` | 壳单元 | 机身/机翼/平尾蒙皮 | `ShellSet` → `gen.add_shell_elements()` |
| `*ELEMENT, TYPE=SPRING2` | 弹簧连接 | 电池绑带(弱约束) | `gen.add_spring()` |
| `*BEAM SECTION` | 梁截面尺寸 | 6×6mm 方管, 壁厚1mm | `BeamSet(section_params=...)` |
| `*SHELL SECTION` | 壳厚度 | 1mm (FDM 打印) | `ShellSet(thickness=...)` |
| `*MATERIAL, *ELASTIC` | E, ν | 碳纤维 60GPa/0.30, PLA+ 3.5GPa/0.35 | `drone_fem/materials.py` |
| `*MATERIAL, *DENSITY` | ρ | 碳纤维 1600, PLA+ 1240 kg/m³ | `drone_fem/materials.py` |
| `*MASS` | 集中质量 | 电机 25g, 电池 300g, 飞控 50g | `gen.add_mass(node, mass)` |
| `*BOUNDARY` | 约束 | — (飞行状态自由-自由) | `gen.add_boundary()` |
| `*FREQUENCY` | 模态阶数 | 前 10-20 阶 | `gen.add_frequency_step(n)` |

---

## V60 模型的 .inp 结构示意

```
** ============================================================
** V60_Modal
** ============================================================

*NODE
... 碳管节点 (沿x轴排列) ...
... 机身壳节点 (表面分布) ...
... 机翼壳节点 ...
... 电机座节点 (集中质量附着点) ...

*ELEMENT, TYPE=B32, ELSET=SparTube
... 翼梁碳管: 穿过机身, 左右套机翼 ...
*ELEMENT, TYPE=B32, ELSET=LongTubes
... 纵向碳管×2: 穿过Beamfix, 后端套平尾 ...

*ELEMENT, TYPE=S4, ELSET=Fuselage
... 机身蒙皮壳 ...
*ELEMENT, TYPE=S4, ELSET=WingL
*ELEMENT, TYPE=S4, ELSET=WingR
*ELEMENT, TYPE=S4, ELSET=Tail

*ELEMENT, TYPE=SPRING2, ELSET=BatteryStraps
... 电池绑带弹簧 ...

*BEAM SECTION, ELSET=SparTube, MATERIAL=CarbonFiber, SECTION=BOX
0.006, 0.006, 0.001, 0.001
0.0, 1.0, 0.0

*BEAM SECTION, ELSET=LongTubes, MATERIAL=CarbonFiber, SECTION=BOX
0.006, 0.006, 0.001, 0.001
0.0, 0.0, 1.0                    ← 注意: 纵向碳管截面方向不同!

*SHELL SECTION, ELSET=Fuselage, MATERIAL=PLAplus
0.001

*MATERIAL, NAME=CarbonFiber
*ELASTIC
60.0e9, 0.30
*DENSITY
1600.0

*MATERIAL, NAME=PLAplus
*ELASTIC
3.5e9, 0.35
*DENSITY
1240.0

*MASS, ELSET=PointMasses
101, 0.025     ← 升力电机 Lf
102, 0.025     ← 升力电机 Lr
...

** 飞行状态: 自由-自由，不施加位移约束
** (无 *BOUNDARY 关键字)

*STEP
*FREQUENCY
20              ← 求 20 阶模态
*NODE FILE
U
*END STEP
```

---

## 连接关系在 .inp 中的表达

| 物理连接 | M_K.md 描述 | CalculiX 实现 |
|---------|------------|--------------|
| 翼梁穿过机身 | 共节点 | 翼梁节点 = 机身中央段节点 (共享 ID) |
| 纵向管被 Beamfix 夹紧 | 共节点 | 管中部节点 = Beamfix 节点 |
| 尾翼套入纵向管后端 | 共节点 | 尾翼根部节点 = 管后端节点 |
| 电机座螺栓固定 | RBE3 质量分配 | 可用 `*KINEMATIC COUPLING` 或简化共节点 |
| 电池绑带 | 弱弹性连接 | `*ELEMENT, TYPE=SPRING2` |
| 左右翼无直接连接 | K=0 | 左右翼不共享节点，间接通过翼梁传递 |

> **关于 RBE3**: CalculiX 用 `*KINEMATIC COUPLING` 实现。格式为 `node_master, node_slave, dof1-dof6`。
> 简化处理时可直接将集中质量放在安装节点的位置（共节点），这样 K/M 耦合自动发生。
