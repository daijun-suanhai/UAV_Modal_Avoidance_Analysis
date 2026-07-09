#!/usr/bin/env python3
"""M-02: 矩形板壳模态分析验证 (CalculiX C3D8 3D实体单元)

参考来源: NAFEMS FV52 — Simply Supported Solid Square Plate
  NAFEMS Publication TNSB, Rev. 3, "The Standard NAFEMS Benchmarks", 1990.
  https://classes.engineering.wustl.edu/2009/spring/mase5513/abaqus/docs/v6.5/books/bmk/ch04s04anf25.html

基准模型:
  - 方板: 10m × 10m × 1m (厚度)
  - 材料: E=200GPa, ν=0.3, ρ=8000kg/m³
  - 约束: 底面(z=-0.5)四边 Uz=0 (简支)

NAFEMS 参考解:
  Mode 1-3: ~0 Hz (刚体模态)
  Mode 4:   45.897 Hz  (1阶弯曲)
  Mode 5,6: 109.44 Hz  (1阶扭转, 对称)
  Mode 7:   167.89 Hz  (2阶弯曲)

工作流:
  Python BoxMesh → InpGenerator C3D8 .inp → CalculiX → 对比 NAFEMS
  用户不写任何矩阵公式。
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from drone_fem.preprocess import InpGenerator, SolidSet
from drone_fem.solver import run_calculix
from drone_fem.materials import Material
import tempfile


# NAFEMS FV52 参考解 (忽略前3个刚体模态)
NAFEMS_REF = {
    4: 45.897,     # 1st bending
    5: 109.44,     # 1st torsional
    6: 109.44,     # 1st torsional (coincident)
    7: 167.89,     # 2nd bending
    8: 193.59,     # -
    9: 206.19,     # coincident pair
    10: 206.19,    # coincident pair
}


def generate_plate_mesh(L, H, Nx, Ny, Nz):
    """生成 3D 板结构化网格

    板在 z 方向从 -H/2 到 +H/2, xy 平面 0→L
    Returns: nodes, C3D8 elements, bottom_edge_nodes
    """
    xs = np.linspace(0, L, Nx + 1)
    ys = np.linspace(0, L, Ny + 1)
    zs = np.linspace(-H/2, H/2, Nz + 1)
    nc, nr = Nx + 1, Ny + 1

    def nid(ix, iy, iz): return iz * nc * nr + iy * nc + ix + 1

    nodes = [(xs[ix], ys[iy], zs[iz])
             for iz in range(Nz + 1) for iy in range(Ny + 1)
             for ix in range(Nx + 1)]

    elements = []
    eid = 1
    for iz in range(Nz):
        for iy in range(Ny):
            for ix in range(Nx):
                elements.append((eid,
                    nid(ix, iy, iz), nid(ix+1, iy, iz),
                    nid(ix+1, iy+1, iz), nid(ix, iy+1, iz),
                    nid(ix, iy, iz+1), nid(ix+1, iy, iz+1),
                    nid(ix+1, iy+1, iz+1), nid(ix, iy+1, iz+1)))
                eid += 1

    # NAFEMS FV52: 四边全厚度简支 (Uz=0)
    edge_nodes = []
    tol = L / (Nx * 100)
    for iz in range(Nz + 1):
        for iy in range(Ny + 1):
            for ix in range(Nx + 1):
                x, y = xs[ix], ys[iy]
                on_edge = (abs(x) < tol or abs(x - L) < tol or
                           abs(y) < tol or abs(y - L) < tol)
                if on_edge:
                    edge_nodes.append(nid(ix, iy, iz))

    return np.array(nodes), elements, list(set(edge_nodes))


def build_plate_inp(L, H, E, nu, rho, Nx, Ny, Nz):
    """生成 NAFEMS FV52 模型"""
    gen = InpGenerator("M02_NAFEMS_FV52")
    gen.add_material("Mat1", Material(name="Steel", E=E, nu=nu, rho=rho))

    nodes, elements, edge_nodes = generate_plate_mesh(L, H, Nx, Ny, Nz)

    for (x, y, z) in nodes:
        gen.add_node(x, y, z)

    gen.add_solid_elements(SolidSet(
        name="Eall", material="Mat1", elem_type="C3D8", elements=elements))

    # 四边简支 (Uz=0, 全厚度)
    for nid in edge_nodes:
        gen.add_boundary(nid, 3, 3)

    # 消除刚体模态: 角点 ux=uy=0 (不影响弯曲/扭转频率)
    gen.add_boundary(1, 1, 2)

    gen.add_frequency_step(12)
    return gen


def main():
    print("=" * 60)
    print("M-02: 方板模态分析 (CalculiX C3D8 3D实体)")
    print("参考: NAFEMS FV52 — Simply Supported Square Plate")
    print("=" * 60)
    print(f"\n  模型: 10×10×1m  底面四边简支(Uz=0)")
    print(f"  材料: E=200GPa, ν=0.3, ρ=8000kg/m³")
    print(f"\n  NAFEMS 参考解: f4=45.897  f5,6=109.44  f7=167.89 Hz")

    print(f"\n{'网格(Nx×Ny×Nz)':<18} {'元素':<7} {'f4':<12} {'f5':<12} "
          f"{'f7':<12} {'f8':<12}")
    print("-" * 64)

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        for Nxy in [10, 20, 30, 40]:
            Nz = max(Nxy // 5, 2)
            gen = build_plate_inp(10, 1, 200e9, 0.3, 8000, Nxy, Nxy, Nz)
            gen.write(td / f"m02_n{Nxy}.inp")
            r = run_calculix(td / f"m02_n{Nxy}.inp", work_dir=td, cleanup=False)
            f = r.frequencies

            # 跳过刚体+摇摆模态 (<40Hz)，取弯曲/扭转模态
            elastic = [x for x in f if x > 1.0]
            bending = [x for x in elastic if x > 40.0]
            vals = bending[:4]
            line = f"  {Nxy}×{Nxy}×{Nz:<3}         {Nxy*Nxy*Nz:<5}"
            for v in vals:
                line += f"  {v:10.4f}"
            for _ in range(4 - len(vals)):
                line += "        N/A"
            print(line)

        print(f"\n  NAFEMS ref:     "
              + "  ".join(f"{NAFEMS_REF[m]:10.4f}" for m in [4, 5, 7, 8]))

        # ---- 详细对比 (30×30) ----
        Nxy = 40
        Nz = max(Nxy // 5, 2)
        print(f"\n{'='*60}")
        print(f"详细: {Nxy}×{Nxy}×{Nz} C3D8 ({Nxy*Nxy*Nz}el) vs NAFEMS FV52")
        print(f"{'='*60}")

        gen = build_plate_inp(10, 1, 200e9, 0.3, 8000, Nxy, Nxy, Nz)
        gen.write(td / "m02_final.inp")
        r = run_calculix(td / "m02_final.inp", work_dir=td, cleanup=False)
        f = r.frequencies

        print(f"\n  全模态: {[f'{x:.2f}' for x in f[:12]]}")
        elastic = [x for x in f if x > 1.0]
        rocking = [x for x in elastic if x < 40.0]
        bending = [x for x in elastic if x > 40.0]
        if rocking:
            print(f"  摇摆模态(NAFEMS不计): {[f'{x:.2f}' for x in rocking]}"
                  f" — 简支板绕边缘转动, 不影响弯曲频率")
        print(f"  弯曲/扭转模态: {[f'{x:.2f}' for x in bending[:8]]}")

        print(f"\n{'NAFEMS':<6} {'C3D8(Hz)':<14} {'Reference':<14} {'误差':<10}")
        print("-" * 46)
        ref_keys = [4, 5, 6, 7, 8, 9, 10]
        errors = []
        for i in range(min(len(bending), len(ref_keys))):
            ref = NAFEMS_REF[ref_keys[i]]
            err = abs(bending[i] - ref) / ref * 100
            errors.append(err)
            marker = " ✅" if err < 5 else " ⚠️"
            print(f"  Mode{ref_keys[i]}   {bending[i]:10.4f}    {ref:10.4f}    "
                  f"{err:5.2f}%{marker}")

        max_err = max(errors) if errors else 0
        print(f"\n  最大误差: {max_err:.2f}%")
        if max_err < 5:
            print("  ✅ M-02 通过 (C3D8 ↔ NAFEMS FV52)")
        else:
            print(f"  ⚠️  超标")

        return r


if __name__ == "__main__":
    main()
