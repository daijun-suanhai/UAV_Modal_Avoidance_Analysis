#!/usr/bin/env python3
"""M-01: 悬臂梁模态分析验证 (CalculiX C3D8 3D实体单元)

参考来源: COMET-FEniCS Modal Analysis (Creative Commons BY-SA 4.0)
  https://comet-fenics.readthedocs.io/en/latest/demo/modal_analysis_dynamics/cantilever_modal.html

基准模型:
  - 3D 悬臂梁: L=20, B=0.5, H=1
  - 材料: E=1e5, nu=0, rho=1e-3
  - 左端(x=0)全约束 (ux=uy=uz=0)
  - SLEPc 特征值求解 (shift-and-invert, sigma=0)

参考结果 (FEniCS SLEPc, Nx=400, Lagrange P1 3D solid):
  Solid FE: 2.04991, 4.04854, 12.81504, 25.12717, 35.74168, 66.94816 Hz
  Beam:     2.01925, 4.03850, 12.65443, 25.30886, 35.43277, 70.86554 Hz

工作流:
  Python BoxMesh → InpGenerator C3D8 .inp → CalculiX → 对比双参考解
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


# FEniCS 参考解 (COMET demo, Nx=400, 3D solid 1st-order Lagrange)
FENICS_REF = np.array([2.04991, 4.04854, 12.81504, 25.12717, 35.74168, 66.94816])
BEAM_REF    = np.array([2.01925, 4.03850, 12.65443, 25.30886, 35.43277, 70.86554])


def generate_box_mesh(L, B, H, Nx, Ny, Nz):
    """生成 3D 结构化六面体网格

    Returns: nodes (N,3), C3D8 elements [(eid,n1..n8),...], left_face_nodes
    """
    xs = np.linspace(0, L, Nx + 1)
    ys = np.linspace(0, B, Ny + 1)
    zs = np.linspace(0, H, Nz + 1)
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

    left_face = [nid(0, iy, iz) for iz in range(Nz + 1) for iy in range(Ny + 1)]
    return np.array(nodes), elements, left_face


def build_beam_inp(L, B, H, E, nu, rho, Nx, Ny, Nz):
    gen = InpGenerator("M01_3D_Cantilever")
    gen.add_material("Mat1", Material(name="Elastic", E=E, nu=nu, rho=rho))
    nodes, elements, left_face = generate_box_mesh(L, B, H, Nx, Ny, Nz)
    for (x, y, z) in nodes:
        gen.add_node(x, y, z)
    gen.add_solid_elements(SolidSet(
        name="Eall", material="Mat1", elem_type="C3D8", elements=elements))
    for nid in left_face:
        gen.add_boundary(nid, 1, 3)  # ux=uy=uz=0
    gen.add_frequency_step(8)
    return gen


def main():
    print("=" * 60)
    print("M-01: 3D悬臂梁模态分析 (CalculiX C3D8)")
    print("参考: COMET-FEniCS cantilever_modal demo")
    print("=" * 60)
    print(f"\n  模型: 20×0.5×1  3D悬臂梁 | E=1e5 ν=0 ρ=1e-3")

    print(f"\n{'网格(Nx×Ny×Nz)':<18} {'元素':<7} {'f1':<12} {'f2':<12} "
          f"{'f4':<12} {'f6':<12}")
    print("-" * 64)

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        for Nx in [50, 100, 200]:
            Ny = max(int(0.5 / 20 * Nx) + 1, 2)
            Nz = max(int(1.0 / 20 * Nx) + 1, 2)
            gen = build_beam_inp(20, 0.5, 1, 1e5, 0, 1e-3, Nx, Ny, Nz)
            gen.write(td / f"m01_n{Nx}.inp")
            r = run_calculix(td / f"m01_n{Nx}.inp", work_dir=td, cleanup=False)
            f = r.frequencies
            idxs = [0, 1, 3, 5]
            vals = [f"{f[i]:10.5f}" if i < len(f) else "        NA" for i in idxs]
            print(f"  {Nx}×{Ny}×{Nz:<3}           {Nx*Ny*Nz:<5}  "
                  + "  ".join(vals))

        idxs = [0, 1, 3, 5]
        print(f"\n  FEniCS(Nx=400):  "
              + "  ".join(f"{FENICS_REF[i]:10.5f}" for i in idxs))
        print(f"  Beam theory:    "
              + "  ".join(f"{BEAM_REF[i]:10.5f}" for i in idxs))

        # ---- 详细对比 Nx=200 ----
        Ny = max(int(0.5 / 20 * 200) + 1, 2)
        Nz = max(int(1.0 / 20 * 200) + 1, 2)
        print(f"\n{'='*60}")
        print(f"详细: 200×{Ny}×{Nz} C3D8 ({200*Ny*Nz}el) vs FEniCS + Beam 理论")
        print(f"{'='*60}")
        print(f"\n{'Mode':<6} {'C3D8(Hz)':<12} {'FEniCS(Hz)':<12} "
              f"{'ΔFenics':<10} {'Beam(Hz)':<12} {'ΔBeam':<10}")
        print("-" * 60)

        gen = build_beam_inp(20, 0.5, 1, 1e5, 0, 1e-3, Nx=200, Ny=Ny, Nz=Nz)
        gen.write(td / "m01_n200.inp")
        r = run_calculix(td / "m01_n200.inp", work_dir=td, cleanup=False)
        f = r.frequencies

        errs = []
        for i in range(min(6, len(f))):
            ef = abs(f[i] - FENICS_REF[i]) / FENICS_REF[i] * 100
            eb = abs(f[i] - BEAM_REF[i]) / BEAM_REF[i] * 100
            errs.append(ef)
            print(f"  {i+1:3d}   {f[i]:10.5f}   {FENICS_REF[i]:10.5f}   "
                  f"{ef:5.2f}%{' ✅' if ef<10 else ' ⚠️'}  "
                  f"{BEAM_REF[i]:10.5f}   {eb:5.2f}%{' ✅' if eb<10 else ' ⚠️'}")

        print(f"\n  最大误差(vs FEniCS): {max(errs):.2f}%")
        if max(errs) < 10:
            print("  ✅ M-01 验证通过 (3D实体C3D8 ↔ FEniCS SLEPc)")
        else:
            print(f"  ⚠️  偏差较大")
        return r


if __name__ == "__main__":
    main()
