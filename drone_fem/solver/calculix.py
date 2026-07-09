"""CalculiX 求解器封装

调用 CalculiX (ccx) 并解析计算结果。
"""

import subprocess
import re
import tempfile
import shutil
from pathlib import Path
from dataclasses import dataclass, field
from typing import Optional
import numpy as np


@dataclass
class ModalResult:
    """模态分析结果"""
    frequencies: np.ndarray       # (n_modes,) Hz
    eigenvalues: np.ndarray       # (n_modes,) ω²
    angular_frequencies: np.ndarray  # (n_modes,) rad/s
    mode_shapes: dict[int, np.ndarray] = field(default_factory=dict)
    # mode_shapes[mode_id] = (n_nodes, 6) displacement array [ux,uy,uz,rx,ry,rz]
    n_modes: int = 0
    backend: str = "CalculiX"
    dat_path: Optional[Path] = None
    frd_path: Optional[Path] = None

    def __repr__(self):
        lines = [f"ModalResult ({self.backend}, {self.n_modes} modes)"]
        for i, f in enumerate(self.frequencies[:10]):
            lines.append(f"  Mode {i+1:2d}: {f:10.4f} Hz")
        if self.n_modes > 10:
            lines.append(f"  ... ({self.n_modes - 10} more)")
        return "\n".join(lines)


def run_calculix(inp_path: str | Path,
                 work_dir: str | Path | None = None,
                 cleanup: bool = True) -> ModalResult:
    """运行 CalculiX 并返回模态分析结果

    Parameters
    ----------
    inp_path : .inp 文件路径
    work_dir : 工作目录 (None = 与 .inp 同目录)
    cleanup : 是否在解析后删除临时文件

    Returns
    -------
    ModalResult(frequencies, eigenvalues, mode_shapes)
    """
    inp_path = Path(inp_path)
    stem = inp_path.stem

    if work_dir is None:
        work_dir = inp_path.parent
    work_dir = Path(work_dir)

    # 确保 .inp 在工作目录下
    if inp_path.parent != work_dir:
        target = work_dir / inp_path.name
        shutil.copy(inp_path, target)
    else:
        target = inp_path

    # 运行 CalculiX
    try:
        proc = subprocess.run(
            ["ccx", stem],
            cwd=work_dir,
            capture_output=True,
            text=True,
            timeout=300,
        )
    except FileNotFoundError:
        raise RuntimeError(
            "CalculiX (ccx) 未找到。请安装: sudo apt install calculix-ccx"
        )
    except subprocess.TimeoutExpired:
        raise RuntimeError("CalculiX 计算超时 (300s)")

    dat_file = work_dir / f"{stem}.dat"
    frd_file = work_dir / f"{stem}.frd"

    if not dat_file.exists():
        raise RuntimeError(
            f"CalculiX 未生成 .dat 文件。计算可能失败。\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )

    # 解析频率
    frequencies, eigenvalues, angular = _parse_dat_frequencies(dat_file)

    # 解析振型（如果存在 .frd）
    mode_shapes = {}
    if frd_file.exists():
        mode_shapes = _parse_frd_displacements(frd_file)

    # 清理
    if cleanup:
        for ext in [".cvg", ".sta", ".12d"]:
            tmp = work_dir / f"{stem}{ext}"
            if tmp.exists():
                tmp.unlink()

    return ModalResult(
        frequencies=frequencies,
        eigenvalues=eigenvalues,
        angular_frequencies=angular,
        mode_shapes=mode_shapes,
        n_modes=len(frequencies),
        dat_path=dat_file,
        frd_path=frd_file if frd_file.exists() else None,
    )


def _parse_dat_frequencies(dat_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """从 .dat 文件解析特征值和频率

    CalculiX .dat 格式:
        MODE NO    EIGENVALUE                       FREQUENCY
                                     REAL PART            IMAGINARY PART
                           (RAD/TIME)      (CYCLES/TIME     (RAD/TIME)
              1   0.1053526E+05   0.1026414E+03   0.1633589E+02   0.0000000E+00

    列: Mode, λ_real, ω_real(rad/s), f_real(cycles/time=Hz), ω_imag, ...
    """
    text = dat_path.read_text()

    freqs = []
    lambdas = []
    omegas = []

    in_eigenvalue_section = False
    for line in text.splitlines():
        if "MODE NO    EIGENVALUE" in line:
            in_eigenvalue_section = True
            continue
        if in_eigenvalue_section:
            # 检查是否离开了特征值区域
            if "REAL PART" in line or "RAD/TIME" in line:
                continue
            parts = line.split()
            if len(parts) >= 4 and parts[0].isdigit():
                mode_num = int(parts[0])
                lam = float(parts[1])
                omega = float(parts[2])
                freq = float(parts[3])
                # 只收集每个 mode 的首次出现（去重）
                if mode_num > len(freqs):
                    freqs.append(freq)
                    lambdas.append(lam)
                    omegas.append(omega)
            else:
                # 非数字开头 → 可能离开了 eigenvalue section
                if len(freqs) > 0 and not any(c.isdigit() for c in line[:5]):
                    break

    return np.array(freqs), np.array(lambdas), np.array(omegas)


def _parse_frd_displacements(frd_path: Path) -> dict[int, np.ndarray]:
    """从 .frd 文件解析振型位移

    返回 {mode_id: (n_nodes, 6) ndarray}
    列顺序: [ux, uy, uz, rx, ry, rz]
    """
    text = frd_path.read_text()
    # 简化解析: 按模态分组
    mode_shapes: dict[int, list] = {}
    current_mode = None
    current_node = 0

    # .frd 格式:
    #   -1  ← 开始新 block
    #  1PSTEP         1  ← mode number
    #  1  1  -0.276E-13  -1.377E+00  ...  ← node_id, ?, ux, uy, uz, rx, ry, rz

    lines = text.splitlines()
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped == "-1":
            # 新数据块
            current_mode = None
            continue
        if stripped.startswith("1PSTEP"):
            continue  # 下一行开始数据
        if stripped.startswith("1CL") or stripped.startswith("1 "):
            continue

        parts = stripped.split()
        if len(parts) < 7:
            continue

        # 检测频率行 (模式号)
        # 频率块格式: 行中有 4 个数字 → mode_no, ?, freq_hz, ?
        if len(parts) == 4:
            try:
                maybe_mode = int(parts[0])
                if 1 <= maybe_mode <= 1000:
                    current_mode = maybe_mode
                    if current_mode not in mode_shapes:
                        mode_shapes[current_mode] = []
            except ValueError:
                pass
            continue

        # 位移行: node_id, 1, ux, uy, uz, rx, ry, rz
        if len(parts) >= 8 and current_mode is not None:
            try:
                node_id = int(parts[0])
                ux = float(parts[2])
                uy = float(parts[3])
                uz = float(parts[4])
                rx = float(parts[5]) if len(parts) > 5 else 0.0
                ry = float(parts[6]) if len(parts) > 6 else 0.0
                rz = float(parts[7]) if len(parts) > 7 else 0.0
                mode_shapes[current_mode].append((node_id, [ux, uy, uz, rx, ry, rz]))
            except (ValueError, IndexError):
                continue

    # 转为数组
    result = {}
    for mode_id, node_disps in mode_shapes.items():
        node_disps.sort(key=lambda x: x[0])
        n_nodes = len(node_disps)
        arr = np.zeros((n_nodes, 6))
        for i, (_, disp) in enumerate(node_disps):
            arr[i] = disp
        result[mode_id] = arr

    return result


def solve_modal(inp_path: str | Path, **kwargs) -> ModalResult:
    """统一的模态分析接口（封装 run_calculix）

    Parameters
    ----------
    inp_path : .inp 文件路径
    **kwargs : 传递给 run_calculix

    Returns
    -------
    ModalResult
    """
    return run_calculix(inp_path, **kwargs)
