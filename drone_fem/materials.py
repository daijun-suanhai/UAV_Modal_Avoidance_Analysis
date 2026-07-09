"""无人机模态分析 —— 材料数据库

定义 V60 各组件使用的材料属性。
所有单位采用 SI (m, kg, Pa)。
"""

from dataclasses import dataclass


@dataclass
class Material:
    """各向同性线弹性材料"""
    name: str
    E: float          # 杨氏模量 (Pa)
    nu: float         # 泊松比
    rho: float        # 密度 (kg/m³)
    damping_ratio: float = 0.01  # 模态阻尼比（默认 1%）

    @property
    def G(self) -> float:
        """剪切模量 G = E / (2(1+ν))"""
        return self.E / (2 * (1 + self.nu))


# ============================================================
# V60 材料库
# ============================================================

MATERIALS = {
    "ABS": Material(
        name="ABS (FDM 3D Print)",
        E=2.0e9,        # FDM 打印件等效各向同性，实际有折减
        nu=0.35,
        rho=1050.0,
        damping_ratio=0.02,
    ),
    "PLA_plus": Material(
        name="PLA+ (FDM 3D Print)",
        E=3.5e9,
        nu=0.35,
        rho=1240.0,
        damping_ratio=0.02,
    ),
    "carbon_tube": Material(
        name="碳纤维方管 (6×6 mm, t=1mm)",
        E=60.0e9,       # 碳纤维纵向模量，取中值
        nu=0.30,
        rho=1600.0,
        damping_ratio=0.005,
    ),
    # 验证用标准材料
    "steel": Material(
        name="结构钢 (A36)",
        E=200e9,
        nu=0.30,
        rho=7850.0,
    ),
    "aluminum": Material(
        name="铝合金 (6061-T6)",
        E=69e9,
        nu=0.33,
        rho=2700.0,
    ),
}


def get_material(name: str) -> Material:
    """按名称获取材料，不存在则抛出 KeyError"""
    if name not in MATERIALS:
        raise KeyError(f"未定义材料 '{name}'，可选：{list(MATERIALS.keys())}")
    return MATERIALS[name]
