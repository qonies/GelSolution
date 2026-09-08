# -*- coding: utf-8 -*-
"""动力学层：弹簧储能与活塞释放参数。

  * 弹簧储能 E = 0.5·k·((x0+s)^2 − x0^2)（k N/mm、x mm → mJ）；
  * 释放速度 v_release = √(2·E·传动效率/m_piston)——活塞被气垫减速前的初速度；
  * 气垫减速、活塞撞击时刻/速度、水弹加速见 ballistics.py（联动积分模型）；
  * 压气指数 = 行程比 × 气缸系数（相对指标，用于压气不足判定）。
"""
import math
from dataclasses import dataclass

from .params import SimConfig
from .components import DeviceParams
from .timing import Timeline


@dataclass
class Dynamics:
    k_n_per_mm: float        # 弹簧刚度
    energy_mj: float         # 活塞释放点弹簧储能 (mJ)
    v_release_m_s: float     # 活塞释放初速度（气垫起作用前）
    air_index: float         # 压气指数（行程比 × 气缸系数）


def compute(cfg: SimConfig, dev: DeviceParams, tl: Timeline) -> Dynamics:
    k = dev.spring_stiffness[cfg.spring]
    s = tl.stroke_mm
    x0 = dev.spring_preload_mm

    # 弹簧储能 (mJ)：0.5·k·((x0+s)^2 − x0^2)，k N/mm × mm² = mJ
    energy_mj = 0.5 * k * ((x0 + s) ** 2 - x0 ** 2)

    m_kg = dev.piston_mass_g / 1000.0
    if s > 1e-6 and energy_mj > 0:
        v_release = math.sqrt(2.0 * (energy_mj / 1000.0)
                              * dev.drive_efficiency / m_kg)
    else:  # 行程被切光：活塞不动
        v_release = 0.0

    air_index = tl.stroke_ratio * dev.cylinder_factor[cfg.cylinder]

    return Dynamics(
        k_n_per_mm=k, energy_mj=energy_mj, v_release_m_s=v_release,
        air_index=air_index,
    )
