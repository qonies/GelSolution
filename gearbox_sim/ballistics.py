# -*- coding: utf-8 -*-
"""弹道层（气垫-弹丸联动积分模型）：估算水弹初速与活塞撞击时序。

物理过程（与真实二号波一致）：
  1. 活塞释放后先在开孔段自由加速（开孔段气体直通大气，不压缩）；
  2. 活塞头盖过气孔后气体密封，随活塞前进被绝热压缩，同时气垫
     反过来减速活塞（缓冲）；
  3. 弹后气压一升高水弹就开始沿内管加速——弹后容积增大又反过来
     限制压力上升（大内径缸下该效应显著，不能按"先压满再膨胀"算）；
  4. 活塞到达缸头（撞击）后容积只随水弹前进增大，气压绝热下降，
     降到大气压后水弹不再受推力（对应"有效推力行程"）；
  5. 管径-弹径间隙泄气、其他损耗用泄气损失 + 气动效率折减末动能；
  6. 撞击后天梯（活塞）反弹回位按主弹簧-活塞简谐模型：回弹速度 = 回弹系数
     × 撞击速度，复位时间 = 2·atan(v_r/(ω·x0))/ω（ω=√(k/m)，x0=撞击时
     弹簧压缩量=预压）——刚度/压缩长度越大复位越快，撞击越重复位越久。

积分步长 2μs，输出初速、撞击时刻/速度、出膛时刻、回位裕量等。
"""
import math
from dataclasses import dataclass
from typing import Optional

from .params import SimConfig
from .components import DeviceParams
from .timing import Timeline

_DT = 2e-6        # 积分步长 (s)
_T_CAP = 0.05     # 积分上限 (s)
_P_CAP = 30000.0  # 数值保护上限 (kPa)


@dataclass
class Ballistics:
    v_m_s: float             # 估算初速
    energy_j: float          # 水弹出口动能 (J)
    p_max_kpa: float         # 密封后缸压峰值
    swept_cm3: float         # 有效排量 = 缸面积 × 行程 × 气缸系数
    useful_stroke_mm: float  # 有效推力行程（气压降至大气压时水弹已行进距离）
    leak_ratio: float        # 泄气能量损失比例
    gap_mm: float            # 管径-弹径单边间隙
    strike_ms: float         # 活塞撞击气缸头时刻（绝对，相对本循环扇齿 0° 基准）
    v_impact_m_s: float      # 撞击时活塞剩余速度（气垫缓冲后）
    v_rebound_m_s: float     # 撞击后反弹速度（回弹系数 × 撞击速度）
    t_fire_ms: float         # 前冲时间（释放→撞击）
    t_settle_ms: float       # 撞击后回位稳定时间
    settle_done_ms: float    # 回位稳定完成时刻（绝对）
    return_margin_ms: float  # 回位裕量 = 下一循环拾取 − 回位稳定完成
    exit_ms: float           # 水弹出膛时刻（估算，绝对）
    p_rise_ms: Optional[float]  # 弹后压力显著建立时刻（绝对）；None=本行程无有效压气


def settle_time_ms(v_impact_m_s: float, k_n_m: float, preload_m: float,
                   piston_mass_kg: float, restitution: float,
                   floor_ms: float = 2.0) -> float:
    """天梯复位（回位稳定）时间：主弹簧-活塞简谐回位模型。

    撞击后天梯以 v_r = 回弹系数×撞击速度 反弹，主弹簧（刚度 k、撞击时
    压缩量 = 预压 x0）使其做简谐回位，复位时间：
        t = 2·atan(v_r/(ω·x0))/ω，ω = √(k/m)
    * 弹簧刚度 k 越大 → ω 越大 → 复位越快（同撞击条件下）；
    * 压缩长度（撞击时弹簧压缩量 = 预压 x0）越大 → 复位越快；
    * 撞击越重（v_r 大）→ 复位越久（t 随 v_r 单调增大，上限 π/ω）。
    注意：整枪对比时刚度更大的弹簧撞得也更重（v_imp 更高），净效果可能
    相互抵消——方向性应在同撞击速度下比较。
    """
    v_r = restitution * v_impact_m_s
    omega = math.sqrt(k_n_m / piston_mass_kg) if piston_mass_kg > 0 else 0.0
    if v_r > 0 and omega > 0 and preload_m > 1e-9:
        return max(floor_ms, 2000.0 * math.atan(v_r / (omega * preload_m)) / omega)
    return floor_ms  # 无反弹或参数退化（预压被覆盖为 0 等）：按下限处理


def compute(cfg: SimConfig, dev: DeviceParams, tl: Timeline, dyn) -> Ballistics:
    gamma = dev.adiabatic_index
    p_atm = dev.atm_kpa
    bore = float(cfg.barrel_bore)
    ball = float(cfg.ball_diameter)
    length = cfg.barrel_length_mm / 1000.0
    s = tl.stroke_mm / 1000.0
    f = dev.cylinder_factor[cfg.cylinder]
    m_p = dev.piston_mass_g / 1000.0
    m_b = dev.gel_mass_for(ball) / 1000.0   # 水弹质量按直径体积缩放
    A_cyl = math.pi * (dev.cylinder_bore_mm / 2.0) ** 2 / 1e6   # m²
    A_push = math.pi * (bore / 2.0) ** 2 / 1e6                  # m²
    V_dead = dev.dead_volume_cm3 / 1e6                          # m³
    x0 = dev.spring_preload_mm / 1000.0
    k = dyn.k_n_per_mm * 1000.0                                 # N/m
    # 活塞从静止开始被弹簧推动（释放瞬间 v=0）；传动效率折算进弹簧推力
    v_p = 0.0

    # 退化为切光（无行程）：活塞不动、无压气
    if s < 1e-6:
        return _degenerate(tl)

    V_seal = V_dead + A_cyl * f * s        # 密封瞬间弹后容积（绝热基准）
    x_p = 0.0
    x_b = 0.0
    v_b = 0.0
    t = 0.0
    p_max = p_atm
    strike_t = None
    v_imp = 0.0
    exit_t = None
    t_rise = None   # 弹后压力首次超过大气压 10% 的时刻（气密判定基准）
    useful = 0.0    # 有效推力行程（有推力期间的弹丸位移，无推力则保持 0）
    ball_done = False

    while t < _T_CAP and not (ball_done and strike_t is not None):
        d = max(s - x_p, 0.0)
        P = p_atm
        # 密封段：活塞头已盖过气孔 → 绝热压缩
        if x_p >= (1.0 - f) * s - 1e-12 and V_seal > 0:
            V = V_dead + A_cyl * d + A_push * x_b
            if V > 1e-12:
                P = min(p_atm * (V_seal / V) ** gamma, _P_CAP)
            p_max = max(p_max, P)
            if t_rise is None and P >= p_atm * 1.1:
                t_rise = t  # 压力显著建立（推嘴须在此之前就位，否则漏气）

        # 水弹：受气压差驱动（气压降至大气压后不再有净推力）
        dp_pa = (P - p_atm) * 1000.0  # kPa → Pa
        if not ball_done:
            if P > p_atm and x_b < length:
                v_b += dp_pa * A_push / m_b * _DT
                x_b += v_b * _DT
                useful = x_b
                if x_b >= length:
                    x_b = length
                    exit_t = t
                    ball_done = True
            elif x_b > 0 or strike_t is not None:
                ball_done = True  # 已获得速度且气压回到大气压（撞击后单调下降）
            # 密封刚开始、弹尚未动时 P≈大气压属正常，继续等活塞压缩

        # 活塞：弹簧推力（含传动效率） − 气垫反力（气垫即"缓冲"，减速活塞）
        if strike_t is None:
            v_p += (k * (x0 + d) * dev.drive_efficiency
                    - dp_pa * A_cyl) / m_p * _DT
            x_p += v_p * _DT
            if x_p >= s:
                x_p = s
                strike_t = t
                v_imp = max(v_p, 0.0)
        t += _DT

    if strike_t is None:
        strike_t, v_imp = _T_CAP, 0.0

    # 泄气 + 气动效率折减末动能
    gap = (bore - ball) / 2.0
    leak = min(0.5, dev.leak_coeff * 4.0 * gap / ball) if ball > 0 else 0.5
    v_raw = v_b
    if exit_t is None and v_b > 0:
        # 未在积分内出膛（气压提前归位）：按匀速滑行估出膛时刻
        exit_t = t + max(length - x_b, 0.0) / v_b
    energy_j = 0.5 * m_b * v_raw ** 2 * dev.aero_efficiency * (1.0 - leak)
    v_m_s = math.sqrt(2.0 * energy_j / m_b) if energy_j > 0 else 0.0

    t_fire_ms = strike_t * 1000.0
    # 回位稳定（天梯复位）：模型见 settle_time_ms（刚度/预压影响复位速度）
    v_reb = dev.piston_head_restitution * v_imp
    t_settle_ms = settle_time_ms(v_imp, k, x0, m_p,
                                 dev.piston_head_restitution)
    strike_abs = tl.release_ms + t_fire_ms
    settle_done = strike_abs + t_settle_ms
    margin = tl.next_pickup_ms - settle_done
    exit_abs = tl.release_ms + (exit_t * 1000.0 if exit_t is not None
                                else float("inf"))
    # 无有效压气时 p_rise_ms = None（与退化路径 _degenerate 哨兵一致）
    p_rise_abs = ((tl.release_ms + t_rise * 1000.0) if t_rise is not None
                  else None)

    return Ballistics(
        v_m_s=v_m_s, energy_j=energy_j, p_max_kpa=p_max,
        swept_cm3=A_cyl * s * f * 1e6,  # m³ → cm³
        useful_stroke_mm=useful * 1000.0,
        leak_ratio=leak, gap_mm=gap,
        strike_ms=strike_abs, v_impact_m_s=v_imp, v_rebound_m_s=v_reb,
        t_fire_ms=t_fire_ms, t_settle_ms=t_settle_ms,
        settle_done_ms=settle_done, return_margin_ms=margin,
        exit_ms=exit_abs, p_rise_ms=p_rise_abs,
    )


def _degenerate(tl: Timeline) -> Ballistics:
    """行程被切光的退化情形：活塞不动、无压气。"""
    return Ballistics(
        v_m_s=0.0, energy_j=0.0, p_max_kpa=101.3, swept_cm3=0.0,
        useful_stroke_mm=0.0, leak_ratio=0.0, gap_mm=0.0,
        strike_ms=tl.release_ms, v_impact_m_s=0.0, v_rebound_m_s=0.0,
        t_fire_ms=0.0, t_settle_ms=2.0,
        settle_done_ms=tl.release_ms + 2.0,
        return_margin_ms=tl.next_pickup_ms - tl.release_ms - 2.0,
        exit_ms=float("inf"), p_rise_ms=None,
    )
