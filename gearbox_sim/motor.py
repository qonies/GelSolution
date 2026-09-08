# -*- coding: utf-8 -*-
"""电机模型：负载转速与堵转判定（准静态法）。

  * 电机转速-扭矩特性近似线性：N(T) = N0 × (1 − T/T堵转)；
  * 曲线数据来自 CHAOLI 官方性能曲线图（测试电压 11.1V，
    见工程目录「电机性能曲线/」）：
      - 超力无刷 4W8：空载 48000 RPM，堵转 473.7 mN·m（222 A）
      - 超力有刷 3W5：空载 35500 RPM，堵转 414.9 mN·m（146 A）
  * 电压工况（✅用户更正）：曲线基于标称 3.7V/芯（11.1V = 3S 标称），
    实际使用均充满至 4.2V/芯（3S 满电 12.6V，较曲线电压高 ~13.5%）——
    配置电池后按满电电压缩放电机性能，电压跌落自满电起算。
  * 准静态法：以拉簧全程的**平均扭矩**求负载转速（扇齿拉簧段转速
    近似恒定）；以拾取瞬间的**峰值扭矩**判断是否堵转；
    忽略电机转子惯性与加速过程（对 ROF 影响为毫秒级瞬态）。

反射关系：扇齿分度圆半径 r = 满行程弧长 ÷ π（满齿弧 180°）；
  弹簧力 F 作用在齿条上 → 扇齿扭矩 τ = F × r → 电机轴 τ_m = τ ÷ 齿轮比。
"""
import math
from dataclasses import dataclass
from typing import Optional

BATT_CELL_V = 3.7    # 锂电每芯标称电压 (V)
FULL_CELL_V = 4.2    # 锂电每芯满电电压 (V)（实际工况，✅用户更正）
CURVE_REF_V = 11.1   # 电机曲线的测试电压 (V = 3.7V × 3S 标称)

MOTOR_CURVES = {
    "超力无刷4W8": {
        "no_load_rpm": 48000.0,
        "stall_torque_mNm": 473.70,
        "no_load_current_a": 3.7,
        "stall_current_a": 222.0,
        "type": "无刷",
        "voltage_v": 11.1,
    },
    "超力有刷3W5": {
        "no_load_rpm": 35500.0,
        "stall_torque_mNm": 414.86,
        "no_load_current_a": 3.5,
        "stall_current_a": 146.0,
        "type": "有刷",
        "voltage_v": 11.1,
    },
}
MOTOR_MODELS = list(MOTOR_CURVES.keys())


@dataclass
class BatteryInfo:
    cells: int            # 电芯数
    capacity_mah: float   # 容量
    c_rate: float         # 放电倍率
    v_nom: float          # 标称电压 = 3.7 × 电芯数（规格展示）
    v_start: float        # 满电起始电压 = 4.2 × 电芯数（实际工况，✅用户更正）
    i_max_a: float        # 最大持续电流 = 容量(Ah) × C
    v_eff: float          # 负载下的有效电压（自满电电压跌落）
    sag: float            # 电压保持率 = 有效电压 ÷ 满电电压（1.0=无跌落）
    i_peak_a: float       # 拾取瞬间电机峰值电流需求
    i_pull_a: float       # 拉簧段平均电流需求
    shots: float          # 理论续航（发；按匀速周期+拉簧平均电流估算，偏保守）


@dataclass
class MotorInfo:
    model: str              # 电机型号
    no_load_rpm: float      # 空载转速（有效电压下）
    stall_torque_mnm: float # 堵转扭矩（有效电压下）
    torque_avg_mnm: float   # 拉簧段平均负载扭矩（反射到电机轴）
    torque_peak_mnm: float  # 峰值负载扭矩（拾取瞬间）
    n_load_rpm: float       # 负载转速
    stall: bool             # 是否堵转（峰值扭矩 ≥ 堵转扭矩）
    v_eff: float            # 有效电压
    sag: float              # 电压保持率


def resolve_loaded_rpm(cfg, dev):
    """返回 (负载转速 RPM, MotorInfo 或 None, BatteryInfo 或 None)。

    未选择电机型号时返回 (标称转速 × 负载系数, None, None)，即固定转速模式
    （固定模式不使用电池数据；负载系数默认 0.8）。
    曲线模式下负载系数作为最终降额乘数（无刷默认 0.9 / 有刷默认 0.8），
    只缩放负载转速，不影响扭矩、堵转与电流计算。
    """
    ratio = cfg.ratio_value
    if not cfg.motor_model:
        return cfg.motor_rpm * cfg.load_factor, None, None

    m = MOTOR_CURVES[cfg.motor_model]
    remain_ratio = max(dev.sector_full_teeth - cfg.front_cut - cfg.rear_cut, 0) \
        / dev.sector_full_teeth
    s = dev.piston_full_stroke_mm / 1000.0 * remain_ratio   # 实际拉程 (m)
    r = dev.piston_full_stroke_mm / 1000.0 / math.pi        # 扇齿分度圆半径 (m)
    k = dev.spring_stiffness[cfg.spring] * 1000.0           # N/m
    x0 = dev.spring_preload_mm / 1000.0

    f_avg = k * (x0 + s / 2.0)     # 拉簧段平均弹簧力 (N)
    f_peak = k * (x0 + s)          # 拾取瞬间峰值弹簧力 (N)
    t_avg = f_avg * r / ratio      # 反射到电机轴的平均扭矩 (N·m)
    t_peak = f_peak * r / ratio
    t_stall_ref = m["stall_torque_mNm"] / 1000.0   # 曲线电压下的堵转扭矩
    i_nl = m["no_load_current_a"]
    i_stall_ref = m["stall_current_a"]

    # 电池（可选）：实际工况按满电电压 4.2V × 电芯数（✅用户更正：电机曲线
    # 基于标称 3.7V/芯，实际使用均充满至 4.2V/芯，如 3S 满电 12.6V）；
    # 最大持续电流 = 容量(Ah) × C
    v_start = CURVE_REF_V
    i_batt_max = None
    batt = None
    if cfg.batt_cells is not None:
        v_start = FULL_CELL_V * cfg.batt_cells
        i_batt_max = cfg.batt_capacity_mah / 1000.0 * cfg.batt_c_rate
        batt = (v_start, i_batt_max)

    # 电机峰值电流需求（曲线电压下；扭矩对应电流与电压无关）
    frac_peak = min(t_peak / t_stall_ref, 1.0)
    i_peak = i_nl + (i_stall_ref - i_nl) * frac_peak

    # 电压跌落：峰值电流需求超过电池能力 → 有效电压按比例下降
    sag = 1.0
    if i_batt_max is not None and i_peak > i_batt_max and i_peak > 0:
        sag = i_batt_max / i_peak
    v_eff = v_start * sag
    scale = v_eff / CURVE_REF_V

    stall = t_peak >= t_stall_ref * scale
    # 负载系数降额（无刷默认 0.9 / 有刷默认 0.8）：曲线为理论值，
    # 实际受摩擦/供电等折损，只缩放负载转速，不影响扭矩与堵转判定
    n_load = 0.0 if stall else m["no_load_rpm"] * scale \
        * (1.0 - t_avg / (t_stall_ref * scale)) * cfg.load_factor

    # 拉簧段平均电流需求（按有效电压下的工作点）
    frac_avg = min(t_avg / (t_stall_ref * scale), 1.0) if t_stall_ref * scale > 0 else 0.0
    i_pull = i_nl + (i_stall_ref * scale - i_nl) * frac_avg
    i_pull = max(i_pull, i_nl)

    info = MotorInfo(
        model=cfg.motor_model,
        no_load_rpm=m["no_load_rpm"] * scale,
        stall_torque_mnm=t_stall_ref * scale * 1000.0,
        torque_avg_mnm=t_avg * 1000.0,
        torque_peak_mnm=t_peak * 1000.0,
        n_load_rpm=n_load,
        stall=stall,
        v_eff=v_eff,
        sag=sag,
    )

    batt_info = None
    if batt is not None:
        period_ms = 60000.0 * ratio / max(n_load, 1.0)
        shots = 3600.0 * cfg.batt_capacity_mah / (i_pull * period_ms) \
            if i_pull > 0 else 0.0
        batt_info = BatteryInfo(
            cells=cfg.batt_cells, capacity_mah=cfg.batt_capacity_mah,
            c_rate=cfg.batt_c_rate, v_nom=BATT_CELL_V * cfg.batt_cells,
            v_start=v_start, i_max_a=i_batt_max,
            v_eff=v_eff, sag=sag, i_peak_a=i_peak, i_pull_a=i_pull,
            shots=shots,
        )

    return max(n_load, 1.0), info, batt_info
