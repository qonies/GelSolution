# -*- coding: utf-8 -*-
"""运动学层：扇齿角度 → 循环事件时序。

模型约定（详见 components.py 注释）：
  * 齿轮角 0° = 无切齿时的活塞拾取角；
  * 前切 n 齿 → 拾取角 = n × 齿距角（活塞被拾取的时刻推迟，相对凸轮事件后移）；
  * 后切 m 齿 → 释放角 = 满齿弧 − m × 齿距角（活塞提前释放）；
  * 活塞行程 = 剩余齿数 × 齿距角 / 满齿弧 × 满行程；
  * 凸轮槽事件（推嘴时序）固定在扇齿上，不随切齿移动。

变转速循环（✅ 用户说明）：拉动天梯受力的弧段（拾取角→释放角）电机带弹簧
  负载转得慢；释放天梯后的空转弧段接近空载转速，转得快。事件时刻按
  分段角度-时间转换（拉动段用负载转速、空转段用空载转速）；
  固定转速模式无更多电机信息，按匀速处理。
"""
from dataclasses import dataclass
from typing import Optional

from .params import SimConfig
from .components import DeviceParams
from . import motor as _motor


@dataclass
class Timeline:
    period_ms: float        # 单循环周期（拉动段 + 空转段）
    rof_rps: float          # 射速（发/秒）
    loaded_rpm: float       # 拉动段电机转速（弹簧负载）或固定转速
    free_rpm: float         # 空转段电机转速（近空载；固定模式=loaded_rpm）
    motor_desc: str         # 电机模式描述（报告/界面用）
    motor_stall: bool       # 曲线模式：峰值扭矩 ≥ 堵转扭矩（电机堵转）
    torque_peak_ratio: Optional[float]  # 峰值负载扭矩 / 堵转扭矩（固定模式为 None）
    batt_desc: str          # 电池描述（报告/界面用）
    batt_sag: Optional[float]       # 电压保持率（None=未配置电池）
    batt_shots: Optional[float]     # 理论续航（发；None=未配置电池）
    pickup_deg: float       # 活塞拾取角（切齿后）
    release_deg: float      # 活塞释放角（切齿后）
    remain_teeth: int       # 切齿后剩余啮合齿数
    stroke_mm: float        # 活塞拉控行程
    stroke_ratio: float     # 行程 / 满行程
    pickup_ms: float        # 本循环活塞拾取时刻
    retract_start_ms: float  # 推嘴开始缩回时刻（=拾取+1齿距，凸轮几何锚定，弧段换算）
    release_ms: float       # 活塞释放时刻
    window_start_ms: float  # 供蛋窗口开始（推嘴完全缩回）
    window_end_ms: float    # 供蛋窗口结束（推嘴开始回位）
    window_end_deg: float   # 回位起始有效角（含延时器占孔角）
    seat_ms: float          # 推嘴回位完成（气密就位）
    seat_deg: float         # 回位完成有效角（含延时器占孔角）
    delay_ms: float         # 延时器推迟量 ms（角度级几何换算；未装为 0）
    next_pickup_ms: float   # 下一循环活塞拾取时刻


def build_timeline(cfg: SimConfig, dev: DeviceParams) -> Timeline:
    ratio = cfg.ratio_value
    rpm, minfo, batt = _motor.resolve_loaded_rpm(cfg, dev)
    if minfo is None:
        motor_desc = "固定 %.0f RPM（负载系数 %.2f）" % (rpm, cfg.load_factor)
        batt_desc = "固定转速模式（电池不参与计算）"
        batt_sag = None
        batt_shots = None
    else:
        if minfo.stall:
            motor_desc = "%s（电机堵转：峰值负载 %0.0f mN·m ≥ 堵转 %0.0f mN·m）" \
                % (minfo.model, minfo.torque_peak_mnm, minfo.stall_torque_mnm)
        else:
            motor_desc = "%s 曲线负载 %.0f RPM（峰值扭矩为堵转的 %.0f%%）" \
                % (minfo.model, rpm, minfo.torque_peak_mnm / minfo.stall_torque_mnm * 100)
        if batt is None:
            batt_desc = "未配置电池（按曲线电压 %.1fV 理想电源计算）" % _motor.CURVE_REF_V
            batt_sag = None
            batt_shots = None
        else:
            sag_pct = (1.0 - batt.sag) * 100
            if sag_pct > 0.1:
                batt_desc = "%dS %.0fmAh %.0fC（最高 %.0fA）→ 负载电压 %.1fV（跌落 %.0f%%）" \
                    % (batt.cells, batt.capacity_mah, batt.c_rate, batt.i_max_a,
                       batt.v_eff, sag_pct)
            else:
                batt_desc = "%dS %.0fmAh %.0fC（最高 %.0fA）→ 供电充足，无明显压降" \
                    % (batt.cells, batt.capacity_mah, batt.c_rate, batt.i_max_a)
            batt_sag = batt.sag
            batt_shots = batt.shots
    pitch = dev.sector_pitch_deg
    full_arc = dev.sector_full_teeth * pitch          # 满齿弧（默认 16×11.25=180°）
    pickup_deg = cfg.front_cut * pitch                # 前切 → 拾取推迟
    release_deg = full_arc - cfg.rear_cut * pitch     # 后切 → 提前释放
    remain = dev.sector_full_teeth - cfg.front_cut - cfg.rear_cut

    stroke_ratio = max(remain, 0) * pitch / full_arc
    stroke_mm = dev.piston_full_stroke_mm * stroke_ratio

    # 变转速循环：拉动弧段（拾取→释放）用负载转速，空转弧段用近空载转速；
    # 角度→时间按分段转换（固定转速模式两段相同，退化为匀速）
    rpm_free = rpm if minfo is None else minfo.no_load_rpm * cfg.load_factor
    msdeg_l = 60000.0 * ratio / (360.0 * max(rpm, 1.0))       # 拉动段 ms/°
    msdeg_f = 60000.0 * ratio / (360.0 * max(rpm_free, 1.0))  # 空转段 ms/°

    pickup_ms = pickup_deg * msdeg_f
    release_ms = pickup_ms + (release_deg - pickup_deg) * msdeg_l

    def _t(a: float) -> float:
        """扇齿齿轮角 a（0~360）→ 时刻 ms（分段：空转/拉动/空转）。"""
        if a <= pickup_deg:
            return a * msdeg_f
        if a <= release_deg:
            return pickup_ms + (a - pickup_deg) * msdeg_l
        return release_ms + (a - release_deg) * msdeg_f

    # 周期 = 空转弧（释放→360 与 0→拾取）+ 拉动弧（拾取→释放）
    period_ms = ((360.0 - release_deg + pickup_deg) * msdeg_f
                 + (release_deg - pickup_deg) * msdeg_l)

    retract_start_ms = _t(dev.cam_retract_start_deg)
    window_start_ms = _t(dev.cam_retract_end_deg)

    # 改装件：延时器推迟推嘴回位（角度级几何：拉桥柱一圈 10 孔、每孔 36°，
    #         延时器使柱子加粗一倍、额外占 1 孔 → 回位起始/完成角 +占孔角；
    #         推迟量按所在弧段转速换算，固定模式下 = 周期 × 占孔数/总孔数）；
    #         切拉桥旗使推嘴回位完成提前（气密更早就位，供蛋窗口不变）
    delay_deg = (dev.delay_device_holes / dev.cam_hole_count * 360.0
                 if cfg.install_delay else 0.0)
    window_end_deg = min(dev.cam_return_start_deg + delay_deg, 359.999)
    seat_deg = min(dev.cam_return_end_deg + delay_deg, 359.999)
    window_end_ms = _t(window_end_deg)
    flag_ms = dev.flag_cut_advance_ms if cfg.cut_flag else 0.0
    seat_ms = max(_t(seat_deg) - flag_ms, window_end_ms)
    delay_ms = (_t(seat_deg) - _t(dev.cam_return_end_deg)
                if cfg.install_delay else 0.0)

    next_pickup_ms = period_ms + pickup_ms

    return Timeline(
        period_ms=period_ms, rof_rps=1000.0 / period_ms,
        loaded_rpm=rpm, free_rpm=rpm_free, motor_desc=motor_desc,
        motor_stall=(minfo.stall if minfo is not None else False),
        torque_peak_ratio=(minfo.torque_peak_mnm / minfo.stall_torque_mnm
                           if minfo is not None else None),
        batt_desc=batt_desc, batt_sag=batt_sag, batt_shots=batt_shots,
        pickup_deg=pickup_deg, release_deg=release_deg, remain_teeth=remain,
        stroke_mm=stroke_mm, stroke_ratio=stroke_ratio,
        pickup_ms=pickup_ms, release_ms=release_ms,
        retract_start_ms=retract_start_ms,
        window_start_ms=window_start_ms, window_end_ms=window_end_ms,
        window_end_deg=window_end_deg,
        seat_ms=seat_ms, seat_deg=seat_deg, delay_ms=delay_ms,
        next_pickup_ms=next_pickup_ms,
    )
