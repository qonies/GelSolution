# -*- coding: utf-8 -*-
"""中文模拟报告生成（一站式流程见 render_full）。"""
import math

from .params import SimConfig
from .components import DeviceParams
from .timing import Timeline, build_timeline
from .dynamics import Dynamics, compute
from .feeding import FeedResult, evaluate
from .ballistics import Ballistics, compute as compute_ballistics
from .diagnosis import (Check, SchemeRow, run_checks, enumerate_cut_schemes,
                        build_conclusion, LEVEL_OK, LEVEL_WARN, LEVEL_DANGER)

_LEVEL_MARK = {LEVEL_OK: "[通过]", LEVEL_WARN: "[警告]", LEVEL_DANGER: "[危险]"}


def _cut_tag(front: int, rear: int) -> str:
    if (front, rear) == (0, 0):
        return "无切"
    parts = []
    if front:
        parts.append("前切%d" % front)
    if rear:
        parts.append("后切%d" % rear)
    return "+".join(parts)


def _scheme_table(rows):
    lines = [
        "  %-8s %8s %10s %10s %10s %7s   %s"
        % ("方案", "行程mm", "回位裕量ms", "气密裕量ms", "供蛋窗口ms", "初速m/s", "判定/问题"),
        "  " + "-" * 88,
    ]
    for r in rows:
        lines.append("  %-8s %8.1f %10.2f %10.2f %10.1f %7.1f   %s%s"
                     % (_cut_tag(r.front_cut, r.rear_cut), r.stroke_mm,
                        r.return_margin_ms, r.seal_margin_ms, r.window_ms, r.v_m_s,
                        _LEVEL_MARK[r.level], "" if r.problems == "无" else "（%s）" % r.problems))
    return "\n".join(lines)


def events_list(cfg: SimConfig, dev: DeviceParams, tl: Timeline, dyn: Dynamics,
                feed: FeedResult, bal: Ballistics):
    """单循环事件列表（文本报告与网页界面共用）。"""
    # 单循环终点 = 下一循环首个事件（✅v3.5：推嘴缩回起点角固定于扇齿 11.25°，
    # 前切 n≥2 时下一循环推嘴缩回早于下一循环活塞拾取，不能固定为拾取）
    margin_note = "回位裕量 %.2f ms" % bal.return_margin_ms
    next_retract_ms = tl.period_ms + tl.retract_start_ms
    if next_retract_ms < tl.next_pickup_ms - 1e-9:
        last = {"t": next_retract_ms, "angle": dev.cam_retract_start_deg + 360,
                "desc": "下一循环推嘴开始缩回（前切：早于下一循环拾取 %.2f ms；%s）"
                        % (tl.next_pickup_ms - next_retract_ms, margin_note)}
    elif next_retract_ms > tl.next_pickup_ms + 1e-9:
        last = {"t": tl.next_pickup_ms, "angle": tl.pickup_deg + 360,
                "desc": "下一循环活塞拾取（%s）" % margin_note}
    else:
        last = {"t": tl.next_pickup_ms, "angle": tl.pickup_deg + 360,
                "desc": "下一循环活塞拾取 = 推嘴开始缩回（前切1，两者重合；%s）"
                        % margin_note}
    return [
        {"t": tl.pickup_ms, "angle": tl.pickup_deg, "desc": "活塞拾取，开始被拉动"},
        {"t": tl.retract_start_ms, "angle": dev.cam_retract_start_deg, "desc": "推嘴开始缩回"},
        {"t": tl.window_start_ms, "angle": dev.cam_retract_end_deg,
         "desc": "推嘴完全缩回 → 供蛋窗口开始"},
        {"t": tl.window_end_ms, "angle": tl.window_end_deg,
         "desc": "推嘴开始回位 → 供蛋窗口结束（窗口 %.1f ms）" % feed.window_ms},
        {"t": tl.release_ms, "angle": tl.release_deg,
         "desc": "活塞释放，弹簧开始推动活塞"},
        {"t": tl.seat_ms, "angle": tl.seat_deg, "desc": "推嘴回位完成（气密就位）"},
        {"t": bal.strike_ms, "angle": None,
         "desc": "活塞撞击气缸头（前冲 %.2f ms，撞击速度 %.1f m/s）"
                 % (bal.t_fire_ms, bal.v_impact_m_s)},
        {"t": bal.exit_ms, "angle": None,
         "desc": "水弹出膛（管内飞行估算，有效推力行程 %.0f mm / 内管 %.0f mm）"
                 % (bal.useful_stroke_mm, cfg.barrel_length_mm)},
        {"t": bal.settle_done_ms, "angle": None, "desc": "活塞回位稳定（裕量止点）"},
        last,
    ]


def render(cfg: SimConfig, dev: DeviceParams, tl: Timeline, dyn: Dynamics,
           feed: FeedResult, bal: Ballistics, checks, rows, conclusions) -> str:
    L = []
    L.append("=" * 62)
    L.append("水弹波箱运作模拟报告（通用二号波 · 数据模拟）")
    L.append("=" * 62)

    L.append("")
    L.append("---------- 模拟配置 ----------")
    if cfg.motor_model:
        from .motor import MOTOR_CURVES
        mc = MOTOR_CURVES[cfg.motor_model]
        mtype = "无刷" if "无刷" in cfg.motor_model else "有刷"
        L.append("电机型号     : %s（%s，%.0fV：空载 %.0f RPM / 堵转 %.0f mN·m）"
                 % (cfg.motor_model, mtype, mc["voltage_v"],
                    mc["no_load_rpm"], mc["stall_torque_mNm"]))
        L.append("负载转速     : %.0f RPM（弹簧负载反射到电机轴，峰值扭矩为堵转的 %.0f%%）"
                 % (tl.loaded_rpm, tl.torque_peak_ratio * 100))
    else:
        L.append("电机标称转速 : %g RPM%s" % (
            cfg.motor_rpm,
            "" if cfg.load_factor == 1.0 else "（负载系数 %.2f → 按 %.0f RPM 计算）"
            % (cfg.load_factor, cfg.motor_rpm * cfg.load_factor)))
    L.append("齿轮比       : %s" % cfg.ratio)
    L.append("切齿方案     : %s（剩余 %g 齿咬合；天梯 %g 齿计 %d 齿，咬合上限 %d 齿）"
             % (_cut_tag(cfg.front_cut, cfg.rear_cut), tl.remain_teeth,
                dev.tappet_teeth, math.ceil(dev.tappet_teeth), dev.tappet_bite()))
    L.append("气缸类型     : %s（气量系数 %.2f，开孔段保留 %.2f）"
             % (cfg.cylinder, dev.cylinder_factor[cfg.cylinder],
                dev.port_retention))
    L.append("气缸规格     : 内径 %.1f mm × 长度 %.0f mm（活塞行程 %.1f mm，LDX 1.0 基准）"
             % (dev.cylinder_bore_mm, dev.cylinder_length_mm, dev.piston_full_stroke_mm))
    L.append("弹簧硬度     : %s（刚度 %.2f N/mm）" % (cfg.spring, dyn.k_n_per_mm))
    L.append("内管/水弹    : 内管 %.0f mm，管径 %s，水弹 %s（单边间隙 %.2f mm，泄气损失 %.0f%%）"
             % (cfg.barrel_length_mm, cfg.barrel_bore, cfg.ball_diameter,
                bal.gap_mm, bal.leak_ratio * 100))
    L.append("供蛋方式     : %s（供蛋能力上限 %.0f 发/秒 = 实际支持发射的能力）"
             % (feed.mode, feed.max_rps))
    mods = []
    if cfg.install_delay:
        mods.append("已安装延时器（占 %.1f 孔，回位推迟 %.1f ms）"
                    % (dev.delay_device_holes, tl.delay_ms))
    if cfg.cut_flag:
        mods.append("已切拉桥旗（S 形缓冲尾切除：回位提前 %.1f ms 且更快恢复；"
                    "需搭配更强拉桥簧）" % dev.flag_cut_advance_ms)
    L.append("改装件       : %s" % ("；".join(mods) if mods else "无"))
    if tl.batt_sag is not None:
        L.append("电池         : %s；理论续航约 %.0f 发"
                 % (tl.batt_desc, tl.batt_shots or 0))

    L.append("")
    L.append("---------- 基本时序 ----------")
    t_loaded = tl.release_ms - tl.pickup_ms
    t_free = tl.period_ms - t_loaded
    if abs(tl.free_rpm - tl.loaded_rpm) > 1.0:
        L.append("扇齿周期     : %.2f ms（射速 %.1f 发/秒）" % (tl.period_ms, tl.rof_rps))
        L.append("  变转速     : 拉动段 %.1f ms @ %.0f RPM（弹簧负载）+ 空转段 %.1f ms @ %.0f RPM（近空载）"
                 % (t_loaded, tl.loaded_rpm, t_free, tl.free_rpm))
    else:
        L.append("扇齿周期     : %.2f ms（射速 %.1f 发/秒，按匀速 %.0f RPM 计算）"
                 % (tl.period_ms, tl.rof_rps, tl.loaded_rpm))
    L.append("活塞行程     : %.1f mm（满行程的 %.0f%%）" % (tl.stroke_mm, tl.stroke_ratio * 100))
    L.append("弹簧储能     : %.0f mJ（理想末速度 %.1f m/s，仅供参考；实际由静止加速并受气垫缓冲）"
             % (dyn.energy_mj, dyn.v_release_m_s))
    L.append("弹簧长度     : 自由 %.0f / 装配 %.0f mm，本配置拉满 %.1f mm（= 装配 − 行程，随切齿变化）"
             % (dev.spring_free_length_mm, dev.spring_installed_length_mm,
                dev.spring_installed_length_mm - tl.stroke_mm))
    L.append("回位稳定     : %.1f ms（撞击 %.1f m/s → 回弹 %.1f m/s；弹簧刚度/预压越大复位越快）"
             % (bal.t_settle_ms, bal.v_impact_m_s, bal.v_rebound_m_s))
    L.append("压气指数     : %.2f | 缸压峰值 %.0f kPa | 有效排量 %.2f cm³"
             % (dyn.air_index, bal.p_max_kpa, bal.swept_cm3))
    L.append("水弹初速     : %.1f m/s（动能 %.2f J）%s"
             % (bal.v_m_s, bal.energy_j,
                " ← 超过 %.0f m/s 警告阈值！" % cfg.threshold.velocity_warn_ms
                if bal.v_m_s > cfg.threshold.velocity_warn_ms else ""))

    L.append("")
    L.append("---------- 循环事件表 ----------")
    L.append("  %10s %10s   %s" % ("时刻(ms)", "扇齿角度", "事件"))
    L.append("  " + "-" * 60)
    for ev in sorted(events_list(cfg, dev, tl, dyn, feed, bal), key=lambda e: e["t"]):
        ang = "%.1f°" % ev["angle"] if ev["angle"] is not None else "-"
        L.append("  %10.2f %10s   %s" % (ev["t"], ang, ev["desc"]))

    L.append("")
    L.append("---------- 判定结果 ----------")
    for c in checks:
        L.append("%s %s : %s" % (_LEVEL_MARK[c.level], c.name, c.detail))

    L.append("")
    L.append("---------- 结论建议 ----------")
    for i, text in enumerate(conclusions, 1):
        L.append("%d. %s" % (i, text))

    L.append("")
    L.append("---------- 切齿方案枚举对比（前切/后切 0~3 齿，按综合裕量排序）----------")
    L.append(_scheme_table(rows))

    L.append("")
    L.append("-" * 62)
    if cfg.motor_model is None and cfg.load_factor == 1.0:
        L.append("提示：当前按电机标称转速计算（未折算负载降速），射速偏高，")
        L.append("      判定偏严苛。选择电机型号后按 CHAOLI 性能曲线计算负载转速。")
    L.append("提示：本报告为简化模型的数据模拟；以下器件参数为估算值，"
             "结论精度依赖校准：")
    L.append("      弹簧刚度表、弹簧预压、活塞质量/行程、气缸内径、凸轮槽角度、"
             "余隙容积、撞击回弹系数。")
    L.append("      实测后通过「器件参数覆盖」修正，或用实测初速反标「气动效率」。")
    return "\n".join(L)


def render_full(cfg: SimConfig) -> str:
    """一站式：跑完整模拟流程并生成中文报告。"""
    dev = cfg.device
    tl = build_timeline(cfg, dev)
    dyn = compute(cfg, dev, tl)
    feed = evaluate(cfg, tl)
    bal = compute_ballistics(cfg, dev, tl, dyn)
    checks = run_checks(cfg, dev, tl, dyn, feed, bal)
    rows = enumerate_cut_schemes(cfg)
    conclusions = build_conclusion(cfg, rows[0], bal, tl)
    return render(cfg, dev, tl, dyn, feed, bal, checks, rows, conclusions)
