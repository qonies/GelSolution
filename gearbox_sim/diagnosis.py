# -*- coding: utf-8 -*-
"""判定与结论建议：打齿 / 气密时序 / 供蛋窗口 / 啮合齿数 / 压气匹配，
以及切齿方案枚举对比与建议（是否需要延时器、是否需要切拉桥旗）。"""
from dataclasses import dataclass, replace
from typing import List

from .params import SimConfig
from .components import DeviceParams
from .timing import build_timeline
from .dynamics import compute
from .feeding import evaluate
from .ballistics import compute as compute_ballistics

LEVEL_OK, LEVEL_WARN, LEVEL_DANGER = "通过", "警告", "危险"


@dataclass
class Check:
    name: str
    level: str
    detail: str


@dataclass
class SchemeRow:
    """一枚切齿方案（供枚举对比表使用）。"""
    front_cut: int
    rear_cut: int
    stroke_mm: float
    return_margin_ms: float
    seal_margin_ms: float
    window_ms: float
    v_m_s: float         # 水弹初速估算
    level: str          # 整体最差判定
    problems: str       # 简短问题说明
    score: float        # 综合裕量得分（越大越好）


def seal_margin_ms(tl, bal) -> float:
    """气密裕量 = 弹后压力显著建立时刻 − 推嘴回位完成时刻（负值=压力建立时嘴未到位）。"""
    if bal.p_rise_ms is None:
        return 999.0  # 无有效压气，无气密风险（哨兵值）
    return bal.p_rise_ms - tl.seat_ms


def run_checks(cfg: SimConfig, dev: DeviceParams, tl, dyn, feed, bal) -> List[Check]:
    th = cfg.threshold
    checks: List[Check] = []

    # 0. 电机负载（曲线模式：峰值扭矩 vs 堵转扭矩；固定模式不计算）
    if tl.motor_stall:
        checks.append(Check("电机负载", LEVEL_DANGER,
                            "电机堵转：拾取瞬间峰值负载扭矩达堵转扭矩的 %.0f%%，"
                            "无法完成循环（换大扭矩电机/降齿轮比齿数负荷/减弱弹簧）"
                            % (tl.torque_peak_ratio * 100)))
    elif tl.torque_peak_ratio is None:
        checks.append(Check("电机负载", LEVEL_OK,
                            "固定转速模式（未启用电机曲线，不计算电机负载）"))
    elif tl.torque_peak_ratio > 0.8:
        checks.append(Check("电机负载", LEVEL_WARN,
                            "峰值负载扭矩达堵转扭矩的 %.0f%% → 接近堵转，电池/电机发热风险"
                            % (tl.torque_peak_ratio * 100)))
    else:
        checks.append(Check("电机负载", LEVEL_OK,
                            "峰值负载扭矩为堵转的 %.0f%%，负载转速 %.0f RPM，余量充足"
                            % (tl.torque_peak_ratio * 100, tl.loaded_rpm)))

    # 0.5 电池放电（配置了电池时：电压跌落 + 续航）
    if tl.batt_sag is None:
        checks.append(Check("电池放电", LEVEL_OK,
                            "未配置电池（按理想电源计算，不计电压跌落）"))
    else:
        sag_pct = (1.0 - tl.batt_sag) * 100
        if sag_pct > 25:
            checks.append(Check("电池放电", LEVEL_DANGER,
                                "电压跌落 %.0f%%（保持率仅 %.0f%%）→ 电池放电能力严重不足，"
                                "射速/初速大幅下降，换大容量或高倍率电池"
                                % (sag_pct, tl.batt_sag * 100)))
        elif sag_pct > 10:
            checks.append(Check("电池放电", LEVEL_WARN,
                                "电压跌落 %.0f%% → 电池放电能力偏弱，建议更大容量或更高倍率"
                                % sag_pct))
        else:
            checks.append(Check("电池放电", LEVEL_OK,
                                "电压保持 %.0f%%，放电能力充足；理论续航约 %.0f 发"
                                % (tl.batt_sag * 100, tl.batt_shots or 0)))

    # 1. 打齿风险（回位裕量来自弹道层的联动积分：撞击时刻受气垫缓冲影响）
    if bal.return_margin_ms < 0:
        checks.append(Check("打齿风险", LEVEL_DANGER,
                            "回位裕量 %.2f ms：活塞尚未归位扇齿即再次啮合 → 高概率打齿"
                            % bal.return_margin_ms))
    elif bal.return_margin_ms < th.gear_clash_margin_ms:
        checks.append(Check("打齿风险", LEVEL_WARN,
                            "回位裕量 %.2f ms，低于安全阈值 %.1f ms → 高射速下有打齿隐患"
                            % (bal.return_margin_ms, th.gear_clash_margin_ms)))
    else:
        checks.append(Check("打齿风险", LEVEL_OK,
                            "回位裕量 %.2f ms ≥ %.1f ms，活塞可完全归位"
                            % (bal.return_margin_ms, th.gear_clash_margin_ms)))

    # 2. 气密时序（推嘴归位须早于弹后压力显著建立；开孔段内无压力不判漏气）
    sm = seal_margin_ms(tl, bal)
    if bal.p_rise_ms is None:
        checks.append(Check("气密时序", LEVEL_OK,
                            "本行程未进入密封段（无有效压气），无气密风险"))
    elif sm < 0:
        checks.append(Check("气密时序", LEVEL_DANGER,
                            "压力显著建立早于推嘴回位完成 %.2f ms → 气密不严、漏气掉速"
                            % (-sm)))
    elif sm < th.air_seal_margin_ms:
        checks.append(Check("气密时序", LEVEL_WARN,
                            "气密裕量仅 %.2f ms（要求 ≥ %.1f ms）→ 时序偏紧，可能漏气"
                            % (sm, th.air_seal_margin_ms)))
    else:
        checks.append(Check("气密时序", LEVEL_OK,
                            "气密裕量 %.2f ms，推嘴在压力建立前就位" % sm))

    # 3. 供蛋窗口
    if feed.window_ms < feed.min_ms * 0.5:
        checks.append(Check("供蛋窗口", LEVEL_DANGER,
                            "窗口 %.1f ms，不足 %.1f ms 要求的一半 → 大概率空发/供不上蛋"
                            % (feed.window_ms, feed.min_ms)))
    elif feed.window_ms < feed.min_ms:
        checks.append(Check("供蛋窗口", LEVEL_WARN,
                            "窗口 %.1f ms < 最小供蛋时间 %.1f ms（%s）→ 供蛋不稳"
                            % (feed.window_ms, feed.min_ms, feed.mode)))
    else:
        checks.append(Check("供蛋窗口", LEVEL_OK,
                            "窗口 %.1f ms ≥ %.1f ms（%s），富余 %.1f ms"
                            % (feed.window_ms, feed.min_ms, feed.mode, feed.slack_ms)))

    # 供蛋方式上限（射速 vs 供蛋机构最高速率）
    if tl.rof_rps > feed.max_rps:
        checks.append(Check("供蛋速率", LEVEL_DANGER,
                            "射速 %.1f 发/秒 超过「%s」最高 %.0f 发/秒 → 供蛋机构跟不上，必空发"
                            % (tl.rof_rps, feed.mode, feed.max_rps)))
    elif tl.rof_rps > feed.max_rps * 0.9:
        checks.append(Check("供蛋速率", LEVEL_WARN,
                            "射速 %.1f 发/秒 已达「%s」上限 %.0f 发/秒 的 90%% 以上 → 供蛋接近极限"
                            % (tl.rof_rps, feed.mode, feed.max_rps)))
    else:
        checks.append(Check("供蛋速率", LEVEL_OK,
                            "射速 %.1f 发/秒，在「%s」上限 %.0f 发/秒 之内"
                            % (tl.rof_rps, feed.mode, feed.max_rps)))

    # 4. 啮合齿数
    if tl.remain_teeth < th.min_remain_teeth:
        checks.append(Check("啮合齿数", LEVEL_DANGER,
                            "切齿后仅剩 %d 齿（建议 ≥ %d）→ 半齿啮合/脱齿风险"
                            % (tl.remain_teeth, th.min_remain_teeth)))
    else:
        checks.append(Check("啮合齿数", LEVEL_OK,
                            "剩余 %d 齿啮合" % tl.remain_teeth))

    # 5. 压气匹配
    if dyn.air_index < th.low_air_index:
        checks.append(Check("压气匹配", LEVEL_WARN,
                            "压气指数 %.2f < %.2f（水桶效应：min(行程比 %.0f%%, 气缸系数 %.2f)）→ 压气不足，初速下降"
                            % (dyn.air_index, th.low_air_index,
                               tl.stroke_ratio * 100, dev.cylinder_factor[cfg.cylinder])))
    else:
        checks.append(Check("压气匹配", LEVEL_OK,
                            "压气指数 %.2f，压气充足" % dyn.air_index))

    # 6. 初速判定
    if bal.v_m_s > th.velocity_warn_ms:
        checks.append(Check("初速", LEVEL_WARN,
                            "估算初速 %.1f m/s，超过警告阈值 %.0f m/s → 超过80m/s警告，注意安全与法规限制"
                            % (bal.v_m_s, th.velocity_warn_ms)))
    else:
        checks.append(Check("初速", LEVEL_OK,
                            "估算初速 %.1f m/s（动能 %.2f J），未超过 %.0f m/s 阈值"
                            % (bal.v_m_s, bal.energy_j, th.velocity_warn_ms)))

    # 7. 气量与管长匹配
    if bal.useful_stroke_mm < cfg.barrel_length_mm * th.barrel_match_ratio:
        checks.append(Check("气量管长匹配", LEVEL_WARN,
                            "有效推力行程 %.0f mm 不足内管长度 %.0f mm 的一半 → 内管偏长或气量不足，"
                            "后段管长基本无效，建议缩短内管或增大气缸/行程"
                            % (bal.useful_stroke_mm, cfg.barrel_length_mm)))
    else:
        checks.append(Check("气量管长匹配", LEVEL_OK,
                            "有效推力行程 %.0f mm / 内管 %.0f mm，匹配良好"
                            % (bal.useful_stroke_mm, cfg.barrel_length_mm)))

    # 8. 出膛时序（上一发须在下一发拾取前出膛，防滞留/双供）
    if bal.exit_ms == float("inf"):
        checks.append(Check("出膛时序", LEVEL_OK,
                            "无有效出膛（本行程未射出弹丸）"))
    elif bal.exit_ms > tl.next_pickup_ms:
        checks.append(Check("出膛时序", LEVEL_WARN,
                            "上一发估算 %.1f ms 才出膛，晚于下一发拾取 %.1f ms → 滞留/双供风险"
                            % (bal.exit_ms, tl.next_pickup_ms)))
    else:
        checks.append(Check("出膛时序", LEVEL_OK,
                            "出膛早于下一发拾取 %.1f ms，无滞留风险"
                            % (tl.next_pickup_ms - bal.exit_ms)))

    return checks


def worst_level(checks: List[Check]) -> str:
    if any(c.level == LEVEL_DANGER for c in checks):
        return LEVEL_DANGER
    if any(c.level == LEVEL_WARN for c in checks):
        return LEVEL_WARN
    return LEVEL_OK


def summarize_scheme(cfg: SimConfig, dev: DeviceParams) -> SchemeRow:
    """对单个切齿方案跑完整流程并汇总为一行对比数据。"""
    tl = build_timeline(cfg, dev)
    dyn = compute(cfg, dev, tl)
    feed = evaluate(cfg, tl)
    bal = compute_ballistics(cfg, dev, tl, dyn)
    checks = run_checks(cfg, dev, tl, dyn, feed, bal)
    sm = seal_margin_ms(tl, bal)
    th = cfg.threshold

    problems = "；".join(c.name for c in checks if c.level != LEVEL_OK) or "无"
    ratios = [
        bal.return_margin_ms / max(th.gear_clash_margin_ms, 1e-6),
        sm / max(th.air_seal_margin_ms, 1e-6),
        feed.window_ms / max(cfg.min_feed_ms, 1e-6),
        bal.useful_stroke_mm / max(cfg.barrel_length_mm * th.barrel_match_ratio, 1e-6),
    ]
    score = min(ratios)
    return SchemeRow(cfg.front_cut, cfg.rear_cut, tl.stroke_mm,
                     bal.return_margin_ms, sm, feed.window_ms, bal.v_m_s,
                     worst_level(checks), problems, score)


def enumerate_cut_schemes(cfg: SimConfig, max_cut: int = 3) -> List[SchemeRow]:
    """枚举 前切0~N × 后切0~N 的切齿方案并按综合裕量排序。"""
    rows = [summarize_scheme(replace(cfg, front_cut=f, rear_cut=r), cfg.device)
            for f in range(max_cut + 1) for r in range(max_cut + 1)]
    rows.sort(key=lambda r: (r.level != LEVEL_OK, r.level == LEVEL_DANGER, -r.score))
    return rows


def build_conclusion(cfg: SimConfig, best: SchemeRow, bal=None, tl=None) -> List[str]:
    """结论建议：推荐切齿方案 / 是否需要延时器 / 是否需要切拉桥旗 / 初速。"""
    out = []
    th = cfg.threshold
    # 延时器推迟量：取时序层的弧段精确值（变转速下与 周期×占孔比 略有差异）
    delay_ms = tl.delay_ms if tl is not None else 0.0
    hole_ms = (tl.period_ms / cfg.device.cam_hole_count
               if tl is not None else None)

    # 0) 初速概况
    if bal is not None:
        if bal.v_m_s > th.velocity_warn_ms:
            out.append("初速提示：估算初速 %.1f m/s 已超过 %.0f m/s 警告阈值，"
                       "可缩短内管/降低气量/减弱弹簧回调"
                       % (bal.v_m_s, th.velocity_warn_ms))
        else:
            out.append("初速提示：估算初速 %.1f m/s（动能 %.2f J），在 %.0f m/s 阈值以内"
                       % (bal.v_m_s, bal.energy_j, th.velocity_warn_ms))

    # 1) 推荐切齿方案
    if best.level == LEVEL_OK:
        tag = "无需切齿" if best.front_cut == 0 and best.rear_cut == 0 else \
              "前切%d齿 + 后切%d齿" % (best.front_cut, best.rear_cut)
        out.append("推荐切齿方案：%s（回位裕量 %.1f ms / 气密裕量 %.1f ms / 供蛋窗口 %.1f ms / "
                   "初速 %.1f m/s，各项达标）"
                   % (tag, best.return_margin_ms, best.seal_margin_ms, best.window_ms,
                      best.v_m_s))
    else:
        out.append("当前参数下枚举的所有切齿方案均未完全达标（最优为前切%d/后切%d，问题：%s），"
                   "建议调整电机/齿比/弹簧后再选切齿方案"
                   % (best.front_cut, best.rear_cut, best.problems))

    # 2) 延时器（已安装则评估效果；未安装则定量建议）
    delay_ms = tl.delay_ms if tl is not None else 0.0
    if cfg.install_delay:
        if best.window_ms >= cfg.min_feed_ms:
            out.append("延时器：已安装（回位推迟 %.1f ms）—— 供蛋窗口已加宽至 %.1f ms，"
                       "满足 %s 供蛋要求；注意气密裕量同步减小"
                       % (delay_ms, best.window_ms, cfg.feed_mode))
        else:
            gap = cfg.min_feed_ms - best.window_ms
            max_delay = max(min(best.seal_margin_ms - th.air_seal_margin_ms, 50.0), 0.0)
            out.append("延时器：已安装（+%.1f ms）但窗口仍缺 %.1f ms；气密裕量只允许再加 "
                       "%.1f ms —— 需提高延时器延时量、更换高级波轮/压力弹匣或降射速"
                       % (delay_ms, gap, max_delay))
    elif best.window_ms < cfg.min_feed_ms:
        gap = cfg.min_feed_ms - best.window_ms
        max_delay = max(min(best.seal_margin_ms - th.air_seal_margin_ms, 50.0), 0.0)
        if gap <= max_delay:
            hole_rec = ("（≈%.2f 孔）" % (gap / hole_ms)) if hole_ms else ""
            out.append("是否需要延时器：需要 —— 供蛋窗口缺 %.1f ms；气密裕量允许最多加 "
                       "%.1f ms 延时，可满足缺口（推荐延时量 %.1f ms%s）"
                       % (gap, max_delay, gap, hole_rec))
        else:
            out.append("是否需要延时器：延时器不足以解决 —— 供蛋窗口缺 %.1f ms，但气密裕量"
                       "只允许 %.1f ms 延时；需改用更高供蛋上限的方式（高级波轮/压力弹匣）、"
                       "降射速，或接受气密损失"
                       % (gap, max_delay))
    else:
        out.append("是否需要延时器：暂不需要 —— 供蛋窗口 %.1f ms 已满足 %s 供蛋要求"
                   % (best.window_ms, cfg.feed_mode))

    # 3) 切拉桥旗（已切则评估效果；未切则按气密裕量建议）
    # 语义：切掉拉桥旗槽 S 形曲线的缓冲尾 → 推嘴更早、更快恢复闭合；
    #       需搭配更强拉桥簧，否则实际气密时刻会晚于几何提前量
    flag_ms = cfg.device.flag_cut_advance_ms if cfg.cut_flag else 0.0
    flag_note = "（需搭配更强拉桥簧，否则实际恢复会慢于估算）"
    if cfg.cut_flag:
        if best.seal_margin_ms >= th.air_seal_margin_ms:
            out.append("切拉桥旗：已切（S 尾切除，推嘴提前 %.1f ms 回位且恢复更快）—— "
                       "气密裕量 %.1f ms，回位时序正常%s"
                       % (flag_ms, best.seal_margin_ms, flag_note))
        else:
            out.append("切拉桥旗：已切但气密裕量仍为 %.1f ms（要求 ≥ %.1f ms）—— "
                       "提前量不足或后切过多，建议减少后切齿数；另确认拉桥簧"
                       "是否足够强（弱簧会让实际恢复晚于估算）"
                       % (best.seal_margin_ms, th.air_seal_margin_ms))
    elif best.seal_margin_ms < th.air_seal_margin_ms:
        out.append("是否需要切拉桥旗：建议切 —— 推嘴回位偏晚（气密裕量 %.1f ms < %.1f ms），"
                   "切掉 S 形缓冲尾可使推嘴更早更快恢复闭合；也可改用更少后切齿数；"
                   "注意需搭配更强拉桥簧"
                   % (best.seal_margin_ms, th.air_seal_margin_ms))
    else:
        out.append("是否需要切拉桥旗：暂不需要 —— 推嘴回位时序正常")
    return out
