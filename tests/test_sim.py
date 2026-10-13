# -*- coding: utf-8 -*-
"""模拟引擎测试：直接 python tests/test_sim.py 运行（无需 pytest）。"""
import io
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from gearbox_sim.params import SimConfig, load_config, ConfigError
from gearbox_sim.timing import build_timeline
from gearbox_sim.dynamics import compute
from gearbox_sim.feeding import evaluate
from gearbox_sim.ballistics import compute as compute_ballistics, settle_time_ms
from gearbox_sim.diagnosis import (run_checks, enumerate_cut_schemes,
                                   seal_margin_ms, LEVEL_OK, LEVEL_WARN,
                                   LEVEL_DANGER)
from gearbox_sim.report import render_full, events_list

FAILS = []


def check(name, cond, detail=""):
    status = "PASS" if cond else "FAIL"
    print("[%s] %s %s" % (status, name, detail))
    if not cond:
        FAILS.append(name)


def make_cfg(**kw):
    base = dict(motor_rpm=30000, load_factor=1.0, ratio="16:1",
                front_cut=0, rear_cut=0, cylinder="70%", spring="M90",
                barrel_length_mm=350.0, barrel_bore="7.5", ball_diameter="7.2",
                feed_mode="普通波轮")
    base.update(kw)
    return SimConfig(**base)


def full(cfg):
    """跑完整流程，返回 (tl, dyn, feed, bal, checks)。"""
    dev = cfg.device
    tl = build_timeline(cfg, dev)
    dyn = compute(cfg, dev, tl)
    feed = evaluate(cfg, tl)
    bal = compute_ballistics(cfg, dev, tl, dyn)
    return tl, dyn, feed, bal, run_checks(cfg, dev, tl, dyn, feed, bal)


# ---- 1. 周期公式 ----
cfg = make_cfg()
tl = build_timeline(cfg, cfg.device)
check("周期公式 60000*R/rpm", abs(tl.period_ms - 32.0) < 1e-9, "T=%.2fms" % tl.period_ms)
check("射速 = 1/T", abs(tl.rof_rps - 1000.0 / 32.0) < 1e-9)

# ---- 2. 切齿时序（满齿 16 齿，每齿 12°（✅v3.9：15 齿距占 180°），满拉弧 192°）----
cfg = make_cfg(front_cut=2, rear_cut=2)
tl = build_timeline(cfg, cfg.device)
check("前切2 → 拾取角 24°", abs(tl.pickup_deg - 24.0) < 1e-9)
check("后切2 → 释放角 168°", abs(tl.release_deg - 168.0) < 1e-9)
check("行程 = 12/14 满行程（✅v4.10 口径：剩余咬合 = min(现存蛋糕齿 16−2−2, 天梯 14) = 12）",
      abs(tl.stroke_mm - 60.5 * 12 / 14) < 1e-9)
check("凸轮事件不随切齿移动（✅v4.6 实测基准：回位完成 160°）",
      abs(tl.seat_ms - 160.0 / 360.0 * 32.0) < 1e-9)

# ---- 3. 动力学 ----
cfg = make_cfg()
tl = build_timeline(cfg, cfg.device)
dyn = compute(cfg, cfg.device, tl)
e_expect = 0.5 * 0.62 * ((cfg.device.spring_preload_mm + 60.5) ** 2
                         - cfg.device.spring_preload_mm ** 2)
check("弹簧储能公式", abs(dyn.energy_mj - e_expect) < 1e-6, "E=%.1fmJ" % dyn.energy_mj)
check("释放速度 = √(2E·效率/m)", abs(dyn.v_release_m_s -
      (2 * e_expect / 1000.0 * 0.85 / 0.02) ** 0.5) < 1e-9,
      "v=%.2f" % dyn.v_release_m_s)
check("释放时刻 192°/360°×T（满拉弧 16×12=192°）",
      abs(tl.release_ms - 192.0 / 360.0 * 32.0) < 1e-9)

tl, dyn, feed, bal, checks = full(make_cfg())
check("回位裕量 = 下一拾取 − 稳定完成", abs(bal.return_margin_ms -
      (tl.next_pickup_ms - bal.settle_done_ms)) < 1e-9,
      "margin=%.2fms" % bal.return_margin_ms)
check("活塞由静止加速（前冲>2ms）", bal.t_fire_ms > 2.0,
      "tfire=%.2fms" % bal.t_fire_ms)

# ---- 4. 高射速打齿预警：40000RPM + 13:1 → T=19.5ms ----
cfg = make_cfg(motor_rpm=40000, ratio="13:1", spring="M90")
tl, dyn, feed, bal, checks = full(cfg)
check("T=19.5ms", abs(tl.period_ms - 19.5) < 1e-9)
check("高射速触发打齿判定", bal.return_margin_ms < 2.0,
      "margin=%.2fms" % bal.return_margin_ms)
lv = {c.name: c.level for c in checks}
check("打齿判定为警告或危险", lv["打齿风险"] in (LEVEL_WARN, LEVEL_DANGER))

# ---- 5. 重后切气密（水桶效应：孔位固定于缸体；✅v0.7.0 开孔段保留模型）----
# 开孔段部分漏气（有漏非全漏）→ 活塞盖孔瞬间弹后压力已建立（>1.1 大气压）
# → 压力建立时刻提前至盖孔时刻；✅v4.6 实测基准（回位完成 160°，seat 较旧
# 估算 164.4° 提前 0.39ms）→ 后切4 三种缸型为临界警告（裕量 +0.07~+0.48），
# 后切5 起转危险；✅v4.10 咬合口径更正（剩余咬合 = min(现存蛋糕齿, 天梯计整齿)）
# 后切4 → 剩余咬合 min(14, 12) = 12、行程比 12/14：70%/50% 缸的 p_rise 均提前
# 至开孔段内（裕量相同 0.57ms），100% 缸仍最紧（0.14ms）
cfg = make_cfg(rear_cut=4, cylinder="100%")
tl, dyn, feed, bal, checks = full(cfg)
sm = bal.p_rise_ms - tl.seat_ms
check("后切4+100%缸 → 气密临界警告（裕量 +0.07ms）",
      0.0 <= sm < 1.0, "seat_margin=%.2fms" % sm)
lv = {c.name: c.level for c in checks}
check("后切4+100%缸 气密判定为警告", lv["气密时序"] == LEVEL_WARN)
cfg = make_cfg(rear_cut=4, cylinder="70%")
tl, dyn, feed, bal, checks = full(cfg)
sm70 = bal.p_rise_ms - tl.seat_ms
lv = {c.name: c.level for c in checks}
check("后切4+70%缸 → 气密临界警告（裕量 +0.14ms）",
      lv["气密时序"] == LEVEL_WARN and sm70 > 0.0, "level=%s" % lv["气密时序"])
cfg = make_cfg(rear_cut=5, cylinder="100%")
tl, dyn, feed, bal, checks = full(cfg)
sm5 = bal.p_rise_ms - tl.seat_ms
lv = {c.name: c.level for c in checks}
check("后切5+100%缸 → 气密危险（裕量 −1.04ms）",
      lv["气密时序"] == LEVEL_DANGER and sm5 < 0.0,
      "level=%s margin=%.2f" % (lv["气密时序"], sm5))
cfg = make_cfg(rear_cut=4, cylinder="50%")
tl, dyn, feed, bal, checks = full(cfg)
sm50 = bal.p_rise_ms - tl.seat_ms
lv = {c.name: c.level for c in checks}
check("后切4+50%缸 → 气密临界警告（孔位居中，裕量 +0.48ms 最宽）",
      lv["气密时序"] == LEVEL_WARN and sm50 >= 0.0,
      "level=%s margin=%.2f" % (lv["气密时序"], sm50))
check("气密裕量排序保持：100%缸最早转危险（70%/50% 缸 p_rise 在开孔段内、裕量相同）",
      sm < sm70 and sm <= sm50, "%.2f < %.2f ≤ %.2f" % (sm, sm70, sm50))

# ---- 6. 压气匹配（水桶效应：min(气缸系数, 行程比)，切齿不叠加缩减）----
cfg = make_cfg(cylinder="50%", front_cut=1, rear_cut=1)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("50%缸 轻切齿 → 压气不足警告（水桶上限 0.5 < 0.6）",
      lv["压气匹配"] == LEVEL_WARN, "air_index=%.2f" % dyn.air_index)
check("50%缸 压气指数 = min(0.5, 行程比)",
      abs(dyn.air_index - min(0.5, tl.stroke_ratio)) < 1e-9)

cfg = make_cfg(front_cut=4, rear_cut=4)   # 70% 缸：✅v4.10 口径 剩余咬合 min(14, 16−8)=8
                                          # → 行程比 8/14≈0.571 < 0.7 → 全程密封
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("70%缸 前后各切4 → 压气匹配警告（行程比 0.571 < 0.6；水桶口径 min(0.7, 行程比) 不变）",
      lv["压气匹配"] == LEVEL_WARN
      and abs(dyn.air_index - min(0.7, tl.stroke_ratio)) < 1e-9,
      "air_index=%.3f" % dyn.air_index)
check("剩余8齿 → 啮合齿数通过", lv["啮合齿数"] == LEVEL_OK)

# 水桶效应弹道验证：行程仍覆盖开孔段 → 有效密封排量与无切相同
b_full50 = full(make_cfg(cylinder="50%"))[3]
b_cut50 = full(make_cfg(cylinder="50%", rear_cut=2))[3]
check("50%缸 后切2（行程仍覆盖开孔段）→ 有效密封排量不变",
      abs(b_cut50.swept_cm3 - b_full50.swept_cm3) < 1e-9,
      "无切 %.2f vs 后切2 %.2f cm³" % (b_full50.swept_cm3, b_cut50.swept_cm3))
# 行程短于开孔段 → 全程密封（无自由段），有效排量 = 全行程排量
cfg = make_cfg(cylinder="50%", front_cut=6, rear_cut=6)
tl, dyn, feed, bal, checks = full(cfg)
a_m2 = math.pi * (cfg.device.cylinder_bore_mm / 2000.0) ** 2
exp_swept = (a_m2 * min(cfg.device.cylinder_factor["50%"]
                        * cfg.device.piston_full_stroke_mm / 1000.0,
                        tl.stroke_mm / 1000.0) * 1e6)
check("50%缸 切至行程短于开孔段 → 全程密封（有效排量 = 全行程排量）",
      abs(bal.swept_cm3 - exp_swept) < 1e-9,
      "swept=%.2f cm³（预期 %.2f）" % (bal.swept_cm3, exp_swept))

# ---- 7. 切光 → 危险 ----
cfg = make_cfg(front_cut=6, rear_cut=6)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("切光 → 啮合齿数危险", lv["啮合齿数"] == LEVEL_DANGER)
check("切光 → 压气匹配警告", lv["压气匹配"] == LEVEL_WARN)
check("切光 → 报告仍可生成", "结论建议" in render_full(make_cfg(front_cut=6, rear_cut=6)))

# ---- 8. 弹道：初速估算 ----
cfg = make_cfg()  # M90 / 70% / 350mm / 7.5管 / 7.2弹
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("默认配置初速度量合理", 60.0 < bal.v_m_s < 85.0, "v=%.1f m/s" % bal.v_m_s)
check("默认配置动能与初速自洽", abs(bal.energy_j - 0.5 * 0.0002 * bal.v_m_s ** 2) < 1e-9)
check("默认配置初速未超阈值", lv["初速"] == LEVEL_OK, "v=%.1f" % bal.v_m_s)

# 强力配置：M110 + 60% 气缸（压力建立快）+ 7.3管配7.3弹（零间隙）→ 初速超 80 警告
cfg = make_cfg(spring="M110", cylinder="60%", barrel_bore="7.3",
               ball_diameter="7.3")
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("强力配置初速 > 80 m/s", bal.v_m_s > 80.0, "v=%.1f m/s" % bal.v_m_s)
check("强力配置触发初速警告", lv["初速"] == LEVEL_WARN)

# 泄气损失：同配置下 7.5 管 + 7.2 弹（大间隙）慢于 7.3 管 + 7.2 弹
v_big_gap = compute_ballistics(make_cfg(barrel_bore="7.5", ball_diameter="7.2"),
                               make_cfg().device,
                               build_timeline(make_cfg(), make_cfg().device),
                               compute(make_cfg(), make_cfg().device,
                                       build_timeline(make_cfg(), make_cfg().device))).v_m_s
v_small_gap = compute_ballistics(make_cfg(barrel_bore="7.3", ball_diameter="7.2"),
                                 make_cfg().device,
                                 build_timeline(make_cfg(), make_cfg().device),
                                 compute(make_cfg(), make_cfg().device,
                                         build_timeline(make_cfg(), make_cfg().device))).v_m_s
check("大间隙泄气 → 初速更低", v_small_gap > v_big_gap,
      "7.3管=%.1f vs 7.5管=%.1f" % (v_small_gap, v_big_gap))

# 内管长度：过短（100mm）来不及做功 → 慢于 350mm
def _v(length):
    c = make_cfg(barrel_length_mm=length)
    d = c.device
    t = build_timeline(c, d)
    return compute_ballistics(c, d, t, compute(c, d, t)).v_m_s
check("短内管初速更低", _v(100.0) < _v(350.0),
      "100mm=%.1f vs 350mm=%.1f" % (_v(100.0), _v(350.0)))

# 切齿缩短行程 → 初速下降
check("切齿后初速下降", _v(350.0) > compute_ballistics(
    make_cfg(rear_cut=3), make_cfg().device,
    build_timeline(make_cfg(rear_cut=3), make_cfg().device),
    compute(make_cfg(rear_cut=3), make_cfg().device,
            build_timeline(make_cfg(rear_cut=3), make_cfg().device))).v_m_s)

# 管长匹配判定：大缸（23.8×60.5）下 350mm 内管全程有推力 → 通过；
# 小缸（50%）+ 超长管 → 有效推力行程不足一半 → 警告
cfg = make_cfg()
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("350mm 内管 → 管长匹配通过", lv["气量管长匹配"] == LEVEL_OK,
      "有效推力 %.0fmm" % bal.useful_stroke_mm)
cfg = make_cfg(cylinder="50%", barrel_length_mm=700)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("50%缸+700mm 内管 → 管长匹配通过（保留模型下有效推力延长至 ~475mm）",
      lv["气量管长匹配"] == LEVEL_OK and bal.useful_stroke_mm > 400,
      "有效推力 %.0fmm" % bal.useful_stroke_mm)
cfg = make_cfg(cylinder="50%", barrel_length_mm=700)
cfg.threshold.barrel_match_ratio = 0.8   # 机制检查：475 < 0.8×700=560 → 警告
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("管长匹配机制：阈值覆盖 0.8 → 警告（475 < 560）",
      lv["气量管长匹配"] == LEVEL_WARN)

# 供蛋方式上限：普通波轮最高 30 发/秒
tl, dyn, feed, bal, checks = full(make_cfg())
lv = {c.name: c.level for c in checks}
check("默认射速 31.2 超普通波轮 30 上限", lv["供蛋速率"] == LEVEL_DANGER,
      "rof=%.1f" % tl.rof_rps)
check("供蛋方式供蛋能力上限 30 发/秒", feed.max_rps == 30.0)
tl, dyn, feed, bal, checks = full(make_cfg(feed_mode="高级波轮"))
lv = {c.name: c.level for c in checks}
check("高级波轮 60 上限 → 射速 31.2 通过", lv["供蛋速率"] == LEVEL_OK)
tl, dyn, feed, bal, checks = full(make_cfg(motor_rpm=40000, ratio="13:1"))
lv = {c.name: c.level for c in checks}
check("高射速 51.3 超普通波轮上限 → 危险", lv["供蛋速率"] == LEVEL_DANGER)
tl, dyn, feed, bal, checks = full(make_cfg(motor_rpm=40000, ratio="13:1",
                                           feed_mode="压力弹匣"))
lv = {c.name: c.level for c in checks}
check("压力弹匣 80 上限 → 射速 51.3 通过", lv["供蛋速率"] == LEVEL_OK)

# ---- 9. 方案枚举排序：通过者排前 ----
rows = enumerate_cut_schemes(make_cfg())
check("枚举 16 方案", len(rows) == 16)
check("排序后无通过者被排到危险者前",
      all(r.level != LEVEL_OK for r in rows) or rows[0].level == LEVEL_OK)

# ---- 9. 电机性能曲线 ----
from gearbox_sim import motor as gmotor

n, info, batt = gmotor.resolve_loaded_rpm(make_cfg(motor_model="超力有刷3W5"),
                                          make_cfg().device)
check("3W5 负载转速（弹簧反射扭矩法；✅v3.9 分度圆半径随满拉弧 192° 缩小）",
      abs(n - 29947.0) < 40.0,
      "n=%.0f RPM" % n)
check("3W5 无电池 → 理想电源", batt is None)
check("3W5 信息完整", info.model == "超力有刷3W5" and not info.stall
      and 18.0 < info.torque_peak_mnm / info.stall_torque_mnm * 100 < 30.0)

tl, dyn, feed, bal, checks = full(make_cfg(motor_model="超力有刷3W5"))
lv = {c.name: c.level for c in checks}
# 变转速：拉动段（192°）按负载转速 n，空转段（168°）按空载转速（3W5 曲线 35500×负载系数）
T_piece = (192.0 / 360.0 * 60000.0 * 16 / n
           + 168.0 / 360.0 * 60000.0 * 16 / (tl.free_rpm))
check("曲线模式分段周期（拉动+空转）", abs(tl.period_ms - T_piece) < 0.2,
      "T=%.2fms（拉动 %.1f + 空转 %.1f）" % (tl.period_ms,
                                             192.0 / 360.0 * 60000.0 * 16 / n,
                                             168.0 / 360.0 * 60000.0 * 16 / tl.free_rpm))
check("曲线模式空转转速 = 空载×负载系数", abs(tl.free_rpm - 35500.0 * 1.0) < 1.0,
      "free=%.0f RPM" % tl.free_rpm)
check("曲线模式 ROF ≈ 分段周期倒数", abs(tl.rof_rps - 1000.0 / T_piece) < 0.2,
      "rof=%.1f" % tl.rof_rps)
check("曲线模式含电机负载判定", "电机负载" in lv and lv["电机负载"] == LEVEL_OK)

tl, dyn, feed, bal, checks = full(make_cfg(motor_model="超力无刷4W8"))
check("4W8 负载转速 > 3W5（更强电机更快）", tl.loaded_rpm > 35000.0,
      "n=%.0f RPM rof=%.1f" % (tl.loaded_rpm, tl.rof_rps))

# 曲线模式负载系数固定为 1（v0.6.4 用户确认）：修改负载系数不生效
tl_a = full(make_cfg(motor_model="超力无刷4W8", load_factor=1.0))[0]
tl_b = full(make_cfg(motor_model="超力无刷4W8", load_factor=0.9))[0]
check("曲线模式负载系数固定为 1（改负载系数不生效）",
      abs(tl_a.loaded_rpm - tl_b.loaded_rpm) < 1e-9
      and abs(tl_a.period_ms - tl_b.period_ms) < 1e-9,
      "%.0f vs %.0f RPM" % (tl_a.loaded_rpm, tl_b.loaded_rpm))

# 堵转：超硬弹簧（覆盖刚度表）+ 13:1 → 峰值扭矩超堵转
cfg = make_cfg(motor_model="超力无刷4W8", ratio="13:1")
cfg.device.spring_stiffness["M90"] = 5.0
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("峰值扭矩超堵转 → 电机堵转危险", tl.motor_stall
      and lv["电机负载"] == LEVEL_DANGER)

# 固定模式不受影响
tl, dyn, feed, bal, checks = full(make_cfg())
lv = {c.name: c.level for c in checks}
check("固定模式无电机负载计算", tl.torque_peak_ratio is None
      and lv["电机负载"] == LEVEL_OK)

# ---- 9b. 电池模型 ----
# 无跌落：4W8 + 4S 2200mAh 40C（持续 88A > 峰值需求 ~58A；✅v3.9 反射半径缩小后峰值需求降）
cfg = make_cfg(motor_model="超力无刷4W8", batt_cells=4,
               batt_capacity_mah=2200.0, batt_c_rate=40.0)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("4S 2200 40C → 供电充足无跌落", abs(tl.batt_sag - 1.0) < 1e-9
      and lv["电池放电"] == LEVEL_OK, "sag=%.3f" % tl.batt_sag)
check("4S 满电 16.8V 提升负载转速（> 曲线 11.1V）", tl.loaded_rpm > 48000.0,
      "n=%.0f" % tl.loaded_rpm)
check("续航估算合理", 1000.0 < tl.batt_shots < 50000.0,
      "shots=%.0f" % tl.batt_shots)

# 压降：4W8 + 3S 1200mAh 30C（持续 36A < 峰值需求 ~41A）→ 轻微跌落警告
cfg = make_cfg(motor_model="超力无刷4W8", batt_cells=3,
               batt_capacity_mah=1200.0, batt_c_rate=30.0)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("3S 1200 30C → 峰值超电池能力", tl.batt_sag < 1.0,
      "sag=%.3f" % tl.batt_sag)
check("轻微跌落 → 电池放电警告", lv["电池放电"] == LEVEL_WARN,
      "sag=%.3f" % tl.batt_sag)

# 严重压降：4W8 + 3S 1000mAh 30C（持续 30A）→ 跌落超 25% → 危险
# （✅v3.9 反射半径缩小后峰值需求 ~43A：3S1100/30C 跌落 ~24% 转为警告，
#   加 3S1000/30C 用例保持危险分支覆盖）
cfg = make_cfg(motor_model="超力无刷4W8", batt_cells=3,
               batt_capacity_mah=1100.0, batt_c_rate=30.0)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("4W8+3S1100 30C 电压跌落约 24% → 警告", lv["电池放电"] == LEVEL_WARN,
      "sag=%.3f" % tl.batt_sag)
check("压降降低负载转速", tl.loaded_rpm < 48000.0)
cfg = make_cfg(motor_model="超力无刷4W8", batt_cells=3,
               batt_capacity_mah=1000.0, batt_c_rate=30.0)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("4W8+3S1000 30C 电压跌落超 25% → 危险", lv["电池放电"] == LEVEL_DANGER,
      "sag=%.3f" % tl.batt_sag)

# 无电池 → 电池判定显示未配置
tl, dyn, feed, bal, checks = full(make_cfg(motor_model="超力无刷4W8"))
lv = {c.name: c.level for c in checks}
check("未配置电池 → 判定为未配置", lv["电池放电"] == LEVEL_OK
      and tl.batt_sag is None)

# ---- 9c. 改装件：延时器 / 切拉桥旗 ----
tl0, d0, f0, b0, c0 = full(make_cfg())
margin0 = b0.p_rise_ms - tl0.seat_ms

win = lambda t: t.window_end_ms - t.window_start_ms
cfgd = make_cfg(install_delay=True)
cfgd.device.delay_hold_deg = 36.0        # 旧默认口径（=原 1 孔；✅v4.7 默认改 0）
tl, dyn, feed, bal, checks = full(cfgd)
check("延时器推迟推嘴回位完成", tl.seat_ms > tl0.seat_ms)
check("延时器加宽供蛋窗口", win(tl) > win(tl0),
      "window %.1f→%.1f" % (win(tl0), win(tl)))
check("延时 36°（=原 1 孔口径）= 周期×1/10",
      abs(tl.delay_ms - tl0.period_ms / 10.0) < 1e-9,
      "delay=%.2f ms (T=%.1f ms)" % (tl.delay_ms, tl0.period_ms))
check("气密裕量随延时器等量减小",
      abs((b0.p_rise_ms - tl0.seat_ms) - (bal.p_rise_ms - tl.seat_ms) - tl.delay_ms) < 1e-6)
check("拉桥柱几何：缩回起点角 = 柱位角 0 − 接触提前角 30° = −30°（✅v4.6 实测基准，拾取前缩回）",
      make_cfg().device.cam_pillar_tooth == 0.0
      and make_cfg().device.cam_retract_start_deg == -30.0)

tl, dyn, feed, bal, checks = full(make_cfg(cut_flag=True))
check("切拉桥旗提前推嘴回位完成", tl.seat_ms < tl0.seat_ms)
check("切拉桥旗：脱离于最大幅度结束 → 供蛋窗口不收窄（回位起始 = 最大幅度结束角）",
      abs(tl.window_end_ms - tl0.window_end_ms) < 1e-9
      and abs(win(tl) - win(tl0)) < 1e-9)
check("切拉桥旗增大气密裕量",
      (bal.p_rise_ms - tl.seat_ms) > (b0.p_rise_ms - tl0.seat_ms))

# 组合 + 下限保护：大延时 + 切旗，回位完成不得早于回位起始
cfg = make_cfg(install_delay=True, cut_flag=True)
cfg.device.delay_hold_deg = 225.0      # 225°（=原 6.25 孔）≈ 20ms @32ms 周期
cfg.device.flag_cut_advance_ms = 10.0
tl, dyn, feed, bal, checks = full(cfg)
check("回位完成不早于回位起始（下限保护）", tl.seat_ms >= tl.window_end_ms)

# 延时器加宽供蛋窗口（1.8ms → +10ms ≈ 11.8ms；✅v0.7.1 窗口判定已移除，仅时序信息）
cfg = make_cfg(install_delay=True)
cfg.device.delay_hold_deg = 112.5      # 112.5°（=原 3.125 孔）≈ 10ms @32ms 周期
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("延时 112.5° → 供蛋窗口加宽至 ~11.8ms（窗口判定已移除）",
      abs(win(tl) - 11.8) < 0.2 and "供蛋窗口" not in lv,
      "window=%.1f" % win(tl))

# ---- 10. 配置解析与校验 ----
cfg = load_config(json.loads(json.dumps({
    "电机": {"型号": "超力有刷3W5"},
    "齿轮比": "13:1", "切齿": {"前切齿数": 1, "后切齿数": 2},
    "气缸类型": "100%", "弹簧": "M110",
    "内管长度mm": 300, "内管管径": "7.5", "水弹直径": "7.3",
    "电池": {"电芯数": 3, "容量mAh": 1400, "放电倍率": 30},
    "改装": {"安装延时器": True, "切拉桥旗": False},
    "供蛋": {"方式": "压力弹匣", "最小供蛋时间ms": 25.0},
    "器件参数覆盖": {"活塞满行程mm": 60.0, "水弹质量g": 0.15},
    "判定阈值覆盖": {"打齿安全裕量ms": 3.0, "初速警告阈值m/s": 90.0},
})))
check("解析-齿轮比", cfg.ratio == "13:1")
check("解析-曲线模式负载系数固定 1", cfg.load_factor == 1.0)
check("解析-供蛋方式 压力弹匣", cfg.feed_mode == "压力弹匣")
check("解析-切齿", cfg.front_cut == 1 and cfg.rear_cut == 2)
check("解析-内管参数", cfg.barrel_length_mm == 300.0 and cfg.barrel_bore == "7.5"
      and cfg.ball_diameter == "7.3")
check("解析-电池", cfg.batt_cells == 3 and cfg.batt_capacity_mah == 1400.0
      and cfg.batt_c_rate == 30.0)
check("解析-改装", cfg.install_delay is True and cfg.cut_flag is False)
check("解析-最小供蛋时间ms 已弃用（键被接受并忽略）",
      not hasattr(cfg, "min_feed_ms") and not hasattr(cfg, "feed_min_ms"))
check("解析-器件覆盖", cfg.device.piston_full_stroke_mm == 60.0
      and cfg.device.gel_mass_g == 0.15)
check("解析-阈值覆盖", cfg.threshold.gear_clash_margin_ms == 3.0
      and cfg.threshold.velocity_warn_ms == 90.0)

# 曲线模式（选型号）负载系数固定 1、标称转速锁定曲线值（v0.6.4 ✅用户确认）；
# 固定转速模式默认 0.8 可覆盖
cfg = load_config(json.loads(json.dumps({"电机": {"型号": "超力无刷4W8"}})))
check("解析-曲线模式负载系数固定 1（无刷）", cfg.load_factor == 1.0
      and cfg.motor_rpm == 48000.0)
cfg = load_config(json.loads(json.dumps({"电机": {"型号": "超力有刷3W5"}})))
check("解析-曲线模式负载系数固定 1（有刷）", cfg.load_factor == 1.0
      and cfg.motor_rpm == 35500.0)
cfg = load_config(json.loads(json.dumps({"电机": {"标称转速RPM": 30000}})))
check("解析-负载默认（固定转速 0.8）", cfg.load_factor == 0.8)
cfg = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000, "负载系数": 0.7}})))
check("解析-固定模式负载系数可覆盖（0.7）", cfg.load_factor == 0.7)

# 输入默认值：齿轮比 13:1 / 气缸类型 50% / 内管长度 210mm
cfg = load_config(json.loads(json.dumps({"电机": {"标称转速RPM": 30000}})))
check("解析-默认齿轮比 13:1", cfg.ratio == "13:1")
check("解析-默认气缸 50%", cfg.cylinder == "50%")
check("解析-默认内管 210mm", cfg.barrel_length_mm == 210.0)

for bad in [{"电机": {"标称转速RPM": 30000}, "齿轮比": "99:1"},
            {"电机": {"标称转速RPM": -5}},
            {"电机": {"标称转速RPM": 30000}, "弹簧": "M120"},
            {"电机": {"标称转速RPM": 30000}, "气缸类型": "90%"},
            {"电机": {"标称转速RPM": 30000}, "切齿": {"前切齿数": -1}},
            {"电机": {"标称转速RPM": 30000}, "内管管径": "7.4"},
            {"电机": {"标称转速RPM": 30000}, "水弹直径": "7.5"},
            {"电机": {"标称转速RPM": 30000}, "内管管径": "7.3", "水弹直径": "7.5"},
            {"电机": {"标称转速RPM": 30000}, "内管长度mm": 1000},
            {"电机": {"标称转速RPM": 30000, "型号": "超力无刷4W8"},
             "电池": {"电芯数": 1}},
            {"电机": {"标称转速RPM": 30000}, "电池": {"电芯数": 3}},
            {"电机": {"标称转速RPM": 30000}, "改装": {"安装延时器": "是"}},
            {"电机": {"标称转速RPM": 30000}, "切齿": {"前切齿数": True}},
            {"电机": {"标称转速RPM": 30000}, "器件参数覆盖": {"不存在的参数": 1}}]:
    try:
        load_config(bad)
        check("校验报错 %s" % json.dumps(bad, ensure_ascii=False), False, "未报错")
    except ConfigError as e:
        check("校验报错: %s" % e, True)

# ---- 11. 完整报告可生成 ----
for kw in [dict(), dict(motor_rpm=40000, ratio="13:1"),
           dict(rear_cut=4, spring="M110", cylinder="100%",
                barrel_bore="7.3", ball_diameter="7.3")]:
    text = render_full(make_cfg(**kw))
    check("报告生成 (%s)" % kw, "循环事件表" in text and "结论建议" in text)

text = render_full(make_cfg())
check("报告含初速行", "水弹初速" in text)
check("报告含内管/水弹配置行", "内管/水弹" in text and "水弹出膛" in text)

# ---- 12. v0.5.9 回归：事件表时刻 / 默认管径 / 曲线模式RPM可省 / 退化哨兵 ----
cfg = make_cfg()
tl = build_timeline(cfg, cfg.device)
check("推嘴缩回起点时刻 = 缩回起点角 × 空转ms/°（✅v4.6 柱位角0 − 提前30° = −30°）",
      abs(tl.retract_start_ms + 30.0 / 360.0 * tl.period_ms) < 1e-9,
      "t=%.2fms" % tl.retract_start_ms)
tl, dyn, feed, bal, checks = full(make_cfg())
ev = [e for e in events_list(cfg, cfg.device, tl, dyn, feed, bal)
      if e["desc"].startswith("推嘴开始缩回")][0]
check("事件表推嘴缩回时刻与角度一致（缩回起点 −30°，早于拾取）",
      abs(ev["t"] - tl.retract_start_ms) < 1e-9
      and ev["t"] < tl.pickup_ms,
      "t=%.2fms" % ev["t"])

cfg = load_config(json.loads(json.dumps({"电机": {"标称转速RPM": 30000}})))
check("解析-默认管径 7.5（与 default.json/网页/基线统一）", cfg.barrel_bore == "7.5")

cfg = load_config(json.loads(json.dumps({"电机": {"型号": "超力无刷4W8"}})))
check("解析-曲线模式可省略标称RPM（缺省按曲线空载转速）",
      cfg.motor_model == "超力无刷4W8" and cfg.motor_rpm == 48000.0)
cfg = load_config(json.loads(json.dumps(
    {"电机": {"型号": "超力有刷3W5", "标称转速RPM": 0}})))
check("解析-曲线模式RPM=0 不报错（按曲线空载转速）", cfg.motor_rpm == 35500.0)

cfg = make_cfg()
cfg.device.piston_full_stroke_mm = 0.0   # 行程切光 → 退化路径
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("退化行程 p_rise_ms 为 None（哨兵统一）", bal.p_rise_ms is None)
check("退化行程 气密判定通过（无有效压气）", lv["气密时序"] == LEVEL_OK)
check("退化行程 气密裕量哨兵 999", seal_margin_ms(tl, bal) == 999.0)

# ---- 13. v0.6.0 回归：满压长度派生 + 天梯复位模型 ----
d0 = make_cfg().device
check("默认满压长度 = 装配长 − 满行程（102.5−60.5=42.0）",
      abs(d0.spring_compressed_length_mm
          - (d0.spring_installed_length_mm - d0.piston_full_stroke_mm)) < 1e-9,
      "满压=%.1f mm" % d0.spring_compressed_length_mm)
cfg = make_cfg(rear_cut=4)
tl, dyn, feed, bal, checks = full(cfg)
check("切齿后拉满长度 = 装配长 − 实际行程（随切齿派生，✅v4.10 口径 后切4 剩余咬合 12）",
      abs((cfg.device.spring_installed_length_mm - tl.stroke_mm)
          - (102.5 - 60.5 * 12 / 14)) < 1e-9,
      "拉满=%.3f mm（行程 %.3f）" % (cfg.device.spring_installed_length_mm - tl.stroke_mm,
                                     tl.stroke_mm))

cfg = make_cfg()
tl, dyn, feed, bal, checks = full(cfg)
k_n = dyn.k_n_per_mm * 1000.0
expect = settle_time_ms(bal.v_impact_m_s, k_n,
                        cfg.device.spring_preload_mm / 1000.0,
                        cfg.device.piston_mass_g / 1000.0,
                        cfg.device.piston_head_restitution)
check("回位稳定 = 简谐回位公式 2·atan(v_r/(ω·x0))/ω",
      abs(bal.t_settle_ms - expect) < 1e-9,
      "settle=%.2fms（回弹 %.2f m/s）" % (bal.t_settle_ms, bal.v_rebound_m_s))

check("同撞击速度下：刚度越大复位越快",
      settle_time_ms(5.0, 850.0, 0.0625, 0.02, 0.5)
      < settle_time_ms(5.0, 450.0, 0.0625, 0.02, 0.5),
      "k850 %.2f vs k450 %.2f ms"
      % (settle_time_ms(5.0, 850.0, 0.0625, 0.02, 0.5),
         settle_time_ms(5.0, 450.0, 0.0625, 0.02, 0.5)))
check("同条件下：撞击越重复位越久",
      settle_time_ms(8.0, 620.0, 0.0625, 0.02, 0.5)
      > settle_time_ms(4.0, 620.0, 0.0625, 0.02, 0.5),
      "v8 %.2f vs v4 %.2f ms"
      % (settle_time_ms(8.0, 620.0, 0.0625, 0.02, 0.5),
         settle_time_ms(4.0, 620.0, 0.0625, 0.02, 0.5)))

c_lo = make_cfg(); c_lo.device.spring_preload_mm = 40.0
c_hi = make_cfg(); c_hi.device.spring_preload_mm = 80.0
b_lo = full(c_lo)[3]
b_hi = full(c_hi)[3]
check("压缩长度（预压）越大 → 天梯复位越快",
      b_hi.t_settle_ms < b_lo.t_settle_ms,
      "预压80 %.2f vs 预压40 %.2f ms" % (b_hi.t_settle_ms, b_lo.t_settle_ms))

c_e = make_cfg(); c_e.device.piston_head_restitution = 0.2
b_e = full(c_e)[3]
check("回弹系数可覆盖且影响复位时间",
      b_e.t_settle_ms < bal.t_settle_ms
      and abs(b_e.v_rebound_m_s - 0.2 * bal.v_impact_m_s) < 1e-9,
      "系数0.2 %.2f vs 默认 %.2f ms" % (b_e.t_settle_ms, bal.t_settle_ms))

# ---- 14. v0.6.2 回归：电池满电 4.2V/芯（实际工况）----
n_nobatt, _, _ = gmotor.resolve_loaded_rpm(make_cfg(motor_model="超力无刷4W8"),
                                           make_cfg().device)
cfg = make_cfg(motor_model="超力无刷4W8", batt_cells=3,
               batt_capacity_mah=2200.0, batt_c_rate=40.0)
_, _, b_3s = gmotor.resolve_loaded_rpm(cfg, cfg.device)
check("3S 满电起始电压 = 4.2×3 = 12.6V（标称仍记 11.1V）",
      abs(b_3s.v_start - 12.6) < 1e-9 and abs(b_3s.v_nom - 11.1) < 1e-9,
      "满电 %.1fV / 标称 %.1fV" % (b_3s.v_start, b_3s.v_nom))
check("无跌落时 v_eff = 满电电压，负载转速高于曲线电压工况",
      abs(b_3s.v_eff - 12.6) < 1e-9, "v_eff=%.2fV" % b_3s.v_eff)
n_3s, _, _ = gmotor.resolve_loaded_rpm(cfg, cfg.device)
check("3S 满电负载转速 > 曲线电压负载转速（12.6 > 11.1V）",
      n_3s > n_nobatt, "%.0f vs %.0f RPM" % (n_3s, n_nobatt))
cfg = make_cfg(motor_model="超力无刷4W8", batt_cells=3,
               batt_capacity_mah=1100.0, batt_c_rate=30.0)
_, _, b_sag = gmotor.resolve_loaded_rpm(cfg, cfg.device)
check("电压跌落自满电起算：v_eff = 12.6 × sag",
      abs(b_sag.v_eff - 12.6 * b_sag.sag) < 1e-9,
      "sag=%.3f v_eff=%.2fV" % (b_sag.sag, b_sag.v_eff))

# ---- 15. v0.6.3 回归：新增电机曲线 超力无刷3W9 / 超力有刷3W3 ----
check("新型号已注册（共 4 款）",
      set(gmotor.MOTOR_MODELS) == {"超力无刷4W8", "超力无刷3W9",
                                   "超力有刷3W5", "超力有刷3W3"})
n39, info39, _ = gmotor.resolve_loaded_rpm(make_cfg(motor_model="超力无刷3W9"),
                                           make_cfg().device)
check("3W9 负载转速（弹簧反射扭矩法；✅v3.9 满拉弧 192°）", abs(n39 - 32702.0) < 40.0,
      "n=%.0f RPM" % n39)
check("3W9 峰值扭矩占堵转比例合理（非堵转）",
      not info39.stall
      and 15.0 < info39.torque_peak_mnm / info39.stall_torque_mnm * 100 < 30.0)
check("3W9 曲线模式负载系数固定 1",
      load_config(json.loads(json.dumps(
          {"电机": {"型号": "超力无刷3W9"}}))).load_factor == 1.0)
n33, info33, _ = gmotor.resolve_loaded_rpm(make_cfg(motor_model="超力有刷3W3"),
                                           make_cfg().device)
check("3W3 负载转速（弹簧反射扭矩法；✅v3.9 满拉弧 192°）", abs(n33 - 28073.0) < 40.0,
      "n=%.0f RPM" % n33)
check("3W3 峰值扭矩占堵转比例合理（非堵转）",
      not info33.stall
      and 15.0 < info33.torque_peak_mnm / info33.stall_torque_mnm * 100 < 30.0)
check("3W3 曲线模式负载系数固定 1",
      load_config(json.loads(json.dumps(
          {"电机": {"型号": "超力有刷3W3"}}))).load_factor == 1.0)
check("3W3 曲线模式可省略标称RPM（缺省按曲线空载转速 32800）",
      load_config(json.loads(json.dumps(
          {"电机": {"型号": "超力有刷3W3"}}))).motor_rpm == 32800.0)
n48, _, _ = gmotor.resolve_loaded_rpm(make_cfg(motor_model="超力无刷4W8"),
                                      make_cfg().device)
check("负载转速排序 4W8 > 3W9 > 3W3", n48 > n39 > n33,
      "%.0f / %.0f / %.0f" % (n48, n39, n33))
tl39, dyn39, feed39, bal39, checks39 = full(make_cfg(motor_model="超力无刷3W9"))
lv39 = {c.name: c.level for c in checks39}
check("3W9 全流程模拟：电机负载判定通过", lv39["电机负载"] == LEVEL_OK)
tl33, dyn33, feed33, bal33, checks33 = full(make_cfg(motor_model="超力有刷3W3"))
lv33 = {c.name: c.level for c in checks33}
check("3W3 全流程模拟：电机负载判定通过", lv33["电机负载"] == LEVEL_OK)

# ---- 16. v0.6.4 回归：曲线模式锁定 负载系数=1 / 标称转速=曲线空载 ----
try:
    load_config(json.loads(json.dumps(
        {"电机": {"型号": "超力无刷4W8", "负载系数": 0.9}})))
    check("曲线模式显式负载系数 → 报错", False, "未报错")
except ConfigError as e:
    check("曲线模式显式负载系数 → 报错",
          "负载系数" in str(e) and "固定为 1" in str(e), str(e))
try:
    load_config(json.loads(json.dumps(
        {"电机": {"型号": "超力有刷3W3", "标称转速RPM": 30000}})))
    check("曲线模式标称RPM与曲线值不符 → 报错", False, "未报错")
except ConfigError as e:
    check("曲线模式标称RPM与曲线值不符 → 报错", "标称转速RPM" in str(e), str(e))
cfg = load_config(json.loads(json.dumps(
    {"电机": {"型号": "超力有刷3W3", "标称转速RPM": 32800}})))
check("曲线模式标称RPM等于曲线值 → 通过", cfg.motor_rpm == 32800.0
      and cfg.load_factor == 1.0)

# ---- 17. v0.6.5 回归：供蛋速率更新（高级波轮 60 / 压力弹匣 80）----
check("高级波轮上限 60 发/秒", make_cfg(feed_mode="高级波轮").feed_max_rps == 60.0)
check("压力弹匣供蛋能力上限 80 发/秒",
      make_cfg(feed_mode="压力弹匣").feed_max_rps == 80.0)
check("旧名称「压力」兼容映射到「压力弹匣」",
      load_config(json.loads(json.dumps(
          {"电机": {"标称转速RPM": 30000},
           "供蛋": {"方式": "压力"}}))).feed_mode == "压力弹匣")

# ---- 18. v0.6.7 回归：弹簧预压派生 + 标称转速上限 ----
cfg = load_config(json.loads(json.dumps({"电机": {"标称转速RPM": 30000}})))
check("默认预压 = 自由 − 装配（62.5）",
      abs(cfg.device.spring_preload_mm
          - (cfg.device.spring_free_length_mm
             - cfg.device.spring_installed_length_mm)) < 1e-9
      and abs(cfg.device.spring_preload_mm - 62.5) < 1e-9,
      "预压=%.1f" % cfg.device.spring_preload_mm)
cfg = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000},
     "器件参数覆盖": {"弹簧自由长度mm": 175.0}})))
check("仅覆盖自由长度 → 预压按新长度派生（72.5）",
      abs(cfg.device.spring_preload_mm - 72.5) < 1e-9,
      "预压=%.1f" % cfg.device.spring_preload_mm)
cfg = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000},
     "器件参数覆盖": {"弹簧自由长度mm": 175.0, "弹簧预压mm": 58.0}})))
check("显式覆盖预压优先（不派生）", cfg.device.spring_preload_mm == 58.0)
try:
    load_config(json.loads(json.dumps(
        {"电机": {"标称转速RPM": 30000},
         "器件参数覆盖": {"弹簧装配长度mm": 170.0}})))
    check("装配长度 ≥ 自由长度 → 报错", False, "未报错")
except ConfigError as e:
    check("装配长度 ≥ 自由长度 → 报错", "弹簧预压mm" in str(e), str(e))
try:
    load_config(json.loads(json.dumps({"电机": {"标称转速RPM": 100001}})))
    check("标称转速RPM 超上限 → 报错", False, "未报错")
except ConfigError as e:
    check("标称转速RPM 超上限 → 报错", "标称转速RPM" in str(e), str(e))
cfg = load_config(json.loads(json.dumps({"电机": {"标称转速RPM": 100000}})))
check("标称转速RPM = 上限 100000 → 通过", cfg.motor_rpm == 100000.0)

# ---- 19. v0.6.9 回归：天梯齿数（默认 13.5 计 14 齿；半齿计整齿；拉程缩放封顶）----
cfgD = make_cfg()
tlD = build_timeline(cfgD, cfgD.device)
check("默认天梯 13.5 计 14 齿 → 咬合 14、行程 60.5（默认行为不变锚点）",
      tlD.remain_teeth == 14 and abs(tlD.stroke_mm - 60.5) < 1e-9)
d_tmp = make_cfg().device
d_tmp.tappet_teeth = 11.5
check("咬合进位：13.5→14 / 11.5→12（半齿磨低仍计整齿）",
      make_cfg().device.tappet_bite() == 14 and d_tmp.tappet_bite() == 12)
d_tmp.tappet_teeth = 12.5
check("咬合进位：12.5→13（任意半齿向上取整）", d_tmp.tappet_bite() == 13)
cfg115 = make_cfg(); cfg115.device.tappet_teeth = 11.5
tl115 = build_timeline(cfg115, cfg115.device)
check("11.5 计 12 齿 → 行程 = 12/14×60.5",
      abs(tl115.remain_teeth - 12) < 1e-9
      and abs(tl115.stroke_mm - 12.0 / 14.0 * 60.5) < 1e-9,
      "行程=%.3f mm" % tl115.stroke_mm)
check("11.5 行程比 = 12/14", abs(tl115.stroke_ratio - 12.0 / 14.0) < 1e-9)
cfg115r = make_cfg(rear_cut=5); cfg115r.device.tappet_teeth = 11.5
tl115r = build_timeline(cfg115r, cfg115r.device)
check("11.5 + 后切5 → 剩余咬合 min(12, 16−5) = 11、行程 = 11/14×60.5（✅v4.10 口径）",
      abs(tl115r.remain_teeth - 11) < 1e-9
      and abs(tl115r.stroke_mm - 11.0 / 14.0 * 60.5) < 1e-9,
      "行程=%.3f mm" % tl115r.stroke_mm)
cfg115c = make_cfg(front_cut=6, rear_cut=4); cfg115c.device.tappet_teeth = 11.5
_, _, _, _, checks115c = full(cfg115c)
lv115c = {c.name: c.level for c in checks115c}
check("11.5 + 前6后4 → 剩余咬合 min(12, 6) = 6 齿 → 啮合齿数危险（%g 格式化正常）",
      lv115c["啮合齿数"] == LEVEL_DANGER)
cfglong = make_cfg(); cfglong.device.tappet_teeth = 16.0
tllong = build_timeline(cfglong, cfglong.device)
check("天梯 16 计 16 齿 > 原装咬合 14 → 拉程按满行程 60.5 封顶",
      tllong.remain_teeth == 16 and abs(tllong.stroke_mm - 60.5) < 1e-9)
_, dynD, _, balD, _ = full(cfgD)
_, dyn115, _, bal115, _ = full(cfg115)
check("11.5 拉程短 → 储能与初速下降",
      dyn115.energy_mj < dynD.energy_mj and bal115.v_m_s < balD.v_m_s,
      "储能 %.0f vs %.0f mJ；初速 %.1f vs %.1f m/s"
      % (dyn115.energy_mj, dynD.energy_mj, bal115.v_m_s, balD.v_m_s))
# 回弹时间经联动积分受天梯齿数影响：拉程短 → 弹簧做功少、气垫做功路径变化 →
# 撞击剩余速度与回位稳定略降（方向由联动模型决定，锁定当前行为）
check("11.5 撞击剩余速度与回位稳定时间略降（联动模型方向）",
      bal115.v_impact_m_s < balD.v_impact_m_s
      and bal115.t_settle_ms < balD.t_settle_ms,
      "撞击 %.3f vs %.3f m/s；回位稳定 %.3f vs %.3f ms"
      % (bal115.v_impact_m_s, balD.v_impact_m_s,
         bal115.t_settle_ms, balD.t_settle_ms))
check("回位稳定模型对撞击速度单调（齿数经撞击速度影响回弹时间的机理）",
      settle_time_ms(5.0, 620.0, 0.0625, 0.02, 0.5)
      > settle_time_ms(3.0, 620.0, 0.0625, 0.02, 0.5))
cmD = make_cfg(motor_model="超力无刷4W8")
cm115 = make_cfg(motor_model="超力无刷4W8"); cm115.device.tappet_teeth = 11.5
nDm, _, _ = gmotor.resolve_loaded_rpm(cmD, cmD.device)
n115m, _, _ = gmotor.resolve_loaded_rpm(cm115, cm115.device)
check("11.5 拉程短 → 弹簧反射扭矩小 → 电机负载转速更高（曲线模式）",
      n115m > nDm, "%.0f vs %.0f RPM" % (n115m, nDm))
cfg = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000}, "器件参数覆盖": {"天梯齿数": 11.5}})))
check("解析-天梯齿数覆盖 11.5", cfg.device.tappet_teeth == 11.5)
for bad_val in [0, 30, "11.5", True]:
    try:
        load_config(json.loads(json.dumps(
            {"电机": {"标称转速RPM": 30000},
             "器件参数覆盖": {"天梯齿数": bad_val}})))
        check("校验报错 天梯齿数=%r" % (bad_val,), False, "未报错")
    except ConfigError as e:
        check("校验报错 天梯齿数=%r" % (bad_val,), "天梯齿数" in str(e), str(e))

# ---- 20. v0.7.0 回归：开孔段保留系数（4 横槽漏气，有漏但非全漏）----
def _vel(**kw):
    cfg = make_cfg(**kw)
    dev = cfg.device
    tl = build_timeline(cfg, dev)
    bal = compute_ballistics(cfg, dev, tl, compute(cfg, dev, tl))
    return bal.v_m_s, bal.p_max_kpa


def _vel_ret(ret):
    cfg = make_cfg()
    cfg.device.port_retention = ret
    dev = cfg.device
    tl = build_timeline(cfg, dev)
    bal = compute_ballistics(cfg, dev, tl, compute(cfg, dev, tl))
    return bal.v_m_s, bal.p_max_kpa


check("默认保留 0.6 → 校准锚点 70缸/350管 ≈70.8（气动效率 0.243 重校准）",
      abs(_vel()[0] - 70.75) < 0.6, "v=%.2f" % _vel()[0])
v0, p0 = _vel_ret(0.0)
v05, p05 = _vel_ret(0.5)
v1, p1 = _vel_ret(1.0)
check("初速随保留系数单调下降（0=全漏最高 → 1=不漏最低）",
      v0 > v05 > v1, "%.2f > %.2f > %.2f" % (v0, v05, v1))
check("缸压峰值随保留系数单调下降（弹丸提前蠕动扩容抑制峰值）",
      p0 > p05 > p1, "%.0f > %.0f > %.0f" % (p0, p05, p1))
v100, _ = _vel(cylinder="100%")
check("保留 1 等效全程密封（70缸保留1 ≈ 100%缸）", abs(v1 - v100) < 0.5,
      "%.2f vs %.2f" % (v1, v100))
cfg = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000},
     "器件参数覆盖": {"开孔段保留系数": 0.3}})))
check("解析-开孔段保留系数覆盖 0.3", cfg.device.port_retention == 0.3)
for bad_val in [-0.1, 1.5, "0.5", True]:
    try:
        load_config(json.loads(json.dumps(
            {"电机": {"标称转速RPM": 30000},
             "器件参数覆盖": {"开孔段保留系数": bad_val}})))
        check("校验报错 开孔段保留系数=%r" % (bad_val,), False, "未报错")
    except ConfigError as e:
        check("校验报错 开孔段保留系数=%r" % (bad_val,),
              "开孔段保留系数" in str(e), str(e))

# ---- 21. v0.7.1 回归：供蛋判定语义（能力上限=实际支持发射的能力）----
# 供蛋方式上限直接与射速比较；「供蛋窗口 vs 1000÷上限」判定移除（10 项判定）
cfg = make_cfg()
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("判定共 10 项（供蛋窗口判定移除）", len(checks) == 10, "n=%d" % len(checks))
check("「供蛋窗口」判定已移除", "供蛋窗口" not in lv)
check("默认射速 30.8 > 普通波轮能力 30 → 供蛋速率危险",
      lv["供蛋速率"] == LEVEL_DANGER and tl.rof_rps > 30.0,
      "rof=%.1f" % tl.rof_rps)
cfg = make_cfg(feed_mode="高级波轮")
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("高级波轮（能力 60）下射速 30.8 → 供蛋速率通过",
      lv["供蛋速率"] == LEVEL_OK and abs(feed.max_rps - 60.0) < 1e-9)
cfg = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000},
     "供蛋": {"方式": "普通波轮", "最小供蛋时间ms": 25.0}})))
check("旧配置 最小供蛋时间ms 被接受并忽略（不再参与判定）",
      cfg.feed_mode == "普通波轮")

# ---- 22. v0.7.2 回归：单循环时序终点 = 下一循环首个事件 ----
# ✅v4.3/v4.6：缩回起点角 = 柱位角 − 接触提前角（默认 −30°，早于拾取）——
# 下一循环推嘴缩回早于下一循环活塞拾取，终点 = 下一循环推嘴开始缩回
def _evs(**kw):
    cfg = make_cfg(**kw)
    dev = cfg.device
    tl = build_timeline(cfg, dev)
    d = compute(cfg, dev, tl)
    f = evaluate(cfg, tl)
    b = compute_ballistics(cfg, dev, tl, d)
    return tl, sorted(events_list(cfg, dev, tl, d, f, b), key=lambda e: e["t"])


tl0, evs0 = _evs()
check("无前切：缩回起点 −30° 早于拾取 → 终点 = 下一循环推嘴开始缩回（早于下一循环拾取）",
      evs0[-1]["desc"].startswith("下一循环推嘴开始缩回")
      and abs(evs0[-1]["t"] - (tl0.period_ms + tl0.retract_start_ms)) < 1e-9
      and evs0[-1]["t"] < tl0.next_pickup_ms - 1e-9)
tl2, evs2 = _evs(front_cut=2)
check("前切2：本循环推嘴开始缩回早于活塞拾取（排序正确）",
      evs2[0]["desc"].startswith("推嘴开始缩回")
      and evs2[0]["t"] < tl2.pickup_ms)
check("前切2：时序终点 = 下一循环推嘴开始缩回（早于下一循环拾取）",
      evs2[-1]["desc"].startswith("下一循环推嘴开始缩回")
      and abs(evs2[-1]["t"] - (tl2.period_ms + tl2.retract_start_ms)) < 1e-9
      and evs2[-1]["t"] < tl2.next_pickup_ms - 1e-9)
tl1, evs1 = _evs(front_cut=1)
check("前切1：下一循环推嘴缩回早于下一循环拾取（缩回 −30° < 拾取 12°）",
      evs1[-1]["desc"].startswith("下一循环推嘴开始缩回")
      and abs(evs1[-1]["t"] - (tl1.period_ms + tl1.retract_start_ms)) < 1e-9
      and evs1[-1]["t"] < tl1.next_pickup_ms - 1e-9)
check("三种前切终点均为下一循环首个事件（时刻 = 周期 + 缩回起点时刻，早于下一循环拾取）",
      all(abs(evsX[-1]["t"] - (tlX.period_ms + tlX.retract_start_ms)) < 1e-9
          and evsX[-1]["t"] < tlX.next_pickup_ms - 1e-9
          for evsX, tlX in ((evs0, tl0), (evs1, tl1), (evs2, tl2))))

# ---- 23. v0.8.2 回归：器件参数覆盖校验（类型/范围/表完整性）+ 延时器孔数除零防护 ----
# 此前 apply_overrides 不校验覆盖值：非数字 → 计算层 TypeError、表缺键 →
# KeyError、拉桥柱孔数 0 → ZeroDivisionError（结论层死代码未防护处），
# 均为英文堆栈；现在统一在解析层给出中文错误
def _override_error(over):
    try:
        load_config(json.loads(json.dumps(
            {"电机": {"标称转速RPM": 30000}, "器件参数覆盖": over})))
        return None
    except ConfigError as e:
        return str(e)


e = _override_error({"拉桥柱孔数": 0})
check("拉桥柱孔数 0 → 中文报错（原 ZeroDivisionError 崩溃）",
      e is not None and "拉桥柱孔数" in e, e)
e = _override_error({"气动效率": "abc"})
check("气动效率 非数字 → 中文报错（原 TypeError 崩溃）",
      e is not None and "必须是数字" in e, e)
e = _override_error({"活塞满行程mm": True})
check("覆盖值为布尔 → 拒绝（与主配置键口径一致）",
      e is not None and "必须是数字" in e, e)
e = _override_error({"气动效率": 5.0})
check("气动效率 5 超范围 → 中文报错（0.01~1）",
      e is not None and "0.01~1" in e, e)
e = _override_error({"弹簧刚度表": "abc"})
check("弹簧刚度表 非对象 → 中文报错",
      e is not None and "弹簧刚度表" in e and "JSON 对象" in e, e)
e = _override_error({"弹簧刚度表": {"M120": 0.9}})
check("弹簧刚度表 缺当前弹簧键 → 中文报错（原 KeyError 崩溃）",
      e is not None and "M90" in e, e)
e = _override_error({"气缸系数表": {"30%": 0.3}})
check("气缸系数表 缺当前缸型键 → 中文报错",
      e is not None and "50%" in e, e)
e = _override_error({"最大供蛋速率表": {"高级波轮": 60.0}})
check("最大供蛋速率表 缺当前方式键 → 中文报错",
      e is not None and "普通波轮" in e, e)
cfgv = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000},
     "器件参数覆盖": {"气动效率": 0.30, "弹簧刚度表": {"M90": 0.62}}})))
check("合法覆盖仍生效（气动效率 0.30 / 单键刚度表替换）",
      cfgv.device.aero_efficiency == 0.30
      and cfgv.device.spring_stiffness == {"M90": 0.62})

# 引擎层防护（绕过解析层直接构造器件参数）：孔数 0 仅影响结论「每孔」换算，
# 延时量已改由 延时度数 角度参数直接计算（✅v4.0），不再依赖孔数
cfgz = make_cfg(install_delay=True)
cfgz.device.cam_hole_count = 0
cfgz.device.delay_hold_deg = 36.0
tlz = build_timeline(cfgz, cfgz.device)
check("引擎层防护：拉桥柱孔数 0 → 不崩、延时按 延时度数 角度计算（36°=周期/10）",
      abs(tlz.delay_ms - tlz.period_ms * 36.0 / 360.0) < 1e-9)
repz = render_full(cfgz)
check("结论层防护：孔数 0 时报告正常生成（无「每孔」换算）",
      "结论建议" in repz and "每孔" not in repz)

# ---- 24. v3.9 回归：扇齿几何实测更正（每齿 12°、满拉弧 192°）----
# ✅用户实测更正：全齿 16 齿，但 15 个齿距占圆周 180° → 每齿 12°（原 11.25° 有误）；
# 满拉弧 = 16 齿 × 12° = 192°（全部 16 齿参与咬合，拉满行程扇齿转 192°）
cfg = make_cfg()
d = cfg.device
check("每齿角默认 12°（15 齿距占 180°）；拉桥柱位置 0 + 提前角 30° → 缩回起点角 −30°（✅v4.6 实测基准）",
      d.sector_pitch_deg == 12.0 and d.cam_pillar_tooth == 0.0
      and d.cam_retract_start_deg == -30.0)
tl = build_timeline(cfg, d)
check("无切满拉弧 = 16×12 = 192°（拾取 0° / 释放 192°）",
      tl.pickup_deg == 0.0 and abs(tl.release_deg - 192.0) < 1e-9)
check("凸轮缩回起点 = 柱位角0 − 提前30° = −30°（拾取前缩回）且早于回位起始（60°）",
      tl.retract_deg == -30.0
      and abs(tl.retract_start_ms + 30.0 / 360.0 * tl.period_ms) < 1e-9
      and d.cam_return_start_deg > d.cam_retract_start_deg)
cfgc = make_cfg(motor_model="超力有刷3W5")
tlc = build_timeline(cfgc, cfgc.device)
check("反射半径随满拉弧参数化：3W5 负载转速升至 ~29947（原 180° 口径 29607）",
      abs(tlc.loaded_rpm - 29947.0) < 40.0, "n=%.0f" % tlc.loaded_rpm)

# ---- 25. v4.0 回归：延时器 提前度数 / 延时度数 角度参数（供弹时序）----
# 提前度数 = 比原拉桥柱更早接触拉桥旗的转角 → 缩回/供蛋窗口开始提前；
# 延时度数 = 比原拉桥柱更晚离开拉桥旗的转角 → 回位起始/完成推迟
d25 = make_cfg().device
check("新参数默认：提前 0° / 延时 0°（✅v4.7 参数直接生效，未配置 = 无延时器）",
      d25.delay_advance_deg == 0.0 and d25.delay_hold_deg == 0.0)

cfg = make_cfg(install_delay=True)
cfg.device.delay_hold_deg = 36.0                 # 延时 36°（旧默认口径）
tl_a = build_timeline(cfg, cfg.device)
cfg.device.delay_advance_deg = 12.0
tl_b = build_timeline(cfg, cfg.device)
msdeg = tl_b.period_ms / 360.0                  # 固定转速模式匀速：ms/°
check("提前度数 12° → 缩回/窗口开始提前 12°（弧段换算 ms，advance_ms 同步）",
      abs(tl_b.retract_start_ms - (tl_a.retract_start_ms - 12.0 * msdeg)) < 1e-6
      and abs(tl_b.window_start_ms - (tl_a.window_start_ms - 12.0 * msdeg)) < 1e-6
      and abs(tl_b.advance_ms - 12.0 * msdeg) < 1e-6)
check("提前度数不改变回位时序（窗口结束/回位完成不变）",
      abs(tl_b.window_end_ms - tl_a.window_end_ms) < 1e-9
      and abs(tl_b.seat_ms - tl_a.seat_ms) < 1e-9)

cfg.device.delay_hold_deg = 72.0
tl_c = build_timeline(cfg, cfg.device)
check("延时度数 72° → 回位起始/完成推迟 72°（推迟量 = 2× 默认 36°）",
      tl_c.window_end_ms > tl_a.window_end_ms
      and abs(tl_c.delay_ms - 2.0 * tl_a.delay_ms) < 1e-6)
check("提前+延时联合加宽供蛋窗口（= 原 + 提前 12° + 延时 36° 对应 ms）",
      abs((tl_c.window_end_ms - tl_c.window_start_ms)
          - (tl_a.window_end_ms - tl_a.window_start_ms)
          - 12.0 * msdeg - tl_a.delay_ms) < 1e-6)

cfg_n = make_cfg()                               # ✅v4.7 参数直接生效：度数非零即生效
cfg_n.device.delay_advance_deg = 30.0
cfg_n.device.delay_hold_deg = 120.0
tl_n = build_timeline(cfg_n, cfg_n.device)
tl_0 = build_timeline(make_cfg(), make_cfg().device)
msdeg0 = tl_0.period_ms / 360.0
check("参数直接生效（✅v4.7）：度数非零即生效（无需安装开关）——提前 30°/延时 120° 全部生效",
      abs(tl_n.advance_ms - 30.0 * msdeg0) < 1e-6
      and abs(tl_n.delay_ms - 120.0 * msdeg0) < 1e-6
      and tl_n.retract_start_ms < tl_0.retract_start_ms
      and tl_n.window_end_ms > tl_0.window_end_ms)

# ✅v4.3：提前度数不再钳制 0°——拉桥柱位置可负，缩回允许早于拾取（负角度/负时刻）
cfg_x = make_cfg(install_delay=True)
cfg_x.device.delay_advance_deg = 30.0
tl_x = build_timeline(cfg_x, cfg_x.device)
check("提前度数 30° → 缩回提前至 −60°（拾取前，负角度/负时刻），窗口开始 40−30=10°",
      tl_x.retract_deg == -60.0 and tl_x.window_start_deg == 10.0
      and abs(tl_x.retract_start_ms + 60.0 * tl_x.period_ms / 360.0) < 1e-9)

# 旧键 延时器占孔数 被接受但忽略（向后兼容，不再参与计算）
cfg_old = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000}, "改装": {"安装延时器": True},
     "器件参数覆盖": {"延时器占孔数": 5.0}})))
tl_old = build_timeline(cfg_old, cfg_old.device)
check("旧键 延时器占孔数 接受但忽略；旧开关兼容（安装延时器=true 且未配置度数 → 按旧默认 36°）",
      cfg_old.device.delay_device_holes == 5.0
      and cfg_old.install_delay
      and abs(tl_old.delay_ms - tl_old.period_ms / 10.0) < 1e-9)

# ✅v4.7 参数直接生效：无需安装开关，仅配置度数即生效；显式度数（含 0）优先于旧开关兼容
cfg_j = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000}, "器件参数覆盖": {"延时器延时度数": 72.0}})))
tl_j = build_timeline(cfg_j, cfg_j.device)
check("参数直接生效（✅v4.7）：JSON 未装开关、仅配置 延时度数 72 → 窗口加宽 72° 弧段",
      cfg_j.install_delay
      and abs(tl_j.delay_ms - 72.0 * tl_j.period_ms / 360.0) < 1e-6)
cfg_k = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000}, "改装": {"安装延时器": True},
     "器件参数覆盖": {"延时器延时度数": 0.0}})))
check("显式 延时度数 0 优先于旧开关兼容（开关=true 不强加 36°）",
      not cfg_k.install_delay and cfg_k.device.delay_hold_deg == 0.0)

# 范围校验（负值 / 超 180°）
try:
    load_config(json.loads(json.dumps(
        {"电机": {"标称转速RPM": 30000},
         "器件参数覆盖": {"延时器提前度数": -5.0}})))
    check("提前度数 -5 → 报错", False, "未报错")
except ConfigError as e:
    check("提前度数 -5 → 报错", "延时器提前度数" in str(e), str(e))
try:
    load_config(json.loads(json.dumps(
        {"电机": {"标称转速RPM": 30000},
         "器件参数覆盖": {"延时器延时度数": 200.0}})))
    check("延时度数 200 → 报错", False, "未报错")
except ConfigError as e:
    check("延时度数 200 → 报错", "延时器延时度数" in str(e), str(e))

# 事件表：缩回/窗口开始事件显示有效角（含提前）并在描述中注明提前量
cfg_ev = make_cfg(install_delay=True)
cfg_ev.device.delay_advance_deg = 6.0
dev_ev = cfg_ev.device
tl_ev = build_timeline(cfg_ev, dev_ev)
d_ev = compute(cfg_ev, dev_ev, tl_ev)
f_ev = evaluate(cfg_ev, tl_ev)
b_ev = compute_ballistics(cfg_ev, dev_ev, tl_ev, d_ev)
evs = sorted(events_list(cfg_ev, dev_ev, tl_ev, d_ev, f_ev, b_ev),
             key=lambda e: e["t"])
re_ev = [e for e in evs if e["desc"].startswith("推嘴开始缩回")][0]
ws_ev = [e for e in evs if e["desc"].startswith("推嘴完全缩回")][0]
check("事件表：缩回/窗口开始显示有效角（−30°−6°=−36°、40°−6°=34°）并注明提前",
      abs(re_ev["angle"] - (-36.0)) < 1e-9
      and abs(re_ev["t"] - tl_ev.retract_start_ms) < 1e-9
      and "提前" in re_ev["desc"]
      and abs(ws_ev["angle"] - 34.0) < 1e-9)

# ---- 27. v4.3/v4.6 回归：拉桥柱位置（凸轮事件角全部随柱位角平移派生）----
cfg = make_cfg()
check("默认 拉桥柱位置 0 + 提前角 30° → 缩回起点角 −30°（✅v4.6 实测基准，拾取前缩回）",
      cfg.device.cam_pillar_tooth == 0.0
      and cfg.device.cam_retract_start_deg == -30.0)
tlp0 = build_timeline(cfg, cfg.device)
check("默认缩回时刻 = −30° 弧段换算（拾取前 2.67 ms @32ms 周期）",
      abs(tlp0.retract_start_ms + 30.0 / 360.0 * tlp0.period_ms) < 1e-9)


def _pillar_tl(pos, front=0):
    """拉桥柱位置用例：经解析层派生缩回起点角（配置镜像 make_cfg 标称参数）。"""
    cfgp = load_config(json.loads(json.dumps(
        {"电机": {"标称转速RPM": 30000, "负载系数": 1.0}, "齿轮比": "16:1",
         "气缸类型": "70%", "弹簧": "M90", "内管长度mm": 350.0,
         "切齿": {"前切齿数": front},
         "器件参数覆盖": {"拉桥柱位置": pos}})))
    return cfgp, build_timeline(cfgp, cfgp.device)


cfgp1, tlp1 = _pillar_tl(1.0)
check("拉桥柱位置 1（第 2 齿下方）→ 缩回起点角派生 −18°（= 12° − 提前 30°）",
      abs(cfgp1.device.cam_retract_start_deg - (-18.0)) < 1e-9
      and abs(tlp1.retract_start_ms - tlp0.period_ms * (-18.0) / 360.0) < 1e-9)
cfgp05, _ = _pillar_tl(0.5)
check("拉桥柱位置 0.5（两齿之间）→ 缩回起点角派生 −24°（= 6° − 提前 30°）",
      abs(cfgp05.device.cam_retract_start_deg - (-24.0)) < 1e-9)
cfgpn, tlpn = _pillar_tl(-2.0)
check("拉桥柱位置 −2 → 缩回起点角派生 −54°（拾取前，负时刻）",
      abs(cfgpn.device.cam_retract_start_deg + 54.0) < 1e-9
      and tlpn.retract_start_ms < 0.0)
cfgpd = load_config(json.loads(json.dumps({"电机": {"标称转速RPM": 30000}})))
check("解析-未配置拉桥柱位置 → 派生缩回起点角 −30°（默认行为）",
      cfgpd.device.cam_retract_start_deg == -30.0)
cfgd = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000}, "器件参数覆盖": {"拉桥柱位置": 1.5}})))
check("解析-拉桥柱位置 1.5 → 缩回起点角派生 −12°（未覆盖缩回起点角时按位置派生）",
      abs(cfgd.device.cam_retract_start_deg - (-12.0)) < 1e-9)
cfgp2, _ = _pillar_tl(2.0)
check("柱位平移派生（✅v4.6）：柱位 2 → 完全缩回 64° / 回位起始 84° / 回位完成 184°（柱位角+40/60/160）",
      abs(cfgp2.device.cam_retract_end_deg - 64.0) < 1e-9
      and abs(cfgp2.device.cam_return_start_deg - 84.0) < 1e-9
      and abs(cfgp2.device.cam_return_end_deg - 184.0) < 1e-9)
# ✅v4.6 用户强调：前切/柱位改变时，拉动活塞（拾取）与拉动拉桥（推嘴缩回）的
# 先后顺序可能不同，时序必须按配置重新数值运算——锁定两类关键情形：
cfgb3, tlb3 = _pillar_tl(3.0)
check("顺序随配置重算：前切0 柱位3 → 缩回（6°）晚于拾取（0°），先后关系翻转",
      abs(tlb3.retract_deg - 6.0) < 1e-9
      and tlb3.retract_start_ms > tlb3.pickup_ms)
cfgmix = load_config(json.loads(json.dumps(
    {"电机": {"型号": "超力有刷3W5"}, "齿轮比": "16:1",
     "切齿": {"前切齿数": 4}, "气缸类型": "70%", "弹簧": "M90",
     "内管长度mm": 350.0, "器件参数覆盖": {"拉桥柱位置": 0}})))
tlmix = build_timeline(cfgmix, cfgmix.device)
msdeg_f = 60000.0 * 16.0 / (360.0 * tlmix.free_rpm)
msdeg_l = 60000.0 * 16.0 / (360.0 * tlmix.loaded_rpm)
check("顺序随配置重算（变转速混合弧段）：前切4 拾取 48°，完全缩回 40°（自由弧）/ "
      "回位起始 60°（拉动弧）按分段转速分别换算",
      abs(tlmix.pickup_deg - 48.0) < 1e-9
      and abs(msdeg_f - msdeg_l) > 0.01
      and abs(tlmix.window_start_ms - 40.0 * msdeg_f) < 1e-6
      and abs(tlmix.window_end_ms - (48.0 * msdeg_f + 12.0 * msdeg_l)) < 1e-6)
cfgd2 = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000},
     "器件参数覆盖": {"拉桥柱位置": 1.5, "推嘴缩回起点角": 30.0}})))
check("显式覆盖 推嘴缩回起点角 优先于 拉桥柱位置派生",
      cfgd2.device.cam_retract_start_deg == 30.0)
e = _override_error({"拉桥柱位置": -100.0})
check("拉桥柱位置 超范围（−100）→ 中文报错", e is not None and "拉桥柱位置" in e, e)

# ---- 28. v4.4/v4.5 回归：拉桥旗接触提前角 + 最大幅度停留区间（超力18比实测）----
# 实测（docs/齿轮测试-1.txt，拾取角 = 0° 基准，柱在第一齿下方 = 位置 0）：
# 接触提前 30° → 缩回起点 −30°；✅v4.5：最大幅度为停留区间——开始达到最大
# 幅度 +40°（= 完全缩回，供蛋窗口开始）、最大幅度结束 +60°（= 回位起始，
# 停留 20°）；拉桥柱与旗完全脱离 +160°（= 气密就位）
cfg18 = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000, "负载系数": 1.0}, "齿轮比": "18:1",
     "器件参数覆盖": {"拉桥柱位置": 0, "拉桥旗接触提前角": 30.0,
                     "推嘴完全缩回角": 40.0, "推嘴回位起始角": 60.0,
                     "推嘴回位完成角": 160.0}})))
tl18 = build_timeline(cfg18, cfg18.device)
check("超力18比实测：柱位0 + 接触提前30° → 缩回起点角 −30°（拾取前开始缩回）",
      abs(cfg18.device.cam_retract_start_deg + 30.0) < 1e-9
      and tl18.retract_start_ms < 0.0)
check("超力18比实测：完全缩回(最大开始) 40° / 回位起始(最大结束) 60° → 供蛋窗口 20°（固定30000RPM×18:1 = 2.0 ms）",
      abs(cfg18.device.cam_retract_end_deg - 40.0) < 1e-9
      and abs(cfg18.device.cam_return_start_deg - 60.0) < 1e-9
      and abs(tl18.window_end_ms - tl18.window_start_ms - 2.0) < 1e-9)
check("超力18比实测：分离 160° = 气密就位（回位完成角）",
      abs(cfg18.device.cam_return_end_deg - 160.0) < 1e-9
      and abs(tl18.seat_deg - 160.0) < 1e-9)
cfgc0 = load_config(json.loads(json.dumps({"电机": {"标称转速RPM": 30000}})))
check("接触提前角默认 30°（✅v4.6 实测基准）→ 柱位0 缩回起点角 −30°",
      cfgc0.device.flag_contact_advance_deg == 30.0
      and cfgc0.device.cam_retract_start_deg == -30.0)
# 切旗语义（✅v4.5 澄清：脱离 = 最大幅度结束时刻，停留段保留、S 形恢复段切除）：
cfgcut = make_cfg(cut_flag=True)
tlcut = build_timeline(cfgcut, cfgcut.device)
tlbase = build_timeline(make_cfg(), make_cfg().device)
check("切旗：脱离于最大幅度结束 → 回位起始 = 最大幅度结束角（供蛋窗口不收窄）",
      abs(tlcut.window_end_deg - tlbase.window_end_deg) < 1e-9
      and abs(win(tlcut) - win(tlbase)) < 1e-9)
check("切旗：回位完成 = 导向回位完成 − 切拉桥旗提前ms（下限保护：不早于回位起始）",
      abs(tlcut.seat_ms - max(tlbase.seat_ms - cfgcut.device.flag_cut_advance_ms,
                              tlcut.window_end_ms)) < 1e-6
      and tlcut.seat_ms >= tlcut.window_end_ms - 1e-9)
# 切旗 + 延时器：延时度数仍生效（✅v4.8 柱维持至最大幅度结束+延时度数才脱离）、提前度数仍生效
cfgdc = make_cfg(install_delay=True, cut_flag=True)
cfgdc.device.delay_hold_deg = 72.0
tldc = build_timeline(cfgdc, cfgdc.device)
check("切旗 + 延时器：延时度数仍生效（✅v4.8 柱维持至最大幅度结束+延时度数才脱离）",
      tldc.delay_ms > 0.0
      and abs(tldc.delay_ms - 72.0 * tlbase.period_ms / 360.0) < 1e-6
      and abs(tldc.window_end_deg - 132.0) < 1e-9)
cfgd_nc = make_cfg(install_delay=True)
cfgd_nc.device.delay_hold_deg = 72.0
tldc_nc = build_timeline(cfgd_nc, cfgd_nc.device)
check("切旗不影响最大供弹窗口：切旗+延时 72° 窗口 = 未切旗+延时 72° 窗口（仅气密提前）",
      abs(tldc.window_end_ms - tldc_nc.window_end_ms) < 1e-9
      and abs((tldc.window_end_ms - tldc.window_start_ms)
              - (tldc_nc.window_end_ms - tldc_nc.window_start_ms)) < 1e-9
      and tldc.seat_ms < tldc_nc.seat_ms)
cfgdc2 = make_cfg(install_delay=True, cut_flag=True)
cfgdc2.device.delay_advance_deg = 6.0
tldc2 = build_timeline(cfgdc2, cfgdc2.device)
check("切旗 + 延时器：提前度数仍生效（缩回起点 = −30−6 = −36°、advance_ms > 0）",
      abs(tldc2.retract_deg - (-36.0)) < 1e-9 and tldc2.advance_ms > 0.0)

# 喷嘴未密闭时段（✅v4.4 新增指标：推嘴开始缩回 → 气密就位；供蛋窗口定义
# 保持「最大开度停留期」不变——无停留齿轮箱/切旗状态窗口为 0 属实）
f18 = evaluate(cfg18, tl18)
check("超力18比：喷嘴未密闭 19.0 ms（−30° 缩回 → 160° 气密；窗口 2.0 ms，喷嘴仍开 ~19ms）",
      abs(f18.nozzle_open_ms - 19.0) < 1e-6 and abs(f18.window_ms - 2.0) < 1e-9)
cfgd3 = load_config(json.loads(json.dumps({"电机": {"标称转速RPM": 30000}})))
tl3 = build_timeline(cfgd3, cfgd3.device)
fd3 = evaluate(cfgd3, tl3)
check("默认二号波：喷嘴未密闭 = 190/360 × 周期（−30° 缩回 → 160° 气密就位，✅v4.6 实测基准）",
      fd3.nozzle_open_ms > 0
      and abs(fd3.nozzle_open_ms - 190.0 / 360.0 * tl3.period_ms) < 1e-6
      and abs(fd3.window_ms - 20.0 / 360.0 * tl3.period_ms) < 1e-6)

# 柱位以未前切（前切=0）的原始齿序为基准、固定于扇齿：前切 n 齿仅使拾取角
# 后移至 n×每齿角，柱位角不变 → 柱位角与拾取角的先后按扇齿实际转角计算
cfgf0, tlf0 = _pillar_tl(1.0, 0)
cfgf2, tlf2 = _pillar_tl(1.0, 2)
check("前切不影响柱位角：前切0/2 同柱位 1 → 缩回起点角均为 −18°（固定于扇齿）",
      abs(tlf0.retract_deg - (-18.0)) < 1e-9 and abs(tlf2.retract_deg - (-18.0)) < 1e-9)
check("前切2 + 柱位 1 → 缩回（−18°）早于拾取（24°），拾取角 = 2×每齿角",
      tlf2.retract_start_ms < tlf2.pickup_ms
      and abs(tlf2.pickup_ms - 24.0 * tlf2.period_ms / 360.0) < 1e-9)
cfgf3, tlf3 = _pillar_tl(5.0, 2)
check("前切2 + 柱位 5 → 缩回（60−30 = 30°）晚于拾取（24°），按扇齿实际转角换算",
      tlf3.retract_start_ms > tlf3.pickup_ms
      and abs(tlf3.retract_start_ms - 30.0 * tlf3.period_ms / 360.0) < 1e-9)

# ---- 26. v0.9.2 回归：负载系数范围校验（0.1~1.0，与 FR-02 / 网页输入框对齐）----
for _bad in (1.5, 0.05, -0.2):
    try:
        load_config(json.loads(json.dumps(
            {"电机": {"标称转速RPM": 30000, "负载系数": _bad}})))
        check("负载系数 %g → 报错" % _bad, False, "未报错")
    except ConfigError as e:
        check("负载系数 %g → 报错" % _bad,
              "负载系数" in str(e) and "0.1~1.0" in str(e), str(e))
_cfg_lo = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000, "负载系数": 0.1}})))
check("负载系数 0.1（下限）→ 通过", _cfg_lo.load_factor == 0.1)
_cfg_hi = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000, "负载系数": 1.0}})))
check("负载系数 1.0（上限）→ 通过", _cfg_hi.load_factor == 1.0)

# ---- 29. v0.9.9 回归：2号延时器实测接入（docs/齿轮测试-2.txt）----
# 超力18比直斜齿（柱位0/常规拉桥）加装 2号延时器实测（与齿轮测试-1 无延时器
# 基线同条件对比）：接触 −30°、完全缩回（最大幅度开始）+40° 不变；最大幅度
# 结束 +60°→+100°、柱旗脱离 +160°→+200°（均 +40°）——即 提前度数 0° /
# 延时度数 40°，由现有 delay_hold_deg 延时模型直接复现（引擎逻辑无改动）
cfg18d = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000, "负载系数": 1.0}, "齿轮比": "18:1",
     "器件参数覆盖": {"拉桥柱位置": 0, "拉桥旗接触提前角": 30.0,
                      "推嘴完全缩回角": 40.0, "推嘴回位起始角": 60.0,
                      "推嘴回位完成角": 160.0, "延时器提前度数": 0.0,
                      "延时器延时度数": 40.0}})))
tl18d = build_timeline(cfg18d, cfg18d.device)
check("2号延时器实测：接触（缩回起点 −30°）与完全缩回（+40°）不受延时器影响",
      abs(cfg18d.device.cam_retract_start_deg + 30.0) < 1e-9
      and abs(cfg18d.device.cam_retract_end_deg - 40.0) < 1e-9
      and abs(tl18d.window_start_ms - tl18.window_start_ms) < 1e-9)
check("2号延时器实测：回位起始（最大幅度结束）60°→100°、柱旗脱离 160°→200°（各推迟 40°）",
      abs(tl18d.window_end_deg - 100.0) < 1e-9
      and abs(tl18d.seat_deg - 200.0) < 1e-9)
check("2号延时器实测：供蛋窗口 20°→60°（固定30000RPM×18:1 周期36ms → 6.0 ms）",
      abs(tl18d.window_end_ms - tl18d.window_start_ms - 6.0) < 1e-9)
check("2号延时器实测：delay_ms = 40° 弧段（4.0 ms）、advance_ms = 0、install_delay 派生",
      abs(tl18d.delay_ms - 4.0) < 1e-9 and abs(tl18d.advance_ms) < 1e-9
      and cfg18d.install_delay)
f18d = evaluate(cfg18d, tl18d)
check("2号延时器实测：喷嘴未密闭 23.0 ms（−30° 缩回 → 200° 气密就位，基线 19.0 ms）",
      abs(f18d.nozzle_open_ms - 23.0) < 1e-6)
# 「改装」预设「2号延时器」（提前 0°/延时 40°）并入配置可解析且生效
with open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "configs", "presets", "改装", "2号延时器.json"),
          encoding="utf-8") as _fd2:
    _d2 = json.load(_fd2)
cfgp_d2 = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000}, "器件参数覆盖": dict(_d2)})))
check("改装预设「2号延时器」：延时 40° / 提前 0° 生效（度数非零即已配置）",
      cfgp_d2.device.delay_hold_deg == 40.0
      and cfgp_d2.device.delay_advance_deg == 0.0 and cfgp_d2.install_delay)

# ---- 30. v0.9.10 回归：咬合口径更正（✅用户更正：切齿切的是蛋糕齿，天梯齿不变
# → 剩余咬合 = min(现存蛋糕齿 = 扇齿满啮合 − 前切 − 后切, 天梯计整齿)，
# 取代旧「咬合上限 14 − 前切 − 后切」错误口径）----
# 前切3+后切4：现存蛋糕齿 16−7 = 9 → 剩余咬合 min(14, 9) = 9（旧口径误算 14−7=7）
cfgf34 = make_cfg(front_cut=3, rear_cut=4)
tlf34 = build_timeline(cfgf34, cfgf34.device)
check("前3后4：剩余咬合 = min(现存蛋糕齿 9, 天梯 14) = 9、行程 = 9/14×60.5 = 38.893mm",
      abs(tlf34.remain_teeth - 9) < 1e-9
      and abs(tlf34.stroke_mm - 9.0 / 14.0 * 60.5) < 1e-9,
      "行程=%.3f mm" % tlf34.stroke_mm)
check("前3后4：时序不受咬合口径影响（拾取 36° / 释放 144°，凸轮事件固定于扇齿）",
      abs(tlf34.pickup_deg - 36.0) < 1e-9 and abs(tlf34.release_deg - 144.0) < 1e-9)
# 切齿合计 ≤2（现存蛋糕齿 ≥ 天梯咬合 14）→ 满行程不减，切齿仅移动拾取/释放时序
cfgf2 = make_cfg(front_cut=2)
tlf2 = build_timeline(cfgf2, cfgf2.device)
check("前切2：现存蛋糕齿 14 ≥ 天梯咬合 14 → 满行程 60.5（仅拾取推迟 24°）",
      abs(tlf2.remain_teeth - 14) < 1e-9 and abs(tlf2.stroke_mm - 60.5) < 1e-9
      and abs(tlf2.pickup_deg - 24.0) < 1e-9)
# min 双向钳制：超短天梯（11.5 计 12）时咬合上限同样生效
cfgt12 = make_cfg(front_cut=2, rear_cut=2); cfgt12.device.tappet_teeth = 11.5
tlt12 = build_timeline(cfgt12, cfgt12.device)
check("天梯 11.5 + 前2后2：剩余咬合 = min(12, 12) = 12、行程 12/14×60.5",
      abs(tlt12.remain_teeth - 12) < 1e-9
      and abs(tlt12.stroke_mm - 12.0 / 14.0 * 60.5) < 1e-9)
# 极限：前6后6（现存蛋糕齿 4）→ 剩余咬合 4、行程比 4/14
cfgf66 = make_cfg(front_cut=6, rear_cut=6)
tlf66 = build_timeline(cfgf66, cfgf66.device)
check("前6后6：剩余咬合 = 4、行程 = 4/14×60.5",
      abs(tlf66.remain_teeth - 4) < 1e-9
      and abs(tlf66.stroke_mm - 4.0 / 14.0 * 60.5) < 1e-9)

# ---- 31. v0.9.11 回归：新增齿比 9:1 / 11:1（✅用户需求）----
for _r, _rv in (("9:1", 9.0), ("11:1", 11.0)):
    cfgr = load_config(json.loads(json.dumps(
        {"电机": {"标称转速RPM": 30000, "负载系数": 1.0}, "齿轮比": _r})))
    tlr = build_timeline(cfgr, cfgr.device)
    check("齿比 %s → ratio_value %.0f、周期 = 60000×%.0f/30000 = %.0f ms（固定模式匀速）"
          % (_r, _rv, _rv, 60000.0 * _rv / 30000.0),
          cfgr.ratio == _r and abs(cfgr.ratio_value - _rv) < 1e-9
          and abs(tlr.period_ms - 60000.0 * _rv / 30000.0) < 1e-9)
# 齿轮预设「9比1」「11比1」并入配置可解析（整区载入口径：齿轮比/切齿/天梯齿数）
for _name in ("9比1", "11比1"):
    with open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "configs", "presets", "齿轮", _name + ".json"),
              encoding="utf-8") as _fg:
        _g = json.load(_fg)
    cfgg = load_config(json.loads(json.dumps(
        {"电机": {"标称转速RPM": 30000}, "齿轮比": _g["齿轮比"],
         "切齿": {"前切齿数": _g.get("前切齿数", 0),
                  "后切齿数": _g.get("后切齿数", 0)},
         "器件参数覆盖": {"天梯齿数": _g["天梯齿数"]}})))
    check("齿轮预设「%s」并入配置可解析（齿轮比 %s）" % (_name, _g["齿轮比"]),
          cfgg.ratio == _g["齿轮比"])
# 齿轮比错误信息列出全部支持档位
try:
    load_config(json.loads(json.dumps(
        {"电机": {"标称转速RPM": 30000}, "齿轮比": "10:1"})))
    check("齿轮比 10:1 → 报错", False, "未报错")
except ConfigError as e:
    check("齿轮比 10:1 → 报错（列出全部支持档位）",
          "齿轮比" in str(e) and "9:1" in str(e) and "11:1" in str(e), str(e))

print()
print()
if FAILS:
    print("失败 %d 项: %s" % (len(FAILS), FAILS))
    sys.exit(1)
print("全部测试通过。")
