# -*- coding: utf-8 -*-
"""模拟引擎测试：直接 python tests/test_sim.py 运行（无需 pytest）。"""
import io
import json
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

# ---- 2. 切齿时序（满齿 16 齿，每齿 11.25°，满弧 180°）----
cfg = make_cfg(front_cut=2, rear_cut=2)
tl = build_timeline(cfg, cfg.device)
check("前切2 → 拾取角 22.5°", abs(tl.pickup_deg - 22.5) < 1e-9)
check("后切2 → 释放角 157.5°", abs(tl.release_deg - 157.5) < 1e-9)
check("行程 = 12/16 满行程", abs(tl.stroke_mm - 60.5 * 12 / 16) < 1e-9)
check("凸轮事件不随切齿移动", abs(tl.seat_ms - 164.4 / 360.0 * 32.0) < 1e-9)

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
check("释放时刻 180°/360°×T", abs(tl.release_ms - 16.0) < 1e-9)

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

# ---- 5. 重后切气密：100% 缸无开孔缓冲 → 压力立即建立 → 危险；70% 缸开孔缓冲 → 放宽 ----
cfg = make_cfg(rear_cut=4, cylinder="100%")
tl, dyn, feed, bal, checks = full(cfg)
sm = bal.p_rise_ms - tl.seat_ms
check("后切4+100%缸 气密裕量为负", sm < 0, "seat_margin=%.2fms" % sm)
lv = {c.name: c.level for c in checks}
check("气密判定为危险", lv["气密时序"] == LEVEL_DANGER)
cfg = make_cfg(rear_cut=4)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("后切4+70%缸 开孔缓冲放宽气密判定", lv["气密时序"] in (LEVEL_WARN, LEVEL_OK),
      "level=%s" % lv["气密时序"])

# ---- 6. 重切齿 → 压气不足 + 剩余齿数警告 ----
cfg = make_cfg(front_cut=3, rear_cut=3)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("前后各切3 → 压气不足警告", lv["压气匹配"] == LEVEL_WARN,
      "air_index=%.2f" % dyn.air_index)
check("剩余10齿 → 啮合齿数通过", lv["啮合齿数"] == LEVEL_OK)

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
check("50%缸+700mm 内管 → 管长匹配警告", lv["气量管长匹配"] == LEVEL_WARN,
      "有效推力 %.0fmm" % bal.useful_stroke_mm)

# 供蛋方式上限：普通波轮最高 30 发/秒
tl, dyn, feed, bal, checks = full(make_cfg())
lv = {c.name: c.level for c in checks}
check("默认射速 31.2 超普通波轮 30 上限", lv["供蛋速率"] == LEVEL_DANGER,
      "rof=%.1f" % tl.rof_rps)
check("供蛋方式上限 30 发/秒", feed.max_rps == 30.0)
check("最小供蛋间隔 = 1000/30", abs(feed.min_ms - 1000.0 / 30.0) < 1e-9)
tl, dyn, feed, bal, checks = full(make_cfg(feed_mode="高级波轮"))
lv = {c.name: c.level for c in checks}
check("高级波轮 50 上限 → 射速 31.2 通过", lv["供蛋速率"] == LEVEL_OK)
tl, dyn, feed, bal, checks = full(make_cfg(motor_rpm=40000, ratio="13:1"))
lv = {c.name: c.level for c in checks}
check("高射速 51.3 超普通波轮上限 → 危险", lv["供蛋速率"] == LEVEL_DANGER)
tl, dyn, feed, bal, checks = full(make_cfg(motor_rpm=40000, ratio="13:1",
                                           feed_mode="压力弹匣"))
lv = {c.name: c.level for c in checks}
check("压力弹匣 60 上限 → 射速 51.3 通过", lv["供蛋速率"] == LEVEL_OK)

# ---- 9. 方案枚举排序：通过者排前 ----
rows = enumerate_cut_schemes(make_cfg())
check("枚举 16 方案", len(rows) == 16)
check("排序后无通过者被排到危险者前",
      all(r.level != LEVEL_OK for r in rows) or rows[0].level == LEVEL_OK)

# ---- 9. 电机性能曲线 ----
from gearbox_sim import motor as gmotor

n, info, batt = gmotor.resolve_loaded_rpm(make_cfg(motor_model="超力有刷3W5"),
                                          make_cfg().device)
check("3W5 负载转速（弹簧反射扭矩法）", abs(n - 29607.0) < 40.0,
      "n=%.0f RPM" % n)
check("3W5 无电池 → 理想电源", batt is None)
check("3W5 信息完整", info.model == "超力有刷3W5" and not info.stall
      and 18.0 < info.torque_peak_mnm / info.stall_torque_mnm * 100 < 30.0)

tl, dyn, feed, bal, checks = full(make_cfg(motor_model="超力有刷3W5"))
lv = {c.name: c.level for c in checks}
# 变转速：拉动段（180°）按负载转速 n，空转段（180°）按空载转速（3W5 曲线 35500×负载系数）
T_piece = (0.5 * 60000.0 * 16 / n
           + 0.5 * 60000.0 * 16 / (tl.free_rpm))
check("曲线模式分段周期（拉动+空转）", abs(tl.period_ms - T_piece) < 0.2,
      "T=%.2fms（拉动 %.1f + 空转 %.1f）" % (tl.period_ms, 0.5 * 60000.0 * 16 / n,
                                            0.5 * 60000.0 * 16 / tl.free_rpm))
check("曲线模式空转转速 = 空载×负载系数", abs(tl.free_rpm - 35500.0 * 1.0) < 1.0,
      "free=%.0f RPM" % tl.free_rpm)
check("曲线模式 ROF ≈ 分段周期倒数", abs(tl.rof_rps - 1000.0 / T_piece) < 0.2,
      "rof=%.1f" % tl.rof_rps)
check("曲线模式含电机负载判定", "电机负载" in lv and lv["电机负载"] == LEVEL_OK)

tl, dyn, feed, bal, checks = full(make_cfg(motor_model="超力无刷4W8"))
check("4W8 负载转速 > 3W5（更强电机更快）", tl.loaded_rpm > 35000.0,
      "n=%.0f RPM rof=%.1f" % (tl.loaded_rpm, tl.rof_rps))

# 曲线模式下负载系数降额：只缩放负载转速（扭矩/堵转判定不变）
tl_a = full(make_cfg(motor_model="超力无刷4W8", load_factor=1.0))[0]
tl_b = full(make_cfg(motor_model="超力无刷4W8", load_factor=0.9))[0]
check("曲线模式负载系数降额（0.9 → 转速×0.9）",
      abs(tl_a.loaded_rpm * 0.9 - tl_b.loaded_rpm) < 1.0,
      "%.0f → %.0f RPM" % (tl_a.loaded_rpm, tl_b.loaded_rpm))

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
# 无跌落：4W8 + 4S 2200mAh 40C（持续 88A > 峰值需求 ~62A）
cfg = make_cfg(motor_model="超力无刷4W8", batt_cells=4,
               batt_capacity_mah=2200.0, batt_c_rate=40.0)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("4S 2200 40C → 供电充足无跌落", abs(tl.batt_sag - 1.0) < 1e-9
      and lv["电池放电"] == LEVEL_OK, "sag=%.3f" % tl.batt_sag)
check("4S 提升负载转速（14.8V > 11.1V）", tl.loaded_rpm > 48000.0,
      "n=%.0f" % tl.loaded_rpm)
check("续航估算合理", 1000.0 < tl.batt_shots < 50000.0,
      "shots=%.0f" % tl.batt_shots)

# 压降：4W8 + 3S 1200mAh 30C（持续 36A < 峰值需求 ~46A）→ 轻微跌落警告
cfg = make_cfg(motor_model="超力无刷4W8", batt_cells=3,
               batt_capacity_mah=1200.0, batt_c_rate=30.0)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("3S 1200 30C → 峰值超电池能力", tl.batt_sag < 1.0,
      "sag=%.3f" % tl.batt_sag)
check("轻微跌落 → 电池放电警告", lv["电池放电"] == LEVEL_WARN,
      "sag=%.3f" % tl.batt_sag)

# 严重压降：4W8 + 3S 1100mAh 30C（持续 33A）→ 跌落超 25% → 危险
cfg = make_cfg(motor_model="超力无刷4W8", batt_cells=3,
               batt_capacity_mah=1100.0, batt_c_rate=30.0)
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("4W8+3S1100 30C 电压严重跌落 → 危险", lv["电池放电"] == LEVEL_DANGER,
      "sag=%.3f" % tl.batt_sag)
check("压降降低负载转速", tl.loaded_rpm < 48000.0)

# 无电池 → 电池判定显示未配置
tl, dyn, feed, bal, checks = full(make_cfg(motor_model="超力无刷4W8"))
lv = {c.name: c.level for c in checks}
check("未配置电池 → 判定为未配置", lv["电池放电"] == LEVEL_OK
      and tl.batt_sag is None)

# ---- 9c. 改装件：延时器 / 切拉桥旗 ----
tl0, d0, f0, b0, c0 = full(make_cfg())
margin0 = b0.p_rise_ms - tl0.seat_ms

win = lambda t: t.window_end_ms - t.window_start_ms
tl, dyn, feed, bal, checks = full(make_cfg(install_delay=True))
check("延时器推迟推嘴回位完成", tl.seat_ms > tl0.seat_ms)
check("延时器加宽供蛋窗口", win(tl) > win(tl0),
      "window %.1f→%.1f" % (win(tl0), win(tl)))
check("延时器几何：1 孔 = 周期×1/10", abs(tl.delay_ms - tl0.period_ms / 10.0) < 1e-9,
      "delay=%.2f ms (T=%.1f ms)" % (tl.delay_ms, tl0.period_ms))
check("气密裕量随延时器等量减小",
      abs((b0.p_rise_ms - tl0.seat_ms) - (bal.p_rise_ms - tl.seat_ms) - tl.delay_ms) < 1e-6)
check("拉桥柱几何：推嘴开始缩回 = 第2齿接触天梯（拾取+1齿距）",
      abs(make_cfg().device.cam_retract_start_deg
          - make_cfg().device.sector_pitch_deg) < 1e-9)

tl, dyn, feed, bal, checks = full(make_cfg(cut_flag=True))
check("切拉桥旗提前推嘴回位完成", tl.seat_ms < tl0.seat_ms)
check("切拉桥旗不改变供蛋窗口", abs(win(tl) - win(tl0)) < 1e-9)
check("切拉桥旗增大气密裕量",
      (bal.p_rise_ms - tl.seat_ms) > (b0.p_rise_ms - tl0.seat_ms))

# 组合 + 下限保护：大延时 + 切旗，回位完成不得早于回位起始
cfg = make_cfg(install_delay=True, cut_flag=True)
cfg.device.delay_device_holes = 6.25   # 6.25 孔 ≈ 20ms @32ms 周期
cfg.device.flag_cut_advance_ms = 10.0
tl, dyn, feed, bal, checks = full(cfg)
check("回位完成不早于回位起始（下限保护）", tl.seat_ms >= tl.window_end_ms)

# 延时器使供蛋窗口判定改善（8.4ms → +10ms = 18.4ms ≥ 16.7ms 上限间隔）
cfg = make_cfg(install_delay=True)
cfg.device.delay_device_holes = 3.125  # 3.125 孔 ≈ 10ms @32ms 周期
tl, dyn, feed, bal, checks = full(cfg)
lv = {c.name: c.level for c in checks}
check("延时 3.125 孔（≈10ms）→ 供蛋窗口升为警告", lv["供蛋窗口"] == LEVEL_WARN,
      "window=%.1f" % win(tl))

# ---- 10. 配置解析与校验 ----
cfg = load_config(json.loads(json.dumps({
    "电机": {"标称转速RPM": 30000, "负载系数": 0.7, "型号": "超力有刷3W5"},
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
check("解析-负载系数", cfg.load_factor == 0.7)
check("解析-切齿", cfg.front_cut == 1 and cfg.rear_cut == 2)
check("解析-内管参数", cfg.barrel_length_mm == 300.0 and cfg.barrel_bore == "7.5"
      and cfg.ball_diameter == "7.3")
check("解析-电池", cfg.batt_cells == 3 and cfg.batt_capacity_mah == 1400.0
      and cfg.batt_c_rate == 30.0)
check("解析-改装", cfg.install_delay is True and cfg.cut_flag is False)
check("解析-最小供蛋覆盖", cfg.min_feed_ms == 25.0)
check("解析-器件覆盖", cfg.device.piston_full_stroke_mm == 60.0
      and cfg.device.gel_mass_g == 0.15)
check("解析-阈值覆盖", cfg.threshold.gear_clash_margin_ms == 3.0
      and cfg.threshold.velocity_warn_ms == 90.0)

# 负载系数默认按电机类型：无刷 0.9 / 有刷 0.8 / 未选型号（固定转速）0.8
cfg = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000, "型号": "超力无刷4W8"}})))
check("解析-负载默认（无刷 0.9）", cfg.load_factor == 0.9)
cfg = load_config(json.loads(json.dumps(
    {"电机": {"标称转速RPM": 30000, "型号": "超力有刷3W5"}})))
check("解析-负载默认（有刷 0.8）", cfg.load_factor == 0.8)
cfg = load_config(json.loads(json.dumps({"电机": {"标称转速RPM": 30000}})))
check("解析-负载默认（固定转速 0.8）", cfg.load_factor == 0.8)

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
check("推嘴缩回起点时刻 = 11.25°×空转ms/°（非硬编码 0）",
      abs(tl.retract_start_ms - 11.25 * tl.period_ms / 360.0) < 1e-9,
      "t=%.2fms" % tl.retract_start_ms)
tl, dyn, feed, bal, checks = full(make_cfg())
ev = [e for e in events_list(cfg, cfg.device, tl, dyn, feed, bal)
      if e["desc"].startswith("推嘴开始缩回")][0]
check("事件表推嘴缩回时刻与角度一致（晚于拾取）",
      abs(ev["t"] - tl.retract_start_ms) < 1e-9 and ev["t"] > tl.pickup_ms,
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
check("切齿后拉满长度 = 装配长 − 实际行程（随切齿派生）",
      abs((cfg.device.spring_installed_length_mm - tl.stroke_mm) - 57.125) < 1e-9,
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

print()
if FAILS:
    print("失败 %d 项: %s" % (len(FAILS), FAILS))
    sys.exit(1)
print("全部测试通过。")
