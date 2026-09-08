# -*- coding: utf-8 -*-
"""器件参数表：通用二号波默认值（以 LDX 1.0 规格为基准）。

⚠ 参数置信度标注：【估算】= 未经实测的社区经验值，结论精度依赖校准；
  已确认参数（扇齿 16 齿 × 11.25° = 满齿弧 180°）来自用户确认。
  所有参数均可在配置「器件参数覆盖」或网页界面中覆盖。
  建议校准顺序：实测初速反标「气动效率」 → 实测弹簧改「弹簧刚度表」
  → 卡尺量「气缸内径」「活塞满行程」→ 实测「弹簧预压」「活塞质量」。

角度基准约定：
  扇齿齿轮角 0° = 无切齿时的活塞拾取角（活塞齿条与扇齿齿面开始啮合）。
  凸轮槽（拉桥旗槽）事件角均以同一齿轮角基准表示，固定在扇齿上，
  因此切齿后活塞拾取/释放角相对凸轮事件移动——这正是切齿影响气密
  时序与回位裕量的机理。
"""
from dataclasses import dataclass, field


@dataclass
class DeviceParams:
    """二号波器件默认参数。"""

    # ---- 扇齿与切齿 ----
    sector_full_teeth: int = 16        # 扇齿满啮合齿数（未切齿）
    sector_pitch_deg: float = 11.25    # 每齿对应的扇齿转角（满弧 180°，16×11.25）

    # ---- 凸轮槽 / 拉桥旗（推嘴时序，齿轮角）----
    # 几何锚定（✅ 用户确认）：拉桥柱位于第 2 颗齿下方——当扇齿第 2 颗齿接触
    # 活塞天梯（= 无切拾取角 + 1 齿距）时，拉桥柱开始拉动拉桥旗、推嘴后缩；
    # 因此前切时拾取角后移，拉桥柱相对拾取更早开始拉推嘴（凸轮事件仍固定于扇齿）。
    cam_retract_start_deg: float = 11.25  # 【✅几何确认】推嘴开始缩回 = 拾取角 + 1 齿距
    cam_retract_end_deg: float = 50.0     # 【估算】推嘴完全缩回（供蛋窗口开始）
    cam_return_start_deg: float = 144.4   # 【估算】推嘴开始回位（供蛋窗口结束）
    cam_return_end_deg: float = 164.4     # 【估算】推嘴回位完成（气密就位）

    # ---- 活塞与弹簧（LDX 1.0 二号波基准）----
    # 弹簧实测：自由长 160~175mm，安装后预压至 100~105mm（预压 55~75mm）；
    #           满压长度按「装配长 − 行程」推算（✅用户更正：原独立实测 40~45mm
    #           可能有误，应以齿轮实际拉过天梯的行程派生）；
    #           M85~M90 刚度 ≈0.55~0.65 N/mm，M100~M110 ≈0.70~0.85 N/mm
    piston_full_stroke_mm: float = 60.5  # 【实测】活塞最大压缩行程 ≈60.5mm
    spring_free_length_mm: float = 165.0    # 【实测区间中值】弹簧自由长度
    spring_installed_length_mm: float = 102.5  # 【实测区间中值】安装后长度（预压 = 自由 − 装配 ≈ 62.5mm）
    spring_compressed_length_mm: float = 42.0  # 【推算】满行程时满压长度 = 装配长 − 满行程（102.5 − 60.5）；
                                               # 实际拉满长度随切齿行程派生 = 装配长 − 实际行程
    piston_mass_g: float = 20.0           # 【估算】活塞组件质量（借用AEG值，建议电子秤实测）
    piston_head_restitution: float = 0.5  # 【估算】撞击回弹系数：天梯撞缸头后的反弹速度比
                                          # （回位稳定模型 v_r = 系数 × 撞击速度；回弹越猛复位越久）
    spring_preload_mm: float = 62.5       # 【推算】弹簧预压量 = 自由长 − 装配长（原 5mm 严重偏低已修正）
    drive_efficiency: float = 0.85        # 弹簧储能 → 活塞动能的效率（校准旋钮）

    # ---- 弹道（气缸/内管/水弹，LDX 1.0 二号波实测基准）----
    cylinder_bore_mm: float = 23.8        # 【实测】气缸内径 23.8mm（大缸）
    cylinder_length_mm: float = 72.5      # 【实测】气缸长度 72.5mm
    dead_volume_cm3: float = 1.0          # 【估算】余隙容积（缸头+hop+管尾死容积）
    gel_mass_g: float = 0.20              # 【实测】水弹质量（泡发后 ≈0.2g，以 7.2mm 弹为基准，其他直径按体积缩放）
    aero_efficiency: float = 0.21         # 气动效率（按实测弹簧 + 0.2g 弹 + M90/70缸 ≈71m/s 校准）
    adiabatic_index: float = 1.4          # 空气绝热指数
    leak_coeff: float = 1.5               # 管径-弹径间隙泄气损失系数
    atm_kpa: float = 101.3                # 大气压 kPa

    # ---- 弹簧 M 值 → 刚度 (N/mm) ----
    # ✅ 按用户提供的二号波实测区间标定：M85~M90 ≈ 0.55~0.65，M100~M110 ≈ 0.70~0.85；
    #    M75/M80 为区间外推值。
    spring_stiffness: dict = field(default_factory=lambda: {
        "M75": 0.45, "M80": 0.50, "M85": 0.55, "M90": 0.62,
        "M95": 0.68, "M100": 0.72, "M110": 0.85,
    })

    # ---- 气缸类型 → 密封行程比例（开孔位置：活塞头盖过气孔后才开始压缩）----
    # 50% 缸 = 活塞走完 50% 行程后气孔才被盖住，实际压缩容积为满行程的 50%
    cylinder_factor: dict = field(default_factory=lambda: {
        "50%": 0.50, "60%": 0.60, "70%": 0.70, "80%": 0.80, "100%": 1.00,
    })

    # ---- 供蛋方式 → 最高供弹速率 (发/秒)；最小供蛋间隔 = 1000 ÷ 速率 ----
    feed_max_rps: dict = field(default_factory=lambda: {
        "普通波轮": 30.0,   # 普通波轮：最高每秒 30 发
        "高级波轮": 50.0,   # 高级波轮：最高每秒 50 发
        "压力弹匣": 60.0,   # 压力弹匣：最高每秒 60 发
    })

    # ---- 改装件时序量（勾选对应改装项后生效）----
    # 拉桥旗槽形（✅ 用户说明）：旗面与拉桥柱接触的槽是一条 S 形曲线——
    #   拉开开始阶段幅度很大（推嘴快速后缩），后期是缓冲段（缓慢恢复推嘴）。
    # 延时器 = 维持推嘴最大开度的时间（几何模型如下）；
    # 切拉桥旗 = 切掉 S 形缓冲尾 → 推嘴更早、更快恢复闭合（水弹进膛后快速气密），
    #   代价是拉桥簧拉力需更大、恢复更快，否则实际气密时刻会晚于几何提前量。
    # 延时器几何（✅ 用户确认）：扇齿一圈有 10 个拉桥柱安装孔（每孔 36°）；
    # 装延时器相当于拉桥柱加粗一倍、占用 2 个孔 → 推嘴保持拉开的时长多出 1 孔（36°），
    # 即回位起始/完成推迟 = 孔数 ÷ 总孔数 × 周期（随射速自动缩放，非固定毫秒）。
    cam_hole_count: int = 10              # 【✅用户确认】拉桥柱安装孔数（一圈）
    delay_device_holes: float = 1.0       # 【✅用户确认】延时器额外占用的孔数（默认 1 孔 = 36°）
    flag_cut_advance_ms: float = 3.0      # 【估算】切拉桥旗使推嘴回位完成提前的时间

    def gel_mass_for(self, ball_diameter_mm: float) -> float:
        """按直径体积缩放水弹质量（gel_mass_g 为 7.2mm 弹基准）。"""
        return self.gel_mass_g * (ball_diameter_mm / 7.2) ** 3


# 器件参数覆盖：JSON 中文键 → dataclass 字段
DEVICE_KEY_MAP = {
    "扇齿满啮合齿数": "sector_full_teeth",
    "每齿扇齿角": "sector_pitch_deg",
    "推嘴缩回起点角": "cam_retract_start_deg",
    "推嘴完全缩回角": "cam_retract_end_deg",
    "推嘴回位起始角": "cam_return_start_deg",
    "推嘴回位完成角": "cam_return_end_deg",
    "活塞满行程mm": "piston_full_stroke_mm",
    "活塞质量g": "piston_mass_g",
    "弹簧预压mm": "spring_preload_mm",
    "弹簧自由长度mm": "spring_free_length_mm",
    "弹簧装配长度mm": "spring_installed_length_mm",
    "弹簧满压长度mm": "spring_compressed_length_mm",
    "撞击回弹系数": "piston_head_restitution",
    "传动效率": "drive_efficiency",
    "气缸内径mm": "cylinder_bore_mm",
    "气缸长度mm": "cylinder_length_mm",
    "气缸余隙容积cm3": "dead_volume_cm3",
    "水弹质量g": "gel_mass_g",
    "气动效率": "aero_efficiency",
    "绝热指数": "adiabatic_index",
    "泄气损失系数": "leak_coeff",
    "大气压kPa": "atm_kpa",
    "弹簧刚度表": "spring_stiffness",
    "气缸系数表": "cylinder_factor",
    "最大供蛋速率表": "feed_max_rps",
    "拉桥柱孔数": "cam_hole_count",
    "延时器占孔数": "delay_device_holes",
    "切拉桥旗提前ms": "flag_cut_advance_ms",
}


@dataclass
class Thresholds:
    """判定阈值（可覆盖）。"""

    gear_clash_margin_ms: float = 2.0   # 打齿判定：回位裕量安全阈值
    air_seal_margin_ms: float = 1.0     # 气密判定：推嘴回位完成到活塞释放的最小裕量
    low_air_index: float = 0.60         # 压气指数低于此值 → 压气不足警告
    min_remain_teeth: int = 8           # 切齿后最少保留啮合齿数
    velocity_warn_ms: float = 80.0      # 水弹初速警告阈值 (m/s)
    barrel_match_ratio: float = 0.5     # 有效推力行程/内管长度 低于此值 → 内管过长警告


THRESHOLD_KEY_MAP = {
    "打齿安全裕量ms": "gear_clash_margin_ms",
    "气密最小裕量ms": "air_seal_margin_ms",
    "压气不足阈值": "low_air_index",
    "最少保留啮合齿数": "min_remain_teeth",
    "初速警告阈值m/s": "velocity_warn_ms",
    "管长匹配比例": "barrel_match_ratio",
}


def apply_overrides(obj, overrides: dict, key_map: dict, errors: list):
    """把 JSON 覆盖字典应用到 dataclass 实例（中文键）。"""
    for key, value in (overrides or {}).items():
        attr = key_map.get(key)
        if attr is None:
            errors.append("未知的覆盖参数: %s" % key)
            continue
        setattr(obj, attr, value)
