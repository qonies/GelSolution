# -*- coding: utf-8 -*-
"""器件参数表：通用二号波默认值（实测与社区经验基准）。

⚠ 参数置信度标注：【估算】= 未经实测的社区经验值，结论精度依赖校准；
  已确认参数（扇齿 16 齿，每齿 12°，满拉弧 = 16×12 = 192°；✅v3.9 用户实测
  更正：15 个齿距占圆周 180° → 每齿 12°，原「16×11.25=180°」口径有误）
  来自用户确认。
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
import math

# 原装天梯咬合齿数基准：13.5 齿（半齿磨低仍计整齿）→ 计 14 齿，对应
# 活塞满行程 60.5mm 的实测值；拉程按「剩余咬合齿数 / 14」缩放并封顶满行程
TAPPET_STOCK_BITE = 14


@dataclass
class DeviceParams:
    """二号波器件默认参数。"""

    # ---- 扇齿与切齿 ----
    sector_full_teeth: int = 16        # 扇齿满啮合齿数（未切齿）
    sector_pitch_deg: float = 12.0     # 每齿对应的扇齿转角（✅v3.9 用户实测更正：
                                       # 15 个齿距占圆周 180° → 每齿 12°，原 11.25° 有误；
                                       # 满拉弧 = 16 齿 × 12° = 192°）
    tappet_teeth: float = 13.5         # 天梯（活塞齿条）齿数（✅v0.6.9 默认=原装 13.5；
                                       # 市售 11.5 齿等半齿规格可）：半齿=第二齿磨低防
                                       # 咬合，仍按整齿咬合 → 计整齿（13.5→14、11.5→12）；
                                       # 蛋糕齿实际咬合齿数 = min(扇齿满啮合齿数, 计整齿数)。
                                       # 原装咬合 14 齿对应活塞满行程实测 60.5mm → 实际
                                       # 拉程 = 满行程 × min(剩余咬合, 14) / 14（超过按
                                       # 满行程封顶）——齿数越少拉程越短，储能/初速下降、
                                       # 电机负载减轻，撞击/回位稳定经联动积分随之变化

    # ---- 凸轮槽 / 拉桥旗（推嘴时序，齿轮角）----
    # 几何口径（✅v4.3 用户更正 + 参数化）：拉桥柱位置 = 拉桥柱在哪一颗齿的下方，
    # 第 1 颗齿下方 = 0（= 拾取角 0° 即开始缩回）、+1 = 下一颗齿（推迟 1 齿距），
    # 可小数（0.5 = 两齿之间）与负数（−2 = 拾取前 2 齿距，缩回早于活塞拾取）；
    # 缩回起点角默认 = 位置 × 每齿角（params.load_config 未显式覆盖
    # 推嘴缩回起点角 时派生，显式覆盖该角则优先）。凸轮槽事件固定于扇齿，
    # 前切时拾取角后移、缩回起点不变（旧「第 2 颗齿下方 / 拾取+1 齿距 12°」
    # 锚点自 v4.3 废弃）。
    cam_pillar_tooth: float = 0.0         # 【✅v4.3 用户新增】拉桥柱位置（齿序，
                                          # 第 1 齿下方 = 0；默认 0 = 拾取即缩回）
    cam_retract_start_deg: float = 0.0    # 【派生】推嘴开始缩回 = 拉桥柱位置 × 每齿角
                                          # （默认 0×12° = 0°；显式覆盖 推嘴缩回起点角 优先）
    cam_retract_end_deg: float = 50.0     # 【估算】推嘴完全缩回（供蛋窗口开始）
    cam_return_start_deg: float = 144.4   # 【估算】推嘴开始回位（供蛋窗口结束）
    cam_return_end_deg: float = 164.4     # 【估算】推嘴回位完成（气密就位）

    # ---- 活塞与弹簧（通用二号波基准）----
    # 弹簧实测：自由长 160~175mm，安装后预压至 100~105mm（预压 55~75mm）；
    #           满压长度按「装配长 − 行程」推算（✅用户更正：原独立实测 40~45mm
    #           可能有误，应以齿轮实际拉过天梯的行程派生）；
    #           M85~M90 刚度 ≈0.55~0.65 N/mm，M100~M110 ≈0.70~0.85 N/mm
    piston_full_stroke_mm: float = 60.5  # 【实测】活塞最大压缩行程 ≈60.5mm
    spring_free_length_mm: float = 165.0    # 【实测区间中值·展示项】弹簧自由长度（参与预压派生）
    spring_installed_length_mm: float = 102.5  # 【实测区间中值·展示项】安装后长度（参与预压派生）
    spring_compressed_length_mm: float = 42.0  # 【推算·仅展示】满行程时满压长度 = 装配长 − 满行程（102.5 − 60.5）；
                                               # 不参与计算：实际拉满长度随切齿行程派生 = 装配长 − 实际行程（report.py）
    piston_mass_g: float = 20.0           # 【估算】活塞组件质量（借用AEG值，建议电子秤实测）
    piston_head_restitution: float = 0.5  # 【估算】撞击回弹系数：天梯撞缸头后的反弹速度比
                                          # （回位稳定模型 v_r = 系数 × 撞击速度；回弹越猛复位越久）
    spring_preload_mm: float = 62.5       # 【推算】弹簧预压量 = 自由长 − 装配长（原 5mm 严重偏低已修正）；
                                          # JSON 未显式覆盖时由解析层按当前 自由/装配 长度自动派生（v0.6.7）
    drive_efficiency: float = 0.85        # 弹簧储能 → 活塞动能的效率（校准旋钮）

    # ---- 弹道（气缸/内管/水弹，通用二号波实测基准）----
    cylinder_bore_mm: float = 23.8        # 【实测】气缸内径 23.8mm（大缸）
    cylinder_length_mm: float = 72.5      # 【实测】气缸长度 72.5mm
    port_retention: float = 0.6           # 【估算】开孔段气压保留系数：50%~100% 缸 =
                                          # 在气缸相应位置对称开 4 条横槽漏气；但活塞
                                          # 经过极快，横槽来不及漏掉全部已压缩气体
                                          # （有漏但非全漏）→ 开孔段按等效多变压缩
                                          # γ×保留系数建模：0=全漏（直通大气，旧模型）、
                                          # 1=完全不漏；建议配合「气动效率」用实测初速反标
    dead_volume_cm3: float = 1.0          # 【估算】余隙容积（缸头+hop+管尾死容积）
    gel_mass_g: float = 0.20              # 【实测】水弹质量（泡发后 ≈0.2g，以 7.2mm 弹为基准，其他直径按体积缩放）
    aero_efficiency: float = 0.243        # 气动效率（✅v0.7.0 随开孔段保留模型重校准：
                                          # 按实测弹簧 + 0.2g 弹 + M90/70缸 ≈71m/s 锚点）
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

    # ---- 气缸类型 → 开孔位置（以满行程标定，孔位固定于缸体）----
    # 50% 缸 = 满行程时活塞走完 50% 行程后气孔才被盖住（密封段 = 50% 满行程）。
    # 水桶效应（✅用户更正）：切齿后孔位不变，有效密封段 = min(系数×满行程,
    # 实际行程)——行程仍覆盖开孔段时密封容积不随切齿缩减；行程短于开孔段
    # 时全程密封（无自由泄压段）。
    cylinder_factor: dict = field(default_factory=lambda: {
        "50%": 0.50, "60%": 0.60, "70%": 0.70, "80%": 0.80, "100%": 1.00,
    })

    # ---- 供蛋方式 → 最高供弹速率 (发/秒)；最小供蛋间隔 = 1000 ÷ 速率 ----
    feed_max_rps: dict = field(default_factory=lambda: {
        "普通波轮": 30.0,   # 普通波轮：最高每秒 30 发
        "高级波轮": 60.0,   # 高级波轮：最高每秒 60 发（✅用户更正 v0.6.5，原 50）
        "压力弹匣": 80.0,   # 压力弹匣：最高每秒 80 发（✅用户更正 v0.6.5，原 60）
    })

    # ---- 改装件时序量（勾选对应改装项后生效）----
    # 拉桥旗槽形（✅ 用户说明）：旗面与拉桥柱接触的槽是一条 S 形曲线——
    #   拉开开始阶段幅度很大（推嘴快速后缩），后期是缓冲段（缓慢恢复推嘴）。
    # 延时器（✅v4.0 角度参数模型）：以 提前度数 / 延时度数 两个转角直接描述——
    #   提前 → 推嘴缩回与供蛋窗口开始提前（✅v4.3 起不再钳制 0°：拉桥柱位置
    #   可为负，缩回允许早于拾取；供蛋窗口开始仍钳制 ≥ 0°）；延时 → 回位起始/
    #   完成推迟（供蛋窗口加宽、气密就位更晚）。
    # 切拉桥旗 = 切掉 S 形缓冲尾 → 推嘴更早、更快恢复闭合（水弹进膛后快速气密），
    #   代价是拉桥簧拉力需更大、恢复更快，否则实际气密时刻会晚于几何提前量。
    # 拉桥柱孔数（✅ 用户确认）：一圈 10 孔（每孔 36°），硬件基准；旧「延时器
    #   占孔数」占孔几何模型（柱子加粗占 2 孔 → 推迟 1 孔 36° = 周期×1/10）
    #   已弃用，默认延时度数 36° = 原「1 孔」口径，旧键接受但忽略。
    cam_hole_count: int = 10              # 【✅用户确认】拉桥柱安装孔数（一圈；
                                          # 结论建议附「每孔 ≈ms」供选择延时度数参考）
    delay_device_holes: float = 1.0       # 【✅v4.0 已弃用】占孔几何模型改为直接的
                                          # 「延时器提前度数 / 延时器延时度数」角度参数
                                          # （36°=1 孔），此键被接受但忽略、不参与计算
    delay_advance_deg: float = 0.0        # 【✅v4.0 用户新增】延时器提前度数：比原拉桥柱
                                          # 更早接触拉桥旗的扇齿转角 → 推嘴开始缩回与
                                          # 完全缩回（供蛋窗口开始）提前该角度
                                          # （✅v4.3 起不钳制，可推至拾取前）
    delay_hold_deg: float = 36.0          # 【✅v4.0 用户新增】延时器延时度数：比原拉桥柱
                                          # 更晚离开拉桥旗的扇齿转角 → 推嘴回位起始/完成
                                          # 推迟该角度（供蛋窗口加宽、气密就位更晚）；
                                          # 默认 36° = 原「1 孔」口径，装延时器默认行为
                                          # 与旧占孔模型等价
    flag_cut_advance_ms: float = 3.0      # 【估算】切拉桥旗使推嘴回位完成提前的时间

    def tappet_bite(self) -> int:
        """天梯有效咬合齿数：半齿（磨低齿）仍计整齿 → 向上取整，
        且不超过扇齿满啮合齿数（13.5 → 14，11.5 → 12）。"""
        return min(self.sector_full_teeth, math.ceil(self.tappet_teeth))

    def gel_mass_for(self, ball_diameter_mm: float) -> float:
        """按直径体积缩放水弹质量（gel_mass_g 为 7.2mm 弹基准）。"""
        return self.gel_mass_g * (ball_diameter_mm / 7.2) ** 3


# 器件参数覆盖：JSON 中文键 → dataclass 字段
DEVICE_KEY_MAP = {
    "扇齿满啮合齿数": "sector_full_teeth",
    "每齿扇齿角": "sector_pitch_deg",
    "天梯齿数": "tappet_teeth",
    "拉桥柱位置": "cam_pillar_tooth",
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
    "开孔段保留系数": "port_retention",
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
    "延时器提前度数": "delay_advance_deg",
    "延时器延时度数": "delay_hold_deg",
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


# 表类型覆盖参数（JSON 键 → 值为「键→数字」映射，整体替换非合并）
OVERRIDE_DICT_KEYS = ("弹簧刚度表", "气缸系数表", "最大供蛋速率表")

# 数值型覆盖参数的合理范围（dataclass 字段名 → (下限, 上限)，闭区间）；
# 未列出的数值参数只做「必须是数字」类型校验。
# 天梯齿数 / 开孔段保留系数 / 弹簧预压mm 不在此表——解析层（params.py）
# 有更详细的专项校验与中文提示，避免被通用范围提示抢先拦截。
OVERRIDE_RANGE = {
    "sector_full_teeth": (1, 64),
    "sector_pitch_deg": (0.1, 90.0),
    "cam_pillar_tooth": (-16.0, 30.0),
    "cam_retract_start_deg": (0.0, 360.0),
    "cam_retract_end_deg": (0.0, 360.0),
    "cam_return_start_deg": (0.0, 360.0),
    "cam_return_end_deg": (0.0, 360.0),
    "piston_full_stroke_mm": (1.0, 500.0),
    "piston_mass_g": (0.1, 1000.0),
    "spring_free_length_mm": (1.0, 2000.0),
    "spring_installed_length_mm": (1.0, 2000.0),
    "spring_compressed_length_mm": (0.0, 2000.0),
    "piston_head_restitution": (0.0, 1.0),
    "drive_efficiency": (0.01, 1.0),
    "cylinder_bore_mm": (1.0, 200.0),
    "cylinder_length_mm": (1.0, 1000.0),
    "dead_volume_cm3": (0.0, 100.0),
    "gel_mass_g": (0.01, 10.0),
    "aero_efficiency": (0.01, 1.0),
    "adiabatic_index": (1.0, 3.0),
    "leak_coeff": (0.0, 100.0),
    "atm_kpa": (10.0, 200.0),
    "cam_hole_count": (1, 100),
    "delay_device_holes": (0.0, 100.0),
    "delay_advance_deg": (0.0, 180.0),
    "delay_hold_deg": (0.0, 180.0),
    "flag_cut_advance_ms": (0.0, 1000.0),
    "gear_clash_margin_ms": (0.0, 1000.0),
    "air_seal_margin_ms": (0.0, 1000.0),
    "low_air_index": (0.0, 1.0),
    "min_remain_teeth": (0, 64),
    "velocity_warn_ms": (1.0, 1000.0),
    "barrel_match_ratio": (0.0, 1.0),
}


def apply_overrides(obj, overrides: dict, key_map: dict, errors: list):
    """把 JSON 覆盖字典应用到 dataclass 实例（中文键，含类型/范围校验）。

    ✅v0.8.2：此前覆盖值不做任何校验——非数字（如 "abc"）会在后续计算层
    抛 TypeError、表键缺失会 KeyError、除零参数会 ZeroDivisionError，
    均为英文堆栈。现在统一在此拦截并给出中文错误（与主配置键校验同风格）。
    """
    for key, value in (overrides or {}).items():
        attr = key_map.get(key)
        if attr is None:
            errors.append("未知的覆盖参数: %s" % key)
            continue
        if key in OVERRIDE_DICT_KEYS:
            # 表类型：必须是 JSON 对象，且所有值为数字（表为整体替换）
            if not isinstance(value, dict):
                errors.append("覆盖参数 %s 必须是 JSON 对象（键→数字 的表），"
                              "实际为: %r" % (key, value))
                continue
            bad = next((v for v in value.values()
                        if isinstance(v, bool) or not isinstance(v, (int, float))),
                       None)
            if bad is not None:
                errors.append("覆盖参数 %s 表中的值必须是数字，实际含: %r"
                              % (key, bad))
                continue
        else:
            # 数值类型：拒绝布尔与非数字
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                errors.append("覆盖参数 %s 必须是数字，实际为: %r" % (key, value))
                continue
            rng = OVERRIDE_RANGE.get(attr)
            if rng is not None and not rng[0] <= value <= rng[1]:
                errors.append("覆盖参数 %s 需在 %g~%g 之间，实际为: %r"
                              % (key, rng[0], rng[1], value))
                continue
        setattr(obj, attr, value)
