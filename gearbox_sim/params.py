# -*- coding: utf-8 -*-
"""模拟配置定义与 JSON 配置解析（含中文校验报错）。"""
from dataclasses import dataclass, field
from typing import Optional

from .components import (DeviceParams, Thresholds, apply_overrides,
                         DEVICE_KEY_MAP, THRESHOLD_KEY_MAP)
from .motor import MOTOR_MODELS, MOTOR_CURVES

VALID_RATIOS = {"13:1": 13.0, "16:1": 16.0, "18:1": 18.0}
VALID_SPRINGS = ["M75", "M80", "M85", "M90", "M95", "M100", "M110"]
VALID_CYLINDERS = ["50%", "60%", "70%", "80%", "100%"]
VALID_FEED_MODES = ["普通波轮", "高级波轮", "压力弹匣"]
# 旧配置兼容：历史名称 → 新名称
FEED_ALIAS = {"波轮": "普通波轮", "压力": "压力弹匣"}
VALID_BORES = ["7.3", "7.5"]      # 内管管径 (mm)
VALID_BALLS = ["7.2", "7.3"]      # 水弹直径 (mm)
MAX_CUT = 6  # 单侧最大切齿数（防止无意义输入）


class ConfigError(Exception):
    """配置错误（中文信息，直接展示给用户）。"""


@dataclass
class SimConfig:
    """一次模拟的全部输入。"""

    motor_rpm: float                      # 电机标称转速（标签值）
    load_factor: float = 1.0              # 负载系数（1.0=按标称转速；用于后期接入电机曲线）
    motor_model: Optional[str] = None     # 电机型号（选型后按性能曲线计算负载转速）
    batt_cells: Optional[int] = None      # 电池电芯数（2S=2, 3S=3, 4S=4；曲线模式生效）
    batt_capacity_mah: Optional[float] = None  # 电池容量 mAh
    batt_c_rate: Optional[float] = None   # 放电倍率 C
    install_delay: bool = False           # 是否安装延时器（推迟推嘴回位，加宽供蛋窗口）
    cut_flag: bool = False                # 是否切拉桥旗（推嘴提前回位）
    ratio: str = "13:1"                   # 齿轮比（默认 13:1）
    front_cut: int = 0                    # 前切齿数（推迟活塞拾取）
    rear_cut: int = 0                     # 后切齿数（提前释放活塞）
    cylinder: str = "50%"                 # 气缸类型（默认 50%）
    spring: str = "M90"                   # 弹簧硬度
    barrel_length_mm: float = 210.0       # 内管长度（默认 210mm）
    barrel_bore: str = "7.5"              # 内管管径（与 default.json / 网页默认统一）
    ball_diameter: str = "7.2"            # 水弹直径
    feed_mode: str = "普通波轮"           # 供蛋方式：普通波轮 / 高级波轮 / 压力弹匣
    feed_min_ms: Optional[float] = None   # 最小供蛋时间（None=按器件表默认）
    device: DeviceParams = field(default_factory=DeviceParams)
    threshold: Thresholds = field(default_factory=Thresholds)

    @property
    def ratio_value(self) -> float:
        return VALID_RATIOS[self.ratio]

    @property
    def min_feed_ms(self) -> float:
        """最小供蛋间隔 = 1000 ÷ 该供蛋方式的最高供弹速率。"""
        if self.feed_min_ms is not None:
            return self.feed_min_ms
        return 1000.0 / self.feed_max_rps

    @property
    def feed_max_rps(self) -> float:
        """当前供蛋方式的最高供弹速率 (发/秒)。"""
        return self.device.feed_max_rps[self.feed_mode]


def _require_number(value, name):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ConfigError("%s 必须是数字，实际为: %r" % (name, value))
    return float(value)


def load_config(data: dict) -> SimConfig:
    """从解析后的 JSON 字典构建 SimConfig，错误信息全部中文。"""
    if not isinstance(data, dict):
        raise ConfigError("配置文件根节点必须是 JSON 对象")

    motor = data.get("电机") or {}
    motor_model = motor.get("型号", None)
    if motor_model is not None and motor_model not in MOTOR_MODELS:
        raise ConfigError("电机.型号 仅支持 %s，实际为: %r"
                          % ("/".join(MOTOR_MODELS), motor_model))

    # 标称转速：固定转速模式（未选型号）必填且 > 0；曲线模式下不参与计算，
    # 可省略或填 0（缺省按所选电机曲线的空载转速填入，仅作记录展示）
    rpm_raw = motor.get("标称转速RPM", None)
    if rpm_raw is None:
        if motor_model:
            rpm = MOTOR_CURVES[motor_model]["no_load_rpm"]
        else:
            raise ConfigError("未选 电机.型号 时必须提供 电机.标称转速RPM（固定转速模式，> 0）")
    else:
        rpm = _require_number(rpm_raw, "电机.标称转速RPM")
        if rpm <= 0:
            if motor_model:
                rpm = MOTOR_CURVES[motor_model]["no_load_rpm"]
            else:
                raise ConfigError("电机.标称转速RPM 必须大于 0")

    # 负载系数默认按电机类型：无刷 0.9 / 有刷 0.8 / 未选型号（固定转速）0.8
    default_lf = 0.9 if (motor_model
                         and MOTOR_CURVES[motor_model]["type"] == "无刷") else 0.8
    load_factor = _require_number(motor.get("负载系数", default_lf), "电机.负载系数")
    if load_factor <= 0:
        raise ConfigError("电机.负载系数 必须大于 0")

    # 电池（可选；须在电机曲线模式下使用；缺省项按 3S / 1400mAh / 30C 补齐）
    batt = data.get("电池") or {}
    batt_cells = batt.get("电芯数", None)
    batt_cap = batt.get("容量mAh", None)
    batt_c = batt.get("放电倍率", None)
    has_batt = batt_cells is not None or batt_cap is not None or batt_c is not None
    if has_batt:
        if not motor.get("型号"):
            raise ConfigError("配置了「电池」但未选择 电机.型号：电池仅在电机曲线模式下参与计算")
        batt_cells = 3 if batt_cells is None else batt_cells
        batt_cap = 1400.0 if batt_cap is None else batt_cap
        batt_c = 30.0 if batt_c is None else batt_c
        if not isinstance(batt_cells, int) or not 2 <= batt_cells <= 8:
            raise ConfigError("电池.电芯数 必须是 2~8 的整数（S 数），实际为: %r" % batt_cells)
        batt_cap = _require_number(batt_cap, "电池.容量mAh")
        if not 100 <= batt_cap <= 20000:
            raise ConfigError("电池.容量mAh 需在 100~20000 之间，实际为: %r" % batt_cap)
        batt_c = _require_number(batt_c, "电池.放电倍率")
        if not 1 <= batt_c <= 200:
            raise ConfigError("电池.放电倍率 需在 1~200 之间，实际为: %r" % batt_c)
    else:
        batt_cells = batt_cap = batt_c = None

    # 改装件（可选布尔开关）
    mods = data.get("改装") or {}
    install_delay = mods.get("安装延时器", False)
    cut_flag = mods.get("切拉桥旗", False)
    for name, v in (("安装延时器", install_delay), ("切拉桥旗", cut_flag)):
        if not isinstance(v, bool):
            raise ConfigError("改装.%s 必须是 true / false，实际为: %r" % (name, v))

    ratio = data.get("齿轮比", "13:1")
    if ratio not in VALID_RATIOS:
        raise ConfigError("齿轮比 仅支持 %s，实际为: %r" % ("/".join(VALID_RATIOS), ratio))

    cut = data.get("切齿") or {}
    for name in ("前切齿数", "后切齿数"):
        v = cut.get(name, 0)
        if isinstance(v, bool) or not isinstance(v, int) or v < 0 or v > MAX_CUT:
            raise ConfigError("切齿.%s 必须是 0~%d 的整数，实际为: %r" % (name, MAX_CUT, v))

    cylinder = data.get("气缸类型", "50%")
    if cylinder not in VALID_CYLINDERS:
        raise ConfigError("气缸类型 仅支持 %s，实际为: %r" % ("/".join(VALID_CYLINDERS), cylinder))

    spring = data.get("弹簧", "M90")
    if spring not in VALID_SPRINGS:
        raise ConfigError("弹簧 仅支持 %s，实际为: %r" % ("/".join(VALID_SPRINGS), spring))

    barrel_length = data.get("内管长度mm", 210.0)
    barrel_length = _require_number(barrel_length, "内管长度mm")
    if not 20.0 <= barrel_length <= 800.0:
        raise ConfigError("内管长度mm 需在 20~800 之间，实际为: %r" % barrel_length)

    barrel_bore = data.get("内管管径", "7.5")
    if str(barrel_bore) not in VALID_BORES:
        raise ConfigError("内管管径 仅支持 %s，实际为: %r" % ("/".join(VALID_BORES), barrel_bore))

    ball_diameter = data.get("水弹直径", "7.2")
    if str(ball_diameter) not in VALID_BALLS:
        raise ConfigError("水弹直径 仅支持 %s，实际为: %r" % ("/".join(VALID_BALLS), ball_diameter))
    if float(ball_diameter) > float(barrel_bore):
        raise ConfigError("水弹直径(%s) 大于内管管径(%s)，无法装入内管"
                          % (ball_diameter, barrel_bore))

    feed = data.get("供蛋") or {}
    feed_mode = feed.get("方式", "普通波轮")
    feed_mode = FEED_ALIAS.get(feed_mode, feed_mode)  # 兼容旧名称
    if feed_mode not in VALID_FEED_MODES:
        raise ConfigError("供蛋.方式 仅支持 %s，实际为: %r"
                          % ("/".join(VALID_FEED_MODES), feed_mode))
    feed_min = feed.get("最小供蛋时间ms", None)
    if feed_min is not None:
        feed_min = _require_number(feed_min, "供蛋.最小供蛋时间ms")
        if feed_min <= 0:
            raise ConfigError("供蛋.最小供蛋时间ms 必须大于 0")

    cfg = SimConfig(
        motor_rpm=rpm, load_factor=load_factor, ratio=ratio,
        motor_model=motor_model,
        batt_cells=batt_cells, batt_capacity_mah=batt_cap, batt_c_rate=batt_c,
        install_delay=install_delay, cut_flag=cut_flag,
        front_cut=cut.get("前切齿数", 0), rear_cut=cut.get("后切齿数", 0),
        cylinder=cylinder, spring=spring,
        barrel_length_mm=barrel_length, barrel_bore=str(barrel_bore),
        ball_diameter=str(ball_diameter),
        feed_mode=feed_mode, feed_min_ms=feed_min,
    )

    errors = []
    apply_overrides(cfg.device, data.get("器件参数覆盖"), DEVICE_KEY_MAP, errors)
    apply_overrides(cfg.threshold, data.get("判定阈值覆盖"), THRESHOLD_KEY_MAP, errors)
    if errors:
        raise ConfigError("；".join(errors))

    return cfg
