# -*- coding: utf-8 -*-
"""供蛋模型：推嘴缩回形成的供蛋时间窗口 + 供蛋方式最高速率。

  供蛋方式按最高供弹速率定义（用户确认）：
    普通波轮 30 发/秒 / 高级波轮 50 发/秒 / 压力弹匣 60 发/秒；
  最小供蛋间隔 = 1000 ÷ 最高速率，可在配置中手动覆盖。
"""
from dataclasses import dataclass

from .params import SimConfig
from .timing import Timeline


@dataclass
class FeedResult:
    mode: str           # 供蛋方式
    max_rps: float      # 该方式最高供弹速率 (发/秒)
    min_ms: float       # 最小供蛋间隔
    window_ms: float    # 实际供蛋窗口
    slack_ms: float     # 窗口富余 = 窗口 − 最小时间（负数=不足）


def evaluate(cfg: SimConfig, tl: Timeline) -> FeedResult:
    window = tl.window_end_ms - tl.window_start_ms
    return FeedResult(mode=cfg.feed_mode, max_rps=cfg.feed_max_rps,
                      min_ms=cfg.min_feed_ms,
                      window_ms=window, slack_ms=window - cfg.min_feed_ms)
