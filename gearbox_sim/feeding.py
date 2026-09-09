# -*- coding: utf-8 -*-
"""供蛋模型：供蛋方式能力上限 + 推嘴缩回形成的供蛋窗口（时序信息）。

  供蛋方式上限（✅v3.4 用户澄清）= 弹匣/波轮**实际支持发射器发射水弹的能力**
  （普通波轮 30 / 高级波轮 60 / 压力弹匣 80 发/秒），直接与射速比较；
  **不换算「每发平均供蛋时间」（1000÷上限）与供蛋窗口对比**——窗口为
  时序信息，供蛋是否跟得上以实测为准。
"""
from dataclasses import dataclass

from .params import SimConfig
from .timing import Timeline


@dataclass
class FeedResult:
    mode: str           # 供蛋方式
    max_rps: float      # 该方式供蛋能力上限 (发/秒，实际支持发射的能力)
    window_ms: float    # 实际供蛋窗口（推嘴完全缩回 → 开始回位；时序信息）


def evaluate(cfg: SimConfig, tl: Timeline) -> FeedResult:
    window = tl.window_end_ms - tl.window_start_ms
    return FeedResult(mode=cfg.feed_mode, max_rps=cfg.feed_max_rps,
                      window_ms=window)
