# -*- coding: utf-8 -*-
"""命令行入口。

用法：
    python run.py                              # 使用 configs/default.json
    python run.py -c configs/examples/xxx.json # 指定配置
"""
import argparse
import json
import sys

from .params import load_config, ConfigError
from .report import render_full


def main(argv=None):
    # Windows 控制台默认 GBK，统一转 UTF-8 避免特殊符号（如 − ≥ ³）编码报错
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")

    parser = argparse.ArgumentParser(
        description="水弹玩具发射器波箱内部运作数据模拟（无图形，输出时序与判定报告）")
    parser.add_argument("-c", "--config", default="configs/default.json",
                        help="配置文件路径（JSON，默认 configs/default.json）")
    args = parser.parse_args(argv)

    try:
        with open(args.config, "r", encoding="utf-8") as f:
            data = json.load(f)
    except OSError as e:
        print("错误：无法读取配置文件 %s：%s" % (args.config, e))
        return 1
    except json.JSONDecodeError as e:
        print("错误：配置文件不是合法 JSON：%s" % e)
        return 1

    try:
        cfg = load_config(data)
    except ConfigError as e:
        print("配置错误：%s" % e)
        return 1

    print(render_full(cfg))
    return 0


if __name__ == "__main__":
    sys.exit(main())
