# -*- coding: utf-8 -*-
"""本地网页界面后端（仅标准库）：
  GET  /              → 界面页面（gearbox_sim/web/index.html）
  GET  /api/default   → 默认配置
  POST /api/simulate  → 实时模拟（请求体=配置JSON，复用引擎与中文校验）
"""
import argparse
import json
import os
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .params import load_config, ConfigError
from .timing import build_timeline
from .dynamics import compute as compute_dyn
from .feeding import evaluate
from .ballistics import compute as compute_bal
from .diagnosis import (run_checks, enumerate_cut_schemes, build_conclusion,
                        seal_margin_ms)
from .report import render, events_list

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")

DEFAULT_CONFIG = {
    "电机": {"标称转速RPM": 30000},
    "齿轮比": "13:1",
    "切齿": {"前切齿数": 0, "后切齿数": 0},
    "气缸类型": "50%",
    "弹簧": "M90",
    "内管长度mm": 210.0,
    "内管管径": "7.5",
    "水弹直径": "7.2",
    "供蛋": {"方式": "普通波轮", "最小供蛋时间ms": None},
    "器件参数覆盖": {},
    "判定阈值覆盖": {},
}


def simulate_payload(data: dict) -> dict:
    """跑一次完整模拟，返回结构化结果（供网页渲染）。抛出 ConfigError 表示配置错误。"""
    cfg = load_config(data)
    dev = cfg.device
    tl = build_timeline(cfg, dev)
    dyn = compute_dyn(cfg, dev, tl)
    feed = evaluate(cfg, tl)
    bal = compute_bal(cfg, dev, tl, dyn)
    checks = run_checks(cfg, dev, tl, dyn, feed, bal)
    rows = enumerate_cut_schemes(cfg)
    conclusions = build_conclusion(cfg, rows[0], bal, tl)
    th = cfg.threshold

    cut = "无切"
    if cfg.front_cut or cfg.rear_cut:
        cut = ""
        if cfg.front_cut:
            cut += "前切%d" % cfg.front_cut
        if cfg.rear_cut:
            cut += "后切%d" % cfg.rear_cut

    return {
        "ok": True,
        "overview": {
            "period_ms": tl.period_ms, "rof_rps": tl.rof_rps,
            "stroke_mm": tl.stroke_mm, "stroke_ratio": tl.stroke_ratio,
            "remain_teeth": tl.remain_teeth,
            "v_m_s": bal.v_m_s, "energy_j": bal.energy_j,
            "p_max_kpa": bal.p_max_kpa, "swept_cm3": bal.swept_cm3,
            "useful_stroke_mm": bal.useful_stroke_mm,
            "barrel_length_mm": cfg.barrel_length_mm,
            "gap_mm": bal.gap_mm, "leak_ratio": bal.leak_ratio,
            "return_margin_ms": bal.return_margin_ms,
            "seal_margin_ms": seal_margin_ms(tl, bal),
            "window_ms": feed.window_ms, "min_feed_ms": cfg.min_feed_ms,
            "feed_max_rps": cfg.feed_max_rps,
            "air_index": dyn.air_index, "t_fire_ms": bal.t_fire_ms,
            "energy_mj": dyn.energy_mj, "v_release_m_s": dyn.v_release_m_s,
            "v_impact_m_s": bal.v_impact_m_s, "p_max_kpa": bal.p_max_kpa,
            "loaded_rpm": tl.loaded_rpm, "motor_desc": tl.motor_desc,
            "motor_stall": tl.motor_stall,
            "torque_peak_ratio": tl.torque_peak_ratio,
            "batt_desc": tl.batt_desc, "batt_sag": tl.batt_sag,
            "batt_shots": tl.batt_shots,
            "cut_tag": cut,
            # 阈值（前端着色用）
            "velocity_warn_ms": th.velocity_warn_ms,
            "gear_clash_margin_ms": th.gear_clash_margin_ms,
            "air_seal_margin_ms": th.air_seal_margin_ms,
            "low_air_index": th.low_air_index,
            "barrel_match_ratio": th.barrel_match_ratio,
        },
        "checks": [{"name": c.name, "level": c.level, "detail": c.detail} for c in checks],
        "conclusions": conclusions,
        "events": sorted(events_list(cfg, dev, tl, dyn, feed, bal), key=lambda e: e["t"]),
        "schemes": [r.__dict__ for r in rows],
        "report_text": render(cfg, dev, tl, dyn, feed, bal, checks, rows,
                              conclusions),
    }


class Handler(BaseHTTPRequestHandler):
    JSON_CT = "application/json; charset=utf-8"

    def _send(self, code, body, ctype=JSON_CT):
        data = body if isinstance(body, bytes) else \
            json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            try:
                with open(os.path.join(WEB_DIR, "index.html"), "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            except OSError:
                self._send(500, {"ok": False, "error": "界面文件缺失: web/index.html"})
        elif path == "/api/default":
            self._send(200, DEFAULT_CONFIG)
        else:
            self._send(404, {"ok": False, "error": "未找到: %s" % path})

    def do_POST(self):
        if self.path.split("?", 1)[0] != "/api/simulate":
            self._send(404, {"ok": False, "error": "未找到: %s" % self.path})
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length).decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            self._send(400, {"ok": False, "error": "请求体不是合法 JSON"})
            return
        try:
            self._send(200, simulate_payload(data))
        except ConfigError as e:
            self._send(400, {"ok": False, "error": str(e)})
        except Exception as e:  # 防止引擎异常拖垮服务
            self._send(500, {"ok": False, "error": "模拟失败：%r" % e})

    def log_message(self, *args):  # 静默访问日志
        pass


def build_server(host="127.0.0.1", port=8765):
    return ThreadingHTTPServer((host, port), Handler)


def main(argv=None):
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description="水弹波箱数据模拟 · 本地网页界面")
    parser.add_argument("--host", default="127.0.0.1", help="监听地址（默认 127.0.0.1）")
    parser.add_argument("--port", type=int, default=8765, help="端口（默认 8765）")
    parser.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    args = parser.parse_args(argv)

    server = build_server(args.host, args.port)
    url = "http://%s:%d/" % (args.host, server.server_address[1])
    print("水弹波箱模拟界面已启动：%s" % url)
    print("按 Ctrl+C 停止。")
    if not args.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
