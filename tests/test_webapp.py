# -*- coding: utf-8 -*-
"""网页界面测试：simulate_payload + HTTP 层。直接 python tests/test_webapp.py 运行。"""
import json
import os
import sys
import threading
import urllib.request
import urllib.error

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

from gearbox_sim.webapp import simulate_payload, build_server, DEFAULT_CONFIG
from gearbox_sim.params import ConfigError

FAILS = []


def check(name, cond, detail=""):
    status = "PASS" if cond else "FAIL"
    print("[%s] %s %s" % (status, name, detail))
    if not cond:
        FAILS.append(name)


VALID = json.loads(json.dumps(DEFAULT_CONFIG))  # 深拷贝

# ---- simulate_payload ----
p = simulate_payload(VALID)
check("payload ok", p["ok"] is True)
check("overview 初速量级（默认配置 50缸/210管 ≈59）", 45.0 < p["overview"]["v_m_s"] < 85.0,
      "v=%.1f" % p["overview"]["v_m_s"])
check("10项判定", len(p["checks"]) == 10)
check("16方案", len(p["schemes"]) == 16 and "v_m_s" in p["schemes"][0])
check("10事件", len(p["events"]) == 10)
check("报告文本", "水弹初速" in p["report_text"] and "结论建议" in p["report_text"])
check("结论非空", len(p["conclusions"]) >= 3)

# 初速超阈值标志传给前端
strong = json.loads(json.dumps(VALID))
strong["弹簧"] = "M110"; strong["气缸类型"] = "70%"
strong["内管管径"] = "7.3"; strong["水弹直径"] = "7.3"
strong["内管长度mm"] = 350
p2 = simulate_payload(strong)
check("强力配置初速超80", p2["overview"]["v_m_s"] > 80.0,
      "v=%.1f" % p2["overview"]["v_m_s"])

# 配置错误 → ConfigError（后端转 400）
try:
    simulate_payload({"电机": {"标称转速RPM": 30000}, "弹簧": "M120"})
    check("配置错误抛ConfigError", False, "未抛出")
except ConfigError as e:
    check("配置错误抛ConfigError: %s" % e, True)

# ---- HTTP 层 ----
server = build_server("127.0.0.1", 0)
port = server.server_address[1]
threading.Thread(target=server.serve_forever, daemon=True).start()
base = "http://127.0.0.1:%d" % port

html = urllib.request.urlopen(base + "/", timeout=5).read().decode("utf-8")
check("GET / 返回界面", "水弹波箱运作数据模拟" in html and "api/simulate" in html)
d = json.loads(urllib.request.urlopen(base + "/api/default", timeout=5).read().decode("utf-8"))
check("GET /api/default", d["电机"]["标称转速RPM"] == 30000)
check("GET /api/default 预填阈值/器件默认值",
      d["判定阈值覆盖"].get("打齿安全裕量ms") == 2.0
      and d["判定阈值覆盖"].get("最少保留啮合齿数") == 8
      and d["器件参数覆盖"].get("气动效率") == 0.243
      and d["器件参数覆盖"].get("开孔段保留系数") == 0.6
      and d["器件参数覆盖"].get("弹簧预压mm") == 62.5
      and "弹簧刚度表" not in d["器件参数覆盖"])

req = urllib.request.Request(base + "/api/simulate", method="POST",
                             data=json.dumps(VALID).encode("utf-8"),
                             headers={"Content-Type": "application/json"})
d = json.loads(urllib.request.urlopen(req, timeout=5).read().decode("utf-8"))
check("POST /api/simulate ok", d["ok"] and d["overview"]["v_m_s"] > 0)

try:
    req = urllib.request.Request(base + "/api/simulate", method="POST",
                                 data=b"not-json", headers={"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=5)
    check("非法JSON→400", False, "未报错")
except urllib.error.HTTPError as e:
    check("非法JSON→400", e.code == 400)

req = urllib.request.Request(base + "/api/simulate", method="POST",
                             data=json.dumps({"电机": {"标称转速RPM": 30000}, "齿轮比": "20:1"}).encode("utf-8"),
                             headers={"Content-Type": "application/json"})
try:
    urllib.request.urlopen(req, timeout=5)
    check("无效配置→400", False, "未报错")
except urllib.error.HTTPError as e:
    body = json.loads(e.read().decode("utf-8"))
    check("无效配置→400", e.code == 400 and "齿轮比" in body["error"], body.get("error", ""))

server.shutdown()
print()
if FAILS:
    print("失败 %d 项: %s" % (len(FAILS), FAILS))
    sys.exit(1)
print("网页接口测试全部通过。")
