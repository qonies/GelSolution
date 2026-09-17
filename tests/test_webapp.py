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
from gearbox_sim.params import ConfigError, load_config

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

# ---- 参数分区预设（GET /api/presets，configs/presets/<分区>/<名称>.json）----
pr = json.loads(urllib.request.urlopen(base + "/api/presets", timeout=5).read().decode("utf-8"))
check("GET /api/presets 五个分区", set(pr.keys()) == {"电机", "齿轮", "电池", "气缸", "弹簧"})
check("预设均有名称且参数为对象",
      all(isinstance(p.get("参数"), dict) and p.get("名称")
          for sec in pr.values() for p in sec),
      str({k: len(v) for k, v in pr.items()}))
check("电机预设含 4 款曲线电机",
      {"超力无刷4W8", "超力无刷3W9", "超力有刷3W5", "超力有刷3W3"}
      <= {p["名称"] for p in pr["电机"]})
check("弹簧预设自然排序（M75 < M100）",
      [p["名称"] for p in pr["弹簧"]][:3] == ["M75", "M80", "M85"])

# 全部预设并入默认配置须可解析（空预设如「不启用」跳过；电池预设须配曲线模式电机）
merge_ok, bad, merged = True, "", 0
for sec, items in pr.items():
    for p in items:
        if p.get("错误"):
            merge_ok, bad = False, "%s/%s 解析失败" % (sec, p["名称"])
            break
        m = p["参数"]
        if not m:
            continue
        cfg = json.loads(json.dumps(DEFAULT_CONFIG))
        if sec == "电机":
            cfg["电机"] = dict(m)
        elif sec == "电池":
            cfg["电机"] = {"型号": "超力无刷4W8"}
            cfg["电池"] = dict(m)
        elif sec == "齿轮":
            cfg["齿轮比"] = m["齿轮比"]
            cfg["切齿"] = {"前切齿数": m.get("前切齿数", 0), "后切齿数": m.get("后切齿数", 0)}
            if "天梯齿数" in m:
                cfg["器件参数覆盖"]["天梯齿数"] = m["天梯齿数"]
        elif sec == "气缸":
            cfg["气缸类型"] = m["气缸类型"]
            for k in ("气缸内径mm", "气缸长度mm", "气缸余隙容积cm3", "开孔段保留系数"):
                if k in m:
                    cfg["器件参数覆盖"][k] = m[k]
        elif sec == "弹簧":
            cfg["弹簧"] = m["弹簧"]
            for k in ("弹簧预压mm", "活塞满行程mm", "活塞质量g", "撞击回弹系数"):
                if k in m:
                    cfg["器件参数覆盖"][k] = m[k]
        try:
            load_config(cfg)
            merged += 1
        except ConfigError as e:
            merge_ok, bad = False, "%s/%s: %s" % (sec, p["名称"], e)
            break
check("全部预设并入配置可解析（%d 套）" % merged, merge_ok, bad)

# 预设端到端：4W8 电机 + 3S 电池预设合并模拟（曲线模式负载转速 > 0）
cfg = json.loads(json.dumps(DEFAULT_CONFIG))
cfg["电机"] = {"型号": "超力无刷4W8"}
cfg["电池"] = {"电芯数": 3, "容量mAh": 1400, "放电倍率": 30}
p3 = simulate_payload(cfg)
check("预设合并端到端模拟 ok", p3["ok"] and p3["overview"]["loaded_rpm"] > 0)

server.shutdown()
print()
if FAILS:
    print("失败 %d 项: %s" % (len(FAILS), FAILS))
    sys.exit(1)
print("网页接口测试全部通过。")
