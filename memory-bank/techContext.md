# 技术背景 (Tech Context)

## 技术栈
- Python 3（仅标准库：dataclasses、json、argparse、math、http.server）
- 运行环境：Windows + PowerShell，工作目录 d:\AppDev\GelSolution

## 环境注意事项
- PATH 上的 `python`/`git` 指向 cygwin shim（D:\AppDev\cygwin64\bin），
  在 PowerShell 管道中不可用、git 对 Windows 路径异常——勿直接使用；
- Python 用 `py -3`（3.9）或
  `%LOCALAPPDATA%\Programs\Python\Python39\python.exe`（本机用户目录下）；
  代码兼容 Python 3.9（__pycache__ 中残留 cpython-312 为旧环境产物）；
- git 用 `C:\Program Files\Git\cmd\git.exe`。

## 运行
- `python run.py [-c configs/xxx.json]`（命令行报告）
- `python webui.py [--port 8765] [--no-browser]`（本地网页界面，实时调参）
- `python tests/test_sim.py`（引擎 127 项）/ `python tests/test_webapp.py`（接口 14 项）

## 网页界面（webapp.py + web/index.html）
- 后端：仅标准库 http.server（ThreadingHTTPServer）
  - GET / → gearbox_sim/web/index.html
  - GET /api/default → 默认配置
  - POST /api/simulate → 实时模拟（复用 load_config 中文校验；ConfigError→400）；
    单次 ~37ms（结构化结果与报告文本同源，report.render 复用已算结果）
- 前端：单文件原生 HTML/JS/CSS，无框架无构建；150ms 防抖实时重算
- 端口默认 8765，仅监听 127.0.0.1

## 版本控制
- git 仓库（2026-09-08 初始化，Windows git）；项目版本号见
  gearbox_sim/__init__.py（当前 0.5.9）；历史沿革见 activeContext.md

## 关键单位约定
- 时间 ms、角度 度（°）、长度 mm、质量 g、弹簧刚度 N/mm、能量 mJ

## 配置 JSON 结构
见 README.md「配置文件」一节。覆盖参数通过中文键映射（components.py 的
DEVICE_KEY_MAP / THRESHOLD_KEY_MAP），未知键会报中文错误。