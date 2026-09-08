# -*- coding: utf-8 -*-
"""一键启动本地网页界面：python webui.py [--port 8765] [--no-browser]"""
import sys

from gearbox_sim.webapp import main

if __name__ == "__main__":
    sys.exit(main())
