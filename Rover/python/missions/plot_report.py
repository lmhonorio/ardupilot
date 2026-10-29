#!/usr/bin/env python3.8
import argparse
import json
from pathlib import Path
from report import render_report
p=argparse.ArgumentParser()
p.add_argument('folder',type=Path)
a=p.parse_args()
render_report(a.folder,json.loads((a.folder/'report.json').read_text()))
print(a.folder/'report.html')
