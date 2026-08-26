#!/usr/bin/env python3
"""
Generate plan.html from plan_state.json plus the latest QC run.

plan.html is the user-facing progress board. It lives in the REAL project root, not in
_work/, because the whole point is that the user opens it to check whether the work is
following the plan. Every change regenerates it with a fresh timestamp.

    python3 tools/planhtml.py                       # regenerate with a fresh timestamp
    python3 tools/planhtml.py --qc                  # run QC first, embed the result
    python3 tools/planhtml.py --out /path/plan.html # non-default destination

The generator is the single writer. Nothing else edits plan.html, so the change log
stays ordered and no concurrent write can lose an entry.
"""

from __future__ import annotations

import argparse
import html
import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
STATE = HERE / "plan_state.json"

STATE_COLORS = {
    "DONE": "ok",
    "IN PROGRESS": "run",
    "TODO": "todo",
    "BLOCKED": "block",
    "PROBLEM": "bad",
}
SEVERITY_COLORS = {"BLOCKER": "bad", "HIGH": "warn", "MEDIUM": "run", "LOW": "todo"}

# Recovered from firmware/src/main.cpp. Rendered on the page so the contract both sides
# must match is always in front of the user.
TOPICS_PUB = [
    ("/gps_localize/odom", "nav_msgs/Odometry"),
    ("/gps_localize/imu/data", "sensor_msgs/Imu"),
    ("/gps_localize/gps/fix", "sensor_msgs/NavSatFix"),
    ("/gps_localize/telemetry", "std_msgs/Float32MultiArray (40 floats)"),
]
TOPICS_SUB = [
    ("/gps_localize/cmd_move", "geometry_msgs/Twist"),
    ("/gps_localize/cmd_manual", "geometry_msgs/Twist"),
    ("/gps_localize/cmd_vel", "geometry_msgs/Twist"),
    ("/gps_localize/heartbeat", "std_msgs/Int32"),
    ("/gps_localize/estop", "std_msgs/Bool"),
    ("/gps_localize/config/pid", "std_msgs/Float32MultiArray"),
]
TELEMETRY = [
    "run state", "heading deg", "target heading", "heading error", "speed m/s",
    "target speed", "wheel L rpm", "wheel R rpm", "wheel L m/s", "wheel R m/s",
    "yaw rate deg/s", "pwm left", "pwm right", "batt V", "batt A",
    "motor A V", "motor A A", "motor B V", "motor B A", "fix quality",
    "satellites", "hdop", "gps speed m/s", "gps course deg", "safety flags",
    "free heap KB", "loop us", "est x", "est y", "compass heading",
    "heading ref", "compass ok/dist", "mag calib state", "estimator health",
    "speed source", "back-EMF rpm L", "back-EMF rpm R", "motor fault bits",
    "fan V", "fan A",
]

CSS = """
:root{
  --bg:#f6f7f9; --surface:#fff; --surface-2:#eef0f4; --border:#d7dbe2;
  --fg:#1a1d23; --fg-dim:#5c6470; --accent:#2563eb;
  --ok:#137a3d; --ok-bg:#dcf5e6; --bad:#b4232c; --bad-bg:#fde4e4;
  --warn:#9a5b00; --warn-bg:#fdefd6; --run:#1d5fb0; --run-bg:#dde9fb;
  --todo:#5c6470; --todo-bg:#e6e9ee; --block:#7a3ba8; --block-bg:#f0e4fa;
  --mono:ui-monospace,SFMono-Regular,"SF Mono",Menlo,Consolas,monospace;
  --sp1:4px; --sp2:8px; --sp3:12px; --sp4:16px; --sp5:24px; --sp6:32px;
  --radius:8px;
}
@media (prefers-color-scheme:dark){
  :root:not([data-theme="light"]){
    --bg:#12141a; --surface:#1a1e26; --surface-2:#232833; --border:#333a47;
    --fg:#e8eaf0; --fg-dim:#98a1b0; --accent:#6ea0ff;
    --ok:#5fd48a; --ok-bg:#12331f; --bad:#ff8b8b; --bad-bg:#3a1a1c;
    --warn:#f0bc63; --warn-bg:#3a2a10; --run:#7fb1ff; --run-bg:#152742;
    --todo:#98a1b0; --todo-bg:#262b35; --block:#c79cf0; --block-bg:#2b1c3d;
  }
}
:root[data-theme="dark"]{
  --bg:#12141a; --surface:#1a1e26; --surface-2:#232833; --border:#333a47;
  --fg:#e8eaf0; --fg-dim:#98a1b0; --accent:#6ea0ff;
  --ok:#5fd48a; --ok-bg:#12331f; --bad:#ff8b8b; --bad-bg:#3a1a1c;
  --warn:#f0bc63; --warn-bg:#3a2a10; --run:#7fb1ff; --run-bg:#152742;
  --todo:#98a1b0; --todo-bg:#262b35; --block:#c79cf0; --block-bg:#2b1c3d;
}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
  font:15px/1.55 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;
  padding:var(--sp5);max-width:1180px;margin-inline:auto}
h1{font-size:26px;margin:0 0 var(--sp1)}
h2{font-size:17px;margin:var(--sp6) 0 var(--sp3);
   border-bottom:1px solid var(--border);padding-bottom:var(--sp2)}
h3{font-size:14px;margin:var(--sp4) 0 var(--sp2);color:var(--fg-dim);
   text-transform:uppercase;letter-spacing:.06em}
.sub{color:var(--fg-dim);font-size:14px;margin:0 0 var(--sp2)}
.stamp{display:inline-block;font-family:var(--mono);font-size:13px;
  background:var(--surface-2);border:1px solid var(--border);
  padding:var(--sp2) var(--sp3);border-radius:var(--radius);margin-top:var(--sp3)}
.wrap{overflow-x:auto;-webkit-overflow-scrolling:touch}
table{border-collapse:collapse;width:100%;background:var(--surface);
  border:1px solid var(--border);border-radius:var(--radius);font-size:14px}
th,td{text-align:left;padding:var(--sp2) var(--sp3);
  border-bottom:1px solid var(--border);vertical-align:top}
th{background:var(--surface-2);font-weight:600;font-size:13px;
   color:var(--fg-dim);white-space:nowrap}
tr:last-child td{border-bottom:none}
code,.mono{font-family:var(--mono);font-size:13px}
.badge{display:inline-block;padding:2px var(--sp2);border-radius:4px;
  font-size:12px;font-weight:600;white-space:nowrap;font-family:var(--mono)}
.ok{color:var(--ok);background:var(--ok-bg)}
.bad{color:var(--bad);background:var(--bad-bg)}
.warn{color:var(--warn);background:var(--warn-bg)}
.run{color:var(--run);background:var(--run-bg)}
.todo{color:var(--todo);background:var(--todo-bg)}
.block{color:var(--block);background:var(--block-bg)}
.gate{padding:var(--sp3) var(--sp4);border-radius:var(--radius);
  font-weight:600;margin-bottom:var(--sp3);border:1px solid var(--border)}
.gate.pass{color:var(--ok);background:var(--ok-bg);border-color:var(--ok)}
.gate.fail{color:var(--bad);background:var(--bad-bg);border-color:var(--bad)}
.note{color:var(--fg-dim);font-size:13px}
.fail-list{margin:var(--sp1) 0 0;padding-left:var(--sp4);
  color:var(--bad);font-family:var(--mono);font-size:12px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(230px,1fr));
  gap:var(--sp2)}
.cell{background:var(--surface);border:1px solid var(--border);
  border-radius:6px;padding:var(--sp2) var(--sp3);font-size:13px}
.cell b{font-family:var(--mono);color:var(--accent);margin-right:var(--sp2)}
footer{margin-top:var(--sp6);padding-top:var(--sp3);
  border-top:1px solid var(--border);color:var(--fg-dim);font-size:13px}
"""


def esc(s) -> str:
    return html.escape(str(s if s is not None else ""))


def render(state: dict, qc: dict | None, stamp: str) -> str:
    p: list[str] = []
    a = p.append

    a("<title>GPS_Localize Plan</title>")
    a(f"<style>{CSS}</style>")

    a(f"<h1>{esc(state['project'])} &mdash; Ubuntu + ROS 2 port</h1>")
    a(f"<p class='sub'>{esc(state['target'])}</p>")
    a(f"<p class='sub'>Staging: <code>{esc(state['staging'])}</code> "
      "&mdash; the real folders are never edited directly. Work is promoted only after "
      "a green QC run.</p>")
    a(f"<div class='stamp'>last updated &nbsp;<strong>{esc(stamp)}</strong></div>")

    # ---- QC gate -----------------------------------------------------------
    a("<h2>QC gate</h2>")
    if qc is None:
        a("<div class='gate fail'>No QC run recorded yet.</div>")
    else:
        allowed = qc.get("promotion_allowed", False)
        mode = "static checks only" if qc.get("static_only") else "full run"
        cls = "pass" if allowed else "fail"
        verdict = ("PROMOTION ALLOWED" if allowed
                   else "PROMOTION BLOCKED &mdash; nothing is written to the real folders")
        a(f"<div class='gate {cls}'>{verdict}</div>")
        a(f"<p class='note'>{esc(qc.get('timestamp',''))} &nbsp;·&nbsp; {mode} "
          f"&nbsp;·&nbsp; <code>{esc(qc.get('root',''))}</code></p>")
        a("<div class='wrap'><table>")
        a("<tr><th>#</th><th>check</th><th>result</th><th>detail</th><th>time</th></tr>")
        for c in qc.get("checks", []):
            badge = ("<span class='badge ok'>PASS</span>" if c["ok"]
                     else "<span class='badge bad'>FAIL</span>")
            fails = ""
            if not c["ok"] and c.get("failures"):
                items = "".join(f"<li>{esc(x)}</li>" for x in c["failures"][:8])
                more = (f"<li>&hellip; and {c['failure_count'] - 8} more</li>"
                        if c["failure_count"] > 8 else "")
                fails = f"<ul class='fail-list'>{items}{more}</ul>"
            a(f"<tr><td class='mono'>{c['number']}</td><td>{esc(c['name'])}</td>"
              f"<td>{badge}</td><td>{esc(c['detail'])}{fails}</td>"
              f"<td class='mono'>{c['seconds']}s</td></tr>")
        a("</table></div>")

    # ---- status board ------------------------------------------------------
    a("<h2>Status board</h2>")
    a("<div class='wrap'><table>")
    a("<tr><th>#</th><th>step</th><th>state</th><th>owner</th>"
      "<th>files</th><th>note</th></tr>")
    for s in state["steps"]:
        cls = STATE_COLORS.get(s["state"], "todo")
        a(f"<tr><td class='mono'>{s['n']}</td><td><strong>{esc(s['title'])}</strong></td>"
          f"<td><span class='badge {cls}'>{esc(s['state'])}</span></td>"
          f"<td class='note'>{esc(s['owner'])}</td>"
          f"<td class='mono note'>{esc(s['files'])}</td>"
          f"<td class='note'>{esc(s['note'])}</td></tr>")
    a("</table></div>")

    # ---- problems ----------------------------------------------------------
    a("<h2>Problems</h2>")
    probs = state.get("problems", [])
    if not probs:
        a("<p class='note'>None open.</p>")
    else:
        a("<div class='wrap'><table>")
        a("<tr><th>found</th><th>severity</th><th>problem</th>"
          "<th>state</th><th>impact</th></tr>")
        for pr in probs:
            cls = SEVERITY_COLORS.get(pr["severity"], "todo")
            a(f"<tr><td class='mono note'>{esc(pr['ts'])}</td>"
              f"<td><span class='badge {cls}'>{esc(pr['severity'])}</span></td>"
              f"<td>{esc(pr['what'])}</td><td class='note'>{esc(pr['state'])}</td>"
              f"<td class='note'>{esc(pr.get('impact',''))}</td></tr>")
        a("</table></div>")

    # ---- change log --------------------------------------------------------
    a("<h2>Change log</h2>")
    a("<p class='note'>Newest first. Every change appends one entry; nothing is "
      "overwritten.</p>")
    a("<div class='wrap'><table>")
    a("<tr><th>when</th><th>step</th><th>what changed</th>"
      "<th>files</th><th>promoted</th></tr>")
    for e in state.get("log", []):
        promoted = ("<span class='badge ok'>YES</span>" if e.get("promoted")
                    else "<span class='badge todo'>no</span>")
        problem = (f"<div class='fail-list'>{esc(e['problem'])}</div>"
                   if e.get("problem") else "")
        a(f"<tr><td class='mono note'>{esc(e['ts'])}</td>"
          f"<td class='mono'>{esc(e['step'])}</td>"
          f"<td>{esc(e['what'])}{problem}</td>"
          f"<td class='mono note'>{esc(e.get('files',''))}</td>"
          f"<td>{promoted}</td></tr>")
    a("</table></div>")

    # ---- contract ----------------------------------------------------------
    a("<h2>Firmware contract</h2>")
    a("<p class='note'>Recovered from <code>firmware/src/main.cpp</code>, which "
      "survived. Namespace <code>/gps_localize</code>, "
      "<code>ROS_DOMAIN_ID 10</code>, node <code>gps_localize_firmware</code>, "
      "micro-ROS agent on <code>UDP4 :8888</code>. The host side must match this "
      "exactly &mdash; QC check 11 enforces it.</p>")

    a("<h3>ESP32 &rarr; host</h3><div class='wrap'><table>")
    a("<tr><th>topic</th><th>type</th></tr>")
    for t, ty in TOPICS_PUB:
        a(f"<tr><td class='mono'>{esc(t)}</td><td class='mono note'>{esc(ty)}</td></tr>")
    a("</table></div>")

    a("<h3>host &rarr; ESP32</h3><div class='wrap'><table>")
    a("<tr><th>topic</th><th>type</th></tr>")
    for t, ty in TOPICS_SUB:
        a(f"<tr><td class='mono'>{esc(t)}</td><td class='mono note'>{esc(ty)}</td></tr>")
    a("</table></div>")

    a("<h3>telemetry array &mdash; 40 floats</h3><div class='grid'>")
    for i, name in enumerate(TELEMETRY):
        a(f"<div class='cell'><b>{i}</b>{esc(name)}</div>")
    a("</div>")

    a("<footer>Generated by <code>tools/planhtml.py</code>. "
      "This file is the one thing written straight to the real project root; "
      "everything else stages in <code>_work/</code> first.</footer>")

    return "\n".join(p)


def main() -> int:
    ap = argparse.ArgumentParser(description="generate plan.html")
    ap.add_argument("--state", default=str(STATE))
    ap.add_argument("--out", default=None)
    ap.add_argument("--qc", action="store_true",
                    help="run qc.py --static-only --json first and embed the result")
    ap.add_argument("--qc-json", default=None,
                    help="embed an existing qc --json file instead of running it")
    args = ap.parse_args()

    state_path = Path(args.state).resolve()
    state = json.loads(state_path.read_text(encoding="utf-8"))

    # plan.html always belongs at the real project root. Find it by walking up for the
    # workspace directory rather than counting parents: tools/ now exists both in the
    # staging tree and in the promoted tree, at different depths.
    def project_root(start: Path) -> Path:
        for d in [start, *start.parents]:
            if (d / "gps_localize_ws").is_dir() and d.name != "_work":
                return d
        return start.parents[1]

    out = (Path(args.out).resolve() if args.out
           else project_root(state_path.parent) / "plan.html")

    qc = None
    if args.qc_json:
        qc = json.loads(Path(args.qc_json).read_text(encoding="utf-8"))
    elif args.qc:
        qc_py = state_path.parent / "port_qc.py"
        target = state_path.parents[1]          # the staging tree
        proc = subprocess.run(
            [sys.executable, str(qc_py), "--static-only", "--json", str(target)],
            capture_output=True, text=True)
        try:
            qc = json.loads(proc.stdout)
        except json.JSONDecodeError:
            print("planhtml: could not parse qc output:\n" + proc.stderr, file=sys.stderr)

    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    out.write_text(render(state, qc, stamp), encoding="utf-8")
    print(f"plan.html written: {out}   ({stamp})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
