# GPS_Localize — instructions for Gemini (CLI or Antigravity)

Gemini CLI reads this file automatically. Codex reads `AGENTS.md` and Claude
Code reads `CLAUDE.md`. All three say the same things on purpose: an agent
should not behave differently on this project depending on which one it is.

1. **Read `HANDOFF.md` first.** It is the live state of the work - what the
   robot is plugged into right now, the task in progress with its measurements
   so far, and the queue. It is how work moves between Claude, Codex and you
   when one of them runs out of allowance. Update it before you stop.
2. **Then read `CLAUDE.md`.** It is the full set of working rules - the QC
   gates, the branch layout, and the list of things that are true and easy to
   get wrong on this robot. Everything in it applies to you.

The rules most often broken, repeated here:

- **Chat is terse ("caveman" style).** Drop articles, filler and pleasantries.
  Answer first. Bullets over paragraphs. Terse but complete - a failure report
  looks like "3 of 12 tests fail. TypeError in `parse_fix()` line 44 - `sats`
  is None when no lock." File contents are normal, complete English.
- **Never command the robot to move** while running unattended. Flashing
  firmware is allowed; driving is not.
- **Never push, publish or install system packages.**
- **Nothing works until the gates say so:**

  ```bash
  python3 tools/qc.py
  python3 tools/port_qc.py --static-only .
  cd firmware && pio test -e native
  pio run -e wifi
  ```

  and `python3 tools/live_probe.py` when the change touches anything the robot
  feeds. Report results as numbers with the command that produced them.
