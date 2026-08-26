# GPS_Localize — working rules

Project-specific rules. They travel with the repository, so they apply to
anyone working on it, on any branch.

## After changing anything, update everything that points at it

A change is not finished when the code works. It is finished when nothing else
in the repository still describes the old behaviour. Documentation that
disagrees with the code is worse than no documentation: it is believed.

**Every change ends with a sweep for references to what changed.** Before
committing, search the whole repository for the old value, the old path and the
old name — not only the file you edited.

```bash
grep -rn "<old value>" --include='*.md' --include='*.py' --include='*.h' \
    --include='*.cpp' --include='*.js' --include='*.yaml' --include='*.sh' . \
    | grep -v '_work\|_patches'
```

Things that go stale, from real cases in this project:

| Changed | Also had to change |
|---|---|
| battery cutoff 9.9 → 10.7 V | `docs/safety.md`, `docs/hardware.md`, `docs/web_ui.md`, `docs/topics.md` |
| diode drop 0.45 → 1.81/1.90 V | `docs/hardware.md` |
| headers moved `lib/` → `src/` | `platformio.ini` include paths **and** `lib_deps`, `tools/qc.py` |
| telemetry 40 → 42 fields | `main.cpp`, `telemetry.py`, `web/app.js` — all three together |
| `mor_luam` removed | `README.md` |
| agent hostname `ros2-agent` → `mannaja` | `docs/install.md` |
| a new HTTP endpoint | `web_server.py`, `viewer_server.py` (parity is enforced), `app.js` |
| a new web UI tab | `index.html`, `app.js`, `style.css`, the tab list in `tools/qc.py` |

Where a rule can be machine-checked, add it to `tools/qc.py` rather than
trusting anyone to remember it. Most checks in there exist because something
broke exactly once.

## Documentation that must stay true

- `README.md` — what this is and how to start it
- `docs/architecture.md` — protocols, ports, data flow, sensor meanings
- `docs/install.md`, `docs/hardware.md`, `docs/safety.md`, `docs/topics.md`,
  `docs/web_ui.md`, `docs/tuning.md`, `docs/acceptance.md`
- `setup/README.md`, `webapp/README.md`
- the `docker` branch `README.md`, written by `tools/publish_branches.sh`

## Before every commit

```bash
python3 tools/qc.py                        # cross-file contracts
python3 tools/port_qc.py --static-only .   # port gate
cd firmware && pio test -e native          # 124 host tests
pio run -e wifi                            # firmware still builds
```

All four must pass. `port_qc.py` without `--static-only` also builds the
workspace, smoke-tests the nodes and checks the live link; the link check
skips itself when no ESP32 is attached rather than failing.

## Where work happens

- Edit in `_work/`, never the real folders directly.
- Promote only after QC is green, and only through a `_patches/<name>_<stamp>/`
  with `before/`, `after/`, `changes.diff` and a working `rollback.sh`.
- `_patches/` is the only backup mechanism; git history covers the rest.
- `plan.html` lives in the real project root and is regenerated after every
  task, so progress can be checked without asking.

## Branches

`main` is the only branch anyone edits. `workspace`, `firmware` and `docker`
are generated from it by `tools/publish_branches.sh` and must never be
hand-edited — run the script after any promotion so all four stay in step.

## Hardware-gated work

Anything needing an ESP32 or the full robot is marked in `plan.html` with the
part it needs and is not started until that hardware exists. A simulation pass
is necessary but never sufficient: it is not marked done from a sim result.

## Things that are true and easy to get wrong

- Pack voltage comes from the **motor rail** INA226s, never the processor one —
  that measures the ESP32 +5 V rail and reading it as the pack latches e-stop
  three seconds after boot.
- `ROS_DOMAIN_ID` is **10** and must match on both sides. A mismatch shows an
  empty topic list with no error.
- The firmware **node** in the graph proves a board is connected. A topic being
  present does not: a host node that only subscribes creates the topic too.
- On Docker Desktop `network_mode: host` does nothing, and `devices: []` does
  **not** clear a device list — Compose merges it. Only `devices: !reset []`
  removes it.
