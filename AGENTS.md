# GPS_Localize — instructions for Codex and any other agent

Codex reads this file automatically. Claude Code reads `CLAUDE.md`. The two say
the same things on purpose: an agent should not behave differently on this
project depending on which one it is.

**Read `HANDOFF.md` first.** It is the live state of the work - what the robot
is plugged into right now, the task in progress with its measurements so far,
and the queue - and it is how work moves between Claude, Codex and Gemini when
one of them runs out of allowance. Update it before you stop, for any reason.

`CLAUDE.md` in this directory is the full set of working rules — the QC gates,
the branch layout, the promotion process, and the list of things that are true
and easy to get wrong on this robot. **Read it.** Everything in it applies here
too. What follows is the part that governs how you talk, repeated because it is
the rule most often broken.

## How to talk on this project — caveman style, everywhere

Every chat message is terse. Drop articles, filler and pleasantries. No
preamble, no "Great question", no "Let me". Answer first, explain only if
asked. Bullets over paragraphs. Fragments are fine. No end-of-turn summary
unless asked for one.

This applies to **all** output the user reads: explanations, teaching, error
and risk reports, disagreement, refusals, plans, recommendations, and review
verdicts. Terse but complete — the facts stay, the filler goes.

Right shape for a failure report:

> 3 of 12 tests fail. TypeError in `parse_fix()` line 44 — `sats` is None when
> no lock. Repro: `pytest -k rtk`.

Wrong shape:

> Unfortunately, it looks like the test suite is currently failing. Let me walk
> you through what I'm seeing here so we can figure out the best path forward.

Structured output that a program parses — the review JSON the claudex-loop
runner validates, for example — keeps its required schema. Caveman applies to
the prose inside those fields, not to the field names or the structure.

It does **not** apply to file contents. Code, comments, docstrings, commit
messages, PR bodies, READMEs, docs and config files are written in normal,
complete English — including this file.

## Permission

Do not ask permission to run a command or make an edit. Run it. Ask only when
there is a real decision with more than one valid answer and the answer changes
what gets built, or when the action leaves this machine — a push, a deploy, a
publish.

## Before you claim anything works

```bash
python3 tools/qc.py                        # cross-file contracts
python3 tools/port_qc.py --static-only .   # port gate
cd firmware && pio test -e native          # 261 host tests
pio run -e wifi                            # firmware still builds
```

All four must pass. A change that has not been through them is not finished,
and "it looks right" is not a result. If something fails, say so with the
output.
