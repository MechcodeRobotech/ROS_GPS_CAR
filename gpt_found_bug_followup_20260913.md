# Codex follow-up — 2026-09-13

The user authorized Codex to continue and Antigravity/Gemini to take over if Codex reaches its limit. The existing relay already runs in `codex agy` order with a four-round cap. Its dry run resolves Antigravity at `/home/mannaja/.gemini/bin/agy`, using `gemini-3.8-flash-high`. The relay switches on detected provider limits or failed/idle rounds; it is bounded, not an unlimited completion guarantee. No second hardware worker was launched.

## Remaining findings reproduced in the current tree

Production Python methods were executed with local fake publishers and a temporary configuration directory. No ROS nodes, robot commands, or live settings were used.

| Original finding | Current result |
| --- | --- |
| GPT-007: persisted invalid configuration | Still reproduced: API status 200; navigation reload raises ValueError for cruise_speed_mps="bad". |
| GPT-013: malformed navigation index | Still reproduced: goto index="bad" raises uncaught ValueError. |
| GPT-004: stale dead-reckoning position | Still reproduced: a 99-second-old pose is accepted. |
| GPT-014: missing position with require_fix=false | Still reproduced: navigation raises TypeError when unpacking None. |

Probe and exact output: `_work/codex_followup_20260913/recheck.py` and `recheck.log`.

Source inspection also confirms that `httpd.py` still accepts negative Content-Length and has no body-read timeout, and `tools/make_patch.py` still emits copy-only rollback operations for paths absent from the before snapshot. These correspond to historical GPT-020 and GPT-019. They were not fault-injected against the live server or existing patches.

No fixes were made. The recovered original 21-finding report remains at `gpt_found_bug_original_21_20260912.md`; the current eight-finding report was preserved. This follow-up is not a complete re-audit of all 21 findings.

## Hardware owner and continuation

The existing Codex relay owns OTA identity verification and passive hardware diagnostics. It already attempted OTA; repeating it merely because firmware_version remains 161 is not justified. Its latest recorded IMU zero-rate samples recovered within approximately 112 ms with the reset count unchanged; the source deliberately reports zero during the first 100 ms of each 10-second measurement window. Session liveliness timeouts remain documented and their underlying cause is unproven.

Antigravity should read the newest HANDOFF.md, finish the passive hardware investigation, and preserve exact measurements. Do not command motion, repeat an upload without evidence, push, install packages, change networking, or start the deferred Docker rebuild. The original no-fixes instruction continues to apply to this follow-up audit unless the user separately authorizes corrections.
