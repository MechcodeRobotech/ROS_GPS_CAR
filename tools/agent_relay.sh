#!/usr/bin/env bash
# Keep the work going across Claude Code, Codex and Gemini.
#
# Each agent's allowance runs out on its own schedule, and the work should not
# stop when one does. This runs one agent at a time, non-interactively, on the
# same instruction: read HANDOFF.md and continue the first unfinished item.
#
#   an agent hits its usage limit   -> it is skipped for RELAY_COOLDOWN_S and
#                                      the next one in RELAY_ORDER takes over
#   an agent fails or changes nothing -> the next one takes over
#   an agent finishes a round normally -> it runs again
#   HANDOFF.md contains RELAY_DONE on a line of its own -> stop
#
# HANDOFF.md is the only thing carried between agents. Every round is a fresh
# session with no memory of the last, which is the point: whatever an agent
# learned and did not write down is lost, so the prompt insists it write it.
#
# Usage:
#   tools/agent_relay.sh --dry-run          which agents are installed, what would run
#   tools/agent_relay.sh                    loop until RELAY_DONE or RELAY_MAX_ROUNDS
#   tools/agent_relay.sh --once codex       one round with one agent, by hand
#   tools/agent_relay.sh --prompt "..."     a different instruction for every round
#
# Settings, all optional, as environment variables:
#   RELAY_ORDER            agents in priority order      default "claude codex agy"
#   RELAY_MAX_ROUNDS       stop after this many rounds   default 12
#   RELAY_ROUND_TIMEOUT_S  longest a single round may run  default 7200
#   RELAY_COOLDOWN_S       skip a limited agent this long  default 3600
#   RELAY_MAX_WAIT_S       longest wait when every agent is limited  default 18000
#   CLAUDE_MODEL, CODEX_MODEL   empty = each tool's own default
#   AGY_MODEL              Antigravity's model   default "gemini-3.8-flash-high"
#                          (~/.gemini/bin/agy models lists what is available)
#   GEMINI_CMD, GEMINI_MODEL   the plain Gemini CLI, if it is ever installed -
#                          add "gemini" to RELAY_ORDER to use it
#
# SAFETY. Every agent here runs with its permission prompts switched off,
# because nobody is watching to answer them. HANDOFF.md forbids commanding the
# robot to move, pushing, publishing and installing system packages, and the
# prompt below repeats it - but that is an instruction, not a lock. Do not start
# this with the battery connected and the wheels on the ground.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1

ORDER=(${RELAY_ORDER:-claude codex agy})
MAX_ROUNDS="${RELAY_MAX_ROUNDS:-12}"
ROUND_TIMEOUT_S="${RELAY_ROUND_TIMEOUT_S:-7200}"
COOLDOWN_S="${RELAY_COOLDOWN_S:-3600}"
MAX_WAIT_S="${RELAY_MAX_WAIT_S:-18000}"
GEMINI_CMD="${GEMINI_CMD:-gemini}"

RELAY_DIR="$ROOT/_relay"
LOG_DIR="$RELAY_DIR/logs"
STATE="$RELAY_DIR/limited"          # lines of: <agent> <epoch the cooldown ends>
HANDOFF="$ROOT/HANDOFF.md"
mkdir -p "$LOG_DIR"
# STATE is created only when an agent is first marked limited. An empty
# placeholder would trip the empty-file check in tools/port_qc.py, which
# exists because 162 CAD files once went out as zero bytes.

PROMPT="You are one of several agents (Claude Code, Codex, Gemini) taking turns on \
this repository; the previous one may have stopped in the middle of a task. Read \
HANDOFF.md first, then CLAUDE.md, and follow both. Continue the first unfinished item \
in the Queue in HANDOFF.md. Verify with the gates in CLAUDE.md before claiming \
anything works. Before you stop for any reason - finished, stuck, or close to your \
own usage limit - update HANDOFF.md: what you did, the measured result with the \
command that produced it, and the exact next step. When every item in the Queue is \
finished and verified, add a line containing only RELAY_DONE under Status in \
HANDOFF.md. Never command the robot to move. Never push or publish. Never install \
system packages."

# A usage-limit failure, as each of the three tools words it. Matched only
# against the END of a round's output, and the broad form only when the agent
# also exited non-zero - an agent that merely DISCUSSES rate limits while doing
# its job must not be mistaken for one that has hit its own.
LIMIT_STRONG='usage limit reached|hit your usage limit|reached your usage limit|quota exceeded|resource_exhausted|limit reached\|'
LIMIT_BROAD='usage limit|rate[ _-]?limit|quota|resource[_ ]exhausted|(^|[^0-9])429([^0-9]|$)|too many requests|limit reached|out of credits|weekly limit'

# ------------------------------------------------------------------ helpers
agent_bin() {
  case "$1" in
    claude) command -v claude ;;
    codex)  command -v codex ;;
    gemini) command -v "$GEMINI_CMD" ;;
    # Antigravity installs its CLI in ~/.gemini/bin and leaves PATH alone
    # unless `agy install` is run, so look there as well.
    agy)    command -v agy 2>/dev/null \
              || { [ -x "$HOME/.gemini/bin/agy" ] && echo "$HOME/.gemini/bin/agy"; } ;;
    *)      return 1 ;;
  esac
}

# The exact command for one round, one argument per line, so a dry run shows
# precisely what would execute and the real run executes exactly that.
agent_argv() {
  local agent="$1" prompt="$2"
  case "$agent" in
    claude)
      printf '%s\n' claude -p --dangerously-skip-permissions
      [ -n "${CLAUDE_MODEL:-}" ] && printf '%s\n' --model "$CLAUDE_MODEL"
      ;;
    codex)
      printf '%s\n' codex exec --dangerously-bypass-approvals-and-sandbox --skip-git-repo-check
      [ -n "${CODEX_MODEL:-}" ] && printf '%s\n' -m "$CODEX_MODEL"
      ;;
    gemini)
      printf '%s\n' "$GEMINI_CMD" --yolo
      [ -n "${GEMINI_MODEL:-}" ] && printf '%s\n' -m "$GEMINI_MODEL"
      printf '%s\n' -p
      ;;
    agy)
      # --print-timeout defaults to five minutes, which would cut a real round
      # off in the middle of its work; match it to the relay's own limit.
      printf '%s\n' "$(agent_bin agy)" --dangerously-skip-permissions \
        --print-timeout "${ROUND_TIMEOUT_S}s" --model "${AGY_MODEL:-gemini-3.8-flash-high}" -p
      ;;
  esac
  # One line: argv is read back one entry per line, so a newline inside a
  # --prompt would otherwise split it into several arguments.
  printf '%s\n' "${prompt//$'\n'/ }"
}

limited_until() {
  [ -f "$STATE" ] || { echo 0; return; }
  awk -v a="$1" '$1 == a { t = $2 } END { print (t ? t : 0) }' "$STATE"
}

mark_limited() {
  local agent="$1" until=$(( $(date +%s) + COOLDOWN_S ))
  grep -v "^$agent " "$STATE" > "$STATE.tmp" 2>/dev/null || true
  echo "$agent $until" >> "$STATE.tmp"
  mv "$STATE.tmp" "$STATE"
}

available() {
  local agent="$1"
  agent_bin "$agent" >/dev/null 2>&1 || return 1
  [ "$(limited_until "$agent")" -le "$(date +%s)" ]
}

is_done() { grep -qx 'RELAY_DONE' "$HANDOFF" 2>/dev/null; }

fingerprint() { md5sum "$HANDOFF" 2>/dev/null | cut -d' ' -f1; }

# Run one round. Prints one of: ok, limited, failed, idle.
run_round() {
  local agent="$1" prompt="$2"
  local log="$LOG_DIR/$(date +%Y%m%d-%H%M%S)-$agent.log"
  local before after rc
  local -a argv
  mapfile -t argv < <(agent_argv "$agent" "$prompt")

  before=$(fingerprint)
  echo ">>> $(date '+%F %T')  round with $agent  (log: ${log#$ROOT/})" >&2
  timeout "$ROUND_TIMEOUT_S" "${argv[@]}" </dev/null 2>&1 | tee "$log" >&2
  rc=${PIPESTATUS[0]}
  after=$(fingerprint)

  if tail -n 15 "$log" | grep -Eiq "$LIMIT_STRONG" \
     || { [ "$rc" -ne 0 ] && tail -n 80 "$log" | grep -Eiq "$LIMIT_BROAD"; }; then
    echo limited
  elif [ "$rc" -ne 0 ]; then
    echo failed
  elif [ "$before" = "$after" ]; then
    # Exited cleanly and left no trace in HANDOFF.md. Either it did nothing or
    # it did something and did not write it down, and the next agent cannot
    # tell which - both are a round that did not move the work forward.
    echo idle
  else
    echo ok
  fi
}

# ------------------------------------------------------------------- modes
if [ "${1:-}" = "--prompt" ]; then
  PROMPT="${2:?--prompt needs text}"; shift 2
fi

if [ "${1:-}" = "--dry-run" ]; then
  now=$(date +%s)
  echo "order: ${ORDER[*]}    max rounds: $MAX_ROUNDS    cooldown: ${COOLDOWN_S}s"
  for agent in "${ORDER[@]}"; do
    bin=$(agent_bin "$agent" 2>/dev/null || true)
    until=$(limited_until "$agent")
    if [ -z "$bin" ]; then
      printf '  %-7s NOT INSTALLED (skipped)\n' "$agent"
      continue
    fi
    if [ "$until" -gt "$now" ]; then
      printf '  %-7s %s  - limited for another %ss\n' "$agent" "$bin" $(( until - now ))
    else
      printf '  %-7s %s  - ready\n' "$agent" "$bin"
    fi
    mapfile -t argv < <(agent_argv "$agent" "<prompt>")
    printf '          would run: %s\n' "${argv[*]}"
  done
  is_done && echo "HANDOFF.md already says RELAY_DONE - a real run would stop at once."
  exit 0
fi

if [ "${1:-}" = "--once" ]; then
  agent="${2:?--once needs an agent name}"
  agent_bin "$agent" >/dev/null 2>&1 || { echo "$agent is not installed" >&2; exit 2; }
  result=$(run_round "$agent" "$PROMPT")
  [ "$result" = limited ] && mark_limited "$agent"
  echo "=== $agent: $result"
  exit 0
fi

# ------------------------------------------------------------------- relay
round=0
idx=0
while [ "$round" -lt "$MAX_ROUNDS" ]; do
  if is_done; then
    echo "=== HANDOFF.md says RELAY_DONE after $round round(s)."
    exit 0
  fi

  # The current agent if it can run, otherwise the next one in order that can.
  pick=""
  for (( k = 0; k < ${#ORDER[@]}; k++ )); do
    cand="${ORDER[$(( (idx + k) % ${#ORDER[@]} ))]}"
    if available "$cand"; then pick="$cand"; idx=$(( (idx + k) % ${#ORDER[@]} )); break; fi
  done

  if [ -z "$pick" ]; then
    now=$(date +%s); soonest=0
    for agent in "${ORDER[@]}"; do
      agent_bin "$agent" >/dev/null 2>&1 || continue
      u=$(limited_until "$agent")
      if [ "$soonest" -eq 0 ] || [ "$u" -lt "$soonest" ]; then soonest=$u; fi
    done
    if [ "$soonest" -eq 0 ]; then
      echo "=== no agent is installed - nothing can run" >&2; exit 2
    fi
    wait_s=$(( soonest - now ))
    if [ "$wait_s" -gt "$MAX_WAIT_S" ]; then
      echo "=== every agent is limited; the first frees up in ${wait_s}s, longer than RELAY_MAX_WAIT_S" >&2
      exit 3
    fi
    echo "=== every agent is limited - waiting ${wait_s}s for the first to free up" >&2
    sleep "$wait_s"
    continue
  fi

  round=$(( round + 1 ))
  result=$(run_round "$pick" "$PROMPT")
  echo "=== round $round: $pick -> $result"
  case "$result" in
    ok)      ;;                                           # same agent again
    limited) mark_limited "$pick"; idx=$(( idx + 1 )) ;;
    *)       idx=$(( idx + 1 )) ;;                        # failed or idle
  esac
done

echo "=== stopped after $MAX_ROUNDS rounds without RELAY_DONE - read HANDOFF.md for where it got to."
exit 4
