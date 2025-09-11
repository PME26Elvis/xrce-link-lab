#!/usr/bin/env bash
set -euo pipefail

DEV="lo"

usage() {
  cat <<'EOF'
Usage:
  netem_profiles.sh apply <profile>
  netem_profiles.sh clear

Profiles:
  delay100ms_loss1       # 100ms latency + 1% loss
  delay50ms_jitter20ms   # 50ms avg delay with 20ms jitter
EOF
}

require_root() {
  if [[ $EUID -ne 0 ]]; then
    echo "This must run as root (sudo)."
    exit 1
  fi
}

apply_profile() {
  local profile="$1"
  case "${profile}" in
    delay100ms_loss1)
      tc qdisc add dev "${DEV}" root netem delay 100ms loss 1%
      ;;
    delay50ms_jitter20ms)
      tc qdisc add dev "${DEV}" root netem delay 50ms 20ms
      ;;
    *)
      echo "Unknown profile: ${profile}"
      usage
      exit 1
      ;;
  esac
  echo "Applied netem profile '${profile}' on ${DEV}."
}

clear_qdisc() {
  tc qdisc del dev "${DEV}" root || true
  echo "Cleared netem on ${DEV}."
}

main() {
  if [[ $# -lt 1 ]]; then usage; exit 1; fi
  case "$1" in
    apply)
      [[ $# -eq 2 ]] || { usage; exit 1; }
      require_root
      clear_qdisc
      apply_profile "$2"
      ;;
    clear)
      require_root
      clear_qdisc
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
