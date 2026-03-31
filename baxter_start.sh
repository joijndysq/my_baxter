#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ensure_service_running() {
    local svc="$1"
    if service "$svc" status >/dev/null 2>&1; then
        echo "[ok] $svc is already running"
    else
        echo "[fix] starting $svc"
        service "$svc" start >/dev/null
        echo "[ok] $svc started"
    fi
}

if [[ ! -f "$WS_DIR/baxter.sh" ]]; then
    echo "[error] baxter.sh not found in $WS_DIR"
    exit 1
fi

ensure_service_running dbus
ensure_service_running avahi-daemon

if [[ -n "${1:-}" ]]; then
    echo "[run] source $WS_DIR/baxter.sh $1"
else
    echo "[run] source $WS_DIR/baxter.sh"
fi

source "$WS_DIR/baxter.sh" "${1:-}"
