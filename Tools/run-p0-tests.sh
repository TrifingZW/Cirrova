#!/usr/bin/env bash
# 运行 Cirrova P0 自动化验证（Git Bash / MSYS2）。
# 用法：./run-p0-tests.sh [all|T0|T0,T1] [输出目录]
# 环境变量 UE 可覆盖引擎安装目录。

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
UE="${UE:-C:/Program Files/Epic Games/UE_5.8}"
SCENARIO="${1:-all}"
OUT_DIR="${2:-$PROJECT_ROOT/Saved/CirrovaTests/$(date +%Y%m%d-%H%M%S)}"
EDITOR="$UE/Engine/Binaries/Win64/UnrealEditor-Cmd.exe"
PROJECT="$PROJECT_ROOT/Vehicle.uproject"

if [ ! -x "$EDITOR" ] && [ ! -f "$EDITOR" ]; then
    echo "未找到 UnrealEditor-Cmd.exe：$EDITOR" >&2
    exit 2
fi

echo "Cirrova P0 自动化验证"
echo "  场景：$SCENARIO"
echo "  输出：$OUT_DIR"
echo

MSYS_NO_PATHCONV=1 "$EDITOR" "$PROJECT" \
    -game -nullrhi -unattended -nosplash -log \
    -Deterministic -FPS=60 \
    -map=/Engine/Maps/Entry \
    "-CirrovaTest=$SCENARIO" \
    "-CirrovaTestOut=$OUT_DIR" \
    -CirrovaTestExit=1

EXIT_CODE=$?

if [ -f "$OUT_DIR/summary.txt" ]; then
    cat "$OUT_DIR/summary.txt"
else
    echo "未找到 summary.txt：$OUT_DIR/summary.txt" >&2
fi

exit "$EXIT_CODE"
