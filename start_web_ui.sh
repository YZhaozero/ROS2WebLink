#!/bin/bash
# Tron Robot Web UI Startup Script
# This script starts the FastAPI web server for robot control

set -e

# Configuration
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8800}"
LOG_FILE="${LOG_FILE:-/tmp/webui.log}"

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "Tron Robot Web UI - 启动脚本"
echo "=========================================="
echo "监听地址: ${HOST}:${PORT}"
echo "日志文件: ${LOG_FILE}"
echo "工作目录: ${SCRIPT_DIR}"
echo "=========================================="

# Change to script directory
cd "${SCRIPT_DIR}"

# Check if port is already in use
if lsof -Pi :${PORT} -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo "⚠️  端口 ${PORT} 已被占用，尝试清理..."
    pkill -9 -f "uvicorn.*mapping_nav_server" || true
    sleep 2
fi

# Start the web server
echo "🚀 启动 Web 服务器..."
exec python3 -m uvicorn web_server.mapping_nav_server:app \
    --host "${HOST}" \
    --port "${PORT}" \
    --log-level info
