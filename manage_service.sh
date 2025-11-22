#!/bin/bash

# ROS2WebLink 服务管理脚本
# 使用方法: ./manage_service.sh [start|stop|restart|status|logs|enable|disable]

SERVICE_NAME="ros2weblink"
COLOR_GREEN='\033[0;32m'
COLOR_YELLOW='\033[1;33m'
COLOR_RED='\033[0;31m'
COLOR_BLUE='\033[0;34m'
COLOR_RESET='\033[0m'

function print_usage() {
    echo -e "${COLOR_BLUE}╔════════════════════════════════════════════════════════════╗${COLOR_RESET}"
    echo -e "${COLOR_BLUE}║       ROS2WebLink 服务管理脚本                            ║${COLOR_RESET}"
    echo -e "${COLOR_BLUE}╚════════════════════════════════════════════════════════════╝${COLOR_RESET}"
    echo ""
    echo "使用方法: $0 [命令]"
    echo ""
    echo "可用命令:"
    echo "  start       - 启动服务"
    echo "  stop        - 停止服务"
    echo "  restart     - 重启服务"
    echo "  status      - 查看服务状态"
    echo "  logs        - 查看实时日志"
    echo "  logs-last   - 查看最近 50 行日志"
    echo "  enable      - 启用开机自启"
    echo "  disable     - 禁用开机自启"
    echo "  health      - 健康检查"
    echo ""
    echo "示例:"
    echo "  $0 status      # 查看服务状态"
    echo "  $0 restart     # 重启服务"
    echo "  $0 logs        # 实时查看日志"
    echo ""
}

function check_service_enabled() {
    if systemctl is-enabled $SERVICE_NAME &>/dev/null; then
        echo -e "${COLOR_GREEN}✓ 开机自启: 已启用${COLOR_RESET}"
    else
        echo -e "${COLOR_YELLOW}! 开机自启: 未启用${COLOR_RESET}"
    fi
}

function do_start() {
    echo -e "${COLOR_BLUE}🚀 正在启动 ROS2WebLink 服务...${COLOR_RESET}"
    sudo systemctl start $SERVICE_NAME
    sleep 3
    do_status
}

function do_stop() {
    echo -e "${COLOR_YELLOW}🛑 正在停止 ROS2WebLink 服务...${COLOR_RESET}"
    sudo systemctl stop $SERVICE_NAME
    echo -e "${COLOR_GREEN}✓ 服务已停止${COLOR_RESET}"
}

function do_restart() {
    echo -e "${COLOR_BLUE}🔄 正在重启 ROS2WebLink 服务...${COLOR_RESET}"
    sudo systemctl restart $SERVICE_NAME
    sleep 3
    do_status
}

function do_status() {
    echo -e "${COLOR_BLUE}════════════════════════════════════════════════════════════${COLOR_RESET}"
    systemctl status $SERVICE_NAME --no-pager -l 2>/dev/null || echo "需要 sudo 权限查看详细状态"
    echo -e "${COLOR_BLUE}════════════════════════════════════════════════════════════${COLOR_RESET}"
    check_service_enabled
    
    # 检查端口
    if lsof -ti:8800 &>/dev/null; then
        echo -e "${COLOR_GREEN}✓ 端口 8800: 正在监听${COLOR_RESET}"
    else
        echo -e "${COLOR_RED}✗ 端口 8800: 未监听${COLOR_RESET}"
    fi
}

function do_logs() {
    echo -e "${COLOR_BLUE}📋 实时日志（Ctrl+C 退出）:${COLOR_RESET}"
    echo ""
    sudo journalctl -u $SERVICE_NAME -f --no-pager
}

function do_logs_last() {
    echo -e "${COLOR_BLUE}📋 最近 50 行日志:${COLOR_RESET}"
    echo ""
    sudo journalctl -u $SERVICE_NAME -n 50 --no-pager
}

function do_enable() {
    echo -e "${COLOR_BLUE}🔧 启用开机自启...${COLOR_RESET}"
    sudo systemctl enable $SERVICE_NAME
    echo -e "${COLOR_GREEN}✓ 已启用开机自启${COLOR_RESET}"
}

function do_disable() {
    echo -e "${COLOR_YELLOW}🔧 禁用开机自启...${COLOR_RESET}"
    sudo systemctl disable $SERVICE_NAME
    echo -e "${COLOR_YELLOW}✓ 已禁用开机自启${COLOR_RESET}"
}

function do_health() {
    echo -e "${COLOR_BLUE}🏥 健康检查...${COLOR_RESET}"
    echo ""
    
    # 检查服务状态
    if systemctl is-active $SERVICE_NAME &>/dev/null; then
        echo -e "${COLOR_GREEN}✓ 服务状态: 运行中${COLOR_RESET}"
    else
        echo -e "${COLOR_RED}✗ 服务状态: 未运行${COLOR_RESET}"
        return
    fi
    
    # 检查端口
    if lsof -ti:8800 &>/dev/null; then
        echo -e "${COLOR_GREEN}✓ 端口 8800: 正在监听${COLOR_RESET}"
    else
        echo -e "${COLOR_RED}✗ 端口 8800: 未监听${COLOR_RESET}"
        return
    fi
    
    # 检查 HTTP 接口
    echo -n "  HTTP 健康检查: "
    response=$(curl -s http://localhost:8800/healthz 2>/dev/null)
    if echo "$response" | grep -q '"status":"ok"'; then
        echo -e "${COLOR_GREEN}✓ OK${COLOR_RESET}"
    else
        echo -e "${COLOR_RED}✗ 失败${COLOR_RESET}"
        return
    fi
    
    echo ""
    echo -e "${COLOR_GREEN}✓ 所有检查通过！${COLOR_RESET}"
    echo ""
    echo "访问地址:"
    echo "  - Web UI:      http://192.168.100.88:8800"
    echo "  - API 文档:    http://192.168.100.88:8800/docs"
    echo "  - 健康检查:    http://192.168.100.88:8800/healthz"
}

# 主逻辑
case "${1:-}" in
    start)
        do_start
        ;;
    stop)
        do_stop
        ;;
    restart)
        do_restart
        ;;
    status)
        do_status
        ;;
    logs)
        do_logs
        ;;
    logs-last)
        do_logs_last
        ;;
    enable)
        do_enable
        ;;
    disable)
        do_disable
        ;;
    health)
        do_health
        ;;
    *)
        print_usage
        exit 1
        ;;
esac

