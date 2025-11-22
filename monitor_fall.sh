#!/bin/bash
# 机器人摔倒状态实时监控脚本
# 使用方法: ./monitor_fall.sh

echo "🤖 机器人摔倒监控启动..."
echo "监控地址: http://localhost:8800"
echo "按 Ctrl+C 退出"
echo "-----------------------------------"

AUTO_RECOVER=false  # 是否自动触发恢复（默认关闭，安全考虑）

while true; do
    # 获取机器人状态
    STATUS=$(curl -s http://localhost:8800/api/robot/status 2>/dev/null | jq -r '.robot_info.status // "UNKNOWN"')
    BATTERY=$(curl -s http://localhost:8800/api/robot/status 2>/dev/null | jq -r '.robot_info.battery // "?"')
    
    # 获取当前时间
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
    
    # 根据状态显示不同信息
    case "$STATUS" in
        "ERROR_FALLOVER")
            echo -e "\033[1;31m⚠️  [$TIMESTAMP] 警告：机器人摔倒了！电量: ${BATTERY}%\033[0m"
            
            if [ "$AUTO_RECOVER" = true ]; then
                echo "   → 自动触发恢复..."
                curl -s -X POST http://localhost:8800/api/robot/mode/recover > /dev/null
            else
                echo "   → 请手动点击'倒地爬起'按钮或运行: curl -X POST http://localhost:8800/api/robot/mode/recover"
            fi
            
            # 发送系统通知（如果支持）
            if command -v notify-send &> /dev/null; then
                notify-send "机器人摔倒警告" "机器人已摔倒，请立即处理！" -u critical
            fi
            ;;
            
        "RECOVER")
            echo -e "\033[1;33m🔄 [$TIMESTAMP] 机器人正在恢复中... 电量: ${BATTERY}%\033[0m"
            ;;
            
        "ERROR_RECOVER")
            echo -e "\033[1;31m❌ [$TIMESTAMP] 错误：恢复失败！需要人工干预！电量: ${BATTERY}%\033[0m"
            
            if command -v notify-send &> /dev/null; then
                notify-send "机器人恢复失败" "恢复失败，需要人工检查机器人状态！" -u critical
            fi
            ;;
            
        "WALK")
            echo -e "\033[1;32m✅ [$TIMESTAMP] 机器人正常行走 电量: ${BATTERY}%\033[0m"
            ;;
            
        "STAND")
            echo -e "\033[1;32m✅ [$TIMESTAMP] 机器人站立中 电量: ${BATTERY}%\033[0m"
            ;;
            
        "SIT")
            echo -e "\033[1;32m✅ [$TIMESTAMP] 机器人坐下中 电量: ${BATTERY}%\033[0m"
            ;;
            
        "STAIR")
            echo -e "\033[1;34m🚶 [$TIMESTAMP] 机器人楼梯模式 电量: ${BATTERY}%\033[0m"
            ;;
            
        "UNKNOWN")
            echo -e "\033[1;90m⚠️  [$TIMESTAMP] 无法获取机器人状态（WebSocket可能未连接）\033[0m"
            ;;
            
        *)
            echo -e "\033[1;36mℹ️  [$TIMESTAMP] 机器人状态: $STATUS 电量: ${BATTERY}%\033[0m"
            ;;
    esac
    
    # 每2秒检查一次
    sleep 2
done

