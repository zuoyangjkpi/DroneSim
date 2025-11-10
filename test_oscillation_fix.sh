#!/bin/bash

# 测试无人机摇晃问题修复效果的脚本
# 需要在drone环境下运行

echo "🔧 测试无人机摇晃问题修复效果"
echo "==============================="

# 设置环境
export PYTHONPATH="/usr/lib/python3/dist-packages:$PYTHONPATH"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ 环境设置完成"

# 检查关键参数修改
echo ""
echo "📋 检查修复的关键参数："

echo "1. NMPC配置参数："
echo "   - OPTIMAL_TRACKING_DISTANCE: $(python3 -c "from src.drone_nmpc_tracker.drone_nmpc_tracker.config import nmpc_config; print(nmpc_config.OPTIMAL_TRACKING_DISTANCE)")"
echo "   - STEP_SIZE: $(python3 -c "from src.drone_nmpc_tracker.drone_nmpc_tracker.config import nmpc_config; print(nmpc_config.STEP_SIZE)")"
echo "   - TARGET_POSITION_SMOOTHING: $(python3 -c "from src.drone_nmpc_tracker.drone_nmpc_tracker.config import nmpc_config; print(nmpc_config.TARGET_POSITION_SMOOTHING)")"

echo ""
GUIDANCE_CFG="src/drone_guidance_controllers/config/controllers.yaml"
echo "2. 低级控制器PID参数 (从YAML文件):"
echo "   - 位置控制kp_xy: $(grep 'kp_xy:' $GUIDANCE_CFG | awk '{print $2}')"
echo "   - 位置控制kd_xy: $(grep 'kd_xy:' $GUIDANCE_CFG | awk '{print $2}')"
echo "   - 姿态控制kp_yaw: $(grep 'kp_yaw:' $GUIDANCE_CFG | awk '{print $2}')"

echo ""
echo "3. 投影模型置信度阈值："
echo "   - 检测置信度阈值已从0.8降低到0.5 (在Projector.cpp中)"

echo ""
echo "🚀 修复摘要："
echo "================"
echo "1. 降低了NMPC跟踪的轨道角速度（从0.15到0.08 rad/s）"
echo "2. 增加了目标跟踪距离（从3.0到4.0米）"
echo "3. 增强了目标位置平滑（从0.7到0.85）"
echo "4. 降低了PID控制器的比例增益，增加了微分增益"
echo "5. 增加了NMPC成本函数中的速度跟踪权重"
echo "6. 降低了优化器步长（从0.05到0.02）"
echo "7. 添加了基于距离的阻尼因子来防止振荡"
echo "8. 降低了投影模型的检测置信度阈值"

echo ""
echo "💡 建议测试步骤："
echo "=================="
echo "1. 运行: ./comprehensive_test_suite.sh"
echo "2. 选择选项5: 全集成测试"
echo "3. 观察无人机行为："
echo "   - 起飞时应该更加平稳"
echo "   - 检测到人后，跟踪应该更加稳定"
echo "   - 圆形轨道跟踪应该更加平滑，减少左右摇晃"
echo "   - 当人移动时，无人机应该平稳跟随"

echo ""
echo "🎯 预期改善效果："
echo "=================="
echo "- 减少无人机在检测到人后的左右摇晃"
echo "- 更平稳的圆形轨道跟踪"
echo "- 更好的人体检测稳定性"
echo "- 减少NMPC优化引起的抖动"
echo "- 改善低级控制器的响应特性"

echo ""
echo "⚠️ 如果仍有问题，可以进一步调整："
echo "=================================="
echo "1. 增加W_POSITION在Z轴的权重以增强高度保持"
echo "2. 增加TARGET_POSITION_SMOOTHING（如0.9）"
echo "3. 进一步降低PID kp_xy增益（如1.0）"
echo "4. 增加NMPC的W_SMOOTH_TRACKING权重"

echo ""
echo "✅ 修复完成! 现在可以测试系统了"
