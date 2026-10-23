#pragma once
#include "RobotArmControl.h"

namespace RobotArm
{
    // 文档采用 Y 轴向上；臂长是相邻两个关节中心之间的距离。
    inline constexpr float upperArmLength = 2.0f;
    inline constexpr float forearmLength = 1.5f;

    // 安装点位于桌面上表面；绘制只读取状态，不处理输入或推进时间。
    void draw(const State& state, float mountX, float tabletopY, float mountZ);
}
