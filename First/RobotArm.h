#pragma once

namespace RobotArm
{
    // 文档采用 Y 轴向上；臂长是相邻两个关节中心之间的距离。
    inline constexpr float upperArmLength = 2.0f;
    inline constexpr float forearmLength = 1.5f;

    // 安装点位于桌面上表面；第一阶段只绘制固定姿态，不处理关节输入。
    void draw(float mountX, float tabletopY, float mountZ);
}
