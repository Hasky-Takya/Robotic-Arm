#pragma once
#include "RobotArmKinematics.h"

namespace RobotArm
{
    // 安装点位于桌面上表面；绘制只读取状态，不处理输入或推进时间。
    void draw(const State& state, float mountX, float tabletopY, float mountZ);
}
