#pragma once

#include <array>
#include <cstddef>

namespace RobotArm
{
    // 角度单位为度；opening 是夹爪张开程度，0 为闭合，1 为完全张开。
    struct State
    {
        float baseAngle = 0;
        // 大臂以水平为零位；55 度保持原有初始外观，负角度可下降到桌面。
        float shoulderAngle = 55;
        float elbowAngle = -65;
        float wristRoll = 90;
        float opening = 1;

        bool operator==(const State&) const = default;
    };

    enum class Action
    {
        BaseLeft, BaseRight,
        ShoulderUp, ShoulderDown,
        ElbowUp, ElbowDown,
        WristLeft, WristRight,
        Close, Open,
        Count
    };

    // 回调只维护按住状态；连续运动由 update 按实际经过时间统一计算。
    struct Controls
    {
        std::array<bool, static_cast<std::size_t>(Action::Count)> held{};
        // 空格切换微调，便于将窄夹爪对准方块；不改变落体模拟速度。
        bool precision = false;

        void set(Action action, bool pressed);
        void clear();
        bool pressed(Action action) const;
    };

    inline constexpr float baseSpeed = 60;
    inline constexpr float pitchSpeed = 45;
    inline constexpr float wristSpeed = 90;
    inline constexpr float openingSpeed = 1;

    // 字母不区分大小写；方向键由窗口层的特殊键回调直接调用 Controls::set。
    bool setLetter(Controls& controls, unsigned char key, bool pressed);
    bool update(State& state, const Controls& controls, double deltaSeconds);

    // 夹指中心偏移为 0.09..0.24；夹持垫内表面距中心线为 offset - 0.09。
    float fingerOffset(float opening);
}
