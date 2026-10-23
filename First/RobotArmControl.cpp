#include "RobotArmControl.h"

#include <algorithm>
#include <cmath>

namespace
{
    int direction(const RobotArm::Controls& controls,
                  RobotArm::Action positive, RobotArm::Action negative)
    {
        // 相反方向同时按住时抵消，不依赖键盘回调的先后顺序。
        return static_cast<int>(controls.pressed(positive))
             - static_cast<int>(controls.pressed(negative));
    }

    float advanceAngle(float angle, double speed, double deltaSeconds)
    {
        double step = speed * deltaSeconds;
        if (!std::isfinite(step))
        {
            // 极大的有限时间也不能让乘法溢出污染状态；先去掉完整转数。
            step = speed * std::fmod(deltaSeconds, 360.0 / std::abs(speed));
        }
        double wrapped = std::fmod(static_cast<double>(angle) + step, 360.0);
        if (wrapped < 0.0)
            wrapped += 360.0;

        const float result = static_cast<float>(wrapped);
        // 转为 float 时，极接近 360 的结果可能向上舍入到 360。
        return result >= 360.0f ? 0.0f : result;
    }

    float advanceClamped(float value, double speed, double deltaSeconds,
                         double minimum, double maximum)
    {
        // 先以 double 计算再限幅；即使极端时间产生无穷大，也会落到端点。
        return static_cast<float>(std::clamp(
            static_cast<double>(value) + speed * deltaSeconds, minimum, maximum));
    }
}

void RobotArm::Controls::set(Action action, bool isPressed)
{
    const auto index = static_cast<std::size_t>(action);
    if (index < held.size())
        held[index] = isPressed;
}

void RobotArm::Controls::clear()
{
    held.fill(false);
}

bool RobotArm::Controls::pressed(Action action) const
{
    const auto index = static_cast<std::size_t>(action);
    return index < held.size() && held[index];
}

bool RobotArm::setLetter(Controls& controls, unsigned char key, bool isPressed)
{
    Action action;
    switch (key)
    {
    case 'a': case 'A': action = Action::BaseLeft; break;
    case 'd': case 'D': action = Action::BaseRight; break;
    case 'w': case 'W': action = Action::ShoulderUp; break;
    case 's': case 'S': action = Action::ShoulderDown; break;
    case 'q': case 'Q': action = Action::WristLeft; break;
    case 'e': case 'E': action = Action::WristRight; break;
    case 'f': case 'F': action = Action::Close; break;
    case 'g': case 'G': action = Action::Open; break;
    default: return false;
    }
    controls.set(action, isPressed);
    return true;
}

bool RobotArm::update(State& state, const Controls& controls, double deltaSeconds)
{
    if (!std::isfinite(deltaSeconds) || deltaSeconds <= 0.0)
        return false;

    // 夹持居中容差较小，关节使用 1/50 速度便于手动对准；夹爪保留 1/10 速度。
    const double openingSeconds = deltaSeconds * (controls.precision ? 0.1 : 1.0);
    if (controls.precision) deltaSeconds *= 0.02;
    const State before = state;
    const int base = direction(controls, Action::BaseLeft, Action::BaseRight);
    const int shoulder = direction(controls, Action::ShoulderUp, Action::ShoulderDown);
    const int elbow = direction(controls, Action::ElbowUp, Action::ElbowDown);
    const int wrist = direction(controls, Action::WristLeft, Action::WristRight);
    const int opening = direction(controls, Action::Open, Action::Close);

    // 无输入或相反输入时跳过该轴，避免静止状态因归一化而变化。
    if (base != 0)
        state.baseAngle = advanceAngle(state.baseAngle, base * baseSpeed, deltaSeconds);
    if (shoulder != 0)
        state.shoulderAngle = advanceClamped(state.shoulderAngle, shoulder * pitchSpeed,
                                            deltaSeconds, -90.0, 90.0);
    if (elbow != 0)
        state.elbowAngle = advanceClamped(state.elbowAngle, elbow * pitchSpeed,
                                         deltaSeconds, -90.0, 90.0);
    if (wrist != 0)
        state.wristRoll = advanceAngle(state.wristRoll, wrist * wristSpeed, deltaSeconds);
    if (opening != 0)
        state.opening = advanceClamped(state.opening, opening * openingSpeed,
                                      openingSeconds, 0.0, 1.0);

    return !(state == before);
}

float RobotArm::fingerOffset(float opening)
{
    // 非数输入按闭合处理；正负无穷分别限幅到完全张开和闭合。
    const float clamped = std::isnan(opening) ? 0.0f : std::clamp(opening, 0.0f, 1.0f);
    return 0.09f + (0.24f - 0.09f) * clamped;
}
