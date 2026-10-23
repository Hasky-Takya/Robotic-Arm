#include "../First/RobotArmControl.h"

#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <limits>

namespace
{
    using RobotArm::Action;
    using RobotArm::Controls;
    using RobotArm::State;

    // 不使用 assert，确保 Release / NDEBUG 下检查仍然生效。
    int failures = 0;

    void check(bool condition, const char* message)
    {
        if (!condition)
        {
            ++failures;
            std::cerr << "FAIL: " << message << '\n';
        }
    }

    bool near(float actual, float expected, float tolerance = 0.0001f)
    {
        return std::isfinite(actual) && std::abs(actual - expected) <= tolerance;
    }

    bool nearState(const State& actual, const State& expected)
    {
        return near(actual.baseAngle, expected.baseAngle)
            && near(actual.shoulderAngle, expected.shoulderAngle)
            && near(actual.elbowAngle, expected.elbowAngle)
            && near(actual.wristRoll, expected.wristRoll)
            && near(actual.opening, expected.opening);
    }

    void testDefaultsAndEquality()
    {
        const State state;
        check(state.baseAngle == 0 && state.shoulderAngle == 55
              && state.elbowAngle == -65 && state.wristRoll == 90
              && state.opening == 1, "default pose");
        check(state == State{}, "equal states compare equal");
        constexpr float State::* fields[] = {
            &State::baseAngle, &State::shoulderAngle, &State::elbowAngle,
            &State::wristRoll, &State::opening
        };
        for (auto field : fields)
        {
            State other = state;
            other.*field += 0.25f;
            check(!(state == other), "equality compares every field");
        }
        check(RobotArm::baseSpeed == 60 && RobotArm::pitchSpeed == 45
              && RobotArm::wristSpeed == 90 && RobotArm::openingSpeed == 1,
              "public speed constants");
        Controls controls;
        for (bool held : controls.held)
            check(!held, "controls default to released");
        State idle;
        check(!RobotArm::update(idle, controls, 0.1) && idle == state,
              "idle update preserves default pose exactly");
    }

    void testEachDirection()
    {
        struct DirectionCase
        {
            Action action;
            float State::* field;
            float displacement;
        };
        const DirectionCase cases[] = {
            {Action::BaseLeft, &State::baseAngle, 6},
            {Action::BaseRight, &State::baseAngle, -6},
            {Action::ShoulderUp, &State::shoulderAngle, 4.5f},
            {Action::ShoulderDown, &State::shoulderAngle, -4.5f},
            {Action::ElbowUp, &State::elbowAngle, 4.5f},
            {Action::ElbowDown, &State::elbowAngle, -4.5f},
            {Action::WristLeft, &State::wristRoll, 9},
            {Action::WristRight, &State::wristRoll, -9},
            {Action::Close, &State::opening, -0.1f},
            {Action::Open, &State::opening, 0.1f}
        };
        for (const auto& item : cases)
        {
            State state{180, 0, 0, 180, 0.5f};
            State expected = state;
            expected.*(item.field) += item.displacement;
            Controls controls;
            controls.set(item.action, true);
            check(controls.pressed(item.action), "set marks action pressed");
            check(RobotArm::update(state, controls, 0.1), "each direction changes state");
            check(nearState(state, expected), "each direction has correct sign and rate");
        }
    }

    void testHeldReleaseAndClear()
    {
        State state;
        Controls controls;
        controls.set(Action::BaseLeft, true);
        controls.set(Action::BaseLeft, true);
        for (int frame = 0; frame < 10; ++frame)
            check(RobotArm::update(state, controls, 0.1), "held action continues moving");
        check(near(state.baseAngle, 60), "held motion accumulates without repeat events");
        controls.set(Action::BaseLeft, false);
        const State released = state;
        check(!controls.pressed(Action::BaseLeft), "release clears action");
        check(!RobotArm::update(state, controls, 0.1) && state == released,
              "released action stops moving");
        controls.held.fill(true);
        controls.clear();
        for (bool held : controls.held)
            check(!held, "clear releases every action");
        check(!RobotArm::update(state, controls, 1) && state == released,
              "clear stops all motion");
    }

    void testCancellationAndSimultaneousAxes()
    {
        const Action pairs[][2] = {
            {Action::BaseLeft, Action::BaseRight},
            {Action::ShoulderUp, Action::ShoulderDown},
            {Action::ElbowUp, Action::ElbowDown},
            {Action::WristLeft, Action::WristRight},
            {Action::Close, Action::Open}
        };
        for (const auto& pair : pairs)
        {
            State state;
            const State before = state;
            Controls controls;
            controls.set(pair[0], true);
            controls.set(pair[1], true);
            check(!RobotArm::update(state, controls, 10) && state == before,
                  "opposite pair cancels exactly");
        }
        State state;
        Controls controls;
        controls.held.fill(true);
        check(!RobotArm::update(state, controls, 1) && state == State{},
              "all opposite pairs cancel together");
        controls.clear();
        for (Action action : {Action::BaseLeft, Action::ShoulderUp, Action::ElbowDown,
                              Action::WristRight, Action::Close})
            controls.set(action, true);
        check(RobotArm::update(state, controls, 0.1), "multiple axes update together");
        check(nearState(state, State{6, 59.5f, -69.5f, 81, 0.9f}),
              "simultaneous axes retain independent rates");
    }

    void testLetters()
    {
        struct LetterCase { unsigned char lower; unsigned char upper; Action action; };
        const LetterCase cases[] = {
            {'a', 'A', Action::BaseLeft}, {'d', 'D', Action::BaseRight},
            {'w', 'W', Action::ShoulderUp}, {'s', 'S', Action::ShoulderDown},
            {'q', 'Q', Action::WristLeft}, {'e', 'E', Action::WristRight},
            {'f', 'F', Action::Close}, {'g', 'G', Action::Open}
        };
        for (const auto& item : cases)
        {
            for (unsigned char key : {item.lower, item.upper})
            {
                Controls controls;
                check(RobotArm::setLetter(controls, key, true), "letter is recognized");
                for (std::size_t index = 0; index < controls.held.size(); ++index)
                    check(controls.held[index] == (index == static_cast<std::size_t>(item.action)),
                          "letter activates only its mapped action");
                check(RobotArm::setLetter(controls, key, false), "letter release recognized");
                check(!controls.pressed(item.action), "letter release clears mapped action");
            }
            Controls controls;
            RobotArm::setLetter(controls, item.lower, true);
            RobotArm::setLetter(controls, item.upper, false);
            check(!controls.pressed(item.action), "case can change between press and release");
        }
        // 遍历整个 unsigned char 范围，验证非字母输入不会改变已有按键状态。
        for (unsigned int key = 0; key <= 255; ++key)
        {
            bool recognized = false;
            for (const auto& item : cases)
                recognized = recognized || key == item.lower || key == item.upper;
            if (!recognized)
            {
                Controls controls;
                controls.set(Action::WristLeft, true);
                const auto before = controls.held;
                check(!RobotArm::setLetter(controls, static_cast<unsigned char>(key), true)
                      && controls.held == before, "unknown press leaves controls unchanged");
                check(!RobotArm::setLetter(controls, static_cast<unsigned char>(key), false)
                      && controls.held == before, "unknown release leaves controls unchanged");
            }
        }
    }

    void testWrapAndClamp()
    {
        Controls controls;
        controls.set(Action::BaseLeft, true);
        controls.set(Action::WristLeft, true);
        State state{358, 0, 0, 355, 0.5f};
        RobotArm::update(state, controls, 0.1);
        check(near(state.baseAngle, 4) && near(state.wristRoll, 4),
              "positive yaw and roll cross 360");
        controls.clear();
        controls.set(Action::BaseRight, true);
        controls.set(Action::WristRight, true);
        state = State{2, 0, 0, 5, 0.5f};
        RobotArm::update(state, controls, 0.1);
        check(near(state.baseAngle, 356) && near(state.wristRoll, 356),
              "negative yaw and roll cross zero");
        state = State{6, 0, 0, 9, 0.5f};
        RobotArm::update(state, controls, 0.1);
        check(state.baseAngle == 0 && state.wristRoll == 0, "exact zero wrap");
        controls.clear();
        controls.set(Action::BaseLeft, true);
        controls.set(Action::WristLeft, true);
        state = State{354, 0, 0, 351, 0.5f};
        RobotArm::update(state, controls, 0.1);
        check(state.baseAngle == 0 && state.wristRoll == 0, "exact full-turn wrap");
        RobotArm::update(state, controls, 12.5);
        check(near(state.baseAngle, 30) && near(state.wristRoll, 45),
              "large normal time steps allow multiple revolutions");

        for (bool positive : {false, true})
        {
            controls.clear();
            controls.set(positive ? Action::ShoulderUp : Action::ShoulderDown, true);
            controls.set(positive ? Action::ElbowUp : Action::ElbowDown, true);
            controls.set(positive ? Action::Open : Action::Close, true);
            state = State{0, 0, 0, 90, 0.5f};
            check(RobotArm::update(state, controls, 10), "motion reaches limits");
            check(state.shoulderAngle == (positive ? 90 : -90)
                  && state.elbowAngle == (positive ? 90 : -90)
                  && state.opening == (positive ? 1 : 0), "pitch and opening clamp both ends");
            const State atLimit = state;
            check(!RobotArm::update(state, controls, 10) && state == atLimit,
                  "pushing against limits reports no change");
        }
    }

    void testFingerOffset()
    {
        check(near(RobotArm::fingerOffset(0), 0.09f), "closed finger offset");
        check(near(RobotArm::fingerOffset(1), 0.24f), "open finger offset");
        check(near(RobotArm::fingerOffset(0.5f), 0.165f), "linear midpoint offset");
        check(near(RobotArm::fingerOffset(-10), 0.09f), "negative opening clamps");
        check(near(RobotArm::fingerOffset(10), 0.24f), "excess opening clamps");
        check(near(RobotArm::fingerOffset(0) - 0.09f, 0), "closed pad inner faces meet");
        check(near(RobotArm::fingerOffset(1) - 0.09f, 0.15f), "open pad inner face distance");
        check(near(RobotArm::fingerOffset(std::numeric_limits<float>::quiet_NaN()), 0.09f),
              "NaN opening falls back to closed");
        check(near(RobotArm::fingerOffset(std::numeric_limits<float>::infinity()), 0.24f)
              && near(RobotArm::fingerOffset(-std::numeric_limits<float>::infinity()), 0.09f),
              "infinite openings clamp to endpoints");
    }

    void testTimeHandling()
    {
        Controls controls;
        for (Action action : {Action::BaseLeft, Action::ShoulderUp, Action::ElbowUp,
                              Action::WristRight, Action::Close})
            controls.set(action, true);
        // 用中间角度检查分步积分，避免将较大 float 默认肩角的累积舍入误差混入测试。
        State whole{0, 0, 0, 90, 1};
        State partitioned = whole;
        RobotArm::update(whole, controls, 0.7);
        for (int frame = 0; frame < 70; ++frame)
            RobotArm::update(partitioned, controls, 0.01);
        check(nearState(whole, partitioned), "partitioned time approximately matches one step");
        const double invalidSteps[] = {
            0, -0.0, -0.1, -100,
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()
        };
        for (double step : invalidSteps)
        {
            State state;
            const State before = state;
            check(!RobotArm::update(state, controls, step) && state == before,
                  "invalid time is ignored without changing any axis");
        }
        // 远大于 float 范围的有限时间，不能在转换/环绕之前溢出。
        for (double step : {1.0e100, std::numeric_limits<double>::max()})
        {
            for (bool positive : {false, true})
            {
                controls.clear();
                controls.set(positive ? Action::BaseLeft : Action::BaseRight, true);
                controls.set(positive ? Action::WristLeft : Action::WristRight, true);
                controls.set(positive ? Action::ShoulderUp : Action::ShoulderDown, true);
                controls.set(positive ? Action::ElbowUp : Action::ElbowDown, true);
                controls.set(positive ? Action::Open : Action::Close, true);
                State state{0, 0, 0, 90, 0.5f};
                check(RobotArm::update(state, controls, step), "huge finite time accepted");
                check(std::isfinite(state.baseAngle) && state.baseAngle >= 0 && state.baseAngle < 360
                      && std::isfinite(state.wristRoll) && state.wristRoll >= 0 && state.wristRoll < 360,
                      "huge finite time keeps wrapped axes finite and in range");
                check(state.shoulderAngle == (positive ? 90 : -90)
                      && state.elbowAngle == (positive ? 90 : -90)
                      && state.opening == (positive ? 1 : 0), "huge finite time clamps bounded axes");
            }
        }
    }

    void testPrecision()
    {
        Controls controls;
        controls.precision = true;
        controls.set(Action::BaseLeft, true);
        controls.set(Action::ShoulderDown, true);
        controls.set(Action::Close, true);
        State state;
        RobotArm::update(state, controls, 0.1);
        check(near(state.baseAngle, 0.12f) && near(state.shoulderAngle, 54.91f)
              && near(state.opening, 0.99f), "precision scales joints to 1/50 and opening to 1/10");
        controls.clear();
        check(controls.precision, "clearing pressed keys preserves selected speed");
    }

    void testInvalidActions()
    {
        Controls controls;
        controls.set(Action::BaseLeft, true);
        const auto before = controls.held;
        for (Action action : {Action::Count, static_cast<Action>(-1), static_cast<Action>(999)})
        {
            controls.set(action, true);
            controls.set(action, false);
            check(!controls.pressed(action) && controls.held == before,
                  "invalid action is safely ignored");
        }
    }
}

int main()
{
    testDefaultsAndEquality();
    testEachDirection();
    testHeldReleaseAndClear();
    testCancellationAndSimultaneousAxes();
    testLetters();
    testWrapAndClamp();
    testFingerOffset();
    testTimeHandling();
    testInvalidActions();
    testPrecision();

    if (failures != 0)
    {
        std::cerr << failures << " check(s) failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "RobotArmControl tests passed\n";
    return EXIT_SUCCESS;
}
