#pragma once
#include "RobotArmKinematics.h"
#include <vector>

namespace CubeWorld
{
    using RobotArm::Pose;
    struct Cube
    {
        int id = 0;
        double size = 0.22;
        Pose pose;
        bool falling = false;
        double velocityY = 0;
    };
    struct Contact
    {
        bool valid = false;
        float opening = 0;
        double score = 0;
    };
    // 检测夹持面而非仅判断中心距离；grasp 原点位于夹持垫中心。
    Contact evaluateContact(const Cube& cube, const Pose& grasp);
    RobotArm::State pickupState(float baseAngle = 0);

    struct World
    {
        std::vector<Cube> cubes;
        int heldIndex = -1;
        int candidateIndex = -1;
        Pose relativePose;
        float contactOpening = 0;
        int lastReleasedId = 0;

        World();
        void reset();
        bool step(RobotArm::State& arm, const RobotArm::Controls& controls, double seconds);
        void release(const RobotArm::State& arm);
        void refreshCandidate(const RobotArm::State& arm);
        void resetArm(RobotArm::State& arm);
    };
}
