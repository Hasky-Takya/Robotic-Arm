#pragma once
#include "RobotArmControl.h"
#include <array>

namespace RobotArm
{
    struct Vec3
    {
        double x = 0, y = 0, z = 0;
        Vec3 operator+(Vec3 rhs) const;
        Vec3 operator-(Vec3 rhs) const;
        Vec3 operator*(double scale) const;
    };
    double dot(Vec3 a, Vec3 b);

    // 列主序刚体矩阵，与 glMultMatrixd 一致；A*B 先应用 B，再应用 A。
    struct Pose
    {
        std::array<double, 16> m{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
        static Pose translation(Vec3 offset);
        static Pose rotationX(double degrees);
        static Pose rotationY(double degrees);
        static Pose rotationZ(double degrees);
        Pose operator*(const Pose& rhs) const;
        Vec3 point(Vec3 value) const;
        Vec3 vector(Vec3 value) const;
        Pose inverse() const;
        Vec3 position() const;
        void setPosition(Vec3 value);
    };

    // 绘制、检测、初始布局统一读取这些尺寸，不复制末端位置公式。
    inline constexpr float upperArmLength = 2.0f;
    inline constexpr float forearmLength = 1.5f;
    inline constexpr double tabletopY = 2.0;
    inline constexpr double tableHalfWidth = 4.0;
    inline constexpr double tableHalfDepth = 2.4;
    inline constexpr Vec3 mountPosition{-1.5, tabletopY, 0};
    inline constexpr double platformHeight = 0.16;
    inline constexpr double baseHeight = 0.44;
    inline constexpr double shoulderHeight = 0.32;
    inline constexpr double wristHeight = 0.16;
    inline constexpr double collarHeight = 0.23;
    inline constexpr double palmHeight = 0.16;
    inline constexpr double fingerLength = 0.48;
    inline constexpr double padCenterY = collarHeight + palmHeight + fingerLength * 0.72;
    inline constexpr double padHalfX = 0.085;
    inline constexpr double padHalfY = fingerLength * 0.22;
    inline constexpr double maxHalfGap = 0.15;

    struct Frames
    {
        Pose mount, base, shoulder, upperArm, elbow, forearm, wrist, gripper, grasp;
    };
    Frames forwardKinematics(const State& state, Vec3 mount = mountPosition);
}
