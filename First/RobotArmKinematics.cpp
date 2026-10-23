#include "RobotArmKinematics.h"
#include <cmath>

namespace RobotArm
{
    namespace { constexpr double radians = 3.14159265358979323846 / 180.0; }
    Vec3 Vec3::operator+(Vec3 rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    Vec3 Vec3::operator-(Vec3 rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    Vec3 Vec3::operator*(double scale) const { return {x * scale, y * scale, z * scale}; }
    double dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

    Pose Pose::translation(Vec3 offset)
    {
        Pose result;
        result.setPosition(offset);
        return result;
    }
    Pose Pose::rotationX(double degrees)
    {
        Pose result;
        const double c = std::cos(degrees*radians), s = std::sin(degrees*radians);
        result.m[5] = c; result.m[6] = s; result.m[9] = -s; result.m[10] = c;
        return result;
    }
    Pose Pose::rotationY(double degrees)
    {
        Pose result;
        const double c = std::cos(degrees*radians), s = std::sin(degrees*radians);
        result.m[0] = c; result.m[2] = -s; result.m[8] = s; result.m[10] = c;
        return result;
    }
    Pose Pose::rotationZ(double degrees)
    {
        Pose result;
        const double c = std::cos(degrees*radians), s = std::sin(degrees*radians);
        result.m[0] = c; result.m[1] = s; result.m[4] = -s; result.m[5] = c;
        return result;
    }
    Pose Pose::operator*(const Pose& rhs) const
    {
        Pose result;
        for (int column = 0; column < 4; ++column)
            for (int row = 0; row < 4; ++row)
            {
                result.m[column*4 + row] = 0;
                for (int k = 0; k < 4; ++k)
                    result.m[column*4 + row] += m[k*4 + row] * rhs.m[column*4 + k];
            }
        return result;
    }
    Vec3 Pose::vector(Vec3 v) const
    {
        return {m[0]*v.x + m[4]*v.y + m[8]*v.z,
                m[1]*v.x + m[5]*v.y + m[9]*v.z,
                m[2]*v.x + m[6]*v.y + m[10]*v.z};
    }
    Vec3 Pose::point(Vec3 v) const { return vector(v) + position(); }
    Vec3 Pose::position() const { return {m[12], m[13], m[14]}; }
    void Pose::setPosition(Vec3 v) { m[12] = v.x; m[13] = v.y; m[14] = v.z; }
    Pose Pose::inverse() const
    {
        Pose result;
        for (int column = 0; column < 3; ++column)
            for (int row = 0; row < 3; ++row)
                result.m[column*4 + row] = m[row*4 + column];
        result.setPosition(result.vector(position()) * -1.0);
        return result;
    }
    Frames forwardKinematics(const State& state, Vec3 mount)
    {
        Frames f;
        f.mount = Pose::translation(mount);
        f.base = f.mount * Pose::translation({0, platformHeight, 0}) * Pose::rotationY(state.baseAngle);
        f.shoulder = f.base * Pose::translation({0, baseHeight + shoulderHeight, 0});
        // 状态肩角以水平为零，初始 55 度仍呈现原来的向上弯曲姿态。
        f.upperArm = f.shoulder * Pose::rotationZ(state.shoulderAngle - 90.0);
        f.elbow = f.upperArm * Pose::translation({0, upperArmLength, 0});
        f.forearm = f.elbow * Pose::rotationX(state.elbowAngle);
        f.wrist = f.forearm * Pose::translation({0, forearmLength, 0});
        f.gripper = f.wrist * Pose::translation({0, wristHeight, 0}) * Pose::rotationY(state.wristRoll);
        f.grasp = f.gripper * Pose::translation({0, padCenterY, 0});
        return f;
    }
}
