#include "CubeWorld.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace CubeWorld
{
    namespace
    {
        using RobotArm::Vec3;
        constexpr double epsilon = 1e-7;
        constexpr double contactBand = 0.012;
        constexpr double minimumContactArea = 0.002;
        constexpr double gravity = 9.81;

        double component(Vec3 p, int axis)
        {
            return axis == 0 ? p.x : axis == 1 ? p.y : p.z;
        }

        // Keep the actual face polygon, not its bounding rectangle: projected
        // corners alone are not enough to establish contact with a finger pad.
        std::vector<Vec3> clip(const std::vector<Vec3>& polygon,
                               int axis, double boundary, bool keepGreater)
        {
            std::vector<Vec3> result;
            if (polygon.empty()) return result;
            Vec3 previous = polygon.back();
            double previousDistance = (component(previous, axis) - boundary)
                                    * (keepGreater ? 1.0 : -1.0);
            for (Vec3 current : polygon)
            {
                const double distance = (component(current, axis) - boundary)
                                      * (keepGreater ? 1.0 : -1.0);
                if ((distance >= 0) != (previousDistance >= 0))
                {
                    const double t = previousDistance / (previousDistance - distance);
                    result.push_back(previous + (current - previous) * t);
                }
                if (distance >= 0) result.push_back(current);
                previous = current;
                previousDistance = distance;
            }
            return result;
        }

        double areaXY(const std::vector<Vec3>& polygon)
        {
            double twiceArea = 0;
            if (polygon.size() < 3) return 0;
            Vec3 previous = polygon.back();
            for (Vec3 p : polygon)
            {
                twiceArea += previous.x * p.y - p.x * previous.y;
                previous = p;
            }
            return std::abs(twiceArea) * 0.5;
        }

        double faceContactArea(const Pose& local, int faceAxis, int sign,
                               double halfSize, double halfGap)
        {
            const int u = (faceAxis + 1) % 3, v = (faceAxis + 2) % 3;
            const int corners[4][2] = {{-1,-1}, {1,-1}, {1,1}, {-1,1}};
            std::vector<Vec3> polygon;
            for (const auto& corner : corners)
            {
                double p[3] = {};
                p[faceAxis] = sign * halfSize;
                p[u] = corner[0] * halfSize;
                p[v] = corner[1] * halfSize;
                polygon.push_back(local.point({p[0], p[1], p[2]}));
            }
            polygon = clip(polygon, 0, -RobotArm::padHalfX, true);
            polygon = clip(polygon, 0, RobotArm::padHalfX, false);
            polygon = clip(polygon, 1, -RobotArm::padHalfY, true);
            polygon = clip(polygon, 1, RobotArm::padHalfY, false);
            const double faceZ = local.m[faceAxis * 4 + 2] * sign;
            if (faceZ > 0)
            {
                polygon = clip(polygon, 2, halfGap - contactBand, true);
                polygon = clip(polygon, 2, halfGap + epsilon, false);
            }
            else
            {
                polygon = clip(polygon, 2, -halfGap - epsilon, true);
                polygon = clip(polygon, 2, -halfGap + contactBand, false);
            }
            return areaXY(polygon);
        }

        bool validHeldIndex(const World& world)
        {
            return world.heldIndex >= 0
                && static_cast<std::size_t>(world.heldIndex) < world.cubes.size();
        }

        double verticalExtent(const Cube& cube)
        {
            return cube.size * 0.5 * (std::abs(cube.pose.m[1])
                 + std::abs(cube.pose.m[5]) + std::abs(cube.pose.m[9]));
        }

        bool overlapsTable(const Cube& cube)
        {
            // The projected cube is the convex hull of its eight vertices.
            // SAT against the table avoids AABB-only overlap at rotated corners.
            Vec3 vertices[8];
            int index = 0;
            const double h = cube.size * 0.5;
            for (int x : {-1, 1}) for (int y : {-1, 1}) for (int z : {-1, 1})
                vertices[index++] = cube.pose.point({x*h, y*h, z*h});
            const Vec3 axes[] = {{1,0,0}, {0,0,1},
                {-cube.pose.m[2],0,cube.pose.m[0]},
                {-cube.pose.m[6],0,cube.pose.m[4]},
                {-cube.pose.m[10],0,cube.pose.m[8]}};
            for (Vec3 axis : axes)
            {
                if (RobotArm::dot(axis, axis) < epsilon * epsilon) continue;
                double low = std::numeric_limits<double>::infinity();
                double high = -low;
                for (Vec3 p : vertices)
                {
                    const double projection = RobotArm::dot(axis, p);
                    low = std::min(low, projection);
                    high = std::max(high, projection);
                }
                const double radius = std::abs(axis.x) * RobotArm::tableHalfWidth
                                    + std::abs(axis.z) * RobotArm::tableHalfDepth;
                if (low >= radius - epsilon || high <= -radius + epsilon) return false;
            }
            return true;
        }

        bool fall(Cube& cube, double dt)
        {
            const Vec3 before = cube.pose.position();
            const double extent = verticalExtent(cube);
            const double bottom = before.y - extent;
            // A cube already below the tabletop must not teleport onto its top.
            const double support = overlapsTable(cube)
                && bottom >= RobotArm::tabletopY - epsilon ? RobotArm::tabletopY : 0.0;
            if (!cube.falling && std::abs(bottom - support) <= epsilon) return false;
            cube.falling = true;
            const double nextVelocity = cube.velocityY - gravity * dt;
            Vec3 after = before;
            after.y += cube.velocityY * dt - 0.5 * gravity * dt * dt;
            cube.velocityY = nextVelocity;
            // 桌面只从上方落下才支撑；地面同时作为下界，恢复被手动压到地下的方块。
            if ((support == 0.0 || bottom >= support - epsilon) && after.y - extent <= support)
            {
                after.y = support + extent;
                cube.velocityY = 0;
                cube.falling = false;
            }
            cube.pose.setPosition(after);
            return after.y != before.y;
        }
    }

    Contact evaluateContact(const Cube& cube, const Pose& grasp)
    {
        Contact result;
        if (!std::isfinite(cube.size) || cube.size <= 0) return result;
        const Pose local = grasp.inverse() * cube.pose;
        for (double value : local.m) if (!std::isfinite(value)) return result;
        const Vec3 center = local.position();
        if (std::abs(center.z) > 0.006 + epsilon) return result;
        const double h = cube.size * 0.5;
        const double halfGap = std::abs(center.z) + h * (std::abs(local.m[2])
                             + std::abs(local.m[6]) + std::abs(local.m[10]));
        // 留出释放所需的张开余量；不能捕获几乎塞满最大开口、之后无法松开的物体。
        if (halfGap > RobotArm::maxHalfGap * 0.90 + epsilon) return result;
        const double minY = center.y - h * (std::abs(local.m[1])
                          + std::abs(local.m[5]) + std::abs(local.m[9]));
        if (minY < -RobotArm::fingerLength * 0.72 - epsilon) return result;
        int faceAxis = 0;
        for (int axis = 1; axis < 3; ++axis)
            if (std::abs(local.m[axis*4+2]) > std::abs(local.m[faceAxis*4+2]))
                faceAxis = axis;
        if (std::abs(local.m[faceAxis*4+2]) < 0.98) return result;
        if (faceContactArea(local, faceAxis, 1, h, halfGap) < minimumContactArea
            || faceContactArea(local, faceAxis, -1, h, halfGap) < minimumContactArea)
            return result;
        result.valid = true;
        // Round upward so the float aperture never lies inside the cube surface.
        result.opening = static_cast<float>(halfGap / RobotArm::maxHalfGap);
        if (result.opening * RobotArm::maxHalfGap < halfGap)
            result.opening = std::nextafter(result.opening, 1.0f);
        result.opening = std::clamp(result.opening, 0.0f, 1.0f);
        result.score = -(center.x*center.x + center.y*center.y + center.z*center.z);
        return result;
    }

    RobotArm::State pickupState(float baseAngle)
    {
        RobotArm::State state;
        state.baseAngle = baseAngle;
        const double reach = RobotArm::upperArmLength + RobotArm::forearmLength
                           + RobotArm::wristHeight + RobotArm::padCenterY;
        const double shoulderY = RobotArm::mountPosition.y + RobotArm::platformHeight
                               + RobotArm::baseHeight + RobotArm::shoulderHeight;
        state.shoulderAngle = static_cast<float>(std::asin(
            (RobotArm::tabletopY + 0.11 - shoulderY) / reach)
            * 180.0 / 3.14159265358979323846);
        state.elbowAngle = 0;
        state.wristRoll = 0;
        state.opening = 1;
        return state;
    }

    World::World() { reset(); }

    void World::reset()
    {
        cubes.clear();
        const float angles[] = {0, -22, 22};
        for (int i = 0; i < 3; ++i)
        {
            Cube cube;
            cube.id = i + 1;
            cube.pose = Pose::rotationY(angles[i]);
            Vec3 center = RobotArm::forwardKinematics(pickupState(angles[i])).grasp.position();
            center.y = RobotArm::tabletopY + cube.size * 0.5;
            cube.pose.setPosition(center);
            cubes.push_back(cube);
        }
        heldIndex = candidateIndex = -1;
        relativePose = Pose{};
        contactOpening = 0;
        lastReleasedId = 0;
    }

    void World::refreshCandidate(const RobotArm::State& arm)
    {
        candidateIndex = -1;
        if (validHeldIndex(*this)) return;
        const Pose grasp = RobotArm::forwardKinematics(arm).grasp;
        double bestScore = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < cubes.size(); ++i)
        {
            if (cubes[i].falling) continue;
            const Contact contact = evaluateContact(cubes[i], grasp);
            if (!contact.valid || arm.opening + epsilon < contact.opening) continue;
            if (candidateIndex < 0 || contact.score > bestScore + epsilon
                || (std::abs(contact.score - bestScore) <= epsilon
                    && cubes[i].id < cubes[candidateIndex].id))
            {
                candidateIndex = static_cast<int>(i);
                bestScore = contact.score;
            }
        }
    }

    void World::release(const RobotArm::State& arm)
    {
        if (validHeldIndex(*this))
        {
            Cube& cube = cubes[heldIndex];
            cube.pose = RobotArm::forwardKinematics(arm).grasp * relativePose;
            cube.falling = true;
            cube.velocityY = 0;
            lastReleasedId = cube.id;
        }
        heldIndex = -1;
        contactOpening = 0;
        refreshCandidate(arm);
    }

    void World::resetArm(RobotArm::State& arm)
    {
        release(arm); // Release in the OLD pose, not at the reset/home pose.
        arm = RobotArm::State{};
        refreshCandidate(arm);
    }

    bool World::step(RobotArm::State& arm, const RobotArm::Controls& controls, double seconds)
    {
        if (!std::isfinite(seconds) || seconds <= 0) return false;
        // Bound work on stalls/untrusted callers, while still supporting 1 s tests.
        const double duration = std::min(seconds, 1.0);
        const int count = static_cast<int>(std::ceil(duration * 120.0));
        const double dt = duration / count;
        const bool closing = controls.pressed(RobotArm::Action::Close)
                          && !controls.pressed(RobotArm::Action::Open);
        const bool opening = controls.pressed(RobotArm::Action::Open)
                          && !controls.pressed(RobotArm::Action::Close);
        bool changed = false;
        for (int substep = 0; substep < count; ++substep)
        {
            const RobotArm::State before = arm;
            changed = RobotArm::update(arm, controls, dt) || changed;
            const Pose grasp = RobotArm::forwardKinematics(arm).grasp;
            int justReleased = -1;
            if (validHeldIndex(*this))
            {
                Cube& cube = cubes[heldIndex];
                const Pose followed = grasp * relativePose;
                changed = cube.pose.m != followed.m || changed;
                cube.pose = followed;
                if (opening && arm.opening > contactOpening + 0.08f)
                {
                    justReleased = heldIndex;
                    release(arm);
                    changed = true;
                }
                else
                    arm.opening = std::max(arm.opening, contactOpening);
            }
            else if (closing && arm.opening < before.opening)
            {
                int chosen = -1;
                Contact best;
                for (std::size_t i = 0; i < cubes.size(); ++i)
                {
                    if (cubes[i].falling) continue;
                    const Contact contact = evaluateContact(cubes[i], grasp);
                    if (!contact.valid || contact.opening > before.opening + epsilon
                        || contact.opening + epsilon < arm.opening) continue;
                    // The first encountered surface wins. Stable IDs break ties.
                    if (chosen < 0 || contact.opening > best.opening + epsilon
                        || (std::abs(contact.opening - best.opening) <= epsilon
                            && cubes[i].id < cubes[chosen].id))
                    {
                        chosen = static_cast<int>(i);
                        best = contact;
                    }
                }
                if (chosen >= 0)
                {
                    heldIndex = chosen;
                    contactOpening = arm.opening = best.opening;
                    relativePose = grasp.inverse() * cubes[chosen].pose;
                    cubes[chosen].falling = false;
                    cubes[chosen].velocityY = 0;
                    changed = true;
                }
            }
            for (std::size_t i = 0; i < cubes.size(); ++i)
                if (static_cast<int>(i) != heldIndex && static_cast<int>(i) != justReleased)
                    changed = fall(cubes[i], dt) || changed;
            changed = !(arm == before) || changed;
        }
        const int oldCandidate = candidateIndex;
        refreshCandidate(arm);
        return changed || candidateIndex != oldCandidate;
    }
}
