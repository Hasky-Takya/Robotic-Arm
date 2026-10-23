#include "../First/CubeWorld.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace
{
    using RobotArm::Action;
    using RobotArm::Controls;
    using RobotArm::Pose;
    using RobotArm::State;
    using RobotArm::Vec3;
    using CubeWorld::Cube;
    using CubeWorld::World;

    // Explicit checks remain active in Release/NDEBUG builds.
    int failures = 0;
    void check(bool condition, const char* message)
    {
        if (!condition)
        {
            ++failures;
            std::cerr << "FAIL: " << message << '\n';
        }
    }
    bool near(double a, double b, double tolerance = 1e-6)
    {
        return std::isfinite(a) && std::isfinite(b) && std::abs(a-b) <= tolerance;
    }
    bool nearVec(Vec3 a, Vec3 b, double tolerance = 1e-6)
    {
        return near(a.x,b.x,tolerance) && near(a.y,b.y,tolerance)
            && near(a.z,b.z,tolerance);
    }
    bool nearPose(const Pose& a, const Pose& b, double tolerance = 1e-6)
    {
        for (std::size_t i = 0; i < a.m.size(); ++i)
            if (!near(a.m[i],b.m[i],tolerance)) return false;
        return true;
    }
    Controls key(Action action)
    {
        Controls result;
        result.set(action, true);
        return result;
    }
    Cube localCube(Vec3 center = {}, Pose rotation = Pose{}, double size = 0.22)
    {
        Cube cube;
        cube.size = size;
        cube.pose = rotation;
        cube.pose.setPosition(center);
        return cube;
    }
    double extentY(const Cube& cube)
    {
        return cube.size * 0.5 * (std::abs(cube.pose.m[1])
             + std::abs(cube.pose.m[5]) + std::abs(cube.pose.m[9]));
    }
    bool capture(World& world, State& arm)
    {
        world.step(arm, key(Action::Close), 0.4);
        return world.heldIndex >= 0;
    }

    void testRigidMathAndKinematics()
    {
        const Pose rotation = Pose::rotationX(31) * Pose::rotationY(-57) * Pose::rotationZ(19);
        const Pose transform = Pose::translation({2,-3,4}) * rotation;
        check(nearPose(transform.inverse()*transform, Pose{}), "rigid inverse left identity");
        check(nearPose(transform*transform.inverse(), Pose{}), "rigid inverse right identity");
        const Vec3 p{0.2,-1.7,3.1};
        check(nearVec(transform.inverse().point(transform.point(p)), p), "rigid point round trip");
        check(nearVec(transform.vector(p), rotation.vector(p)), "vectors ignore translation");
        check(nearVec((Pose::translation({1,2,3})*Pose::rotationZ(90)).point({1,0,0}),
                      {1,3,3}), "matrix order and degree convention");
        check(nearVec(Pose::rotationY(90).vector({1,0,0}), {0,0,-1}), "yaw convention");
        check(nearVec(Pose::rotationX(90).vector({0,1,0}), {0,0,1}), "X rotation convention");
        check(State{}.shoulderAngle == 55, "home shoulder is 55 degrees");

        State arm = CubeWorld::pickupState();
        arm.shoulderAngle = 0;
        const auto horizontal = RobotArm::forwardKinematics(arm);
        const double shoulderY = RobotArm::tabletopY + RobotArm::platformHeight
                               + RobotArm::baseHeight + RobotArm::shoulderHeight;
        check(near(horizontal.shoulder.position().y, shoulderY), "shoulder height from dimensions");
        check(nearVec(horizontal.elbow.position()-horizontal.shoulder.position(), {2,0,0}),
              "zero shoulder gives horizontal upper arm");
        check(nearVec(horizontal.wrist.position()-horizontal.elbow.position(), {1.5,0,0}),
              "zero elbow continues upper arm");
        arm.elbowAngle = 90;
        const auto bent = RobotArm::forwardKinematics(arm);
        const Vec3 upper = bent.elbow.position()-bent.shoulder.position();
        const Vec3 lower = bent.wrist.position()-bent.elbow.position();
        check(near(RobotArm::dot(upper,lower), 0), "elbow rotates on perpendicular local axis");
        check(near(lower.z,1.5), "elbow positive bend is out of shoulder plane");
        const auto custom = RobotArm::forwardKinematics(arm, {0,0,0});
        check(nearVec(bent.grasp.position()-custom.grasp.position(), RobotArm::mountPosition),
              "custom mount translates full chain");
        arm.wristRoll = 47;
        const auto rolled = RobotArm::forwardKinematics(arm);
        check(nearVec(rolled.grasp.position(), bent.grasp.position()), "wrist roll preserves grasp center");
        check(!nearPose(rolled.grasp,bent.grasp), "wrist roll changes full grasp orientation");
    }

    void testSpawnAndContacts()
    {
        World world;
        check(world.cubes.size() == 3, "three initial cubes");
        const float angles[] = {0,-22,22};
        for (std::size_t i = 0; i < world.cubes.size(); ++i)
        {
            const State arm = CubeWorld::pickupState(angles[i]);
            const Cube& cube = world.cubes[i];
            const Pose grasp = RobotArm::forwardKinematics(arm).grasp;
            check(near(arm.shoulderAngle, -10.618877, 1e-5), "reachable pickup shoulder angle");
            check(arm.baseAngle == angles[i] && arm.elbowAngle == 0 && arm.wristRoll == 0
                  && arm.opening == 1, "pickup remaining state fields");
            check(cube.id == static_cast<int>(i)+1 && near(cube.size,0.22), "stable IDs and cube sizes");
            check(nearVec(cube.pose.position(),grasp.position()), "spawn uses actual grasp FK center");
            check(near(cube.pose.position().y-extentY(cube),RobotArm::tabletopY), "spawn rests on table");
            Pose expectedYaw = Pose::rotationY(angles[i]);
            expectedYaw.setPosition(cube.pose.position());
            check(nearPose(cube.pose,expectedYaw), "spawn yaw matches closure direction");
            const auto contact = CubeWorld::evaluateContact(cube,grasp);
            check(contact.valid && near(contact.opening,0.22/0.30), "spawn has bilateral face contact");
            check(contact.opening*RobotArm::maxHalfGap >= 0.11, "contact aperture does not penetrate");
            const double openInner = RobotArm::fingerOffset(arm.opening)-0.09;
            check(openInner > cube.size*0.5 + 0.03, "fully open fingers clear cube faces");
            world.refreshCandidate(arm);
            check(world.candidateIndex == static_cast<int>(i), "each spawn is an individual candidate");
        }
        check(CubeWorld::evaluateContact(localCube(),Pose{}).valid, "centered cube accepted");
        check(CubeWorld::evaluateContact(localCube({0.08,0.07,0}),Pose{}).valid,
              "meaningful pad overlap allows cube overhang in x and y");
        check(!CubeWorld::evaluateContact(localCube({1,0,0}),Pose{}).valid, "far cube rejected");
        check(!CubeWorld::evaluateContact(localCube({0,0,0.007}),Pose{}).valid, "off-center closure rejected");
        check(!CubeWorld::evaluateContact(localCube({},Pose::rotationY(20)),Pose{}).valid,
              "misaligned face normals rejected");
        check(!CubeWorld::evaluateContact(localCube({0,0,0.006},Pose::rotationY(5)),Pose{}).valid,
              "aligned projected faces with only corner contact in depth band are rejected");
        check(!CubeWorld::evaluateContact(localCube({},Pose{},0.31),Pose{}).valid, "oversize gap rejected");
        check(!CubeWorld::evaluateContact(localCube({},Pose{},0.29),Pose{}).valid,
              "near-full aperture cube rejected to reserve release clearance");
        check(!CubeWorld::evaluateContact(localCube({0,-0.3,0}),Pose{}).valid, "palm/throat side rejected");
        check(!CubeWorld::evaluateContact(localCube({0.194,0,0}),Pose{}).valid,
              "tiny face sliver is not meaningful bilateral contact");
        check(!CubeWorld::evaluateContact(localCube({0.19,0.19,0},Pose::rotationZ(45)),Pose{}).valid,
              "overlapping projected bounds without real face overlap rejected");
        check(!CubeWorld::evaluateContact(localCube({},Pose{},0),Pose{}).valid, "zero size rejected");
        Cube invalid = localCube();
        invalid.pose.m[12] = std::numeric_limits<double>::quiet_NaN();
        check(!CubeWorld::evaluateContact(invalid,Pose{}).valid, "nonfinite pose rejected");
        const auto centered = CubeWorld::evaluateContact(localCube(),Pose{});
        const auto shifted = CubeWorld::evaluateContact(localCube({0.02,0,0}),Pose{});
        check(centered.score > shifted.score, "score favors centered contact");
        const Pose arbitrary = Pose::translation({-3,4,2})*Pose::rotationY(63)*Pose::rotationX(-12);
        Cube transformed = localCube();
        transformed.pose = arbitrary*transformed.pose;
        check(CubeWorld::evaluateContact(transformed,arbitrary).valid,
              "contact is evaluated in grasp coordinates rather than world axes");
        check(CubeWorld::evaluateContact(localCube({0,0,0.005}),Pose{}).valid,
              "small centering tolerance retains bilateral contact band");
    }

    void testCaptureCrossingsAndSelection()
    {
        for (float yaw : {0.0f,-22.0f,22.0f})
        {
            World world;
            State arm = CubeWorld::pickupState(yaw);
            const int target = yaw == 0 ? 0 : yaw < 0 ? 1 : 2;
            const Pose oldPose = world.cubes[target].pose;
            check(capture(world,arm), "each of three cubes can be captured");
            check(world.heldIndex == target, "capture selects correct reachable cube");
            check(nearPose(world.cubes[target].pose,oldPose), "capture never snaps cube into center");
            check(near(arm.opening,world.contactOpening), "capture stops at surface opening");
            world.step(arm,key(Action::Close),1.0);
            check(near(arm.opening,world.contactOpening), "continued close cannot crush held cube");
            world.step(arm,Controls{},0.1);
            check(near(arm.opening,world.contactOpening), "idle preserves held contact opening");
        }
        {
            World world;
            State arm = CubeWorld::pickupState();
            world.step(arm,Controls{},0.1);
            check(world.heldIndex == -1, "proximity without closing never attaches");
            world.step(arm,key(Action::Open),0.1);
            check(world.heldIndex == -1, "opening never attaches");
            Controls both = key(Action::Close);
            both.set(Action::Open,true);
            world.step(arm,both,0.5);
            check(world.heldIndex == -1, "opposed open and close never attach");
            arm.opening = 0.5f;
            world.refreshCandidate(arm);
            check(world.candidateIndex == -1, "candidate must fit current aperture");
            world.step(arm,key(Action::Close),0.1);
            check(world.heldIndex == -1, "already-too-small aperture cannot capture");
            arm.opening = 0;
            arm.baseAngle = 359.5f;
            world.step(arm,key(Action::BaseLeft),1.0/120);
            check(world.heldIndex == -1, "moving closed gripper onto cube cannot attach");
        }
        {
            World world;
            State arm = CubeWorld::pickupState();
            const auto contact = CubeWorld::evaluateContact(world.cubes[0],RobotArm::forwardKinematics(arm).grasp);
            arm.opening = contact.opening + 0.002f;
            world.step(arm,key(Action::Close),0.004);
            check(world.heldIndex == 0 && near(arm.opening,contact.opening),
                  "old-to-proposed aperture crossing catches surface within substep");
        }
        {
            World world;
            State arm = CubeWorld::pickupState();
            world.cubes.resize(1);
            Vec3 center = world.cubes[0].pose.position();
            center.z += 0.004;
            world.cubes[0].pose.setPosition(center);
            const Pose before = world.cubes[0].pose;
            check(capture(world,arm), "slightly off-center cube can be captured bilaterally");
            check(nearPose(world.cubes[0].pose,before), "off-center capture preserves cube position and orientation");
            check(arm.opening*RobotArm::maxHalfGap >= 0.114,
                  "off-center aperture uses outermost face rather than average face depth");
        }
        {
            World world;
            State arm = CubeWorld::pickupState();
            Cube small = world.cubes[0];
            small.size = 0.20;
            small.pose.setPosition({small.pose.position().x,RobotArm::tabletopY+0.10,small.pose.position().z});
            small.id = 1;
            Cube big = world.cubes[0];
            big.id = 9;
            world.cubes = {small,big};
            check(capture(world,arm) && world.cubes[world.heldIndex].id == 9,
                  "largest required aperture is encountered before smaller cube");
        }
        {
            World world;
            State arm = CubeWorld::pickupState();
            Cube duplicate = world.cubes[0];
            duplicate.id = 2;
            world.cubes[0].id = 9;
            world.cubes = {world.cubes[0],duplicate};
            world.refreshCandidate(arm);
            check(world.candidateIndex == 1, "candidate ties use stable lower ID");
            check(capture(world,arm) && world.heldIndex == 1, "one deterministic target attaches on tie");
            check(world.candidateIndex == -1, "no candidate while already holding");
        }
        {
            World world;
            State arm = CubeWorld::pickupState();
            world.cubes.resize(1);
            world.cubes[0].falling = true;
            world.refreshCandidate(arm);
            check(world.candidateIndex == -1, "falling cube is not a candidate");
            // Cross the contact aperture while the cube is explicitly in flight.
            const auto contact = CubeWorld::evaluateContact(world.cubes[0],
                RobotArm::forwardKinematics(arm).grasp);
            arm.opening = contact.opening + 0.002f;
            world.cubes[0].velocityY = 10;
            world.step(arm,key(Action::Close),0.004);
            check(world.heldIndex == -1, "falling cube cannot be captured");
        }
    }

    void testFollowingAndRelease()
    {
        World world;
        State arm = CubeWorld::pickupState();
        check(capture(world,arm), "follow test starts held");
        if (world.heldIndex < 0) return;
        const int index = world.heldIndex;
        const Pose relative = world.relativePose;
        for (Action action : {Action::BaseLeft,Action::ShoulderUp,Action::ElbowUp,
                              Action::WristLeft,Action::ShoulderDown,Action::ElbowDown})
        {
            world.step(arm,key(action),0.12);
            check(nearPose(world.cubes[index].pose,RobotArm::forwardKinematics(arm).grasp*relative),
                  "held full rigid pose follows each joint and wrist");
            check(nearPose(world.relativePose,relative), "relative held transform remains unchanged");
        }
        const Pose beforeRoll = world.cubes[index].pose;
        world.step(arm,key(Action::WristLeft),0.1);
        check(!nearPose(beforeRoll,world.cubes[index].pose), "wrist roll visibly rotates held cube");
        arm.opening = world.contactOpening + 0.04f;
        world.step(arm,Controls{},0.001);
        check(world.heldIndex == index, "small opening clearance does not release");
        arm.opening = world.contactOpening + 0.079f;
        const Pose beforeRelease = world.cubes[index].pose;
        world.step(arm,key(Action::Open),0.002);
        check(world.heldIndex == -1, "net open releases only after clearance threshold");
        check(nearPose(world.cubes[index].pose,beforeRelease), "release substep preserves entire pose without gravity jump");
        check(world.cubes[index].falling && world.cubes[index].velocityY == 0,
              "release initializes falling with zero velocity");
        check(world.lastReleasedId == world.cubes[index].id, "release records stable cube ID");
        const Pose releasedRotation = world.cubes[index].pose;
        const double height = world.cubes[index].pose.position().y;
        world.step(arm,Controls{},0.01);
        check(world.cubes[index].pose.position().y < height, "gravity begins on subsequent substep");
        Pose rotationOnly = world.cubes[index].pose;
        rotationOnly.setPosition(releasedRotation.position());
        check(nearPose(rotationOnly,releasedRotation), "falling preserves release rotation");
    }

    void testGravityAndResets()
    {
        State arm;
        {
            World world;
            const auto initial = world.cubes;
            check(!world.step(arm,Controls{},0.5), "resting cubes and idle arm remain unchanged");
            for (std::size_t i = 0; i < initial.size(); ++i)
                check(nearPose(initial[i].pose,world.cubes[i].pose) && !world.cubes[i].falling,
                      "initial cube support is stable");
            const State before = arm;
            for (double dt : {0.0,-1.0,std::numeric_limits<double>::infinity(),
                              std::numeric_limits<double>::quiet_NaN()})
                check(!world.step(arm,key(Action::BaseLeft),dt) && arm == before,
                      "invalid dt does not modify arm or world");
            world.step(arm,key(Action::BaseLeft),1.0);
            check(near(arm.baseAngle,60,0.001), "one-second step integrates full requested interval");
            arm = State{};
            world.step(arm,key(Action::BaseLeft),std::numeric_limits<double>::max());
            check(near(arm.baseAngle,60,0.001), "huge finite interval has bounded one-second work");
        }
        for (bool table : {true,false})
        {
            World world;
            Cube cube = localCube({table ? 0.0 : 10.0,3.5,0},
                                  Pose::rotationX(30)*Pose::rotationZ(25));
            cube.falling = true;
            world.cubes = {cube};
            world.step(arm,Controls{},1.0);
            const double support = table ? RobotArm::tabletopY : 0;
            check(near(world.cubes[0].pose.position().y-extentY(world.cubes[0]),support),
                  "oriented cube lands using full vertical extent on table or floor");
            check(!world.cubes[0].falling && world.cubes[0].velocityY == 0, "landing stops gravity");
            Pose expected = cube.pose;
            expected.setPosition(world.cubes[0].pose.position());
            check(nearPose(world.cubes[0].pose,expected), "landing preserves orientation");
        }
        {
            World world;
            world.cubes = {localCube({0,1,0})};
            world.cubes[0].falling = true;
            world.step(arm,Controls{},0.01);
            check(world.cubes[0].pose.position().y < 1, "under-table cube falls rather than teleporting up");
            world.step(arm,Controls{},1.0);
            check(near(world.cubes[0].pose.position().y,0.11), "under-table cube lands on floor");
        }
        {
            World world;
            State heldArm = CubeWorld::pickupState();
            check(capture(world, heldArm), "capture before below-floor release");
            heldArm.shoulderAngle = -90;
            world.step(heldArm, Controls{}, 0.001);
            const int index = world.heldIndex;
            check(world.cubes[index].pose.position().y < 0, "manual arm can place held cube below floor");
            world.step(heldArm, key(Action::Open), 0.2);
            check(world.heldIndex == -1
                  && near(world.cubes[index].pose.position().y, extentY(world.cubes[index])),
                  "released below-floor cube recovers to ground instead of falling forever");
        }
        {
            World world;
            // Center outside table but its projected footprint still overlaps edge.
            world.cubes = {localCube({RobotArm::tableHalfWidth+0.05,3,0})};
            world.step(arm,Controls{},1.0);
            check(near(world.cubes[0].pose.position().y,RobotArm::tabletopY+0.11),
                  "partial projected footprint overlap supports cube on table");
            Vec3 center = world.cubes[0].pose.position();
            center.x = RobotArm::tableHalfWidth+0.2;
            world.cubes[0].pose.setPosition(center);
            world.step(arm,Controls{},1.0);
            check(near(world.cubes[0].pose.position().y,0.11), "entirely outside table falls to floor");
        }
        {
            World world;
            State pickup = CubeWorld::pickupState();
            check(capture(world,pickup), "home-reset test starts held");
            if (world.heldIndex < 0) return;
            world.step(pickup,key(Action::ShoulderUp),0.15);
            const int held = world.heldIndex;
            // release/reset must also honor direct arm changes since the last step.
            pickup.baseAngle += 5;
            const Pose poseBefore = RobotArm::forwardKinematics(pickup).grasp*world.relativePose;
            const int id = world.cubes[held].id;
            world.resetArm(pickup);
            check(pickup == State{}, "home reset restores default arm");
            check(world.heldIndex == -1 && world.lastReleasedId == id, "home reset releases and retains history");
            check(nearPose(world.cubes[held].pose,poseBefore), "home reset releases in pre-reset arm pose");
            check(world.cubes[held].falling, "home reset released cube falls");
            world.reset();
            World fresh;
            check(world.heldIndex == -1 && world.candidateIndex == -1 && world.lastReleasedId == 0
                  && world.contactOpening == 0, "world/end reset clears interaction and history");
            check(nearPose(world.relativePose,Pose{}), "world reset clears relative transform");
            for (std::size_t i = 0; i < fresh.cubes.size(); ++i)
                check(nearPose(world.cubes[i].pose,fresh.cubes[i].pose)
                      && !world.cubes[i].falling && world.cubes[i].velocityY == 0,
                      "world/end reset restores all spawn poses and physics");
        }
    }
}

int main()
{
    testRigidMathAndKinematics();
    testSpawnAndContacts();
    testCaptureCrossingsAndSelection();
    testFollowingAndRelease();
    testGravityAndResets();
    if (failures)
    {
        std::cerr << failures << " CubeWorld checks failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "CubeWorld tests passed\n";
    return EXIT_SUCCESS;
}
