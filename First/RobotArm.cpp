#include "RobotArm.h"
#include "Geometry.h"
#include <GL/freeglut.h>
#include <initializer_list>

namespace
{
    constexpr float platformHeightF = static_cast<float>(RobotArm::platformHeight);
    constexpr float baseHeightF = static_cast<float>(RobotArm::baseHeight);
    constexpr float shoulderHeightF = static_cast<float>(RobotArm::shoulderHeight);
    constexpr float fingerLength = static_cast<float>(RobotArm::fingerLength);

    const Geometry::Material armPaint{ 0.93f, 0.43f, 0.08f, 0.38f, 48.0f };
    const Geometry::Material darkMetal{ 0.15f, 0.19f, 0.24f, 0.55f, 64.0f };
    const Geometry::Material silver{ 0.63f, 0.70f, 0.76f, 0.7f, 72.0f };
    const Geometry::Material rubber{ 0.055f, 0.065f, 0.075f, 0.04f, 8.0f };

    // 默认沿局部 Z 绘制关节；肘关节调用时额外转向局部 X，使两根转轴垂直。
    void drawPitchJoint(float radius, float width)
    {
        Geometry::setMaterial(darkMetal);
        glPushMatrix();
        glTranslatef(0.0f, 0.0f, -width / 2.0f);
        glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
        Geometry::drawCylinder(radius, width);
        Geometry::setMaterial(silver);
        glTranslatef(0.0f, -0.025f, 0.0f);
        Geometry::drawCylinder(radius * 0.62f, 0.025f);
        glTranslatef(0.0f, width + 0.025f, 0.0f);
        Geometry::drawCylinder(radius * 0.62f, 0.025f);
        glPopMatrix();
    }

    // 两条细银色饰条附着在臂段两侧，不参与父子关节变换。
    void drawLink(float length, float width, float depth)
    {
        Geometry::setMaterial(armPaint);
        Geometry::drawBox(0.0f, length / 2.0f, 0.0f, width, length, depth);
        Geometry::setMaterial(silver);
        for (const float side : { -1.0f, 1.0f })
        {
            Geometry::drawBox(0.0f, length / 2.0f, side * (depth / 2.0f + 0.012f),
                              width * 0.40f, length * 0.70f, 0.024f);
        }
    }

    // 掌座沿小臂轴线连接；两侧夹指和夹持垫对称移动，闭合时内侧刚好接触。
    void drawGripper(float opening)
    {
        const float fingerOffset = RobotArm::fingerOffset(opening);
        Geometry::setMaterial(silver);
        Geometry::drawCylinder(0.18f, static_cast<float>(RobotArm::collarHeight));
        glTranslated(0.0, RobotArm::collarHeight, 0.0);
        Geometry::setMaterial(darkMetal);
        constexpr float palmHeightF = static_cast<float>(RobotArm::palmHeight);
        Geometry::drawBox(0.0f, palmHeightF / 2.0f, 0.0f, 0.30f, palmHeightF, 0.68f);
        for (const float side : { -1.0f, 1.0f })
        {
            Geometry::setMaterial(silver);
            Geometry::drawBox(0.0f, palmHeightF + fingerLength / 2.0f, side * fingerOffset,
                              0.16f, fingerLength, 0.12f);
            // 内侧深色夹持垫与银色手指区分，张开间隙中不放置抓取物。
            Geometry::setMaterial(rubber);
            Geometry::drawBox(0.0f, palmHeightF + fingerLength * 0.72f,
                              side * (fingerOffset - 0.075f),
                              static_cast<float>(RobotArm::padHalfX * 2),
                              static_cast<float>(RobotArm::padHalfY * 2), 0.03f);
        }
    }
}

void RobotArm::draw(const State& state, float mountX, float tabletopY, float mountZ)
{
    const Frames frames = forwardKinematics(state, {mountX, tabletopY, mountZ});
    glPushMatrix();
    glMultMatrixd(frames.mount.m.data());

    // 固定平台不继承旋转底座的角度，底面与桌面上表面重合。
    Geometry::setMaterial(darkMetal);
    Geometry::drawBox(0.0f, platformHeightF / 2.0f, 0.0f, 1.30f, platformHeightF, 1.10f);
    Geometry::setMaterial(silver);
    for (const float x : { -0.49f, 0.49f })
    {
        for (const float z : { -0.39f, 0.39f })
        {
            glPushMatrix();
            glTranslatef(x, platformHeightF, z);
            Geometry::drawCylinder(0.055f, 0.035f);
            glPopMatrix();
        }
    }

    glPopMatrix();

    // 每个世界矩阵都由共享 FK 生成，抓取检测与画面使用同一套父子变换。
    glPushMatrix();
    glMultMatrixd(frames.base.m.data());
    Geometry::setMaterial(silver);
    Geometry::drawCylinder(0.48f, 0.10f);
    glTranslatef(0.0f, 0.10f, 0.0f);
    Geometry::setMaterial(armPaint);
    Geometry::drawCylinder(0.39f, baseHeightF - 0.10f);
    glTranslatef(0.0f, baseHeightF - 0.10f, 0.0f);
    Geometry::setMaterial(darkMetal);
    Geometry::drawBox(0.0f, shoulderHeightF / 2.0f, 0.0f, 0.48f, shoulderHeightF, 0.50f);
    glPopMatrix();

    glPushMatrix();
    glMultMatrixd(frames.shoulder.m.data());
    drawPitchJoint(0.30f, 0.68f);
    glPopMatrix();
    glPushMatrix();
    glMultMatrixd(frames.upperArm.m.data());
    drawLink(upperArmLength, 0.34f, 0.38f);
    glPopMatrix();

    // 肘部圆柱沿大臂局部 X；模型转向不混入小臂的运动学矩阵。
    glPushMatrix();
    glMultMatrixd(frames.elbow.m.data());
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    drawPitchJoint(0.25f, 0.60f);
    glPopMatrix();
    glPushMatrix();
    glMultMatrixd(frames.forearm.m.data());
    drawLink(forearmLength, 0.27f, 0.30f);
    glPopMatrix();

    glPushMatrix();
    glMultMatrixd(frames.wrist.m.data());
    Geometry::setMaterial(darkMetal);
    Geometry::drawCylinder(0.21f, static_cast<float>(wristHeight));
    glPopMatrix();
    glPushMatrix();
    glMultMatrixd(frames.gripper.m.data());
    drawGripper(state.opening);
    glPopMatrix();
}
