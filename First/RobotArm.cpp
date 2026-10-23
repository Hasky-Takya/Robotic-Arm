#include "RobotArm.h"
#include "Geometry.h"
#include <GL/freeglut.h>
#include <initializer_list>

namespace
{
    // 第一阶段的静态展示角度；后续控制应修改关节参数，而不是各部件的世界坐标。
    constexpr float baseAngle = 0.0f;
    constexpr float shoulderAngle = -35.0f;
    constexpr float elbowAngle = -65.0f;
    // 让两指在默认视角下分开可见，避免改变肘轴后沿视线互相遮挡。
    constexpr float wristRoll = 90.0f;
    constexpr float platformHeight = 0.16f;
    constexpr float baseHeight = 0.44f;
    constexpr float shoulderHeight = 0.32f;
    constexpr float fingerOffset = 0.24f;
    constexpr float fingerLength = 0.48f;

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

    // 掌座沿小臂轴线连接；夹指位于局部 Z 的两侧，保持固定张开。
    void drawGripper()
    {
        Geometry::setMaterial(silver);
        Geometry::drawCylinder(0.18f, 0.23f);
        glTranslatef(0.0f, 0.23f, 0.0f);
        Geometry::setMaterial(darkMetal);
        Geometry::drawBox(0.0f, 0.08f, 0.0f, 0.30f, 0.16f, 0.68f);
        for (const float side : { -1.0f, 1.0f })
        {
            Geometry::setMaterial(silver);
            Geometry::drawBox(0.0f, 0.16f + fingerLength / 2.0f, side * fingerOffset,
                              0.16f, fingerLength, 0.12f);
            // 内侧深色夹持垫与银色手指区分，张开间隙中不放置抓取物。
            Geometry::setMaterial(rubber);
            Geometry::drawBox(0.0f, 0.16f + fingerLength * 0.72f,
                              side * (fingerOffset - 0.075f),
                              0.17f, fingerLength * 0.44f, 0.03f);
        }
    }
}

void RobotArm::draw(float mountX, float tabletopY, float mountZ)
{
    glPushMatrix();
    glTranslatef(mountX, tabletopY, mountZ);

    // 固定平台不继承旋转底座的角度，底面与桌面上表面重合。
    Geometry::setMaterial(darkMetal);
    Geometry::drawBox(0.0f, platformHeight / 2.0f, 0.0f, 1.30f, platformHeight, 1.10f);
    Geometry::setMaterial(silver);
    for (const float x : { -0.49f, 0.49f })
    {
        for (const float z : { -0.39f, 0.39f })
        {
            glPushMatrix();
            glTranslatef(x, platformHeight, z);
            Geometry::drawCylinder(0.055f, 0.035f);
            glPopMatrix();
        }
    }

    // 底座 → 肩关节：从这里开始，所有下游部件都继承底座坐标系。
    glTranslatef(0.0f, platformHeight, 0.0f);
    glRotatef(baseAngle, 0.0f, 1.0f, 0.0f);
    Geometry::setMaterial(silver);
    Geometry::drawCylinder(0.48f, 0.10f);
    glTranslatef(0.0f, 0.10f, 0.0f);
    Geometry::setMaterial(armPaint);
    Geometry::drawCylinder(0.39f, baseHeight - 0.10f);
    glTranslatef(0.0f, baseHeight - 0.10f, 0.0f);
    Geometry::setMaterial(darkMetal);
    Geometry::drawBox(0.0f, shoulderHeight / 2.0f, 0.0f, 0.48f, shoulderHeight, 0.50f);
    glTranslatef(0.0f, shoulderHeight, 0.0f);
    drawPitchJoint(0.30f, 0.68f);

    // 大臂沿局部 +Y 延伸。图元缩放被封装隔离，不会缩放后续关节位置。
    glRotatef(shoulderAngle, 0.0f, 0.0f, 1.0f);
    drawLink(upperArmLength, 0.34f, 0.38f);
    glTranslatef(0.0f, upperArmLength, 0.0f);
    // 只将肘关节模型的圆柱轴从 Z 转到 X，避免影响小臂本身的初始朝向。
    glPushMatrix();
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    drawPitchJoint(0.25f, 0.60f);
    glPopMatrix();

    // 肩绕局部 Z 扫过 XY 平面；肘绕大臂局部 X，使小臂扫过与之垂直的 YZ 平面。
    // 该 X 轴继承大臂的姿态，不是世界 X 轴；腕部和夹爪继续继承小臂坐标系。
    glRotatef(elbowAngle, 1.0f, 0.0f, 0.0f);
    drawLink(forearmLength, 0.27f, 0.30f);
    glTranslatef(0.0f, forearmLength, 0.0f);
    Geometry::setMaterial(darkMetal);
    Geometry::drawCylinder(0.21f, 0.16f);
    glTranslatef(0.0f, 0.16f, 0.0f);

    // 腕部只绕小臂纵轴旋转，不能误用世界 Y 轴或额外改变小臂俯仰。
    glRotatef(wristRoll, 0.0f, 1.0f, 0.0f);
    drawGripper();
    glPopMatrix();
}
