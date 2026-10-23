#include "Geometry.h"
#include <GL/freeglut.h>

namespace Geometry
{
    // 分别设置漫反射和高光，使关节金属与喷漆臂段具有不同的质感。
    void setMaterial(const Material& material)
    {
        const GLfloat diffuse[] = { material.red, material.green, material.blue, 1.0f };
        const GLfloat specular[] = { material.specular, material.specular, material.specular, 1.0f };
        glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, diffuse);
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, material.shininess);
    }

    void drawBox(float x, float y, float z, float width, float height, float depth)
    {
        glPushMatrix();
        glTranslatef(x, y, z);
        glScalef(width, height, depth);
        glutSolidCube(1.0);
        glPopMatrix();
    }

    void drawCylinder(float radius, float height)
    {
        glPushMatrix();
        // freeglut 的圆柱沿 +Z 延伸；绕 X 轴旋转 -90 度后改为 +Y。
        glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
        glutSolidCylinder(radius, height, 48, 4);
        glPopMatrix();
    }
}
