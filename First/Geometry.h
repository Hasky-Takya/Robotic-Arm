#pragma once

// 基础图元只负责自己的局部变换，不改变调用者的模型矩阵。
namespace Geometry
{
    struct Material
    {
        float red;
        float green;
        float blue;
        float specular = 0.25f;
        float shininess = 32.0f;
    };

    void setMaterial(const Material& material);
    // 长方体以 (x, y, z) 为中心，三个尺寸分别沿局部 X、Y、Z 轴。
    void drawBox(float x, float y, float z, float width, float height, float depth);
    // 圆柱底面中心位于局部原点，沿 +Y 延伸；两个端面均封闭。
    void drawCylinder(float radius, float height);
}
