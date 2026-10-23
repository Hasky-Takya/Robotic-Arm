// 防止同一个头文件被重复包含。
#pragma once

// 使用动态数组保存 RGB 像素，像素数量由贴图尺寸决定。
#include <vector>

// 保存一张带地貌配色的等距圆柱投影贴图；像素按从左到右、从上到下排列。
struct EarthTextureImage
{
    // 贴图宽度为 2048 像素，对应从西经 180 度到东经 180 度。
    int width = 2048;
    // 贴图高度为 1024 像素，对应从北纬 90 度到南纬 90 度。
    int height = 1024;
    // 每个像素依次存储红、绿、蓝三个无符号字节；第一行是北极方向。
    std::vector<unsigned char> pixels;
};

// 声明地貌示意贴图生成函数；实现在 EarthTexture.cpp 中，main.cpp 通过此声明调用。
EarthTextureImage createEarthTexture();
