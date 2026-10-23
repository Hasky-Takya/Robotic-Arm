// 引入贴图数据结构与函数声明，使此文件的实现和 main.cpp 使用相同接口。
#include "EarthTexture.h"
// 引入排序与数值范围限制函数，用于配对扫描线交点并防止写入越界。
#include <algorithm>
// 引入向上取整函数，用于将海岸线位置转换为像素范围。
#include <cmath>
// 使用固定长度数组保存 RGB 颜色和六类地貌的混合权重。
#include <array>

// 将地理数据类型限制在当前文件中，避免与其他源文件的名字冲突。
namespace
{
    // 保存海岸线上的一个点，单位为度。
    struct GeoPoint
    {
        // 经度：向东为正，范围约为 [-180, 180]。
        double longitude;
        // 纬度：向北为正，范围约为 [-90, 90]。
        double latitude;
    };

    // 描述一个闭合海岸线环在顶点数组中的位置。
    struct LandRing
    {
        // 本轮廓第一个点在 landPoints 中的下标。
        int firstPoint;
        // 本轮廓包含的点数；连接最后一点和第一点即可闭合轮廓。
        int pointCount;
    };

    // 每一类地貌占一个二进制位，使同一位置可以同时属于高原、森林或山地。
    enum class TerrainKind : unsigned char
    {
        Forest = 1,     // 森林：深绿色。
        Grassland = 2,  // 草原：黄绿色。
        Desert = 4,     // 沙漠：沙黄色。
        Plateau = 8,    // 高原：赭色，并适当保留原来的植被颜色。
        Mountain = 16,  // 山地：岩石色与山脊纹理。
        Ice = 32        // 冰川、冰盖：偏蓝的白色。
    };

    // 一个地理区域可以包含多个轮廓，也可以与其他地貌区域重叠。
    struct TerrainRegion
    {
        int firstRing;    // 本区域第一个轮廓在 terrainRings 中的位置。
        int ringCount;    // 本区域包含的轮廓数量。
        TerrainKind kind; // 本区域对应的地貌类别。
    };

    // 用三个浮点分量保存颜色，便于在不同地貌之间平滑混色。
    using Color = std::array<double, 3>;

    // 引入 Natural Earth 1:110m 的全球陆地轮廓常量；数据编译进程序，无需运行时下载。
    // 数据包含各大洲、南极洲和主要岛屿；细小海湾与小岛会按此比例尺简化。
    #include "earth_land_data.inc"
    // 引入沙漠、山地、高原、部分草原/森林区域和冰川的简化参考边界。
    // 此数据是地理区域示意，不是精确的植被分类或数字高程模型。
    #include "earth_terrain_data.inc"

    // 将多边形按等距圆柱投影填入掩码；一个字节可保存多个叠加的地貌标记。
    void rasterizeMask(std::vector<unsigned char>& mask, int width, int height,
                       const GeoPoint* points, const LandRing* rings,
                       std::size_t ringCount, unsigned char flag)
    {
        // 初始化当前区域的纬度范围，随后用轮廓顶点收紧扫描范围。
        double minLatitude = 90.0;
        // 从南极开始寻找区域最北侧的纬度。
        double maxLatitude = -90.0;
        // 遍历当前区域中的各个轮廓。
        for (std::size_t ringIndex = 0; ringIndex < ringCount; ++ringIndex)
        {
            // 获取该轮廓在顶点数组中的位置和长度。
            const LandRing& ring = rings[ringIndex];
            // 遍历顶点，求出区域的南北边界。
            for (int vertex = 0; vertex < ring.pointCount; ++vertex)
            {
                // 读取当前顶点的纬度。
                const double latitude = points[ring.firstPoint + vertex].latitude;
                // 更新区域最南侧的纬度。
                minLatitude = std::min(minLatitude, latitude);
                // 更新区域最北侧的纬度。
                maxLatitude = std::max(maxLatitude, latitude);
            }
        }

        // 将区域最北侧纬度转换为起始行，像素中心位于 y + 0.5。
        const int firstY = std::clamp(
            static_cast<int>(std::ceil((90.0 - maxLatitude) * height / 180.0 - 0.5)), 0, height);
        // 将区域最南侧纬度转换为结束行；范围外不需要参与扫描。
        const int endY = std::clamp(
            static_cast<int>(std::ceil((90.0 - minLatitude) * height / 180.0 - 0.5)), 0, height);
        // 保存水平扫描线与当前区域所有边界的交点。
        std::vector<double> intersections;

        // 从北向南扫描当前区域覆盖的图像行。
        for (int y = firstY; y < endY; ++y)
        {
            // 将当前像素行的中心换算成纬度。
            const double latitude = 90.0 - (y + 0.5) * 180.0 / height;
            // 清空上一行交点，同时复用动态数组容量。
            intersections.clear();
            // 同一个区域的所有轮廓一起计算，保留内部孔洞。
            for (std::size_t ringIndex = 0; ringIndex < ringCount; ++ringIndex)
            {
                // 获取当前闭合轮廓。
                const LandRing& ring = rings[ringIndex];
                // 依次读取每条边，第一条边连接最后一点和第一点。
                for (int current = 0, previous = ring.pointCount - 1;
                     current < ring.pointCount; previous = current++)
                {
                    // 读取边的起点经纬度。
                    const GeoPoint& a = points[ring.firstPoint + previous];
                    // 读取边的终点经纬度。
                    const GeoPoint& b = points[ring.firstPoint + current];
                    // 只处理跨过当前纬度的边；半开区间可避免顶点重复计数，并跳过水平边。
                    if ((a.latitude > latitude) != (b.latitude > latitude))
                    {
                        // 线性插值求出扫描线与边相交的经度。
                        const double longitude = a.longitude +
                            (latitude - a.latitude) * (b.longitude - a.longitude) /
                            (b.latitude - a.latitude);
                        // 将交点经度映射到贴图的横向像素坐标。
                        intersections.push_back((longitude + 180.0) * width / 360.0);
                    }
                }
            }

            // 按从左到右的顺序排序交点，以奇偶规则配对成填充区间。
            std::sort(intersections.begin(), intersections.end());
            // 每对交点表示扫描线上的一段区域。
            for (std::size_t pair = 0; pair + 1 < intersections.size(); pair += 2)
            {
                // 计算区间内第一个像素下标，并限制在图像范围内。
                const int firstX = std::clamp(
                    static_cast<int>(std::ceil(intersections[pair] - 0.5)), 0, width);
                // 计算不参与填充的结束下标。
                const int endX = std::clamp(
                    static_cast<int>(std::ceil(intersections[pair + 1] - 0.5)), 0, width);
                // 遍历区间，将对应的地貌位写入掩码。
                for (int x = firstX; x < endX; ++x)
                {
                    // 使用按位或保留之前的标记，防止重叠山脉或高原把已有区域抵消掉。
                    mask[static_cast<std::size_t>(y) * width + x] |= flag;
                }
            }
        }
    }

    // 在两个边界之间生成 0 到 1 的平滑权重，用于植被、积雪和颜色的柔和过渡。
    double smoothStep(double lower, double upper, double value)
    {
        // 先将数值归一化，再限制在 [0, 1]。
        const double t = std::clamp((value - lower) / (upper - lower), 0.0, 1.0);
        // 用三次曲线避免过渡区两端出现突变。
        return t * t * (3.0 - 2.0 * t);
    }

    // 在两个 RGB 颜色之间混合；amount 为 0 时取 a，为 1 时取 b。
    Color mixColor(const Color& a, const Color& b, double amount)
    {
        // 保证混合权重不越界。
        amount = std::clamp(amount, 0.0, 1.0);
        // 对红、绿、蓝三个分量分别进行线性插值。
        return { a[0] + (b[0] - a[0]) * amount,
                 a[1] + (b[1] - a[1]) * amount,
                 a[2] + (b[2] - a[2]) * amount };
    }

    // 用球面上的三维位置生成确定性的细节，不使用随机数，因此每次启动图案都相同。
    double surfaceNoise(double x, double y, double z)
    {
        // 大尺度起伏决定斑块走势；坐标取自球面，所以经度接缝和两极不会出现噪声断裂。
        const double coarse = std::sin(x * 17.0 + std::sin(z * 13.0)) * std::sin(y * 19.0 - z * 7.0);
        // 中尺度起伏用于地貌内部的颜色变化。
        const double medium = std::sin(x * 53.0 + y * 31.0) * std::sin(z * 47.0 - y * 29.0);
        // 小尺度起伏提供植被、砂砾和岩石的细节。
        const double fine = std::sin(x * 149.0 + z * 113.0) * std::sin(y * 137.0 - x * 71.0);
        // 加权得到 [-1, 1] 范围内的多尺度程序纹理；它只表示视觉细节，不代表真实海拔。
        return coarse * 0.55 + medium * 0.30 + fine * 0.15;
    }

    // 在当前像素附近采样地貌掩码，将硬边界过渡成略微柔和的区域边缘。
    std::array<double, 6> terrainCoverage(const std::vector<unsigned char>& mask,
                                         int x, int y, int width, int height)
    {
        // 分别累计森林、草原、沙漠、高原、山地、冰雪的权重。
        std::array<double, 6> coverage = {};
        // 设置三点滤波权重，中间像素的影响大于两侧像素。
        constexpr int weights[] = { 1, 2, 1 };
        // 采样当前行及其上下各 2 像素的邻近行。
        for (int row = 0; row < 3; ++row)
        {
            // 限制纵坐标，防止在南北极跨过图像边界。
            const int sampleY = std::clamp(y + (row - 1) * 2, 0, height - 1);
            // 采样每行的三个横向位置。
            for (int column = 0; column < 3; ++column)
            {
                // 横向循环采样，让东、西经 180 度两侧的区域权重连续衔接。
                const int sampleX = (x + (column - 1) * 2 + width) % width;
                // 读取邻近像素的所有地貌标记。
                const unsigned char flags = mask[static_cast<std::size_t>(sampleY) * width + sampleX];
                // 横纵权重相乘并除以总权重 16，使最终覆盖率保持在 [0, 1]。
                const double weight = weights[row] * weights[column] / 16.0;
                // 逐个检查六类地貌，允许同一个像素同时贡献给多类地貌。
                for (int layer = 0; layer < 6; ++layer)
                {
                    // 当前位存在时，将其覆盖率加入对应地貌。
                    if ((flags & (1U << layer)) != 0) coverage[layer] += weight;
                }
            }
        }
        // 返回六类地貌的平滑覆盖率。
        return coverage;
    }

    // 按经纬度、参考地理区域和程序纹理绘制一个陆地像素。
    Color landColor(double longitude, double latitude, const std::array<double, 6>& coverage)
    {
        // 定义本文件使用的圆周率，避免依赖非标准的 M_PI 宏。
        constexpr double pi = 3.14159265358979323846;
        // 将经度从度转换为弧度。
        const double lon = longitude * pi / 180.0;
        // 将纬度从度转换为弧度。
        const double lat = latitude * pi / 180.0;
        // 计算球面 X 坐标，用于生成经度可循环的地貌细节。
        const double nx = std::cos(lat) * std::cos(lon);
        // 计算球面 Y 坐标，使噪声在两极收敛到同一方向。
        const double ny = std::sin(lat);
        // 计算球面 Z 坐标。
        const double nz = std::cos(lat) * std::sin(lon);
        // 生成大中小尺度混合的颜色变化。
        const double noise = surfaceNoise(nx, ny, nz);
        // 再用不同尺度生成岩石和山脊细节，避免所有地貌共享同一种图案。
        const double detail = surfaceNoise(nx * 2.7 + 3.0, ny * 2.7, nz * 2.7 - 5.0);
        // 南北半球使用纬度绝对值描述寒冷程度。
        const double absoluteLatitude = std::abs(latitude);
        // 赤道附近默认较湿润，向副热带逐渐过渡；这只是植被示意规则。
        const double tropicalForest = 1.0 - smoothStep(10.0, 25.0, absoluteLatitude);
        // 用中高纬带补充加拿大、北欧和西伯利亚等地的森林示意。
        const double borealForest = smoothStep(43.0, 53.0, absoluteLatitude) *
            (1.0 - smoothStep(63.0, 71.0, absoluteLatitude));
        // 合并湿润带和寒温带权重，并用程序纹理打散边缘。
        const double forestAmount = std::clamp(std::max(tropicalForest, borealForest) * 0.85 + noise * 0.12, 0.0, 1.0);
        // 设置黄绿色的草地底色，未被其他地貌覆盖的位置保留此基础配色。
        Color color = { 142.0, 166.0, 88.0 };
        // 按纬度混入森林颜色，补齐地理区域数据未提供的植被分布。
        color = mixColor(color, { 42.0, 103.0, 60.0 }, forestAmount);
        // 高纬陆地逐渐变为低饱和的苔原色，避免直接从森林跳到纯白冰雪。
        color = mixColor(color, { 157.0, 157.0, 126.0 }, smoothStep(64.0, 75.0, absoluteLatitude));
        // 在大平原、欧亚草原和潘帕斯等参考区域加强草原配色。
        color = mixColor(color, { 156.0, 174.0, 88.0 }, coverage[1] * 0.90);
        // 亚马孙和刚果盆地的参考区域使用更深的森林颜色。
        color = mixColor(color, { 35.0, 100.0, 54.0 }, coverage[0] * 0.95);
        // 高原以赭色表达，但保留部分原植被；寒温带高原减少裸地颜色，避免整片森林变棕。
        const double plateauAmount = coverage[3] * (0.78 - borealForest * 0.45);
        // 根据高原覆盖率混入高原的基础颜色。
        color = mixColor(color, { 170.0, 145.0, 108.0 }, plateauAmount);
        // 为沙漠生成细微的砂色变化，突出与周围草地的区别。
        const Color sand = { 221.0 + detail * 9.0, 189.0 + detail * 8.0, 121.0 + detail * 6.0 };
        // 在撒哈拉、戈壁、阿拉伯半岛和澳洲等沙漠区域应用沙黄色。
        color = mixColor(color, sand, coverage[2]);
        // 从细节纹理中提取山脊强度；数值越大，岩石纹理越明亮。
        const double ridge = std::clamp(1.0 - std::abs(detail) * 1.7, 0.0, 1.0);
        // 在贴图内生成岩石的深浅变化，这些颜色不会随相机或场景光源变化。
        const double rockBrightness = 0.78 + ridge * 0.35 + noise * 0.12;
        // 设置岩石颜色；这是烘焙到贴图中的示意起伏，不会改变球面高度。
        Color rock = { 134.0 * rockBrightness, 128.0 * rockBrightness, 111.0 * rockBrightness };
        // 标记主要高山带，供山地内部的局部雪峰示意使用，不代表逐点真实海拔。
        const bool alpineRegion =
            (longitude > 65.0 && longitude < 105.0 && latitude > 26.0 && latitude < 40.0) ||
            (longitude > -82.0 && longitude < -65.0 && latitude > -45.0 && latitude < 5.0) ||
            (longitude > -130.0 && longitude < -105.0 && latitude > 38.0) ||
            (longitude > 5.0 && longitude < 16.0 && latitude > 44.0 && latitude < 48.0);
        // 高山带或寒冷地区的部分山脊使用白色，表达高处积雪。
        const double peakSnow = (alpineRegion || absoluteLatitude > 58.0) ?
            smoothStep(0.82, 0.98, ridge) * 0.72 : 0.0;
        // 将积雪混入山脊颜色，只在地理山地范围内显示。
        rock = mixColor(rock, { 232.0, 239.0, 237.0 }, peakSnow);
        // 叠加山地纹理，同时保留部分山坡植被和沙漠颜色。
        color = mixColor(color, rock, coverage[4] * 0.85);
        // 使用低幅度变化为森林、草地和高原增加斑驳细节。
        const double variation = 1.0 + noise * 0.08 + detail * 0.025;
        // 依次调整 RGB 三个颜色分量。
        for (double& channel : color) channel *= variation;
        // 冰川数据决定主要冰盖位置；极高纬陆地用少量雪地补足简化数据的边缘。
        double snowAmount = std::max(coverage[5], smoothStep(76.0, 84.0, absoluteLatitude + noise * 2.0));
        // 南极大陆统一覆盖冰雪，避免轮廓简化差异产生绿色边条。
        if (latitude < -60.0) snowAmount = 1.0;
        // 使用略带蓝色和细节变化的白色表现冰雪；此步骤优先于其他地貌。
        color = mixColor(color, { 231.0 + detail * 4.0, 240.0 + detail * 3.0, 241.0 + detail * 2.0 }, snowAmount);
        // 返回该陆地像素的最终 RGB 颜色。
        return color;
    }
}

// 生成含冰雪、沙漠、山地、高原、草原和森林的全球地貌示意贴图。
EarthTextureImage createEarthTexture()
{
    // 使用原有的 2048 × 1024 尺寸，保持贴图映射和相机控制接口不变。
    EarthTextureImage image;
    // 计算像素总数，后续掩码每个像素使用一个字节。
    const std::size_t pixelCount = static_cast<std::size_t>(image.width) * image.height;
    // 为每个像素分配三个 RGB 字节。
    image.pixels.resize(pixelCount * 3);
    // 创建初始全为 0 的海陆掩码；填充后 1 表示陆地，0 表示海洋。
    std::vector<unsigned char> landMask(pixelCount, 0);
    // 创建独立的地貌掩码，避免地貌区域轮廓改变原有海岸线。
    std::vector<unsigned char> terrainMask(pixelCount, 0);
    // 将原有全球陆地轮廓填入海陆掩码。
    rasterizeMask(landMask, image.width, image.height, landPoints, landRings,
                  sizeof(landRings) / sizeof(landRings[0]), 1);
    // 分别填充各地貌区域；不同区域按位叠加，不会因相互重叠而消失。
    for (const TerrainRegion& region : terrainRegions)
    {
        // 将该区域的轮廓按对应的地貌类型填入掩码。
        rasterizeMask(terrainMask, image.width, image.height, terrainPoints,
                      terrainRings + region.firstRing, region.ringCount,
                      static_cast<unsigned char>(region.kind));
    }

    // 从北极方向向南极方向逐行生成贴图，赤道仍位于图像中间。
    for (int y = 0; y < image.height; ++y)
    {
        // 使用像素中心计算该行纬度。
        const double latitude = 90.0 - (y + 0.5) * 180.0 / image.height;
        // 逐列处理同一纬度上的像素。
        for (int x = 0; x < image.width; ++x)
        {
            // 使用像素中心计算当前经度。
            const double longitude = -180.0 + (x + 0.5) * 360.0 / image.width;
            // 将二维图像坐标转换为掩码中的一维下标。
            const std::size_t pixel = static_cast<std::size_t>(y) * image.width + x;
            // 先设置蓝色海洋，陆地像素会在后面改用地貌配色。
            Color color = { 31.0, 93.0, 157.0 };
            // 只在真实陆地掩码内应用陆地地貌，防止山地或沙漠区域覆盖海洋。
            if (landMask[pixel] != 0)
            {
                // 采样当前陆地像素的六类地貌覆盖率。
                const auto coverage = terrainCoverage(terrainMask, x, y, image.width, image.height);
                // 按位置、植被规则和参考区域生成陆地颜色。
                color = landColor(longitude, latitude, coverage);
            }
            // 为北冰洋添加示意性的高纬海冰；不模拟季节或具体日期的海冰边界。
            else if (latitude > 77.0)
            {
                // 用连续的经向变化打散海冰边界，东西接缝两侧保持周期一致。
                const double iceEdge = latitude + 1.2 * std::sin(longitude * 3.14159265358979323846 / 60.0);
                // 从海洋蓝色平滑过渡到偏蓝的白色海冰。
                color = mixColor(color, { 222.0, 237.0, 241.0 }, smoothStep(78.0, 83.0, iceEdge));
            }
            // 逐个写入红、绿、蓝三个颜色通道。
            for (std::size_t channel = 0; channel < 3; ++channel)
            {
                // 四舍五入并限制到字节范围，避免混色或细节计算产生越界颜色。
                image.pixels[pixel * 3 + channel] = static_cast<unsigned char>(
                    std::clamp(std::lround(color[channel]), 0L, 255L));
            }
        }
    }

    // 返回内存中的地貌贴图，由 main.cpp 上传；本函数不会修改地球自转和相机参数。
    return image;
}
