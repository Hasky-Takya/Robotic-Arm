# OpenGL 机械臂 — 第一阶段

按照 `一.docx` 的第一阶段路线实现静态机械臂场景。沿用原工程的 **C++20 + OpenGL 固定管线 + GLU + freeglut**，没有新增 GLFW、GLAD、GLM 等依赖，也未修改规划文档。

## 已实现

- 三维透视场景、地面、桌面、桌腿及固定支架。
- 固定安装平台、圆柱底座、肩关节、大臂、肘关节、小臂、腕部及左右夹爪。
- 基础方向光、环境光、材质高光、深度遮挡和双缓冲。
- 固定弯曲展示姿态；大臂长度 `2.0`，小臂长度 `1.5`。
- 层级变换：底座 → 肩 → 大臂 → 肘 → 小臂 → 腕部 → 双指夹爪。
- 鼠标拖动观察和默认视角复位；窗口缩放时同步更新投影。

**尚未实现：** 关节旋转控制、夹爪开合、键盘操纵机械臂（第二阶段）；方块检测、抓取、跟随与释放（第三阶段）。没有自动动画、碰撞检测或阴影贴图。

## 构建与运行

### Visual Studio

1. 打开当前目录的 `First.slnx`。
2. 选择平台 **x64** 和配置 **Debug** 或 **Release**。
3. 生成解决方案，按 `Ctrl+F5` 运行。

本机工程使用 Visual Studio 2026 的 **v145** 工具集、Windows SDK 10 和 C++20。Win32 配置保留原样，本阶段没有配置或验证 Win32。

依赖默认目录：

```text
L:\OpenGL\freeglut\include\GL\freeglut.h
L:\OpenGL\freeglut\lib\freeglut_staticd.lib  （Debug）
L:\OpenGL\freeglut\lib\freeglut_static.lib   （Release）
```

`FreeglutRoot` 是可覆盖的 MSBuild 属性，默认 `L:\OpenGL\freeglut`。换电脑时可在项目属性/属性表中设置它，或者在构建命令中增加 `-p:FreeglutRoot="新的 freeglut 目录"`；库必须是相应架构和 CRT 配置。

### 本机命令行

以下命令适用于 PowerShell：

```powershell
& 'T:\AppSystemFiles\Visual_Studio\MSBuild\Current\Bin\MSBuild.exe' .\First.slnx -m -p:Configuration=Debug -p:Platform=x64
& 'T:\AppSystemFiles\Visual_Studio\MSBuild\Current\Bin\MSBuild.exe' .\First.slnx -m -p:Configuration=Release -p:Platform=x64
.\x64\Release\First.exe
```

从解决方案构建时，程序输出为 `x64\Debug\First.exe` 或 `x64\Release\First.exe`。直接构建内部 `First\First.vcxproj` 时，默认输出可能位于 `First\x64\...`，以构建日志为准。

freeglut 使用静态链接，不需要复制 freeglut DLL；C/C++ 运行库仍采用 `/MDd`（Debug）和 `/MD`（Release），与本机安装的 freeglut 库一致，**静态链接 freeglut 不等于静态链接 CRT**。

## 观察操作

| 操作 | 效果 |
| --- | --- |
| 按住鼠标左键拖动 | 环绕观察，不改变机械臂姿态 |
| `R` | 恢复默认相机方向 |
| `Esc` / 关闭窗口 | 退出 |

本阶段不支持 `W/S`、`A/D`、方向键、`Q/E`、`F/G` 控制机械臂。

## 源文件职责

```text
First/main.cpp       窗口、相机、光照、桌面和场景回调
First/Geometry.h/cpp 材质、长方体与沿局部 +Y 的封口圆柱
First/RobotArm.h/cpp 机械臂尺寸、静态姿态和父子层级绘制
```

世界坐标 **Y 向上**，桌面上表面位于 `y=2.0`；机械臂安装点位于 `(-1.5, 2.0, 0)`。臂段沿局部 +Y 建模，肩关节绕局部 Z 轴转动，肘关节绕继承大臂姿态后的局部 X 轴转动；两根关节转轴相互垂直，小臂扫过的平面与大臂扫过的平面垂直。肘部圆柱模型也与实际转轴对齐。腕部绕小臂局部 Y 轴旋转。基础图元在内部压栈/出栈隔离缩放，防止缩放污染后续关节位置。

原 `EarthTexture.cpp/.h` 及地理数据文件仍保留在磁盘，但已移出当前项目编译项。

## 验证结果

- x64 Debug 与 Release 均构建成功；原 Debug 混入 Release freeglut 库产生的 `LNK4098` 冲突已消除。
- Release 构建无警告。Debug 的已安装 freeglut 静态库缺少 `freeglut_staticd.pdb`，链接器会提示 `LNK4099`；不影响运行或本项目源码调试，但无法完整调试第三方 freeglut 内部。未通过忽略默认库或屏蔽警告掩盖此情况。
- 使用临时测试入口复用应用的初始化、场景绘制和回调，在真实 OpenGL 上下文中验证首帧、鼠标回调、`R` 复位、尺寸切换和 `Esc` 退出；读取帧缓冲检查了默认视角、环绕视角和竖屏画面。
- 测试窗口尺寸包括 `1100×780`、`600×900`、`100×780`，渲染均返回 `GL_NO_ERROR`。极窄窗口会自动拉远相机并同步调整裁剪面，避免整个模型消失。
- 测试通过直接调用应用回调进行，不代表已完成所有人工窗口交互测试；最小化恢复及不同显卡上的表现尚未逐一验证。
