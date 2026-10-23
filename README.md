# OpenGL 机械臂 — 第二阶段

按照 `一.docx` 实现第一阶段场景和第二阶段键盘手动控制，保留已确认的**小臂扫动平面垂直于大臂扫动平面**。沿用 **C++20 + OpenGL 固定管线 + GLU + freeglut**，未增加 GLFW、GLAD、GLM 等依赖，未修改原规划文档。

## 已实现

- 三维场景、桌面/桌腿、安装平台、底座、大臂、小臂、腕部和双指夹爪。
- 基础光照、材质、深度遮挡、双缓冲及鼠标环绕观察。
- 底座旋转、大臂与小臂关节转动、腕部旋转、双指夹爪对称开合。
- 按住键连续运动、松开停止、多关节组合操作、反向键相互抵消。
- 关节限位、连续旋转角度回绕、开合比例限幅。
- 屏幕显示帮助和实时角度/开合比例，机械臂及相机可分别复位。

**本次不包含第三阶段：** 没有可抓取方块、抓取检测、物体跟随或释放。也未实现碰撞检测、自相交限制、阴影贴图或运动学求解；某些合法关节组合可能穿过桌面或其他部件。

## 键盘与鼠标

先让机械臂窗口获得焦点；使用英文字母键，支持大小写。

| 按键 | 功能 | 速度/范围 |
| --- | --- | --- |
| `A` / `D` | 底座左/右旋转（角度增加/减少） | 60°/秒；[0°,360°) 循环 |
| `W` / `S` | 大臂正/反方向俯仰 | 45°/秒；[-90°,90°] |
| `↑` / `↓` | 小臂正/反方向转动，扫动平面与大臂垂直 | 45°/秒；[-90°,90°] |
| `Q` / `E` | 腕部绕小臂纵轴正/反方向旋转 | 90°/秒；[0°,360°) 循环 |
| `F` / `G` | 夹爪闭合/张开 | 每秒完整开合一次；[0,1] |
| `Home` | 机械臂恢复初始姿态，清空按住状态 | 不改变相机 |
| `R` | 恢复默认相机方向 | 不改变机械臂状态 |
| 鼠标左键拖动 | 环绕观察 | 不改变关节状态 |
| `Esc` / 关闭窗口 | 退出 | — |

- 各运动键需**按住**，松开即停止，不依赖系统键盘重复速度。
- 不同关节可以同时运动，例如按住 `A + W + ↑`；同一轴的反向键同时按住则抵消。
- 初始角度：底座 `0°`、大臂 `-35°`、小臂 `-65°`、腕部 `90°`、夹爪全开。
- `W` 和 `↑` 增加角度，在初始负角度下朝零位抬起；由于采用有符号关节角和垂直肘轴，不保证整个角度区间中末端的世界高度始终单调上升。以屏幕实时角度与实际姿态为准。
- Windows 下切换前台窗口或最小化时暂停并清空按键，回来后重新按键；仅把鼠标移出窗口不会暂停键盘运动。
- 状态每约 16ms 更新，按真实经过时间计算位移。暂停/卡顿后的单次积分最多 50ms，避免跳动；渲染只读取状态，不改变姿态。

## 构建与运行

### Visual Studio

1. 打开 `First.slnx`。
2. 选择平台 **x64** 和配置 **Debug** 或 **Release**。
3. 生成解决方案，按 `Ctrl+F5` 运行。

本机使用 Visual Studio 2026 的 **v145** 工具集、Windows SDK 10 和 C++20。Win32 配置保留原样，未配置或验证 Win32；失焦检测使用 Windows 原生窗口句柄。

默认依赖路径：

```text
L:\OpenGL\freeglut\include\GL\freeglut.h
L:\OpenGL\freeglut\lib\freeglut_staticd.lib  （Debug）
L:\OpenGL\freeglut\lib\freeglut_static.lib   （Release）
```

`FreeglutRoot` 可通过项目属性/属性表或 MSBuild `-p:FreeglutRoot="新目录"` 覆盖，默认 `L:\OpenGL\freeglut`。库必须与 x64 架构及 CRT 配置匹配。

本机 PowerShell 构建命令：

```powershell
& 'T:\AppSystemFiles\Visual_Studio\MSBuild\Current\Bin\MSBuild.exe' .\First.slnx -m -p:Configuration=Debug -p:Platform=x64
& 'T:\AppSystemFiles\Visual_Studio\MSBuild\Current\Bin\MSBuild.exe' .\First.slnx -m -p:Configuration=Release -p:Platform=x64
.\x64\Release\First.exe
```

从解决方案生成的程序位于 `x64\Debug\First.exe` 或 `x64\Release\First.exe`。直接构建内部 `.vcxproj` 时输出目录可能不同，以日志为准。

freeglut 静态链接，不需要复制 freeglut DLL；CRT 仍使用 `/MDd`（Debug）和 `/MD`（Release），与已安装的库一致。

## 文件与坐标约定

```text
First/main.cpp               窗口、场景、相机、键盘/计时/焦点处理、状态显示
First/Geometry.h/.cpp        材质及基础长方体/封口圆柱
First/RobotArm.h/.cpp        读取状态，按父子层级绘制机械臂
First/RobotArmControl.h/.cpp 与 OpenGL 无关的状态、按键动作、速度、限位和更新
tests/RobotArmControlTests.cpp 独立控制逻辑回归测试（不编入应用）
```

- 世界 Y 向上，桌面上表面 `y=2.0`，安装点 `(-1.5,2.0,0)`。
- 大臂长度 `2.0`，小臂长度 `1.5`，均沿自己的局部 +Y 建模。
- 底座绕 Y；肩绕局部 Z；肘绕继承大臂姿态后的局部 X（与肩轴垂直）；腕绕小臂局部 Y。
- 肘关节圆柱与肘轴对齐，模型转向用独立矩阵压栈隔离，不影响小臂初始方向。
- 两指中心偏移由闭合 `0.09` 线性变化到全开 `0.24`；内侧夹持垫闭合时接触，不交叉。
- 几何缩放使用独立压栈/出栈，不污染下游关节长度。
- 原 EarthTexture 源文件和地理数据保留在磁盘，不参与当前构建。

## 测试

### 独立控制逻辑

在 **x64 Native Tools Command Prompt for VS 2026** 中，于项目根目录执行：

```bat
cl /nologo /std:c++20 /EHsc /utf-8 /W4 /MD tests\RobotArmControlTests.cpp First\RobotArmControl.cpp /Fe:RobotArmControlTests.exe
.\RobotArmControlTests.exe
```

测试不需要 OpenGL 或测试框架，即使定义 NDEBUG 也不会跳过检查。覆盖初始状态、每个方向的速度、按住/释放、组合及反向键、大小写、角度回绕、限位、开合几何端点、时间步一致性及无效时间。

### 渲染与回调验证

本次另外用临时测试入口在真实 OpenGL 上下文中调用应用的按下/松开回调、状态更新及绘制函数，检查：

- 字母及方向键松开停止、大小写切换、多关节操作与反向抵消。
- 大小臂双向限位、底座/腕部回绕、夹爪全闭/全开。
- R 只复位相机，Home 复位机械臂并清空输入。
- 失焦状态清空路径、重新激活不继续旧动作、长暂停时间截断及计时器回绕。
- 实际帧缓冲中的初始/抬臂/夹爪全闭/全开/竖屏画面，以及图元矩阵栈恢复和 `GL_NO_ERROR`。

x64 Debug/Release 构建成功。Debug 所安装 freeglut 缺少 `freeglut_staticd.pdb`，仍有第三方 `LNK4099` 警告，不影响运行或本项目源码调试；不通过屏蔽默认库掩盖问题。

回调测试不等于人工完整键盘操作测试：尚未逐一人工验证真实 Alt-Tab、最小化恢复、特殊键盘布局及不同显卡。实际使用时可按住运动键后切换窗口，松开后切回，确认机械臂保持停止。
