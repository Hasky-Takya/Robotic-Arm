#define NOMINMAX
#include <GL/freeglut.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include "Geometry.h"
#include "RobotArm.h"

namespace
{
    constexpr double pi = 3.14159265358979323846;
    constexpr double defaultCameraYaw = 28.0;
    constexpr double defaultCameraPitch = 22.0;
    // 为大小臂完全抬起的姿态预留画面空间，而不只适配初始弯曲姿态。
    constexpr double cameraDistance = 15.5;
    constexpr double mouseSensitivity = 0.35;
    constexpr double cameraTargetY = 2.9;
    constexpr float tabletopY = 2.0f;
    constexpr float tabletopThickness = 0.22f;
    constexpr float tableWidth = 8.0f;
    constexpr float tableDepth = 4.8f;

    double cameraYaw = defaultCameraYaw;
    double cameraPitch = defaultCameraPitch;
    // 窄窗口适当拉远相机，防止桌面和伸出的夹爪被左右裁切。
    double aspectRatio = 1100.0 / 780.0;
    bool isDragging = false;
    int lastMouseX = 0;
    int lastMouseY = 0;
    bool renderFailed = false;
    int windowWidth = 1100;
    int windowHeight = 780;
    RobotArm::State armState;
    RobotArm::Controls armControls;
    std::uint32_t previousUpdateTime = 0;
    bool controlsActive = false;
#ifdef _WIN32
    HWND nativeWindow = nullptr;
#endif

    double viewDistance()
    {
        return cameraDistance * std::max(1.0, 1.25 / aspectRatio);
    }
}

// 初始化在创建窗口之后执行；场景使用基础光照，不需要贴图资源。
void initializeOpenGL()
{
    glClearColor(0.085f, 0.115f, 0.16f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    // 长方体采用非均匀缩放；自动归一化法线可避免缩放改变光照强度。
    glEnable(GL_NORMALIZE);
    glDisable(GL_TEXTURE_2D);
    glShadeModel(GL_SMOOTH);

    const GLfloat ambient[] = { 0.30f, 0.30f, 0.33f, 1.0f };
    const GLfloat diffuse[] = { 0.88f, 0.86f, 0.80f, 1.0f };
    const GLfloat specular[] = { 0.72f, 0.72f, 0.72f, 1.0f };
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
}

// 窗口首次显示或改变大小时更新投影，宽高至少为 1，避免除零。
void reshape(int width, int height)
{
    width = std::max(width, 1);
    height = std::max(height, 1);
    windowWidth = width;
    windowHeight = height;
    aspectRatio = static_cast<double>(width) / height;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // 裁剪面随观察距离同步拉远，极窄窗口也不会把整个场景裁掉。
    const double distance = viewDistance();
    gluPerspective(45.0, aspectRatio, std::max(0.1, distance - 25.0), distance + 25.0);
    glMatrixMode(GL_MODELVIEW);
    glutPostRedisplay();
}

// 复用轨道相机：只改变观察方向，不修改机械臂各关节的状态。
void applyCamera()
{
    const double yaw = cameraYaw * pi / 180.0;
    const double pitch = cameraPitch * pi / 180.0;
    const double distance = viewDistance();
    const double horizontalRadius = distance * std::cos(pitch);
    const double eyeX = horizontalRadius * std::sin(yaw);
    const double eyeY = cameraTargetY + distance * std::sin(pitch);
    const double eyeZ = horizontalRadius * std::cos(yaw);
    gluLookAt(eyeX, eyeY, eyeZ,
              0.0, cameraTargetY, 0.0,
              0.0, 1.0, 0.0);
}

// 桌面上表面是机械臂的安装平面，桌腿和脚垫一直延伸到地面。
void drawEnvironment()
{
    const Geometry::Material floor{ 0.23f, 0.28f, 0.34f, 0.04f, 8.0f };
    const Geometry::Material wood{ 0.48f, 0.31f, 0.17f, 0.12f, 20.0f };
    const Geometry::Material frame{ 0.19f, 0.24f, 0.29f, 0.40f, 48.0f };
    const Geometry::Material feet{ 0.055f, 0.065f, 0.075f, 0.02f, 8.0f };

    Geometry::setMaterial(floor);
    Geometry::drawBox(0.0f, -0.04f, 0.0f, 18.0f, 0.08f, 14.0f);
    Geometry::setMaterial(wood);
    Geometry::drawBox(0.0f, tabletopY - tabletopThickness / 2.0f, 0.0f,
                      tableWidth, tabletopThickness, tableDepth);

    const float undersideY = tabletopY - tabletopThickness;
    constexpr float footHeight = 0.08f;
    const float legHeight = undersideY - footHeight;
    Geometry::setMaterial(frame);
    // 桌面下的前后横梁和左右横梁，与四条桌腿组成固定支架。
    for (const float z : { -1.98f, 1.98f })
    {
        Geometry::drawBox(0.0f, undersideY - 0.16f, z, 7.30f, 0.30f, 0.14f);
    }
    for (const float x : { -3.55f, 3.55f })
    {
        Geometry::setMaterial(frame);
        Geometry::drawBox(x, undersideY - 0.16f, 0.0f, 0.14f, 0.30f, 4.08f);
        for (const float z : { -1.95f, 1.95f })
        {
            Geometry::setMaterial(frame);
            Geometry::drawBox(x, footHeight + legHeight / 2.0f, z,
                              0.24f, legHeight, 0.24f);
            Geometry::setMaterial(feet);
            Geometry::drawBox(x, footHeight / 2.0f, z, 0.34f, footHeight, 0.34f);
        }
    }
}

// 使用独立二维投影绘制英文帮助，避免 GLUT 位图字体不支持中文。
void drawStatus()
{
    glPushAttrib(GL_ENABLE_BIT | GL_CURRENT_BIT | GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, windowWidth, 0.0, windowHeight, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glColor3f(0.95f, 0.96f, 0.98f);

    char status[192];
    std::snprintf(status, sizeof(status),
                  "Base: %.1f  Shoulder: %.1f  Elbow: %.1f  Wrist: %.1f  Open: %.0f%%",
                  armState.baseAngle, armState.shoulderAngle, armState.elbowAngle,
                  armState.wristRoll, armState.opening * 100.0f);
    const char* lines[] = {
        "A/D: base   W/S: shoulder   Up/Down: elbow   Q/E: wrist   F/G: close/open",
        "Hold to move | Drag: orbit | R: camera reset | Home: arm reset | Esc: exit",
        status
    };
    for (int line = 0; line < 3; ++line)
    {
        glRasterPos2i(12, windowHeight - 22 - line * 20);
        glutBitmapString(GLUT_BITMAP_8_BY_13, reinterpret_cast<const unsigned char*>(lines[line]));
    }
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopAttrib();
}

// 绘制只读取关节状态；运动更新在定时回调中执行，不依赖显示刷新频率。
void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    applyCamera();

    // 在相机之后、模型变换之前设置世界方向光，拖动时灯不会跟着相机转。
    const GLfloat lightDirection[] = { -3.0f, 6.0f, 4.0f, 0.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightDirection);
    drawEnvironment();
    RobotArm::draw(armState, -1.5f, tabletopY, 0.0f);
    drawStatus();

    // 首帧及后续重绘若出现 OpenGL 错误，打印原因并以失败状态退出。
    const GLenum error = glGetError();
    if (error != GL_NO_ERROR)
    {
        std::fprintf(stderr, "OpenGL rendering error: 0x%04X\n", static_cast<unsigned>(error));
        renderFailed = true;
        glutLeaveMainLoop();
        return;
    }
    glutSwapBuffers();
}

void mouseButton(int button, int state, int x, int y)
{
    if (button != GLUT_LEFT_BUTTON) return;
    isDragging = (state == GLUT_DOWN);
    lastMouseX = x;
    lastMouseY = y;
}

void mouseMotion(int x, int y)
{
    if (!isDragging) return;
    const int deltaX = x - lastMouseX;
    const int deltaY = y - lastMouseY;
    cameraYaw = std::remainder(cameraYaw - deltaX * mouseSensitivity, 360.0);
    // 不允许观察方向与向上向量平行，避免相机在极点翻转。
    cameraPitch = std::clamp(cameraPitch + deltaY * mouseSensitivity, -80.0, 80.0);
    lastMouseX = x;
    lastMouseY = y;
    glutPostRedisplay();
}

bool hasControlFocus()
{
#ifdef _WIN32
    // 鼠标移出窗口仍可控制；切换前台窗口或最小化才暂停。
    return nativeWindow != nullptr && GetForegroundWindow() == nativeWindow && !IsIconic(nativeWindow);
#else
    return glutGet(GLUT_WINDOW_VISIBLE) != 0;
#endif
}

// 无符号时间差支持毫秒计时器回绕；失焦及恢复时清空输入，丢弃暂停时间。
void advanceControls(std::uint32_t now, bool active)
{
    const std::uint32_t elapsed = now - previousUpdateTime;
    previousUpdateTime = now;
    if (!active || !controlsActive)
    {
        armControls.clear();
        if (!active) isDragging = false;
        controlsActive = active;
        return;
    }
    const double seconds = std::min(static_cast<double>(elapsed) / 1000.0, 0.05);
    if (RobotArm::update(armState, armControls, seconds)) glutPostRedisplay();
}

void synchronizeControls()
{
    advanceControls(static_cast<std::uint32_t>(glutGet(GLUT_ELAPSED_TIME)), hasControlFocus());
}

void updateTimer(int)
{
    synchronizeControls();
    glutTimerFunc(16, updateTimer, 0);
}

void keyboard(unsigned char key, int, int)
{
    // 先结算旧按键状态对应的时间，再处理按键边沿，避免把按下之前的时间算进去。
    synchronizeControls();
    if (key == 27)
    {
        armControls.clear();
        glutLeaveMainLoop();
    }
    else if (key == 'r' || key == 'R')
    {
        cameraYaw = defaultCameraYaw;
        cameraPitch = defaultCameraPitch;
        isDragging = false;
        glutPostRedisplay();
    }
    else if (controlsActive)
    {
        RobotArm::setLetter(armControls, key, true);
    }
}

void keyboardUp(unsigned char key, int, int)
{
    synchronizeControls();
    // 按住字母后再按 Ctrl，freeglut 可能用控制字符报告松键；仍需释放原动作。
    if (key >= 1 && key <= 26) key = static_cast<unsigned char>('a' + key - 1);
    RobotArm::setLetter(armControls, key, false);
}

void specialKeyboard(int key, int, int)
{
    synchronizeControls();
    if (!controlsActive) return;
    if (key == GLUT_KEY_HOME)
    {
        armControls.clear();
        armState = RobotArm::State{};
        glutPostRedisplay();
    }
    else if (key == GLUT_KEY_UP)
    {
        armControls.set(RobotArm::Action::ElbowUp, true);
    }
    else if (key == GLUT_KEY_DOWN)
    {
        armControls.set(RobotArm::Action::ElbowDown, true);
    }
}

void specialKeyboardUp(int key, int, int)
{
    synchronizeControls();
    if (key == GLUT_KEY_UP) armControls.set(RobotArm::Action::ElbowUp, false);
    else if (key == GLUT_KEY_DOWN) armControls.set(RobotArm::Action::ElbowDown, false);
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1100, 780);
    glutInitWindowPosition(80, 60);
    glutCreateWindow("OpenGL Robot Arm - Phase 2 | Keyboard joint control");
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

    initializeOpenGL();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouseButton);
    glutMotionFunc(mouseMotion);
    glutKeyboardFunc(keyboard);
    glutKeyboardUpFunc(keyboardUp);
    glutSpecialFunc(specialKeyboard);
    glutSpecialUpFunc(specialKeyboardUp);
    glutIgnoreKeyRepeat(1);
#ifdef _WIN32
    nativeWindow = WindowFromDC(wglGetCurrentDC());
    if (nativeWindow == nullptr)
    {
        std::fputs("Cannot identify the OpenGL window for keyboard focus checks.\n", stderr);
        glutDestroyWindow(glutGetWindow());
        return EXIT_FAILURE;
    }
#endif
    previousUpdateTime = static_cast<std::uint32_t>(glutGet(GLUT_ELAPSED_TIME));
    controlsActive = hasControlFocus();
    glutTimerFunc(16, updateTimer, 0);
    std::puts("Phase 2: hold A/D base, W/S shoulder, Up/Down elbow, Q/E wrist, F/G close/open.");
    std::puts("Drag: orbit | R: camera reset | Home: arm reset | Esc: exit. Focus loss stops movement.");
    glutMainLoop();
    return renderFailed ? EXIT_FAILURE : EXIT_SUCCESS;
}
