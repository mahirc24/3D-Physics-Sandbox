// viewer_main.cpp — real-time 3D viewer for the physics sandbox.
//
// Opens a window and renders the running simulation live. You can orbit the
// camera, click the body *under the cursor* to grab and drag it (via a real
// point-to-point constraint), shoot objects into the scene, toggle a debug
// wireframe of collision geometry and contacts, pause/step, and switch
// scenarios — all without pausing the physics unless you ask it to.
//
// Uses legacy fixed-function OpenGL so it needs no shader loader and builds with
// just GLFW + the platform OpenGL library.

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <GLFW/glfw3.h>

#include "DebugDrawer.h"
#include "PhysicsWorld.h"
#include "RayCaster.h"
#include "Scenarios.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace pbx;

namespace {

constexpr float kPi = 3.14159265358979323846f;
const char* kScenarios[] = {"stack", "bounce", "projectile", "pendulum",
                            "friction_ramp", "domino", "ccd", "raycast", "perf"};
constexpr int kNumScenarios = 9;

// ---- orbit camera -----------------------------------------------------------
struct OrbitCamera {
    btVector3 target{0, 2, 0};
    float dist = 16.f, yaw = 0.7f, pitch = 0.35f, fovy = 60.f;

    btVector3 eye() const {
        const float cp = std::cos(pitch), sp = std::sin(pitch);
        const float cy = std::cos(yaw), sy = std::sin(yaw);
        return target + btVector3(dist * cp * sy, dist * sp, dist * cp * cy);
    }
    btVector3 forward() const { return (target - eye()).normalized(); }
    btVector3 right() const { return forward().cross(btVector3(0, 1, 0)).normalized(); }
    btVector3 up() const { return right().cross(forward()); }
};

// ---- application state ------------------------------------------------------
struct App {
    std::unique_ptr<PhysicsWorld> world;
    std::string scenario = "stack";
    OrbitCamera cam;

    bool paused = false;
    bool stepOnce = false;
    bool showDebug = false;

    // Interaction state.
    enum class Drag { None, Orbit, Body } drag = Drag::None;
    int    grabbedId = -1;
    double lastX = 0, lastY = 0;
    float  pickDist = 0.f;

    int winW = 1280, winH = 800;

    void rebuild(const std::string& name) {
        scenario = name;
        WorldConfig cfg;  // defaults: 1/60 step, 1/120 fixed, 10 substeps
        world = std::make_unique<PhysicsWorld>(cfg);
        buildScenario(name, *world, ScenarioParams{});
        paused = false;
    }

    // World-space ray from the cursor position.
    void screenRay(double mx, double my, btVector3& origin, btVector3& dir) const {
        const float ndcx = 2.f * float(mx) / winW - 1.f;
        const float ndcy = 1.f - 2.f * float(my) / winH;
        const float t = std::tan(cam.fovy * 0.5f * kPi / 180.f);
        const float aspect = float(winW) / float(winH);
        origin = cam.eye();
        dir = (cam.forward() + cam.right() * (ndcx * t * aspect) +
               cam.up() * (ndcy * t)).normalized();
    }
};

// ---- immediate-mode meshes --------------------------------------------------
void drawCube(float hx, float hy, float hz) {
    glBegin(GL_QUADS);
    // +X, -X, +Y, -Y, +Z, -Z faces with outward normals.
    glNormal3f(1, 0, 0);
    glVertex3f(hx, -hy, -hz); glVertex3f(hx, hy, -hz); glVertex3f(hx, hy, hz); glVertex3f(hx, -hy, hz);
    glNormal3f(-1, 0, 0);
    glVertex3f(-hx, -hy, hz); glVertex3f(-hx, hy, hz); glVertex3f(-hx, hy, -hz); glVertex3f(-hx, -hy, -hz);
    glNormal3f(0, 1, 0);
    glVertex3f(-hx, hy, -hz); glVertex3f(-hx, hy, hz); glVertex3f(hx, hy, hz); glVertex3f(hx, hy, -hz);
    glNormal3f(0, -1, 0);
    glVertex3f(-hx, -hy, hz); glVertex3f(-hx, -hy, -hz); glVertex3f(hx, -hy, -hz); glVertex3f(hx, -hy, hz);
    glNormal3f(0, 0, 1);
    glVertex3f(-hx, -hy, hz); glVertex3f(hx, -hy, hz); glVertex3f(hx, hy, hz); glVertex3f(-hx, hy, hz);
    glNormal3f(0, 0, -1);
    glVertex3f(hx, -hy, -hz); glVertex3f(-hx, -hy, -hz); glVertex3f(-hx, hy, -hz); glVertex3f(hx, hy, -hz);
    glEnd();
}

void drawSphere(float r, int slices = 18, int stacks = 14) {
    for (int i = 0; i < stacks; ++i) {
        const float t0 = kPi * (-0.5f + float(i) / stacks);
        const float t1 = kPi * (-0.5f + float(i + 1) / stacks);
        const float y0 = std::sin(t0), y1 = std::sin(t1);
        const float r0 = std::cos(t0), r1 = std::cos(t1);
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= slices; ++j) {
            const float a = 2.f * kPi * float(j) / slices;
            const float ca = std::cos(a), sa = std::sin(a);
            glNormal3f(r0 * ca, y0, r0 * sa); glVertex3f(r * r0 * ca, r * y0, r * r0 * sa);
            glNormal3f(r1 * ca, y1, r1 * sa); glVertex3f(r * r1 * ca, r * y1, r * r1 * sa);
        }
        glEnd();
    }
}

void drawCylinder(float r, float halfH, int slices = 20) {
    glBegin(GL_QUAD_STRIP);
    for (int j = 0; j <= slices; ++j) {
        const float a = 2.f * kPi * float(j) / slices;
        const float ca = std::cos(a), sa = std::sin(a);
        glNormal3f(ca, 0, sa);
        glVertex3f(r * ca, halfH, r * sa);
        glVertex3f(r * ca, -halfH, r * sa);
    }
    glEnd();
    for (int sgn = -1; sgn <= 1; sgn += 2) {  // caps
        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(0, float(sgn), 0);
        glVertex3f(0, sgn * halfH, 0);
        for (int j = 0; j <= slices; ++j) {
            const float a = 2.f * kPi * float(j) / slices * sgn;
            glVertex3f(r * std::cos(a), sgn * halfH, r * std::sin(a));
        }
        glEnd();
    }
}

void drawGround() {
    glDisable(GL_LIGHTING);
    const int n = 20;
    const float s = 1.0f;
    // Floor fill.
    glColor3f(0.16f, 0.17f, 0.19f);
    glBegin(GL_QUADS);
    glVertex3f(-n * s, 0, -n * s); glVertex3f(n * s, 0, -n * s);
    glVertex3f(n * s, 0, n * s);  glVertex3f(-n * s, 0, n * s);
    glEnd();
    // Grid lines.
    glColor3f(0.28f, 0.30f, 0.34f);
    glBegin(GL_LINES);
    for (int i = -n; i <= n; ++i) {
        glVertex3f(i * s, 0.001f, -n * s); glVertex3f(i * s, 0.001f, n * s);
        glVertex3f(-n * s, 0.001f, i * s); glVertex3f(n * s, 0.001f, i * s);
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

// Draw a body using its Bullet collision shape.
void drawShape(const btCollisionShape* shape) {
    switch (shape->getShapeType()) {
        case BOX_SHAPE_PROXYTYPE: {
            const auto* b = static_cast<const btBoxShape*>(shape);
            const btVector3 h = b->getHalfExtentsWithMargin();
            drawCube(h.x(), h.y(), h.z());
            break;
        }
        case SPHERE_SHAPE_PROXYTYPE:
            drawSphere(static_cast<const btSphereShape*>(shape)->getRadius());
            break;
        case CAPSULE_SHAPE_PROXYTYPE: {
            const auto* c = static_cast<const btCapsuleShape*>(shape);
            drawCylinder(c->getRadius(), c->getHalfHeight());
            break;
        }
        case CYLINDER_SHAPE_PROXYTYPE: {
            const auto* c = static_cast<const btCylinderShape*>(shape);
            const btVector3 h = c->getHalfExtentsWithMargin();
            drawCylinder(h.x(), h.y());
            break;
        }
        default:  // static plane etc. -> drawn as the ground grid
            break;
    }
}

// ---- rendering --------------------------------------------------------------
void render(App& app) {
    int fbw, fbh;
    // (framebuffer size handled by caller via glViewport)
    glClearColor(0.09f, 0.10f, 0.12f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Projection.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const float aspect = float(app.winW) / float(app.winH);
    const float top = 0.05f * std::tan(app.cam.fovy * 0.5f * kPi / 180.f);
    glFrustum(-top * aspect, top * aspect, -top, top, 0.05f, 500.f);

    // View.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const btVector3 e = app.cam.eye(), c = app.cam.target, u = app.cam.up();
    float m[16];
    {
        const btVector3 f = (c - e).normalized();
        const btVector3 s = f.cross(u).normalized();
        const btVector3 uu = s.cross(f);
        m[0] = s.x();  m[4] = s.y();  m[8]  = s.z();  m[12] = -s.dot(e);
        m[1] = uu.x(); m[5] = uu.y(); m[9]  = uu.z(); m[13] = -uu.dot(e);
        m[2] = -f.x(); m[6] = -f.y(); m[10] = -f.z(); m[14] = f.dot(e);
        m[3] = 0;      m[7] = 0;      m[11] = 0;      m[15] = 1;
    }
    glLoadMatrixf(m);

    // Light (positioned in world space after the view is set).
    const GLfloat lightPos[4] = {12.f, 30.f, 18.f, 1.f};
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    drawGround();

    // Bodies.
    for (const auto& rec : app.world->bodies()) {
        if (!rec || !rec->body) continue;
        const btCollisionShape* shape = rec->body->getCollisionShape();
        if (shape->getShapeType() == STATIC_PLANE_PROXYTYPE) continue;  // = ground

        glPushMatrix();
        float t[16];
        rec->body->getWorldTransform().getOpenGLMatrix(t);
        glMultMatrixf(t);

        const bool grabbed = app.world->hasPick() && app.drag == App::Drag::Body &&
                             rec->id == app.grabbedId;
        if (grabbed)                 glColor3f(1.0f, 0.75f, 0.15f);   // highlighted
        else if (rec->mass == 0)     glColor3f(0.55f, 0.55f, 0.58f);  // static
        else                         glColor3f(0.26f, 0.53f, 0.82f);  // dynamic
        drawShape(shape);
        glPopMatrix();
    }

    // Debug geometry (wireframe + contact points).
    if (app.showDebug) {
        DebugDrawer dd;
        dd.capture(*app.world);
        glDisable(GL_LIGHTING);
        glBegin(GL_LINES);
        for (const auto& l : dd.lines()) {
            glColor3f(l.color.x(), l.color.y(), l.color.z());
            glVertex3f(l.from.x(), l.from.y(), l.from.z());
            glVertex3f(l.to.x(), l.to.y(), l.to.z());
        }
        glEnd();
        glEnable(GL_LIGHTING);
    }
    (void)fbw; (void)fbh;
}

// ---- input handlers (dispatched from GLFW callbacks) ------------------------
void shoot(App& app) {
    BodyDesc d;
    d.name = "shot";
    d.type = ShapeType::Sphere;
    d.radius = 0.25f;
    d.mass = 2.0f;
    d.friction = 0.5f;
    d.restitution = 0.3f;
    d.position = app.cam.eye() + app.cam.forward() * 1.5f;
    d.linearVelocity = app.cam.forward() * 30.0f;
    d.ccd = true;
    app.world->spawn(d);
}

void onMouseButton(GLFWwindow* w, int button, int action, int) {
    App& app = *static_cast<App*>(glfwGetWindowUserPointer(w));
    glfwGetCursorPos(w, &app.lastX, &app.lastY);

    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        app.drag = (action == GLFW_PRESS) ? App::Drag::Orbit : App::Drag::None;
        return;
    }
    if (button != GLFW_MOUSE_BUTTON_LEFT) return;

    if (action == GLFW_PRESS) {
        btVector3 o, dir;
        app.screenRay(app.lastX, app.lastY, o, dir);
        RayCaster rc(*app.world);
        RayHit hit = rc.closest(o, o + dir * 500.f);
        const BodyRecord* rec = hit.hit ? app.world->record(hit.bodyId) : nullptr;
        if (rec && rec->mass > 0) {                 // grab a dynamic body
            app.world->createPickConstraint(hit.bodyId, hit.point);
            app.grabbedId = hit.bodyId;
            app.pickDist = hit.distance;
            app.drag = App::Drag::Body;
        } else {                                     // empty space -> orbit
            app.drag = App::Drag::Orbit;
        }
    } else {  // release
        if (app.drag == App::Drag::Body) {
            app.world->clearPick();
            app.grabbedId = -1;
        }
        app.drag = App::Drag::None;
    }
}

void onCursor(GLFWwindow* w, double x, double y) {
    App& app = *static_cast<App*>(glfwGetWindowUserPointer(w));
    const double dx = x - app.lastX, dy = y - app.lastY;
    app.lastX = x; app.lastY = y;

    if (app.drag == App::Drag::Orbit) {
        app.cam.yaw -= float(dx) * 0.005f;
        app.cam.pitch += float(dy) * 0.005f;
        const float lim = kPi * 0.49f;
        if (app.cam.pitch > lim) app.cam.pitch = lim;
        if (app.cam.pitch < -lim) app.cam.pitch = -lim;
    } else if (app.drag == App::Drag::Body && app.world->hasPick()) {
        btVector3 o, dir;
        app.screenRay(x, y, o, dir);
        app.world->updatePick(o + dir * app.pickDist);
    }
}

void onScroll(GLFWwindow* w, double, double dy) {
    App& app = *static_cast<App*>(glfwGetWindowUserPointer(w));
    app.cam.dist *= (1.f - float(dy) * 0.1f);
    if (app.cam.dist < 2.f) app.cam.dist = 2.f;
    if (app.cam.dist > 120.f) app.cam.dist = 120.f;
}

void onKey(GLFWwindow* w, int key, int, int action, int) {
    if (action != GLFW_PRESS && action != GLFW_REPEAT) return;
    App& app = *static_cast<App*>(glfwGetWindowUserPointer(w));
    switch (key) {
        case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(w, 1); break;
        case GLFW_KEY_SPACE:  app.paused = !app.paused; break;
        case GLFW_KEY_N:      app.stepOnce = true; break;
        case GLFW_KEY_G:      app.showDebug = !app.showDebug; break;
        case GLFW_KEY_F:      shoot(app); break;
        case GLFW_KEY_R:      app.rebuild(app.scenario); break;
        default:
            if (key >= GLFW_KEY_1 && key <= GLFW_KEY_9) {
                const int idx = key - GLFW_KEY_1;
                if (idx < kNumScenarios) app.rebuild(kScenarios[idx]);
            }
            break;
    }
}

void onFramebuffer(GLFWwindow* w, int fbw, int fbh) {
    glViewport(0, 0, fbw, fbh);
    (void)w;
}

void printControls() {
    std::puts(
        "physics-sandbox viewer controls:\n"
        "  Left-drag empty space .... orbit camera\n"
        "  Left-click + drag a body . grab and move it (the body under the cursor)\n"
        "  Right-drag ............... orbit camera\n"
        "  Scroll ................... zoom\n"
        "  F ........................ shoot a sphere from the camera\n"
        "  G ........................ toggle debug wireframe + contact points\n"
        "  Space .................... pause / resume\n"
        "  N ........................ single step (while paused)\n"
        "  R ........................ reset current scenario\n"
        "  1-9 ...................... switch scenario\n"
        "  Esc ...................... quit");
}

}  // namespace

int main(int argc, char** argv) {
    std::string scenario = "stack";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--scenario") == 0 && i + 1 < argc)
            scenario = argv[++i];
        else if (std::strcmp(argv[i], "--list") == 0) {
            for (auto& s : listScenarios()) std::printf("  %s\n", s.first.c_str());
            return 0;
        }
    }

    if (!glfwInit()) { std::fprintf(stderr, "Failed to init GLFW\n"); return 1; }

    App app;
    GLFWwindow* window = glfwCreateWindow(app.winW, app.winH,
                                          "physics-sandbox viewer", nullptr, nullptr);
    if (!window) { std::fprintf(stderr, "Failed to create window\n"); glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // vsync
    glfwSetWindowUserPointer(window, &app);
    glfwSetMouseButtonCallback(window, onMouseButton);
    glfwSetCursorPosCallback(window, onCursor);
    glfwSetScrollCallback(window, onScroll);
    glfwSetKeyCallback(window, onKey);
    glfwSetFramebufferSizeCallback(window, onFramebuffer);

    // GL state.
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
    const GLfloat amb[4] = {0.25f, 0.25f, 0.28f, 1.f};
    const GLfloat dif[4] = {0.85f, 0.85f, 0.85f, 1.f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);

    app.rebuild(scenario);
    printControls();

    double last = glfwGetTime(), fpsT = last;
    int frames = 0;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        glfwGetFramebufferSize(window, /*out*/ nullptr, nullptr);
        int fbw, fbh; glfwGetFramebufferSize(window, &fbw, &fbh);
        glViewport(0, 0, fbw, fbh);
        glfwGetWindowSize(window, &app.winW, &app.winH);

        // Advance physics (fixed real-time-ish stepping).
        if (!app.paused || app.stepOnce) {
            app.world->step();
            app.stepOnce = false;
        }

        render(app);
        glfwSwapBuffers(window);

        // Title HUD.
        if (++frames, glfwGetTime() - fpsT >= 0.5) {
            char title[256];
            std::snprintf(title, sizeof(title),
                "physics-sandbox | %s | %s | bodies=%zu | %.0f fps%s",
                app.scenario.c_str(), app.paused ? "PAUSED" : "running",
                app.world->bodies().size(),
                frames / (glfwGetTime() - fpsT),
                app.showDebug ? " | debug" : "");
            glfwSetWindowTitle(window, title);
            frames = 0; fpsT = glfwGetTime();
        }
    }

    app.world.reset();  // release physics before GL/GLFW teardown
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
