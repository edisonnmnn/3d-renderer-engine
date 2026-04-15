#include <SDL2/SDL.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include "math/vec3.h"
#include "math/mat4.h"

// ── Constants ────────────────────────────────────────────────────────────────

const int   WIDTH            = 800;
const int   HEIGHT           = 600;
const float FOV              = 5.0f;
const float GRAVITY          = 0.0f;   // zero-g — objects bounce forever
const float CUBE_RESTITUTION = 0.95f;
const float BALL_RESTITUTION = 0.95f;
const float BOUND            = 5.5f;
const float CAM_RADIUS       = 11.0f;
const float CAM_HEIGHT       = 5.0f;
const float CAM_SPEED        = 0.35f;

// ── Drawing ──────────────────────────────────────────────────────────────────

void drawPixel(uint32_t* pixels, int x, int y, uint32_t color) {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT)
        pixels[y * WIDTH + x] = color;
}

void drawLine(uint32_t* pixels, int x0, int y0, int x1, int y1, uint32_t color) {
    int dx  = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx  = (x1 > x0) ? 1 : -1, sy = (y1 > y0) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        drawPixel(pixels, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

void fillTriangle(uint32_t* pixels,
                  int x0, int y0, int x1, int y1, int x2, int y2,
                  uint32_t color) {
    if (y0 > y1) { std::swap(x0,x1); std::swap(y0,y1); }
    if (y0 > y2) { std::swap(x0,x2); std::swap(y0,y2); }
    if (y1 > y2) { std::swap(x1,x2); std::swap(y1,y2); }
    int total_h = y2 - y0;
    if (total_h == 0) return;
    for (int y = y0; y <= y2; y++) {
        bool  lower = (y > y1) || (y1 == y0);
        int   seg_h = lower ? (y2 - y1) : (y1 - y0);
        if (seg_h == 0) continue;
        float alpha = (float)(y - y0) / total_h;
        float beta  = lower ? (float)(y - y1) / seg_h : (float)(y - y0) / seg_h;
        int ax = x0 + (int)((x2 - x0) * alpha);
        int bx = lower ? x1 + (int)((x2 - x1) * beta) : x0 + (int)((x1 - x0) * beta);
        if (ax > bx) std::swap(ax, bx);
        for (int x = ax; x <= bx; x++) drawPixel(pixels, x, y, color);
    }
}

void fillQuad(uint32_t* pixels,
              int x0, int y0, int x1, int y1,
              int x2, int y2, int x3, int y3, uint32_t color) {
    fillTriangle(pixels, x0,y0, x1,y1, x2,y2, color);
    fillTriangle(pixels, x0,y0, x2,y2, x3,y3, color);
}

// ── Color ────────────────────────────────────────────────────────────────────

uint32_t makeColor(float r, float g, float b) {
    r = std::max(0.0f, std::min(1.0f, r));
    g = std::max(0.0f, std::min(1.0f, g));
    b = std::max(0.0f, std::min(1.0f, b));
    return (0xFF << 24) | ((uint8_t)(r*255) << 16) | ((uint8_t)(g*255) << 8) | (uint8_t)(b*255);
}

// Vertical gradient background — dark navy top, deeper navy bottom
void clearBackground(uint32_t* pixels) {
    // top: #1A2A3F  bottom: #0C1520
    const float tr=0.102f, tg=0.165f, tb=0.247f;
    const float br=0.047f, bg=0.082f, bb=0.125f;
    for (int y = 0; y < HEIGHT; y++) {
        float t = (float)y / (HEIGHT - 1);
        uint32_t col = makeColor(tr+(br-tr)*t, tg+(bg-tg)*t, tb+(bb-tb)*t);
        for (int x = 0; x < WIDTH; x++)
            pixels[y * WIDTH + x] = col;
    }
}

// Two-light Phong shading for cube faces
// Key light: bright from upper-right.  Fill light: soft from lower-left (no specular).
uint32_t phongShade(Vec3 N, Vec3 face_center, Vec3 light_pos, Vec3 cam_pos, uint32_t base) {
    // Key light
    Vec3  L1   = (light_pos - face_center).normalized();
    Vec3  V    = (cam_pos   - face_center).normalized();
    float NdL1 = N.dot(L1);
    Vec3  R    = N * (2.0f * NdL1) - L1;
    float diff = std::max(0.0f, NdL1);
    float spec = std::pow(std::max(0.0f, R.dot(V)), 64.0f);

    // Fill light (fixed direction, no specular — lifts shadows)
    Vec3  L2   = Vec3(-0.6f, -0.8f, 0.4f).normalized();
    float fill = std::max(0.0f, N.dot(L2)) * 0.22f;

    float i = 0.18f + 0.60f * diff + 0.40f * spec + fill;
    i = std::min(1.15f, i);  // allow slight overexposure at specular peak

    float r = ((base >> 16) & 0xFF) / 255.0f;
    float g = ((base >>  8) & 0xFF) / 255.0f;
    float b = ( base        & 0xFF) / 255.0f;
    return makeColor(r*i, g*i, b*i);
}

// ── Geometry ─────────────────────────────────────────────────────────────────

struct Face { int v[4]; Vec3 normal; };

static const Vec3 CUBE_VERTS[8] = {
    {-1, 1, 1},{-1,-1, 1},{ 1,-1, 1},{ 1, 1, 1},
    {-1, 1,-1},{-1,-1,-1},{ 1,-1,-1},{ 1, 1,-1},
};
static const Face CUBE_FACES[6] = {
    {{0,1,2,3},{ 0, 0, 1}}, {{7,6,5,4},{ 0, 0,-1}},
    {{4,5,1,0},{-1, 0, 0}}, {{3,2,6,7},{ 1, 0, 0}},
    {{4,0,3,7},{ 0, 1, 0}}, {{1,5,6,2},{ 0,-1, 0}},
};

// ── Projection ───────────────────────────────────────────────────────────────

Vec3 project(const Vec3& v) {
    float aspect = (float)WIDTH / (float)HEIGHT;
    float px = ( v.x / (v.z + FOV)) * HEIGHT * aspect * 0.5f + WIDTH  * 0.5f;
    float py = (-v.y / (v.z + FOV)) * HEIGHT           * 0.5f + HEIGHT * 0.5f;  // negate y: screen y grows down
    return Vec3(px, py, v.z);
}

// ── World environment ─────────────────────────────────────────────────────────

// Draw the 12 edges of the bounding box so walls are visible
void drawBoundingBox(uint32_t* pixels, const Mat4& view) {
    const float B = BOUND;
    Vec3 corners[8] = {
        {-B,-B,-B},{-B,-B, B},{-B, B,-B},{-B, B, B},
        { B,-B,-B},{ B,-B, B},{ B, B,-B},{ B, B, B},
    };
    int edges[12][2] = {
        {0,1},{2,3},{4,5},{6,7},
        {0,2},{1,3},{4,6},{5,7},
        {0,4},{1,5},{2,6},{3,7},
    };
    Vec3 scrn[8]; bool ok[8];
    for (int i = 0; i < 8; i++) {
        Vec3 c = view * corners[i];
        ok[i]   = (c.z + FOV > 0.1f);
        scrn[i] = ok[i] ? project(c) : Vec3(0,0,0);
    }
    uint32_t col = makeColor(0.20f, 0.32f, 0.48f);
    for (auto& e : edges)
        if (ok[e[0]] && ok[e[1]])
            drawLine(pixels, (int)scrn[e[0]].x,(int)scrn[e[0]].y,
                             (int)scrn[e[1]].x,(int)scrn[e[1]].y, col);
}

// Draw a grid on the floor (y = -BOUND)
void drawFloorGrid(uint32_t* pixels, const Mat4& view) {
    const float B = BOUND;
    const float y = -B;
    uint32_t col = makeColor(0.12f, 0.20f, 0.30f);
    for (float x = -B; x <= B + 0.01f; x += 1.0f) {
        Vec3 a = view * Vec3(x, y, -B), b = view * Vec3(x, y, B);
        if (a.z+FOV > 0.1f && b.z+FOV > 0.1f) {
            Vec3 pa = project(a), pb = project(b);
            drawLine(pixels,(int)pa.x,(int)pa.y,(int)pb.x,(int)pb.y,col);
        }
    }
    for (float z = -B; z <= B + 0.01f; z += 1.0f) {
        Vec3 a = view * Vec3(-B, y, z), b = view * Vec3(B, y, z);
        if (a.z+FOV > 0.1f && b.z+FOV > 0.1f) {
            Vec3 pa = project(a), pb = project(b);
            drawLine(pixels,(int)pa.x,(int)pa.y,(int)pb.x,(int)pb.y,col);
        }
    }
}

// ── Physics ──────────────────────────────────────────────────────────────────

struct RigidBody {
    Vec3  pos, vel, omega;
    float angleX = 0, angleY = 0;
    float mass   = 1.0f;
};

void physicsUpdate(RigidBody& b, float dt) {
    b.vel.y += GRAVITY * dt;
    b.pos    = b.pos + b.vel * dt;
    b.angleX += b.omega.x * dt;
    b.angleY += b.omega.y * dt;
    b.omega   = b.omega * 0.996f;
    const float ext = 1.0f;
    auto bounce = [&](float& p, float& v, float lo, float hi) {
        if (p-ext < lo) { p = lo+ext; v = -v * CUBE_RESTITUTION; }
        if (p+ext > hi) { p = hi-ext; v = -v * CUBE_RESTITUTION; }
    };
    bounce(b.pos.x, b.vel.x, -BOUND, BOUND);
    bounce(b.pos.y, b.vel.y, -BOUND, BOUND);
    bounce(b.pos.z, b.vel.z, -BOUND, BOUND);
}

void resolveCubeCube(RigidBody& a, RigidBody& b) {
    Vec3 d = a.pos - b.pos;
    float px = 2.0f - std::abs(d.x), py = 2.0f - std::abs(d.y), pz = 2.0f - std::abs(d.z);
    if (px<=0||py<=0||pz<=0) return;
    Vec3 n; float ov;
    if (px<py&&px<pz) { n=Vec3(d.x>0?1.f:-1.f,0,0); ov=px; }
    else if (py<pz)   { n=Vec3(0,d.y>0?1.f:-1.f,0); ov=py; }
    else              { n=Vec3(0,0,d.z>0?1.f:-1.f); ov=pz; }
    a.pos=a.pos+n*(ov*0.5f); b.pos=b.pos-n*(ov*0.5f);
    Vec3 rv=a.vel-b.vel; float vn=rv.dot(n);
    if (vn>0) return;
    float j=-(1.55f)*vn/(1.f/a.mass+1.f/b.mass);
    a.vel=a.vel+n*(j/a.mass); b.vel=b.vel-n*(j/b.mass);
    a.omega.x+=n.y*j*0.5f; a.omega.y+=n.x*j*0.5f;
    b.omega.x-=n.y*j*0.5f; b.omega.y-=n.x*j*0.5f;
}

// ── Ball ─────────────────────────────────────────────────────────────────────

struct Ball {
    Vec3     pos, vel;
    float    radius = 0.55f;
    float    mass   = 0.35f;
    uint32_t color  = 0xFFFFFFFF;
};

void updateBall(Ball& b, float dt) {
    b.vel.y += GRAVITY * dt;
    b.pos    = b.pos + b.vel * dt;
    auto bounce = [&](float& p, float& v, float lo, float hi) {
        if (p-b.radius < lo) { p=lo+b.radius; v=-v*BALL_RESTITUTION; }
        if (p+b.radius > hi) { p=hi-b.radius; v=-v*BALL_RESTITUTION; }
    };
    bounce(b.pos.x, b.vel.x, -BOUND, BOUND);
    bounce(b.pos.y, b.vel.y, -BOUND, BOUND);
    bounce(b.pos.z, b.vel.z, -BOUND, BOUND);
}

void resolveSphereCube(Ball& ball, RigidBody& cube) {
    // Closest point on cube AABB to ball center
    float cx = std::max(cube.pos.x-1.f, std::min(ball.pos.x, cube.pos.x+1.f));
    float cy = std::max(cube.pos.y-1.f, std::min(ball.pos.y, cube.pos.y+1.f));
    float cz = std::max(cube.pos.z-1.f, std::min(ball.pos.z, cube.pos.z+1.f));
    Vec3  diff  = ball.pos - Vec3(cx,cy,cz);
    float dist  = diff.length();
    if (dist >= ball.radius || dist < 0.0001f) return;
    Vec3  n     = diff.normalized();
    float pen   = ball.radius - dist;
    ball.pos    = ball.pos + n * (pen * 0.6f);
    cube.pos    = cube.pos - n * (pen * 0.4f);
    Vec3  rv    = ball.vel - cube.vel;
    float vn    = rv.dot(n);
    if (vn > 0) return;
    float j = -(1.6f) * vn / (1.f/ball.mass + 1.f/cube.mass);
    ball.vel    = ball.vel + n*(j/ball.mass);
    cube.vel    = cube.vel - n*(j/cube.mass);
    cube.omega.x += n.y * j * 0.6f;
    cube.omega.y += n.x * j * 0.6f;
}

void resolveSphereSphere(Ball& a, Ball& b) {
    Vec3  d    = a.pos - b.pos;
    float dist = d.length();
    float rsum = a.radius + b.radius;
    if (dist >= rsum || dist < 0.0001f) return;
    Vec3  n    = d.normalized();
    float pen  = rsum - dist;
    a.pos = a.pos + n*(pen*0.5f); b.pos = b.pos - n*(pen*0.5f);
    Vec3  rv   = a.vel - b.vel;
    float vn   = rv.dot(n);
    if (vn > 0) return;
    float j = -(1.7f) * vn / (1.f/a.mass + 1.f/b.mass);
    a.vel = a.vel + n*(j/a.mass);
    b.vel = b.vel - n*(j/b.mass);
}

// Sphere rendering with per-pixel Phong shading in camera space
void renderBall(uint32_t* pixels, const Ball& ball,
                const Mat4& view, Vec3 light_pos) {
    Vec3  cv     = view * ball.pos;
    float zdepth = cv.z + FOV;
    if (zdepth <= 0.1f) return;

    Vec3  proj = project(cv);
    int   cx   = (int)proj.x, cy = (int)proj.y;
    float aspect = (float)WIDTH / (float)HEIGHT;
    int   rx   = (int)(ball.radius * HEIGHT * aspect * 0.5f / zdepth);
    int   ry   = (int)(ball.radius * HEIGHT           * 0.5f / zdepth);
    if (ry < 1) return;

    // Transform light direction to camera space (rotation only, no translation)
    Vec3 Lw(
        (light_pos - ball.pos).x,
        (light_pos - ball.pos).y,
        (light_pos - ball.pos).z
    );
    float Ll = Lw.length(); if (Ll < 0.001f) return; Lw = Lw * (1.0f/Ll);
    Vec3 Lc(
        view.m[0][0]*Lw.x + view.m[0][1]*Lw.y + view.m[0][2]*Lw.z,
        view.m[1][0]*Lw.x + view.m[1][1]*Lw.y + view.m[1][2]*Lw.z,
        view.m[2][0]*Lw.x + view.m[2][1]*Lw.y + view.m[2][2]*Lw.z
    );
    // View direction from ball center toward camera (camera at origin in cam space)
    Vec3 Vc = (-cv).normalized();

    float base_r = ((ball.color >> 16) & 0xFF) / 255.0f;
    float base_g = ((ball.color >>  8) & 0xFF) / 255.0f;
    float base_b = ( ball.color        & 0xFF) / 255.0f;

    for (int dy = -ry; dy <= ry; dy++) {
        float ny = (float)dy / ry;
        for (int dx = -rx; dx <= rx; dx++) {
            float nx  = (float)dx / rx;
            float nz2 = 1.0f - nx*nx - ny*ny;
            if (nz2 < 0) continue;
            Vec3 N(nx, -ny, std::sqrt(nz2));  // camera-space normal (-ny: screen y flipped)

            float NdL  = std::max(0.0f, N.dot(Lc));
            Vec3  R    = N * (2.0f * NdL) - Lc;
            float spec = std::pow(std::max(0.0f, R.dot(Vc)), 80.0f);
            // Fresnel rim: glow at silhouette edges (N perpendicular to view = edge of sphere)
            float rim  = std::pow(1.0f - std::abs(N.dot(Vc)), 3.0f) * 0.70f;
            float i    = 0.14f + 0.62f * NdL + 0.50f * spec + rim;

            drawPixel(pixels, cx+dx, cy+dy,
                      makeColor(base_r*i, base_g*i, base_b*i));
        }
    }
}

// ── Render cube ───────────────────────────────────────────────────────────────

void renderCube(uint32_t* pixels, const RigidBody& body,
                const Mat4& view, Vec3 light_pos, Vec3 cam_pos, uint32_t base) {
    Mat4 rot = Mat4::rotateX(body.angleX) * Mat4::rotateY(body.angleY);

    Vec3 world_v[8], cam_v[8], screen_v[8];
    for (int i = 0; i < 8; i++) {
        Vec3 r  = rot * CUBE_VERTS[i];
        world_v[i]  = Vec3(r.x+body.pos.x, r.y+body.pos.y, r.z+body.pos.z);
        cam_v[i]    = view * world_v[i];
        screen_v[i] = project(cam_v[i]);
    }

    int   order[6]={0,1,2,3,4,5}; float fz[6];
    for (int i=0;i<6;i++) {
        fz[i]=0; for(int j=0;j<4;j++) fz[i]+=cam_v[CUBE_FACES[i].v[j]].z; fz[i]/=4;
    }
    std::sort(order,order+6,[&](int a,int b){return fz[a]>fz[b];});

    Vec3 cam_fwd(view.m[2][0], view.m[2][1], view.m[2][2]);

    for (int fi=0;fi<6;fi++) {
        const Face& face = CUBE_FACES[order[fi]];
        Vec3 wn = rot * face.normal;
        if (wn.dot(cam_fwd) > 0) continue;

        Vec3 center(0,0,0);
        for (int j=0;j<4;j++) center=center+world_v[face.v[j]]*0.25f;

        uint32_t color = phongShade(wn, center, light_pos, cam_pos, base);
        auto& p0=screen_v[face.v[0]]; auto& p1=screen_v[face.v[1]];
        auto& p2=screen_v[face.v[2]]; auto& p3=screen_v[face.v[3]];

        fillQuad(pixels,(int)p0.x,(int)p0.y,(int)p1.x,(int)p1.y,
                        (int)p2.x,(int)p2.y,(int)p3.x,(int)p3.y, color);

        uint32_t edge=0xFFCCDDEE;  // light blue-white outline — silhouettes the cube against the background
        drawLine(pixels,(int)p0.x,(int)p0.y,(int)p1.x,(int)p1.y,edge);
        drawLine(pixels,(int)p1.x,(int)p1.y,(int)p2.x,(int)p2.y,edge);
        drawLine(pixels,(int)p2.x,(int)p2.y,(int)p3.x,(int)p3.y,edge);
        drawLine(pixels,(int)p3.x,(int)p3.y,(int)p0.x,(int)p0.y,edge);
    }
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main() {
    srand(SDL_GetTicks());

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init: " << SDL_GetError() << "\n";
        return 1;
    }
    SDL_Window*   window   = SDL_CreateWindow("Level 3 — Full Engine  |  SPACE = spawn ball",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture*  texture  = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);

    static uint32_t pixels[WIDTH * HEIGHT];

    // Three cubes
    RigidBody bodies[3];
    bodies[0].pos={-3, 1, 1};   bodies[0].vel={ 2.5f, 1.8f, 1.2f};
    bodies[0].omega={1.5f,1.0f,0}; bodies[0].angleX=0;    bodies[0].angleY=0;

    bodies[1].pos={ 3,-1,-1};   bodies[1].vel={-2.0f, 2.2f,-1.5f};
    bodies[1].omega={0.8f,2.0f,0}; bodies[1].angleX=0.5f; bodies[1].angleY=1.0f;

    bodies[2].pos={ 0, 3, 2};   bodies[2].vel={ 1.2f,-2.5f, 2.0f};
    bodies[2].omega={2.5f,0.5f,0}; bodies[2].angleX=1.0f; bodies[2].angleY=0.5f;

    uint32_t cube_colors[3] = { 0xFFFF4433, 0xFF44FF88, 0xFFFFCC22 };  // red, green, yellow — high contrast vs navy

    std::vector<Ball> balls;
    Vec3  light_pos(6.0f, 9.0f, -4.0f);
    float cam_angle = 0.0f;

    uint32_t prev = SDL_GetTicks();
    bool running = true;
    SDL_Event event;

    while (running) {
        uint32_t now = SDL_GetTicks();
        float dt = std::min((now - prev) / 1000.0f, 0.05f);
        prev = now;

        // ── Events ───────────────────────────────────────────────────────────
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) running = false;

                if (event.key.keysym.sym == SDLK_SPACE) {
                    Ball b;
                    // Spawn near a random wall face
                    const float edge = BOUND - 0.6f;
                    int face = rand() % 6;
                    float scatter = BOUND * 0.6f;
                    float s1 = (float)(rand()%1000-500)/500.0f * scatter;
                    float s2 = (float)(rand()%1000-500)/500.0f * scatter;
                    switch (face) {
                        case 0: b.pos = Vec3(-edge, s1, s2); break;
                        case 1: b.pos = Vec3( edge, s1, s2); break;
                        case 2: b.pos = Vec3(s1, -edge, s2); break;
                        case 3: b.pos = Vec3(s1,  edge, s2); break;
                        case 4: b.pos = Vec3(s1, s2, -edge); break;
                        default:b.pos = Vec3(s1, s2,  edge); break;
                    }
                    // Fire toward center with high speed + small random spread
                    float speed = 10.0f + (float)(rand()%400)/100.0f;  // 10–14 units/s
                    Vec3 dir = (Vec3(0,0,0) - b.pos).normalized();
                    float spread = 0.3f;
                    dir = Vec3(
                        dir.x + (float)(rand()%200-100)/100.0f * spread,
                        dir.y + (float)(rand()%200-100)/100.0f * spread,
                        dir.z + (float)(rand()%200-100)/100.0f * spread
                    ).normalized();
                    b.vel = dir * speed;
                    // Saturated hue color — HSV(random_hue, 1, 1) → RGB
                    // Avoids muddy pastels from random-all-channels approach
                    float hue = (float)(rand() % 360);
                    float h = hue / 60.0f;
                    int   qi = (int)h;
                    float f  = h - qi;
                    // V=1, S=1: p=0, q=1-f, t=f
                    uint8_t br2=0, bg2=0, bb2=0;
                    switch (qi % 6) {
                        case 0: br2=255; bg2=(uint8_t)(f*255);   bb2=0;            break;
                        case 1: br2=(uint8_t)((1-f)*255); bg2=255; bb2=0;          break;
                        case 2: br2=0;   bg2=255; bb2=(uint8_t)(f*255);            break;
                        case 3: br2=0;   bg2=(uint8_t)((1-f)*255); bb2=255;        break;
                        case 4: br2=(uint8_t)(f*255); bg2=0; bb2=255;              break;
                        default:br2=255; bg2=0; bb2=(uint8_t)((1-f)*255);          break;
                    }
                    b.color = (0xFF<<24)|(br2<<16)|(bg2<<8)|bb2;
                    balls.push_back(b);
                }
            }
        }

        // ── Physics ──────────────────────────────────────────────────────────
        for (auto& b : bodies) physicsUpdate(b, dt);
        for (auto& b : balls)  updateBall(b, dt);

        // Cube-cube collisions
        resolveCubeCube(bodies[0], bodies[1]);
        resolveCubeCube(bodies[0], bodies[2]);
        resolveCubeCube(bodies[1], bodies[2]);

        // Ball-cube and ball-ball collisions
        for (auto& ball : balls) {
            for (auto& cube : bodies) resolveSphereCube(ball, cube);
        }
        for (int i = 0; i < (int)balls.size(); i++)
            for (int j = i+1; j < (int)balls.size(); j++)
                resolveSphereSphere(balls[i], balls[j]);

        // ── Camera ───────────────────────────────────────────────────────────
        cam_angle += CAM_SPEED * dt;
        Vec3 cam_pos(
            std::sin(cam_angle) * CAM_RADIUS,
            CAM_HEIGHT,
           -std::cos(cam_angle) * CAM_RADIUS
        );
        Mat4 view = Mat4::lookAt(cam_pos, Vec3(0,0,0), Vec3(0,1,0));

        // ── Render ────────────────────────────────────────────────────────────
        clearBackground(pixels);

        drawFloorGrid(pixels, view);
        drawBoundingBox(pixels, view);

        for (int i = 0; i < 3; i++)
            renderCube(pixels, bodies[i], view, light_pos, cam_pos, cube_colors[i]);

        for (auto& b : balls)
            renderBall(pixels, b, view, light_pos);

        SDL_UpdateTexture(texture, nullptr, pixels, WIDTH * sizeof(uint32_t));
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
