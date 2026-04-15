#include <SDL2/SDL.h>
#include <iostream>
#include <algorithm>
#include <cmath>
#include "math/vec3.h"
#include "math/mat4.h"

// ── Constants ────────────────────────────────────────────────────────────────

const int   WIDTH       = 800;
const int   HEIGHT      = 600;
const float FOV         = 5.0f;
const float GRAVITY     = -9.8f;
const float RESTITUTION = 0.75f;
const float BOUND       = 3.0f;   // world bounces at ±BOUND

// ── Drawing ──────────────────────────────────────────────────────────────────

void drawPixel(uint32_t* pixels, int x, int y, uint32_t color) {
    if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT)
        pixels[y * WIDTH + x] = color;
}

void drawLine(uint32_t* pixels, int x0, int y0, int x1, int y1, uint32_t color) {
    int dx  = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx  = (x1 > x0) ? 1 : -1;
    int sy  = (y1 > y0) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        drawPixel(pixels, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

// Scanline fill for a triangle — sorts vertices by y, fills between edges
void fillTriangle(uint32_t* pixels,
                  int x0, int y0, int x1, int y1, int x2, int y2,
                  uint32_t color) {
    // Sort by y ascending (top to bottom in screen space)
    if (y0 > y1) { std::swap(x0,x1); std::swap(y0,y1); }
    if (y0 > y2) { std::swap(x0,x2); std::swap(y0,y2); }
    if (y1 > y2) { std::swap(x1,x2); std::swap(y1,y2); }

    int total_h = y2 - y0;
    if (total_h == 0) return;

    for (int y = y0; y <= y2; y++) {
        bool lower_half   = (y > y1) || (y1 == y0);
        int  segment_h    = lower_half ? (y2 - y1) : (y1 - y0);
        if (segment_h == 0) continue;

        float alpha = (float)(y - y0) / total_h;
        float beta  = lower_half
            ? (float)(y - y1) / segment_h
            : (float)(y - y0) / segment_h;

        int ax = x0 + (int)((x2 - x0) * alpha);
        int bx = lower_half
            ? x1 + (int)((x2 - x1) * beta)
            : x0 + (int)((x1 - x0) * beta);

        if (ax > bx) std::swap(ax, bx);
        for (int x = ax; x <= bx; x++)
            drawPixel(pixels, x, y, color);
    }
}

// Split quad (4 verts, winding order must be consistent) into 2 triangles and fill
void fillQuad(uint32_t* pixels,
              int x0, int y0, int x1, int y1,
              int x2, int y2, int x3, int y3,
              uint32_t color) {
    fillTriangle(pixels, x0, y0, x1, y1, x2, y2, color);
    fillTriangle(pixels, x0, y0, x2, y2, x3, y3, color);
}

// ── Color ────────────────────────────────────────────────────────────────────

// Vertical gradient background — dark navy top, deeper navy bottom
void clearBackground(uint32_t* pixels) {
    const float tr=0.102f, tg=0.165f, tb=0.247f;  // #1A2A3F top
    const float br=0.047f, bg=0.082f, bb=0.125f;  // #0C1520 bottom
    for (int y = 0; y < HEIGHT; y++) {
        float t = (float)y / (HEIGHT - 1);
        uint8_t R = (uint8_t)((tr+(br-tr)*t)*255);
        uint8_t G = (uint8_t)((tg+(bg-tg)*t)*255);
        uint8_t B = (uint8_t)((tb+(bb-tb)*t)*255);
        uint32_t col = (0xFF<<24)|(R<<16)|(G<<8)|B;
        for (int x = 0; x < WIDTH; x++)
            pixels[y * WIDTH + x] = col;
    }
}

uint32_t makeColor(float r, float g, float b) {
    r = std::max(0.0f, std::min(1.0f, r));
    g = std::max(0.0f, std::min(1.0f, g));
    b = std::max(0.0f, std::min(1.0f, b));
    return (0xFF << 24)
         | ((uint8_t)(r * 255) << 16)
         | ((uint8_t)(g * 255) <<  8)
         |  (uint8_t)(b * 255);
}

// Scale an ARGB color by intensity (0–1), preserving alpha
uint32_t shade(uint32_t color, float intensity) {
    intensity = std::max(0.0f, std::min(1.0f, intensity));
    float r = ((color >> 16) & 0xFF) / 255.0f;
    float g = ((color >>  8) & 0xFF) / 255.0f;
    float b = ( color        & 0xFF) / 255.0f;
    return makeColor(r * intensity, g * intensity, b * intensity);
}

// ── Geometry ─────────────────────────────────────────────────────────────────

struct Face {
    int  v[4];    // indices into transformed vertex array
    Vec3 normal;  // local-space outward normal
};

// Unit cube: 8 vertices at ±1
static const Vec3 CUBE_VERTS[8] = {
    {-1,  1,  1},  // 0 front-top-left
    {-1, -1,  1},  // 1 front-bot-left
    { 1, -1,  1},  // 2 front-bot-right
    { 1,  1,  1},  // 3 front-top-right
    {-1,  1, -1},  // 4 back-top-left
    {-1, -1, -1},  // 5 back-bot-left
    { 1, -1, -1},  // 6 back-bot-right
    { 1,  1, -1},  // 7 back-top-right
};

static const Face CUBE_FACES[6] = {
    {{0,1,2,3}, { 0, 0, 1}},  // front  (z+)
    {{7,6,5,4}, { 0, 0,-1}},  // back   (z-)
    {{4,5,1,0}, {-1, 0, 0}},  // left   (x-)
    {{3,2,6,7}, { 1, 0, 0}},  // right  (x+)
    {{4,0,3,7}, { 0, 1, 0}},  // top    (y+)
    {{1,5,6,2}, { 0,-1, 0}},  // bottom (y-)
};

// ── Projection ───────────────────────────────────────────────────────────────

// Simple perspective divide (camera at origin looking +z)
Vec3 project(const Vec3& v) {
    float aspect = (float)WIDTH / (float)HEIGHT;
    float px = ( v.x / (v.z + FOV)) * HEIGHT * aspect * 0.5f + WIDTH  * 0.5f;
    float py = (-v.y / (v.z + FOV)) * HEIGHT           * 0.5f + HEIGHT * 0.5f;  // negate y: screen y grows down
    return Vec3(px, py, v.z);
}

// ── Physics ──────────────────────────────────────────────────────────────────

struct RigidBody {
    Vec3  pos;
    Vec3  vel;
    float angleX, angleY;
    float spinX,  spinY;    // radians/sec
};

void physicsUpdate(RigidBody& b, float dt) {
    b.vel.y += GRAVITY * dt;

    b.pos.x += b.vel.x * dt;
    b.pos.y += b.vel.y * dt;
    b.pos.z += b.vel.z * dt;

    // Bounce off axis-aligned walls (cube half-extent = 1)
    const float ext = 1.0f;
    if (b.pos.y - ext < -BOUND) { b.pos.y = -BOUND + ext; b.vel.y = -b.vel.y * RESTITUTION; }
    if (b.pos.y + ext >  BOUND) { b.pos.y =  BOUND - ext; b.vel.y = -b.vel.y * RESTITUTION; }
    if (b.pos.x - ext < -BOUND) { b.pos.x = -BOUND + ext; b.vel.x = -b.vel.x * RESTITUTION; }
    if (b.pos.x + ext >  BOUND) { b.pos.x =  BOUND - ext; b.vel.x = -b.vel.x * RESTITUTION; }

    b.angleX += b.spinX * dt;
    b.angleY += b.spinY * dt;
}

// ── Render ───────────────────────────────────────────────────────────────────

void renderCube(uint32_t* pixels, const RigidBody& body, Vec3 light_dir, uint32_t base_color) {
    Mat4 rotation = Mat4::rotateX(body.angleX) * Mat4::rotateY(body.angleY);

    // Transform all 8 vertices into world space and project to screen
    Vec3 world[8], screen[8];
    for (int i = 0; i < 8; i++) {
        Vec3 rot = rotation * CUBE_VERTS[i];
        world[i]  = Vec3(rot.x + body.pos.x, rot.y + body.pos.y, rot.z + body.pos.z);
        screen[i] = project(world[i]);
    }

    // Painter's algorithm: compute average world-z per face, sort back-to-front
    int order[6] = {0,1,2,3,4,5};
    float fz[6];
    for (int i = 0; i < 6; i++) {
        fz[i] = 0;
        for (int j = 0; j < 4; j++) fz[i] += world[CUBE_FACES[i].v[j]].z;
        fz[i] /= 4.0f;
    }
    std::sort(order, order + 6, [&](int a, int b){ return fz[a] > fz[b]; });

    for (int fi = 0; fi < 6; fi++) {
        const Face& face = CUBE_FACES[order[fi]];

        // Rotate the face normal to world space
        Vec3 wn = rotation * face.normal;

        // Back-face culling: skip faces pointing away from camera (wn.z > 0 means facing away)
        if (wn.z > 0.0f) continue;

        // Flat shading: ambient + diffuse
        float diffuse   = std::max(0.0f, wn.dot(light_dir));
        float intensity = 0.22f + 0.78f * diffuse;

        // Project face corners to screen
        auto& p0 = screen[face.v[0]];
        auto& p1 = screen[face.v[1]];
        auto& p2 = screen[face.v[2]];
        auto& p3 = screen[face.v[3]];

        uint32_t fill_color = shade(base_color, intensity);
        fillQuad(pixels,
                 (int)p0.x, (int)p0.y,
                 (int)p1.x, (int)p1.y,
                 (int)p2.x, (int)p2.y,
                 (int)p3.x, (int)p3.y,
                 fill_color);

        // Light edges — silhouettes the cube against the dark background
        uint32_t edge = 0xFFCCDDEE;
        drawLine(pixels, (int)p0.x,(int)p0.y, (int)p1.x,(int)p1.y, edge);
        drawLine(pixels, (int)p1.x,(int)p1.y, (int)p2.x,(int)p2.y, edge);
        drawLine(pixels, (int)p2.x,(int)p2.y, (int)p3.x,(int)p3.y, edge);
        drawLine(pixels, (int)p3.x,(int)p3.y, (int)p0.x,(int)p0.y, edge);
    }
}

// ── Main ─────────────────────────────────────────────────────────────────────

int main() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init: " << SDL_GetError() << "\n";
        return 1;
    }

    SDL_Window*   window   = SDL_CreateWindow("Level 2 — Solid Physics",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture*  texture  = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);

    static uint32_t pixels[WIDTH * HEIGHT];

    // Bouncing, spinning cube
    RigidBody body;
    body.pos    = Vec3(0.0f, 2.0f, 0.0f);
    body.vel    = Vec3(1.8f, 0.0f, 0.0f);
    body.angleX = 0.0f; body.angleY = 0.3f;
    body.spinX  = 1.2f; body.spinY  = 0.9f;

    Vec3 light_dir = Vec3(1.0f, 2.0f, -1.0f).normalized();

    uint32_t prev = SDL_GetTicks();
    bool running = true;
    SDL_Event event;

    while (running) {
        uint32_t now = SDL_GetTicks();
        float dt = std::min((now - prev) / 1000.0f, 0.05f);  // cap to avoid spiral of death
        prev = now;

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) running = false;
        }

        clearBackground(pixels);

        physicsUpdate(body, dt);
        renderCube(pixels, body, light_dir, 0xFFFF4433);  // vivid red — pops against navy

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
