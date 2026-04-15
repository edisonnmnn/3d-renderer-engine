# 3D Renderer & Physics Engine — Level 3

A real-time 3D renderer and rigid body physics engine written from scratch in C++. No external math or graphics libraries — custom `Vec3`, `Mat4`, software rasterizer, and physics solver built on top of SDL2 for windowing.

![demo](demo.gif)
<!-- Record a gif: brew install ffmpeg, then: ffmpeg -f avfoundation -i "1" -vf scale=800:600 -r 30 demo.gif -->

---

## Features

**Rendering**
- Software rasterizer — CPU-side pixel buffer uploaded to GPU via SDL streaming texture each frame
- Perspective projection with a `Mat4::lookAt` view matrix — orbiting camera
- Painter's algorithm with back-face culling for correct face draw order
- Two-light Phong shading — key light (diffuse + specular) + fill light (lifts shadows)
- Per-pixel sphere shading with Fresnel rim lighting — each pixel computes its own surface normal
- Scanline triangle fill, Bresenham's line algorithm

**Physics**
- Rigid body simulation — position, velocity, angular velocity, spin damping
- AABB collision detection between cubes (axis of minimum penetration)
- Sphere-AABB collision detection (closest point on box to sphere center)
- Sphere-sphere collision detection
- Linear impulse-based collision response — `j = -(1+e)·v_rel·n / (1/m₁ + 1/m₂)`
- Angular kick on cube-ball contact

**Interaction**
- `SPACE` — spawn a ball at a random position with a random velocity and color
- `ESC` / window close — quit
- Camera orbits the scene continuously

---

## Build

Requires SDL2:
```bash
brew install sdl2   # macOS
```

```bash
make        # builds ./physics-engine
make run    # builds and launches
make clean  # removes binary
```

---

## Architecture

```
src/
├── main.cpp          — render loop, physics, SDL setup
└── math/
    ├── vec3.h        — Vec3: dot, cross, normalize, operators
    └── mat4.h        — Mat4: rotateX/Y/Z, translate, lookAt, matrix/vector multiply
```

**Frame pipeline:**

```
Physics update (integrate velocity, resolve collisions)
    ↓
Camera: compute lookAt view matrix from orbiting position
    ↓
For each cube:
    rotate vertices → world space → camera space (view matrix) → project to screen
    sort faces by camera-space z (painter's algorithm)
    for each visible face: two-light Phong shade → scanline fill → draw edges
    ↓
For each ball:
    project center to screen, compute screen radius from depth
    for each pixel in ellipse: compute sphere normal, Phong + rim shade
    ↓
Upload pixel buffer to GPU (SDL_UpdateTexture) → present
```

---

## Implementation Notes

**Perspective projection**
Uses a simplified perspective divide rather than a full projection matrix:
```
px = (x / (z + FOV)) * HEIGHT * aspect * 0.5 + WIDTH/2
py = (-y / (z + FOV)) * HEIGHT * 0.5 + HEIGHT/2   ← y negated: screen y increases down
```
`FOV` acts as the camera's distance from the image plane.

**View matrix (`Mat4::lookAt`)**
Constructs an orthonormal camera basis from `eye`, `target`, and `up`:
```
f = normalize(target - eye)     // forward
r = normalize(up × f)           // right  (order matters: up × f not f × up)
u = f × r                       // up (orthogonalized)
```
The view matrix rows are `[r, u, f]` with translation `[-r·eye, -u·eye, -f·eye]`.

**Phong shading**
Per face: `I = ambient + kd·max(0, N·L) + ks·max(0, R·V)^n + fill`

**Per-pixel sphere shading**
The sphere normal at screen pixel `(dx, dy)` relative to the projected center is computed in camera space:
```
N = normalize(dx/rx, -dy/ry, sqrt(1 - (dx/rx)² - (dy/ry)²))
```
This avoids any 3D-to-2D normal transformation — the sphere looks correct from the camera's perspective directly.

**Fresnel rim**
```
rim = (1 - |N·V|)^4 * 0.4
```
High at silhouette edges where the surface is nearly perpendicular to the view direction, giving a subtle glow.

**AABB collision**
Penetration depths on all 3 axes are computed from `|Δpos| - (ext_a + ext_b)`. The axis with smallest penetration is the collision normal. Objects are separated by half the penetration each, then an impulse is applied along the normal.

---

## What This Demonstrates

- **Linear algebra applied** — rotation matrices, homogeneous coordinates, view transforms, dot/cross products used throughout for real geometric reasoning
- **Systems-level C++** — manual memory layout (`uint32_t pixels[WIDTH * HEIGHT]`), row-major indexing, minimal allocations in the hot loop
- **Algorithm implementation** — Bresenham's line (integer-only), scanline fill, painter's algorithm, impulse resolution — no library implementations
- **Physics from scratch** — collision detection and response derived from first principles, not a physics engine

---

*Built as part of a self-directed C++ and applied math project. See also: [Level 1 (terminal wireframe)](https://github.com/edisonnmnn/3d-renderer-spinning-cube) | Level 2 (solid rendering)*
