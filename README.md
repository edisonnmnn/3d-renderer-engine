# 3D Renderer & Physics Engine

A real-time 3D renderer and rigid body physics engine built from scratch in C++. No external math or graphics libraries — custom `Vec3`, `Mat4`, software rasterizer, and physics solver on top of SDL2.

Three self-contained implementations, each building on the last.

---

## Projects

### [`wireframe-terminal`](3d-renderer-engine/wireframe-terminal) — ASCII Wireframe Renderer
Spinning wireframe cube rendered entirely in the terminal. No dependencies beyond the C++ standard library.

- Rotation matrices (X, Y, Z) composed per frame
- Perspective divide: `x / (z + fov)` with aspect correction for terminal character dimensions
- Bresenham's line algorithm (integer-only, no floating point)
- `char` screen buffer flushed to stdout each frame

```bash
cd 3d-renderer-engine/wireframe-terminal && make run
```

---

### [`solid-physics`](3d-renderer-engine/solid-physics) — Solid Rendering + Gravity
Solid shaded cube with gravity, bouncing, and flat Phong shading. First use of SDL2 for a pixel buffer window.

- `uint32_t pixels[WIDTH * HEIGHT]` uploaded to GPU each frame via SDL streaming texture
- Painter's algorithm with back-face culling for correct face ordering
- Flat shading: `I = ambient + kd · max(0, N·L)`
- Rigid body: position, velocity, gravity integration, wall restitution

```bash
cd 3d-renderer-engine/solid-physics && make run
```

---

### [`mini-physics-engine`](3d-renderer-engine/mini-physics-engine) — Full Renderer + Physics Engine
Multiple rigid bodies, spawnable balls, full lighting model, and impulse-based collision response.

- `Mat4::lookAt` view matrix — orbiting camera with orthonormal basis constructed from cross products
- Two-light Phong shading (key + fill) on cubes; per-pixel sphere shading in camera space
- Fresnel rim lighting on balls: `rim = (1 - |N·V|)³ · 0.7`
- AABB–AABB, sphere–AABB, and sphere–sphere collision detection
- Impulse resolution: `j = -(1+e)·v_rel·n / (1/m₁ + 1/m₂)`
- Angular kick on cube–ball contact: spin imparted proportional to impulse magnitude
- `SPACE` spawns a ball from a random wall at 10–14 units/s aimed at the scene center

```bash
cd 3d-renderer-engine/mini-physics-engine && make run
```

---

## Build

Requires SDL2 (for `solid-physics` and `mini-physics-engine`):

```bash
brew install sdl2   # macOS
```

Each project builds with a single `make` — no cmake, no external math libraries.

---

## What This Demonstrates

- **Linear algebra applied** — rotation matrices, homogeneous coordinates, cross products for camera basis, dot products for lighting and collision normals — used throughout for real geometric reasoning, not just API calls
- **Systems-level C++** — manual pixel buffer layout (`uint32_t pixels[WIDTH * HEIGHT]`), row-major indexing, minimal heap allocation in the hot path
- **Algorithm implementation from scratch** — Bresenham's line, scanline triangle fill, painter's algorithm, impulse collision response — no library implementations
- **Physics from first principles** — AABB penetration depth, closest-point sphere–box test, and impulse derivation from Newton's law, not a physics engine

---

*Self-directed C++ and applied math project.*
