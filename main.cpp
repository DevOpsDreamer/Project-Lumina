// =============================================================================
//  PROJECT LUMINA v2.0 — Advanced Command-Line Rendering Engine
// =============================================================================
//  A portfolio-grade, physically-based path tracer demonstrating:
//    - Tile-based parallel rendering with a thread-safe work queue
//    - Physically accurate Dielectric (glass) materials via Snell's Law
//    - Depth-of-field camera with adjustable aperture
//    - Dynamic procedural scene generation
//    - Full CLI control over all render parameters
//
//  Author  : Principal Systems Architect & Lead Graphics Engineer
//  Deps    : ZERO external libraries. C++ standard library + OS threading API.
//  Output  : Uncompressed .ppm (Portable Pixmap) image.
// =============================================================================

// --- Standard Library -------------------------------------------------------
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <chrono>
#include <string>
#include <sstream>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <algorithm>

// --- Platform-Specific Threading & Atomics ----------------------------------
//  MinGW (win32 threading model) does not support std::thread/std::mutex.
//  We use the native OS threading API on each platform for maximum portability.
//
//  Windows: CreateThread, CRITICAL_SECTION, InterlockedIncrement
//  POSIX:   pthread_create, pthread_mutex, __sync_fetch_and_add
// ----------------------------------------------------------------------------
#ifdef _WIN32
    #include <windows.h>
#else
    #include <pthread.h>
    #include <unistd.h>
#endif

// =============================================================================
//  PLATFORM ABSTRACTION LAYER
// =============================================================================
//  Thin wrappers around OS-specific primitives so the rest of the engine
//  is written in clean, platform-agnostic C++.
// =============================================================================

// --- Mutex ------------------------------------------------------------------
struct Mutex {
#ifdef _WIN32
    CRITICAL_SECTION cs;
    void init()    { InitializeCriticalSection(&cs); }
    void lock()    { EnterCriticalSection(&cs); }
    void unlock()  { LeaveCriticalSection(&cs); }
    void destroy() { DeleteCriticalSection(&cs); }
#else
    pthread_mutex_t mtx;
    void init()    { pthread_mutex_init(&mtx, NULL); }
    void lock()    { pthread_mutex_lock(&mtx); }
    void unlock()  { pthread_mutex_unlock(&mtx); }
    void destroy() { pthread_mutex_destroy(&mtx); }
#endif
};

// --- Atomic Integer ---------------------------------------------------------
//  Lock-free atomic increment for the progress counter.
//  Each worker thread increments this after completing a tile.
struct AtomicInt {
#ifdef _WIN32
    volatile LONG value;
    void set(int v)    { InterlockedExchange(&value, (LONG)v); }
    int  get()         { return (int)value; }
    int  increment()   { return (int)InterlockedIncrement(&value); }
#else
    volatile int value;
    void set(int v)    { __sync_lock_test_and_set(&value, v); }
    int  get()         { return __sync_fetch_and_add(&value, 0); }
    int  increment()   { return __sync_fetch_and_add(&value, 1) + 1; }
#endif
};

// --- Get Hardware Thread Count ----------------------------------------------
int get_hardware_threads() {
#ifdef _WIN32
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return (int)si.dwNumberOfProcessors;
#else
    int n = (int)sysconf(_SC_NPROCESSORS_ONLN);
    return (n > 0) ? n : 1;
#endif
}

// =============================================================================
//  CONSTANTS
// =============================================================================

const double PI        = 3.14159265358979323846;
const double INF       = 1e30;
const double EPSILON   = 0.001; // Shadow acne prevention offset

// =============================================================================
//  SECTION 1: Vec3 — 3D Vector / Color / Point
// =============================================================================
//  The fundamental mathematical type. Used for positions, directions, and
//  RGB colors. All vector math for the entire engine flows through here.
// =============================================================================

struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(double t)      const { return Vec3(x*t, y*t, z*t); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x*v.x, y*v.y, z*v.z); }
    Vec3 operator/(double t)      const { double inv = 1.0/t; return Vec3(x*inv, y*inv, z*inv); }
    Vec3 operator-()              const { return Vec3(-x, -y, -z); }

    Vec3& operator+=(const Vec3& v) { x+=v.x; y+=v.y; z+=v.z; return *this; }
    Vec3& operator*=(double t)      { x*=t; y*=t; z*=t; return *this; }

    double dot(const Vec3& v)   const { return x*v.x + y*v.y + z*v.z; }
    Vec3   cross(const Vec3& v) const {
        return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
    }

    double length_squared() const { return x*x + y*y + z*z; }
    double length()         const { return std::sqrt(length_squared()); }

    Vec3 normalized() const {
        double len = length();
        return (len < 1e-12) ? Vec3(0,0,0) : *this / len;
    }

    // Reflect vector V about surface normal N.
    static Vec3 reflect(const Vec3& v, const Vec3& n) {
        return v - n * 2.0 * v.dot(n);
    }

    // Refract vector UV through surface with normal N.
    // etai_over_etat = n_incident / n_transmitted (ratio of refractive indices).
    //
    // Derivation (Snell's Law in vector form):
    //   r_perp     = (ni/nt) * (uv + cos_theta * n)
    //   r_parallel = -sqrt(1 - |r_perp|^2) * n
    //   refracted  = r_perp + r_parallel
    static Vec3 refract(const Vec3& uv, const Vec3& n, double etai_over_etat) {
        double cos_theta = std::fmin((-uv).dot(n), 1.0);
        Vec3 r_out_perp = (uv + n * cos_theta) * etai_over_etat;
        Vec3 r_out_parallel = n * (-std::sqrt(std::fabs(1.0 - r_out_perp.length_squared())));
        return r_out_perp + r_out_parallel;
    }

    // Clamp each component to [0, 1].
    Vec3 clamped() const {
        return Vec3(std::fmax(0.0, std::fmin(1.0, x)),
                    std::fmax(0.0, std::fmin(1.0, y)),
                    std::fmax(0.0, std::fmin(1.0, z)));
    }

    // Check if the vector is near-zero in all dimensions.
    bool near_zero() const {
        return (std::fabs(x) < 1e-8) && (std::fabs(y) < 1e-8) && (std::fabs(z) < 1e-8);
    }
};

Vec3 operator*(double t, const Vec3& v) { return v * t; }


// =============================================================================
//  SECTION 2: Ray
// =============================================================================

struct Ray {
    Vec3 origin, direction;
    Ray() {}
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    Vec3 at(double t) const { return origin + direction * t; }
};


// =============================================================================
//  SECTION 3: Per-Thread Random Number Generator (xorshift64)
// =============================================================================
//  Each worker thread creates its own RNG instance with a unique seed.
//  xorshift64 is extremely fast (3 XOR + shift ops) with a period of 2^64-1.
//  No locks, no contention, no thread_local keyword needed.
// =============================================================================

struct RNG {
    unsigned long long state;

    RNG(unsigned long long seed = 1ULL) : state(seed ? seed : 1ULL) {}

    unsigned long long next() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    }

    // Uniform random double in [0, 1).
    double rand01() {
        return (next() & 0xFFFFFFFFFFFFFULL) / (double)(0x10000000000000ULL);
    }

    // Uniform random double in [min, max).
    double rand_range(double lo, double hi) {
        return lo + (hi - lo) * rand01();
    }
};

// Random point inside the unit sphere (rejection sampling).
Vec3 random_in_unit_sphere(RNG& rng) {
    while (true) {
        Vec3 p(rng.rand_range(-1,1), rng.rand_range(-1,1), rng.rand_range(-1,1));
        if (p.length_squared() < 1.0) return p;
    }
}

// Random unit vector (for Lambertian scattering).
Vec3 random_unit_vector(RNG& rng) {
    return random_in_unit_sphere(rng).normalized();
}

// Random point inside the unit disk (for depth-of-field lens sampling).
Vec3 random_in_unit_disk(RNG& rng) {
    while (true) {
        Vec3 p(rng.rand_range(-1,1), rng.rand_range(-1,1), 0);
        if (p.length_squared() < 1.0) return p;
    }
}


// =============================================================================
//  SECTION 4: PHYSICALLY-BASED MATERIAL SYSTEM
// =============================================================================
//  Three material models implemented as a discriminated union:
//
//  ┌──────────────┬──────────────────────────────────────────────────────────┐
//  │ LAMBERTIAN   │ Ideal diffuse: scatters light uniformly in the          │
//  │              │ hemisphere above the hit point. Albedo controls color.   │
//  ├──────────────┼──────────────────────────────────────────────────────────┤
//  │ METAL        │ Specular reflection about the surface normal.           │
//  │              │ 'fuzz' parameter adds random perturbation (roughness).  │
//  │              │ fuzz=0 → perfect mirror, fuzz=1 → very rough.           │
//  ├──────────────┼──────────────────────────────────────────────────────────┤
//  │ DIELECTRIC   │ Glass/water: uses Snell's Law for refraction and the   │
//  │              │ Schlick approximation for Fresnel reflectance. Handles  │
//  │              │ total internal reflection when the critical angle is    │
//  │              │ exceeded.                                               │
//  └──────────────┴──────────────────────────────────────────────────────────┘
// =============================================================================

enum MaterialType { MAT_LAMBERTIAN, MAT_METAL, MAT_DIELECTRIC };

struct Material {
    MaterialType type;
    Vec3 albedo;      // Surface color (Lambertian & Metal)
    double param;     // Fuzz for Metal, refractive index for Dielectric

    Material() : type(MAT_LAMBERTIAN), albedo(Vec3(0.5,0.5,0.5)), param(0) {}
    Material(MaterialType t, const Vec3& a, double p) : type(t), albedo(a), param(p) {}

    // --- Factory Methods (clean scene construction syntax) ------------------
    static Material lambertian(const Vec3& color) {
        return Material(MAT_LAMBERTIAN, color, 0.0);
    }
    static Material metal(const Vec3& color, double fuzz) {
        return Material(MAT_METAL, color, std::fmin(fuzz, 1.0));
    }
    static Material glass(double refractive_index) {
        return Material(MAT_DIELECTRIC, Vec3(1,1,1), refractive_index);
    }
};

// Schlick's approximation for Fresnel reflectance.
//
// At grazing angles (theta → 90°), ALL surfaces become mirror-like.
// This is the "sheen" you see on a lake at sunset, or on a car's paint
// viewed at a low angle.
//
//   R(theta) = R0 + (1 - R0) * (1 - cos(theta))^5
//   where R0 = ((n1 - n2) / (n1 + n2))^2
//
// For glass (n=1.5): R0 ≈ 0.04 → 4% reflection at normal incidence,
// rising to 100% at grazing angles.
double schlick_reflectance(double cosine, double ref_idx) {
    double r0 = (1.0 - ref_idx) / (1.0 + ref_idx);
    r0 = r0 * r0;
    return r0 + (1.0 - r0) * std::pow(1.0 - cosine, 5.0);
}


// =============================================================================
//  SECTION 5: GEOMETRY — Hit Record & Sphere
// =============================================================================

struct HitRecord {
    Vec3 p;              // Intersection point
    Vec3 normal;         // Surface normal (always facing against the ray)
    double t;            // Ray parameter at intersection
    bool front_face;     // True if ray hit the outer surface
    Material material;

    void set_face_normal(const Ray& r, const Vec3& outward_normal) {
        front_face = r.direction.dot(outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};

struct Sphere {
    Vec3 center;
    double radius;
    Material material;

    Sphere() : radius(0) {}
    Sphere(const Vec3& c, double r, const Material& m)
        : center(c), radius(r), material(m) {}

    // Ray-Sphere intersection via the quadratic formula.
    // Uses the half-b optimization to reduce floating-point operations.
    bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {
        Vec3 oc = r.origin - center;
        double a = r.direction.length_squared();
        double half_b = oc.dot(r.direction);
        double c = oc.length_squared() - radius * radius;
        double disc = half_b * half_b - a * c;
        if (disc < 0) return false;

        double sqrtd = std::sqrt(disc);
        double root = (-half_b - sqrtd) / a;
        if (root < t_min || root > t_max) {
            root = (-half_b + sqrtd) / a;
            if (root < t_min || root > t_max) return false;
        }

        rec.t = root;
        rec.p = r.at(root);
        rec.set_face_normal(r, (rec.p - center) / radius);
        rec.material = material;
        return true;
    }
};


// =============================================================================
//  SECTION 6: RAY-SCENE INTERSECTION & PATH TRACING
// =============================================================================
//  ray_color() is the core path tracing function. For each ray:
//    1. Find the closest sphere intersection.
//    2. Based on the hit material, compute a scattered ray:
//       - Lambertian: random hemisphere scatter (diffuse)
//       - Metal:      reflection with optional fuzz
//       - Dielectric: refraction (Snell) or reflection (Fresnel/Schlick)
//    3. Recurse with the scattered ray, attenuating by the material albedo.
//    4. If no hit, return the sky gradient (environment lighting).
//    5. If depth exhausted, return black (energy absorbed).
// =============================================================================

bool hit_scene(const std::vector<Sphere>& world, const Ray& r,
               double t_min, double t_max, HitRecord& rec)
{
    HitRecord temp;
    bool hit = false;
    double closest = t_max;
    for (size_t i = 0; i < world.size(); ++i) {
        if (world[i].hit(r, t_min, closest, temp)) {
            hit = true;
            closest = temp.t;
            rec = temp;
        }
    }
    return hit;
}

Vec3 ray_color(const Ray& r, const std::vector<Sphere>& world, int depth, RNG& rng) {
    if (depth <= 0) return Vec3(0,0,0);

    HitRecord rec;
    if (!hit_scene(world, r, EPSILON, INF, rec)) {
        // --- Sky Gradient (Environment Light) ---
        Vec3 unit_dir = r.direction.normalized();
        double t = 0.5 * (unit_dir.y + 1.0);
        return Vec3(1,1,1) * (1.0 - t) + Vec3(0.5, 0.7, 1.0) * t;
    }

    // === MATERIAL SCATTERING ===
    Vec3 attenuation;
    Ray scattered;

    switch (rec.material.type) {

    case MAT_LAMBERTIAN: {
        // Lambertian diffuse: scatter in a random direction in the hemisphere.
        Vec3 scatter_dir = rec.normal + random_unit_vector(rng);
        if (scatter_dir.near_zero()) scatter_dir = rec.normal;
        scattered = Ray(rec.p, scatter_dir);
        attenuation = rec.material.albedo;
        break;
    }

    case MAT_METAL: {
        // Specular reflection + fuzz perturbation.
        Vec3 reflected = Vec3::reflect(r.direction.normalized(), rec.normal);
        Vec3 fuzzed = reflected + random_in_unit_sphere(rng) * rec.material.param;
        scattered = Ray(rec.p, fuzzed);
        attenuation = rec.material.albedo;
        // Absorb if the fuzzed reflection goes below the surface.
        if (scattered.direction.dot(rec.normal) <= 0) return Vec3(0,0,0);
        break;
    }

    case MAT_DIELECTRIC: {
        // --- GLASS / WATER (Snell's Law + Schlick Approximation) ---
        //
        // The refractive index (param) determines how much light bends:
        //   Air   ≈ 1.0
        //   Water ≈ 1.33
        //   Glass ≈ 1.5
        //   Diamond ≈ 2.42
        //
        // When a ray crosses the boundary between two media:
        //   n1 * sin(θ1) = n2 * sin(θ2)   (Snell's Law)
        //
        // If n1/n2 * sin(θ1) > 1, no refraction is possible →
        //   Total Internal Reflection (light bounces inside the material).
        //
        // At shallow angles, Schlick's approximation determines whether
        // the ray reflects or refracts based on Fresnel equations.

        attenuation = Vec3(1.0, 1.0, 1.0); // Glass is perfectly transparent
        double ri = rec.front_face ? (1.0 / rec.material.param) : rec.material.param;

        Vec3 unit_dir = r.direction.normalized();
        double cos_theta = std::fmin((-unit_dir).dot(rec.normal), 1.0);
        double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);

        // Total Internal Reflection check:
        bool cannot_refract = (ri * sin_theta) > 1.0;

        Vec3 direction;
        if (cannot_refract || schlick_reflectance(cos_theta, ri) > rng.rand01()) {
            // Reflect (either TIR or Fresnel reflection)
            direction = Vec3::reflect(unit_dir, rec.normal);
        } else {
            // Refract through the surface
            direction = Vec3::refract(unit_dir, rec.normal, ri);
        }

        scattered = Ray(rec.p, direction);
        break;
    }

    default:
        return Vec3(0,0,0);
    }

    // Recurse: trace the scattered ray, attenuated by material color.
    return attenuation * ray_color(scattered, world, depth - 1, rng);
}


// =============================================================================
//  SECTION 7: ADVANCED CAMERA SYSTEM (with Depth-of-Field)
// =============================================================================
//  The camera simulates a thin lens for depth-of-field (defocus blur / bokeh).
//
//  Key parameters:
//    - aperture:   The lens diameter. Larger → more blur. 0 → pinhole (no DOF).
//    - focus_dist: Distance from the camera to the plane of perfect focus.
//                  Objects at this distance are sharp; closer/farther are blurred.
//
//  Implementation:
//    Instead of all rays originating from a single point (pinhole), they
//    originate from random points on a disk (the lens). This causes objects
//    not on the focal plane to appear blurred — exactly like a real camera.
// =============================================================================

struct Camera {
    Vec3 origin;
    Vec3 lower_left_corner;
    Vec3 horizontal;
    Vec3 vertical;
    Vec3 u, v, w;       // Camera orthonormal basis
    double lens_radius;  // Half of aperture diameter

    Camera(Vec3 lookfrom, Vec3 lookat, Vec3 vup,
           double vfov, double aspect_ratio,
           double aperture, double focus_dist)
    {
        double theta = vfov * PI / 180.0;
        double h = std::tan(theta / 2.0);
        double viewport_h = 2.0 * h;
        double viewport_w = aspect_ratio * viewport_h;

        // Camera basis vectors
        w = (lookfrom - lookat).normalized();
        u = vup.cross(w).normalized();
        v = w.cross(u);

        origin = lookfrom;
        // Scale viewport by focus_dist so the focal plane is at the right distance.
        horizontal = u * viewport_w * focus_dist;
        vertical   = v * viewport_h * focus_dist;
        lower_left_corner = origin - horizontal/2.0 - vertical/2.0 - w * focus_dist;

        lens_radius = aperture / 2.0;
    }

    // Generate a ray from a random point on the lens through viewport coords (s, t).
    Ray get_ray(double s, double t, RNG& rng) const {
        // Sample a random point on the lens disk for DOF blur.
        Vec3 rd = random_in_unit_disk(rng) * lens_radius;
        Vec3 offset = u * rd.x + v * rd.y;

        Vec3 dir = lower_left_corner + horizontal*s + vertical*t - origin - offset;
        return Ray(origin + offset, dir);
    }
};


// =============================================================================
//  SECTION 8: DYNAMIC SCENE GENERATOR
// =============================================================================
//  Procedurally generates a showcase scene with:
//    - A massive ground sphere
//    - Three large hero spheres: glass (center), matte (left), metal (right)
//    - Hundreds of small random spheres with varied materials
//
//  This creates a visually rich scene that exercises all material types
//  and generates heavy computational load for the parallelism demo.
// =============================================================================

std::vector<Sphere> random_scene(RNG& rng) {
    std::vector<Sphere> world;

    // --- Ground Plane -------------------------------------------------------
    world.push_back(Sphere(
        Vec3(0, -1000, 0), 1000,
        Material::lambertian(Vec3(0.5, 0.5, 0.5))
    ));

    // --- Scatter Small Spheres Across the Ground ----------------------------
    //  Loop over an 11x11 grid centered at origin, placing small spheres
    //  with random materials at each grid cell (with jitter).
    for (int a = -11; a < 11; ++a) {
        for (int b = -11; b < 11; ++b) {
            double choose_mat = rng.rand01();
            Vec3 center(a + 0.9*rng.rand01(), 0.2, b + 0.9*rng.rand01());

            // Skip if too close to the hero spheres.
            if ((center - Vec3(4, 0.2, 0)).length() < 0.9) continue;
            if ((center - Vec3(0, 0.2, 0)).length() < 0.9) continue;
            if ((center - Vec3(-4, 0.2, 0)).length() < 0.9) continue;

            if (choose_mat < 0.65) {
                // 65% chance: Diffuse (Lambertian)
                Vec3 albedo = Vec3(rng.rand01()*rng.rand01(),
                                   rng.rand01()*rng.rand01(),
                                   rng.rand01()*rng.rand01());
                world.push_back(Sphere(center, 0.2, Material::lambertian(albedo)));
            }
            else if (choose_mat < 0.85) {
                // 20% chance: Metal
                Vec3 albedo(rng.rand_range(0.5,1), rng.rand_range(0.5,1), rng.rand_range(0.5,1));
                double fuzz = rng.rand_range(0, 0.5);
                world.push_back(Sphere(center, 0.2, Material::metal(albedo, fuzz)));
            }
            else {
                // 15% chance: Glass (Dielectric)
                world.push_back(Sphere(center, 0.2, Material::glass(1.5)));
            }
        }
    }

    // --- Three Hero Spheres -------------------------------------------------
    //  Center: Large glass sphere (dielectric, n=1.5)
    world.push_back(Sphere(Vec3(0, 1, 0), 1.0, Material::glass(1.5)));

    //  Left: Large matte sphere (Lambertian, earthy brown)
    world.push_back(Sphere(Vec3(-4, 1, 0), 1.0, Material::lambertian(Vec3(0.4, 0.2, 0.1))));

    //  Right: Large metal sphere (polished copper/gold, near-zero fuzz)
    world.push_back(Sphere(Vec3(4, 1, 0), 1.0, Material::metal(Vec3(0.7, 0.6, 0.5), 0.0)));

    return world;
}


// =============================================================================
//  SECTION 9: TILE-BASED THREAD POOL WITH WORK-STEALING QUEUE
// =============================================================================
//  [UNIT V DEMONSTRATION: SYMMETRIC MULTIPROCESSING & DOMAIN DECOMPOSITION]
//
//  Instead of static row-band decomposition, we use a TILE-BASED work queue:
//
//  ┌────────────────────────────────────────────────────────────┐
//  │ Image divided into 32×32 pixel tiles pushed to a queue.   │
//  │                                                           │
//  │  ┌────┬────┬────┬────┬────┬────┬────┬────┐               │
//  │  │ T0 │ T1 │ T2 │ T3 │ T4 │ T5 │ T6 │ T7 │  ...         │
//  │  ├────┼────┼────┼────┼────┼────┼────┼────┤               │
//  │  │ T8 │ T9 │T10 │T11 │T12 │T13 │T14 │T15 │  ...         │
//  │  ├────┼────┼────┼────┼────┼────┼────┼────┤               │
//  │  │    │    │    │    │    │    │    │    │  ...            │
//  │  └────┴────┴────┴────┴────┴────┴────┴────┘               │
//  │                                                           │
//  │  Worker threads DYNAMICALLY pull tiles from the queue.    │
//  │  When a thread finishes one tile, it grabs the next.      │
//  │  This ensures ZERO idle cores — if one region is complex  │
//  │  (many reflections), other threads continue on simpler    │
//  │  tiles, achieving natural load balancing.                 │
//  │                                                           │
//  │  QUEUE ACCESS:                                            │
//  │  A Mutex protects the tile index. Each thread:            │
//  │    1. lock()                                              │
//  │    2. pop tile (increment shared index)                   │
//  │    3. unlock()                                            │
//  │    4. render tile (no lock held — pure computation)       │
//  │    5. atomically increment completed tile counter         │
//  │    6. repeat until queue empty                            │
//  └────────────────────────────────────────────────────────────┘
// =============================================================================

const int TILE_SIZE = 32; // Pixels per tile edge. 32x32 = 1024 pixels/tile.

struct Tile {
    int x0, y0; // Top-left pixel (inclusive)
    int x1, y1; // Bottom-right pixel (exclusive)
};

// --- Thread-Safe Work Queue -------------------------------------------------
//  Tiles are pre-loaded into a vector. An index tracks the next tile to assign.
//  Workers lock the mutex, read and increment the index, then unlock.
//  This is simpler and faster than a std::queue for our use case because
//  the full workload is known ahead of time.
struct WorkQueue {
    std::vector<Tile> tiles;
    int next_index;
    Mutex mutex;

    void init(const std::vector<Tile>& t) {
        tiles = t;
        next_index = 0;
        mutex.init();
    }

    // Attempt to pop the next tile. Returns false if queue is empty.
    bool pop(Tile& out) {
        mutex.lock();
        bool got = false;
        if (next_index < (int)tiles.size()) {
            out = tiles[next_index++];
            got = true;
        }
        mutex.unlock();
        return got;
    }

    void destroy() { mutex.destroy(); }
};

// --- Global State (shared by all threads) -----------------------------------
//  These globals are read-only during rendering (except progress_counter).
struct RenderContext {
    int image_width;
    int image_height;
    int samples_per_pixel;
    int max_depth;
    const Camera* camera;
    const std::vector<Sphere>* world;
    Vec3* pixel_buffer;
    WorkQueue* work_queue;
    AtomicInt* progress;
    int total_tiles;
    Mutex print_mutex;   // Protects std::cout for progress output
};

// --- Render a Single Tile ---------------------------------------------------
void render_tile(const Tile& tile, RenderContext* ctx, RNG& rng) {
    for (int j = tile.y0; j < tile.y1; ++j) {
        for (int i = tile.x0; i < tile.x1; ++i) {
            Vec3 color(0,0,0);

            // --- Anti-Aliasing: Monte Carlo Integration Over Pixel Area ---
            for (int s = 0; s < ctx->samples_per_pixel; ++s) {
                double u = (i + rng.rand01()) / (ctx->image_width - 1);
                double v = (j + rng.rand01()) / (ctx->image_height - 1);
                Ray r = ctx->camera->get_ray(u, v, rng);
                color += ray_color(r, *(ctx->world), ctx->max_depth, rng);
            }

            // Average and gamma correct (gamma = 2.0 → sqrt).
            double scale = 1.0 / ctx->samples_per_pixel;
            color *= scale;
            color = Vec3(std::sqrt(color.x), std::sqrt(color.y), std::sqrt(color.z));
            color = color.clamped();

            // Write to buffer (top-to-bottom for PPM).
            int row = ctx->image_height - 1 - j;
            ctx->pixel_buffer[row * ctx->image_width + i] = color;
        }
    }
}

// --- Worker Thread Function -------------------------------------------------
//  Each worker thread loops: pop a tile → render it → report progress.
//  When the queue is empty, the thread exits naturally.
void worker_func(int thread_id, RenderContext* ctx) {
    RNG rng((unsigned long long)(thread_id + 1) * 6364136223846793005ULL + 1442695040888963407ULL);

    Tile tile;
    while (ctx->work_queue->pop(tile)) {
        render_tile(tile, ctx, rng);

        int done = ctx->progress->increment();
        int total = ctx->total_tiles;

        // Print progress at every 2% milestone.
        int pct = (int)(100.0 * done / total);
        int prev = (int)(100.0 * (done - 1) / total);
        if (pct != prev || done == total) {
            ctx->print_mutex.lock();
            std::cout << "\r  Rendering: [";
            int bar_width = 40;
            int filled = (int)(bar_width * done / (double)total);
            for (int b = 0; b < bar_width; ++b)
                std::cout << (b < filled ? '#' : '-');
            std::cout << "] " << pct << "% (" << done << "/" << total << " tiles)" << std::flush;
            ctx->print_mutex.unlock();
        }
    }
}

// --- Platform Thread Entry Points -------------------------------------------
#ifdef _WIN32
struct ThreadStartData {
    int thread_id;
    RenderContext* ctx;
};

DWORD WINAPI win32_worker(LPVOID arg) {
    ThreadStartData* data = (ThreadStartData*)arg;
    worker_func(data->thread_id, data->ctx);
    return 0;
}
#else
struct ThreadStartData {
    int thread_id;
    RenderContext* ctx;
};

void* posix_worker(void* arg) {
    ThreadStartData* data = (ThreadStartData*)arg;
    worker_func(data->thread_id, data->ctx);
    return NULL;
}
#endif


// =============================================================================
//  SECTION 10: COMMAND-LINE INTERFACE (CLI) PARSER
// =============================================================================

struct Config {
    int width;
    int height;
    int samples;
    int threads;
    int max_depth;
    std::string output;
    bool show_help;

    Config() : width(1200), height(0), samples(50), threads(0),
               max_depth(50), output("lumina_render.ppm"), show_help(false) {}
};

void print_usage() {
    std::cout << "\n  PROJECT LUMINA v2.0 — Advanced Command-Line Rendering Engine\n\n";
    std::cout << "  Usage: lumina [options]\n\n";
    std::cout << "  Options:\n";
    std::cout << "    -w, --width    <int>   Image width in pixels      (default: 1200)\n";
    std::cout << "    -h, --height   <int>   Image height in pixels     (default: auto from 16:9)\n";
    std::cout << "    -s, --samples  <int>   Anti-aliasing samples/px   (default: 50)\n";
    std::cout << "    -t, --threads  <int>   Worker thread count        (default: all cores)\n";
    std::cout << "    -o, --out      <str>   Output filename            (default: lumina_render.ppm)\n";
    std::cout << "        --help             Show this help message\n\n";
    std::cout << "  Examples:\n";
    std::cout << "    lumina -w 1920 -s 200 -t 8 -o scene.ppm\n";
    std::cout << "    lumina --width 640 --samples 10     (fast preview)\n\n";
}

Config parse_args(int argc, char* argv[]) {
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0) {
            cfg.show_help = true;
        }
        else if ((strcmp(argv[i], "-w") == 0 || strcmp(argv[i], "--width") == 0) && i+1 < argc) {
            cfg.width = atoi(argv[++i]);
        }
        else if ((strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--height") == 0) && i+1 < argc) {
            cfg.height = atoi(argv[++i]);
        }
        else if ((strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--samples") == 0) && i+1 < argc) {
            cfg.samples = atoi(argv[++i]);
        }
        else if ((strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--threads") == 0) && i+1 < argc) {
            cfg.threads = atoi(argv[++i]);
        }
        else if ((strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--out") == 0) && i+1 < argc) {
            cfg.output = argv[++i];
        }
        else {
            std::cerr << "  Warning: Unknown argument '" << argv[i] << "'\n";
        }
    }
    return cfg;
}


// =============================================================================
//  SECTION 11: TERMINAL OUTPUT FORMATTING
// =============================================================================

void pad_to(int len, int target) {
    for (int i = len; i < target; ++i) std::cout << " ";
}

void print_banner() {
    std::cout << "\n";
    std::cout << "  +==============================================================+\n";
    std::cout << "  |        PROJECT LUMINA v2.0 -- Rendering Engine                |\n";
    std::cout << "  |   Advanced Path Tracer with Tile-Based Thread Pool            |\n";
    std::cout << "  |   Unit V: Parallel Organization Demonstration                 |\n";
    std::cout << "  +==============================================================+\n\n";
}

void print_config_table(const Config& cfg, int sphere_count, int tile_count) {
    std::cout << "  +------------------ RENDER CONFIGURATION --------------------+\n";

    std::ostringstream s;

    s.str(""); s << cfg.width << " x " << cfg.height << " pixels (" << (cfg.width * cfg.height) << " total)";
    std::cout << "  |  Resolution      : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << cfg.samples << " rays/pixel";
    std::cout << "  |  Anti-Aliasing    : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << cfg.max_depth << " bounces";
    std::cout << "  |  Max Ray Depth   : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << sphere_count << " spheres";
    std::cout << "  |  Scene Geometry   : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << tile_count << " tiles (" << TILE_SIZE << "x" << TILE_SIZE << " px each)";
    std::cout << "  |  Work Units       : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << cfg.threads << " threads";
    std::cout << "  |  Worker Threads   : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    std::string mode = (cfg.threads == 1) ? "SEQUENTIAL (Single-Core)" : "PARALLEL (Multi-Core SMP)";
    std::cout << "  |  Execution Mode   : " << mode; pad_to((int)mode.size(), 39); std::cout << "|\n";

    std::cout << "  |  Output File      : " << cfg.output; pad_to((int)cfg.output.size(), 39); std::cout << "|\n";

    std::cout << "  +------------------------------------------------------------+\n\n";
}


// =============================================================================
//  MAIN — ENGINE ORCHESTRATION
// =============================================================================

int main(int argc, char* argv[]) {

    // --- Parse Command-Line Arguments ---------------------------------------
    Config cfg = parse_args(argc, argv);

    if (cfg.show_help) {
        print_usage();
        return 0;
    }

    // Apply defaults for unspecified values.
    const double ASPECT_RATIO = 16.0 / 9.0;
    if (cfg.width < 1) cfg.width = 1200;
    if (cfg.height < 1) cfg.height = (int)(cfg.width / ASPECT_RATIO);
    if (cfg.samples < 1) cfg.samples = 50;
    if (cfg.threads < 1) cfg.threads = get_hardware_threads();
    if (cfg.threads < 1) cfg.threads = 1;

    const int IMAGE_WIDTH  = cfg.width;
    const int IMAGE_HEIGHT = cfg.height;
    const int SAMPLES      = cfg.samples;
    const int MAX_DEPTH    = cfg.max_depth;
    const int NUM_THREADS  = cfg.threads;

    // --- Generate Scene -----------------------------------------------------
    RNG scene_rng(42);
    std::vector<Sphere> world = random_scene(scene_rng);

    // --- Setup Camera -------------------------------------------------------
    Vec3 lookfrom(13, 2, 3);
    Vec3 lookat(0, 0, 0);
    Vec3 vup(0, 1, 0);
    double vfov = 20.0;
    double aperture = 0.1;       // Subtle depth-of-field
    double focus_dist = 10.0;    // Focus plane at the hero spheres

    Camera cam(lookfrom, lookat, vup, vfov,
               (double)IMAGE_WIDTH / IMAGE_HEIGHT,
               aperture, focus_dist);

    // --- Allocate Pixel Buffer ----------------------------------------------
    std::vector<Vec3> pixel_buffer(IMAGE_WIDTH * IMAGE_HEIGHT);

    // --- Generate Tile Work Queue -------------------------------------------
    std::vector<Tile> tiles;
    for (int y = 0; y < IMAGE_HEIGHT; y += TILE_SIZE) {
        for (int x = 0; x < IMAGE_WIDTH; x += TILE_SIZE) {
            Tile t;
            t.x0 = x;
            t.y0 = y;
            t.x1 = std::min(x + TILE_SIZE, IMAGE_WIDTH);
            t.y1 = std::min(y + TILE_SIZE, IMAGE_HEIGHT);
            tiles.push_back(t);
        }
    }

    WorkQueue queue;
    queue.init(tiles);

    // --- Print Configuration ------------------------------------------------
    print_banner();
    print_config_table(cfg, (int)world.size(), (int)tiles.size());

    std::cout << "  >> Materials: Lambertian (diffuse) + Metal (reflective) + Dielectric (glass)\n";
    std::cout << "  >> Camera: DOF enabled (aperture=" << aperture << ", focus=" << focus_dist << ")\n\n";

    // =========================================================================
    //  [UNIT V DEMONSTRATION: SYMMETRIC MULTIPROCESSING & DOMAIN DECOMPOSITION]
    //
    //  TILE-BASED WORK QUEUE PARALLELISM:
    //
    //  1. The image is divided into TILE_SIZE x TILE_SIZE pixel tiles.
    //  2. All tiles are pushed to a shared, mutex-protected work queue.
    //  3. NUM_THREADS worker threads are spawned.
    //  4. Each worker thread DYNAMICALLY pulls tiles from the queue,
    //     renders them, and increments a shared atomic progress counter.
    //  5. When the queue is empty, each thread exits.
    //  6. The main thread joins all workers (synchronization barrier).
    //
    //  ADVANTAGES OVER STATIC DECOMPOSITION:
    //  - DYNAMIC LOAD BALANCING: If some tiles are computationally heavier
    //    (e.g., many glass spheres with deep recursion), faster threads pick
    //    up slack by rendering more of the simpler tiles. No core sits idle.
    //  - SCALABILITY: Works efficiently whether NUM_THREADS = 1 or 64.
    //  - CACHE-FRIENDLY: 32x32 tiles fit well within L1 cache.
    // =========================================================================

    // --- Setup Render Context -----------------------------------------------
    AtomicInt progress;
    progress.set(0);

    RenderContext ctx;
    ctx.image_width = IMAGE_WIDTH;
    ctx.image_height = IMAGE_HEIGHT;
    ctx.samples_per_pixel = SAMPLES;
    ctx.max_depth = MAX_DEPTH;
    ctx.camera = &cam;
    ctx.world = &world;
    ctx.pixel_buffer = &pixel_buffer[0];
    ctx.work_queue = &queue;
    ctx.progress = &progress;
    ctx.total_tiles = (int)tiles.size();
    ctx.print_mutex.init();

    // --- Start Timer --------------------------------------------------------
    auto t_start = std::chrono::high_resolution_clock::now();

    // --- Spawn Worker Threads -----------------------------------------------
    std::vector<ThreadStartData> thread_data(NUM_THREADS);

#ifdef _WIN32
    std::vector<HANDLE> handles(NUM_THREADS);
    for (int i = 0; i < NUM_THREADS; ++i) {
        thread_data[i].thread_id = i;
        thread_data[i].ctx = &ctx;
        handles[i] = CreateThread(NULL, 0, win32_worker, &thread_data[i], 0, NULL);
        if (handles[i] == NULL) {
            std::cerr << "\n  ERROR: Failed to create thread " << i << "\n";
            return 1;
        }
    }

    // --- Synchronization Barrier: Wait for all threads ----------------------
    WaitForMultipleObjects(NUM_THREADS, &handles[0], TRUE, INFINITE);
    for (int i = 0; i < NUM_THREADS; ++i) CloseHandle(handles[i]);

#else
    std::vector<pthread_t> pthreads(NUM_THREADS);
    for (int i = 0; i < NUM_THREADS; ++i) {
        thread_data[i].thread_id = i;
        thread_data[i].ctx = &ctx;
        if (pthread_create(&pthreads[i], NULL, posix_worker, &thread_data[i]) != 0) {
            std::cerr << "\n  ERROR: Failed to create thread " << i << "\n";
            return 1;
        }
    }
    for (int i = 0; i < NUM_THREADS; ++i) pthread_join(pthreads[i], NULL);
#endif

    // --- Stop Timer ---------------------------------------------------------
    auto t_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = t_end - t_start;

    std::cout << "\n\n  >> All " << NUM_THREADS << " threads completed.\n\n";

    // --- Write PPM File -----------------------------------------------------
    std::cout << "  >> Writing image to " << cfg.output << "... ";
    std::cout.flush();

    std::ofstream file(cfg.output.c_str());
    if (!file.is_open()) {
        std::cerr << "\n  ERROR: Cannot open " << cfg.output << " for writing.\n";
        return 1;
    }

    file << "P3\n" << IMAGE_WIDTH << " " << IMAGE_HEIGHT << "\n255\n";
    for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; ++i) {
        int r = (int)(255.999 * pixel_buffer[i].x);
        int g = (int)(255.999 * pixel_buffer[i].y);
        int b = (int)(255.999 * pixel_buffer[i].z);
        file << r << " " << g << " " << b << "\n";
    }
    file.close();
    std::cout << "Done.\n\n";

    // --- Performance Report -------------------------------------------------
    long long total_rays = (long long)IMAGE_WIDTH * IMAGE_HEIGHT * SAMPLES;

    std::cout << "  +==============================================================+\n";
    std::cout << "  |                    PERFORMANCE REPORT                         |\n";
    std::cout << "  +==============================================================+\n";

    std::ostringstream s;

    s.str(""); s << NUM_THREADS;
    std::cout << "  |  Worker Threads   : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << (int)tiles.size() << " tiles (" << TILE_SIZE << "x" << TILE_SIZE << ")";
    std::cout << "  |  Work Units       : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << (IMAGE_WIDTH * IMAGE_HEIGHT);
    std::cout << "  |  Total Pixels     : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << total_rays;
    std::cout << "  |  Primary Rays     : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << (int)world.size();
    std::cout << "  |  Scene Spheres    : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << std::fixed; s.precision(4); s << elapsed.count() << " seconds";
    std::cout << "  |  Render Time      : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    double mrays = total_rays / elapsed.count() / 1e6;
    s.str(""); s << std::fixed; s.precision(2); s << mrays << " Mrays/sec";
    std::cout << "  |  Throughput       : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << cfg.output;
    std::cout << "  |  Output File      : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    std::cout << "  +--------------------------------------------------------------+\n";

    if (NUM_THREADS == 1) {
        std::cout << "  |  MODE: SEQUENTIAL (Baseline)                                |\n";
        std::cout << "  |  Re-run with -t N to observe parallel speedup.              |\n";
    } else {
        std::cout << "  |  MODE: PARALLEL (Tile-Based SMP Thread Pool)                |\n";
        std::cout << "  |  Compare with -t 1 to measure speedup ratio.                |\n";
    }

    std::cout << "  +==============================================================+\n\n";
    std::cout << "  >> Render complete. Open " << cfg.output << " to view the result.\n\n";

    // Cleanup
    queue.destroy();
    ctx.print_mutex.destroy();

    return 0;
}

// =============================================================================
//  COMPILATION INSTRUCTIONS
// =============================================================================
//
//  +-------------------------------------------------------------------------+
//  |  WINDOWS (MinGW g++) — YOUR SETUP                                       |
//  |  ────────────────────────────────                                        |
//  |  g++ -std=c++11 -O3 -o lumina.exe main.cpp                              |
//  |  .\lumina.exe                                                            |
//  |  .\lumina.exe -w 1920 -s 200 -t 8 -o hires.ppm                          |
//  |  .\lumina.exe -w 400 -s 10 -t 1       (fast single-core preview)        |
//  |                                                                          |
//  |  WINDOWS (MSVC — Developer Command Prompt)                               |
//  |  ─────────────────────────────────────────                               |
//  |  cl /EHsc /O2 /std:c++14 main.cpp /Fe:lumina.exe                         |
//  |  .\lumina.exe                                                            |
//  |                                                                          |
//  |  LINUX / macOS (g++ or clang++)                                          |
//  |  ──────────────────────────────                                          |
//  |  g++ -std=c++11 -O3 -pthread -o lumina main.cpp                          |
//  |  ./lumina -w 1200 -s 100 -t 8 -o render.ppm                             |
//  |                                                                          |
//  |  PERFORMANCE TIPS:                                                       |
//  |  • -O3 enables aggressive optimizations (auto-vectorization, inlining).  |
//  |  • -march=native tunes for your specific CPU (SSE/AVX).                  |
//  |  • Higher -s (samples) = smoother image but longer render.               |
//  |  • Lower -w = faster preview renders for testing.                        |
//  +-------------------------------------------------------------------------+
//
// =============================================================================
