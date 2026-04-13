// =============================================================================
//  PROJECT LUMINA v3.0 — Portfolio-Grade Command-Line Rendering Engine
// =============================================================================
//
//  A production-quality, physically-based path tracer featuring:
//
//    * LIVE ASCII PROGRESS MAP — real-time tile completion visualization
//    * TILE-BASED THREAD POOL  — dynamic load balancing via work-stealing queue
//    * PBR MATERIALS           — Lambertian, Metal, Dielectric (Snell + Schlick)
//    * DEPTH-OF-FIELD CAMERA   — adjustable aperture and focal distance
//    * RANDOM SCENE GENERATOR  — hundreds of spheres with varied materials
//    * CLI ENGINE              — full command-line control of all parameters
//
//  Designed for academic presentation: Unit V — Parallel Organization
//  Demonstrates: Symmetric Multiprocessing & Domain Decomposition
//
//  Dependencies: ZERO external libraries.
//  Compiler:     C++11 or newer. MinGW, MSVC, GCC, Clang.
//  Output:       .ppm (Portable Pixmap) rendered to disk.
//
// =============================================================================

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
#include <cstdio>

// --- Platform-Specific Headers ----------------------------------------------
//  Windows: Win32 API for threads, critical sections, atomic ops, console control.
//  POSIX:   pthreads for threads, sync intrinsics for atomics.
#ifdef _WIN32
    #include <windows.h>
#else
    #include <pthread.h>
    #include <unistd.h>
#endif


// =============================================================================
//  PLATFORM ABSTRACTION LAYER
// =============================================================================
//  Portable wrappers around OS primitives so the engine code is platform-agnostic.
// =============================================================================

// --- Mutex (Critical Section) -----------------------------------------------
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

// --- Atomic Integer (lock-free thread-safe counter) -------------------------
struct AtomicInt {
#ifdef _WIN32
    volatile LONG value;
    void set(int v)  { InterlockedExchange(&value, (LONG)v); }
    int  get()       { return (int)value; }
    int  increment() { return (int)InterlockedIncrement(&value); }
#else
    volatile int value;
    void set(int v)  { __sync_lock_test_and_set(&value, v); }
    int  get()       { return __sync_fetch_and_add(&value, 0); }
    int  increment() { return __sync_fetch_and_add(&value, 1) + 1; }
#endif
};

// --- Hardware Thread Count --------------------------------------------------
int get_hw_threads() {
#ifdef _WIN32
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return (int)si.dwNumberOfProcessors;
#else
    int n = (int)sysconf(_SC_NPROCESSORS_ONLN);
    return (n > 0) ? n : 1;
#endif
}

// --- Console Cursor Control (for Live Progress Map) -------------------------
#ifdef _WIN32
int get_cursor_row() {
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi);
    return (int)csbi.dwCursorPosition.Y;
}
void set_cursor(int col, int row) {
    COORD pos = {(SHORT)col, (SHORT)row};
    SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), pos);
}
#endif


// =============================================================================
//  CONSTANTS
// =============================================================================
const double PI      = 3.14159265358979323846;
const double INF     = 1e30;
const double EPSILON = 0.001;  // Shadow acne prevention


// =============================================================================
//  SECTION 1: Vec3 — 3D Vector / Color / Point
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
    Vec3& operator+=(const Vec3& v)     { x+=v.x; y+=v.y; z+=v.z; return *this; }
    Vec3& operator*=(double t)          { x*=t; y*=t; z*=t; return *this; }

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

    bool near_zero() const {
        return std::fabs(x) < 1e-8 && std::fabs(y) < 1e-8 && std::fabs(z) < 1e-8;
    }

    // Reflect V about surface normal N.
    static Vec3 reflect(const Vec3& v, const Vec3& n) {
        return v - n * 2.0 * v.dot(n);
    }

    // Refract UV through surface with normal N (Snell's Law in vector form).
    //   r_perp     = eta * (uv + cos_theta * n)
    //   r_parallel = -sqrt(1 - |r_perp|^2) * n
    static Vec3 refract(const Vec3& uv, const Vec3& n, double eta) {
        double cos_theta = std::fmin((-uv).dot(n), 1.0);
        Vec3 r_perp = (uv + n * cos_theta) * eta;
        Vec3 r_para = n * (-std::sqrt(std::fabs(1.0 - r_perp.length_squared())));
        return r_perp + r_para;
    }

    Vec3 clamped() const {
        return Vec3(std::fmax(0.0, std::fmin(1.0, x)),
                    std::fmax(0.0, std::fmin(1.0, y)),
                    std::fmax(0.0, std::fmin(1.0, z)));
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
//  SECTION 3: Per-Thread RNG (xorshift64)
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

    double rand01() {
        return (next() & 0xFFFFFFFFFFFFFULL) / (double)(0x10000000000000ULL);
    }

    double rr(double lo, double hi) { return lo + (hi - lo) * rand01(); }
};

Vec3 random_in_unit_sphere(RNG& rng) {
    while (true) {
        Vec3 p(rng.rr(-1,1), rng.rr(-1,1), rng.rr(-1,1));
        if (p.length_squared() < 1.0) return p;
    }
}

Vec3 random_unit_vector(RNG& rng) {
    return random_in_unit_sphere(rng).normalized();
}

Vec3 random_in_unit_disk(RNG& rng) {
    while (true) {
        Vec3 p(rng.rr(-1,1), rng.rr(-1,1), 0);
        if (p.length_squared() < 1.0) return p;
    }
}


// =============================================================================
//  SECTION 4: PHYSICALLY-BASED MATERIALS
// =============================================================================
//
//  ┌──────────────┬──────────────────────────────────────────────────────┐
//  │ LAMBERTIAN   │ Diffuse: scatters in random hemisphere direction.   │
//  │              │ Albedo = surface color / absorption spectrum.       │
//  ├──────────────┼──────────────────────────────────────────────────────┤
//  │ METAL        │ Specular reflection about the normal.              │
//  │              │ param = fuzz (0=mirror, 1=rough).                  │
//  ├──────────────┼──────────────────────────────────────────────────────┤
//  │ DIELECTRIC   │ Glass: refraction via Snell's Law.                 │
//  │              │ param = refractive index (glass=1.5, water=1.33).  │
//  │              │ Fresnel reflectance via Schlick's approximation.   │
//  │              │ Total internal reflection when angle exceeds       │
//  │              │ the critical angle.                                │
//  └──────────────┴──────────────────────────────────────────────────────┘
//
// =============================================================================

enum MaterialType { MAT_LAMBERTIAN, MAT_METAL, MAT_DIELECTRIC };

struct Material {
    MaterialType type;
    Vec3 albedo;
    double param; // fuzz (Metal) or refractive index (Dielectric)

    Material() : type(MAT_LAMBERTIAN), albedo(Vec3(0.5,0.5,0.5)), param(0) {}
    Material(MaterialType t, const Vec3& a, double p) : type(t), albedo(a), param(p) {}

    static Material lambertian(const Vec3& c)       { return Material(MAT_LAMBERTIAN, c, 0); }
    static Material metal(const Vec3& c, double f)  { return Material(MAT_METAL, c, std::fmin(f,1.0)); }
    static Material glass(double ior)               { return Material(MAT_DIELECTRIC, Vec3(1,1,1), ior); }
};

// Schlick's approximation for Fresnel reflectance.
// R(theta) = R0 + (1-R0)(1-cos(theta))^5  where  R0 = ((n1-n2)/(n1+n2))^2
double schlick(double cosine, double ref_idx) {
    double r0 = (1.0 - ref_idx) / (1.0 + ref_idx);
    r0 = r0 * r0;
    return r0 + (1.0 - r0) * std::pow(1.0 - cosine, 5.0);
}


// =============================================================================
//  SECTION 5: GEOMETRY — HitRecord & Sphere
// =============================================================================

struct HitRecord {
    Vec3 p, normal;
    double t;
    bool front_face;
    Material material;

    void set_face_normal(const Ray& r, const Vec3& outward_n) {
        front_face = r.direction.dot(outward_n) < 0;
        normal = front_face ? outward_n : -outward_n;
    }
};

struct Sphere {
    Vec3 center;
    double radius;
    Material material;

    Sphere() : radius(0) {}
    Sphere(const Vec3& c, double r, const Material& m) : center(c), radius(r), material(m) {}

    // Ray-Sphere intersection (quadratic with half-b optimization).
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
//  SECTION 6: SCENE INTERSECTION & PATH TRACING
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

// --- Recursive Path Tracer --------------------------------------------------
//  For each ray:
//    1. Find closest intersection.
//    2. Scatter based on material (diffuse, metal, glass).
//    3. Recurse with scattered ray, attenuating by albedo.
//    4. If nothing hit, return sky gradient (environment light).
//    5. If max depth reached, return black (energy absorbed).
// ---------------------------------------------------------------------------
Vec3 ray_color(const Ray& r, const std::vector<Sphere>& world, int depth, RNG& rng) {
    if (depth <= 0) return Vec3(0,0,0);

    HitRecord rec;
    if (!hit_scene(world, r, EPSILON, INF, rec)) {
        // Sky gradient: white at horizon, blue at zenith.
        Vec3 ud = r.direction.normalized();
        double t = 0.5 * (ud.y + 1.0);
        return Vec3(1,1,1) * (1.0-t) + Vec3(0.5, 0.7, 1.0) * t;
    }

    Vec3 attenuation;
    Ray scattered;

    switch (rec.material.type) {

    case MAT_LAMBERTIAN: {
        Vec3 dir = rec.normal + random_unit_vector(rng);
        if (dir.near_zero()) dir = rec.normal;
        scattered = Ray(rec.p, dir);
        attenuation = rec.material.albedo;
        break;
    }

    case MAT_METAL: {
        Vec3 refl = Vec3::reflect(r.direction.normalized(), rec.normal);
        Vec3 fuzzed = refl + random_in_unit_sphere(rng) * rec.material.param;
        scattered = Ray(rec.p, fuzzed);
        attenuation = rec.material.albedo;
        if (scattered.direction.dot(rec.normal) <= 0) return Vec3(0,0,0);
        break;
    }

    case MAT_DIELECTRIC: {
        // Glass: refractive index stored in material.param.
        // Determine if ray is entering (front_face) or exiting the glass.
        attenuation = Vec3(1,1,1); // Glass is perfectly transparent.
        double ri = rec.front_face ? (1.0 / rec.material.param) : rec.material.param;

        Vec3 ud = r.direction.normalized();
        double cos_theta = std::fmin((-ud).dot(rec.normal), 1.0);
        double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);

        // Total Internal Reflection: when ri * sin(theta) > 1,
        // refraction is physically impossible. Light bounces back.
        bool cannot_refract = (ri * sin_theta) > 1.0;

        Vec3 dir;
        if (cannot_refract || schlick(cos_theta, ri) > rng.rand01()) {
            dir = Vec3::reflect(ud, rec.normal);   // Reflect
        } else {
            dir = Vec3::refract(ud, rec.normal, ri); // Refract
        }
        scattered = Ray(rec.p, dir);
        break;
    }

    default: return Vec3(0,0,0);
    }

    return attenuation * ray_color(scattered, world, depth - 1, rng);
}


// =============================================================================
//  SECTION 7: CAMERA (Depth-of-Field)
// =============================================================================
//  Thin-lens camera model. Rays originate from random points on the lens disk,
//  converging at the focal plane. Objects at focus_dist are sharp; others blur.
// =============================================================================

struct Camera {
    Vec3 origin, lower_left, horizontal, vertical;
    Vec3 u, v, w;
    double lens_radius;

    Camera(Vec3 lookfrom, Vec3 lookat, Vec3 vup,
           double vfov, double aspect, double aperture, double focus_dist)
    {
        double theta = vfov * PI / 180.0;
        double h = std::tan(theta / 2.0);
        double vp_h = 2.0 * h;
        double vp_w = aspect * vp_h;

        w = (lookfrom - lookat).normalized();
        u = vup.cross(w).normalized();
        v = w.cross(u);

        origin     = lookfrom;
        horizontal = u * vp_w * focus_dist;
        vertical   = v * vp_h * focus_dist;
        lower_left = origin - horizontal/2.0 - vertical/2.0 - w * focus_dist;
        lens_radius = aperture / 2.0;
    }

    Ray get_ray(double s, double t, RNG& rng) const {
        Vec3 rd = random_in_unit_disk(rng) * lens_radius;
        Vec3 offset = u * rd.x + v * rd.y;
        return Ray(origin + offset,
                   lower_left + horizontal*s + vertical*t - origin - offset);
    }
};


// =============================================================================
//  SECTION 8: RANDOM SCENE GENERATOR
// =============================================================================
//  Populates the world with a ground plane, three hero spheres, and hundreds
//  of small random spheres with varied materials.
// =============================================================================

std::vector<Sphere> random_scene(RNG& rng) {
    std::vector<Sphere> world;

    // Ground
    world.push_back(Sphere(Vec3(0,-1000,0), 1000, Material::lambertian(Vec3(0.5,0.5,0.5))));

    // Random small spheres
    for (int a = -11; a < 11; ++a) {
        for (int b = -11; b < 11; ++b) {
            double mat = rng.rand01();
            Vec3 center(a + 0.9*rng.rand01(), 0.2, b + 0.9*rng.rand01());

            if ((center - Vec3(4,0.2,0)).length() < 0.9) continue;
            if ((center - Vec3(0,0.2,0)).length() < 0.9) continue;
            if ((center - Vec3(-4,0.2,0)).length() < 0.9) continue;

            if (mat < 0.65) {
                // Lambertian
                Vec3 albedo(rng.rand01()*rng.rand01(), rng.rand01()*rng.rand01(), rng.rand01()*rng.rand01());
                world.push_back(Sphere(center, 0.2, Material::lambertian(albedo)));
            } else if (mat < 0.85) {
                // Metal
                Vec3 albedo(rng.rr(0.5,1), rng.rr(0.5,1), rng.rr(0.5,1));
                world.push_back(Sphere(center, 0.2, Material::metal(albedo, rng.rr(0,0.5))));
            } else {
                // Glass
                world.push_back(Sphere(center, 0.2, Material::glass(1.5)));
            }
        }
    }

    // Three hero spheres
    world.push_back(Sphere(Vec3( 0,1,0), 1.0, Material::glass(1.5)));                        // Center: Glass
    world.push_back(Sphere(Vec3(-4,1,0), 1.0, Material::lambertian(Vec3(0.4, 0.2, 0.1))));   // Left: Matte
    world.push_back(Sphere(Vec3( 4,1,0), 1.0, Material::metal(Vec3(0.7, 0.6, 0.5), 0.0)));   // Right: Metal

    return world;
}


// =============================================================================
//  SECTION 9: TILE-BASED WORK QUEUE
// =============================================================================
//  The image is divided into TILE_SIZE x TILE_SIZE pixel tiles. All tiles are
//  loaded into a shared queue. Worker threads dynamically pull tiles until the
//  queue is empty. A Mutex serializes access to the queue index.
//
//  This achieves DYNAMIC LOAD BALANCING: if some tiles are computationally
//  heavier (glass reflections, dense sphere clusters), faster threads
//  automatically pick up more of the simpler tiles. Zero idle cores.
// =============================================================================

const int TILE_SIZE = 16;

struct Tile {
    int x0, y0, x1, y1; // Pixel bounds [x0,x1) x [y0,y1)
};

struct WorkQueue {
    std::vector<Tile> tiles;
    int next_idx;
    Mutex mutex;

    void init(const std::vector<Tile>& t) { tiles = t; next_idx = 0; mutex.init(); }
    bool pop(Tile& out) {
        mutex.lock();
        bool got = (next_idx < (int)tiles.size());
        if (got) out = tiles[next_idx++];
        mutex.unlock();
        return got;
    }
    void destroy() { mutex.destroy(); }
};


// =============================================================================
//  SECTION 10: LIVE ASCII PROGRESS MAP
// =============================================================================
//  [UNIT V DEMONSTRATION: SYMMETRIC MULTIPROCESSING & DOMAIN DECOMPOSITION]
//
//  The progress map is a real-time terminal visualization of the render:
//
//    +-- Live Render Map (50 x 29 tiles, char = thread ID) --+
//    |..................................................|
//    |..0011..3322.44.......55667.......................|
//    |.0011.33224.44.556667..............................|
//    |..................................................|
//    +--------------------------------------------------+
//    Progress: [################------------------------] 42%
//
//  Each '.' is a pending tile. As threads complete tiles, their ID number
//  (0-9, A-Z) replaces the dot IN REAL-TIME. This directly visualizes:
//    - Which core rendered which region of the image
//    - How dynamic load balancing distributes work
//    - The parallel wavefront sweeping across the domain
//
//  WINDOWS: Uses SetConsoleCursorPosition for flicker-free in-place updates.
//  POSIX:   Uses carriage-return progress bar + post-render completion map.
// =============================================================================

struct ProgressMap {
    int tiles_x, tiles_y, total;
    std::vector<char> grid;
    AtomicInt completed;
    Mutex print_mutex;

    // Console coordinates (Windows live map)
    int grid_row;     // Buffer row of first grid line
    int grid_col;     // Buffer column of first tile char
    int progress_row; // Buffer row of progress bar
    bool live_enabled; // True if terminal is large enough for live map

    void init(int tx, int ty) {
        tiles_x = tx;
        tiles_y = ty;
        total = tx * ty;
        grid.assign(tx * ty, '.');
        completed.set(0);
        print_mutex.init();
        grid_col = 3; // "  |" = 3 chars before first tile

        // Enable live map only if grid fits reasonably in terminal.
        live_enabled = (tiles_x <= 80 && tiles_y <= 40);
    }

    // Print the initial empty map to the terminal.
    void print_initial() {
        if (!live_enabled) {
            std::cout << "  (Grid too large for live map. Showing progress bar.)\n\n";
            return;
        }

        std::cout << "\n";
        std::cout << "  Live Render Map (" << tiles_x << " x " << tiles_y
                  << " tiles, char = thread ID)\n";

        // Top border
        std::cout << "  +";
        for (int i = 0; i < tiles_x; ++i) std::cout << "-";
        std::cout << "+\n";

        std::cout.flush();

#ifdef _WIN32
        grid_row = get_cursor_row();
#else
        grid_row = 0; // Tracked manually (not used for POSIX live map)
#endif

        // Grid rows (all dots)
        for (int y = 0; y < tiles_y; ++y) {
            std::cout << "  |";
            for (int x = 0; x < tiles_x; ++x) std::cout << ".";
            std::cout << "|\n";
        }

        // Bottom border
        std::cout << "  +";
        for (int i = 0; i < tiles_x; ++i) std::cout << "-";
        std::cout << "+\n";

        std::cout.flush();

#ifdef _WIN32
        progress_row = get_cursor_row();
#endif

        // Initial progress bar
        std::cout << "  Progress: [";
        for (int i = 0; i < 50; ++i) std::cout << "-";
        std::cout << "] 0% (0/" << total << " tiles)   \n\n";
        std::cout.flush();
    }

    // Called by worker threads after each tile completion.
    void update(int tx, int ty, int thread_id) {
        // Determine the display character for this thread.
        char ch;
        if (thread_id < 10) ch = '0' + thread_id;
        else ch = 'A' + (thread_id - 10);

        grid[ty * tiles_x + tx] = ch;
        int done = completed.increment();
        int pct = (int)(100.0 * done / total);

        print_mutex.lock();

#ifdef _WIN32
        if (live_enabled) {
            HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);

            // --- Update the tile character in the live map ---
            COORD tile_pos = {(SHORT)(grid_col + tx), (SHORT)(grid_row + ty)};
            SetConsoleCursorPosition(h, tile_pos);
            putchar(ch);

            // --- Update the progress bar ---
            COORD prog_pos = {0, (SHORT)progress_row};
            SetConsoleCursorPosition(h, prog_pos);

            int bar_w = 50;
            int filled = bar_w * done / total;
            printf("  Progress: [");
            for (int i = 0; i < bar_w; ++i) putchar(i < filled ? '#' : '-');
            printf("] %d%% (%d/%d tiles)   ", pct, done, total);

            // Park cursor below the map.
            COORD park = {0, (SHORT)(progress_row + 2)};
            SetConsoleCursorPosition(h, park);
            fflush(stdout);
        } else {
            // Fallback: simple progress bar with carriage return.
            int bar_w = 50;
            int filled = bar_w * done / total;
            printf("\r  Progress: [");
            for (int i = 0; i < bar_w; ++i) putchar(i < filled ? '#' : '-');
            printf("] %d%% (%d/%d tiles)   ", pct, done, total);
            fflush(stdout);
        }
#else
        // POSIX: carriage-return progress bar (no cursor positioning).
        int bar_w = 50;
        int filled = bar_w * done / total;
        printf("\r  Progress: [");
        for (int i = 0; i < bar_w; ++i) putchar(i < filled ? '#' : '-');
        printf("] %d%% (%d/%d tiles)   ", pct, done, total);
        fflush(stdout);
#endif

        print_mutex.unlock();
    }

    // Print the completion map after rendering (for POSIX or as summary).
    void print_completion_map() {
#ifndef _WIN32
        // On POSIX, the live map wasn't shown. Print the final result.
        std::cout << "\n\n  Completion Map (char = thread ID):\n";
        std::cout << "  +";
        for (int i = 0; i < tiles_x; ++i) std::cout << "-";
        std::cout << "+\n";
        for (int y = 0; y < tiles_y; ++y) {
            std::cout << "  |";
            for (int x = 0; x < tiles_x; ++x)
                std::cout << grid[y * tiles_x + x];
            std::cout << "|\n";
        }
        std::cout << "  +";
        for (int i = 0; i < tiles_x; ++i) std::cout << "-";
        std::cout << "+\n";
#endif
    }

    void destroy() { print_mutex.destroy(); }
};


// =============================================================================
//  SECTION 11: RENDER WORKER
// =============================================================================

struct RenderContext {
    int image_width, image_height;
    int samples, max_depth;
    const Camera* camera;
    const std::vector<Sphere>* world;
    Vec3* pixel_buffer;
    WorkQueue* queue;
    ProgressMap* progress_map;
};

// Render all pixels within a single tile.
void render_tile(const Tile& tile, RenderContext* ctx, RNG& rng) {
    for (int j = tile.y0; j < tile.y1; ++j) {
        for (int i = tile.x0; i < tile.x1; ++i) {
            Vec3 color(0,0,0);
            for (int s = 0; s < ctx->samples; ++s) {
                double u = (i + rng.rand01()) / (ctx->image_width - 1);
                double v = (j + rng.rand01()) / (ctx->image_height - 1);
                color += ray_color(ctx->camera->get_ray(u, v, rng),
                                   *(ctx->world), ctx->max_depth, rng);
            }
            double scale = 1.0 / ctx->samples;
            color *= scale;
            color = Vec3(std::sqrt(color.x), std::sqrt(color.y), std::sqrt(color.z)).clamped();
            int row = ctx->image_height - 1 - j;
            ctx->pixel_buffer[row * ctx->image_width + i] = color;
        }
    }
}

// Worker thread main loop: pull tiles, render, update progress.
void worker_func(int thread_id, RenderContext* ctx) {
    RNG rng((unsigned long long)(thread_id + 1) * 6364136223846793005ULL + 1442695040888963407ULL);
    Tile tile;
    while (ctx->queue->pop(tile)) {
        render_tile(tile, ctx, rng);
        int tx = tile.x0 / TILE_SIZE;
        int ty = tile.y0 / TILE_SIZE;
        ctx->progress_map->update(tx, ty, thread_id);
    }
}

// --- Platform Thread Entry Points -------------------------------------------
struct ThreadData { int id; RenderContext* ctx; };

#ifdef _WIN32
DWORD WINAPI win32_worker(LPVOID arg) {
    ThreadData* d = (ThreadData*)arg;
    worker_func(d->id, d->ctx);
    return 0;
}
#else
void* posix_worker(void* arg) {
    ThreadData* d = (ThreadData*)arg;
    worker_func(d->id, d->ctx);
    return NULL;
}
#endif


// =============================================================================
//  SECTION 12: CLI PARSER
// =============================================================================

struct Config {
    int width, height, samples, threads, max_depth;
    std::string output;
    bool help;

    Config() : width(800), height(0), samples(50), threads(0),
               max_depth(50), output("lumina_render.ppm"), help(false) {}
};

void print_usage() {
    std::cout << "\n  PROJECT LUMINA v3.0 -- Advanced Rendering Engine\n\n";
    std::cout << "  Usage: lumina [options]\n\n";
    std::cout << "  Options:\n";
    std::cout << "    --width    <int>   Image width in pixels       (default: 800)\n";
    std::cout << "    --height   <int>   Image height in pixels      (default: auto 16:9)\n";
    std::cout << "    --samples  <int>   Anti-aliasing rays/pixel    (default: 50)\n";
    std::cout << "    --threads  <int>   Worker thread count         (default: all cores)\n";
    std::cout << "    --out      <str>   Output filename             (default: lumina_render.ppm)\n";
    std::cout << "    --help             Show this help\n\n";
    std::cout << "  Examples:\n";
    std::cout << "    lumina --width 1920 --samples 200 --threads 8\n";
    std::cout << "    lumina --width 400 --samples 10 --threads 1     (fast preview)\n\n";
}

Config parse_args(int argc, char* argv[]) {
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--help") == 0)  { cfg.help = true; }
        else if (strcmp(argv[i], "--width")   == 0 && i+1 < argc) cfg.width   = atoi(argv[++i]);
        else if (strcmp(argv[i], "--height")  == 0 && i+1 < argc) cfg.height  = atoi(argv[++i]);
        else if (strcmp(argv[i], "--samples") == 0 && i+1 < argc) cfg.samples = atoi(argv[++i]);
        else if (strcmp(argv[i], "--threads") == 0 && i+1 < argc) cfg.threads = atoi(argv[++i]);
        else if (strcmp(argv[i], "--out")     == 0 && i+1 < argc) cfg.output  = argv[++i];
    }
    return cfg;
}


// =============================================================================
//  SECTION 13: TERMINAL OUTPUT
// =============================================================================

void pad_to(int len, int target) {
    for (int i = len; i < target; ++i) std::cout << " ";
}

void print_banner() {
    std::cout << "\n";
    std::cout << "  +==============================================================+\n";
    std::cout << "  |          PROJECT LUMINA v3.0 -- Rendering Engine              |\n";
    std::cout << "  |   Path Tracer with Live Progress Map & Thread Pool            |\n";
    std::cout << "  |   Unit V: Parallel Organization (SMP Demonstration)           |\n";
    std::cout << "  +==============================================================+\n\n";
}

void print_config(const Config& c, int spheres, int tile_count) {
    std::cout << "  +------------------ RENDER CONFIGURATION --------------------+\n";
    std::ostringstream s;

    s.str(""); s << c.width << " x " << c.height << " (" << c.width*c.height << " pixels)";
    std::cout << "  |  Resolution      : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << c.samples << " rays/pixel";
    std::cout << "  |  Anti-Aliasing    : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << c.max_depth << " bounces";
    std::cout << "  |  Max Ray Depth   : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << spheres << " spheres (procedural)";
    std::cout << "  |  Scene Geometry   : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << tile_count << " tiles (" << TILE_SIZE << "x" << TILE_SIZE << " px)";
    std::cout << "  |  Work Tiles       : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    s.str(""); s << c.threads << " threads";
    std::cout << "  |  Worker Threads   : " << s.str(); pad_to((int)s.str().size(), 39); std::cout << "|\n";

    std::string mode = (c.threads == 1) ? "SEQUENTIAL (Single-Core)" : "PARALLEL (Multi-Core SMP)";
    std::cout << "  |  Execution Mode   : " << mode; pad_to((int)mode.size(), 39); std::cout << "|\n";

    std::cout << "  |  Output           : " << c.output; pad_to((int)c.output.size(), 39); std::cout << "|\n";

    std::cout << "  +------------------------------------------------------------+\n\n";
    std::cout << "  Materials: Lambertian + Metal (fuzz) + Dielectric (glass)\n";
    std::cout << "  Camera:    DOF enabled | aperture=0.1 | focus=10.0\n";
}


// =============================================================================
//  MAIN
// =============================================================================

int main(int argc, char* argv[]) {

    // --- CLI ----------------------------------------------------------------
    Config cfg = parse_args(argc, argv);
    if (cfg.help) { print_usage(); return 0; }

    const double ASPECT = 16.0 / 9.0;
    if (cfg.width  < 1) cfg.width  = 800;
    if (cfg.height < 1) cfg.height = (int)(cfg.width / ASPECT);
    if (cfg.samples < 1) cfg.samples = 50;
    if (cfg.threads < 1) cfg.threads = get_hw_threads();
    if (cfg.threads < 1) cfg.threads = 1;
    if (cfg.threads > 64) cfg.threads = 64; // WaitForMultipleObjects limit

    const int W = cfg.width, H = cfg.height;

    // --- Scene --------------------------------------------------------------
    RNG scene_rng(42);
    std::vector<Sphere> world = random_scene(scene_rng);

    // --- Camera -------------------------------------------------------------
    Camera cam(Vec3(13,2,3), Vec3(0,0,0), Vec3(0,1,0),
               20.0, (double)W/H, 0.1, 10.0);

    // --- Pixel Buffer -------------------------------------------------------
    std::vector<Vec3> pixels(W * H);

    // --- Generate Tiles & Shuffle for Interesting Pattern -------------------
    //  Shuffling the tile order creates a scattered render pattern in the
    //  live map (tiles light up across the whole image, not just top-to-bottom).
    //  This also improves load balancing for scenes with uneven complexity.
    std::vector<Tile> tile_list;
    for (int y = 0; y < H; y += TILE_SIZE)
        for (int x = 0; x < W; x += TILE_SIZE)
            tile_list.push_back({x, y, std::min(x+TILE_SIZE, W), std::min(y+TILE_SIZE, H)});

    RNG shuffle_rng(12345);
    for (int i = (int)tile_list.size() - 1; i > 0; --i) {
        int j = (int)(shuffle_rng.rand01() * (i + 1));
        if (j > i) j = i;
        std::swap(tile_list[i], tile_list[j]);
    }

    int tiles_x = (W + TILE_SIZE - 1) / TILE_SIZE;
    int tiles_y = (H + TILE_SIZE - 1) / TILE_SIZE;

    WorkQueue queue;
    queue.init(tile_list);

    // --- Progress Map -------------------------------------------------------
    ProgressMap pm;
    pm.init(tiles_x, tiles_y);

    // --- Print Config -------------------------------------------------------
    print_banner();
    print_config(cfg, (int)world.size(), (int)tile_list.size());
    pm.print_initial();

    // --- Render Context (shared by all workers) -----------------------------
    RenderContext ctx;
    ctx.image_width  = W;
    ctx.image_height = H;
    ctx.samples      = cfg.samples;
    ctx.max_depth    = cfg.max_depth;
    ctx.camera       = &cam;
    ctx.world        = &world;
    ctx.pixel_buffer = &pixels[0];
    ctx.queue        = &queue;
    ctx.progress_map = &pm;

    // =========================================================================
    //  [UNIT V DEMONSTRATION: SYMMETRIC MULTIPROCESSING & DOMAIN DECOMPOSITION]
    //
    //  TILE-BASED THREAD POOL WITH WORK-STEALING QUEUE:
    //
    //  1. Image divided into 16x16 pixel tiles (TILE_SIZE), shuffled randomly.
    //  2. All tiles pushed to a thread-safe WorkQueue (Mutex-protected index).
    //  3. N worker threads spawned (one per core).
    //  4. Each worker DYNAMICALLY pops tiles, renders them, updates progress.
    //  5. Workers exit when queue is empty. Main thread joins all (barrier).
    //
    //  This achieves:
    //    * ZERO idle cores — dynamic load balancing
    //    * CACHE-FRIENDLY — 16x16 tiles fit in L1 cache
    //    * SCALABLE — works for 1 to 64+ threads
    //    * OBSERVABLE — live map shows which thread rendered each tile
    // =========================================================================

    auto t_start = std::chrono::high_resolution_clock::now();

    int N = cfg.threads;
    std::vector<ThreadData> td(N);

#ifdef _WIN32
    std::vector<HANDLE> handles(N);
    for (int i = 0; i < N; ++i) {
        td[i].id = i;
        td[i].ctx = &ctx;
        handles[i] = CreateThread(NULL, 0, win32_worker, &td[i], 0, NULL);
    }
    WaitForMultipleObjects(N, &handles[0], TRUE, INFINITE);
    for (int i = 0; i < N; ++i) CloseHandle(handles[i]);
#else
    std::vector<pthread_t> pt(N);
    for (int i = 0; i < N; ++i) {
        td[i].id = i;
        td[i].ctx = &ctx;
        pthread_create(&pt[i], NULL, posix_worker, &td[i]);
    }
    for (int i = 0; i < N; ++i) pthread_join(pt[i], NULL);
#endif

    auto t_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = t_end - t_start;

    // Finalize display
    pm.print_completion_map();

#ifdef _WIN32
    if (pm.live_enabled) {
        // Cursor is parked 2 rows below progress bar. Print from there.
    } else {
        std::cout << "\n";
    }
#else
    std::cout << "\n";
#endif

    std::cout << "\n  >> All " << N << " threads completed.\n\n";

    // --- Write PPM ----------------------------------------------------------
    std::cout << "  >> Writing " << cfg.output << "... ";
    std::cout.flush();

    std::ofstream file(cfg.output.c_str());
    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot write to " << cfg.output << "\n";
        return 1;
    }
    file << "P3\n" << W << " " << H << "\n255\n";
    for (int i = 0; i < W * H; ++i) {
        file << (int)(255.999*pixels[i].x) << " "
             << (int)(255.999*pixels[i].y) << " "
             << (int)(255.999*pixels[i].z) << "\n";
    }
    file.close();
    std::cout << "Done.\n\n";

    // --- Performance Report -------------------------------------------------
    long long total_rays = (long long)W * H * cfg.samples;

    std::cout << "  +==============================================================+\n";
    std::cout << "  |                    PERFORMANCE REPORT                         |\n";
    std::cout << "  +==============================================================+\n";

    std::ostringstream s;

    s.str(""); s << N;
    std::cout << "  |  Worker Threads   : " << s.str(); pad_to((int)s.str().size(),39); std::cout << "|\n";

    s.str(""); s << (int)tile_list.size() << " tiles (" << TILE_SIZE << "x" << TILE_SIZE << ")";
    std::cout << "  |  Work Units       : " << s.str(); pad_to((int)s.str().size(),39); std::cout << "|\n";

    s.str(""); s << W*H;
    std::cout << "  |  Total Pixels     : " << s.str(); pad_to((int)s.str().size(),39); std::cout << "|\n";

    s.str(""); s << total_rays;
    std::cout << "  |  Primary Rays     : " << s.str(); pad_to((int)s.str().size(),39); std::cout << "|\n";

    s.str(""); s << (int)world.size();
    std::cout << "  |  Scene Spheres    : " << s.str(); pad_to((int)s.str().size(),39); std::cout << "|\n";

    s.str(""); s << std::fixed; s.precision(4); s << elapsed.count() << " seconds";
    std::cout << "  |  Render Time      : " << s.str(); pad_to((int)s.str().size(),39); std::cout << "|\n";

    double mrps = total_rays / elapsed.count() / 1e6;
    s.str(""); s << std::fixed; s.precision(2); s << mrps << " Mrays/sec";
    std::cout << "  |  Throughput       : " << s.str(); pad_to((int)s.str().size(),39); std::cout << "|\n";

    s.str(""); s << cfg.output;
    std::cout << "  |  Output File      : " << s.str(); pad_to((int)s.str().size(),39); std::cout << "|\n";

    std::cout << "  +--------------------------------------------------------------+\n";

    if (N == 1) {
        std::cout << "  |  MODE: SEQUENTIAL (Baseline)                                |\n";
        std::cout << "  |  Re-run with --threads N to observe parallel speedup.       |\n";
    } else {
        std::cout << "  |  MODE: PARALLEL (Tile-Based SMP Thread Pool)                |\n";
        std::cout << "  |  Compare with --threads 1 to measure speedup ratio.         |\n";
    }
    std::cout << "  +==============================================================+\n\n";
    std::cout << "  >> Render complete. Open " << cfg.output << " to view.\n\n";

    queue.destroy();
    pm.destroy();
    return 0;
}

// =============================================================================
//  COMPILATION INSTRUCTIONS
// =============================================================================
//
//  WINDOWS (MinGW g++):
//    g++ -std=c++11 -O3 -o lumina.exe main.cpp
//    .\lumina.exe
//    .\lumina.exe --width 400 --samples 10 --threads 1   (fast single-core test)
//    .\lumina.exe --width 800 --samples 100 --threads 8   (quality render)
//
//  WINDOWS (MSVC):
//    cl /EHsc /O2 /std:c++14 main.cpp /Fe:lumina.exe
//
//  LINUX / macOS (g++ or clang++):
//    g++ -std=c++11 -O3 -pthread -o lumina main.cpp
//    ./lumina --width 1200 --samples 100 --threads 8
//
// =============================================================================
