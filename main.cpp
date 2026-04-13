// =============================================================================
//  PROJECT LUMINA v4.0 — Gold Standard Ray Tracing Engine
// =============================================================================
//
//  ARCHITECTURE UPGRADES FROM v3.0:
//
//    * BVH ACCELERATION   — Bounding Volume Hierarchy reduces intersection
//                           tests from O(N) to O(log N) per ray.
//    * GAMMA 1/2.2        — Physically correct sRGB gamma curve.
//    * COLORED LIVE MAP   — Each thread renders in a distinct console color.
//    * ALL v3.0 FEATURES  — Tile pool, PBR materials, DOF camera, CLI.
//
//  Dependencies: ZERO external libraries.
//  Compiler:     C++11 or newer.
//  Output:       .ppm (Portable Pixmap).
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

#ifdef _WIN32
    #include <windows.h>
#else
    #include <pthread.h>
    #include <unistd.h>
#endif


// =============================================================================
//  PLATFORM ABSTRACTION
// =============================================================================

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

int get_hw_threads() {
#ifdef _WIN32
    SYSTEM_INFO si; GetSystemInfo(&si); return (int)si.dwNumberOfProcessors;
#else
    int n = (int)sysconf(_SC_NPROCESSORS_ONLN); return n > 0 ? n : 1;
#endif
}

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

// --- Console Color Palette for Thread Visualization -------------------------
//  Each thread gets a unique color so the live map shows which core rendered
//  which tile, using color as the primary visual differentiator.
const WORD THREAD_COLORS[] = {
    10,  // 0: Bright Green
    11,  // 1: Bright Cyan
    12,  // 2: Bright Red
    13,  // 3: Bright Magenta
    14,  // 4: Bright Yellow
    15,  // 5: Bright White
    9,   // 6: Bright Blue
    6,   // 7: Dark Cyan
    10, 11, 12, 13, 14, 15, 9, 6  // Repeat for threads 8-15
};
const int NUM_THREAD_COLORS = 16;
const WORD COLOR_DIM     = 8;  // Dark gray (pending tiles)
const WORD COLOR_DEFAULT = 7;  // White (normal text)
#endif

// =============================================================================
//  CONSTANTS
// =============================================================================

const double PI      = 3.14159265358979323846;
const double INF     = 1e30;
const double EPSILON = 0.001;
const double GAMMA   = 1.0 / 2.2;  // sRGB gamma correction exponent


// =============================================================================
//  SECTION 1: Vec3
// =============================================================================

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x,y+v.y,z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x,y-v.y,z-v.z); }
    Vec3 operator*(double t)      const { return Vec3(x*t,y*t,z*t); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x*v.x,y*v.y,z*v.z); }
    Vec3 operator/(double t)      const { double i=1.0/t; return Vec3(x*i,y*i,z*i); }
    Vec3 operator-()              const { return Vec3(-x,-y,-z); }
    Vec3& operator+=(const Vec3& v) { x+=v.x;y+=v.y;z+=v.z; return *this; }
    Vec3& operator*=(double t)      { x*=t;y*=t;z*=t; return *this; }

    double dot(const Vec3& v)   const { return x*v.x+y*v.y+z*v.z; }
    Vec3   cross(const Vec3& v) const { return Vec3(y*v.z-z*v.y,z*v.x-x*v.z,x*v.y-y*v.x); }
    double length_squared()     const { return x*x+y*y+z*z; }
    double length()             const { return std::sqrt(length_squared()); }
    Vec3   normalized()         const { double l=length(); return l<1e-12?Vec3(0,0,0):*this/l; }
    bool   near_zero()          const { return std::fabs(x)<1e-8&&std::fabs(y)<1e-8&&std::fabs(z)<1e-8; }

    static Vec3 reflect(const Vec3& v, const Vec3& n) { return v-n*2.0*v.dot(n); }
    static Vec3 refract(const Vec3& uv, const Vec3& n, double eta) {
        double ct=std::fmin((-uv).dot(n),1.0);
        Vec3 perp=(uv+n*ct)*eta;
        Vec3 para=n*(-std::sqrt(std::fabs(1.0-perp.length_squared())));
        return perp+para;
    }
    Vec3 clamped() const {
        return Vec3(std::fmax(0.0,std::fmin(1.0,x)),std::fmax(0.0,std::fmin(1.0,y)),std::fmax(0.0,std::fmin(1.0,z)));
    }
};

Vec3 operator*(double t, const Vec3& v) { return v*t; }

// Access Vec3 component by axis index (0=x, 1=y, 2=z).
// Used by AABB slab test and BVH axis sorting.
inline double vec3_axis(const Vec3& v, int a) {
    return a==0 ? v.x : (a==1 ? v.y : v.z);
}


// =============================================================================
//  SECTION 2: Ray
// =============================================================================

struct Ray {
    Vec3 origin, direction;
    Ray() {}
    Ray(const Vec3& o, const Vec3& d) : origin(o), direction(d) {}
    Vec3 at(double t) const { return origin+direction*t; }
};


// =============================================================================
//  SECTION 3: RNG (xorshift64)
// =============================================================================

struct RNG {
    unsigned long long state;
    RNG(unsigned long long s=1ULL) : state(s?s:1ULL) {}
    unsigned long long next() { state^=state<<13; state^=state>>7; state^=state<<17; return state; }
    double rand01() { return (next()&0xFFFFFFFFFFFFFULL)/(double)0x10000000000000ULL; }
    double rr(double lo, double hi) { return lo+(hi-lo)*rand01(); }
};

Vec3 random_in_unit_sphere(RNG& r) { while(1){Vec3 p(r.rr(-1,1),r.rr(-1,1),r.rr(-1,1));if(p.length_squared()<1)return p;} }
Vec3 random_unit_vector(RNG& r)    { return random_in_unit_sphere(r).normalized(); }
Vec3 random_in_unit_disk(RNG& r)   { while(1){Vec3 p(r.rr(-1,1),r.rr(-1,1),0);if(p.length_squared()<1)return p;} }


// =============================================================================
//  SECTION 4: AABB (Axis-Aligned Bounding Box)
// =============================================================================
//  An AABB is the simplest 3D bounding volume: a box whose faces are aligned
//  with the coordinate axes. It's defined by two corner points (mn, mx).
//
//  Ray-AABB intersection uses the "slab method" (Kay & Kajiya, 1986):
//    For each axis, compute the entry and exit t-values of the ray.
//    The ray hits the box iff the intervals overlap across all three axes.
//
//  This test is EXTREMELY fast (~6 multiplies, ~6 compares) compared to
//  a full ray-sphere test (~17 ops). The BVH exploits this by testing the
//  cheap AABB first and skipping entire subtrees when the ray misses.
// =============================================================================

struct AABB {
    Vec3 mn, mx; // Minimum and maximum corners

    AABB() {}
    AABB(const Vec3& a, const Vec3& b) : mn(a), mx(b) {}

    // Slab-based ray-AABB intersection test.
    // Returns true if the ray passes through this box within [tmin, tmax].
    bool hit(const Ray& r, double tmin, double tmax) const {
        for (int a = 0; a < 3; ++a) {
            double invD = 1.0 / vec3_axis(r.direction, a);
            double t0 = (vec3_axis(mn, a) - vec3_axis(r.origin, a)) * invD;
            double t1 = (vec3_axis(mx, a) - vec3_axis(r.origin, a)) * invD;
            if (invD < 0.0) { double tmp = t0; t0 = t1; t1 = tmp; }
            if (t0 > tmin) tmin = t0;
            if (t1 < tmax) tmax = t1;
            if (tmax <= tmin) return false;
        }
        return true;
    }
};

// Union of two AABBs: the smallest box enclosing both.
AABB surrounding_box(const AABB& a, const AABB& b) {
    return AABB(
        Vec3(std::fmin(a.mn.x,b.mn.x), std::fmin(a.mn.y,b.mn.y), std::fmin(a.mn.z,b.mn.z)),
        Vec3(std::fmax(a.mx.x,b.mx.x), std::fmax(a.mx.y,b.mx.y), std::fmax(a.mx.z,b.mx.z))
    );
}


// =============================================================================
//  SECTION 5: MATERIALS (Lambertian, Metal, Dielectric)
// =============================================================================

enum MaterialType { MAT_LAMBERTIAN, MAT_METAL, MAT_DIELECTRIC };

struct Material {
    MaterialType type;
    Vec3 albedo;
    double param; // fuzz (Metal) or IOR (Dielectric)

    Material() : type(MAT_LAMBERTIAN), albedo(Vec3(0.5,0.5,0.5)), param(0) {}
    Material(MaterialType t, const Vec3& a, double p) : type(t), albedo(a), param(p) {}

    static Material lambertian(const Vec3& c) { return Material(MAT_LAMBERTIAN,c,0); }
    static Material metal(const Vec3& c, double f) { return Material(MAT_METAL,c,std::fmin(f,1.0)); }
    static Material glass(double ior) { return Material(MAT_DIELECTRIC,Vec3(1,1,1),ior); }
};

// Schlick's approximation: R(θ) = R0 + (1-R0)(1-cosθ)^5
double schlick(double cosine, double ri) {
    double r0 = (1.0-ri)/(1.0+ri); r0 *= r0;
    return r0 + (1.0-r0)*std::pow(1.0-cosine, 5.0);
}


// =============================================================================
//  SECTION 6: GEOMETRY — HitRecord & Sphere
// =============================================================================

struct HitRecord {
    Vec3 p, normal;
    double t;
    bool front_face;
    Material material;
    void set_face_normal(const Ray& r, const Vec3& on) {
        front_face = r.direction.dot(on)<0;
        normal = front_face ? on : -on;
    }
};

struct Sphere {
    Vec3 center;
    double radius;
    Material material;

    Sphere() : radius(0) {}
    Sphere(const Vec3& c, double r, const Material& m) : center(c), radius(r), material(m) {}

    bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {
        Vec3 oc = r.origin - center;
        double a = r.direction.length_squared();
        double hb = oc.dot(r.direction);
        double c = oc.length_squared() - radius*radius;
        double disc = hb*hb - a*c;
        if (disc < 0) return false;
        double sq = std::sqrt(disc);
        double root = (-hb-sq)/a;
        if (root<t_min||root>t_max) { root=(-hb+sq)/a; if(root<t_min||root>t_max) return false; }
        rec.t = root;
        rec.p = r.at(root);
        rec.set_face_normal(r, (rec.p-center)/radius);
        rec.material = material;
        return true;
    }

    // Compute the AABB for this sphere.
    AABB bounding_box() const {
        Vec3 rv(radius, radius, radius);
        return AABB(center - rv, center + rv);
    }
};


// =============================================================================
//  SECTION 7: BOUNDING VOLUME HIERARCHY (BVH)
// =============================================================================
//  The "Big Brain" data structure upgrade. Instead of testing every ray against
//  all N spheres (O(N) per ray), we organize spheres into a binary tree of
//  bounding boxes. Each internal node's AABB encloses all its children.
//
//  CONSTRUCTION (top-down, recursive):
//    1. Compute the centroid bounding box of all primitives in the range.
//    2. Choose the LONGEST AXIS of that box as the split dimension.
//    3. Sort primitives along that axis by centroid position.
//    4. Split at the midpoint into left and right halves.
//    5. Recurse on each half until only 1 primitive remains (leaf node).
//
//  TRAVERSAL (for each ray):
//    1. Test ray against the root node's AABB.
//    2. If miss → skip entire tree (massive savings for rays aimed at sky).
//    3. If hit → recurse into children. After hitting the left child, tighten
//       t_max to rec.t so the right child is only tested if it's closer.
//
//  COMPLEXITY ANALYSIS:
//    Without BVH: Each ray tests ALL N spheres       → O(N) per ray
//    With BVH:    Each ray tests ~2·log₂(N) AABBs    → O(log N) per ray
//
//    For 478 spheres:
//      O(N)     = 478 tests/ray  ×  18M rays  =  ~8.6 BILLION tests
//      O(log N) = ~18 tests/ray  ×  18M rays  =  ~324 MILLION tests
//      SPEEDUP: ~26× fewer intersection tests!
//
//  NODE LAYOUT:
//    Stored in a flat std::vector<BVHNode> (cache-friendly, no pointer chasing).
//    Leaf nodes store a sphere index. Internal nodes store child indices.
// =============================================================================

struct BVHNode {
    AABB box;
    int left, right;  // Child node indices (-1 for leaves)
    int sphere;       // Sphere index (>= 0 for leaves, -1 for internal)
};

// Global stat tracker for BVH build depth.
int g_bvh_depth = 0;

// Recursive BVH construction.
// indices[start..start+count) are indices into the spheres array.
// Returns the index of the created node in the nodes vector.
int build_bvh(std::vector<BVHNode>& nodes, std::vector<int>& indices,
              int start, int count, const std::vector<Sphere>& spheres, int depth)
{
    if (depth > g_bvh_depth) g_bvh_depth = depth;

    // Reserve our slot in the node array.
    int my_idx = (int)nodes.size();
    nodes.push_back(BVHNode());

    if (count == 1) {
        // --- LEAF NODE: contains a single sphere ---
        int si = indices[start];
        nodes[my_idx].sphere = si;
        nodes[my_idx].left = nodes[my_idx].right = -1;
        nodes[my_idx].box = spheres[si].bounding_box();
        return my_idx;
    }

    // --- INTERNAL NODE: split primitives along longest axis ---

    // Step 1: Compute the centroid bounding box to find the best split axis.
    Vec3 cmin(INF, INF, INF), cmax(-INF, -INF, -INF);
    for (int i = start; i < start + count; ++i) {
        Vec3 c = spheres[indices[i]].center;
        cmin = Vec3(std::fmin(cmin.x,c.x), std::fmin(cmin.y,c.y), std::fmin(cmin.z,c.z));
        cmax = Vec3(std::fmax(cmax.x,c.x), std::fmax(cmax.y,c.y), std::fmax(cmax.z,c.z));
    }
    Vec3 extent = cmax - cmin;

    // Step 2: Choose the longest axis for splitting.
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (vec3_axis(extent, 2) > vec3_axis(extent, axis)) axis = 2;

    // Step 3: Sort indices by sphere centroid along the chosen axis.
    int s = start, e = start + count;
    std::sort(indices.begin() + s, indices.begin() + e,
        [&spheres, axis](int a, int b) {
            return vec3_axis(spheres[a].center, axis) < vec3_axis(spheres[b].center, axis);
        }
    );

    // Step 4: Split at midpoint and recurse.
    int mid = count / 2;
    nodes[my_idx].sphere = -1;
    nodes[my_idx].left  = build_bvh(nodes, indices, start, mid, spheres, depth + 1);
    nodes[my_idx].right = build_bvh(nodes, indices, start + mid, count - mid, spheres, depth + 1);
    nodes[my_idx].box   = surrounding_box(nodes[nodes[my_idx].left].box,
                                           nodes[nodes[my_idx].right].box);
    return my_idx;
}

// Recursive BVH traversal: find the closest sphere hit along the ray.
bool hit_bvh(const std::vector<BVHNode>& nodes, int idx,
             const std::vector<Sphere>& spheres,
             const Ray& r, double t_min, double t_max, HitRecord& rec)
{
    const BVHNode& node = nodes[idx];

    // Test ray against this node's bounding box.
    // If the ray misses the box, skip the ENTIRE subtree.
    if (!node.box.hit(r, t_min, t_max)) return false;

    if (node.sphere >= 0) {
        // LEAF: test against the actual sphere.
        return spheres[node.sphere].hit(r, t_min, t_max, rec);
    }

    // INTERNAL: test both children, keeping the closest hit.
    // Key optimization: after hitting left child, tighten t_max for right child.
    bool hit_left  = hit_bvh(nodes, node.left,  spheres, r, t_min, t_max, rec);
    bool hit_right = hit_bvh(nodes, node.right, spheres, r, t_min,
                             hit_left ? rec.t : t_max, rec);
    return hit_left || hit_right;
}


// =============================================================================
//  SECTION 8: PATH TRACING
// =============================================================================

Vec3 ray_color(const Ray& r, const std::vector<BVHNode>& bvh, int root,
               const std::vector<Sphere>& world, int depth, RNG& rng)
{
    if (depth <= 0) return Vec3(0,0,0);

    HitRecord rec;
    if (!hit_bvh(bvh, root, world, r, EPSILON, INF, rec)) {
        // Sky gradient
        Vec3 ud = r.direction.normalized();
        double t = 0.5*(ud.y+1.0);
        return Vec3(1,1,1)*(1.0-t) + Vec3(0.5,0.7,1.0)*t;
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
        scattered = Ray(rec.p, refl + random_in_unit_sphere(rng)*rec.material.param);
        attenuation = rec.material.albedo;
        if (scattered.direction.dot(rec.normal) <= 0) return Vec3(0,0,0);
        break;
    }
    case MAT_DIELECTRIC: {
        attenuation = Vec3(1,1,1);
        double ri = rec.front_face ? (1.0/rec.material.param) : rec.material.param;
        Vec3 ud = r.direction.normalized();
        double ct = std::fmin((-ud).dot(rec.normal), 1.0);
        double st = std::sqrt(1.0-ct*ct);
        bool tir = (ri*st) > 1.0;
        Vec3 dir = (tir || schlick(ct,ri) > rng.rand01())
            ? Vec3::reflect(ud, rec.normal)
            : Vec3::refract(ud, rec.normal, ri);
        scattered = Ray(rec.p, dir);
        break;
    }
    default: return Vec3(0,0,0);
    }

    return attenuation * ray_color(scattered, bvh, root, world, depth-1, rng);
}


// =============================================================================
//  SECTION 9: CAMERA (Depth-of-Field)
// =============================================================================

struct Camera {
    Vec3 origin, lower_left, horizontal, vertical, u, v, w;
    double lens_radius;

    Camera(Vec3 from, Vec3 at, Vec3 vup, double vfov, double aspect,
           double aperture, double focus_dist)
    {
        double th = vfov*PI/180.0, h = std::tan(th/2.0);
        double vh = 2.0*h, vw = aspect*vh;
        w = (from-at).normalized(); u = vup.cross(w).normalized(); v = w.cross(u);
        origin = from;
        horizontal = u*vw*focus_dist;
        vertical   = v*vh*focus_dist;
        lower_left = origin - horizontal/2.0 - vertical/2.0 - w*focus_dist;
        lens_radius = aperture/2.0;
    }

    Ray get_ray(double s, double t, RNG& rng) const {
        Vec3 rd = random_in_unit_disk(rng)*lens_radius;
        Vec3 off = u*rd.x + v*rd.y;
        return Ray(origin+off, lower_left+horizontal*s+vertical*t-origin-off);
    }
};


// =============================================================================
//  SECTION 10: RANDOM SCENE GENERATOR
// =============================================================================

std::vector<Sphere> random_scene(RNG& rng) {
    std::vector<Sphere> world;
    world.push_back(Sphere(Vec3(0,-1000,0),1000,Material::lambertian(Vec3(0.5,0.5,0.5))));

    for (int a=-11; a<11; ++a) {
        for (int b=-11; b<11; ++b) {
            double m = rng.rand01();
            Vec3 c(a+0.9*rng.rand01(), 0.2, b+0.9*rng.rand01());
            if ((c-Vec3(4,.2,0)).length()<0.9) continue;
            if ((c-Vec3(0,.2,0)).length()<0.9) continue;
            if ((c-Vec3(-4,.2,0)).length()<0.9) continue;

            if (m<0.65)
                world.push_back(Sphere(c,0.2,Material::lambertian(
                    Vec3(rng.rand01()*rng.rand01(),rng.rand01()*rng.rand01(),rng.rand01()*rng.rand01()))));
            else if (m<0.85)
                world.push_back(Sphere(c,0.2,Material::metal(
                    Vec3(rng.rr(.5,1),rng.rr(.5,1),rng.rr(.5,1)),rng.rr(0,.5))));
            else
                world.push_back(Sphere(c,0.2,Material::glass(1.5)));
        }
    }
    world.push_back(Sphere(Vec3(0,1,0),1.0,Material::glass(1.5)));
    world.push_back(Sphere(Vec3(-4,1,0),1.0,Material::lambertian(Vec3(0.4,0.2,0.1))));
    world.push_back(Sphere(Vec3(4,1,0),1.0,Material::metal(Vec3(0.7,0.6,0.5),0.0)));
    return world;
}


// =============================================================================
//  SECTION 11: TILE-BASED WORK QUEUE
// =============================================================================

const int TILE_SIZE = 16;

struct Tile { int x0,y0,x1,y1; };

struct WorkQueue {
    std::vector<Tile> tiles;
    int next_idx;
    Mutex mutex;
    void init(const std::vector<Tile>& t) { tiles=t; next_idx=0; mutex.init(); }
    bool pop(Tile& out) {
        mutex.lock();
        bool got=(next_idx<(int)tiles.size());
        if(got) out=tiles[next_idx++];
        mutex.unlock();
        return got;
    }
    void destroy() { mutex.destroy(); }
};


// =============================================================================
//  SECTION 12: LIVE PROGRESS MAP (with ANSI / Win32 Colors)
// =============================================================================
//
//  The progress map is a terminal grid where each character represents a 16x16
//  pixel tile. As worker threads complete tiles, characters light up in the
//  thread's assigned COLOR, creating a vivid visualization of parallel work
//  distribution across CPU cores.
//
//  Legend:
//    '.'         = Pending (dim gray)
//    '0'-'9'     = Completed by thread 0-9 (colored by thread ID)
//    'A'-'Z'     = Completed by thread 10-35 (colored by thread ID)
//
// =============================================================================

struct ProgressMap {
    int tiles_x, tiles_y, total;
    std::vector<char> grid;
    AtomicInt completed;
    Mutex print_mutex;
    int grid_row, grid_col, progress_row;
    bool live;

    void init(int tx, int ty) {
        tiles_x=tx; tiles_y=ty; total=tx*ty;
        grid.assign(tx*ty, '.');
        completed.set(0);
        print_mutex.init();
        grid_col = 3;
        live = (tx<=80 && ty<=40);
    }

    void print_initial() {
        if (!live) { std::cout << "  (Grid too large for live map. Progress bar only.)\n\n"; return; }

        std::cout << "\n  Live Render Map (" << tiles_x << " x " << tiles_y
                  << " tiles, color = thread ID)\n";
        std::cout << "  +";
        for(int i=0;i<tiles_x;++i) std::cout<<"-";
        std::cout << "+\n";
        std::cout.flush();

#ifdef _WIN32
        grid_row = get_cursor_row();
        HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
#endif
        for (int y=0; y<tiles_y; ++y) {
            std::cout << "  |";
#ifdef _WIN32
            SetConsoleTextAttribute(h, COLOR_DIM);
#endif
            for(int x=0;x<tiles_x;++x) std::cout<<".";
#ifdef _WIN32
            SetConsoleTextAttribute(h, COLOR_DEFAULT);
#endif
            std::cout << "|\n";
        }

        std::cout << "  +";
        for(int i=0;i<tiles_x;++i) std::cout<<"-";
        std::cout << "+\n";
        std::cout.flush();

#ifdef _WIN32
        progress_row = get_cursor_row();
#endif
        std::cout << "  Progress: [";
        for(int i=0;i<50;++i) std::cout<<"-";
        std::cout << "] 0% (0/" << total << " tiles)   \n\n";
        std::cout.flush();
    }

    void update(int tx, int ty, int thread_id) {
        char ch = thread_id<10 ? ('0'+thread_id) : ('A'+thread_id-10);
        grid[ty*tiles_x+tx] = ch;
        int done = completed.increment();
        int pct = (int)(100.0*done/total);

        print_mutex.lock();

#ifdef _WIN32
        if (live) {
            HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);

            // Paint the tile character in the thread's color.
            COORD tp = {(SHORT)(grid_col+tx), (SHORT)(grid_row+ty)};
            SetConsoleCursorPosition(h, tp);
            SetConsoleTextAttribute(h, THREAD_COLORS[thread_id % NUM_THREAD_COLORS]);
            putchar(ch);
            SetConsoleTextAttribute(h, COLOR_DEFAULT);

            // Update progress bar.
            COORD pp = {0, (SHORT)progress_row};
            SetConsoleCursorPosition(h, pp);
            int bw=50, filled=bw*done/total;
            printf("  Progress: [");
            for(int i=0;i<bw;++i) putchar(i<filled?'#':'-');
            printf("] %d%% (%d/%d tiles)   ", pct, done, total);

            COORD pk = {0, (SHORT)(progress_row+2)};
            SetConsoleCursorPosition(h, pk);
            fflush(stdout);
        } else {
            int bw=50, filled=bw*done/total;
            printf("\r  Progress: [");
            for(int i=0;i<bw;++i) putchar(i<filled?'#':'-');
            printf("] %d%% (%d/%d tiles)   ", pct, done, total);
            fflush(stdout);
        }
#else
        int bw=50, filled=bw*done/total;
        printf("\r  Progress: [");
        for(int i=0;i<bw;++i) putchar(i<filled?'#':'-');
        printf("] %d%% (%d/%d tiles)   ", pct, done, total);
        fflush(stdout);
#endif
        print_mutex.unlock();
    }

    void print_completion_map() {
#ifndef _WIN32
        std::cout << "\n\n  Completion Map (char = thread ID):\n";
        std::cout << "  +";
        for(int i=0;i<tiles_x;++i) std::cout<<"-";
        std::cout<<"+\n";
        for(int y=0;y<tiles_y;++y) {
            std::cout<<"  |";
            for(int x=0;x<tiles_x;++x) std::cout<<grid[y*tiles_x+x];
            std::cout<<"|\n";
        }
        std::cout<<"  +";
        for(int i=0;i<tiles_x;++i) std::cout<<"-";
        std::cout<<"+\n";
#endif
    }

    void destroy() { print_mutex.destroy(); }
};


// =============================================================================
//  SECTION 13: RENDER WORKER
// =============================================================================

struct RenderContext {
    int W, H, samples, max_depth;
    const Camera* cam;
    const std::vector<Sphere>* world;
    const std::vector<BVHNode>* bvh;
    int bvh_root;
    Vec3* pixels;
    WorkQueue* queue;
    ProgressMap* pm;
};

void render_tile(const Tile& tile, RenderContext* ctx, RNG& rng) {
    for (int j=tile.y0; j<tile.y1; ++j) {
        for (int i=tile.x0; i<tile.x1; ++i) {
            Vec3 col(0,0,0);
            for (int s=0; s<ctx->samples; ++s) {
                double u = (i+rng.rand01())/(ctx->W-1);
                double v = (j+rng.rand01())/(ctx->H-1);
                col += ray_color(ctx->cam->get_ray(u,v,rng),
                                 *(ctx->bvh), ctx->bvh_root,
                                 *(ctx->world), ctx->max_depth, rng);
            }
            double sc = 1.0/ctx->samples;
            col *= sc;
            // Gamma correction: 1/2.2 (physically correct sRGB).
            col = Vec3(std::pow(col.x, GAMMA),
                       std::pow(col.y, GAMMA),
                       std::pow(col.z, GAMMA)).clamped();
            int row = ctx->H-1-j;
            ctx->pixels[row*ctx->W+i] = col;
        }
    }
}

void worker_func(int tid, RenderContext* ctx) {
    RNG rng((unsigned long long)(tid+1)*6364136223846793005ULL+1442695040888963407ULL);
    Tile tile;
    while (ctx->queue->pop(tile)) {
        render_tile(tile, ctx, rng);
        ctx->pm->update(tile.x0/TILE_SIZE, tile.y0/TILE_SIZE, tid);
    }
}

struct ThreadData { int id; RenderContext* ctx; };

#ifdef _WIN32
DWORD WINAPI win32_worker(LPVOID arg) {
    ThreadData* d=(ThreadData*)arg; worker_func(d->id, d->ctx); return 0;
}
#else
void* posix_worker(void* arg) {
    ThreadData* d=(ThreadData*)arg; worker_func(d->id, d->ctx); return NULL;
}
#endif


// =============================================================================
//  SECTION 14: CLI
// =============================================================================

struct Config {
    int width, height, samples, threads, max_depth;
    std::string output;
    bool help;
    Config() : width(800),height(0),samples(50),threads(0),max_depth(50),
               output("lumina_render.ppm"),help(false) {}
};

void print_usage() {
    std::cout << "\n  PROJECT LUMINA v4.0 -- Gold Standard Rendering Engine\n\n"
              << "  Usage: lumina [options]\n\n"
              << "    --width    <int>   Image width            (default: 800)\n"
              << "    --height   <int>   Image height           (default: auto 16:9)\n"
              << "    --samples  <int>   Rays per pixel         (default: 50)\n"
              << "    --threads  <int>   Worker threads          (default: all cores)\n"
              << "    --out      <str>   Output filename         (default: lumina_render.ppm)\n"
              << "    --help             Show this help\n\n";
}

Config parse_args(int argc, char* argv[]) {
    Config c;
    for (int i=1; i<argc; ++i) {
        if (!strcmp(argv[i],"--help"))    c.help=true;
        else if (!strcmp(argv[i],"--width")  &&i+1<argc) c.width=atoi(argv[++i]);
        else if (!strcmp(argv[i],"--height") &&i+1<argc) c.height=atoi(argv[++i]);
        else if (!strcmp(argv[i],"--samples")&&i+1<argc) c.samples=atoi(argv[++i]);
        else if (!strcmp(argv[i],"--threads")&&i+1<argc) c.threads=atoi(argv[++i]);
        else if (!strcmp(argv[i],"--out")    &&i+1<argc) c.output=argv[++i];
    }
    return c;
}


// =============================================================================
//  SECTION 15: TERMINAL OUTPUT
// =============================================================================

void pad_to(int len, int tgt) { for(int i=len;i<tgt;++i) std::cout<<" "; }

void print_banner() {
    std::cout << "\n"
    "  +==============================================================+\n"
    "  |          PROJECT LUMINA v4.0 -- Gold Standard Engine          |\n"
    "  |   BVH-Accelerated Path Tracer with Colored Live Map          |\n"
    "  |   Unit V: Parallel Organization (SMP Demonstration)          |\n"
    "  +==============================================================+\n\n";
}

void print_config(const Config& c, int spheres, int tiles, int bvh_nodes, int bvh_depth) {
    std::cout << "  +------------------ RENDER CONFIGURATION --------------------+\n";
    std::ostringstream s;

    s.str(""); s<<c.width<<" x "<<c.height<<" ("<<c.width*c.height<<" px)";
    std::cout<<"  |  Resolution      : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<c.samples<<" rays/pixel";
    std::cout<<"  |  Anti-Aliasing    : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<c.max_depth<<" bounces";
    std::cout<<"  |  Max Ray Depth   : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<spheres<<" spheres";
    std::cout<<"  |  Scene Geometry   : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<bvh_nodes<<" nodes, depth "<<bvh_depth<<" -> O(log n)";
    std::cout<<"  |  BVH Tree        : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<tiles<<" tiles ("<<TILE_SIZE<<"x"<<TILE_SIZE<<" px)";
    std::cout<<"  |  Work Tiles       : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<c.threads<<" threads";
    std::cout<<"  |  Worker Threads   : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    std::string mode = c.threads==1 ? "SEQUENTIAL" : "PARALLEL (SMP)";
    std::cout<<"  |  Execution Mode   : "<<mode; pad_to((int)mode.size(),39); std::cout<<"|\n";

    s.str(""); s<<"gamma 1/2.2 (sRGB)";
    std::cout<<"  |  Gamma Correction : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    std::cout << "  +------------------------------------------------------------+\n\n";
}


// =============================================================================
//  MAIN
// =============================================================================

int main(int argc, char* argv[]) {
    Config cfg = parse_args(argc, argv);
    if (cfg.help) { print_usage(); return 0; }

    const double ASPECT = 16.0/9.0;
    if (cfg.width<1)   cfg.width=800;
    if (cfg.height<1)  cfg.height=(int)(cfg.width/ASPECT);
    if (cfg.samples<1) cfg.samples=50;
    if (cfg.threads<1) cfg.threads=get_hw_threads();
    if (cfg.threads<1) cfg.threads=1;
    if (cfg.threads>64) cfg.threads=64;

    const int W=cfg.width, H=cfg.height;

    // --- Scene --------------------------------------------------------------
    RNG scene_rng(42);
    std::vector<Sphere> world = random_scene(scene_rng);

    // --- BVH Construction ---------------------------------------------------
    //  Build the acceleration structure BEFORE rendering starts.
    //  This is a one-time O(N log N) cost that saves O(N) per ray during render.
    std::vector<BVHNode> bvh_nodes;
    bvh_nodes.reserve(2 * world.size() + 2);
    std::vector<int> bvh_indices(world.size());
    for (int i = 0; i < (int)world.size(); ++i) bvh_indices[i] = i;
    g_bvh_depth = 0;

    auto bvh_start = std::chrono::high_resolution_clock::now();
    int bvh_root = build_bvh(bvh_nodes, bvh_indices, 0, (int)world.size(), world, 0);
    auto bvh_end = std::chrono::high_resolution_clock::now();
    double bvh_ms = std::chrono::duration<double,std::milli>(bvh_end-bvh_start).count();

    // --- Camera -------------------------------------------------------------
    Camera cam(Vec3(13,2,3), Vec3(0,0,0), Vec3(0,1,0), 20.0,
               (double)W/H, 0.1, 10.0);

    // --- Pixel Buffer -------------------------------------------------------
    std::vector<Vec3> pixels(W*H);

    // --- Tile Generation (shuffled for scattered render pattern) -------------
    std::vector<Tile> tile_list;
    for (int y=0; y<H; y+=TILE_SIZE)
        for (int x=0; x<W; x+=TILE_SIZE)
            tile_list.push_back({x,y,std::min(x+TILE_SIZE,W),std::min(y+TILE_SIZE,H)});

    RNG shuf_rng(12345);
    for (int i=(int)tile_list.size()-1; i>0; --i) {
        int j=(int)(shuf_rng.rand01()*(i+1)); if(j>i)j=i;
        std::swap(tile_list[i], tile_list[j]);
    }

    int tiles_x=(W+TILE_SIZE-1)/TILE_SIZE, tiles_y=(H+TILE_SIZE-1)/TILE_SIZE;
    WorkQueue queue;
    queue.init(tile_list);

    ProgressMap pm;
    pm.init(tiles_x, tiles_y);

    // --- Print Config -------------------------------------------------------
    print_banner();
    print_config(cfg, (int)world.size(), (int)tile_list.size(),
                 (int)bvh_nodes.size(), g_bvh_depth);

    std::cout << "  BVH built in " << std::fixed;
    std::cout.precision(2);
    std::cout << bvh_ms << " ms (" << bvh_nodes.size() << " nodes, depth "
              << g_bvh_depth << ")\n";
    std::cout << "  Materials: Lambertian + Metal + Dielectric (Schlick)\n";
    std::cout << "  Camera: DOF (aperture=0.1, focus=10)\n";

    pm.print_initial();

    // --- Render Context -----------------------------------------------------
    RenderContext ctx;
    ctx.W=W; ctx.H=H; ctx.samples=cfg.samples; ctx.max_depth=cfg.max_depth;
    ctx.cam=&cam; ctx.world=&world; ctx.bvh=&bvh_nodes; ctx.bvh_root=bvh_root;
    ctx.pixels=&pixels[0]; ctx.queue=&queue; ctx.pm=&pm;

    // =========================================================================
    //  [UNIT V: SYMMETRIC MULTIPROCESSING & DOMAIN DECOMPOSITION]
    //
    //  TILE-BASED THREAD POOL WITH BVH-ACCELERATED INTERSECTION:
    //
    //  1. Scene organized into a BVH tree → O(log N) per-ray intersection.
    //  2. Image divided into 16×16 tiles, shuffled, pushed to work queue.
    //  3. N worker threads dynamically steal tiles from the queue.
    //  4. Each thread renders its tile using the shared (read-only) BVH.
    //  5. Atomic progress counter + colored live map show real-time status.
    //  6. Main thread joins all workers (synchronization barrier).
    //
    //  COMBINED SPEEDUP:
    //    Threading:  ~Nx   (N cores → near-linear scaling)
    //    BVH:        ~26x  (O(log N) vs O(N) intersection)
    //    Total:      ~26N× compared to single-core brute-force
    // =========================================================================

    auto t0 = std::chrono::high_resolution_clock::now();

    int N = cfg.threads;
    std::vector<ThreadData> td(N);

#ifdef _WIN32
    std::vector<HANDLE> handles(N);
    for (int i=0;i<N;++i) { td[i]={i,&ctx}; handles[i]=CreateThread(NULL,0,win32_worker,&td[i],0,NULL); }
    WaitForMultipleObjects(N, &handles[0], TRUE, INFINITE);
    for (int i=0;i<N;++i) CloseHandle(handles[i]);
#else
    std::vector<pthread_t> pt(N);
    for (int i=0;i<N;++i) { td[i]={i,&ctx}; pthread_create(&pt[i],NULL,posix_worker,&td[i]); }
    for (int i=0;i<N;++i) pthread_join(pt[i], NULL);
#endif

    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = t1-t0;

    pm.print_completion_map();

#ifdef _WIN32
    if (!pm.live) std::cout<<"\n";
#else
    std::cout<<"\n";
#endif
    std::cout << "\n  >> All " << N << " threads completed.\n\n";

    // --- Write PPM ----------------------------------------------------------
    std::cout << "  >> Writing " << cfg.output << "... ";
    std::cout.flush();
    std::ofstream file(cfg.output.c_str());
    if (!file.is_open()) { std::cerr<<"ERROR\n"; return 1; }
    file << "P3\n" << W << " " << H << "\n255\n";
    for (int i=0; i<W*H; ++i)
        file << (int)(255.999*pixels[i].x) << " "
             << (int)(255.999*pixels[i].y) << " "
             << (int)(255.999*pixels[i].z) << "\n";
    file.close();
    std::cout << "Done.\n\n";

    // --- Performance Report -------------------------------------------------
    long long total_rays = (long long)W*H*cfg.samples;
    std::cout << "  +==============================================================+\n";
    std::cout << "  |                    PERFORMANCE REPORT                         |\n";
    std::cout << "  +==============================================================+\n";
    std::ostringstream s;

    s.str(""); s<<N;
    std::cout<<"  |  Worker Threads   : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<(int)tile_list.size()<<" tiles ("<<TILE_SIZE<<"x"<<TILE_SIZE<<")";
    std::cout<<"  |  Work Units       : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<W*H;
    std::cout<<"  |  Total Pixels     : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<total_rays;
    std::cout<<"  |  Primary Rays     : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<(int)world.size()<<" spheres";
    std::cout<<"  |  Scene Objects    : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<(int)bvh_nodes.size()<<" nodes, depth "<<g_bvh_depth;
    std::cout<<"  |  BVH Structure    : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<std::fixed; s.precision(4); s<<elapsed.count()<<" seconds";
    std::cout<<"  |  Render Time      : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    double mrps = total_rays/elapsed.count()/1e6;
    s.str(""); s<<std::fixed; s.precision(2); s<<mrps<<" Mrays/sec";
    std::cout<<"  |  Throughput       : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<"gamma 1/2.2 (sRGB)";
    std::cout<<"  |  Gamma            : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    s.str(""); s<<cfg.output;
    std::cout<<"  |  Output File      : "<<s.str(); pad_to((int)s.str().size(),39); std::cout<<"|\n";

    std::cout << "  +--------------------------------------------------------------+\n";
    if (N==1) {
        std::cout << "  |  MODE: SEQUENTIAL (Baseline)                                |\n";
        std::cout << "  |  Re-run with --threads N to observe parallel speedup.       |\n";
    } else {
        std::cout << "  |  MODE: PARALLEL (BVH + Tile-Based SMP Thread Pool)          |\n";
        std::cout << "  |  Compare with --threads 1 to measure speedup ratio.         |\n";
    }
    std::cout << "  +==============================================================+\n\n";
    std::cout << "  >> Render complete. Open " << cfg.output << " to view.\n\n";

    queue.destroy(); pm.destroy();
    return 0;
}

// =============================================================================
//  COMPILATION
// =============================================================================
//
//  WINDOWS (MinGW):
//    g++ -std=c++11 -O3 -march=native -o lumina.exe main.cpp
//
//  WINDOWS (MSVC):
//    cl /EHsc /O2 /std:c++14 main.cpp /Fe:lumina.exe
//
//  LINUX / macOS:
//    g++ -std=c++11 -O3 -march=native -pthread -o lumina main.cpp
//
// =============================================================================
