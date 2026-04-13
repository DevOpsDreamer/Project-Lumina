// =============================================================================
//  PROJECT LUMINA — Real-Time Ray Tracing Engine
// =============================================================================
//  Author  : Senior C++ Graphics & Systems Engineer
//  Purpose : Academic demonstration of Symmetric Multiprocessing (SMP) and
//            Domain Decomposition for Unit V: Parallel Organization.
//  Output  : Renders a photorealistic scene to a .ppm image file.
//  Deps    : ZERO external libraries. Standard C++ + OS threading API.
//  Compile : See bottom of file for exact commands (Windows & Linux/Mac).
// =============================================================================

// --- Standard Library Includes (ZERO external dependencies) -----------------
#include <iostream>   // Console I/O
#include <fstream>    // File output (.ppm)
#include <cmath>      // sqrt, pow, fabs, fmin, fmax, tan
#include <vector>     // Dynamic arrays for pixel buffer
#include <chrono>     // High-resolution timing
#include <string>     // String formatting
#include <sstream>    // String stream for formatted output
#include <cstdint>    // Fixed-width integer types
#include <cstdlib>    // rand, srand
#include <algorithm>  // std::min, std::max

// --- Platform-Specific Threading --------------------------------------------
//  On Windows (MinGW/MSVC): use the native Win32 threading API.
//  On Linux/macOS: use POSIX threads via <pthread.h>.
//  This avoids the common MinGW issue where std::thread is unavailable
//  when compiled with the "win32" threading model.
// ----------------------------------------------------------------------------
#ifdef _WIN32
    #include <windows.h>  // CreateThread, WaitForMultipleObjects, HANDLE
#else
    #include <pthread.h>  // pthread_create, pthread_join
#endif


// =============================================================================
//  SECTION 1: MATHEMATICAL FOUNDATIONS — Vec3 Struct
// =============================================================================
//  A general-purpose 3D vector used for both spatial coordinates (points,
//  directions) and color representation (R, G, B). This is the atomic unit
//  of ALL ray tracing math.
// =============================================================================

struct Vec3 {
    double x, y, z;

    // --- Constructors -------------------------------------------------------
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    // --- Operator Overloads (Vector Arithmetic) -----------------------------
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t)       const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
    Vec3 operator/(double t)       const { return Vec3(x / t, y / t, z / t); }

    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator*=(double t)      { x *= t;   y *= t;   z *= t;   return *this; }

    Vec3 operator-() const { return Vec3(-x, -y, -z); }

    // --- Core Vector Operations ---------------------------------------------

    // Dot Product: Measures alignment between two vectors.
    //   > 0 : same general direction
    //   = 0 : perpendicular
    //   < 0 : opposing directions
    // Used extensively in lighting calculations (Lambert's cosine law).
    double dot(const Vec3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // Cross Product: Returns a vector perpendicular to both input vectors.
    // Follows the right-hand rule. Essential for computing surface normals.
    Vec3 cross(const Vec3& v) const {
        return Vec3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    // Squared length — avoids expensive sqrt when only comparisons are needed.
    double length_squared() const { return x * x + y * y + z * z; }

    // Euclidean length of the vector.
    double length() const { return std::sqrt(length_squared()); }

    // Unit vector (direction only, magnitude = 1). Critical for ray directions
    // and surface normals to ensure consistent lighting math.
    Vec3 normalized() const {
        double len = length();
        if (len < 1e-12) return Vec3(0, 0, 0); // Guard against division by zero
        return *this / len;
    }

    // Reflect vector V about normal N. Used for metallic/mirror surfaces.
    //   Formula: V - 2 * dot(V, N) * N
    static Vec3 reflect(const Vec3& v, const Vec3& n) {
        return v - n * 2.0 * v.dot(n);
    }

    // Clamp each component to [0, 1] — prevents color overflow/underflow.
    Vec3 clamped() const {
        return Vec3(
            std::fmax(0.0, std::fmin(1.0, x)),
            std::fmax(0.0, std::fmin(1.0, y)),
            std::fmax(0.0, std::fmin(1.0, z))
        );
    }
};

// Free-function overload so we can write: scalar * Vec3
Vec3 operator*(double t, const Vec3& v) { return v * t; }


// =============================================================================
//  SECTION 2: RAY STRUCT
// =============================================================================
//  A ray is the fundamental query object in ray tracing. It represents a
//  half-line defined by an origin point and a direction vector.
//
//  Parametric form:  P(t) = Origin + t * Direction,  where t >= 0
//
//  For each pixel, we cast one or more rays from the camera into the scene
//  and determine what they intersect.
// =============================================================================

struct Ray {
    Vec3 origin;
    Vec3 direction;

    Ray() {}
    Ray(const Vec3& origin, const Vec3& direction)
        : origin(origin), direction(direction) {}

    // Returns the point along the ray at parameter t.
    Vec3 at(double t) const { return origin + direction * t; }
};


// =============================================================================
//  SECTION 3: MATERIAL SYSTEM
// =============================================================================
//  Each surface in the scene has a material that controls how light interacts
//  with it. We support two material types:
//    - DIFFUSE  (Lambertian): Scatters light uniformly in all directions.
//                             Think: matte rubber, paper, chalk.
//    - METAL    (Reflective): Reflects rays like a mirror, with optional fuzz.
//                             Think: polished steel, chrome, brushed aluminum.
// =============================================================================

enum MaterialType {
    DIFFUSE,
    METAL
};

struct Material {
    MaterialType type;
    Vec3 albedo;     // Base color (reflectance per RGB channel)
    double fuzz;     // Metal roughness: 0.0 = perfect mirror, 1.0 = very rough

    Material() : type(DIFFUSE), albedo(Vec3(0.5, 0.5, 0.5)), fuzz(0.0) {}
    Material(MaterialType type, const Vec3& albedo, double fuzz = 0.0)
        : type(type), albedo(albedo), fuzz(std::fmin(fuzz, 1.0)) {}
};


// =============================================================================
//  SECTION 4: HIT RECORD & SPHERE GEOMETRY
// =============================================================================
//  When a ray intersects a surface, we record:
//    - The intersection point (p)
//    - The surface normal at that point (normal)
//    - The parameter t at which the intersection occurred
//    - Whether the normal faces outward (front_face)
//    - A pointer to the material of the hit surface
//
//  The Sphere is our only geometric primitive. Its intersection test is based
//  on solving the quadratic equation derived from substituting the ray
//  parametric form into the sphere implicit equation:
//
//    |P - C|^2 = r^2
//    |O + tD - C|^2 = r^2
//
//  Expanding and collecting terms in t gives a standard quadratic at^2+bt+c=0.
// =============================================================================

struct HitRecord {
    Vec3 p;             // Intersection point
    Vec3 normal;        // Surface normal at intersection
    double t;           // Ray parameter at intersection
    bool front_face;    // True if ray hit the outside of the surface
    Material material;  // Material of the intersected surface

    // Ensures the normal always points against the incident ray.
    // This is critical for correct shading on both sides of a surface.
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
    Sphere(const Vec3& center, double radius, const Material& material)
        : center(center), radius(radius), material(material) {}

    // --- Ray-Sphere Intersection Test ---------------------------------------
    //  Solves:  |O + tD - C|^2 = r^2
    //
    //  Let oc = O - C (vector from sphere center to ray origin)
    //  Then:
    //    a = D . D            (always positive for non-degenerate rays)
    //    b = 2 * (oc . D)     (we use half_b optimization below)
    //    c = oc . oc - r^2
    //
    //  Discriminant = b^2 - 4ac
    //    < 0 : no intersection (ray misses sphere)
    //    = 0 : tangent (ray grazes sphere)
    //    > 0 : two intersections (ray passes through sphere)
    //
    //  We use the "half_b" optimization to reduce arithmetic:
    //    half_b = oc . D
    //    discriminant = half_b^2 - a * c
    //    t = (-half_b +/- sqrt(discriminant)) / a
    // -----------------------------------------------------------------------
    bool hit(const Ray& r, double t_min, double t_max, HitRecord& rec) const {
        Vec3 oc = r.origin - center;
        double a = r.direction.length_squared();
        double half_b = oc.dot(r.direction);
        double c = oc.length_squared() - radius * radius;

        double discriminant = half_b * half_b - a * c;
        if (discriminant < 0) return false;

        double sqrtd = std::sqrt(discriminant);

        // Find the nearest root in the acceptable range [t_min, t_max].
        double root = (-half_b - sqrtd) / a;
        if (root < t_min || root > t_max) {
            root = (-half_b + sqrtd) / a;
            if (root < t_min || root > t_max)
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        Vec3 outward_normal = (rec.p - center) / radius;
        rec.set_face_normal(r, outward_normal);
        rec.material = material;
        return true;
    }
};


// =============================================================================
//  SECTION 5: SCENE DEFINITION & RAY COLOR COMPUTATION
// =============================================================================
//  The scene is a collection of spheres. For each ray, we test intersection
//  against every sphere and keep the closest hit. The color of a pixel is
//  determined by recursively bouncing rays off surfaces:
//
//    - DIFFUSE surfaces: scatter the ray in a random hemisphere direction
//      above the hit point (Lambertian reflection).
//
//    - METAL surfaces: reflect the ray about the surface normal, with
//      optional random perturbation (fuzz).
//
//    - If no surface is hit, the ray samples the sky gradient (background).
//
//  The recursion depth limit prevents infinite bouncing in enclosed scenes.
// =============================================================================

// --- Per-Thread Random Number Generation ------------------------------------
//  Each thread gets a unique seed to avoid correlation between samples.
//  We use a simple but effective xorshift64 PRNG — fast and portable,
//  no dependency on <random> or thread_local (which can be problematic
//  on some MinGW builds).

struct RNG {
    unsigned long long state;

    RNG(unsigned long long seed = 12345ULL) : state(seed) {
        if (state == 0) state = 1;
    }

    // xorshift64 — period 2^64 - 1, excellent statistical properties.
    unsigned long long next() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    }

    // Random double in [0, 1)
    double random_double() {
        return (next() & 0xFFFFFFFFFFFFFULL) / (double)(0x10000000000000ULL);
    }

    // Random double in [min, max)
    double random_double_range(double min, double max) {
        return min + (max - min) * random_double();
    }
};

// Generate a random point inside the unit sphere using rejection sampling.
// Used for Lambertian (diffuse) scattering.
Vec3 random_in_unit_sphere(RNG& rng) {
    while (true) {
        Vec3 p(rng.random_double_range(-1, 1),
               rng.random_double_range(-1, 1),
               rng.random_double_range(-1, 1));
        if (p.length_squared() < 1.0) return p;
    }
}

// Lambertian scatter direction: normal + random unit vector.
Vec3 random_unit_vector(RNG& rng) {
    return random_in_unit_sphere(rng).normalized();
}

// --- Scene intersection: test all spheres, keep closest hit -----------------
bool hit_scene(const std::vector<Sphere>& world, const Ray& r,
               double t_min, double t_max, HitRecord& rec)
{
    HitRecord temp_rec;
    bool hit_anything = false;
    double closest_so_far = t_max;

    for (size_t i = 0; i < world.size(); ++i) {
        if (world[i].hit(r, t_min, closest_so_far, temp_rec)) {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }
    return hit_anything;
}

// --- Recursive ray color computation ----------------------------------------
//  This is where the core rendering equation is evaluated:
//
//    Color = Material.albedo * ray_color(scattered_ray, depth - 1)
//
//  The recursion terminates when:
//    - depth reaches 0 (return black — energy fully absorbed), OR
//    - the ray escapes to the sky (return sky gradient color)
//
//  For DIFFUSE materials:
//    - Scatter direction = hit_point_normal + random_unit_vector
//    - Attenuate by albedo (surface absorbs some wavelengths)
//
//  For METAL materials:
//    - Reflect the ray about the surface normal
//    - Add fuzz (random perturbation proportional to roughness)
//    - If the reflected ray scatters below the surface, absorb it (return black)
// ---------------------------------------------------------------------------
Vec3 ray_color(const Ray& r, const std::vector<Sphere>& world, int depth, RNG& rng) {
    // Recursion depth limit — prevents stack overflow and simulates energy loss.
    if (depth <= 0) return Vec3(0, 0, 0);

    HitRecord rec;

    // t_min = 0.001 to avoid "shadow acne" (self-intersection at t ~ 0)
    if (hit_scene(world, r, 0.001, 1e30, rec)) {

        if (rec.material.type == DIFFUSE) {
            // --- Lambertian Diffuse Reflection ---
            // New ray direction = surface normal + random unit vector
            // This naturally produces a cosine-weighted distribution,
            // which is physically correct for Lambertian surfaces.
            Vec3 scatter_dir = rec.normal + random_unit_vector(rng);

            // Catch degenerate scatter direction (when random vector ~ -normal)
            if (scatter_dir.length_squared() < 1e-8)
                scatter_dir = rec.normal;

            Ray scattered(rec.p, scatter_dir);
            return rec.material.albedo * ray_color(scattered, world, depth - 1, rng);
        }
        else if (rec.material.type == METAL) {
            // --- Specular (Mirror) Reflection ---
            Vec3 reflected = Vec3::reflect(r.direction.normalized(), rec.normal);

            // Add fuzz: perturb the reflection direction randomly.
            // fuzz = 0 -> perfect mirror; fuzz = 1 -> very rough metal.
            Vec3 fuzzed = reflected + random_in_unit_sphere(rng) * rec.material.fuzz;
            Ray scattered(rec.p, fuzzed);

            // If the fuzzed reflection goes below the surface, absorb the ray.
            if (scattered.direction.dot(rec.normal) > 0)
                return rec.material.albedo * ray_color(scattered, world, depth - 1, rng);
            else
                return Vec3(0, 0, 0);
        }
    }

    // --- Skybox Gradient (no surface hit) ---
    // Lerp between white and a sky blue based on the ray's y-component.
    // This creates a pleasant gradient that acts as ambient lighting.
    Vec3 unit_direction = r.direction.normalized();
    double t = 0.5 * (unit_direction.y + 1.0); // Map y from [-1,1] to [0,1]
    Vec3 sky_white(1.0, 1.0, 1.0);
    Vec3 sky_blue(0.5, 0.7, 1.0);
    return sky_white * (1.0 - t) + sky_blue * t; // Linear interpolation
}


// =============================================================================
//  SECTION 6: CAMERA SYSTEM
// =============================================================================
//  A simple pinhole camera model. The camera is defined by:
//    - lookfrom : camera position in world space
//    - lookat   : the point the camera is aimed at
//    - vup      : the "up" direction (usually world Y-axis)
//    - vfov     : vertical field of view in degrees
//    - aspect   : aspect ratio (width / height)
//
//  The camera constructs an orthonormal basis (u, v, w) and computes the
//  viewport corners. To generate a ray for pixel (s, t), we interpolate
//  across the viewport plane.
// =============================================================================

struct Camera {
    Vec3 origin;
    Vec3 lower_left_corner;
    Vec3 horizontal;
    Vec3 vertical;

    Camera(Vec3 lookfrom, Vec3 lookat, Vec3 vup, double vfov, double aspect_ratio) {
        // Convert vertical FOV from degrees to radians.
        double theta = vfov * 3.14159265358979323846 / 180.0;
        double h = std::tan(theta / 2.0);

        // Viewport dimensions in world-space units.
        double viewport_height = 2.0 * h;
        double viewport_width = aspect_ratio * viewport_height;

        // Construct orthonormal camera basis vectors:
        //   w = unit vector pointing from lookat toward lookfrom (backward)
        //   u = unit vector pointing camera-right
        //   v = unit vector pointing camera-up
        Vec3 w = (lookfrom - lookat).normalized();
        Vec3 u = vup.cross(w).normalized();
        Vec3 v = w.cross(u);

        origin = lookfrom;
        horizontal = u * viewport_width;
        vertical = v * viewport_height;

        // Lower-left corner of the virtual viewport in world space.
        lower_left_corner = origin - horizontal / 2.0 - vertical / 2.0 - w;
    }

    // Generate a ray from the camera through viewport coordinates (s, t).
    // s in [0,1] sweeps left to right; t in [0,1] sweeps bottom to top.
    Ray get_ray(double s, double t) const {
        Vec3 direction = lower_left_corner + horizontal * s + vertical * t - origin;
        return Ray(origin, direction);
    }
};


// =============================================================================
//  SECTION 7: TERMINAL OUTPUT FORMATTING
// =============================================================================
//  Professional console output with aligned fields.
// =============================================================================

// Simple padding helper to align console columns
void pad_to(int current_len, int target_len) {
    for (int i = current_len; i < target_len; ++i) std::cout << " ";
}

void print_header() {
    std::cout << "\n";
    std::cout << "  +==============================================================+\n";
    std::cout << "  |           PROJECT LUMINA -- Ray Tracing Engine                |\n";
    std::cout << "  |      Unit V: Parallel Organization Demonstration             |\n";
    std::cout << "  +==============================================================+\n";
    std::cout << "\n";
}

void print_config(int width, int height, int samples, int max_depth, int num_cores, int num_spheres) {
    std::cout << "  +------------------ RENDER CONFIGURATION --------------------+\n";

    // Resolution
    std::ostringstream res; res << width << " x " << height << " pixels";
    std::cout << "  |  Resolution      : " << res.str();
    pad_to((int)res.str().size(), 39);
    std::cout << "|\n";

    // Samples
    std::ostringstream ss1; ss1 << samples;
    std::cout << "  |  Samples/Pixel   : " << ss1.str();
    pad_to((int)ss1.str().size(), 39);
    std::cout << "|\n";

    // Max depth
    std::ostringstream ss2; ss2 << max_depth;
    std::cout << "  |  Max Ray Depth   : " << ss2.str();
    pad_to((int)ss2.str().size(), 39);
    std::cout << "|\n";

    // Spheres
    std::ostringstream ss3; ss3 << num_spheres << " spheres";
    std::cout << "  |  Scene Objects   : " << ss3.str();
    pad_to((int)ss3.str().size(), 39);
    std::cout << "|\n";

    // Cores
    std::ostringstream ss4; ss4 << num_cores;
    std::cout << "  |  Active Cores    : " << ss4.str();
    pad_to((int)ss4.str().size(), 39);
    std::cout << "|\n";

    // Execution mode
    std::string mode = (num_cores == 1) ? "SEQUENTIAL (Single-Core)" : "PARALLEL (Multi-Core SMP)";
    std::cout << "  |  Execution Mode  : " << mode;
    pad_to((int)mode.size(), 39);
    std::cout << "|\n";

    std::cout << "  +------------------------------------------------------------+\n\n";
}


// =============================================================================
//  [UNIT V DEMONSTRATION: SYMMETRIC MULTIPROCESSING & DOMAIN DECOMPOSITION]
// =============================================================================
//
//  +=========================================================================+
//  |  CONCEPT: DOMAIN DECOMPOSITION                                          |
//  |                                                                         |
//  |  The image is our "domain" -- a 2D grid of pixels. We decompose this    |
//  |  domain by splitting it into HORIZONTAL BANDS (row ranges). Each band   |
//  |  is assigned to a separate thread (processing core).                    |
//  |                                                                         |
//  |  +----------------------------------+                                   |
//  |  |  Thread 0: rows 0   - 149       |  <- Core 0                        |
//  |  |  Thread 1: rows 150 - 299       |  <- Core 1                        |
//  |  |  Thread 2: rows 300 - 449       |  <- Core 2                        |
//  |  |  Thread 3: rows 450 - 599       |  <- Core 3                        |
//  |  +----------------------------------+                                   |
//  |                                                                         |
//  |  WHY THIS WORKS:                                                        |
//  |  * Each pixel's color calculation is INDEPENDENT -- no data deps.       |
//  |    This makes ray tracing "embarrassingly parallel".                    |
//  |  * Each thread writes to its own region of the shared pixel buffer,     |
//  |    so NO synchronization (mutex/lock) is needed for the buffer itself.  |
//  |  * The only synchronization point is the join barrier at the end,       |
//  |    where the main thread waits for all workers to finish.               |
//  |                                                                         |
//  |  EXPECTED RESULT:                                                       |
//  |  * NUM_CORES = 1 -> baseline time T                                     |
//  |  * NUM_CORES = N -> time ~ T / N  (near-linear speedup)                |
//  |  * Demonstrates Amdahl's Law in practice.                               |
//  +=========================================================================+
//
//  SECTION 8: RENDER WORKER FUNCTION (Executed by each thread)
// =============================================================================
//
//  Each thread executes this function with its own [row_start, row_end) range.
//  The function iterates over its assigned rows, and for each pixel:
//    1. Casts SAMPLES_PER_PIXEL rays with slight random jitter (anti-aliasing)
//    2. Averages the returned colors
//    3. Applies gamma correction (gamma = 2.0, i.e., sqrt)
//    4. Writes the final color to the shared pixel buffer
//
//  NOTE: The pixel buffer is a flat std::vector<Vec3> shared across threads,
//        but each thread writes ONLY to its assigned row range, so there is
//        NO data race and NO locking is required for the buffer.
// =============================================================================

// --- Thread argument structure ----------------------------------------------
//  Bundles all data a worker thread needs. Passed as a void* on Windows
//  (CreateThread requirement) and directly on POSIX.
struct ThreadArgs {
    int thread_id;
    int row_start;
    int row_end;
    int image_width;
    int image_height;
    int samples_per_pixel;
    int max_depth;
    const Camera* cam;
    const std::vector<Sphere>* world;
    Vec3* pixel_buffer;
};

// --- The core render function executed by each thread -----------------------
void render_chunk(ThreadArgs* args) {
    int thread_id       = args->thread_id;
    int row_start       = args->row_start;
    int row_end         = args->row_end;
    int image_width     = args->image_width;
    int image_height    = args->image_height;
    int samples_per_pixel = args->samples_per_pixel;
    int max_depth       = args->max_depth;
    const Camera& cam   = *(args->cam);
    const std::vector<Sphere>& world = *(args->world);
    Vec3* pixel_buffer  = args->pixel_buffer;

    // Each thread gets a unique RNG seed so samples are uncorrelated.
    RNG rng((unsigned long long)(thread_id * 7919 + 42));

    // --- Per-pixel rendering loop -------------------------------------------
    for (int j = row_start; j < row_end; ++j) {
        for (int i = 0; i < image_width; ++i) {

            Vec3 pixel_color(0, 0, 0);

            // --- ANTI-ALIASING: Multiple Samples Per Pixel ------------------
            //  Instead of casting ONE ray through each pixel center, we cast
            //  SAMPLES_PER_PIXEL rays through randomly jittered positions
            //  within the pixel footprint. This smooths edges and reduces
            //  aliasing artifacts.
            //
            //  Each sample contributes equally to the final color.
            //  This is a Monte Carlo integration of the rendering equation
            //  over the pixel area.
            // ----------------------------------------------------------------
            for (int s = 0; s < samples_per_pixel; ++s) {
                // Map pixel coordinates to viewport [0,1] with subpixel jitter.
                double u = (i + rng.random_double()) / (image_width - 1);
                double v = (j + rng.random_double()) / (image_height - 1);

                // Generate primary ray from camera through this viewport point.
                Ray r = cam.get_ray(u, v);

                // Trace the ray into the scene and accumulate color.
                pixel_color += ray_color(r, world, max_depth, rng);
            }

            // --- Average and Gamma Correct ----------------------------------
            //  Divide accumulated color by number of samples.
            //  Apply gamma correction (gamma = 2.0 -> sqrt) to convert from
            //  linear light space to sRGB for display.
            // ----------------------------------------------------------------
            double scale = 1.0 / samples_per_pixel;
            pixel_color *= scale;
            pixel_color = Vec3(
                std::sqrt(pixel_color.x),  // Gamma correction: linear -> sRGB
                std::sqrt(pixel_color.y),
                std::sqrt(pixel_color.z)
            );
            pixel_color = pixel_color.clamped();

            // Write to pixel buffer. Row j, column i.
            // PPM format is top-to-bottom, so we flip vertically:
            //   buffer row 0 = image top = scene row (image_height - 1)
            int buffer_row = image_height - 1 - j;
            pixel_buffer[buffer_row * image_width + i] = pixel_color;
        }
    }
}

// --- Platform-specific thread entry points ----------------------------------
#ifdef _WIN32
// Windows CreateThread requires a function with signature: DWORD WINAPI func(LPVOID)
DWORD WINAPI thread_entry_win32(LPVOID arg) {
    render_chunk((ThreadArgs*)arg);
    return 0;
}
#else
// POSIX pthread_create requires: void* func(void*)
void* thread_entry_posix(void* arg) {
    render_chunk((ThreadArgs*)arg);
    return NULL;
}
#endif


// =============================================================================
//  SECTION 9: MAIN — ORCHESTRATION, TIMING, AND OUTPUT
// =============================================================================

int main() {

    // =========================================================================
    //   ____  ___  _   _ _____ ___ ____
    //  / ___|/ _ \| \ | |  ___|_ _/ ___|
    // | |  | | | |  \| | |_   | | |  _
    // | |__| |_| | |\  |  _|  | | |_| |
    //  \____\___/|_| \_|_|   |___\____|
    //
    //  EASILY EDITABLE RENDER PARAMETERS
    //  Change these values to control image quality and parallelism.
    // =========================================================================

    // --- Image Dimensions ---------------------------------------------------
    const int IMAGE_WIDTH  = 800;                              // Horizontal resolution
    const int IMAGE_HEIGHT = 450;                              // Vertical resolution
    const double ASPECT_RATIO = (double)IMAGE_WIDTH / IMAGE_HEIGHT;

    // --- Rendering Quality --------------------------------------------------
    const int SAMPLES_PER_PIXEL = 100;  // Anti-aliasing samples (higher = smoother, slower)
    const int MAX_DEPTH         = 50;   // Maximum ray bounces (higher = more realism)

    // =========================================================================
    //  +===============================================================+
    //  |  >>> NUM_CORES -- THE KEY VARIABLE FOR SMP DEMONSTRATION <<<  |
    //  |                                                               |
    //  |  Set to 1  -> Sequential execution (single-threaded baseline) |
    //  |  Set to 2  -> 2-way parallelism                               |
    //  |  Set to 4  -> 4-way parallelism                               |
    //  |  Set to 8  -> 8-way parallelism                               |
    //  |  Set to 16 -> 16-way parallelism                              |
    //  |                                                               |
    //  |  CHANGE THIS VALUE AND RECOMPILE TO SEE THE SPEEDUP!          |
    //  +===============================================================+
    // =========================================================================
    const int NUM_CORES = 8;  // <<<<< CHANGE THIS TO COMPARE EXECUTION TIMES

    // --- Output File --------------------------------------------------------
    const std::string OUTPUT_FILE = "lumina_render.ppm";


    // =========================================================================
    //  SCENE CONSTRUCTION
    //  Build the world: a collection of spheres with different materials.
    //  The scene is designed to be visually impressive while generating
    //  heavy computational load (many ray-sphere intersections and bounces).
    // =========================================================================

    std::vector<Sphere> world;

    // --- Ground Plane -------------------------------------------------------
    //  A very large sphere centered far below the scene acts as an infinite
    //  ground plane. Its surface is close to y = 0 at the tangent point.
    world.push_back(Sphere(
        Vec3(0, -1000, 0), 1000,
        Material(DIFFUSE, Vec3(0.45, 0.65, 0.35))  // Olive green ground
    ));

    // --- Central Matte Sphere (Lambertian Diffuse) --------------------------
    //  A large, prominent sphere in the center of the scene.
    //  Diffuse material scatters light in all directions — soft appearance.
    world.push_back(Sphere(
        Vec3(0, 1, 0), 1.0,
        Material(DIFFUSE, Vec3(0.7, 0.15, 0.15))  // Deep crimson
    ));

    // --- Left Metal Sphere (Highly Reflective) ------------------------------
    //  A polished metal sphere. fuzz = 0.0 -> perfect mirror reflection.
    //  Reflections of other spheres and the sky will be clearly visible.
    world.push_back(Sphere(
        Vec3(-2.2, 0.8, 0), 0.8,
        Material(METAL, Vec3(0.85, 0.85, 0.90), 0.0)  // Polished silver
    ));

    // --- Right Metal Sphere (Brushed Metal / Fuzzy Reflection) --------------
    //  A rougher metal sphere. fuzz = 0.3 -> blurred reflections.
    //  This contrast with the polished sphere highlights material diversity.
    world.push_back(Sphere(
        Vec3(2.0, 0.6, -0.5), 0.6,
        Material(METAL, Vec3(0.95, 0.75, 0.30), 0.3)  // Brushed gold
    ));

    // --- Small Accent Sphere (Far Left Background) --------------------------
    world.push_back(Sphere(
        Vec3(-4.0, 0.4, -2.0), 0.4,
        Material(DIFFUSE, Vec3(0.1, 0.3, 0.7))  // Cobalt blue
    ));

    // --- Small Accent Sphere (Far Right Background) -------------------------
    world.push_back(Sphere(
        Vec3(3.5, 0.35, -1.5), 0.35,
        Material(METAL, Vec3(0.8, 0.4, 0.8), 0.1)  // Lavender metal
    ));

    // --- Small Foreground Sphere --------------------------------------------
    world.push_back(Sphere(
        Vec3(0.8, 0.25, 1.5), 0.25,
        Material(DIFFUSE, Vec3(0.95, 0.55, 0.1))  // Orange
    ));

    // --- Additional Spheres for Visual Complexity and Heavier Workload ------
    world.push_back(Sphere(
        Vec3(-1.0, 0.3, 2.0), 0.3,
        Material(METAL, Vec3(0.9, 0.9, 0.95), 0.05)  // Near-perfect mirror
    ));

    world.push_back(Sphere(
        Vec3(1.5, 0.2, 2.5), 0.2,
        Material(DIFFUSE, Vec3(0.3, 0.8, 0.4))  // Emerald green
    ));

    world.push_back(Sphere(
        Vec3(-0.5, 0.15, 3.0), 0.15,
        Material(DIFFUSE, Vec3(0.9, 0.2, 0.6))  // Hot pink
    ));


    // =========================================================================
    //  CAMERA SETUP
    //  Position the camera to capture the full scene from a slightly elevated
    //  angle, providing depth and perspective to the composition.
    // =========================================================================

    Vec3 lookfrom(0, 2.5, 6);    // Camera position: slightly above and in front
    Vec3 lookat(0, 0.5, 0);      // Look at the center of the scene
    Vec3 vup(0, 1, 0);           // World up direction
    double vfov = 30.0;          // Narrow FOV for a "telephoto" feel

    Camera cam(lookfrom, lookat, vup, vfov, ASPECT_RATIO);


    // =========================================================================
    //  PIXEL BUFFER ALLOCATION
    //  A flat array of Vec3 values, one per pixel. Stored in row-major order.
    //  All threads write to disjoint regions -> no data races, no locks.
    // =========================================================================

    std::vector<Vec3> pixel_buffer(IMAGE_WIDTH * IMAGE_HEIGHT);


    // =========================================================================
    //  TERMINAL OUTPUT — RENDER CONFIGURATION
    // =========================================================================

    print_header();
    print_config(IMAGE_WIDTH, IMAGE_HEIGHT, SAMPLES_PER_PIXEL, MAX_DEPTH,
                 NUM_CORES, (int)world.size());


    // =========================================================================
    //  [UNIT V DEMONSTRATION: SYMMETRIC MULTIPROCESSING & DOMAIN DECOMPOSITION]
    //
    //  This is the CORE of the parallelism demonstration.
    //
    //  DOMAIN DECOMPOSITION STRATEGY:
    //    The image has IMAGE_HEIGHT rows total. We divide these rows into
    //    NUM_CORES contiguous, non-overlapping bands:
    //
    //    Thread i gets rows:   [i * chunk_size,  min((i+1) * chunk_size, HEIGHT))
    //
    //    This ensures:
    //    [OK] Complete coverage:   every row is assigned to exactly one thread
    //    [OK] Load balance:        each thread gets ~ HEIGHT/NUM_CORES rows
    //    [OK] No overlap:          threads never write to the same memory
    //    [OK] Last-thread fixup:   the last thread absorbs any remainder rows
    //
    //  THREAD LIFECYCLE:
    //    1. SPAWN:  OS creates a new thread -> maps to a CPU core
    //    2. EXECUTE: each thread runs render_chunk() independently
    //    3. JOIN:   main thread blocks until worker finishes
    //
    //  The timing measurement captures ONLY the parallel render phase:
    //    start = now()
    //    spawn threads -> execute -> join all
    //    end = now()
    //    elapsed = end - start
    // =========================================================================

    std::cout << "  +------------ THREAD ALLOCATION (DOMAIN DECOMPOSITION) ------+\n";

    // --- Prepare thread arguments -------------------------------------------
    int rows_per_chunk = IMAGE_HEIGHT / NUM_CORES;
    std::vector<ThreadArgs> thread_args(NUM_CORES);

    for (int t = 0; t < NUM_CORES; ++t) {
        thread_args[t].thread_id       = t;
        thread_args[t].row_start       = t * rows_per_chunk;
        thread_args[t].row_end         = (t == NUM_CORES - 1) ? IMAGE_HEIGHT : (t + 1) * rows_per_chunk;
        thread_args[t].image_width     = IMAGE_WIDTH;
        thread_args[t].image_height    = IMAGE_HEIGHT;
        thread_args[t].samples_per_pixel = SAMPLES_PER_PIXEL;
        thread_args[t].max_depth       = MAX_DEPTH;
        thread_args[t].cam             = &cam;
        thread_args[t].world           = &world;
        thread_args[t].pixel_buffer    = &pixel_buffer[0];

        // Print thread allocation info
        int rows_assigned = thread_args[t].row_end - thread_args[t].row_start;
        std::cout << "    [THREAD " << t << "]  Assigned rows "
                  << thread_args[t].row_start << " - " << thread_args[t].row_end;
        std::cout.precision(1);
        std::cout << std::fixed;
        std::cout << "  (" << rows_assigned << " rows, "
                  << (100.0 * rows_assigned / IMAGE_HEIGHT) << "% of image)\n";
    }

    std::cout << "  +------------------------------------------------------------+\n\n";

    // --- Start the high-resolution timer ------------------------------------
    auto time_start = std::chrono::high_resolution_clock::now();

    // =========================================================================
    //  [UNIT V DEMONSTRATION: SYMMETRIC MULTIPROCESSING & DOMAIN DECOMPOSITION]
    //
    //  SPAWNING WORKER THREADS:
    //  Each thread is created by the OS and mapped to an available CPU core.
    //
    //  KEY INSIGHT FOR UNIT V:
    //    - With NUM_CORES = 1, this loop runs once -> single thread -> sequential.
    //    - With NUM_CORES = N, N threads execute SIMULTANEOUSLY on N cores.
    //    - The workload per thread is ~1/N of the total -> near-linear speedup.
    //
    //  PLATFORM ABSTRACTION:
    //    Windows: CreateThread() + WaitForMultipleObjects()
    //    Linux:   pthread_create() + pthread_join()
    //    Both achieve the same SMP parallelism.
    // =========================================================================

    std::cout << "  >> Rendering in progress... (all " << NUM_CORES << " cores active)\n\n";

#ifdef _WIN32
    // --- WINDOWS: Win32 Threading API ---------------------------------------
    std::vector<HANDLE> handles(NUM_CORES);

    for (int t = 0; t < NUM_CORES; ++t) {
        // CreateThread: Spawns a new OS thread.
        //   - Security attrs: NULL (default)
        //   - Stack size: 0 (default, typically 1 MB)
        //   - Start routine: thread_entry_win32
        //   - Argument: pointer to this thread's ThreadArgs
        //   - Flags: 0 (start immediately)
        //   - Thread ID: NULL (we don't need it)
        handles[t] = CreateThread(NULL, 0, thread_entry_win32, &thread_args[t], 0, NULL);

        if (handles[t] == NULL) {
            std::cerr << "  ERROR: Failed to create thread " << t << "\n";
            return 1;
        }
    }

    // --- SYNCHRONIZATION BARRIER: Wait for ALL threads to complete ----------
    //  WaitForMultipleObjects blocks the main thread until every worker
    //  thread has finished executing. This is the join barrier.
    //
    //  bWaitAll = TRUE -> wait for ALL threads (not just one)
    //  INFINITE -> no timeout, wait as long as needed
    WaitForMultipleObjects(NUM_CORES, &handles[0], TRUE, INFINITE);

    // Clean up thread handles.
    for (int t = 0; t < NUM_CORES; ++t) {
        CloseHandle(handles[t]);
    }

#else
    // --- LINUX/macOS: POSIX Threads (pthreads) ------------------------------
    std::vector<pthread_t> pthreads(NUM_CORES);

    for (int t = 0; t < NUM_CORES; ++t) {
        int result = pthread_create(&pthreads[t], NULL, thread_entry_posix, &thread_args[t]);
        if (result != 0) {
            std::cerr << "  ERROR: Failed to create thread " << t << "\n";
            return 1;
        }
    }

    // --- SYNCHRONIZATION BARRIER: Join all threads --------------------------
    for (int t = 0; t < NUM_CORES; ++t) {
        pthread_join(pthreads[t], NULL);
    }
#endif

    // --- Stop the timer -----------------------------------------------------
    auto time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = time_end - time_start;

    std::cout << "  >> All threads completed.\n\n";


    // =========================================================================
    //  FILE OUTPUT — WRITE PPM IMAGE
    //  The PPM (Portable Pixmap) format is the simplest uncompressed image
    //  format. No external libraries needed.
    //
    //  Format:
    //    P3              <- Magic number (ASCII RGB)
    //    width height    <- Image dimensions
    //    255             <- Maximum color value
    //    r g b           <- One pixel per line, values in [0, 255]
    //    r g b
    //    ...
    // =========================================================================

    std::cout << "  >> Writing image to " << OUTPUT_FILE << "... ";
    std::cout.flush();

    std::ofstream file(OUTPUT_FILE.c_str());
    if (!file.is_open()) {
        std::cerr << "\n  ERROR: Could not open file for writing: " << OUTPUT_FILE << "\n";
        return 1;
    }

    // PPM header
    file << "P3\n" << IMAGE_WIDTH << " " << IMAGE_HEIGHT << "\n255\n";

    // Write pixel data row by row (buffer is already in top-to-bottom order)
    for (int idx = 0; idx < IMAGE_WIDTH * IMAGE_HEIGHT; ++idx) {
        int r = static_cast<int>(255.999 * pixel_buffer[idx].x);
        int g = static_cast<int>(255.999 * pixel_buffer[idx].y);
        int b = static_cast<int>(255.999 * pixel_buffer[idx].z);
        file << r << " " << g << " " << b << "\n";
    }

    file.close();
    std::cout << "Done.\n\n";


    // =========================================================================
    //  FINAL PERFORMANCE REPORT
    // =========================================================================

    std::cout << "  +==============================================================+\n";
    std::cout << "  |                    PERFORMANCE REPORT                         |\n";
    std::cout << "  +==============================================================+\n";

    std::ostringstream coreStr; coreStr << NUM_CORES;
    std::cout << "  |  Cores Used       : " << coreStr.str();
    pad_to((int)coreStr.str().size(), 39);
    std::cout << "|\n";

    std::ostringstream pixStr; pixStr << (IMAGE_WIDTH * IMAGE_HEIGHT);
    std::cout << "  |  Total Pixels     : " << pixStr.str();
    pad_to((int)pixStr.str().size(), 39);
    std::cout << "|\n";

    long long total_rays = (long long)IMAGE_WIDTH * IMAGE_HEIGHT * SAMPLES_PER_PIXEL;
    std::ostringstream rayStr; rayStr << total_rays;
    std::cout << "  |  Primary Rays     : " << rayStr.str();
    pad_to((int)rayStr.str().size(), 39);
    std::cout << "|\n";

    std::ostringstream timeStr;
    timeStr << std::fixed;
    timeStr.precision(4);
    timeStr << elapsed.count() << " seconds";
    std::cout << "  |  Render Time      : " << timeStr.str();
    pad_to((int)timeStr.str().size(), 39);
    std::cout << "|\n";

    double mrays_per_sec = total_rays / elapsed.count() / 1000000.0;
    std::ostringstream tpStr;
    tpStr << std::fixed;
    tpStr.precision(2);
    tpStr << mrays_per_sec << " Mrays/sec";
    std::cout << "  |  Throughput       : " << tpStr.str();
    pad_to((int)tpStr.str().size(), 39);
    std::cout << "|\n";

    std::cout << "  |  Output File      : " << OUTPUT_FILE;
    pad_to((int)OUTPUT_FILE.size(), 39);
    std::cout << "|\n";

    std::cout << "  +--------------------------------------------------------------+\n";

    if (NUM_CORES == 1) {
        std::cout << "  |  MODE: SEQUENTIAL (Baseline)                                |\n";
        std::cout << "  |  Re-run with NUM_CORES > 1 to observe parallel speedup.     |\n";
    } else {
        std::cout << "  |  MODE: PARALLEL (Symmetric Multiprocessing)                  |\n";
        std::cout << "  |  Compare with NUM_CORES = 1 to measure speedup ratio.       |\n";
    }

    std::cout << "  +==============================================================+\n\n";

    std::cout << "  >> Render complete. Open " << OUTPUT_FILE << " to view the result.\n\n";

    return 0;
}

// =============================================================================
//  COMPILATION INSTRUCTIONS
// =============================================================================
//
//  +-------------------------------------------------------------------------+
//  |  WINDOWS (MinGW g++)                                                    |
//  |  --------------------                                                   |
//  |  g++ -std=c++11 -O2 -o lumina.exe main.cpp                             |
//  |  .\lumina.exe                                                           |
//  |  (Uses Win32 CreateThread natively; no -pthread flag needed.)           |
//  |                                                                         |
//  |  WINDOWS (MSVC -- Developer Command Prompt / PowerShell)                |
//  |  --------------------------------------------------------              |
//  |  cl /EHsc /O2 /std:c++14 main.cpp /Fe:lumina.exe                       |
//  |  .\lumina.exe                                                           |
//  |  (MSVC uses Windows threads natively; no -pthread flag needed.)         |
//  |                                                                         |
//  |  LINUX / macOS (g++ or clang++)                                         |
//  |  ---------------------------------                                      |
//  |  g++ -std=c++11 -O2 -pthread -o lumina main.cpp                        |
//  |  ./lumina                                                               |
//  |                                                                         |
//  |  NOTE: The -pthread flag is REQUIRED on Linux to link the POSIX         |
//  |        threading library. Without it, pthread_create will cause a       |
//  |        linker error.                                                    |
//  |                                                                         |
//  |  OPTIONAL -- Higher optimization:                                       |
//  |  g++ -std=c++17 -O3 -march=native -pthread -o lumina main.cpp          |
//  +-------------------------------------------------------------------------+
//
// =============================================================================
