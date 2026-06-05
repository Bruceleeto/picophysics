/**
 * # Picophysics
 *
 * Picophysics is a very basic physics engine for games. It is not supposed to be fully realistic
 * and is designed to be used particularly on low power systems (N64, Dreamcast). It was built
 * for the N64 port of the Dreamcast game Driving Strikers, where every cycle counts and I needed
 * full control and understanding of what was happening.
 *
 * # Features
 *
 * - Composite bodies made of multiple sphere and box shapes with local offsets
 * - Create fully dynamic spheres and boxes and apply linear and angular forces
 * - Create static environments using triangles
 * - Easy to use collision callback system to respond to detected collisions and to
 *   choose whether to respond at at all (return true to respond)
 * - Ray casting
 * - Axis-locking of rotations
 * - Bodies have friction and bounciness coefficients
 * - Very fast, no dynamic memory allocations
 *
 * # Overview
 *
 * Bodies are composable: each body can hold zero or more shapes (spheres and/or boxes), each
 * with a local offset from the body center. The mass and inertia tensor are automatically
 * computed from the shapes using the parallel axis theorem.
 *
 * Shape primitives:
 *
 * - Spheres (fully dynamic and responsive)
 * - Boxes (only collides with spheres currently)
 * - Triangles (static environment, not a body shape)
 *
 * Bodies can also be joined together through constraints. Currently only fixed distance constraints are supported.
 *
 * Currently Spheres are the only fully dynamic and responsive object. Ray-casting
 * the world is also supported.
 *
 * There is only one global "world", all things are created with in it, and you can empty
 * it with pp_physics_clear(). You also don't need to initialise the world, just start creating
 * bodies and call pp_physics_step(dt, iterations) to update.
 *
 * The library statically allocates memory, by default you can have:
 *
 *  - 32 bodies (spheres + boxes)
 *  - 128 triangles
 *
 * If you need more than that you can define PICOPHYSICS_MAX_OBJECTS or PICOPHYSICS_MAX_TRIANGLES
 * before including physics.h.
 *
 * # Support
 *
 * Although this is an open-source project, I do not have the time to provide support for it. If you send me an MR
 * I'll review it and merge it if it's good - that's about as much as I can do. This has been written for my own purposes
 * primarily.
 *
 * If you find this project useful in some way, please consider buying me a coffee at https://ko-fi.com/kazade or supporting me
 * on Patreon at https://www.patreon.com/kazade
 *
 * # Help needed!
 *
 * I am *not* a mathematician! Collision response is something I'm finding quite
 * difficult to understand (particularly angular/torque responses). There are definitely
 * issues in the collision response code. If you can help fix it, I'd appreciate it!
 *
 * # Roadmap
 *
 * - Proper multi-point manifold generation for GJK/EPA
 * - Potentially switch to SAT as GJK is only being used for box/box collisions and SAT seems more suitable for one-shot manifold generation
 * - Add box/triangle collisions
 * - Add box/box collisions
 * - Broad-phase collision detection (spatial hashing)
 * - Add fixed and spring joints (links) between objects
 * - Optimisations (replacing divisions where possible)
 * - Slab allocation (so it's possible to overflow the static array)
 * - Make structs opaque
 *
 * I have no intention of adding more than this! If you want something more there are a bunch
 * of great open-source physics engines out there (e.g. Bullet, Box2D, ODE, Bounce..)
 *
 * # Usage
 *
 * Picophysics is a single-file header library (in the spirit of stb). To use it
 * you must do this in a single .c/.cpp file:
 *
 * #define PICOPHYSICS_IMPLEMENTATION
 * #include "picophysics.h"
 *
 * You must regularly call `pp_physics_step(step, vel_iterations, pos_iterations)` to run the simulation.
 *
 * # Examples
 *
 * The examples are written using the Simulant engine. To build them you'll need Docker, Python
 * and [Simulant Tools](https://gitlab.com/simulant/simulant-tools) which you can install with:
 *
 * - pip3 install -U --user git+https://gitlab.com/simulant/simulant-tools.git
 *
 * Once installed, from the example directory run:
 *
 *  - simulant update -b next
 *  - simulant run --rebuild
 *
 * CHANGELOG
 *
 * - ALPHA - no releases yet
 *
 * The MIT License (MIT)
 * Copyright © 2026 Luke Benstead
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PICOPHYSICS_H
#define PICOPHYSICS_H

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _PPVec3
{
    union {
        struct
        {
            float x, y, z;
        };
        float xyz[3];
    };
} PPVec3;

typedef struct _PPQuaternion
{
    union {
        struct
        {
            float x, y, z, w;
        };
        float xyzw[4];
    };
} PPQuaternion;

typedef struct _PPMat3
{
    float m[9];
} PPMat3;

typedef struct _PPPlane
{
    PPVec3 n;
    float d;
} PPPlane;

typedef uint8_t BodyKind;

typedef enum _PPObjectType {
    PP_OBJECT_TYPE_SPHERE,
    PP_OBJECT_TYPE_BOX,
    PP_OBJECT_TYPE_TRIANGLE,
} PPObjectType;

#ifndef PP_MAX_SHAPES_PER_BODY
#define PP_MAX_SHAPES_PER_BODY 8
#endif

typedef struct _PPShape {
    PPObjectType type;
    PPVec3 offset;
    float mass;
    BodyKind kind;
    union {
        struct { float radius; } sphere;
        struct { PPVec3 whd; PPVec3 half_extents; float bounding_radius; } box;
    };
} PPShape;

typedef struct _PPBody PPBody;

typedef struct _PPCollision
{
    PPVec3 p;
    PPVec3 n;
    PPBody *obj1;
    PPBody *obj2;

    // These are stored separately as obj1/obj2 will
    // be null if we collided with geometry, but these
    // values are still necessary in the null case
    float obj1_bounce;
    float obj2_bounce;
    float obj1_friction;
    float obj2_friction;

    PPObjectType type1;
    PPObjectType type2;

    BodyKind kind1;
    BodyKind kind2;

    float dist; // Penetration depth (positive when overlapping)

    // Signed gap between the two surfaces: negative when penetrating, positive
    // when there is still a gap. A positive value indicates a *speculative*
    // contact -- the bodies are not touching yet but are predicted to make
    // contact within the current step. The velocity solver uses this to brake
    // an approaching body so it closes the gap without overshooting (passing
    // through), which is what prevents fast movers from tunnelling.
    float separation;
} PPCollision;

typedef enum _PPAxisLock {
    PP_AXIS_LOCK_NONE,
    PP_AXIS_LOCK_PITCH = 0x1,
    PP_AXIS_LOCK_YAW = 0x2,
    PP_AXIS_LOCK_ROLL = 0x4,
    PP_AXIS_LOCK_PITCH_AND_ROLL = PP_AXIS_LOCK_PITCH | PP_AXIS_LOCK_ROLL,
    PP_AXIS_LOCK_PITCH_AND_YAW = PP_AXIS_LOCK_PITCH | PP_AXIS_LOCK_YAW,
    PP_AXIS_LOCK_YAW_AND_ROLL = PP_AXIS_LOCK_YAW | PP_AXIS_LOCK_ROLL,
    PP_AXIS_LOCK_ALL = PP_AXIS_LOCK_PITCH | PP_AXIS_LOCK_YAW | PP_AXIS_LOCK_ROLL
} PPAxisLock;

typedef enum _PPConstraintType {
    PP_CONSTRAINT_TYPE_FIXED_DISTANCE,
    PP_CONSTRAINT_TYPE_FIXED,
} PPConstraintType;

typedef struct _PPFixedDistanceConstraint {
    float distance;
} PPFixedDistanceConstraint;

typedef struct _PPConstraint {
    bool is_alive;
    PPConstraintType type;
    PPBody* body1;
    PPBody* body2;

    union {
        PPFixedDistanceConstraint fixed_distance;
    };

    // Combined mass of both bodies.
    float inv_mass_sum;
} PPConstraint;

typedef struct _PPBody
{
    PPVec3 pos;
    PPQuaternion rot;
    PPVec3 vel;
    PPVec3 acc;
    PPVec3 a_vel;
    PPVec3 a_acc;

    float bounce;
    float friction;
    float damping;
    float a_damping;

    float mass;
    float inv_mass;

    PPMat3 inertia;
    PPMat3 inv_inertia;

    BodyKind kind;

    PPAxisLock lock;

    bool is_alive;
    void *user_data;

    float vel_limit;
    float a_vel_limit;

    // Used to reduce or increase gravity applied
    // to a particular body
    float gravity_multiplier;

    PPShape shapes[PP_MAX_SHAPES_PER_BODY];
    int shape_count;
} PPBody;

typedef struct _PPTriangle
{
    PPVec3 v[3];
    PPVec3 n;
    PPPlane p;
    BodyKind kind;
    float bounce;
    float friction;
    // Precomputed AABB for fast sphere culling
    float aabb_min_x, aabb_min_y, aabb_min_z;
    float aabb_max_x, aabb_max_y, aabb_max_z;
} PPTriangle;

PPVec3 *pp_vec3_init(PPVec3 *v);
PPVec3 *pp_vec3_set(PPVec3 *v, float x, float y, float z);
PPVec3 *pp_vec3_assign(PPVec3 *target, const PPVec3 *source);
bool pp_vec3_normalize(PPVec3 *target);

static inline float pp_vec3_length(const PPVec3 *v1)
{
    return sqrtf(v1->x * v1->x + v1->y * v1->y + v1->z * v1->z);
}

static inline float pp_vec3_dot(const PPVec3 *v1, const PPVec3 *v2)
{
    return v1->x * v2->x + v1->y * v2->y + v1->z * v2->z;
}

static inline PPVec3 *pp_vec3_scale(const PPVec3 *v1, float t, PPVec3 *out)
{
    out->x = v1->x * t;
    out->y = v1->y * t;
    out->z = v1->z * t;
    return out;
}


PPQuaternion *pp_quat_init(PPQuaternion *q);
void pp_quat_between(const PPVec3 *v0, const PPVec3 *q1, PPQuaternion *result);
void pp_quat_slerp(const PPQuaternion *q0, const PPQuaternion *q1, float t, PPQuaternion *result);
PPQuaternion *pp_quat_assign(PPQuaternion *target, const PPQuaternion *source);
float pp_quat_angle_between(const PPQuaternion *q0, const PPQuaternion *q1);

/**
 * Run the world physics step.
 *
 * @t time step, this should be something fixed (E.g. 1 / 50)
 * @iterations The number of iterations to run in the solver. 8 is a fairly reliable value
 *             but more == slower.
 */
void pp_physics_step(float t, int vel_iterations, int pos_iterations);
bool pp_physics_ray_intersect(const PPVec3 *origin,
                              const PPVec3 *direction,
                              BodyKind *ignore_kinds,
                              const PPBody **body_hit,
                              const PPTriangle **tri_hit,
                              float *distance,
                              PPVec3 *intersection);
void pp_physics_clear();
void pp_physics_set_gravity(const PPVec3 *v);
bool pp_physics_collision_map_add(BodyKind kind1,
                                  BodyKind kind2,
                                  void *user_data,
                                  bool (*callback)(const void *,
                                                   const void *,
                                                   BodyKind,
                                                   BodyKind,
                                                   const PPCollision *c,
                                                   const void *));

PPTriangle *pp_physics_create_triangle(const PPVec3 *v1,
                                       const PPVec3 *v2,
                                       const PPVec3 *v3,
                                       BodyKind kind);
size_t pp_physics_triangle_count();
const PPTriangle *pp_physics_triangle_at(size_t i);

PPBody *pp_physics_create_sphere(float radius, const PPVec3 *pos, float mass, BodyKind kind);

PPBody *pp_physics_create_box(
    float width, float height, float depth, const PPVec3 *pos, float mass, BodyKind kind);
PPBody *pp_physics_create_body(const PPVec3 *pos);
PPShape *pp_body_add_box(PPBody *body, float w, float h, float d, float mass, BodyKind kind, float offset_x, float offset_y, float offset_z);
PPShape *pp_body_add_sphere(PPBody *body, float radius, float mass, BodyKind kind, float offset_x, float offset_y, float offset_z);
size_t pp_body_get_shape_count(const PPBody *body);
const PPShape *pp_body_get_shape(const PPBody *body, size_t index);

size_t pp_body_get_constraint_count(PPBody *body);
PPConstraint* pp_physics_create_fixed_distance_constraint(PPBody* body1, PPBody* body2, float distance);

void pp_physics_destroy_body(PPBody *b);
const PPBody *pp_physics_body_at(size_t i);
size_t pp_physics_body_count();
size_t pp_physics_body_total_count();
bool pp_body_set_bounce(PPBody *s, float b);
void pp_body_add_force(PPBody *s, float x, float y, float z);
void pp_body_add_force_at_position(PPBody *b, const PPVec3 *world_pos, const PPVec3 *force);
void pp_body_add_angular_force(PPBody *s, float x, float y, float z);
void pp_body_lock_axis(PPBody *s, PPAxisLock lock);
void pp_body_set_angular_damping(PPBody *s, float d);
void pp_body_set_damping(PPBody *s, float d);

void pp_body_get_forward(PPBody *s, PPVec3 *f);
void pp_body_get_right(PPBody *s, PPVec3 *r);
void pp_body_get_up(PPBody *s, PPVec3 *u);

void pp_body_set_position(PPBody *s, float x, float y, float z);
void pp_body_get_position(PPBody *s, PPVec3 *pos);
void pp_body_set_rotation(PPBody *s, float x, float y, float z, float w);
void pp_body_get_rotation(PPBody *s, PPQuaternion *rot);
float pp_body_get_radius(PPBody *s);
void pp_body_set_user_data(PPBody *s, void *data);
void *pp_body_get_user_data(const PPBody *s);
void pp_body_set_kind(PPBody *b, BodyKind kind);
BodyKind pp_body_get_kind(const PPBody *b);
bool pp_body_set_friction(PPBody *s, float f);
void pp_body_set_mass(PPBody* b, float mass);
float pp_body_get_mass(const PPBody* b);
void pp_body_get_velocity(const PPBody *s, PPVec3 *vel);
void pp_body_get_velocity_at_position(const PPBody *b, const PPVec3 *p, PPVec3 *ret);
void pp_body_set_angular_velocity(PPBody *s, float x, float y, float z);
void pp_body_set_velocity(PPBody *s, float x, float y, float z);
void pp_body_set_angular_acceleration(PPBody *s, float x, float y, float z);
void pp_body_set_acceleration(PPBody *s, float x, float y, float z);
void pp_body_look_at(PPBody *s, float x, float y, float z);
void pp_body_set_gravity_multiplier(PPBody *b, float multiplier);
float pp_body_get_gravity_multiplier(const PPBody *b);
/**
 * Limit the maximium velocity of the object. The limit will be applied after
 * adding acceleration forces. Passing 0.0f will remove the limit.
 *
 * @param body The body whose velocity will be restricted
 * @param speed A the desired velocity limit.
 */
void pp_body_limit_velocity(PPBody *body, float speed);

/**
 * Limit the maximium angular velocity of the object. The limit will be applied after
 * adding acceleration forces. Passing 0.0f will remove the limit.
 *
 * @param body The body whose angular velocity will be restricted
 * @param speed A the desired angular velocity limit.
 */
void pp_body_limit_angular_velocity(PPBody *body, float speed);

#ifdef __cplusplus
}
#endif

#endif

#ifdef PICOPHYSICS_IMPLEMENTATION

#define EPA_DEBUG 0

#ifndef PICOPHYSICS_MAX_OBJECTS
#define PICOPHYSICS_MAX_OBJECTS 32
#endif

#ifndef PICOPHYSICS_MAX_TRIANGLES
#define PICOPHYSICS_MAX_TRIANGLES 128
#endif

#ifndef PICOPHYSICS_MAX_CONSTRAINTS
#define PICOPHYSICS_MAX_CONSTRAINTS 256
#endif

#ifndef PICOPHYSICS_MAX_MANIFOLDS
#define PICOPHYSICS_MAX_MANIFOLDS 256
#endif

static PPBody objects[PICOPHYSICS_MAX_OBJECTS];
static int object_count = 0;
static int dead_object_count = 0;
static PPConstraint constraints[PICOPHYSICS_MAX_CONSTRAINTS];
static int constraint_count = 0;
static int dead_constraint_count = 0;

/* This body is used to represent the entire tri-mesh. It's static,
 * has a mass of zero and an inv_mass of zero and so it should never change
 * position. The only thing to be careful of is that the position and rotation
 * is totally irrelevant for the simulation! Always take this into account when
 * working on the collision response code. */
static PPBody trimesh_body = {
    .is_alive = true,
};

static PPTriangle tris[PICOPHYSICS_MAX_TRIANGLES];
static int tri_count = 0;

static struct _PPCollisionMapEntry
{
    BodyKind kind1;
    BodyKind kind2;
    void *user_data;
    bool (*collision_callback)(
        const void *, const void *, BodyKind, BodyKind, const PPCollision *c, const void *);
} collision_map[32];

static int collision_map_count = 0;

static PPVec3 gravity = {.xyz = {0.0f, 0.0f, 0.0f}};
static float gravity_magnitude = 0.0f;

PPVec3 *pp_vec3_init(PPVec3 *v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
    return v;
}

PPVec3 *pp_vec3_set(PPVec3 *v, float x, float y, float z)
{
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
}

PPVec3 *pp_vec3_assign(PPVec3 *target, const PPVec3 *source)
{
    pp_vec3_set(target, source->x, source->y, source->z);
    return target;
}

static inline PPVec3 *pp_vec3_add(const PPVec3 *v1, const PPVec3 *v2, PPVec3 *out)
{
    out->x = v1->x + v2->x;
    out->y = v1->y + v2->y;
    out->z = v1->z + v2->z;
    return out;
}

static inline PPVec3 *pp_vec3_sub(const PPVec3 *v1, const PPVec3 *v2, PPVec3 *out)
{
    out->x = v1->x - v2->x;
    out->y = v1->y - v2->y;
    out->z = v1->z - v2->z;
    return out;
}

static inline PPVec3 *pp_vec3_neg(const PPVec3 *v1, PPVec3 *out)
{
    out->x = -v1->x;
    out->y = -v1->y;
    out->z = -v1->z;
    return out;
}

static inline float pp_vec3_length_sq(const PPVec3 *v1)
{
    return v1->x * v1->x + v1->y * v1->y + v1->z * v1->z;
}

static inline float pp_vec3_dist(const PPVec3 *v1, const PPVec3 *v2)
{
    PPVec3 tmp;
    pp_vec3_sub(v2, v1, &tmp);
    return pp_vec3_length(&tmp);
}

static inline float pp_vec3_dist_sq(const PPVec3 *v1, const PPVec3 *v2)
{
    PPVec3 tmp;
    pp_vec3_sub(v2, v1, &tmp);
    return pp_vec3_length_sq(&tmp);
}

static inline PPVec3 *pp_vec3_cross(const PPVec3 *v1, const PPVec3 *v2, PPVec3 *out)
{
    assert(v1 != out);
    assert(v2 != out);

    out->x = v1->y * v2->z - v1->z * v2->y;
    out->y = v1->z * v2->x - v1->x * v2->z;
    out->z = v1->x * v2->y - v1->y * v2->x;
    return out;
}

bool pp_vec3_normalize(PPVec3 *v)
{
    float length = pp_vec3_length(v);

    // Check for zero-length vector to avoid division by zero
    if (length > 0.0f) {
        float inv_length = 1.0f / length;
        v->x *= inv_length;
        v->y *= inv_length;
        v->z *= inv_length;
        return true;
    } else {
        pp_vec3_init(v);
        return false;
    }
}

void pp_mat3_inverse(const PPMat3 *in, PPMat3 *out)
{
    const float *m = in->m;

    float a = m[0], d = m[3], g = m[6];
    float b = m[1], e = m[4], h = m[7];
    float c = m[2], f = m[5], i = m[8];

    // Cofactors
    float A = (e * i - f * h);
    float B = -(d * i - f * g);
    float C = (d * h - e * g);
    float D = -(b * i - c * h);
    float E = (a * i - c * g);
    float F = -(a * h - b * g);
    float G = (b * f - c * e);
    float H = -(a * f - c * d);
    float I = (a * e - b * d);

    // Determinant
    float det = a * A + d * D + g * G;

    float inv_det = 1.0f / det;

    // Adjugate (still column-major)
    out->m[0] = A * inv_det;
    out->m[1] = D * inv_det;
    out->m[2] = G * inv_det;

    out->m[3] = B * inv_det;
    out->m[4] = E * inv_det;
    out->m[5] = H * inv_det;

    out->m[6] = C * inv_det;
    out->m[7] = F * inv_det;
    out->m[8] = I * inv_det;
}

void pp_mat3_mult(const PPMat3 *m, const PPVec3 *p, PPVec3 *pout)
{
    float x = p->x;
    float y = p->y;
    float z = p->z;

    pout->x = m->m[0] * x + m->m[3] * y + m->m[6] * z;
    pout->y = m->m[1] * x + m->m[4] * y + m->m[7] * z;
    pout->z = m->m[2] * x + m->m[5] * y + m->m[8] * z;
}

PPQuaternion *pp_quat_init(PPQuaternion *q)
{
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
    q->w = 1.0f;
    return q;
}

PPQuaternion *pp_quat_set(PPQuaternion *q, float x, float y, float z, float w)
{
    q->x = x;
    q->y = y;
    q->z = z;
    q->w = w;
    return q;
}

PPQuaternion *pp_quat_assign(PPQuaternion *target, const PPQuaternion *source)
{
    pp_quat_set(target, source->x, source->y, source->z, source->w);
    return target;
}

void pp_quat_from_angular_velocity(const PPVec3 *a_vel, float dt, PPQuaternion *q_rot)
{
    float angle = pp_vec3_length(a_vel) * dt; // Calculate the rotation angle
    if (angle > 0.0f) {
        PPVec3 axis;
        pp_vec3_assign(&axis, a_vel);
        pp_vec3_normalize(&axis); // Normalize the angular velocity to get the axis of rotation

        // Calculate sine and cosine of the half angle
        float sin_half_angle = sinf(angle / 2);
        float cos_half_angle = cosf(angle / 2);

        // Create the quaternion
        q_rot->x = axis.xyz[0] * sin_half_angle;
        q_rot->y = axis.xyz[1] * sin_half_angle;
        q_rot->z = axis.xyz[2] * sin_half_angle;
        q_rot->w = cos_half_angle;
    } else {
        // If there is no rotation
        pp_quat_init(q_rot);
    }
}

void pp_quat_multiply(const PPQuaternion *q1, const PPQuaternion *q2, PPQuaternion *result)
{
    PPQuaternion tmp;
    tmp.xyzw[0] = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    tmp.xyzw[1] = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    tmp.xyzw[2] = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    tmp.xyzw[3] = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;

    *result = tmp;
}

float pp_quat_angle_between(const PPQuaternion *q1, const PPQuaternion *q2)
{
    float dot = q1->w * q2->w + q1->x * q2->x + q1->y * q2->y + q1->z * q2->z;
    return acosf(dot) * 2.0f;
}

void pp_quat_transform(const PPQuaternion *q, const PPVec3 *v, PPVec3 *ret)
{
    PPQuaternion conjugate, vq, temp, v_rot_q;
    pp_quat_set(&conjugate, -q->x, -q->y, -q->z, q->w);
    pp_quat_set(&vq, v->x, v->y, v->z, 0.0f);
    pp_quat_multiply(q, &vq, &temp);
    pp_quat_multiply(&temp, &conjugate, &v_rot_q);
    pp_vec3_set(ret, v_rot_q.x, v_rot_q.y, v_rot_q.z);
}

void pp_quat_normalize(PPQuaternion *q)
{
    float norm = sqrtf(q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w);
    if (norm > 0) {
        float inv_norm = 1.0f / norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
        q->w *= inv_norm;
    }
}

static inline void pp_quat_conjugate(const PPQuaternion *q, PPQuaternion *out)
{
    out->x = -q->x;
    out->y = -q->y;
    out->z = -q->z;
    out->w = q->w;
}

void pp_quat_forward(const PPQuaternion *q, PPVec3 *out)
{
    float x = q->x;
    float y = q->y;
    float z = q->z;
    float w = q->w;

    out->x = 2.0f * (x * z + w * y);
    out->y = 2.0f * (y * z - w * x);
    out->z = 1.0f - 2.0f * (x * x + y * y);
}

void pp_quat_up(const PPQuaternion *q, PPVec3 *out)
{
    float x = q->x;
    float y = q->y;
    float z = q->z;
    float w = q->w;

    out->x = 2.0f * (x * y - w * z);
    out->y = 1.0f - 2.0f * (x * x + z * z);
    out->z = 2.0f * (y * z + w * x);
}

void pp_quat_right(const PPQuaternion *q, PPVec3 *out)
{
    float x = q->x;
    float y = q->y;
    float z = q->z;
    float w = q->w;

    out->x = 1.0f - 2.0f * (y * y + z * z);
    out->y = 2.0f * (x * y + w * z);
    out->z = 2.0f * (x * z - w * y);
}

void pp_quat_between(const PPVec3 *v0, const PPVec3 *v1, PPQuaternion *result)
{
    float dot = pp_vec3_dot(v0, v1);

    // If the vectors are exactly opposite, return a 180-degree rotation around an arbitrary axis
    if (dot < -1.0f + 1e-6f) {
        // Rotate around the Y axis
        result->x = 0.0f;
        result->y = 1.0f;
        result->z = 0.0f;
        result->w = 0.0f;
        return;
    }

    // If the vectors are exactly the same, return the identity quaternion
    if (dot > 1.0f - 1e-6f) {
        result->x = 0.0f;
        result->y = 0.0f;
        result->z = 0.0f;
        result->w = 1.0f;
        return;
    }

    // Calculate the axis of rotation
    PPVec3 axis;
    axis.xyz[0] = v0->y * v1->z - v0->z * v1->y;
    axis.xyz[1] = v0->z * v1->x - v0->x * v1->z;
    axis.xyz[2] = v0->x * v1->y - v0->y * v1->x;

    pp_vec3_normalize(&axis);

    // Calculate the angle of rotation
    float angle = acosf(dot);

    // Calculate the quaternion
    float half_angle = angle * 0.5f;
    float sin_half_angle = sinf(half_angle);
    result->x = axis.xyz[0] * sin_half_angle;
    result->y = axis.xyz[1] * sin_half_angle;
    result->z = axis.xyz[2] * sin_half_angle;
    result->w = cosf(half_angle);
}

void pp_quat_slerp(const PPQuaternion *q0, const PPQuaternion *q1, float t, PPQuaternion *result)
{
    float dot = q0->x * q1->x + q0->y * q1->y + q0->z * q1->z + q0->w * q1->w;

    PPQuaternion q1_temp;
    if (dot < 0.0f) {
        q1_temp.xyzw[0] = -q1->x;
        q1_temp.xyzw[1] = -q1->y;
        q1_temp.xyzw[2] = -q1->z;
        q1_temp.xyzw[3] = -q1->w;
        dot = -dot;
    } else {
        q1_temp = *q1;
    }

    // If the quaternions are very close, use linear interpolation
    if (dot > 0.9995f) {
        result->x = q0->x + t * (q1_temp.xyzw[0] - q0->x);
        result->y = q0->y + t * (q1_temp.xyzw[1] - q0->y);
        result->z = q0->z + t * (q1_temp.xyzw[2] - q0->z);
        result->w = q0->w + t * (q1_temp.xyzw[3] - q0->w);
        return;
    }

    // Calculate the angle between the quaternions
    float theta = acosf(dot);

    // Calculate the coefficients for spherical linear interpolation
    float sin_theta = sinf(theta);
    float s0 = sinf((1.0f - t) * theta) / sin_theta;
    float s1 = sinf(t * theta) / sin_theta;

    // Perform the interpolation
    result->x = s0 * q0->x + s1 * q1_temp.xyzw[0];
    result->y = s0 * q0->y + s1 * q1_temp.xyzw[1];
    result->z = s0 * q0->z + s1 * q1_temp.xyzw[2];
    result->w = s0 * q0->w + s1 * q1_temp.xyzw[3];
}

bool pp_box_intersect(
    const PPVec3 *box_pos, const PPQuaternion *box_rot, const PPVec3 *whd, const PPVec3 *o, const PPVec3 *d, PPVec3 *out, float *distance);
bool pp_sphere_intersect(
    const PPVec3 *sphere_pos, float radius, const PPVec3 *o, const PPVec3 *d, PPVec3 *out, float *distance);
bool pp_tri_intersect(
    const PPTriangle *tri, const PPVec3 *o, const PPVec3 *d, PPVec3 *out, float *distance);

bool pp_contains_kind(BodyKind *kinds, BodyKind kind)
{
    if (!kinds) {
        return false;
    }

    BodyKind *k = kinds;
    while (*k) {
        if (*k == kind) {
            return true;
        }
        k++;
    }

    return false;
}

/**
 * Intersects the world with the specified ray. Returns true if something was hit.
 *
 * If a hit was detected, then either sphere_hit or tri_hit will be populated (depending on what was hit) and the distance from the
 * origin to the hit will be returned.
 *
 * ignore_kinds is an array of kinds to ignore, with a 0 terminated final entry.
 */
bool pp_physics_ray_intersect(const PPVec3 *origin,
                              const PPVec3 *direction,
                              BodyKind *ignore_kinds,
                              const PPBody **body_hit,
                              const PPTriangle **tri_hit,
                              float *distance,
                              PPVec3 *intersection)
{
    float closest_dist = FLT_MAX;
    const PPTriangle *closest_tri = NULL;
    const PPBody *closest_body = NULL;
    PPVec3 closest_intersection = {.xyz={0.0f, 0.0f, 0.0f}};

    for (size_t i = 0; i < pp_physics_triangle_count(); ++i) {
        const PPTriangle *t = pp_physics_triangle_at(i);

        PPVec3 hit;
        float dist;
        if (pp_tri_intersect(t, origin, direction, &hit, &dist)) {
            if (pp_contains_kind(ignore_kinds, t->kind)) {
                continue;
            }

            if (dist < closest_dist) {
                closest_tri = t;
                closest_dist = dist;
                pp_vec3_assign(&closest_intersection, &hit);
            }
        }
    }

    for (size_t i = 0; i < pp_physics_body_total_count(); ++i) {
        const PPBody *body = pp_physics_body_at(i);
        if (!body->is_alive) {
            continue;
        }

        for (int si = 0; si < body->shape_count; ++si) {
            const PPShape *shape = &body->shapes[si];
            PPVec3 shape_world_pos;
            PPVec3 rotated_offset;
            pp_quat_transform(&body->rot, &shape->offset, &rotated_offset);
            pp_vec3_add(&body->pos, &rotated_offset, &shape_world_pos);

            if (pp_contains_kind(ignore_kinds, shape->kind)) {
                continue;
            }

            PPVec3 hit;
            float dist;
            bool did_hit = false;

            if (shape->type == PP_OBJECT_TYPE_SPHERE) {
                did_hit = pp_sphere_intersect(&shape_world_pos, shape->sphere.radius, origin, direction, &hit, &dist);
            } else if (shape->type == PP_OBJECT_TYPE_BOX) {
                did_hit = pp_box_intersect(&shape_world_pos, &body->rot, &shape->box.whd, origin, direction, &hit, &dist);
            }

            if (did_hit && dist < closest_dist) {
                closest_body = body;
                closest_tri = NULL;
                closest_dist = dist;
                pp_vec3_assign(&closest_intersection, &hit);
            }
        }
    }

    if (closest_dist == FLT_MAX) {
        return false;
    }

    if (body_hit) {
        *body_hit = closest_body;
    }

    if (tri_hit) {
        *tri_hit = (PPTriangle *) closest_tri;
    }
    if (distance) {
        *distance = closest_dist;
    }
    if (intersection) {
        pp_vec3_assign(intersection, &closest_intersection);
    }
    return true;
}

bool pp_aabb_intersect(const PPVec3 *pos,
                       const PPVec3 *whd,
                       const PPVec3 *origin,
                       const PPVec3 *direction,
                       PPVec3 *out,
                       float *distance)
{
    PPVec3 extents = {.xyz={whd->x * 0.5f, whd->y * 0.5f, whd->z * 0.5f}};
    PPVec3 min = {.xyz={pos->x - extents.x, pos->y - extents.y, pos->z - extents.z}};
    PPVec3 max = {.xyz={pos->x + extents.x, pos->y + extents.y, pos->z + extents.z}};
    PPVec3 n_inv = {.xyz={1.0f / direction->x, 1.0f / direction->y, 1.0f / direction->z}};

    const float t1 = (min.x - origin->x) * n_inv.x;
    const float t2 = (max.x - origin->x) * n_inv.x;
    const float t3 = (min.y - origin->y) * n_inv.y;
    const float t4 = (max.y - origin->y) * n_inv.y;
    const float t5 = (min.z - origin->z) * n_inv.z;
    const float t6 = (max.z - origin->z) * n_inv.z;

    const float tmin = fmaxf(fmaxf(fminf(t1, t2), fminf(t3, t4)), fminf(t5, t6));
    const float tmax = fminf(fminf(fmaxf(t1, t2), fmaxf(t3, t4)), fmaxf(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behind us
    if (tmax < 0) {
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax) {
        return false;
    }

    *distance = tmin;
    pp_vec3_scale(direction, tmin, out);
    pp_vec3_add(origin, out, out);
    return true;
}

bool pp_box_intersect(
    const PPVec3 *box_pos, const PPQuaternion *box_rot, const PPVec3 *whd, const PPVec3 *o, const PPVec3 *d, PPVec3 *out, float *distance)
{
    return pp_aabb_intersect(box_pos, whd, o, d, out, distance);
}

bool pp_sphere_intersect(
    const PPVec3 *sphere_pos, float radius, const PPVec3 *o, const PPVec3 *d, PPVec3 *out, float *distance)
{
    PPVec3 oc;
    pp_vec3_sub(sphere_pos, o, &oc);

    float b = pp_vec3_dot(&oc, d);
    float c = pp_vec3_dot(&oc, &oc) - radius * radius;

    float discriminant = b * b - c;

    // No matter what, always init the distance
    if (distance) {
        *distance = FLT_MAX;
    }

    // If the discriminant is negative, there are no real roots, so no intersection
    if (discriminant < 0) {
        return false;
    }

    // Calculate the two points of intersection. oc points from the ray origin
    // to the sphere centre, so b = oc·d is positive when the sphere is ahead of
    // the origin and the roots are t = b ± sqrt(disc). (Using -b here would only
    // ever return hits for spheres *behind* the ray.)
    float sqrtDiscriminant = sqrtf(discriminant);
    float t1 = b - sqrtDiscriminant;
    float t2 = b + sqrtDiscriminant;

    // Check if the points of intersection are in front of the ray origin
    if (t1 > 0 && (t2 <= 0 || t1 < t2)) {
        PPVec3 add;
        pp_vec3_scale(d, t1, &add);
        pp_vec3_add(o, &add, out);
        if (distance) {
            *distance = t1;
        }
        return true;
    } else if (t2 > 0) {
        PPVec3 add;
        pp_vec3_scale(d, t2, &add);
        pp_vec3_add(o, &add, out);
        if (distance) {
            *distance = t2;
        }
        return true;
    }

    return false;
}

bool pp_tri_intersect(
    const PPTriangle *tri, const PPVec3 *o, const PPVec3 *d, PPVec3 *out, float *distance)
{
    const float e = FLT_EPSILON;

    // Möller-Trumbore ray-triangle intersection, optimized with direct arithmetic
    float edge1_x = tri->v[1].x - tri->v[0].x;
    float edge1_y = tri->v[1].y - tri->v[0].y;
    float edge1_z = tri->v[1].z - tri->v[0].z;
    float edge2_x = tri->v[2].x - tri->v[0].x;
    float edge2_y = tri->v[2].y - tri->v[0].y;
    float edge2_z = tri->v[2].z - tri->v[0].z;

    // cross_e2 = d × edge2
    float cross_e2_x = d->y * edge2_z - d->z * edge2_y;
    float cross_e2_y = d->z * edge2_x - d->x * edge2_z;
    float cross_e2_z = d->x * edge2_y - d->y * edge2_x;

    // det = edge1 · cross_e2
    float det = edge1_x * cross_e2_x + edge1_y * cross_e2_y + edge1_z * cross_e2_z;

    if (det > -e && det < e) {
        return false;
    }

    float inv_det = 1.0f / det;

    // s = o - v0
    float s_x = o->x - tri->v[0].x;
    float s_y = o->y - tri->v[0].y;
    float s_z = o->z - tri->v[0].z;

    float u = inv_det * (s_x * cross_e2_x + s_y * cross_e2_y + s_z * cross_e2_z);
    if ((u < 0.0f && -u > e) || (u > 1.0f && fabsf(u - 1.0f) > e)) {
        return false;
    }

    // cross_e1 = s × edge1
    float cross_e1_x = s_y * edge1_z - s_z * edge1_y;
    float cross_e1_y = s_z * edge1_x - s_x * edge1_z;
    float cross_e1_z = s_x * edge1_y - s_y * edge1_x;

    float v = inv_det * (d->x * cross_e1_x + d->y * cross_e1_y + d->z * cross_e1_z);
    if ((v < 0.0f && -v > e) || (u + v > 1.0f && fabsf(u + v - 1.0f) > e)) {
        return false;
    }

    float t = inv_det * (edge2_x * cross_e1_x + edge2_y * cross_e1_y + edge2_z * cross_e1_z);
    if (t <= e) {
        return false;
    }

    if (out) {
        out->x = o->x + d->x * t;
        out->y = o->y + d->y * t;
        out->z = o->z + d->z * t;
    }

    if (distance) {
        *distance = t;
    }

    return true;
}

void pp_body_set_kind(PPBody *b, BodyKind kind)
{
    b->kind = kind;
    for (int i = 0; i < b->shape_count; ++i) {
        b->shapes[i].kind = kind;
    }
}

BodyKind pp_body_get_kind(const PPBody *b)
{
    return b->kind;
}

void pp_body_set_angular_velocity(PPBody *s, float x, float y, float z)
{
    pp_vec3_set(&s->a_vel, x, y, z);
}

void pp_body_set_velocity(PPBody *s, float x, float y, float z)
{
    pp_vec3_set(&s->vel, x, y, z);
}

void pp_body_set_angular_acceleration(PPBody *s, float x, float y, float z)
{
    pp_vec3_set(&s->a_acc, x, y, z);
}

void pp_body_set_acceleration(PPBody *s, float x, float y, float z)
{
    pp_vec3_set(&s->acc, x, y, z);
}

void pp_body_set_gravity_multiplier(PPBody *b, float multiplier) {
    b->gravity_multiplier = multiplier;
}

float pp_body_get_gravity_multiplier(const PPBody *b) {
    return b->gravity_multiplier;
}

void pp_body_set_angular_damping(PPBody *s, float d)
{
    if (d < 0.0f || d > 1.0f) {
        return;
    }

    s->a_damping = d;
}

void pp_body_set_damping(PPBody *s, float d)
{
    if (d < 0.0f || d > 1.0f) {
        return;
    }

    s->damping = d;
}

static void pp_body_recompute_mass_inertia(PPBody *body);

void pp_body_set_mass(PPBody* b, float mass) {
    // Scale all shape masses proportionally
    if (b->mass > 0.0f && b->shape_count > 0) {
        float ratio = mass / b->mass;
        for (int i = 0; i < b->shape_count; ++i) {
            b->shapes[i].mass *= ratio;
        }
        pp_body_recompute_mass_inertia(b);
    } else {
        b->mass = mass;
        b->inv_mass = (mass == 0.0f) ? 0.0f : 1.0f / mass;
    }
}

float pp_body_get_mass(const PPBody* b) {
    return b->mass;
}

void pp_body_get_forward(PPBody *s, PPVec3 *f)
{
    pp_quat_forward(&s->rot, f);
}

void pp_body_get_up(PPBody *s, PPVec3 *u)
{
    pp_quat_up(&s->rot, u);
}

void pp_body_get_right(PPBody *s, PPVec3 *r)
{
    pp_quat_right(&s->rot, r);
}

size_t pp_body_get_shape_count(const PPBody *body)
{
    return body->shape_count;
}

const PPShape *pp_body_get_shape(const PPBody *body, size_t index)
{
    assert(index < (size_t)body->shape_count);
    return &body->shapes[index];
}

size_t pp_body_get_constraint_count(PPBody *body) {
    size_t count = 0;
    for(int i = 0; i < constraint_count; ++i) {
        if(!constraints[i].is_alive) {
            continue;
        }

        if (constraints[i].body1 == body || constraints[i].body2 == body) {
            count++;
        }
    }

    return count;
}

float pp_body_get_radius(PPBody *s)
{
    float max_r = 0.0f;
    for (int i = 0; i < s->shape_count; ++i) {
        const PPShape *shape = &s->shapes[i];
        float offset_len = pp_vec3_length(&shape->offset);
        float r = 0.0f;
        if (shape->type == PP_OBJECT_TYPE_SPHERE) {
            r = shape->sphere.radius + offset_len;
        } else if (shape->type == PP_OBJECT_TYPE_BOX) {
            r = shape->box.bounding_radius + offset_len;
        }
        if (r > max_r) max_r = r;
    }
    return max_r;
}

void pp_quat_from_axis_angle(PPQuaternion *q, const PPVec3 *axis, float angle)
{
    PPVec3 a;
    pp_vec3_assign(&a, axis);
    pp_vec3_normalize(&a);

    float half = angle * 0.5f;
    float s = sinf(half); // sin(θ/2)
    float c = cosf(half); // cos(θ/2)

    q->x = a.xyz[0] * s; // axis.x * sin(θ/2)
    q->y = a.xyz[1] * s; // axis.y * sin(θ/2)
    q->z = a.xyz[2] * s; // axis.z * sin(θ/2)
    q->w = c;            // cos(θ/2)

    pp_quat_normalize(q);
}

void pp_body_look_at(PPBody *s, float x, float y, float z)
{
    PPVec3 t, f, c;
    pp_vec3_set(&t, x, y, z);
    pp_body_get_forward(s, &f);
    pp_vec3_cross(&f, &t, &c);
    float d = pp_vec3_dot(&f, &t);

    // Build error quaternion
    PPQuaternion q_err;
    if (d < -0.9999f) { // opposite direction
        PPVec3 ortho, axis;
        if (fabsf(f.xyz[0]) < 0.9f) {
            pp_vec3_set(&ortho, 1, 0, 0);
        } else {
            pp_vec3_set(&ortho, 0, 1, 0);
        }

        pp_vec3_cross(&f, &ortho, &axis);
        pp_vec3_normalize(&axis);

        // 180° rotation
        pp_quat_from_axis_angle(&q_err, &axis, M_PI);
    } else {
        PPVec3 axis;
        float s = sqrtf((1.0f + d) * 2.0f);
        axis.xyz[0] = c.xyz[0] / s;
        axis.xyz[1] = c.xyz[1] / s;
        axis.xyz[2] = c.xyz[2] / s;

        pp_quat_set(&q_err, axis.xyz[0], axis.xyz[1], axis.xyz[2], s * 0.5f);
    }

    pp_quat_normalize(&q_err);

    // Angular error vector (approximation)
    PPVec3 error;
    pp_vec3_set(&error, 2.0f * q_err.xyzw[0], 2.0f * q_err.xyzw[1], 2.0f * q_err.xyzw[2]);

    PPVec3 torque, a, b;

    // Configurable
    float kp = 10.0f;
    float kd = 1.0f;

    // Vector3 torque = -cfg.kp * error - cfg.kd * a_vel;
    pp_vec3_scale(&error, -kp, &a);
    pp_vec3_scale(&s->a_vel, kd, &b);
    pp_vec3_sub(&a, &b, &torque);
    pp_body_add_angular_force(s, torque.xyz[0], torque.xyz[1], torque.xyz[2]);
}

void pp_body_limit_velocity(PPBody *body, float speed)
{
    if (speed < 0.0f) {
        return;
    }

    body->vel_limit = speed;
}

void pp_body_limit_angular_velocity(PPBody *body, float speed)
{
    if (speed < 0.0f) {
        return;
    }

    body->a_vel_limit = speed;
}

static void pp_body_init(PPBody *body, const PPVec3 *pos, float mass, BodyKind kind)
{
    memset(body, 0, sizeof(PPBody));
    body->is_alive = true;
    body->user_data = NULL;
    body->shape_count = 0;

    pp_vec3_init(&body->vel);
    pp_vec3_init(&body->acc);
    pp_vec3_init(&body->a_vel);
    pp_vec3_init(&body->a_acc);
    pp_quat_init(&body->rot);

    if (pos) {
        pp_vec3_set(&body->pos, pos->x, pos->y, pos->z);
    } else {
        pp_vec3_set(&body->pos, 0.0f, 0.0f, 0.0f);
    }

    body->kind = kind;
    body->mass = mass;
    body->inv_mass = (mass == 0.0f) ? 0.0f : 1.0f / mass;
    body->friction = 0.75f;
    body->damping = 0.01f;
    body->a_damping = 0.02f;
    body->vel_limit = 0.0f;
    body->a_vel_limit = 0.0f;
    body->gravity_multiplier = 1.0f;
    pp_body_set_bounce(body, 0.5f);
}

static void pp_body_recompute_mass_inertia(PPBody *body)
{
    float total_mass = 0.0f;
    memset(body->inertia.m, 0, sizeof(body->inertia.m));

    for (int i = 0; i < body->shape_count; ++i) {
        const PPShape *s = &body->shapes[i];
        float m = s->mass;
        total_mass += m;

        if (m <= 0.0f) continue;

        // Compute local inertia for this shape
        float Ixx = 0.0f, Iyy = 0.0f, Izz = 0.0f;
        if (s->type == PP_OBJECT_TYPE_SPHERE) {
            float r2 = s->sphere.radius * s->sphere.radius;
            Ixx = Iyy = Izz = (2.0f / 5.0f) * m * r2;
        } else if (s->type == PP_OBJECT_TYPE_BOX) {
            float w2 = s->box.whd.x * s->box.whd.x;
            float h2 = s->box.whd.y * s->box.whd.y;
            float d2 = s->box.whd.z * s->box.whd.z;
            const float oot = 1.0f / 12.0f;
            Ixx = oot * m * (h2 + d2);
            Iyy = oot * m * (w2 + d2);
            Izz = oot * m * (w2 + h2);
        }

        // Parallel axis theorem: I_total += I_local + m * (d·d * I3 - outer(d,d))
        float dx = s->offset.x, dy = s->offset.y, dz = s->offset.z;
        float d_dot_d = dx*dx + dy*dy + dz*dz;

        body->inertia.m[0] += Ixx + m * (d_dot_d - dx*dx);
        body->inertia.m[4] += Iyy + m * (d_dot_d - dy*dy);
        body->inertia.m[8] += Izz + m * (d_dot_d - dz*dz);
        body->inertia.m[1] += -m * dx * dy;
        body->inertia.m[3] += -m * dx * dy;
        body->inertia.m[2] += -m * dx * dz;
        body->inertia.m[6] += -m * dx * dz;
        body->inertia.m[5] += -m * dy * dz;
        body->inertia.m[7] += -m * dy * dz;
    }

    body->mass = total_mass;
    body->inv_mass = (total_mass == 0.0f) ? 0.0f : 1.0f / total_mass;

    if (total_mass > 0.0f) {
        pp_mat3_inverse(&body->inertia, &body->inv_inertia);
    } else {
        memset(body->inertia.m, 0, sizeof(body->inertia.m));
        memset(body->inv_inertia.m, 0, sizeof(body->inv_inertia.m));
    }
}

PPShape *pp_body_add_sphere(PPBody *body, float radius, float mass, BodyKind kind, float offset_x, float offset_y, float offset_z)
{
    assert(body->shape_count < PP_MAX_SHAPES_PER_BODY);
    PPShape *s = &body->shapes[body->shape_count++];
    memset(s, 0, sizeof(PPShape));
    s->type = PP_OBJECT_TYPE_SPHERE;
    s->sphere.radius = radius;
    s->mass = mass;
    s->kind = kind;
    pp_vec3_set(&s->offset, offset_x, offset_y, offset_z);

    // Update body kind to match first shape if not yet set
    if (body->shape_count == 1) {
        body->kind = kind;
    }

    pp_body_recompute_mass_inertia(body);
    return s;
}

PPShape *pp_body_add_box(PPBody *body, float w, float h, float d, float mass, BodyKind kind, float offset_x, float offset_y, float offset_z)
{
    assert(body->shape_count < PP_MAX_SHAPES_PER_BODY);
    PPShape *s = &body->shapes[body->shape_count++];
    memset(s, 0, sizeof(PPShape));
    s->type = PP_OBJECT_TYPE_BOX;
    pp_vec3_set(&s->box.whd, w, h, d);
    pp_vec3_set(&s->box.half_extents, w * 0.5f, h * 0.5f, d * 0.5f);
    s->box.bounding_radius = pp_vec3_length(&s->box.whd);
    s->mass = mass;
    s->kind = kind;
    pp_vec3_set(&s->offset, offset_x, offset_y, offset_z);

    if (body->shape_count == 1) {
        body->kind = kind;
    }

    pp_body_recompute_mass_inertia(body);
    return s;
}

void pp_body_set_user_data(PPBody *s, void *data)
{
    s->user_data = data;
}

void *pp_body_get_user_data(const PPBody *s)
{
    return s->user_data;
}

void pp_body_lock_axis(PPBody *s, PPAxisLock lock)
{
    s->lock = lock;
}

void pp_body_get_position(PPBody *s, PPVec3 *pos)
{
    pp_vec3_assign(pos, &s->pos);
}

void pp_body_set_position(PPBody *s, float x, float y, float z)
{
    pp_vec3_set(&s->pos, x, y, z);
}

void pp_body_set_rotation(PPBody *s, float x, float y, float z, float w)
{
    pp_quat_set(&s->rot, x, y, z, w);
}

void pp_body_get_velocity_at_position(const PPBody *b, const PPVec3 *p, PPVec3 *ret)
{
    PPVec3 rel_pos, a_vel_contrib;
    pp_vec3_sub(p, &b->pos, &rel_pos);
    pp_vec3_cross(&b->a_vel, &rel_pos, &a_vel_contrib);
    pp_vec3_add(&b->vel, &a_vel_contrib, ret);
}

void pp_body_get_velocity(const PPBody *s, PPVec3 *vel)
{
    pp_vec3_assign(vel, &s->vel);
}

void pp_body_get_rotation(PPBody *s, PPQuaternion *rot)
{
    pp_quat_assign(rot, &s->rot);
}

bool pp_body_set_friction(PPBody *s, float f)
{
    if (!s || f < 0.0f || f > 1.0f) {
        return false;
    }

    s->friction = f;
    return true;
}

bool pp_body_set_bounce(PPBody *s, float b)
{
    if (!s || b < 0.0f || b > 1.0f) {
        return false;
    }

    s->bounce = b;
    return true;
}

void pp_body_add_force(PPBody *s, float x, float y, float z)
{
    if(!s) {
        return;
    }

    PPVec3 force;
    pp_vec3_set(&force, x, y, z);

    PPVec3 acceleration;

    // Ensure you do not divide by zero
    if (s->mass > 0) {
        // a = F / m
        pp_vec3_scale(&force, 1.0f / s->mass, &acceleration);

        // Add acceleration to the sphere's current acceleration
        pp_vec3_add(&s->acc, &acceleration, &s->acc);
    }
}

void pp_body_add_angular_force(PPBody *s, float tx, float ty, float tz)
{
    PPVec3 torque;
    pp_vec3_set(&torque, tx, ty, tz);

    // FIXME: Implement for boxes!
    // Simplified inertia for Spheres
    float I = s->inertia.m[0];

    if (I <= 0.0f)
        return; // nothing to do for mass‑less or zero‑radius objects

    PPVec3 ang_acc;
    pp_vec3_scale(&torque, 1.0f / I, &ang_acc);
    pp_vec3_add(&s->a_acc, &ang_acc, &s->a_acc);
}

void pp_body_add_force_at_position(PPBody *b, const PPVec3 *world_pos, const PPVec3 *force)
{
    // Apply linear force
    pp_body_add_force(b, force->x, force->y, force->z);

    // Calculate torque from offset position
    // torque = (position - center_of_mass) × force
    PPVec3 rel_pos;
    pp_vec3_sub(world_pos, &b->pos, &rel_pos);

    PPVec3 torque;
    pp_vec3_cross(&rel_pos, force, &torque);

    // Apply angular force (torque)
    pp_body_add_angular_force(b, torque.x, torque.y, torque.z);
}

const struct _PPCollisionMapEntry *pp_physics_collision_map_search(BodyKind kind1, BodyKind kind2)
{
    for (int i = 0; i < collision_map_count; ++i) {
        struct _PPCollisionMapEntry *entry = &collision_map[i];
        if ((entry->kind1 == kind1 && entry->kind2 == kind2)
            || (entry->kind2 == kind1 && entry->kind1 == kind2)) {
            return entry;
        }
    }

    return NULL;
}

bool pp_physics_collision_map_add(BodyKind kind1,
                                  BodyKind kind2,
                                  void *user_data,
                                  bool (*callback)(const void *,
                                                   const void *,
                                                   BodyKind,
                                                   BodyKind,
                                                   const PPCollision *c,
                                                   const void *))
{
    if (!pp_physics_collision_map_search(kind1, kind2)) {
        struct _PPCollisionMapEntry *entry = &collision_map[collision_map_count++];
        entry->kind1 = kind1;
        entry->kind2 = kind2;
        entry->user_data = user_data;
        entry->collision_callback = callback;
        return true;
    }

    return false;
}

static inline void pp_fill_collision_info_sphere_box(PPBody *lhs_body,
                                       const PPShape *lhs_shape,
                                       PPBody *rhs_body,
                                       const PPShape *rhs_shape,
                                       const PPVec3 *contact_point,
                                       const PPVec3 *n,
                                       PPCollision *c,
                                       float d)
{
    c->dist = d;
    c->separation = -d; // only reported while overlapping, so never speculative
    c->p.x = contact_point->x; c->p.y = contact_point->y; c->p.z = contact_point->z;
    c->n.x = n->x; c->n.y = n->y; c->n.z = n->z;
    c->obj1 = lhs_body;
    c->obj2 = rhs_body;
    c->obj1_bounce = lhs_body->bounce;
    c->obj2_bounce = rhs_body->bounce;
    c->obj1_friction = lhs_body->friction;
    c->obj2_friction = rhs_body->friction;
    c->type1 = PP_OBJECT_TYPE_SPHERE;
    c->type2 = PP_OBJECT_TYPE_BOX;
    c->kind1 = lhs_shape->kind;
    c->kind2 = rhs_shape->kind;
}

static inline void pp_fill_collision_info_sphere_sphere(PPBody *lhs_body,
                                          const PPShape *lhs_shape,
                                          const PPVec3 *lhs_pos,
                                          PPBody *rhs_body,
                                          const PPShape *rhs_shape,
                                          const PPVec3 *rhs_pos,
                                          float dist,
                                          PPCollision *c)
{
    float total_radius = lhs_shape->sphere.radius + rhs_shape->sphere.radius;
    c->n.x = rhs_pos->x - lhs_pos->x;
    c->n.y = rhs_pos->y - lhs_pos->y;
    c->n.z = rhs_pos->z - lhs_pos->z;
    c->dist = dist - total_radius;
    c->separation = dist - total_radius; // only reported while overlapping
    if (dist > 0) {
        float inv = 1.0f / dist;
        c->n.x *= inv; c->n.y *= inv; c->n.z *= inv;
    }

    float wr1 = lhs_shape->sphere.radius / total_radius;
    float wr2 = rhs_shape->sphere.radius / total_radius;

    c->p.x = lhs_pos->x * wr1 + rhs_pos->x * wr2;
    c->p.y = lhs_pos->y * wr1 + rhs_pos->y * wr2;
    c->p.z = lhs_pos->z * wr1 + rhs_pos->z * wr2;

    c->obj1 = lhs_body;
    c->obj2 = rhs_body;
    c->obj1_bounce = lhs_body->bounce;
    c->obj2_bounce = rhs_body->bounce;
    c->obj1_friction = lhs_body->friction;
    c->obj2_friction = rhs_body->friction;
    c->type1 = PP_OBJECT_TYPE_SPHERE;
    c->type2 = PP_OBJECT_TYPE_SPHERE;
    c->kind1 = lhs_shape->kind;
    c->kind2 = rhs_shape->kind;
}

static inline void pp_fill_collision_info_sphere_triangle(
    PPBody *lhs_body, const PPShape *lhs_shape, const PPTriangle *tri, const PPVec3 *p, float dist, PPCollision *c)
{
    c->n.x = -tri->n.x;
    c->n.y = -tri->n.y;
    c->n.z = -tri->n.z;
    c->p.x = p->x;
    c->p.y = p->y;
    c->p.z = p->z;
    c->dist = dist;
    c->separation = -dist; // dist is penetration (radius - perp distance)
    c->obj1 = lhs_body;
    c->obj2 = &trimesh_body;
    c->obj1_bounce = lhs_body->bounce;
    c->obj2_bounce = 0.0f;
    c->obj1_friction = lhs_body->friction;
    c->obj2_friction = tri->friction;
    c->type1 = PP_OBJECT_TYPE_SPHERE;
    c->type2 = PP_OBJECT_TYPE_TRIANGLE;
    c->kind1 = lhs_shape->kind;
    c->kind2 = tri->kind;
}

PPBody *pp_physics_create_body(const PPVec3 *pos)
{
    PPBody *ret = NULL;

    if (dead_object_count) {
        for (int i = 0; i < object_count; ++i) {
            PPBody *body = &objects[i];
            if (!body->is_alive) {
                dead_object_count--;
                ret = body;
                break;
            }
        }
    }

    if (!ret) {
        ret = &objects[object_count++];
    }

    pp_body_init(ret, pos, 0.0f, 0);
    return ret;
}

PPBody *pp_physics_create_sphere(float radius, const PPVec3 *pos, float mass, BodyKind kind)
{
    PPBody *body = pp_physics_create_body(pos);
    pp_body_add_sphere(body, radius, mass, kind, 0.0f, 0.0f, 0.0f);
    return body;
}

PPConstraint* pp_physics_create_fixed_distance_constraint(PPBody* body1, PPBody* body2, float distance) {
    PPConstraint* entry = NULL;

    if(dead_constraint_count) {
        for (int i = 0; i < constraint_count; ++i) {
            PPConstraint* c = &constraints[i];
            if (!c->is_alive) {
                dead_constraint_count--;
                entry = c;
                break;
            }
        }
    } else {
        entry = &constraints[constraint_count++];
    }

    entry->is_alive = true;
    entry->body1 = body1;
    entry->body2 = body2;
    entry->type = PP_CONSTRAINT_TYPE_FIXED_DISTANCE;
    entry->fixed_distance.distance = distance;
    entry->inv_mass_sum = body1->inv_mass + body2->inv_mass;
    return entry;
}

PPBody *pp_physics_create_box(
    float width, float height, float depth, const PPVec3 *pos, float mass, BodyKind kind)
{
    PPBody *body = pp_physics_create_body(pos);
    pp_body_add_box(body, width, height, depth, mass, kind, 0.0f, 0.0f, 0.0f);
    return body;
}

PPTriangle *pp_physics_create_triangle(const PPVec3 *v1,
                                       const PPVec3 *v2,
                                       const PPVec3 *v3,
                                       BodyKind kind)
{
    PPTriangle *tri = &tris[tri_count++];
    pp_vec3_assign(&tri->v[0], v1);
    pp_vec3_assign(&tri->v[1], v2);
    pp_vec3_assign(&tri->v[2], v3);

    PPVec3 e1, e2;
    pp_vec3_sub(v2, v1, &e1);
    pp_vec3_sub(v3, v1, &e2);

    pp_vec3_cross(&e1, &e2, &tri->n);
    pp_vec3_normalize(&tri->n);

    tri->bounce = 0.0f;
    tri->friction = 0.9f;
    tri->kind = kind;

    // Precompute AABB for fast sphere culling
    tri->aabb_min_x = fminf(v1->x, fminf(v2->x, v3->x));
    tri->aabb_min_y = fminf(v1->y, fminf(v2->y, v3->y));
    tri->aabb_min_z = fminf(v1->z, fminf(v2->z, v3->z));
    tri->aabb_max_x = fmaxf(v1->x, fmaxf(v2->x, v3->x));
    tri->aabb_max_y = fmaxf(v1->y, fmaxf(v2->y, v3->y));
    tri->aabb_max_z = fmaxf(v1->z, fmaxf(v2->z, v3->z));

    return tri;
}

void pp_physics_clear()
{
    tri_count = 0;
    object_count = 0;
    dead_object_count = 0;
    constraint_count = 0;
    dead_constraint_count = 0;
    memset(tris, 0, sizeof(tris));
    memset(objects, 0, sizeof(objects));
    memset(constraints, 0, sizeof(constraints));
}

size_t pp_physics_triangle_count()
{
    return tri_count;
}

size_t pp_physics_body_count()
{
    return object_count - dead_object_count;
}

size_t pp_physics_body_total_count()
{
    return object_count;
}

const PPBody *pp_physics_body_at(size_t i)
{
    return &objects[i];
}

const PPTriangle *pp_physics_triangle_at(size_t i)
{
    return tris + i;
}

void pp_physics_destroy_body(PPBody *b)
{
    if (!b) {
        return;
    }

    // Destroy any constraints
    for(int i = 0; i < constraint_count; ++i) {
        PPConstraint* c = &constraints[i];
        if(c->is_alive && (c->body1 == b || c->body2 == b)) {
            c->is_alive = false;
            ++dead_constraint_count;
        }
    }

    b->is_alive = false;
    ++dead_object_count;
}

void pp_physics_set_gravity(const PPVec3 *v)
{
    pp_vec3_assign(&gravity, v);
    gravity_magnitude = pp_vec3_length(&gravity);
}

static void pp_integrate_forces(float t)
{
    PPVec3 scaled_vel;
    // Apply acceleration to velocity
    for (int i = 0; i < object_count; ++i) {
        PPBody *body = &objects[i];

        if (!body->is_alive || body->inv_mass == 0.0f) {
            continue;
        }

        // Apply gravity to acceleration before applying acceleration
        // to velocity
        PPVec3 total_acc, grv;
        pp_vec3_scale(&gravity, body->gravity_multiplier, &grv);
        pp_vec3_add(&body->acc, &grv, &total_acc);

        pp_vec3_scale(&total_acc, t, &scaled_vel);
        pp_vec3_add(&body->vel, &scaled_vel, &body->vel);
        pp_vec3_init(&body->acc);

        // Apply linear damping (linear approximation of exp(-x) ≈ 1-x for small x)
        float damp = 1.0f - body->damping * t;
        pp_vec3_scale(&body->vel, damp, &body->vel);

        if (body->lock) {
            if ((body->lock & PP_AXIS_LOCK_PITCH) == PP_AXIS_LOCK_PITCH) {
                body->a_acc.xyz[0] = 0.0f;
                body->a_vel.xyz[0] = 0.0f;
            }

            if ((body->lock & PP_AXIS_LOCK_YAW) == PP_AXIS_LOCK_YAW) {
                body->a_acc.xyz[1] = 0.0f;
                body->a_vel.xyz[1] = 0.0f;
            }

            if ((body->lock & PP_AXIS_LOCK_ROLL) == PP_AXIS_LOCK_ROLL) {
                body->a_acc.xyz[2] = 0.0f;
                body->a_vel.xyz[2] = 0.0f;
            }
        }

        PPVec3 scaled_ang_vel; // Temporary variable to store scaled angular velocity
        pp_vec3_scale(&body->a_acc, t, &scaled_ang_vel);
        pp_vec3_add(&body->a_vel, &scaled_ang_vel, &body->a_vel);

        // Apply angular damping (linear approximation of exp(-x) ≈ 1-x for small x)
        float a_damp = 1.0f - body->a_damping * t;
        pp_vec3_scale(&body->a_vel, a_damp, &body->a_vel);

        // Reset the acceleration
        pp_vec3_init(&body->a_acc);

        float vl_sq = body->vel_limit * body->vel_limit;
        float avl_sq = body->a_vel_limit * body->a_vel_limit;

        // Apply any limits
        if (body->vel_limit != 0.0f && pp_vec3_length_sq(&body->vel) > vl_sq) {
            pp_vec3_normalize(&body->vel);
            pp_vec3_scale(&body->vel, body->vel_limit, &body->vel);
        }

        if (body->a_vel_limit != 0.0f && pp_vec3_length_sq(&body->a_vel) > avl_sq) {
            pp_vec3_normalize(&body->a_vel);
            pp_vec3_scale(&body->a_vel, body->a_vel_limit, &body->a_vel);
        }
    }
}

static void pp_solve_constraint_fixed_distance_velocities(PPBody* b1, PPBody* b2, float inv_sum, float target, float dt) {
    float inv_m1 = b1->inv_mass;
    float inv_m2 = b2->inv_mass;

    PPVec3 delta;
    pp_vec3_sub(&b2->pos, &b1->pos, &delta);

    float dist = pp_vec3_length(&delta);
    if(dist < 1e-6f){
        // Already close enough
        return;
    }

    PPVec3 n;
    pp_vec3_scale(&delta, 1.0f / dist, &n);

    PPVec3 rel_vel;
    pp_vec3_sub(&b2->vel, &b1->vel, &rel_vel);
    float v_rel = pp_vec3_dot(&rel_vel, &n);

    float Cpos = dist - target;

    /* Baumgarte bias */
    const float beta = 0.2f;             // bias factor
    const float slop_vel = 0.001f;       // tiny dead zone for bias
    float bias = 0.0f;
    if (fabsf(Cpos) > slop_vel && dt > 0.0f) {
        // Correct sign: positive Cpos (too far apart) should produce a
        // positive bias so that the computed impulse pulls bodies together.
        bias = (beta / dt) * Cpos;
    }

    /* effective mass (linear only) */
    float K = inv_sum;

    if (K < 1e-8f) {
        return;
    }

    float J = -(v_rel + bias) / K;

    /* clamp impulse to avoid instability */
    const float maxJ = 1000.0f;
    if (J > maxJ) J = maxJ;
    else if (J < -maxJ) J = -maxJ;

    /* apply linear impulses */
    PPVec3 impulse;
    pp_vec3_scale(&n, J, &impulse);

    PPVec3 tmp;
    // b1 gets negative impulse
    pp_vec3_scale(&impulse, inv_m1, &tmp);
    pp_vec3_sub(&b1->vel, &tmp, &b1->vel);

    // b2 gets positive impulse
    pp_vec3_scale(&impulse, inv_m2, &tmp);
    pp_vec3_add(&b2->vel, &tmp, &b2->vel);
}

static void pp_solve_constraint_velocities(PPConstraint* entry, float dt) {
    if(!entry || !entry->is_alive) {
        return;
    }

    PPBody* b1 = entry->body1;
    PPBody* b2 = entry->body2;
    if(!b1 || !b2 || !b1->is_alive || !b2->is_alive || entry->inv_mass_sum < 1e-8f){
        return;
    }

    if (entry->type == PP_CONSTRAINT_TYPE_FIXED_DISTANCE) {
        pp_solve_constraint_fixed_distance_velocities(b1,
                                                      b2,
                                                      entry->inv_mass_sum,
                                                      entry->fixed_distance.distance,
                                                      dt);
    } else {
        fprintf(stderr, "Constraint unimplemented\n");
    }
}

static void pp_solve_constraint_fixed_distance_positions(PPBody* b1, PPBody* b2, float inv_mass_sum, float target, float dt) {
    float inv_m1 = b1->inv_mass;
    float inv_m2 = b2->inv_mass;

    PPVec3 delta;
    pp_vec3_sub(&b2->pos, &b1->pos, &delta);

    float current = pp_vec3_length(&delta);
    if (current < 1e-6f) {
        // degenerate; nothing sensible to do
        return;
    }

    PPVec3 n;
    pp_vec3_scale(&delta, 1.0f / current, &n);

    float Cpos = current - target;

    const float slop = 0.01f;   // positional dead zone
    if (fabsf(Cpos) <= slop) {
        return;
    }

    const float percent = 0.2f; // apply 20% of correction per iteration
    float correction = (Cpos - (Cpos > 0.0f ? slop : -slop)) * percent;

    // distribute correction by inverse mass
    PPVec3 corr;
    // Move bodies TOWARD each other: b1 moves along +n, b2 moves along -n
    pp_vec3_scale(&n, correction * (inv_m1 / inv_mass_sum), &corr);
    pp_vec3_add(&b1->pos, &corr, &b1->pos);

    pp_vec3_scale(&n, correction * (inv_m2 / inv_mass_sum), &corr);
    pp_vec3_sub(&b2->pos, &corr, &b2->pos);
}

static void pp_solve_constraint_positions(PPConstraint *entry, float dt)
{
    if(!entry || !entry->is_alive) {
        return;
    }

    PPBody* b1 = entry->body1;
    PPBody* b2 = entry->body2;
    if(!b1 || !b2 || !b1->is_alive || !b2->is_alive || entry->inv_mass_sum < 1e-8f){
        return;
    }

    if(entry->type == PP_CONSTRAINT_TYPE_FIXED_DISTANCE){
        pp_solve_constraint_fixed_distance_positions(b1,
                                                     b2,
                                                     entry->inv_mass_sum,
                                                     entry->fixed_distance.distance,
                                                     dt);
    } else {
        fprintf(stderr, "Constraint unimplemented\n");
    }
}

static void pp_solve_velocities(const PPCollision *manifold, float t)
{
    PPBody *lhs = manifold->obj1;
    PPBody *rhs = manifold->obj2;

    if (!lhs->is_alive || !rhs->is_alive) {
        return;
    }

    float inv_mass_sum = lhs->inv_mass + rhs->inv_mass;

    if (inv_mass_sum < 1e-8f) {
        // Both bodies are static
        return;
    }

    const PPVec3 *n = &manifold->n;

    PPVec3 lhs_pcp = {.xyz = {0.0f, 0.0f, 0.0f}}, rhs_pcp = {.xyz = {0.0f, 0.0f, 0.0f}};

    // Position (CoM) to contact point (r_a/r_b). Don't set this
    // if the inv_mass is zero and then it has no effect later on
    pp_vec3_sub(&manifold->p, &lhs->pos, &lhs_pcp);
    pp_vec3_sub(&manifold->p, &rhs->pos, &rhs_pcp);

    // Velocities at contact point
    PPVec3 lhs_vel, rhs_vel, rel_vel;
    pp_vec3_cross(&lhs->a_vel, &lhs_pcp, &lhs_vel);
    pp_vec3_cross(&rhs->a_vel, &rhs_pcp, &rhs_vel);
    pp_vec3_add(&lhs_vel, &lhs->vel, &lhs_vel);
    pp_vec3_add(&rhs_vel, &rhs->vel, &rhs_vel);

    if (lhs->inv_mass == 0.0f) {
        pp_vec3_init(&lhs_vel);
    }

    if (rhs->inv_mass == 0.0f) {
        pp_vec3_init(&rhs_vel);
    }

    // Relative velocity at contact point
    pp_vec3_sub(&rhs_vel, &lhs_vel, &rel_vel); // v_rhs – v_lhs

    float vel_along_normal;
    vel_along_normal = pp_vec3_dot(&rel_vel, n);

    // Speculative contact bias. When there is still a gap (separation > 0) the
    // bodies are not touching yet, but were predicted to make contact this
    // step. Rather than zeroing the approach velocity (which would freeze the
    // body short of the surface), we permit it to approach just fast enough to
    // close the gap in exactly one step: allowed = separation / t. The solver
    // then only removes the velocity *in excess* of that, so a fast mover is
    // braked to land on the surface instead of passing through it.
    float allowed = (manifold->separation > 0.0f) ? (manifold->separation / t) : 0.0f;

    // Moving towards each other
    float r = fmaxf(manifold->obj1_bounce, manifold->obj2_bounce);

    const float bounce_threshold = 0.1f;
    if (fabsf(vel_along_normal) < bounce_threshold) {
        // If we're not moving, then don't add bounce!
        r = 0.0f;
    }

    if (manifold->separation > 0.0f) {
        // Don't bounce off a gap; restitution only applies on real contact.
        r = 0.0f;
    }

    // Target: vel_along_normal should end at >= -allowed. The required impulse
    // (numerator) is positive only when the body approaches faster than the
    // gap permits; the clamp below discards the negative (separating) case,
    // which is why no explicit "moving apart" early-out is needed here.
    float numerator = -(1.0f + r) * vel_along_normal - allowed;
    float linear_term = inv_mass_sum;

    // rhs_pcp might be zero here, but that's fine as it'll cause
    // zero contribution to the angular velocity
    PPVec3 lhs_pcp_cross_n, rhs_pcp_cross_n;
    pp_vec3_cross(&lhs_pcp, &manifold->n, &lhs_pcp_cross_n);
    pp_vec3_cross(&rhs_pcp, &manifold->n, &rhs_pcp_cross_n);

    PPVec3 i_lhs_pcp_cross_n, i_rhs_pcp_cross_n;
    pp_mat3_mult(&lhs->inv_inertia, &lhs_pcp_cross_n, &i_lhs_pcp_cross_n);
    pp_mat3_mult(&rhs->inv_inertia, &rhs_pcp_cross_n, &i_rhs_pcp_cross_n);

    PPVec3 lhs_angular_term, rhs_angular_term;
    pp_vec3_cross(&i_lhs_pcp_cross_n, &lhs_pcp, &lhs_angular_term);
    pp_vec3_cross(&i_rhs_pcp_cross_n, &rhs_pcp, &rhs_angular_term);

    float angular_term = pp_vec3_dot(&lhs_angular_term, n) + pp_vec3_dot(&rhs_angular_term, n);
    float denominator = linear_term + angular_term;

    if (fabsf(denominator) < 1e-8f) {
        // Prevent divide by zero
        return;
    }

    float J = numerator / denominator;
    if (J < 0.0f) {
        J = 0.0f;
    }

    PPVec3 impulse, ang_imp;
    pp_vec3_scale(n, J, &impulse);

    PPVec3 i_tmp;
    // Add linear impulse to the left
    pp_vec3_scale(&impulse, lhs->inv_mass, &i_tmp);
    pp_vec3_sub(&lhs->vel, &i_tmp, &lhs->vel); // v_lhs ← v_lhs + Δv

    // then add it to the right
    pp_vec3_scale(&impulse, rhs->inv_mass, &i_tmp);
    pp_vec3_add(&rhs->vel, &i_tmp, &rhs->vel);

    // Calculate and apply the angular impulse to the left. The sign must match
    // the linear impulse above: the impulse on lhs is -P (it subtracts P/m from
    // the linear velocity), so the angular change is -I^-1 (r_a x P). Using +
    // here made the normal impulse inconsistent with both the linear term and
    // the friction block below, injecting energy on any off-centre (lever-arm)
    // contact -- e.g. an offset sphere shape would spin up and fly off.
    pp_vec3_cross(&lhs_pcp, &impulse, &ang_imp);
    pp_mat3_mult(&lhs->inv_inertia, &ang_imp, &ang_imp);
    pp_vec3_sub(&lhs->a_vel, &ang_imp, &lhs->a_vel);

    // Same with the right (impulse on rhs is +P).
    pp_vec3_cross(&rhs_pcp, &impulse, &ang_imp);
    pp_mat3_mult(&rhs->inv_inertia, &ang_imp, &ang_imp);
    pp_vec3_add(&rhs->a_vel, &ang_imp, &rhs->a_vel);

    // Friction!!!
    float f = manifold->obj1_friction * manifold->obj2_friction;
    PPVec3 lhs_fv, rhs_fv;

    // Calculate velocities at the contact point
    pp_vec3_cross(&lhs->a_vel, &lhs_pcp, &lhs_fv);
    pp_vec3_add(&lhs_fv, &lhs->vel, &lhs_fv);

    pp_vec3_cross(&rhs->a_vel, &rhs_pcp, &rhs_fv);
    pp_vec3_add(&rhs_fv, &rhs->vel, &rhs_fv);

    // Get the relative velocity at the contact point
    // between the two objects
    PPVec3 fvr;
    pp_vec3_sub(&rhs_fv, &lhs_fv, &fvr);

    // Remove normal component to get tangent velocity
    float rv_dot_n = pp_vec3_dot(&fvr, n);

    PPVec3 normal_component;
    pp_vec3_scale(n, rv_dot_n, &normal_component);

    PPVec3 ftangent;
    pp_vec3_sub(&fvr, &normal_component, &ftangent);

    float ftangent_len = pp_vec3_length(&ftangent);
    if (ftangent_len < 1e-6f) {
        // No tangent velocity, so no friction impulse
        return;
    }

    pp_vec3_scale(&ftangent, 1.0f / ftangent_len, &ftangent); // Normalize

    // Effective mass for friction

    PPVec3 lhs_rt, rhs_rt;
    pp_vec3_cross(&lhs_pcp, &ftangent, &lhs_rt);
    pp_vec3_cross(&rhs_pcp, &ftangent, &rhs_rt);

    PPVec3 lhs_i_rt, rhs_i_rt;
    pp_mat3_mult(&lhs->inv_inertia, &lhs_rt, &lhs_i_rt);
    pp_mat3_mult(&rhs->inv_inertia, &rhs_rt, &rhs_i_rt);

    PPVec3 lhs_fangular_term, rhs_fangular_term;
    pp_vec3_cross(&lhs_i_rt, &lhs_pcp, &lhs_fangular_term);
    pp_vec3_cross(&rhs_i_rt, &rhs_pcp, &rhs_fangular_term);

    float friction_mass = inv_mass_sum + pp_vec3_dot(&lhs_fangular_term, &ftangent)
                          + pp_vec3_dot(&rhs_fangular_term, &ftangent);

    if (friction_mass < 1e-8f) {
        return;
    }

    // Friction impulse scalar
    float jt = -pp_vec3_dot(&fvr, &ftangent);
    jt /= friction_mass;

    // Coulomb friction clamp
    float max_friction = f * J;

    if (jt > max_friction)
        jt = max_friction;
    if (jt < -max_friction)
        jt = -max_friction;

    PPVec3 friction_impulse;
    pp_vec3_scale(&ftangent, jt, &friction_impulse);

    // --- Apply linear impulse ---

    PPVec3 temp;

    pp_vec3_scale(&friction_impulse, lhs->inv_mass, &temp);
    pp_vec3_sub(&lhs->vel, &temp, &lhs->vel);

    pp_vec3_scale(&friction_impulse, rhs->inv_mass, &temp);
    pp_vec3_add(&rhs->vel, &temp, &rhs->vel);

    // --- Apply angular impulse (matrix inertia) ---

    PPVec3 fang_imp;

    // lhs
    pp_vec3_cross(&lhs_pcp, &friction_impulse, &fang_imp);
    pp_mat3_mult(&lhs->inv_inertia, &fang_imp, &fang_imp);
    pp_vec3_sub(&lhs->a_vel, &fang_imp, &lhs->a_vel);

    // rhs
    pp_vec3_cross(&rhs_pcp, &friction_impulse, &fang_imp);
    pp_mat3_mult(&rhs->inv_inertia, &fang_imp, &fang_imp);
    pp_vec3_add(&rhs->a_vel, &fang_imp, &rhs->a_vel);
}

typedef struct _PPSupportPoint
{
    PPVec3 point;
    PPVec3 a;
    PPVec3 b;
} PPSupportPoint;

typedef struct _PPSimplex
{
    PPSupportPoint points[4];
    size_t count;
} PPSimplex;

typedef struct _PPPolytopeFace
{
    union {
        struct
        {
            uint8_t a;
            uint8_t b;
            uint8_t c;
        };
        uint8_t abc[3];
    };
    PPVec3 n;
    float d;
} PPPolytopeFace;

typedef struct _PPPolytopeEdge
{
    uint8_t a;
    uint8_t b;
} PPPolytopeEdge;

#define MAX_POLYTOPE_FACES 64
#define MAX_POLYTOPE_POINTS 128
#define MAX_POLYTOPE_EDGES 32

typedef struct _PPPolytope
{
    PPSupportPoint points[MAX_POLYTOPE_POINTS];
    size_t point_count;

    PPPolytopeFace faces[MAX_POLYTOPE_FACES];
    size_t face_count;

    PPPolytopeEdge edges[MAX_POLYTOPE_EDGES];
    size_t edge_count;
} PPPolytope;

static inline bool pp_same_direction(const PPVec3 *direction, const PPVec3 *ao)
{
    return pp_vec3_dot(direction, ao) > 0.0f;
}

PPSimplex *pp_simplex_init(PPSimplex *s)
{
    s->count = 0;
    return s;
}

bool pp_simplex_push(PPSimplex *simplex, const PPSupportPoint *point)
{
    assert(simplex->count < 4);

    memcpy(&simplex->points[3], &simplex->points[2], sizeof(PPSupportPoint));
    memcpy(&simplex->points[2], &simplex->points[1], sizeof(PPSupportPoint));
    memcpy(&simplex->points[1], &simplex->points[0], sizeof(PPSupportPoint));
    memcpy(&simplex->points[0], point, sizeof(PPSupportPoint));
    simplex->count++;

    return true;
}

PPSupportPoint *pp_simplex_at(PPSimplex *simplex, size_t i)
{
    assert(i < simplex->count);
    return &simplex->points[i];
}

void pp_simplex_replace1(PPSimplex *simplex, const PPSupportPoint *point)
{
    simplex->count = 1;
    memcpy(&simplex->points[0], point, sizeof(PPSupportPoint));
}

void pp_simplex_replace2(PPSimplex *simplex, const PPSupportPoint *p0, const PPSupportPoint *p1)
{
    simplex->count = 2;

    // Temp variables are necessary as p0 or p1 may be
    // the points we're replacing
    PPSupportPoint t0, t1;
    memcpy(&t0, p0, sizeof(PPSupportPoint));
    memcpy(&t1, p1, sizeof(PPSupportPoint));

    memcpy(&simplex->points[0], &t0, sizeof(PPSupportPoint));
    memcpy(&simplex->points[1], &t1, sizeof(PPSupportPoint));
}

void pp_simplex_replace3(PPSimplex *simplex,
                         const PPSupportPoint *p0,
                         const PPSupportPoint *p1,
                         const PPSupportPoint *p2)
{
    // FIXME: This needs performance testing. There are probably faster ways than just creating
    // 3 temporary variables
    simplex->count = 3;

    // Temp variables are necessary as p0 or p1 may be
    // the points we're replacing
    PPSupportPoint t0, t1, t2;
    memcpy(&t0, p0, sizeof(PPSupportPoint));
    memcpy(&t1, p1, sizeof(PPSupportPoint));
    memcpy(&t2, p2, sizeof(PPSupportPoint));

    memcpy(&simplex->points[0], &t0, sizeof(PPSupportPoint));
    memcpy(&simplex->points[1], &t1, sizeof(PPSupportPoint));
    memcpy(&simplex->points[2], &t2, sizeof(PPSupportPoint));
}

bool pp_simplex_next_line(PPSimplex *simplex, PPVec3 *direction)
{
    PPVec3 ab, ao;

    PPSupportPoint *a = pp_simplex_at(simplex, 0);
    PPSupportPoint *b = pp_simplex_at(simplex, 1);

    pp_vec3_sub(&b->point, &a->point, &ab);
    pp_vec3_neg(&a->point, &ao);

    if (pp_same_direction(&ab, &ao)) {
        PPVec3 t;
        pp_vec3_cross(&ab, &ao, &t);
        pp_vec3_cross(&t, &ab, direction);
    } else {
        pp_simplex_replace1(simplex, a);
        pp_vec3_assign(direction, &ao);
    }

    return false;
}

bool pp_simplex_next_triangle(PPSimplex *simplex, PPVec3 *direction)
{
    PPVec3 ab, ac, ao, abc;

    PPSupportPoint *a = pp_simplex_at(simplex, 0);
    PPSupportPoint *b = pp_simplex_at(simplex, 1);
    PPSupportPoint *c = pp_simplex_at(simplex, 2);

    pp_vec3_sub(&b->point, &a->point, &ab);
    pp_vec3_sub(&c->point, &a->point, &ac);
    pp_vec3_neg(&a->point, &ao);
    pp_vec3_cross(&ab, &ac, &abc);

    PPVec3 n;
    pp_vec3_cross(&abc, &ac, &n);
    if (pp_same_direction(&n, &ao)) {
        if (pp_same_direction(&ac, &ao)) {
            pp_simplex_replace2(simplex, a, c);

            PPVec3 t;
            pp_vec3_cross(&ac, &ao, &t);
            pp_vec3_cross(&t, &ac, direction);
        } else {
            pp_simplex_replace2(simplex, a, b);
            return pp_simplex_next_line(simplex, direction);
        }
    } else {
        pp_vec3_cross(&ab, &abc, &n);
        if (pp_same_direction(&n, &ao)) {
            pp_simplex_replace2(simplex, a, b);
            return pp_simplex_next_line(simplex, direction);
        } else {
            if (pp_same_direction(&abc, &ao)) {
                pp_vec3_assign(direction, &abc);
            } else {
                pp_simplex_replace3(simplex, a, c, b);
                pp_vec3_neg(&abc, direction);
            }
        }
    }

    return false;
}

bool pp_simplex_next_tetrahedron(PPSimplex *simplex, PPVec3 *direction)
{
    assert(simplex->count == 4);

    PPVec3 ab, ac, ad, ao;
    PPVec3 abc, acd, adb;

    PPSupportPoint *a = pp_simplex_at(simplex, 0);
    PPSupportPoint *b = pp_simplex_at(simplex, 1);
    PPSupportPoint *c = pp_simplex_at(simplex, 2);
    PPSupportPoint *d = pp_simplex_at(simplex, 3);

    pp_vec3_sub(&b->point, &a->point, &ab);
    pp_vec3_sub(&c->point, &a->point, &ac);
    pp_vec3_sub(&d->point, &a->point, &ad);
    pp_vec3_neg(&a->point, &ao);

    pp_vec3_cross(&ab, &ac, &abc);
    if (pp_same_direction(&abc, &ao)) {
        // FIXME: this is just a size reduction. Potential optimisation.
        pp_simplex_replace3(simplex, a, b, c);
        return pp_simplex_next_triangle(simplex, direction);
    }

    pp_vec3_cross(&ac, &ad, &acd);
    if (pp_same_direction(&acd, &ao)) {
        pp_simplex_replace3(simplex, a, c, d);
        return pp_simplex_next_triangle(simplex, direction);
    }

    pp_vec3_cross(&ad, &ab, &adb);
    if (pp_same_direction(&adb, &ao)) {
        pp_simplex_replace3(simplex, a, d, b);
        return pp_simplex_next_triangle(simplex, direction);
    }

    return true;
}

bool pp_simplex_next(PPSimplex *simplex, PPVec3 *direction)
{
    if (simplex->count == 2) {
        return pp_simplex_next_line(simplex, direction);
    } else if (simplex->count == 3) {
        return pp_simplex_next_triangle(simplex, direction);
    } else if (simplex->count == 4) {
        return pp_simplex_next_tetrahedron(simplex, direction);
    }

    return false;
}

void pp_find_furthest_point_sphere(const PPVec3 *center, float radius, const PPVec3 *direction, PPVec3 *point)
{
    PPVec3 ndir;
    pp_vec3_assign(&ndir, direction);
    pp_vec3_normalize(&ndir);

    pp_vec3_scale(&ndir, radius, &ndir);
    pp_vec3_add(center, &ndir, point);
}

void pp_find_furthest_point_box(const PPVec3 *center, const PPQuaternion *rot, const PPVec3 *half_extents, const PPVec3 *direction, PPVec3 *point)
{
    PPVec3 vertices[8];

    for (int i = 0; i < 8; ++i) {
        vertices[i].xyz[0] = (i & 1) ? half_extents->x : -half_extents->x;
        vertices[i].xyz[1] = (i & 2) ? half_extents->y : -half_extents->y;
        vertices[i].xyz[2] = (i & 4) ? half_extents->z : -half_extents->z;

        pp_quat_transform(rot, &vertices[i], &vertices[i]);
        pp_vec3_add(&vertices[i], center, &vertices[i]);
    }

    float max_dot = pp_vec3_dot(direction, &vertices[0]);
    int max_index = 0;

    for (int i = 1; i < 8; ++i) {
        float dot = pp_vec3_dot(direction, &vertices[i]);
        if (dot > max_dot) {
            max_dot = dot;
            max_index = i;
        }
    }

    pp_vec3_assign(point, &vertices[max_index]);
}

static void pp_find_furthest_point_shape(const PPShape *shape, const PPVec3 *world_pos, const PPQuaternion *body_rot, const PPVec3 *direction, PPVec3 *point)
{
    if (shape->type == PP_OBJECT_TYPE_SPHERE) {
        pp_find_furthest_point_sphere(world_pos, shape->sphere.radius, direction, point);
    } else if (shape->type == PP_OBJECT_TYPE_BOX) {
        pp_find_furthest_point_box(world_pos, body_rot, &shape->box.half_extents, direction, point);
    }
}

bool pp_gjk_support_shapes(const PPShape *s1, const PPVec3 *pos1, const PPQuaternion *rot1,
                           const PPShape *s2, const PPVec3 *pos2, const PPQuaternion *rot2,
                           const PPVec3 *direction, PPSupportPoint *out)
{
    PPVec3 reverse;
    pp_vec3_neg(direction, &reverse);
    pp_find_furthest_point_shape(s1, pos1, rot1, direction, &out->a);
    pp_find_furthest_point_shape(s2, pos2, rot2, &reverse, &out->b);
    pp_vec3_sub(&out->a, &out->b, &out->point);
    return true;
}

/* Context for GJK/EPA: shapes + world positions */
typedef struct _PPGJKContext {
    const PPShape *s1;
    const PPVec3 *pos1;
    const PPQuaternion *rot1;
    const PPShape *s2;
    const PPVec3 *pos2;
    const PPQuaternion *rot2;
} PPGJKContext;

static PPGJKContext gjk_ctx;

bool pp_gjk_support(const PPBody *b1, const PPBody *b2, const PPVec3 *direction, PPSupportPoint *out)
{
    return pp_gjk_support_shapes(gjk_ctx.s1, gjk_ctx.pos1, gjk_ctx.rot1,
                                 gjk_ctx.s2, gjk_ctx.pos2, gjk_ctx.rot2,
                                 direction, out);
}

bool pp_gjk_collide(const PPBody *b1, const PPBody *b2, PPSimplex *simplex)
{
    // ab = a.pos - b.pos
    //  c = Vector3(ab.z, ab.z, -ab.x - ab.y)
    //  if c == Vector3.zero():
    //      c = Vector3(-ab.y - ab.z, ab.x, ab.x)

    PPVec3 ab, c, abc;
    pp_vec3_sub(&b1->pos, &b2->pos, &ab);
    pp_vec3_set(&c, ab.z, ab.z, -ab.x - ab.y);
    if (pp_vec3_length_sq(&c) < FLT_EPSILON) {
        pp_vec3_set(&c, -ab.y - ab.z, ab.x, ab.x);
    }

    pp_vec3_cross(&ab, &c, &abc);

    PPSupportPoint support;
    if (!pp_gjk_support(b1, b2, &abc, &support)) {
#ifndef NDEBUG
        fprintf(stderr, "Invalid body types\n");
#endif
        return false;
    }
    pp_simplex_push(simplex, &support);

    PPVec3 direction;
    pp_vec3_neg(&support.point, &direction);

    int i = 0;
    for (i = 0; i < 50; ++i) {
        if (!pp_gjk_support(b1, b2, &direction, &support)) {
            return false;
        }

        if (pp_vec3_dot(&support.point, &direction) <= 0) {
            return false;
        }

        pp_simplex_push(simplex, &support);

        if (pp_simplex_next(simplex, &direction)) {
            return true;
        }
    }

    if (i == 50) {
        fprintf(stderr, "Max iterations reached\n");
        return false;
    }

    return false;
}

void pp_polytope_clear_edges(PPPolytope *polytope)
{
    polytope->edge_count = 0;
}

void pp_polytope_push_edge(PPPolytope *polytope, uint8_t a, uint8_t b)
{
    assert(a != b);
    assert(polytope->edge_count < MAX_POLYTOPE_EDGES);

    for (size_t i = 0; i < polytope->edge_count; ++i) {
        if (polytope->edges[i].a == b && polytope->edges[i].b == a) {
            // We need to erase i
            for (int j = i; j < ((int) polytope->edge_count) - 1; ++j) {
                polytope->edges[j].a = polytope->edges[j + 1].a;
                polytope->edges[j].b = polytope->edges[j + 1].b;
            }
            polytope->edge_count--;
            break;
        }
    }

    polytope->edges[polytope->edge_count].a = a;
    polytope->edges[polytope->edge_count].b = b;
    polytope->edge_count++;
}

static inline void pp_vec3_average(const PPVec3 *a, const PPVec3 *b, const PPVec3 *c, PPVec3 *out)
{
    const float one_third = 1.0f / 3.0f;
    for (int i = 0; i < 3; ++i) {
        out->xyz[i] = (a->xyz[i] + b->xyz[i] + c->xyz[i]) * one_third;
    }
}

bool pp_polytope_calc_face_normal(PPPolytope *polytope, PPPolytopeFace *face)
{
    PPVec3 ab, ac;

    const PPVec3 *a = &polytope->points[face->a].point;
    const PPVec3 *b = &polytope->points[face->b].point;
    const PPVec3 *c = &polytope->points[face->c].point;

    pp_vec3_sub(b, a, &ab);
    pp_vec3_sub(c, a, &ac);
    pp_vec3_cross(&ab, &ac, &face->n);

    bool ret = pp_vec3_normalize(&face->n);
    // assert(ret); // This should always be a valid normal

    if (!ret) {
        return false;
    }

    // PPVec3 average;
    // pp_vec3_average(a, b, c, &average);
    // polytope->faces[i].d = pp_vec3_length(&average);
    face->d = pp_vec3_dot(&face->n, a);

    if (face->d < 0) {
        pp_vec3_neg(&face->n, &face->n);
        face->d *= -1.0f;
    }

    return true;
}

float pp_polytope_push_face(PPPolytope *polytope, uint8_t a, uint8_t b, uint8_t c)
{
    assert(polytope->face_count < MAX_POLYTOPE_FACES - 1);
    assert(a != b);
    assert(a != c);

    PPPolytopeFace *new_face = &polytope->faces[polytope->face_count];
    new_face->a = a;
    new_face->b = b;
    new_face->c = c;

    // If the face is degenerate, skip it
    if (!pp_polytope_calc_face_normal(polytope, new_face)) {
        return FLT_MAX;
    }

    polytope->face_count++;
    return new_face->d;
}

void pp_polytope_erase_face(PPPolytope *polytope, size_t face_index)
{
    assert(face_index < polytope->face_count);
    assert(polytope->face_count > 0);

    const PPPolytopeFace *back = &polytope->faces[polytope->face_count - 1];
    PPPolytopeFace *erase = &polytope->faces[face_index];
    memcpy(erase, back, sizeof(PPPolytopeFace));

    polytope->face_count--;
}

void pp_polytope_write(const PPPolytope *polytope, const char *filename)
{
    FILE *out = fopen(filename, "wt");
    for (size_t i = 0; i < polytope->point_count; ++i) {
        fprintf(out,
                "v %f %f %f\n",
                polytope->points[i].point.x,
                polytope->points[i].point.y,
                polytope->points[i].point.z);
    }

    for (size_t i = 0; i < polytope->face_count; ++i) {
        fprintf(out,
                "f %d %d %d\n",
                polytope->faces[i].a + 1,
                polytope->faces[i].b + 1,
                polytope->faces[i].c + 1);
    }

    fclose(out);
}

void pp_triangle_get_barycentric(const PPTriangle *tri, const PPVec3 *p, PPVec3 *coords)
{
    const PPVec3 *a = &tri->v[0];
    const PPVec3 *b = &tri->v[1];
    const PPVec3 *c = &tri->v[2];

    PPVec3 v0, v1, v2;
    pp_vec3_sub(b, a, &v0);
    pp_vec3_sub(c, a, &v1);
    pp_vec3_sub(p, a, &v2);

    float d00 = pp_vec3_dot(&v0, &v0);
    float d01 = pp_vec3_dot(&v0, &v1);
    float d11 = pp_vec3_dot(&v1, &v1);
    float d20 = pp_vec3_dot(&v2, &v0);
    float d21 = pp_vec3_dot(&v2, &v1);

    float denom = d00 * d11 - d01 * d01;
    coords->y = (d11 * d20 - d01 * d21) / denom;
    coords->z = (d00 * d21 - d01 * d20) / denom;
    coords->x = 1.0f - coords->y - coords->z;
}

int pp_find_min_face(const PPPolytope *polytope)
{
    int min_face = 0;
    float min_dot = FLT_MAX;

    for (size_t i = 0; i < polytope->face_count; i++) {
        const PPPolytopeFace *face = &polytope->faces[i];
        float dot = face->d;
        if (dot < min_dot) {
            min_dot = dot;
            min_face = i;
        }
    }

    return min_face;
}

bool pp_epa(PPSimplex *simplex,
            const PPBody *lhs,
            const PPBody *rhs,
            PPVec3 *n,
            float *intersection,
            PPVec3 *contact)
{
    assert(simplex->count == 4);

    const PPSupportPoint *a = pp_simplex_at(simplex, 0);
    const PPSupportPoint *b = pp_simplex_at(simplex, 1);
    const PPSupportPoint *c = pp_simplex_at(simplex, 2);
    const PPSupportPoint *d = pp_simplex_at(simplex, 3);

    static PPPolytope polytope;
    polytope.face_count = 0;
    polytope.edge_count = 0;

    polytope.point_count = 4;
    memcpy(&polytope.points[0], a, sizeof(PPSupportPoint));
    memcpy(&polytope.points[1], b, sizeof(PPSupportPoint));
    memcpy(&polytope.points[2], c, sizeof(PPSupportPoint));
    memcpy(&polytope.points[3], d, sizeof(PPSupportPoint));

    pp_polytope_push_face(&polytope, 0, 1, 2);
    pp_polytope_push_face(&polytope, 0, 3, 1);
    pp_polytope_push_face(&polytope, 0, 2, 3);
    pp_polytope_push_face(&polytope, 1, 3, 2);

    int min_face = pp_find_min_face(&polytope);
    assert(min_face > -1);
    if (min_face < 0) {
        // Degenerate face
        return false;
    }

    const int MAX_ITERATIONS = 32;

    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        // Always pick the genuinely closest face by scanning all of them.
        // Tracking this index incrementally is unsafe: pp_polytope_erase_face
        // swaps the last face into the erased slot, so any stored index or
        // pointer goes stale mid-expansion. That stale index was the root of
        // EPA reporting the depth of a *far* face (the huge push-apart we saw)
        // and of the runaway expansion that overflowed the face budget.
        min_face = pp_find_min_face(&polytope);
        PPVec3 face_n = polytope.faces[min_face].n;
        float face_d = polytope.faces[min_face].d;

#if EPA_DEBUG
        char filename[100];
        sprintf(filename, "%d.obj", iter);
        pp_polytope_write(&polytope, filename);
#endif

        PPSupportPoint support;
        if (!pp_gjk_support(lhs, rhs, &face_n, &support)) {
            break;
        }

        float s_dist = pp_vec3_dot(&support.point, &face_n);

        // Converged: the support point in the closest-face direction is no
        // further out than that face, so the face lies on the Minkowski hull.
        if (s_dist - face_d < 0.001f) {
            break;
        }

        // Expand the polytope toward the new support point: remove every face
        // it can "see", record the horizon edges, then fan the horizon to the
        // new vertex.
        pp_polytope_clear_edges(&polytope);

        bool edge_budget_hit = false;
        for (size_t i = 0; i < polytope.face_count; ++i) {
            PPVec3 dir;
            pp_vec3_sub(&support.point, &polytope.points[polytope.faces[i].a].point, &dir);

            if (pp_same_direction(&polytope.faces[i].n, &dir)) {
                // Each face contributes three horizon edges; bail out cleanly
                // rather than overrunning the edge buffer on a complex horizon.
                if (polytope.edge_count + 3 > MAX_POLYTOPE_EDGES) {
                    edge_budget_hit = true;
                    break;
                }

                pp_polytope_push_edge(&polytope, polytope.faces[i].a, polytope.faces[i].b);
                pp_polytope_push_edge(&polytope, polytope.faces[i].b, polytope.faces[i].c);
                pp_polytope_push_edge(&polytope, polytope.faces[i].c, polytope.faces[i].a);

                pp_polytope_erase_face(&polytope, i);

                --i;
            }
        }

        if (edge_budget_hit || !polytope.edge_count) {
            break;
        }

        if (polytope.point_count >= MAX_POLYTOPE_POINTS) {
            break; // out of vertex budget; use the best face found so far
        }

        uint8_t new_point_index = (uint8_t) polytope.point_count;
        memcpy(&polytope.points[polytope.point_count++], &support, sizeof(PPSupportPoint));

        bool face_budget_hit = false;
        for (size_t i = 0; i < polytope.edge_count; ++i) {
            if (polytope.face_count >= MAX_POLYTOPE_FACES - 1) {
                face_budget_hit = true;
                break;
            }

            pp_polytope_push_face(&polytope,
                                  polytope.edges[i].a,
                                  polytope.edges[i].b,
                                  new_point_index);
        }

        if (face_budget_hit) {
            break;
        }
    }

    // Recompute the closest face one final time so the result reflects the
    // current polytope rather than any index left over from the loop.
    min_face = pp_find_min_face(&polytope);
    PPPolytopeFace *min_face_ptr = &polytope.faces[min_face];

    *intersection = min_face_ptr->d + 0.001f;

    PPTriangle tri;
    const PPPolytopeFace *f = min_face_ptr;
    pp_vec3_assign(&tri.v[0], &polytope.points[f->a].point);
    pp_vec3_assign(&tri.v[1], &polytope.points[f->b].point);
    pp_vec3_assign(&tri.v[2], &polytope.points[f->c].point);

    PPVec3 barycentric, point;
    pp_vec3_scale(&f->n, *intersection, &point);
    pp_triangle_get_barycentric(&tri, &point, &barycentric);

    PPVec3 p0, p1, p2;
    pp_vec3_scale(&polytope.points[f->a].a, barycentric.x, &p0);
    pp_vec3_scale(&polytope.points[f->b].a, barycentric.y, &p1);
    pp_vec3_scale(&polytope.points[f->c].a, barycentric.z, &p2);
    pp_vec3_add(&p0, &p1, contact);
    pp_vec3_add(contact, &p2, contact);

    // The closest-face normal points away from the origin in Minkowski (A-B)
    // space; that is exactly the contact normal the solver expects -- pointing
    // from obj1 (lhs) toward obj2 (rhs). (Negating it here pushed overlapping
    // bodies *together*, which made box stacks gain energy and explode.)
    pp_vec3_assign(n, &min_face_ptr->n);

    return true;
}

static inline bool flt_close(const float a, const float b)
{
    return (a + FLT_EPSILON > b) && (a - FLT_EPSILON) < b;
}

bool pp_sphere_box_intersect(
    const PPVec3 *sphere_pos, float sphere_radius,
    const PPVec3 *box_pos, const PPQuaternion *box_rot, const PPVec3 *box_half_extents, float box_bounding_radius,
    PPVec3 *contact_point, PPVec3 *n, float *intersection)
{
    // Early-out: bounding sphere check (cheap squared distance)
    PPVec3 diff;
    pp_vec3_sub(sphere_pos, box_pos, &diff);
    float diff_dist_sq = pp_vec3_dot(&diff, &diff);
    float sum_radius = sphere_radius + box_bounding_radius;
    if (diff_dist_sq > sum_radius * sum_radius) {
        return false;
    }

    // Transform sphere center to box local space
    PPVec3 sphere_center_local;
    PPVec3 tmp;
    pp_vec3_sub(sphere_pos, box_pos, &tmp);

    PPQuaternion box_rot_inv;
    pp_quat_conjugate(box_rot, &box_rot_inv);
    pp_quat_transform(&box_rot_inv, &tmp, &sphere_center_local);

    // Clamp to box half_extents (local space)
    float hx = box_half_extents->x, hy = box_half_extents->y, hz = box_half_extents->z;
    float cx = sphere_center_local.x;
    float cy = sphere_center_local.y;
    float cz = sphere_center_local.z;
    if (cx < -hx) cx = -hx; else if (cx > hx) cx = hx;
    if (cy < -hy) cy = -hy; else if (cy > hy) cy = hy;
    if (cz < -hz) cz = -hz; else if (cz > hz) cz = hz;

    // Delta from closest point to sphere center (local space)
    float dx = sphere_center_local.x - cx;
    float dy = sphere_center_local.y - cy;
    float dz = sphere_center_local.z - cz;
    float dist_sq = dx*dx + dy*dy + dz*dz;

    if (dist_sq > sphere_radius * sphere_radius) {
        return false;
    }

    // Transform contact point back to world space
    PPVec3 contact_world;
    pp_vec3_set(&sphere_center_local, cx, cy, cz);
    pp_quat_transform(box_rot, &sphere_center_local, &contact_world);
    pp_vec3_add(&contact_world, box_pos, contact_point);

    // Penetration depth
    float dist = sqrtf(dist_sq);
    if (intersection) {
        *intersection = sphere_radius - dist;
    }

    if (n) {
        PPVec3 normal_local;
        if (dist > 1e-6f) {
            float inv_dist = -1.0f / dist;
            pp_vec3_set(&normal_local, dx * inv_dist, dy * inv_dist, dz * inv_dist);
        } else {
            // Sphere center is inside box
            pp_vec3_set(&normal_local, -1.0f, 0.0f, 0.0f);
        }
        pp_quat_transform(box_rot, &normal_local, n);
    }

    return true;
}

static void pp_integrate_velocities(float t)
{
    // Move all bodies by their velocity
    for (int i = 0; i < object_count; ++i) {
        PPBody *body = &objects[i];
        if (!body->is_alive || body->inv_mass == 0.0f) {
            continue;
        }

        PPVec3 scaled_vel;
        pp_vec3_scale(&body->vel, t, &scaled_vel);
        pp_vec3_add(&body->pos, &scaled_vel, &body->pos);

        PPQuaternion q_rot;
        pp_quat_from_angular_velocity(&body->a_vel,
                                      t,
                                      &q_rot); // Get rotation quaternion from angular velocity
        pp_quat_multiply(&body->rot, &q_rot, &body->rot); // Combine with current rotation
        pp_quat_normalize(&body->rot);                    // Normalize the quaternion
    }
}

static void pp_solve_positions(const PPCollision *manifold)
{
    PPBody *lhs = manifold->obj1;
    PPBody *rhs = manifold->obj2;

    if (!lhs->is_alive || !rhs->is_alive) {
        return;
    }

    float inv_mass_sum = lhs->inv_mass + rhs->inv_mass;
    if (inv_mass_sum < 1e-8f) {
        return; // both static
    }

    // Recompute penetration depth.
    PPVec3 delta;
    pp_vec3_sub(&rhs->pos, &lhs->pos, &delta);

    float penetration = manifold->dist;

    if (penetration <= 0.0f)
        return;

    const float slop = 0.01f;
    const float percent = 0.2f; // 20% correction per iteration

    float correction_mag = penetration - slop;
    if (correction_mag < 0.0f)
        return;

    correction_mag *= percent;

    PPVec3 correction;
    pp_vec3_scale(&manifold->n, correction_mag, &correction);

    float lhs_ratio = lhs->inv_mass / inv_mass_sum;
    float rhs_ratio = rhs->inv_mass / inv_mass_sum;

    PPVec3 lhs_correction, rhs_correction;

    pp_vec3_scale(&correction, lhs_ratio, &lhs_correction);
    pp_vec3_scale(&correction, rhs_ratio, &rhs_correction);

    pp_vec3_sub(&lhs->pos, &lhs_correction, &lhs->pos);
    pp_vec3_add(&rhs->pos, &rhs_correction, &rhs->pos);
}

static inline void pp_shape_world_pos(const PPBody *body, const PPShape *shape, PPVec3 *out)
{
    PPVec3 rotated_offset;
    pp_quat_transform(&body->rot, &shape->offset, &rotated_offset);
    pp_vec3_add(&body->pos, &rotated_offset, out);
}

void pp_physics_step(float t, int vel_iterations, int pos_iterations)
{
    int manifold_count = 0;
    static PPCollision manifolds[PICOPHYSICS_MAX_MANIFOLDS];

    for (int i = 0; i < object_count; ++i) {
        PPBody *lhs_body = &objects[i];

        if (!lhs_body->is_alive || lhs_body->shape_count == 0) {
            continue;
        }

        const struct _PPCollisionMapEntry *cb = NULL;

        // Sphere/triangle collisions: iterate shapes of this body
        for (int si = 0; si < lhs_body->shape_count; ++si) {
            const PPShape *lhs_shape = &lhs_body->shapes[si];
            if (lhs_shape->type != PP_OBJECT_TYPE_SPHERE) continue;

            PPVec3 shape_pos;
            pp_shape_world_pos(lhs_body, lhs_shape, &shape_pos);

            float radius = lhs_shape->sphere.radius;
            float pos_x = shape_pos.x;
            float pos_y = shape_pos.y;
            float pos_z = shape_pos.z;

            // Expand the broadphase by the distance the sphere may travel this
            // step. A fast mover that the static radius check would cull is
            // still considered, so it can be caught speculatively before it
            // tunnels through a triangle.
            float speed = pp_vec3_length(&lhs_body->vel);
            float reach = radius + speed * t;
            float reach_sq = reach * reach;

            int last_kind = -1;
            for (int j = 0; j < tri_count; ++j) {
                const PPTriangle *tri = tris + j;

                float dx = fmaxf(0.0f, fmaxf(tri->aabb_min_x - pos_x, pos_x - tri->aabb_max_x));
                float dy = fmaxf(0.0f, fmaxf(tri->aabb_min_y - pos_y, pos_y - tri->aabb_max_y));
                float dz = fmaxf(0.0f, fmaxf(tri->aabb_min_z - pos_z, pos_z - tri->aabb_max_z));

                float dist_sq = dx*dx + dy*dy + dz*dz;
                if (dist_sq > reach_sq) {
                    continue;
                }

                if(tri->kind != last_kind) {
                    last_kind = tri->kind;
                    cb = pp_physics_collision_map_search(lhs_shape->kind, tri->kind);
                }

                if(cb && !cb->collision_callback) {
                    continue;
                }

                PPVec3 p, d;
                pp_vec3_scale(&tri->n, -1.0f, &d);
                float dist; // perpendicular distance from sphere centre to plane
                if (pp_tri_intersect(tri, &shape_pos, &d, &p, &dist)) {
                    float separation = dist - radius;

                    // Decide whether to register a contact this step. Either the
                    // sphere is already touching/penetrating, or it has a gap but
                    // is approaching fast enough to close it within this step --
                    // a speculative contact, which is what stops fast spheres
                    // tunnelling straight through the triangle.
                    bool contact = false;
                    if (separation <= 0.0f) {
                        contact = true;
                    } else {
                        // Approach speed is the velocity component heading into
                        // the front face of the triangle.
                        float approach = -pp_vec3_dot(&lhs_body->vel, &tri->n);
                        if (approach > 0.0f && separation <= approach * t) {
                            contact = true;
                        }
                    }

                    if (contact) {
                        PPCollision c;
                        pp_fill_collision_info_sphere_triangle(lhs_body, lhs_shape,
                                                               tri,
                                                               &p,
                                                               radius - dist,
                                                               &c);

                        bool respond = true;
                        if (cb) {
                            respond = cb->collision_callback(lhs_body,
                                                             tri,
                                                             lhs_shape->kind,
                                                             tri->kind,
                                                             &c,
                                                             cb->user_data);
                        }

                        if (respond) {
                            manifolds[manifold_count++] = c;
                        }
                    }
                }
            }
        }

        // Body-body collisions
        for (int j = i + 1; j < object_count; ++j) {
            PPBody *rhs_body = &objects[j];

            if (!rhs_body->is_alive || rhs_body->shape_count == 0) {
                continue;
            }

            // Iterate shape pairs
            for (int si = 0; si < lhs_body->shape_count; ++si) {
                const PPShape *lhs_shape = &lhs_body->shapes[si];
                PPVec3 lhs_spos;
                pp_shape_world_pos(lhs_body, lhs_shape, &lhs_spos);

                for (int sj = 0; sj < rhs_body->shape_count; ++sj) {
                    const PPShape *rhs_shape = &rhs_body->shapes[sj];
                    PPVec3 rhs_spos;
                    pp_shape_world_pos(rhs_body, rhs_shape, &rhs_spos);

                    cb = pp_physics_collision_map_search(lhs_shape->kind, rhs_shape->kind);
                    if (cb && !cb->collision_callback) {
                        continue;
                    }

                    if (lhs_shape->type == PP_OBJECT_TYPE_SPHERE && rhs_shape->type == PP_OBJECT_TYPE_SPHERE) {
                        float dist = pp_vec3_dist_sq(&lhs_spos, &rhs_spos);
                        float radius_sum = lhs_shape->sphere.radius + rhs_shape->sphere.radius;
                        if (dist <= (radius_sum * radius_sum)) {
                            dist = sqrtf(dist);
                            PPCollision c;
                            pp_fill_collision_info_sphere_sphere(lhs_body, lhs_shape, &lhs_spos,
                                                                 rhs_body, rhs_shape, &rhs_spos, dist, &c);

                            bool respond = true;
                            if (cb) {
                                respond = cb->collision_callback(lhs_body,
                                                                 rhs_body,
                                                                 lhs_shape->kind,
                                                                 rhs_shape->kind,
                                                                 &c,
                                                                 cb->user_data);
                            }

                            if (respond) {
                                manifolds[manifold_count++] = c;
                            }
                        }
                    } else if ((lhs_shape->type == PP_OBJECT_TYPE_SPHERE && rhs_shape->type == PP_OBJECT_TYPE_BOX) ||
                               (lhs_shape->type == PP_OBJECT_TYPE_BOX && rhs_shape->type == PP_OBJECT_TYPE_SPHERE)) {
                        const PPShape *sphere_shape, *box_shape;
                        const PPVec3 *sphere_pos, *box_pos;
                        PPBody *sphere_body, *box_body;

                        if (lhs_shape->type == PP_OBJECT_TYPE_SPHERE) {
                            sphere_shape = lhs_shape; sphere_pos = &lhs_spos; sphere_body = lhs_body;
                            box_shape = rhs_shape; box_pos = &rhs_spos; box_body = rhs_body;
                        } else {
                            sphere_shape = rhs_shape; sphere_pos = &rhs_spos; sphere_body = rhs_body;
                            box_shape = lhs_shape; box_pos = &lhs_spos; box_body = lhs_body;
                        }

                        PPVec3 contact, n;
                        float d;
                        if (pp_sphere_box_intersect(sphere_pos, sphere_shape->sphere.radius,
                                                    box_pos, &box_body->rot,
                                                    &box_shape->box.half_extents, box_shape->box.bounding_radius,
                                                    &contact, &n, &d)) {
                            PPCollision c;
                            pp_fill_collision_info_sphere_box(sphere_body, sphere_shape,
                                                              box_body, box_shape,
                                                              &contact, &n, &c, d);

                            bool respond = true;
                            if (cb) {
                                respond = cb->collision_callback(sphere_body,
                                                                 box_body,
                                                                 sphere_shape->kind,
                                                                 box_shape->kind,
                                                                 &c,
                                                                 cb->user_data);
                            }

                            if (respond) {
                                manifolds[manifold_count++] = c;
                            }
                        }
                    } else if (lhs_shape->type == PP_OBJECT_TYPE_BOX && rhs_shape->type == PP_OBJECT_TYPE_BOX) {
                        // Box vs Box via GJK/EPA
                        gjk_ctx.s1 = lhs_shape; gjk_ctx.pos1 = &lhs_spos; gjk_ctx.rot1 = &lhs_body->rot;
                        gjk_ctx.s2 = rhs_shape; gjk_ctx.pos2 = &rhs_spos; gjk_ctx.rot2 = &rhs_body->rot;

                        PPSimplex simplex;
                        pp_simplex_init(&simplex);
                        // Use dummy bodies with positions at shape world positions for GJK
                        PPBody tmp_b1 = *lhs_body;
                        tmp_b1.pos = lhs_spos;
                        PPBody tmp_b2 = *rhs_body;
                        tmp_b2.pos = rhs_spos;
                        if (pp_gjk_collide(&tmp_b1, &tmp_b2, &simplex)) {
                            PPVec3 n;
                            float depth;
                            PPVec3 contact;
                            if (pp_epa(&simplex, &tmp_b1, &tmp_b2, &n, &depth, &contact)) {
                                PPCollision c;
                                c.p = contact;
                                c.n = n;
                                c.dist = depth;
                                c.separation = -depth; // only reported while overlapping
                                c.obj1 = lhs_body;
                                c.obj2 = rhs_body;
                                c.obj1_bounce = lhs_body->bounce;
                                c.obj2_bounce = rhs_body->bounce;
                                c.obj1_friction = lhs_body->friction;
                                c.obj2_friction = rhs_body->friction;
                                c.type1 = PP_OBJECT_TYPE_BOX;
                                c.type2 = PP_OBJECT_TYPE_BOX;
                                c.kind1 = lhs_shape->kind;
                                c.kind2 = rhs_shape->kind;

                                bool respond = true;
                                if (cb) {
                                    respond = cb->collision_callback(lhs_body,
                                                                     rhs_body,
                                                                     lhs_shape->kind,
                                                                     rhs_shape->kind,
                                                                     &c,
                                                                     cb->user_data);
                                }
                                if (respond) {
                                    manifolds[manifold_count++] = c;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    pp_integrate_forces(t);

    for (int j = 0; j < vel_iterations; ++j) {
        for (int i = 0; i < manifold_count; ++i) {
            pp_solve_velocities(&manifolds[i], t);
        }

        for(int i = 0; i < constraint_count; ++i) {
            pp_solve_constraint_velocities(&constraints[i], t);
        }
    }

    pp_integrate_velocities(t);

    for (int j = 0; j < pos_iterations; ++j) {
        for (int i = 0; i < manifold_count; ++i) {
            pp_solve_positions(&manifolds[i]);
        }

        for(int i = 0; i < constraint_count; ++i) {
            pp_solve_constraint_positions(&constraints[i], t);
        }
    }
}

#endif
