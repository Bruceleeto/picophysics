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
 * - Boxes (fully dynamic; box/box and box/triangle use SAT with a multi-point
 *   contact manifold, so boxes rest and stack stably)
 * - Triangles (static environment, not a body shape)
 *
 * Bodies can also be joined together through constraints. Currently only fixed distance constraints are supported.
 *
 * Ray-casting the world is also supported.
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

#define PP_MAX_BOX_CONTACTS 8

typedef struct _PPBoxContact {
    PPVec3 normal;                   /* unit, points from box A toward box B */
    int count;
    PPVec3 points[PP_MAX_BOX_CONTACTS];
    float depths[PP_MAX_BOX_CONTACTS];
} PPBoxContact;

static inline void pp_box_world_axes(const PPQuaternion *rot, PPVec3 axes[3])
{
    PPVec3 x = {.xyz = {1.0f, 0.0f, 0.0f}};
    PPVec3 y = {.xyz = {0.0f, 1.0f, 0.0f}};
    PPVec3 z = {.xyz = {0.0f, 0.0f, 1.0f}};
    pp_quat_transform(rot, &x, &axes[0]);
    pp_quat_transform(rot, &y, &axes[1]);
    pp_quat_transform(rot, &z, &axes[2]);
}

/* Project a box's half-extents onto a (unit) axis: the box's radius along L. */
static inline float pp_box_project_radius(const PPVec3 *he, const PPVec3 axes[3], const PPVec3 *L)
{
    return he->x * fabsf(pp_vec3_dot(&axes[0], L)) +
           he->y * fabsf(pp_vec3_dot(&axes[1], L)) +
           he->z * fabsf(pp_vec3_dot(&axes[2], L));
}

/* dot(P - c, axis) */
static inline float pp_dot_rel(const PPVec3 *P, const PPVec3 *c, const PPVec3 *axis)
{
    return (P->x - c->x) * axis->x + (P->y - c->y) * axis->y + (P->z - c->z) * axis->z;
}

/* Clip polygon `in` (n verts) to the half-space { P : dot(P - c, axis) <= limit }
 * (Sutherland-Hodgman), writing survivors to `out`. Returns the new vertex count. */
static int pp_clip_halfspace(const PPVec3 *in, int n, PPVec3 *out,
                             const PPVec3 *c, const PPVec3 *axis, float limit)
{
    int m = 0;
    for (int i = 0; i < n; ++i) {
        const PPVec3 *A = &in[i];
        const PPVec3 *B = &in[(i + 1) % n];
        float da = pp_dot_rel(A, c, axis) - limit;
        float db = pp_dot_rel(B, c, axis) - limit;
        bool a_in = da <= 0.0f;
        bool b_in = db <= 0.0f;

        if (a_in && m < PP_MAX_BOX_CONTACTS * 2) {
            out[m++] = *A;
        }
        if (a_in != b_in && m < PP_MAX_BOX_CONTACTS * 2) {
            float denom = da - db;
            float u = (denom != 0.0f) ? da / denom : 0.0f;
            PPVec3 P = {.xyz = {A->x + u * (B->x - A->x),
                                A->y + u * (B->y - A->y),
                                A->z + u * (B->z - A->z)}};
            out[m++] = P;
        }
    }
    return m;
}

/* Closest points between segments (p1,q1) and (p2,q2); midpoint written to out. */
static void pp_closest_segment_segment(const PPVec3 *p1, const PPVec3 *q1,
                                       const PPVec3 *p2, const PPVec3 *q2, PPVec3 *out)
{
    PPVec3 d1, d2, r;
    pp_vec3_sub(q1, p1, &d1);
    pp_vec3_sub(q2, p2, &d2);
    pp_vec3_sub(p1, p2, &r);

    float a = pp_vec3_dot(&d1, &d1);
    float e = pp_vec3_dot(&d2, &d2);
    float f = pp_vec3_dot(&d2, &r);

    float s, tt;
    const float EPS = 1e-8f;
    if (a <= EPS && e <= EPS) {
        s = tt = 0.0f;
    } else if (a <= EPS) {
        s = 0.0f;
        tt = f / e;
    } else {
        float c = pp_vec3_dot(&d1, &r);
        if (e <= EPS) {
            tt = 0.0f;
            s = -c / a;
        } else {
            float b = pp_vec3_dot(&d1, &d2);
            float denom = a * e - b * b;
            s = (denom != 0.0f) ? (b * f - c * e) / denom : 0.0f;
            tt = (b * s + f) / e;
        }
    }
    s = s < 0.0f ? 0.0f : (s > 1.0f ? 1.0f : s);
    tt = tt < 0.0f ? 0.0f : (tt > 1.0f ? 1.0f : tt);

    PPVec3 c1 = {.xyz = {p1->x + d1.x * s, p1->y + d1.y * s, p1->z + d1.z * s}};
    PPVec3 c2 = {.xyz = {p2->x + d2.x * tt, p2->y + d2.y * tt, p2->z + d2.z * tt}};
    out->x = 0.5f * (c1.x + c2.x);
    out->y = 0.5f * (c1.y + c2.y);
    out->z = 0.5f * (c1.z + c2.z);
}

static bool pp_sat_box_box(const PPVec3 *posA, const PPQuaternion *rotA, const PPVec3 *heA, float brA,
                           const PPVec3 *posB, const PPQuaternion *rotB, const PPVec3 *heB, float brB,
                           PPBoxContact *out)
{
    /* Cheap bounding-sphere reject. */
    PPVec3 dc;
    pp_vec3_sub(posB, posA, &dc);
    float sum_br = brA + brB;
    if (pp_vec3_dot(&dc, &dc) > sum_br * sum_br) {
        return false;
    }

    PPVec3 ua[3], ub[3];
    pp_box_world_axes(rotA, ua);
    pp_box_world_axes(rotB, ub);

    float min_overlap = FLT_MAX;
    PPVec3 best_axis = {.xyz = {0.0f, 0.0f, 0.0f}};
    int best_type = -1;          /* 0: face of A, 1: face of B, 2: edge-edge */
    int best_index = -1;         /* face index for type 0/1 */
    int best_ea = -1, best_eb = -1;

    /* Face axes of A. */
    for (int i = 0; i < 3; ++i) {
        PPVec3 L = ua[i];
        float rA = heA->xyz[i];
        float rB = pp_box_project_radius(heB, ub, &L);
        float overlap = rA + rB - fabsf(pp_vec3_dot(&dc, &L));

        if (overlap <= 0.0f) {
            return false;
        }

        if (overlap < min_overlap) {
            min_overlap = overlap;
            best_axis = L;
            best_type = 0;
            best_index = i;
        }
    }
    /* Face axes of B. */
    for (int j = 0; j < 3; ++j) {
        PPVec3 L = ub[j];
        float rA = pp_box_project_radius(heA, ua, &L);
        float rB = heB->xyz[j];
        float overlap = rA + rB - fabsf(pp_vec3_dot(&dc, &L));

        if (overlap <= 0.0f) {
            return false;
        }

        if (overlap < min_overlap) {
            min_overlap = overlap;
            best_axis = L;
            best_type = 1;
            best_index = j;
        }
    }
    /* Edge-edge axes. A small relative bias keeps a face axis when an edge axis
     * is only marginally smaller, avoiding spurious edge contacts (and the
     * resulting jitter) on near-parallel faces. */
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            PPVec3 L;
            pp_vec3_cross(&ua[i], &ub[j], &L);
            float len = pp_vec3_length(&L);
            if (len < 1e-6f) {
                continue; /* near-parallel edges: degenerate axis */
            }

            pp_vec3_scale(&L, 1.0f / len, &L);
            float rA = pp_box_project_radius(heA, ua, &L);
            float rB = pp_box_project_radius(heB, ub, &L);
            float overlap = rA + rB - fabsf(pp_vec3_dot(&dc, &L));
            if (overlap <= 0.0f) {
                return false;
            }

            if (overlap < min_overlap * 0.999f) {
                min_overlap = overlap;
                best_axis = L;
                best_type = 2;
                best_index = -1;
                best_ea = i;
                best_eb = j;
            }
        }
    }

    /* Orient the normal from A toward B. */
    if (pp_vec3_dot(&dc, &best_axis) < 0.0f) {
        pp_vec3_neg(&best_axis, &best_axis);
    }
    out->normal = best_axis;
    out->count = 0;

    if (best_type == 2) {
        /* Edge-edge: one contact at the closest points of the supporting edges. */
        PPVec3 ptA = *posA;
        for (int k = 0; k < 3; ++k) {
            if (k == best_ea) {
                continue;
            }

            float sgn = (pp_vec3_dot(&ua[k], &best_axis) >= 0.0f) ? 1.0f : -1.0f;
            PPVec3 add;
            pp_vec3_scale(&ua[k], sgn * heA->xyz[k], &add);
            pp_vec3_add(&ptA, &add, &ptA);
        }

        PPVec3 ea;
        pp_vec3_scale(&ua[best_ea], heA->xyz[best_ea], &ea);

        PPVec3 a1, a2;
        pp_vec3_sub(&ptA, &ea, &a1);
        pp_vec3_add(&ptA, &ea, &a2);

        PPVec3 ptB = *posB;
        for (int k = 0; k < 3; ++k) {
            if (k == best_eb) {
                continue;
            }

            float sgn = (pp_vec3_dot(&ub[k], &best_axis) >= 0.0f) ? -1.0f : 1.0f;

            PPVec3 add;
            pp_vec3_scale(&ub[k], sgn * heB->xyz[k], &add);
            pp_vec3_add(&ptB, &add, &ptB);
        }

        PPVec3 eb;
        pp_vec3_scale(&ub[best_eb], heB->xyz[best_eb], &eb);

        PPVec3 b1, b2;
        pp_vec3_sub(&ptB, &eb, &b1);
        pp_vec3_add(&ptB, &eb, &b2);

        pp_closest_segment_segment(&a1, &a2, &b1, &b2, &out->points[0]);
        out->depths[0] = min_overlap;
        out->count = 1;
        return true;
    }

    /* Face case: clip the incident face against the reference face. */
    const PPVec3 *refPos, *refHe, *incPos, *incHe;
    const PPVec3 *refAxes, *incAxes;
    int ref_idx = best_index;
    PPVec3 refN; /* outward normal of the reference face */

    if (best_type == 0) {
        refPos = posA; refHe = heA; refAxes = ua;
        incPos = posB; incHe = heB; incAxes = ub;
        refN = best_axis;                 /* A's contacting face faces B (+normal) */
    } else {
        refPos = posB; refHe = heB; refAxes = ub;
        incPos = posA; incHe = heA; incAxes = ua;
        pp_vec3_neg(&best_axis, &refN);   /* B's contacting face faces A (-normal) */
    }

    /* Reference face centre and its two tangent axes. */
    PPVec3 faceCenter;
    {
        PPVec3 off;
        pp_vec3_scale(&refN, refHe->xyz[ref_idx], &off);
        pp_vec3_add(refPos, &off, &faceCenter);
    }
    int ta_i = (ref_idx + 1) % 3, tb_i = (ref_idx + 2) % 3;
    PPVec3 ta = refAxes[ta_i], tb = refAxes[tb_i];
    float hta = refHe->xyz[ta_i], htb = refHe->xyz[tb_i];

    /* Incident face: the face of the other box most anti-parallel to refN. */
    int inc_idx = 0;
    float min_dot = 0.0f;
    for (int k = 0; k < 3; ++k) {
        float d = pp_vec3_dot(&incAxes[k], &refN);
        if (k == 0 || fabsf(d) > fabsf(min_dot)) {
            min_dot = d;
            inc_idx = k;
        }
    }
    PPVec3 incN = incAxes[inc_idx];
    if (pp_vec3_dot(&incN, &refN) > 0.0f) {
        pp_vec3_neg(&incN, &incN); /* outward normal pointing back toward the ref box */
    }
    PPVec3 incCenter;
    {
        PPVec3 off;
        pp_vec3_scale(&incN, incHe->xyz[inc_idx], &off);
        pp_vec3_add(incPos, &off, &incCenter);
    }

    int ia_i = (inc_idx + 1) % 3;
    int ib_i = (inc_idx + 2) % 3;
    float hia = incHe->xyz[ia_i];
    float hib = incHe->xyz[ib_i];

    /* Four incident-face vertices, in order around the quad. */
    PPVec3 poly[PP_MAX_BOX_CONTACTS * 2];
    const float sa[4] = { 1.0f, 1.0f, -1.0f, -1.0f };
    const float sb[4] = { 1.0f, -1.0f, -1.0f, 1.0f };
    for (int v = 0; v < 4; ++v) {
        PPVec3 p = incCenter, add;
        pp_vec3_scale(&incAxes[ia_i], sa[v] * hia, &add);
        pp_vec3_add(&p, &add, &p);

        pp_vec3_scale(&incAxes[ib_i], sb[v] * hib, &add);
        pp_vec3_add(&p, &add, &p);
        poly[v] = p;
    }

    /* Clip against the reference face's four side planes. */
    PPVec3 buf[PP_MAX_BOX_CONTACTS * 2];
    PPVec3 ta_neg, tb_neg;
    pp_vec3_neg(&ta, &ta_neg);
    pp_vec3_neg(&tb, &tb_neg);
    int n = 4;
    n = pp_clip_halfspace(poly, n, buf, &faceCenter, &ta,     hta);
    n = pp_clip_halfspace(buf,  n, poly, &faceCenter, &ta_neg, hta);
    n = pp_clip_halfspace(poly, n, buf, &faceCenter, &tb,     htb);
    n = pp_clip_halfspace(buf,  n, poly, &faceCenter, &tb_neg, htb);

    /* Keep clipped points that lie behind the reference face (penetrating). */
    for (int i = 0; i < n && out->count < PP_MAX_BOX_CONTACTS; ++i) {
        float pen = -pp_dot_rel(&poly[i], &faceCenter, &refN);
        if (pen >= -1e-4f) {
            out->points[out->count] = poly[i];
            out->depths[out->count] = pen > 0.0f ? pen : 0.0f;
            out->count++;
        }
    }

    /* Degenerate fallback: if clipping produced nothing, use the deepest point. */
    if (out->count == 0) {
        out->points[0] = faceCenter;
        out->depths[0] = min_overlap;
        out->count = 1;
    }
    return true;
}

/* One-sided box vs triangle, matching the sphere/triangle convention: the
 * triangle is a one-sided face and the box is only resolved when its centre is
 * on the front (+normal) side. The box's incident face (the one most directly
 * facing the triangle) is clipped against the triangle's three edge planes, and
 * each surviving corner becomes a contact. Penetrating corners are real
 * contacts; corners still in front become *speculative* contacts when the box
 * is closing fast enough to cross this step, which stops fast boxes tunnelling.
 *
 * The contact normal points from the box into the triangle (-tri->n); the solver
 * then pushes the box out along +tri->n. out->depths[] are SIGNED: positive =
 * penetration, negative = remaining gap (the caller forms dist / separation). */
static int pp_sat_box_triangle(const PPVec3 *box_pos, const PPQuaternion *box_rot,
                               const PPVec3 *box_he, const PPTriangle *tri,
                               const PPVec3 *box_vel, float t, PPBoxContact *out)
{
    const PPVec3 *n = &tri->n;

    PPVec3 ub[3];
    pp_box_world_axes(box_rot, ub);

    /* One-sided: the box centre must be in front of the triangle plane. This
     * both implements the one-sided behaviour (a box approaching from behind is
     * ignored, like a sphere) and guarantees the incident face faces the
     * triangle. */
    float sd = pp_dot_rel(box_pos, &tri->v[0], n);
    if (sd <= 0.0f) {
        out->count = 0;
        return 0;
    }

    /* Incident face: box face whose outward normal is most opposed to n. */
    int inc = 0;
    float best = -1.0f;
    for (int k = 0; k < 3; ++k) {
        float a = fabsf(pp_vec3_dot(&ub[k], n));
        if (a > best) { best = a; inc = k; }
    }
    PPVec3 incN = ub[inc];
    if (pp_vec3_dot(&incN, n) > 0.0f) {
        pp_vec3_neg(&incN, &incN); /* point toward the triangle (-n side) */
    }

    PPVec3 faceCenter;
    {
        PPVec3 off; pp_vec3_scale(&incN, box_he->xyz[inc], &off);
        pp_vec3_add(box_pos, &off, &faceCenter);
    }
    int ia = (inc + 1) % 3, ib = (inc + 2) % 3;
    float hia = box_he->xyz[ia], hib = box_he->xyz[ib];

    /* Four incident-face corners. */
    PPVec3 bufA[PP_MAX_BOX_CONTACTS * 2], bufB[PP_MAX_BOX_CONTACTS * 2];
    const float sa[4] = { 1.0f, 1.0f, -1.0f, -1.0f };
    const float sb[4] = { 1.0f, -1.0f, -1.0f, 1.0f };
    for (int v = 0; v < 4; ++v) {
        PPVec3 p = faceCenter, add;
        pp_vec3_scale(&ub[ia], sa[v] * hia, &add); pp_vec3_add(&p, &add, &p);
        pp_vec3_scale(&ub[ib], sb[v] * hib, &add); pp_vec3_add(&p, &add, &p);
        bufA[v] = p;
    }

    /* Clip against the triangle's three edge half-spaces (each plane is
     * perpendicular to the triangle, through an edge, normal pointing outward).
     * A corner outside any edge is removed, so a box face that does not overlap
     * the triangle laterally yields no contacts. */
    const PPVec3 *V[3] = { &tri->v[0], &tri->v[1], &tri->v[2] };
    PPVec3 *src = bufA, *dst = bufB;
    int cnt = 4;
    for (int e = 0; e < 3; ++e) {
        const PPVec3 *a = V[e];
        const PPVec3 *b = V[(e + 1) % 3];
        const PPVec3 *opp = V[(e + 2) % 3];
        PPVec3 edge; pp_vec3_sub(b, a, &edge);
        PPVec3 m; pp_vec3_cross(&edge, n, &m);
        PPVec3 to_opp; pp_vec3_sub(opp, a, &to_opp);
        if (pp_vec3_dot(&m, &to_opp) > 0.0f) {
            pp_vec3_neg(&m, &m); /* orient outward (away from the interior) */
        }
        cnt = pp_clip_halfspace(src, cnt, dst, a, &m, 0.0f);
        PPVec3 *tmp = src; src = dst; dst = tmp;
        if (cnt == 0) break;
    }

    pp_vec3_neg(n, &out->normal); /* box -> triangle */
    out->count = 0;
    if (cnt == 0) {
        return 0;
    }

    /* Approach speed: velocity component heading into the front face. */
    float approach = -pp_vec3_dot(box_vel, n);

    /* Penetrating corners form the real, multi-point manifold (needed for a
     * stable resting box). While the box is still entirely in front, emit a
     * SINGLE speculative contact placed at the box centre projected onto the
     * triangle plane. Two reasons: braking N speculative contacts independently
     * over-applies the impulse and launches the box; and on a multi-triangle
     * floor a box straddling an edge would otherwise get one speculative contact
     * per triangle at conflicting offset positions. The projected centre is the
     * same point for every coplanar triangle, so they reinforce instead of
     * fighting, and it carries ~no lever arm so the brake adds no spin. */
    int gap_n = 0;
    float min_gap = FLT_MAX;

    for (int i = 0; i < cnt; ++i) {
        float front = pp_dot_rel(&src[i], &tri->v[0], n); /* >0 in front, <0 behind */
        if (front <= 0.0f) {
            /* Penetrating: real contact, depth = how far behind the plane. */
            if (out->count < PP_MAX_BOX_CONTACTS) {
                out->points[out->count] = src[i];
                out->depths[out->count] = -front;
                out->count++;
            }
        } else {
            gap_n++;
            if (front < min_gap) min_gap = front;
        }
    }

    if (out->count == 0 && gap_n > 0 && approach > 0.0f && min_gap <= approach * t) {
        PPVec3 off, cp;
        pp_vec3_scale(n, sd, &off);
        pp_vec3_sub(box_pos, &off, &cp); /* box centre projected onto the plane */
        out->points[0] = cp;
        out->depths[0] = -min_gap; /* negative -> gap -> caller forms separation */
        out->count = 1;
    }
    return out->count;
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

        // Shape/triangle collisions: iterate shapes of this body. Spheres and
        // boxes both collide with the static triangle soup.
        for (int si = 0; si < lhs_body->shape_count; ++si) {
            const PPShape *lhs_shape = &lhs_body->shapes[si];
            if (lhs_shape->type != PP_OBJECT_TYPE_SPHERE &&
                lhs_shape->type != PP_OBJECT_TYPE_BOX) {
                continue;
            }

            PPVec3 shape_pos;
            pp_shape_world_pos(lhs_body, lhs_shape, &shape_pos);

            float radius = (lhs_shape->type == PP_OBJECT_TYPE_SPHERE)
                               ? lhs_shape->sphere.radius
                               : lhs_shape->box.bounding_radius;
            float pos_x = shape_pos.x;
            float pos_y = shape_pos.y;
            float pos_z = shape_pos.z;

            // Expand the broadphase by the distance the shape may travel this
            // step. A fast mover that the static radius check would cull is
            // still considered, so it can be caught speculatively before it
            // tunnels through a triangle.
            float speed = pp_vec3_length(&lhs_body->vel);
            float reach = radius + speed * t;
            float reach_sq = reach * reach;

            // Box/triangle contacts are deduplicated across triangles for this
            // shape: a box spanning a shared edge of two coplanar triangles
            // would otherwise get the same corner contact from both, and those
            // doubled contacts make the solver converge to a wrong, energy-
            // injecting state. Track emitted (point,normal) pairs and skip
            // near-coincident repeats.
            PPVec3 emitted_pt[32];
            PPVec3 emitted_n[32];
            int emitted_count = 0;

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

                if (lhs_shape->type == PP_OBJECT_TYPE_SPHERE) {
                    PPVec3 p, d;
                    pp_vec3_scale(&tri->n, -1.0f, &d);
                    float dist; // perpendicular distance from sphere centre to plane
                    if (pp_tri_intersect(tri, &shape_pos, &d, &p, &dist)) {
                        float separation = dist - lhs_shape->sphere.radius;

                        // Either the sphere is already touching/penetrating, or it
                        // has a gap but is approaching fast enough to close it this
                        // step -- a speculative contact that stops fast spheres
                        // tunnelling through the triangle.
                        bool contact = false;
                        if (separation <= 0.0f) {
                            contact = true;
                        } else {
                            float approach = -pp_vec3_dot(&lhs_body->vel, &tri->n);
                            if (approach > 0.0f && separation <= approach * t) {
                                contact = true;
                            }
                        }

                        if (contact) {
                            PPCollision c;
                            pp_fill_collision_info_sphere_triangle(lhs_body, lhs_shape,
                                                                   tri, &p,
                                                                   lhs_shape->sphere.radius - dist,
                                                                   &c);

                            bool respond = true;
                            if (cb) {
                                respond = cb->collision_callback(lhs_body, tri,
                                                                 lhs_shape->kind, tri->kind,
                                                                 &c, cb->user_data);
                            }

                            if (respond) {
                                manifolds[manifold_count++] = c;
                            }
                        }
                    }
                } else {
                    // Box vs triangle via SAT, one-sided, with a clipped
                    // multi-point manifold and speculative contacts.
                    PPBoxContact bc;
                    int nc = pp_sat_box_triangle(&shape_pos, &lhs_body->rot,
                                                 &lhs_shape->box.half_extents, tri,
                                                 &lhs_body->vel, t, &bc);
                    if (nc > 0) {
                        int deepest = 0;
                        for (int k = 1; k < nc; ++k) {
                            if (bc.depths[k] > bc.depths[deepest]) deepest = k;
                        }

                        PPCollision rep;
                        rep.n = bc.normal;
                        rep.p = bc.points[deepest];
                        rep.dist = bc.depths[deepest];
                        rep.separation = -bc.depths[deepest];
                        rep.obj1 = lhs_body;
                        rep.obj2 = &trimesh_body;
                        rep.obj1_bounce = lhs_body->bounce;
                        rep.obj2_bounce = tri->bounce;
                        rep.obj1_friction = lhs_body->friction;
                        rep.obj2_friction = tri->friction;
                        rep.type1 = PP_OBJECT_TYPE_BOX;
                        rep.type2 = PP_OBJECT_TYPE_TRIANGLE;
                        rep.kind1 = lhs_shape->kind;
                        rep.kind2 = tri->kind;

                        bool respond = true;
                        if (cb) {
                            respond = cb->collision_callback(lhs_body, tri,
                                                             lhs_shape->kind, tri->kind,
                                                             &rep, cb->user_data);
                        }

                        if (respond) {
                            for (int k = 0; k < nc &&
                                            manifold_count < PICOPHYSICS_MAX_MANIFOLDS; ++k) {
                                // Skip a contact coincident with one already
                                // emitted for this shape with the same normal.
                                bool dup = false;
                                for (int e = 0; e < emitted_count; ++e) {
                                    PPVec3 diff;
                                    pp_vec3_sub(&bc.points[k], &emitted_pt[e], &diff);
                                    if (pp_vec3_length_sq(&diff) < 1e-6f &&
                                        pp_vec3_dot(&bc.normal, &emitted_n[e]) > 0.999f) {
                                        dup = true;
                                        break;
                                    }
                                }
                                if (dup) continue;

                                if (emitted_count < 32) {
                                    emitted_pt[emitted_count] = bc.points[k];
                                    emitted_n[emitted_count] = bc.normal;
                                    emitted_count++;
                                }

                                PPCollision c = rep;
                                c.p = bc.points[k];
                                c.dist = bc.depths[k];
                                c.separation = -bc.depths[k];
                                manifolds[manifold_count++] = c;
                            }
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
                        // Box vs Box via SAT, producing a multi-point manifold.
                        PPBoxContact bc;
                        if (pp_sat_box_box(&lhs_spos, &lhs_body->rot, &lhs_shape->box.half_extents,
                                           lhs_shape->box.bounding_radius,
                                           &rhs_spos, &rhs_body->rot, &rhs_shape->box.half_extents,
                                           rhs_shape->box.bounding_radius, &bc)) {
                            // The callback decides per-pair; query it once using the
                            // deepest contact as the representative collision.
                            int deepest = 0;
                            for (int k = 1; k < bc.count; ++k) {
                                if (bc.depths[k] > bc.depths[deepest]) deepest = k;
                            }

                            bool respond = true;
                            PPCollision rep;
                            rep.p = bc.points[deepest];
                            rep.n = bc.normal;
                            rep.dist = bc.depths[deepest];
                            rep.separation = -bc.depths[deepest];
                            rep.obj1 = lhs_body;
                            rep.obj2 = rhs_body;
                            rep.obj1_bounce = lhs_body->bounce;
                            rep.obj2_bounce = rhs_body->bounce;
                            rep.obj1_friction = lhs_body->friction;
                            rep.obj2_friction = rhs_body->friction;
                            rep.type1 = PP_OBJECT_TYPE_BOX;
                            rep.type2 = PP_OBJECT_TYPE_BOX;
                            rep.kind1 = lhs_shape->kind;
                            rep.kind2 = rhs_shape->kind;

                            if (cb) {
                                respond = cb->collision_callback(lhs_body, rhs_body,
                                                                 lhs_shape->kind, rhs_shape->kind,
                                                                 &rep, cb->user_data);
                            }

                            if (respond) {
                                // Emit one manifold per contact point; they share
                                // the normal but carry their own contact point and
                                // penetration so the solver can resist tipping.
                                for (int k = 0; k < bc.count &&
                                                 manifold_count < PICOPHYSICS_MAX_MANIFOLDS; ++k) {
                                    PPCollision c = rep;
                                    c.p = bc.points[k];
                                    c.dist = bc.depths[k];
                                    c.separation = -bc.depths[k];
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
