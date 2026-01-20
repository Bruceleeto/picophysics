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
 * - Create fully dynamic spheres and boxes and apply linear and angular forces
 * - Create environments using triangles and boxes
 * - Easy to use collision callback system to respond to detected collisions and to
 *   choose whether to respond at at all (return true to respond)
 * - Ray casting
 * - Axis-locking of rotations
 * - Bodies have friction and bounciness coefficients
 * - Very fast, no dynamic memory allocations
 *
 * # Overview
 *
 * Picophysics only supports the following primitives:
 *
 * - Spheres (fully dynamic and responsive)
 * - Boxes (only collides with other boxes and spheres)
 * - Triangles (static environment)
 *
 * Bodies are not composable; there's no separation between a body and a collider like in other
 * physics engines there are simply Spheres and Boxes which react to each other and the environment.
 *
 * Currently Spheres are the only fully dynamic and responsive object. Ray-casting
 * the world is also supported.
 *
 * There is only one global "world", all things are created with in it, and you can empty
 * it with pp_physics_clear(). You also don't need to initialise the world, just start creating
 * bodies and call pp_physics_step(dt) to update.
 *
 * The library statically allocates memory, by default you can have:
 *
 *  - 32 bodies (spheres + boxes)
 *  - 128 triangles
 *
 * If you need more than that you can define PHYSICS_MAX_OBJECTS or PHYSICS_MAX_TRIANGLES
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
 * issues in the collision response code. I would like objects to roll correctly, and for
 * friction to impact angular velocity (currently only angular damping is respected) if you
 * can help fix it, I'd appreciate it!
 *
 * # Roadmap
 *
 * - Add proper angular collision response
 * - Proper manifold generation for GJK/EPA
 * - Allow objects to be marked as static
 * - Broad-phase collision detection (spatial hashing)
 * - Add fixed and spring joints (links) between objects
 * - Simplify/share collision response logic across all things
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

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef struct _PPVec3 {
    union {
        struct {
            float x, y, z;
        };
        float xyz[3];
    };
} PPVec3;

typedef struct _PPQuaternion {
    union {
        struct {
            float x, y, z, w;
        };
        float xyzw[4];
    };
} PPQuaternion;

typedef struct _PPMat3 {
    float m[9];
} PPMat3;

typedef struct _PPPlane {
    PPVec3 n;
    float d;
} PPPlane;

struct _PPSphere;

typedef uint8_t BodyKind;

typedef enum _PPObjectType {
    PP_OBJECT_TYPE_SPHERE,
    PP_OBJECT_TYPE_BOX,
    PP_OBJECT_TYPE_TRIANGLE,
} PPObjectType;

typedef struct _PPBody PPBody;

typedef struct _PPCollision {
    PPVec3 p;
    PPVec3 n;
    PPBody* obj1;
    PPBody* obj2;

    PPObjectType type1;
    PPObjectType type2;

    BodyKind kind1;
    BodyKind kind2;

    float dist; // Distance between object CoM
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

typedef struct _PPBody {
    PPObjectType type;

    PPVec3 pos;
    float bounce;
    PPQuaternion rot;

    PPVec3 vel;
    PPVec3 acc;
    float damping;

    PPVec3 a_vel;
    PPVec3 a_acc;
    float a_damping;

    float mass;
    float friction;
    PPMat3 inertia;
    float inv_mass;

    BodyKind kind;

    PPAxisLock lock;

    bool is_alive;
    void* user_data;
} PPBody;


typedef struct _PPSphere {
    PPBody body;
    float radius;
} PPSphere;

typedef struct _PPBox {
    PPBody body;
    PPVec3 whd; // Width/height/depth

    float radius;  // Used to shortcut collisions
} PPBox;

typedef struct _PPTriangle {
    PPVec3 v[3];
    PPVec3 n;
    PPPlane p;
    BodyKind kind;
    float friction;
} PPTriangle;

#define PP_BODY(p) ((PPBody*) p)
#define PP_SPHERE(p) ((p->type == PP_OBJECT_TYPE_SPHERE) ? (PPSphere*) p : NULL)
#define PP_BOX(p) ((p->type == PP_OBJECT_TYPE_BOX) ? (PPBox*) p : NULL)

PPVec3* pp_vec3_init(PPVec3* v);
PPVec3* pp_vec3_set(PPVec3* v, float x, float y, float z);
PPVec3* pp_vec3_scale(const PPVec3* v1, float t, PPVec3* out);
PPVec3* pp_vec3_assign(PPVec3* target, const PPVec3* source);
bool pp_vec3_normalize(PPVec3* target);
float pp_vec3_length(const PPVec3* v1);

PPQuaternion* pp_quat_init(PPQuaternion* q);
void pp_quat_between(const PPVec3* v0, const PPVec3* q1, PPQuaternion* result);
void pp_quat_slerp(const PPQuaternion* q0, const PPQuaternion* q1, float t, PPQuaternion* result);
PPQuaternion* pp_quat_assign(PPQuaternion* target, const PPQuaternion* source);
float pp_quat_angle_between(const PPQuaternion* q0, const PPQuaternion* q1);

void pp_physics_step(float t);
bool pp_physics_ray_intersect(const PPVec3* origin, const PPVec3* direction, BodyKind* ignore_kinds, const PPBody** body_hit, const PPTriangle** tri_hit, float* distance, PPVec3* intersection);
void pp_physics_clear();
void pp_physics_set_gravity(const PPVec3* v);
bool pp_physics_collision_map_add(BodyKind kind1, BodyKind kind2, void* user_data, bool (*callback)(const void*, const void*, BodyKind, BodyKind, const PPCollision* c, const void*));

PPTriangle* pp_physics_create_triangle(const PPVec3* v1, const PPVec3* v2, const PPVec3* v3, BodyKind kind);
size_t pp_physics_triangle_count();
const PPTriangle* pp_physics_triangle_at(size_t i);

PPSphere* pp_physics_create_sphere(float radius, const PPVec3* pos, float mass, BodyKind kind);
float pp_sphere_get_radius(const PPSphere* s);

PPBox* pp_physics_create_box(float width, float height, float depth, const PPVec3* pos, float mass, BodyKind kind);
float pp_box_get_width(const PPBox* b);
float pp_box_get_height(const PPBox* b);
float pp_box_get_depth(const PPBox* b);

void pp_physics_destroy_body(PPBody* b);
const PPBody* pp_physics_body_at(size_t i);
size_t pp_physics_body_count();
size_t pp_physics_body_total_count();
bool pp_body_set_bounce(PPBody* s, float b);
void pp_body_add_force(PPBody* s, float x, float y, float z);
void pp_body_add_angular_force(PPBody* s, float x, float y, float z);
void pp_body_lock_axis(PPBody* s, PPAxisLock lock);
void pp_body_set_angular_damping(PPBody* s, float d);
void pp_body_set_damping(PPBody* s, float d);
void pp_body_get_forward(PPBody* s, PPVec3* f);
void pp_body_set_position(PPBody* s, float x, float y, float z);
void pp_body_get_position(PPBody* s, PPVec3* pos);
void pp_body_set_rotation(PPBody* s, float x, float y, float z, float w);
void pp_body_get_rotation(PPBody* s, PPQuaternion* rot);
float pp_body_get_radius(PPBody* s);
void pp_body_set_user_data(PPBody* s, void* data);
void* pp_body_get_user_data(const PPBody* s);
bool pp_body_set_friction(PPBody* s, float f);
void pp_body_get_velocity(const PPBody* s, PPVec3* vel);
void pp_body_get_velocity_at_position(const PPBody* b, const PPVec3* p, PPVec3* ret);
void pp_body_set_angular_velocity(PPBody* s, float x, float y, float z);
void pp_body_set_velocity(PPBody* s, float x, float y, float z);
void pp_body_set_angular_acceleration(PPBody* s, float x, float y, float z);
void pp_body_set_acceleration(PPBody* s, float x, float y, float z);
void pp_body_look_at(PPBody* s, float x, float y, float z);

#ifdef __cplusplus
}
#endif

#endif

#ifdef PICOPHYSICS_IMPLEMENTATION

#define EPA_DEBUG 0

#ifndef PHYSICS_MAX_OBJECTS
    #define PHYSICS_MAX_OBJECTS 32
#endif

#ifndef PHYSICS_MAX_TRIANGLES
    #define PHYSICS_MAX_TRIANGLES 128
#endif

typedef union _PPObject {
    struct _PPSphere s;
    struct _PPBox b;
} PPObject;

static PPObject objects[PHYSICS_MAX_OBJECTS];
static int object_count = 0;
static int dead_object_count = 0;

static PPTriangle tris[PHYSICS_MAX_TRIANGLES];
static int tri_count = 0;

static struct _PPCollisionMapEntry {
    BodyKind kind1;
    BodyKind kind2;
    void* user_data;
    bool (*collision_callback)(const void*, const void*, BodyKind, BodyKind, const PPCollision* c, const void*);
} collision_map[32];

static int collision_map_count = 0;

static PPVec3 gravity = {.xyz = {0.0f, 0.0f, 0.0f}};
static float gravity_magnitude = 0.0f;

PPVec3* pp_vec3_init(PPVec3* v) {
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
    return v;
}

PPVec3* pp_vec3_set(PPVec3* v, float x, float y, float z) {
    v->x = x;
    v->y = y;
    v->z = z;
    return v;
}

PPVec3* pp_vec3_assign(PPVec3* target, const PPVec3* source) {
    pp_vec3_set(target, source->x, source->y, source->z);
    return target;
}

PPVec3* pp_vec3_add(const PPVec3* v1, const PPVec3* v2, PPVec3* out) {
    out->x = v1->x + v2->x;
    out->y = v1->y + v2->y;
    out->z = v1->z + v2->z;
    return out;
}

PPVec3* pp_vec3_sub(const PPVec3* v1, const PPVec3* v2, PPVec3* out) {
    out->x = v1->x - v2->x;
    out->y = v1->y - v2->y;
    out->z = v1->z - v2->z;
    return out;
}

PPVec3* pp_vec3_scale(const PPVec3* v1, float t, PPVec3* out) {
    out->x = v1->x * t;
    out->y = v1->y * t;
    out->z = v1->z * t;
    return out;
}

PPVec3* pp_vec3_neg(const PPVec3* v1, PPVec3* out) {
    out->x = -v1->x;
    out->y = -v1->y;
    out->z = -v1->z;
    return out;
}

float pp_vec3_length(const PPVec3* v1) {
    return sqrtf(v1->x * v1->x + v1->y * v1->y + v1->z * v1->z);
}

float pp_vec3_length_sq(const PPVec3* v1) {
    return v1->x * v1->x + v1->y * v1->y + v1->z * v1->z;
}

float pp_vec3_dist(const PPVec3* v1, const PPVec3* v2) {
    PPVec3 tmp;
    pp_vec3_sub(v2, v1, &tmp);
    return pp_vec3_length(&tmp);
}

PPVec3* pp_vec3_cross(const PPVec3 *v1, const PPVec3 *v2, PPVec3 *out) {
    assert(v1 != out);
    assert(v2 != out);

    out->x = v1->y * v2->z - v1->z * v2->y;
    out->y = v1->z * v2->x - v1->x * v2->z;
    out->z = v1->x * v2->y - v1->y * v2->x;
    return out;
}

float pp_vec3_dot(const PPVec3 *v1, const PPVec3 *v2) {
    return v1->x * v2->x +
           v1->y * v2->y +
           v1->z * v2->z;
}

bool pp_vec3_normalize(PPVec3 *v) {
    float length = pp_vec3_length(v);

    // Check for zero-length vector to avoid division by zero
    if (length > 0.0f) {
        v->x /= length;
        v->y /= length;
        v->z /= length;
        return true;
    } else {
        pp_vec3_init(v);
        return false;
    }
}

PPQuaternion* pp_quat_init(PPQuaternion* q) {
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
    q->w = 1.0f;
    return q;
}

PPQuaternion* pp_quat_set(PPQuaternion* q, float x, float y, float z, float w) {
    q->x = x;
    q->y = y;
    q->z = z;
    q->w = w;
    return q;
}

PPQuaternion* pp_quat_assign(PPQuaternion* target, const PPQuaternion* source) {
    pp_quat_set(target, source->x, source->y, source->z, source->w);
    return target;
}

void pp_quat_from_angular_velocity(const PPVec3* a_vel, float dt, PPQuaternion* q_rot) {
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

void pp_quat_multiply(const PPQuaternion* q1, const PPQuaternion* q2, PPQuaternion* result) {
    PPQuaternion tmp;
    tmp.xyzw[0] = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    tmp.xyzw[1] = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    tmp.xyzw[2] = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    tmp.xyzw[3] = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;

    *result = tmp;
}

float pp_quat_angle_between(const PPQuaternion* q1, const PPQuaternion* q2) {
    float dot = q1->w * q2->w + q1->x * q2->x + q1->y * q2->y + q1->z * q2->z;
    return acosf(dot) * 2.0f;
}

void pp_quat_transform(const PPQuaternion* q, const PPVec3* v, PPVec3* ret) {
    PPQuaternion conjugate, vq, temp, v_rot_q;
    pp_quat_set(&conjugate, -q->x, -q->y, -q->z, q->w);
    pp_quat_set(&vq, v->x, v->y, v->z, 0.0f);
    pp_quat_multiply(q, &vq, &temp);
    pp_quat_multiply(&temp, &conjugate, &v_rot_q);
    pp_vec3_set(ret, v_rot_q.x, v_rot_q.y, v_rot_q.z);
}

void pp_quat_normalize(PPQuaternion* q) {
    float norm = sqrtf(q->x * q->x + q->y * q->y + q->z * q->z + q->w * q->w);
    if (norm > 0) {
        q->x /= norm;
        q->y /= norm;
        q->z /= norm;
        q->w /= norm;
    }
}

void pp_quat_forward(const PPQuaternion* q, PPVec3* out)
{
    float x = q->x;
    float y = q->y;
    float z = q->z;
    float w = q->w;

    out->x = 2.0f * (x * z + w * y);
    out->y = 2.0f * (y * z - w * x);
    out->z = 1.0f - 2.0f * (x * x + y * y);
}

void pp_quat_between(const PPVec3* v0, const PPVec3* v1, PPQuaternion* result) {
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

void pp_quat_slerp(const PPQuaternion* q0, const PPQuaternion* q1, float t, PPQuaternion* result) {
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

static float pp_plane_distance(const PPVec3* n, const float d, const PPVec3* p) {
    float numerator = fabsf(n->x * p->x + n->y * p->y + n->z * p->z + d);
    float denominator = sqrtf(n->x * n->x + n->y * n->y + n->z * n->z);
    return numerator / denominator;
}

bool pp_box_intersect(const PPBox* sphere, const PPVec3* o, const PPVec3* d, PPVec3* out, float* distance);
bool pp_sphere_intersect(const PPSphere* sphere, const PPVec3* o, const PPVec3* d, PPVec3* out, float* distance);
bool pp_tri_intersect(const PPTriangle* tri, const PPVec3* o, const PPVec3* d, PPVec3* out, float* distance);

bool pp_contains_kind(BodyKind* kinds, BodyKind kind) {
    if(!kinds) {
        return false;
    }

    BodyKind* k = kinds;
    while(*k) {
        if(*k == kind) {
            return true;
        }
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
 bool pp_physics_ray_intersect(const PPVec3* origin, const PPVec3* direction, BodyKind* ignore_kinds, const PPBody** body_hit, const PPTriangle** tri_hit, float* distance, PPVec3* intersection) {
     float closest_dist = FLT_MAX;
     const PPTriangle* closest_tri = NULL;
     const PPBody* closest_body = NULL;
     PPVec3 closest_intersection;

     for(int i = 0; i < pp_physics_triangle_count(); ++i) {
         const PPTriangle* t = pp_physics_triangle_at(i);

         PPVec3 hit;
         float dist;
         if(pp_tri_intersect(t, origin, direction, &hit, &dist)){
             if(pp_contains_kind(ignore_kinds, t->kind)) {
                 continue;
             }

             if(dist < closest_dist) {
                 closest_tri = t;
                 closest_dist = dist;
                 pp_vec3_assign(&closest_intersection, &hit);
             }
         }
     }

     for(int i = 0; i < pp_physics_body_total_count(); ++i) {
         const PPBody* body = pp_physics_body_at(i);
         if(!body->is_alive) {
             continue;
         }

         if(body->type == PP_OBJECT_TYPE_SPHERE) {
             const PPSphere* s = PP_SPHERE(body);

             assert(s);
             PPVec3 hit;
             float dist;
             if(pp_sphere_intersect(s, origin, direction, &hit, &dist)) {
                 if(pp_contains_kind(ignore_kinds, s->body.kind)) {
                     continue;
                 }

                 if(dist < closest_dist) {
                     closest_body = body;
                     closest_tri = NULL;
                     closest_dist = dist;
                     pp_vec3_assign(&closest_intersection, &hit);
                 }
             }
         } else if(body->type == PP_OBJECT_TYPE_BOX) {
             const PPBox* b = PP_BOX(body);

             assert(b);
             PPVec3 hit;
             float dist;
             if(pp_box_intersect(b, origin, direction, &hit, &dist)) {
                 if(pp_contains_kind(ignore_kinds, b->body.kind)) {
                     continue;
                 }

                 if(dist < closest_dist) {
                     closest_body = body;
                     closest_tri = NULL;
                     closest_dist = dist;
                     pp_vec3_assign(&closest_intersection, &hit);
                 }
             }
         }
     }

     if(closest_dist == FLT_MAX) {
         return false;
     }

     if(body_hit) {
         *body_hit = closest_body;
     }

     if(tri_hit) {
         *tri_hit = (PPTriangle*) closest_tri;
     }
     if(distance) {
         *distance = closest_dist;
     }
     if(intersection) {
         pp_vec3_assign(intersection, &closest_intersection);
     }
     return true;
}

bool pp_aabb_intersect(const PPVec3* pos, const PPVec3* whd, const PPVec3* origin, const PPVec3* direction, PPVec3* out, float* distance) {
    PPVec3 extents = {whd->x * 0.5f, whd->y * 0.5f, whd->z * 0.5f};
    PPVec3 min = {pos->x - extents.x, pos->y - extents.y, pos->z - extents.z};
    PPVec3 max = {pos->x + extents.x, pos->y + extents.y, pos->z + extents.z};
    PPVec3 n_inv = {1.0f / direction->x, 1.0f / direction->y, 1.0f / direction->z};

    const float t1 = (min.x - origin->x) * n_inv.x;
    const float t2 = (max.x - origin->x) * n_inv.x;
    const float t3 = (min.y - origin->y) * n_inv.y;
    const float t4 = (max.y - origin->y) * n_inv.y;
    const float t5 = (min.z - origin->z) * n_inv.z;
    const float t6 = (max.z - origin->z) * n_inv.z;

    const float tmin = fmax(fmax(fmin(t1, t2), fmin(t3, t4)), fmin(t5, t6));
    const float tmax = fmin(fmin(fmax(t1, t2), fmax(t3, t4)), fmax(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behind us
    if(tmax < 0) {
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

bool pp_box_intersect(const PPBox* box, const PPVec3* o, const PPVec3* d, PPVec3* out, float* distance) {
    // Transform the ray into the box's local space
    PPVec3 local_origin;
    PPVec3 local_direction;
    pp_vec3_sub(o, &box->body.pos, &local_origin);
    pp_quat_transform(&box->body.rot, &local_origin, &local_origin);
    pp_quat_transform(&box->body.rot, d, &local_direction);

    return pp_aabb_intersect(&box->body.pos, &box->whd, o, d, out, distance);
}

bool pp_sphere_intersect(const PPSphere* sphere, const PPVec3* o, const PPVec3* d, PPVec3* out, float* distance) {
    PPVec3 oc;
    pp_vec3_sub(&sphere->body.pos, o, &oc);

    float b = pp_vec3_dot(&oc, d);
    float c = pp_vec3_dot(&oc, &oc) - sphere->radius * sphere->radius;

    float discriminant = b * b - c;

    // No matter what, always init the distance
    if(distance) {
        *distance = FLT_MAX;
    }

    // If the discriminant is negative, there are no real roots, so no intersection
    if (discriminant < 0) {
        return false;
    }

    // Calculate the two points of intersection
    float sqrtDiscriminant = sqrtf(discriminant);
    float t1 = -b - sqrtDiscriminant;
    float t2 = -b + sqrtDiscriminant;

    // Check if the points of intersection are in front of the ray origin
    if (t1 > 0 && (t2 <= 0 || t1 < t2)) {
        PPVec3 add;
        pp_vec3_scale(d, t1, &add);
        pp_vec3_add(o, &add, out);
        return true;
    } else if (t2 > 0) {
        PPVec3 add;
        pp_vec3_scale(d, t2, &add);
        pp_vec3_add(o, &add, out);
        return true;
    }

    return false;
}

bool pp_tri_intersect(const PPTriangle* tri, const PPVec3* o, const PPVec3* d, PPVec3* out, float* distance) {
    // PPVec3 v0v1, v0v2, pvec;
    // pp_vec3_sub(&tri->v[1], &tri->v[0], &v0v1);
    // pp_vec3_sub(&tri->v[1], &tri->v[0], &v0v1);
    // pp_vec3_cross(d, &v0v2, &pvec);
    // float det = pp_vec3_dot(&v0v1, &pvec);

    // // If the determinant is negative, the triangle is back-facing.
    // // If the determinant is close to 0, the ray misses the triangle.
    // // if (det < FLT_EPSILON) return false;

    // // If det is close to 0, the ray and triangle are parallel.
    // if (fabs(det) < FLT_EPSILON) {
    //     return false;
    // }

    // float invDet = 1 / det;

    // PPVec3 tvec;
    // pp_vec3_sub(o, &tri->v[0], &tvec);
    // float u = pp_vec3_dot(&tvec, &pvec) * invDet;

    // if (u < 0 || u > 1) {
    //     return false;
    // }

    // PPVec3 qvec;
    // pp_vec3_cross(&tvec, &v0v1, &qvec);

    // float v = pp_vec3_dot(d, &qvec) * invDet;
    // if (v < 0 || u + v > 1) {
    //     return false;
    // }

    // float t = pp_vec3_dot(&v0v2, &qvec) * invDet;

    // return true;

    const float e = FLT_EPSILON;
    PPVec3 edge1, edge2, cross_e1, cross_e2, s;
    pp_vec3_sub(&tri->v[1], &tri->v[0], &edge1);
    pp_vec3_sub(&tri->v[2], &tri->v[0], &edge2);
    pp_vec3_cross(d, &edge2, &cross_e2);

    float det = pp_vec3_dot(&edge1, &cross_e2);

    if(det > -e && det < e) {
        return false;
    }

    float inv_det = 1.0f / det;
    pp_vec3_sub(o, &tri->v[0], &s);
    float u = inv_det * pp_vec3_dot(&s, &cross_e2);

    if ((u < 0 && fabsf(u) > e) || (u > 1 && fabsf(u-1) > e)) {
        return false;
    }

    pp_vec3_cross(&s, &edge1, &cross_e1);
    float v = inv_det * pp_vec3_dot(d, &cross_e1);

    if ((v < 0 && fabsf(v) > e) || (u + v > 1 && fabsf(u + v - 1) > e)) {
        return false;
    }

    float t = inv_det * pp_vec3_dot(&edge2, &cross_e1);

    if(t <= e) {
        return false;
    }

    pp_vec3_scale(d, t, out);
    pp_vec3_add(out, o, out);

    if(distance) {
        *distance = t;
    }

    return true;
}

void pp_body_set_angular_velocity(PPBody* s, float x, float y, float z) {
    pp_vec3_set(&s->a_vel, x, y, z);
}

void pp_body_set_velocity(PPBody* s, float x, float y, float z) {
    pp_vec3_set(&s->vel, x, y, z);
}

void pp_body_set_angular_acceleration(PPBody* s, float x, float y, float z) {
    pp_vec3_set(&s->a_acc, x, y, z);
}

void pp_body_set_acceleration(PPBody* s, float x, float y, float z) {
    pp_vec3_set(&s->acc, x, y, z);
}

void pp_body_set_angular_damping(PPBody* s, float d) {
    if(d < 0.0f || d > 1.0f) {
        return;
    }

    s->a_damping = d;
}

void pp_body_set_damping(PPBody* s, float d) {
    if(d < 0.0f || d > 1.0f) {
        return;
    }

    s->damping = d;
}

void pp_body_get_forward(PPBody* s, PPVec3* f) {
    pp_quat_forward(&s->rot, f);
}

float pp_box_get_width(const PPBox* b) {
    return b->whd.x;
}

float pp_box_get_height(const PPBox* b) {
    return b->whd.y;
}

float pp_box_get_depth(const PPBox* b) {
    return b->whd.z;
}

float pp_sphere_get_radius(const PPSphere* s) {
    return s->radius;
}

void pp_quat_from_axis_angle(PPQuaternion* q, const PPVec3* axis, float angle) {
    PPVec3 a;
    pp_vec3_assign(&a, axis);
    pp_vec3_normalize(&a);

    float half = angle * 0.5f;
    float s = sinf(half);   // sin(θ/2)
    float c = cosf(half);   // cos(θ/2)

    q->x = a.xyz[0] * s;   // axis.x * sin(θ/2)
    q->y = a.xyz[1] * s;   // axis.y * sin(θ/2)
    q->z = a.xyz[2] * s;   // axis.z * sin(θ/2)
    q->w = c;         // cos(θ/2)

    pp_quat_normalize(q);
}

void pp_body_look_at(PPBody* s, float x, float y, float z) {
    PPVec3 t, f, c;
    pp_vec3_set(&t, x, y, z);
    pp_body_get_forward(s, &f);
    pp_vec3_cross(&f, &t, &c);
    float d = pp_vec3_dot(&f, &t);

    // Build error quaternion
    PPQuaternion q_err;
    if (d < -0.9999f) {                     // opposite direction
        PPVec3 ortho, axis;
        if(fabsf(f.xyz[0]) < 0.9f) {
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

static void pp_body_init(PPBody* body, const PPVec3* pos, float mass, BodyKind kind) {
    body->is_alive = true;
    body->user_data = NULL;

    pp_vec3_init(&body->vel);
    pp_vec3_init(&body->acc);
    pp_vec3_init(&body->a_vel);
    pp_vec3_init(&body->a_acc);
    pp_quat_init(&body->rot);
    pp_vec3_set(&body->pos, pos->x, pos->y, pos->z);
    body->kind = kind;
    body->mass = mass;
    body->inv_mass = 1.0f / mass;
    body->friction = 0.3f;
    body->damping = 0.01f;
    body->a_damping = 0.02f;
    pp_body_set_bounce(body, 0.5f);
}

PPSphere* pp_sphere_init(PPSphere* s, float radius, const PPVec3* pos, float mass, BodyKind kind) {
    s->radius = radius;
    s->body.type = PP_OBJECT_TYPE_SPHERE;
    s->body.inertia.m[0] = (2.0f / 5.0f) * mass * radius * radius;
    pp_body_init(&s->body, pos, mass, kind);
    return s;
}

PPBox* pp_box_init(PPBox* s, float width, float height, float depth, const PPVec3* pos, float mass, BodyKind kind) {
    s->body.type = PP_OBJECT_TYPE_BOX;

    float w2 = width * width;
    float h2 = height * height;
    float d2 = depth * depth;
    const float oot = 1.0f / 12.0f;

    memset(s->body.inertia.m, 0, sizeof(s->body.inertia.m));
    s->body.inertia.m[0] = oot * mass * (h2 + w2);
    s->body.inertia.m[4] = oot * mass * (d2 + h2);
    s->body.inertia.m[8] = oot * mass * (d2 + w2);

    pp_body_init(&s->body, pos, mass, kind);

    pp_vec3_set(&s->whd, width, height, depth);
    s->radius = pp_vec3_length(&s->whd);

    return s;
}

void pp_body_set_user_data(PPBody* s, void* data) {
    s->user_data = data;
}

void* pp_body_get_user_data(const PPBody* s) {
    return s->user_data;
}

void pp_body_lock_axis(PPBody* s, PPAxisLock lock) {
    s->lock = lock;
}

void pp_body_get_position(PPBody *s, PPVec3* pos) {
    pp_vec3_assign(pos, &s->pos);
}

void pp_body_set_position(PPBody *s, float x, float y, float z) {
    pp_vec3_set(&s->pos, x, y, z);
}

void pp_body_set_rotation(PPBody *s, float x, float y, float z, float w) {
    pp_quat_set(&s->rot, x, y, z, w);
}

void pp_body_get_velocity_at_position(const PPBody* b, const PPVec3* p, PPVec3* ret) {
    PPVec3 rel_pos, local_rel_pos, a_vel_contrib;
    pp_vec3_sub(p, &b->pos, &rel_pos);
    pp_vec3_cross(&b->a_vel, &local_rel_pos, &a_vel_contrib);
    pp_vec3_add(&b->pos, &a_vel_contrib, ret);
}

void pp_body_get_velocity(const PPBody* s, PPVec3* vel) {
    pp_vec3_assign(vel, &s->vel);
}

void pp_body_get_rotation(PPBody* s, PPQuaternion* rot) {
    pp_quat_assign(rot, &s->rot);
}

bool pp_body_set_bounce(PPBody* s, float b) {
    if(!s || b < 0.0f || b > 1.0f) {
        return false;
    }

    s->bounce = b;
    return true;
}

void pp_body_add_force(PPBody* s, float x, float y, float z) {
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

void pp_body_add_angular_force(PPBody* s, float tx, float ty, float tz)
{
    PPVec3 torque;
    pp_vec3_set(&torque, tx, ty, tz);

    // FIXME: Implement for boxes!
    // Simplified inertia for Spheres
    float I = s->inertia.m[0];

    if (I <= 0.0f) return;   // nothing to do for mass‑less or zero‑radius objects

    PPVec3 ang_acc;
    pp_vec3_scale(&torque, 1.0f / I, &ang_acc);
    pp_vec3_add(&s->a_acc, &ang_acc, &s->a_acc);
}

const struct _PPCollisionMapEntry* pp_physics_collision_map_search(BodyKind kind1, BodyKind kind2) {
    for(int i = 0; i < collision_map_count; ++i) {
        struct _PPCollisionMapEntry* entry = &collision_map[i];
        if((entry->kind1 == kind1 && entry->kind2 == kind2) || (entry->kind2 == kind1 && entry->kind1 == kind2)) {
            return entry;
        }
    }

    return NULL;
}

bool pp_physics_collision_map_add(BodyKind kind1, BodyKind kind2, void* user_data, bool (*callback)(const void*, const void*, BodyKind, BodyKind, const PPCollision* c, const void*)) {
    if(!pp_physics_collision_map_search(kind1, kind2)) {
        struct _PPCollisionMapEntry* entry = &collision_map[collision_map_count++];
        entry->kind1 = kind1;
        entry->kind2 = kind2;
        entry->user_data = user_data;
        entry->collision_callback = callback;
        return true;
    }

    return false;
}

void pp_fill_collision_info_sphere_box(const PPSphere* lhs, const PPBox* rhs, const PPVec3* contact_point, const PPVec3* n, PPCollision* c, float d) {
    c->dist = d;
    pp_vec3_assign(&c->p, contact_point);
    pp_vec3_assign(&c->n, n);
    c->obj1 = PP_BODY(lhs);
    c->obj2 = PP_BODY(rhs);
    c->type1 = PP_OBJECT_TYPE_SPHERE;
    c->type2 = PP_OBJECT_TYPE_BOX;
    c->kind1 = lhs->body.kind;
    c->kind2 = rhs->body.kind;
}

void pp_fill_collision_info_sphere_sphere(const PPSphere* lhs, const PPSphere* rhs, float dist, PPCollision* c) {
    pp_vec3_sub(&lhs->body.pos, &rhs->body.pos, &c->n);
    c->dist = dist;
    if(dist > 0) {
        c->n.xyz[0] /= dist;
        c->n.xyz[1] /= dist;
        c->n.xyz[2] /= dist;
    }

    float total_radius = lhs->radius + rhs->radius;
    float wr1 = lhs->radius / total_radius;
    float wr2 = rhs->radius / total_radius;

    for(int i = 0; i < 3; ++i) {
        c->p.xyz[i] = lhs->body.pos.xyz[i] * wr1 + rhs->body.pos.xyz[i] * wr2;
    }

    c->obj1 = PP_BODY(lhs);
    c->obj2 = PP_BODY(rhs);
    c->type1 = PP_OBJECT_TYPE_SPHERE;
    c->type2 = PP_OBJECT_TYPE_SPHERE;
    c->kind1 = lhs->body.kind;
    c->kind2 = rhs->body.kind;
}

void pp_fill_collision_info_sphere_triangle(const PPSphere* lhs, const PPTriangle* tri, const PPVec3* p, float dist, PPCollision* c) {
    pp_vec3_scale(&tri->n, 1.0f, &c->n); // Copy
    pp_vec3_scale(p, 1.0f, &c->p); // Copy
    c->dist = dist;
    c->obj1 = PP_BODY(lhs);
    c->obj2 = NULL;
    c->type1 = PP_OBJECT_TYPE_SPHERE;
    c->type2 = PP_OBJECT_TYPE_TRIANGLE;
    c->kind1 = lhs->body.kind;
    c->kind2 = tri->kind;
}

PPSphere* pp_physics_create_sphere(float radius, const PPVec3* pos, float mass, BodyKind kind) {
    PPSphere* ret = NULL;

    if(dead_object_count) {
        for(int i = 0; i < object_count; ++i) {
            PPBody* body = (PPBody*) &objects[i];
            if(!body->is_alive) {
                dead_object_count--;
                ret = &objects[i].s;
                break;
            }
        }
    }

    if(!ret) {
        PPObject* obj = &objects[object_count++];
        ret = (PPSphere*) &obj->s;
        assert(ret);
    }

    pp_sphere_init(ret, radius, pos, mass, kind);
    return ret;
}

PPBox* pp_physics_create_box(float width, float height, float depth, const PPVec3* pos, float mass, BodyKind kind) {
    PPBox* ret = NULL;

    if(dead_object_count) {
        for(int i = 0; i < object_count; ++i) {
            PPBody* body = (PPBody*) &objects[i];
            if(!body->is_alive) {
                dead_object_count--;
                ret = &objects[i].b;
                break;
            }
        }
    }

    if(!ret) {
        PPObject* obj = &objects[object_count++];
        ret = (PPBox*) &obj->b;
        assert(ret);
    }

    pp_box_init(ret, width, height, depth, pos, mass, kind);
    return ret;
}


PPTriangle* pp_physics_create_triangle(const PPVec3* v1, const PPVec3* v2, const PPVec3* v3, BodyKind kind) {
    PPTriangle* tri = &tris[tri_count++];
    pp_vec3_assign(&tri->v[0], v1);
    pp_vec3_assign(&tri->v[1], v2);
    pp_vec3_assign(&tri->v[2], v3);

    PPVec3 e1, e2;
    pp_vec3_sub(v2, v1, &e1);
    pp_vec3_sub(v3, v1, &e2);

    pp_vec3_cross(&e1, &e2, &tri->n);
    pp_vec3_normalize(&tri->n);

    tri->friction = 0.1f;
    tri->kind = kind;

    return tri;
}

void pp_physics_clear() {
    tri_count = 0;
    object_count = 0;
    dead_object_count = 0;
    memset(tris, 0, sizeof(tris));
    memset(objects, 0, sizeof(objects));
}

size_t pp_physics_triangle_count() {
    return tri_count;
}

size_t pp_physics_body_count() {
    return object_count - dead_object_count;
}

size_t pp_physics_body_total_count() {
    return object_count;
}

const PPBody* pp_physics_body_at(size_t i) {
    return (const PPBody*) (objects + i);
}

const PPTriangle* pp_physics_triangle_at(size_t i) {
    return tris + i;
}

void pp_physics_destroy_body(PPBody* b) {
    if(!b) {
        return;
    }

    b->is_alive = false;
    ++dead_object_count;
}

void pp_physics_set_gravity(const PPVec3* v) {
    pp_vec3_assign(&gravity, v);
    gravity_magnitude = pp_vec3_length(&gravity);
}


static void pp_solve(const PPCollision* manifold) {
    PPBody* lhs = PP_BODY(manifold->obj1);
    PPBody* rhs = PP_BODY(manifold->obj2);

    float overlap = manifold->dist;

    PPVec3 adjustment_lhs, adjustment_rhs;

    float vel_along_normal = 0.0f;
    PPVec3 rel_vel = {.xyz={0.0f, 0.0f, 0.0f}};

    if(rhs) {
        pp_vec3_scale(&manifold->n, overlap * 0.5f, &adjustment_lhs);
        pp_vec3_scale(&manifold->n, overlap * 0.5f, &adjustment_rhs);
        pp_vec3_add(&lhs->pos, &adjustment_lhs, &lhs->pos);
        pp_vec3_sub(&rhs->pos, &adjustment_rhs, &rhs->pos);

        pp_vec3_sub(&rhs->vel, &lhs->vel, &rel_vel);   // v_rhs – v_lhs
        vel_along_normal = pp_vec3_dot(&rel_vel, &manifold->n);
    } else {
        pp_vec3_scale(&manifold->n, overlap, &adjustment_lhs);
        pp_vec3_add(&lhs->pos, &adjustment_lhs, &lhs->pos);
        pp_vec3_assign(&rel_vel, &lhs->vel);

        vel_along_normal = pp_vec3_dot(&lhs->vel, &manifold->n);
    }

    if (vel_along_normal < 0) {
        PPVec3 penetration, tangent;
        pp_vec3_scale(&manifold->n, vel_along_normal, &penetration);
        pp_vec3_sub(&rel_vel, &penetration, &tangent);

        // Moving towards each other
        float r = fmax(lhs->bounce, (rhs) ? rhs->bounce : 0.0f);
        float f = fmin(lhs->friction, (rhs) ? rhs->friction : 10000.0f);

        float inv_mass_sum = lhs->inv_mass + ((rhs && rhs->inv_mass) ? rhs->inv_mass : 0.0f);
        float j_n = -(1.0f + r) * vel_along_normal / inv_mass_sum;
        PPVec3 impulse_n;
        pp_vec3_scale(&manifold->n, j_n, &impulse_n);

        float jt = -pp_vec3_dot(&rel_vel, &tangent) / inv_mass_sum;
        jt = fmax(-j_n * f, fmin(jt, j_n * f));   // clamp to μ·|j_n|
        PPVec3 impulse_t;
        pp_vec3_normalize(&tangent);      // ensure unit tangent
        pp_vec3_scale(&tangent, jt, &impulse_t);

        PPVec3 impulse;
        pp_vec3_add(&impulse_n, &impulse_t, &impulse);

        PPVec3 dv_lhs, dv_rhs;
        pp_vec3_scale(&impulse, lhs->inv_mass, &dv_lhs);
        pp_vec3_add(&lhs->vel, &dv_lhs, &lhs->vel);   // v_lhs ← v_lhs + Δv

        if(rhs) {
            pp_vec3_scale(&impulse, rhs->inv_mass, &dv_rhs);
            pp_vec3_sub(&rhs->vel, &dv_rhs, &rhs->vel);   // v_rhs ← v_rhs + Δv
        }
    }
}

static void pp_sphere_box_response(PPSphere* sphere, PPBox* box, const PPCollision* c, float t) {
    // c->dist is the distance between the sphere center and the contact point
    // so the "overlap" is the radius minus the distance
    float overlap = sphere->radius - c->dist;

    PPVec3 adjustment_lhs, adjustment_rhs;
    pp_vec3_scale(&c->n, overlap * 0.5f, &adjustment_lhs);
    pp_vec3_scale(&c->n, overlap * 0.5f, &adjustment_rhs);
    pp_vec3_add(&sphere->body.pos, &adjustment_lhs, &sphere->body.pos);
    pp_vec3_sub(&box->body.pos, &adjustment_rhs, &box->body.pos);

    PPVec3 rel_vel;
    pp_vec3_sub(&box->body.vel, &sphere->body.vel, &rel_vel);   // v_rhs – v_lhs
    float vel_along_normal = pp_vec3_dot(&rel_vel, &c->n);
    if (vel_along_normal < 0) {
        PPVec3 penetration, tangent;
        pp_vec3_scale(&c->n, vel_along_normal, &penetration);
        pp_vec3_sub(&rel_vel, &penetration, &tangent);

        // Moving towards each other
        float r = fmax(sphere->body.bounce, box->body.bounce);
        float f = fmin(sphere->body.friction, box->body.friction);

        float inv_mass_sum = sphere->body.inv_mass + box->body.inv_mass;
        float j_n = -(1.0f + r) * vel_along_normal / inv_mass_sum;
        PPVec3 impulse_n;
        pp_vec3_scale(&c->n, j_n, &impulse_n);

        float jt = -pp_vec3_dot(&rel_vel, &tangent) / inv_mass_sum;
        jt = fmax(-j_n * f, fmin(jt, j_n * f));   // clamp to μ·|j_n|
        PPVec3 impulse_t;
        pp_vec3_normalize(&tangent);      // ensure unit tangent
        pp_vec3_scale(&tangent, jt, &impulse_t);

        PPVec3 impulse;
        pp_vec3_add(&impulse_n, &impulse_t, &impulse);

        PPVec3 dv_lhs, dv_rhs;
        pp_vec3_scale(&impulse, sphere->body.inv_mass, &dv_lhs);
        pp_vec3_scale(&impulse, box->body.inv_mass, &dv_rhs);

        pp_vec3_add(&sphere->body.vel, &dv_lhs, &sphere->body.vel);   // v_lhs ← v_lhs + Δv
        pp_vec3_sub(&box->body.vel, &dv_rhs, &box->body.vel);   // v_rhs ← v_rhs + Δv
    }
}

static void pp_sphere_sphere_response(PPSphere* lhs, PPSphere* rhs, const PPCollision* c) {
    float overlap = (lhs->radius + rhs->radius) - c->dist;

    PPVec3 adjustment_lhs, adjustment_rhs;
    pp_vec3_scale(&c->n, overlap * 0.5f, &adjustment_lhs);
    pp_vec3_scale(&c->n, overlap * 0.5f, &adjustment_rhs);
    pp_vec3_add(&lhs->body.pos, &adjustment_lhs, &lhs->body.pos);
    pp_vec3_sub(&rhs->body.pos, &adjustment_rhs, &rhs->body.pos);

    PPVec3 rel_vel;
    pp_vec3_sub(&rhs->body.vel, &lhs->body.vel, &rel_vel);   // v_rhs – v_lhs
    float vel_along_normal = pp_vec3_dot(&rel_vel, &c->n);
    if (vel_along_normal < 0) {
        PPVec3 penetration, tangent;
        pp_vec3_scale(&c->n, vel_along_normal, &penetration);
        pp_vec3_sub(&rel_vel, &penetration, &tangent);

        // Moving towards each other
        float r = fmax(lhs->body.bounce, rhs->body.bounce);
        float f = fmin(lhs->body.friction, rhs->body.friction);

        float inv_mass_sum = lhs->body.inv_mass + rhs->body.inv_mass;
        float j_n = -(1.0f + r) * vel_along_normal / inv_mass_sum;
        PPVec3 impulse_n;
        pp_vec3_scale(&c->n, j_n, &impulse_n);

        float jt = -pp_vec3_dot(&rel_vel, &tangent) / inv_mass_sum;
        jt = fmax(-j_n * f, fmin(jt, j_n * f));   // clamp to μ·|j_n|
        PPVec3 impulse_t;
        pp_vec3_normalize(&tangent);      // ensure unit tangent
        pp_vec3_scale(&tangent, jt, &impulse_t);

        PPVec3 impulse;
        pp_vec3_add(&impulse_n, &impulse_t, &impulse);

        PPVec3 dv_lhs, dv_rhs;
        pp_vec3_scale(&impulse, lhs->body.inv_mass, &dv_lhs);
        pp_vec3_scale(&impulse, rhs->body.inv_mass, &dv_rhs);

        pp_vec3_add(&lhs->body.vel, &dv_lhs, &lhs->body.vel);   // v_lhs ← v_lhs + Δv
        pp_vec3_sub(&rhs->body.vel, &dv_rhs, &rhs->body.vel);   // v_rhs ← v_rhs + Δv
    }
}

typedef struct _PPSupportPoint {
    PPVec3 point;
    PPVec3 a;
    PPVec3 b;
} PPSupportPoint;

typedef struct _PPSimplex {
    PPSupportPoint points[4];
    size_t count;
} PPSimplex;

typedef struct _PPPolytopeFace {
    union {
        struct {
            uint8_t a;
            uint8_t b;
            uint8_t c;
        };
        uint8_t abc[3];
    };
    PPVec3 n;
    float d;
} PPPolytopeFace;

typedef struct _PPPolytopeEdge {
    uint8_t a;
    uint8_t b;
} PPPolytopeEdge;

#define MAX_POLYTOPE_FACES 64
#define MAX_POLYTOPE_POINTS 128
#define MAX_POLYTOPE_EDGES 32

typedef struct _PPPolytope {
    PPSupportPoint points[MAX_POLYTOPE_POINTS];
    size_t point_count;

    PPPolytopeFace faces[MAX_POLYTOPE_FACES];
    size_t face_count;

    PPPolytopeEdge edges[MAX_POLYTOPE_EDGES];
    size_t edge_count;
} PPPolytope;

static inline bool pp_same_direction(const PPVec3* direction, const PPVec3* ao) {
    return pp_vec3_dot(direction, ao) > 0.0f;
}

PPSimplex* pp_simplex_init(PPSimplex* s) {
    s->count = 0;
    return s;
}

bool pp_simplex_push(PPSimplex* simplex, const PPSupportPoint* point) {
    assert(simplex->count < 4);

    memcpy(&simplex->points[3], &simplex->points[2], sizeof(PPSupportPoint));
    memcpy(&simplex->points[2], &simplex->points[1], sizeof(PPSupportPoint));
    memcpy(&simplex->points[1], &simplex->points[0], sizeof(PPSupportPoint));
    memcpy(&simplex->points[0], point, sizeof(PPSupportPoint));
    simplex->count++;

    return true;
}

PPSupportPoint* pp_simplex_at(PPSimplex* simplex, size_t i) {
    assert(i < simplex->count);
    return &simplex->points[i];
}

void pp_simplex_replace1(PPSimplex* simplex, const PPSupportPoint* point) {
    simplex->count = 1;
    memcpy(&simplex->points[0], point, sizeof(PPSupportPoint));
}

void pp_simplex_replace2(PPSimplex* simplex, const PPSupportPoint* p0, const PPSupportPoint* p1) {
    simplex->count = 2;

    // Temp variables are necessary as p0 or p1 may be
    // the points we're replacing
    PPSupportPoint t0, t1;
    memcpy(&t0, p0, sizeof(PPSupportPoint));
    memcpy(&t1, p1, sizeof(PPSupportPoint));

    memcpy(&simplex->points[0], &t0, sizeof(PPSupportPoint));
    memcpy(&simplex->points[1], &t1, sizeof(PPSupportPoint));
}

void pp_simplex_replace3(PPSimplex* simplex, const PPSupportPoint* p0, const PPSupportPoint* p1, const PPSupportPoint* p2) {
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

bool pp_simplex_next_line(PPSimplex* simplex, PPVec3* direction) {
    PPVec3 ab, ao;

    PPSupportPoint* a = pp_simplex_at(simplex, 0);
    PPSupportPoint* b = pp_simplex_at(simplex, 1);

    pp_vec3_sub(&b->point, &a->point, &ab);
    pp_vec3_neg(&a->point, &ao);

    if(pp_same_direction(&ab, &ao)) {
        PPVec3 t;
        pp_vec3_cross(&ab, &ao, &t);
        pp_vec3_cross(&t, &ab, direction);
    } else {
        pp_simplex_replace1(simplex, a);
        pp_vec3_assign(direction, &ao);
    }

    return false;
}

bool pp_simplex_next_triangle(PPSimplex* simplex, PPVec3* direction) {
    PPVec3 ab, ac, ao, abc;

    PPSupportPoint* a = pp_simplex_at(simplex, 0);
    PPSupportPoint* b = pp_simplex_at(simplex, 1);
    PPSupportPoint* c = pp_simplex_at(simplex, 2);

    pp_vec3_sub(&b->point, &a->point, &ab);
    pp_vec3_sub(&c->point, &a->point, &ac);
    pp_vec3_neg(&a->point, &ao);
    pp_vec3_cross(&ab, &ac, &abc);

    PPVec3 n;
    pp_vec3_cross(&abc, &ac, &n);
    if(pp_same_direction(&n, &ao)) {
        if(pp_same_direction(&ac, &ao)) {
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
        if(pp_same_direction(&n, &ao)) {
            pp_simplex_replace2(simplex, a, b);
            return pp_simplex_next_line(simplex, direction);
        } else {
            if(pp_same_direction(&abc, &ao)) {
                pp_vec3_assign(direction, &abc);
            } else {
                pp_simplex_replace3(simplex, a, c, b);
                pp_vec3_neg(&abc, direction);
            }
        }
    }

    return false;
}

bool pp_simplex_next_tetrahedron(PPSimplex* simplex, PPVec3* direction) {
    assert(simplex->count == 4);

    PPVec3 ab, ac, ad, ao;
    PPVec3 abc, acd, adb;

    PPSupportPoint* a = pp_simplex_at(simplex, 0);
    PPSupportPoint* b = pp_simplex_at(simplex, 1);
    PPSupportPoint* c = pp_simplex_at(simplex, 2);
    PPSupportPoint* d = pp_simplex_at(simplex, 3);

    pp_vec3_sub(&b->point, &a->point, &ab);
    pp_vec3_sub(&c->point, &a->point, &ac);
    pp_vec3_sub(&d->point, &a->point, &ad);
    pp_vec3_neg(&a->point, &ao);

    pp_vec3_cross(&ab, &ac, &abc);
    if(pp_same_direction(&abc, &ao)) {
        // FIXME: this is just a size reduction. Potential optimisation.
        pp_simplex_replace3(simplex, a, b, c);
        return pp_simplex_next_triangle(simplex, direction);
    }

    pp_vec3_cross(&ac, &ad, &acd);
    if(pp_same_direction(&acd, &ao)) {
        pp_simplex_replace3(simplex, a, c, d);
        return pp_simplex_next_triangle(simplex, direction);
    }

    pp_vec3_cross(&ad, &ab, &adb);
    if(pp_same_direction(&adb, &ao)) {
        pp_simplex_replace3(simplex, a, d, b);
        return pp_simplex_next_triangle(simplex, direction);
    }

    return true;
}

bool pp_simplex_next(PPSimplex* simplex, PPVec3* direction) {
    if(simplex->count == 2) {
        return pp_simplex_next_line(simplex, direction);
    } else if(simplex->count == 3) {
        return pp_simplex_next_triangle(simplex, direction);
    } else if(simplex->count == 4) {
        return pp_simplex_next_tetrahedron(simplex, direction);
    }

    return false;
}

void pp_find_furthest_point_sphere(const PPSphere* sphere, const PPVec3* direction, PPVec3* point) {
    PPVec3 ndir;
    pp_vec3_assign(&ndir, direction);
    pp_vec3_normalize(&ndir);

    pp_vec3_scale(&ndir, sphere->radius, &ndir);
    pp_vec3_add(&sphere->body.pos, &ndir, point);
}

void pp_find_furthest_point_box(const PPBox* box, const PPVec3* direction, PPVec3* point) {
    PPVec3 extents = {0};
    extents.x = box->whd.x * 0.5f;
    extents.y = box->whd.y * 0.5f;
    extents.z = box->whd.z * 0.5f;

    PPVec3 vertices[8];

    for(int i = 0; i < 8; ++i) {
        vertices[i].xyz[0] = (i & 1) ? extents.x : -extents.x;
        vertices[i].xyz[1] = (i & 2) ? extents.y : -extents.y;
        vertices[i].xyz[2] = (i & 4) ? extents.z : -extents.z;

        pp_quat_transform(&box->body.rot, &vertices[i], &vertices[i]);
        pp_vec3_add(&vertices[i], &box->body.pos, &vertices[i]);
    }

    float max_dot = pp_vec3_dot(direction, &vertices[0]);
    int max_index = 0;

    for(int i = 1; i < 8; ++i) {
        float dot = pp_vec3_dot(direction, &vertices[i]);
        if(dot > max_dot) {
            max_dot = dot;
            max_index = i;
        }
    }

    pp_vec3_assign(point, &vertices[max_index]);
}

bool pp_gjk_support(const PPBody* b1, const PPBody* b2, const PPVec3* direction, PPSupportPoint* out) {
    if(b1->type == PP_OBJECT_TYPE_BOX && b2->type == PP_OBJECT_TYPE_BOX) {
        PPVec3 reverse;
        pp_vec3_neg(direction, &reverse);
        pp_find_furthest_point_box(PP_BOX(b1), direction, &out->a);
        pp_find_furthest_point_box(PP_BOX(b2), &reverse, &out->b);
        pp_vec3_sub(&out->a, &out->b, &out->point);
        return true;
    } else if(b1->type == PP_OBJECT_TYPE_BOX && b2->type == PP_OBJECT_TYPE_SPHERE) {
        PPVec3 reverse, first, second;
        pp_vec3_neg(direction, &reverse);
        pp_find_furthest_point_box(PP_BOX(b1), direction, &out->a);
        pp_find_furthest_point_sphere(PP_SPHERE(b2), &reverse, &out->b);
        pp_vec3_sub(&out->a, &out->b, &out->point);
        return true;
    } else if(b1->type == PP_OBJECT_TYPE_SPHERE && b2->type == PP_OBJECT_TYPE_BOX) {
        PPVec3 reverse, first, second;
        pp_vec3_neg(direction, &reverse);
        pp_find_furthest_point_sphere(PP_SPHERE(b1), direction, &out->a);
        pp_find_furthest_point_box(PP_BOX(b2), &reverse, &out->b);
        pp_vec3_sub(&out->a, &out->b, &out->point);
        return true;
    }

    return false;
}

bool pp_gjk_collide(const PPBody* b1, const PPBody* b2, PPSimplex* simplex) {

    // ab = a.pos - b.pos
    //  c = Vector3(ab.z, ab.z, -ab.x - ab.y)
    //  if c == Vector3.zero():
    //      c = Vector3(-ab.y - ab.z, ab.x, ab.x)

    PPVec3 ab, c, abc;
    pp_vec3_sub(&b1->pos, &b2->pos, &ab);
    pp_vec3_set(&c, ab.z, ab.z, -ab.x - ab.y);
    if(pp_vec3_length_sq(&c) < FLT_EPSILON) {
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
    for(i = 0; i < 50; ++i) {
        if (!pp_gjk_support(b1, b2, &direction, &support)) {
            return false;
        }

        if (pp_vec3_dot(&support.point, &direction) <= 0) {
            return false;
        }

        pp_simplex_push(simplex, &support);

        if(pp_simplex_next(simplex, &direction)) {
            return true;
        }
    }

    if(i == 50) {
        fprintf(stderr, "Max iterations reached\n");
        return false;
    }

    return false;
}

void pp_polytope_clear_edges(PPPolytope* polytope) {
    polytope->edge_count = 0;
}

void pp_polytope_push_edge(PPPolytope* polytope, uint8_t a, uint8_t b) {
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
    for (int i = 0; i < 3; ++i) {
        out->xyz[i] = (a->xyz[i] + b->xyz[i] + c->xyz[i]) / 3;
    }
}

bool pp_polytope_calc_face_normal(PPPolytope* polytope, PPPolytopeFace* face) {
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

    if(face->d < 0) {
        pp_vec3_neg(&face->n, &face->n);
        face->d *= -1.0f;
    }

    return true;
}

float pp_polytope_push_face(PPPolytope* polytope, uint8_t a, uint8_t b, uint8_t c) {
    assert(polytope->face_count < MAX_POLYTOPE_FACES - 1);
    assert(a != b);
    assert(a != c);

    PPPolytopeFace* new_face = &polytope->faces[polytope->face_count];
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

void pp_polytope_erase_face(PPPolytope* polytope, size_t face_index) {
    assert(face_index < polytope->face_count);
    assert(polytope->face_count > 0);

    const PPPolytopeFace *back = &polytope->faces[polytope->face_count - 1];
    PPPolytopeFace* erase = &polytope->faces[face_index];
    memcpy(erase, back, sizeof(PPPolytopeFace));

    polytope->face_count--;
}

void pp_polytope_write(const PPPolytope *polytope, const char *filename)
{
    FILE *out = fopen(filename, "wt");
    for (int i = 0; i < polytope->point_count; ++i) {
        fprintf(out,
                "v %f %f %f\n",
                polytope->points[i].point.x,
                polytope->points[i].point.y,
                polytope->points[i].point.z);
    }

    for (int i = 0; i < polytope->face_count; ++i) {
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

int pp_find_min_face(const PPPolytope* polytope) {
    int min_face = 0;
    float min_dot = FLT_MAX;

    for (int i = 0; i < polytope->face_count; i++) {
        const PPPolytopeFace* face = &polytope->faces[i];
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

    PPPolytope polytope = {.faces = {},
                           .face_count = 0,
                           .edge_count = 0};

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

    const int MAX_ITERATIONS = 10;

    PPPolytopeFace* min_face_ptr = NULL;
    for(int i = 0; i < MAX_ITERATIONS; ++i) {
        min_face_ptr = &polytope.faces[min_face];

#if EPA_DEBUG
        char filename[100];
        sprintf(filename, "%d.obj", i);
        pp_polytope_write(&polytope, filename);
#endif

        PPSupportPoint support;
        bool ok = pp_gjk_support(lhs, rhs, &min_face_ptr->n, &support);
        assert(ok);
        if (!ok) {
            continue;
        }

        float s_dist = pp_vec3_dot(&support.point, &min_face_ptr->n);

        // If the support point is further from the origin than the
        // current min face, then we need to expand the polytope
        bool expand = s_dist > min_face_ptr->d + 0.001f;
        if (expand) {
            pp_polytope_clear_edges(&polytope);

            for (size_t i = 0; i < polytope.face_count; ++i) {
                PPVec3 dir;
                pp_vec3_sub(&support.point, &polytope.points[polytope.faces[i].a].point, &dir);

                if (pp_same_direction(&polytope.faces[i].n, &dir)) {
                    assert(polytope.faces[i].a != polytope.faces[i].b);
                    assert(polytope.faces[i].a != polytope.faces[i].c);

                    pp_polytope_push_edge(&polytope, polytope.faces[i].a, polytope.faces[i].b);
                    pp_polytope_push_edge(&polytope, polytope.faces[i].b, polytope.faces[i].c);
                    pp_polytope_push_edge(&polytope, polytope.faces[i].c, polytope.faces[i].a);

                    pp_polytope_erase_face(&polytope, i);

                    --i;
                }
            }

            if (!polytope.edge_count) {
                fprintf(stderr, "No edges found\n");
                break;
            }

            size_t new_face_index = polytope.face_count;
            size_t new_point_index = polytope.point_count;

            memcpy(&polytope.points[polytope.point_count++], &support, sizeof(PPSupportPoint));

            for(size_t i = 0; i < polytope.edge_count; ++i) {
                assert(polytope.edges[i].a != new_point_index);
                assert(polytope.edges[i].a != polytope.edges[i].b);

                float d = pp_polytope_push_face(&polytope,
                                      polytope.edges[i].a,
                                      polytope.edges[i].b,
                                      new_point_index);

                if(d < (min_face_ptr->d - 0.001f)) {
                    min_face = polytope.face_count - 1;
                    min_face_ptr = &polytope.faces[min_face];
                }
            }
        } else {
            break;
        }
    }

    if(!min_face_ptr) {
        return false;
    }

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
    pp_vec3_neg(&min_face_ptr->n, n);

    return true;
}

static inline bool flt_close(const float a, const float b) {
    return (a + FLT_EPSILON > b) && (a - FLT_EPSILON) < b;
}

bool pp_sphere_box_intersect(const PPSphere* lhs, const PPBox* rhs, PPVec3* contact_point, PPVec3* n, float* intersection) {
    PPVec3 diff;
    pp_vec3_sub(&lhs->body.pos, &rhs->body.pos, &diff);

    float diff_dist_sq = pp_vec3_dot(&diff, &diff);
    float sphere_radius = lhs->radius;
    float box_radius = rhs->radius;

    if (diff_dist_sq > (sphere_radius + box_radius) * (sphere_radius + box_radius)) {
        return false;
    }

    PPVec3 sphere_center_local;
    PPVec3 box_center = rhs->body.pos;
    PPVec3 sphere_center = lhs->body.pos;
    PPVec3 tmp;
    pp_vec3_sub(&sphere_center, &box_center, &tmp);

    // Inverse rotate by box orientation
    PPQuaternion box_rot = rhs->body.rot;
    PPQuaternion box_rot_inv = box_rot;
    box_rot_inv.x = -box_rot.x;
    box_rot_inv.y = -box_rot.y;
    box_rot_inv.z = -box_rot.z;

    pp_quat_transform(&box_rot_inv, &tmp, &sphere_center_local);

    // Clamp sphere center to box extents (local space)
    PPVec3 half_extents = { rhs->whd.x * 0.5f, rhs->whd.y * 0.5f, rhs->whd.z * 0.5f };
    PPVec3 closest_local = sphere_center_local;
    if (closest_local.x < -half_extents.x) closest_local.x = -half_extents.x;
    if (closest_local.x >  half_extents.x) closest_local.x =  half_extents.x;
    if (closest_local.y < -half_extents.y) closest_local.y = -half_extents.y;
    if (closest_local.y >  half_extents.y) closest_local.y =  half_extents.y;
    if (closest_local.z < -half_extents.z) closest_local.z = -half_extents.z;
    if (closest_local.z >  half_extents.z) closest_local.z =  half_extents.z;

    // Compute vector from closest point to sphere center (local space)
    PPVec3 delta_local;
    pp_vec3_sub(&sphere_center_local, &closest_local, &delta_local);
    float dist_sq = pp_vec3_dot(&delta_local, &delta_local);
    float radius = lhs->radius;

    if (dist_sq > radius * radius) {
        return false;
    }

    // Transform contact point back to world space
    PPVec3 contact_world;
    pp_quat_transform(&rhs->body.rot, &closest_local, &contact_world);
    pp_vec3_add(&contact_world, &rhs->body.pos, contact_point);

    // Penetration depth
    float dist = sqrtf(dist_sq);
    if (intersection){
        *intersection = radius - dist;
    }

    if (n) {
        PPVec3 normal_local;
        if (dist > 1e-6f) {
            pp_vec3_scale(&delta_local, 1.0f / dist, &normal_local);
        } else {
            // Sphere center is inside box, pick arbitrary normal (e.g., x axis)
            pp_vec3_set(&normal_local, 1.0f, 0.0f, 0.0f);
        }
        // Rotate normal to world space
        pp_quat_transform(&rhs->body.rot, &normal_local, n);
        pp_vec3_normalize(n);
    }

    return true;

}

void pp_physics_step(float t) {
    PPVec3 scaled_vel;

    // Apply acceleration to velocity
    for(int i = 0; i < object_count; ++i) {
        PPBody* body = (PPBody*) &objects[i];

        if(!body->is_alive) {
            continue;
        }

        // Apply gravity to acceleration before applying acceleration
        // to velocity
        pp_vec3_add(&body->acc, &gravity, &body->acc);

        pp_vec3_scale(&body->acc, t, &scaled_vel);
        pp_vec3_add(&body->vel, &scaled_vel, &body->vel);
        pp_vec3_init(&body->acc);

        // Apply linear damping
        pp_vec3_scale(&body->vel, 1.0f - body->damping, &body->vel);

        if(body->lock) {
            if((body->lock & PP_AXIS_LOCK_PITCH) == PP_AXIS_LOCK_PITCH) {
                body->a_acc.xyz[0] = 0.0f;
                body->a_vel.xyz[0] = 0.0f;
            }

            if((body->lock & PP_AXIS_LOCK_YAW) == PP_AXIS_LOCK_YAW) {
                body->a_acc.xyz[1] = 0.0f;
                body->a_vel.xyz[1] = 0.0f;
            }

            if((body->lock & PP_AXIS_LOCK_ROLL) == PP_AXIS_LOCK_ROLL) {
                body->a_acc.xyz[2] = 0.0f;
                body->a_vel.xyz[2] = 0.0f;
            }
        }

        PPVec3 scaled_ang_vel; // Temporary variable to store scaled angular velocity
        pp_vec3_scale(&body->a_acc, t, &scaled_ang_vel);
        pp_vec3_add(&body->a_vel, &scaled_ang_vel, &body->a_vel);

        // Apply angular damping if desired
        pp_vec3_scale(&body->a_vel, 1.0f - body->a_damping, &body->a_vel);

        // Reset the acceleration
        pp_vec3_init(&body->a_acc);
    }

    // Move all spheres by their velocity
    for(int i = 0; i < object_count; ++i) {
        PPBody* body = (PPBody*) &objects[i];
        if(!body->is_alive) {
            continue;
        }

        pp_vec3_scale(&body->vel, t, &scaled_vel);
        pp_vec3_add(&body->pos, &scaled_vel, &body->pos);

        PPQuaternion q_rot;
        pp_quat_from_angular_velocity(&body->a_vel, t, &q_rot); // Get rotation quaternion from angular velocity
        pp_quat_multiply(&body->rot, &q_rot, &body->rot); // Combine with current rotation
        pp_quat_normalize(&body->rot); // Normalize the quaternion
    }


    for(int i = 0; i < object_count; ++i) {
        // Check collision between spheres
        PPBody* lhs_body = (PPBody*) &objects[i];
        PPSphere* lhs_sphere = PP_SPHERE(lhs_body);
        PPBox* lhs_box = PP_BOX(lhs_body);

        if(!lhs_box && !lhs_sphere) {
            continue;
        }

        if(!lhs_body->is_alive) {
            continue;
        }

        for(int j = i + 1; j < object_count; ++j) {
            PPBody* rhs_body = (PPBody*) &objects[j];
            PPSphere* rhs_sphere = PP_SPHERE(rhs_body);
            PPBox* rhs_box = PP_BOX(rhs_body);

            if(!rhs_box && !rhs_sphere) {
                continue;
            }

            if(!rhs_body->is_alive) {
                continue;
            }

            if(lhs_sphere && rhs_sphere) {
                // Sphere vs Sphere
                float dist = pp_vec3_dist(&lhs_sphere->body.pos, &rhs_sphere->body.pos);
                if (dist <= (lhs_sphere->radius + rhs_sphere->radius)) {
                    PPCollision c;
                    pp_fill_collision_info_sphere_sphere(lhs_sphere, rhs_sphere, dist, &c);

                    bool respond = true;
                    const struct _PPCollisionMapEntry* cb = pp_physics_collision_map_search(lhs_body->kind, rhs_body->kind);
                    if (cb) {
                        respond = cb->collision_callback(lhs_sphere, rhs_sphere, lhs_body->kind, rhs_body->kind, &c, cb->user_data);
                    }

                    if (respond) {
                        // pp_sphere_sphere_response(lhs_sphere, rhs_sphere, &c);
                        pp_solve(&c);
                    }
                }
            } else if((lhs_sphere && rhs_box) || (rhs_sphere && lhs_box)) {
                // Sphere vs box
                //
                PPSphere* sphere = (lhs_sphere) ? lhs_sphere : rhs_sphere;
                PPBox* box = (lhs_box) ? lhs_box : rhs_box;
                PPVec3 contact, n;
                float d;
                if(pp_sphere_box_intersect(sphere, box, &contact, &n, &d)) {
                    PPCollision c;
                    pp_fill_collision_info_sphere_box(sphere, box, &contact, &n, &c, d);

                    bool respond = true;
                    const struct _PPCollisionMapEntry* cb = pp_physics_collision_map_search(lhs_body->kind, rhs_body->kind);
                    if (cb) {
                        respond = cb->collision_callback(sphere, box, sphere->body.kind, box->body.kind, &c, cb->user_data);
                    }

                    if (respond) {
                        // pp_sphere_box_response(sphere, box, &c, t);
                        pp_solve(&c);
                    }
                }
            }
        }

        if(lhs_sphere) {
            for(int j = 0; j < tri_count; ++j) {
                const PPTriangle* tri = tris + j;

                PPVec3 p, d;
                pp_vec3_scale(&tri->n, -1.0f, &d);
                float dist;
                if(pp_tri_intersect(tri, &lhs_body->pos, &d, &p, &dist)) {
                    if(dist <= lhs_sphere->radius) {
                        PPCollision c;
                        pp_fill_collision_info_sphere_triangle(lhs_sphere, tri, &p, lhs_sphere->radius - dist, &c);

                        bool respond = true;
                        const struct _PPCollisionMapEntry* cb = pp_physics_collision_map_search(lhs_body->kind, tri->kind);

                        if(cb) {
                            respond = cb->collision_callback(lhs_sphere, tri, lhs_body->kind, tri->kind, &c, cb->user_data);
                        }

                        if(respond) {
                            pp_solve(&c);
                        }
                    }
                }
            }
        }
    }
}

#endif
