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
 * - Create fully sphere dynamic spheres and apply linear and angular forces
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
 * - Boxes (currently kinematic, for platforms / obstacles etc.)
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
 * - Add collision response for Boxes
 * - Add fixed and spring joints (links) between objects
 * - Simplify/share collision response logic across all things
 * - Optimisations (replacing divisions where possible)
 *
 * I have no intention of adding more than this! If you want something more there are a bunch
 * of great open-source physics engines out there (e.g. Bullet, Box2D, ODE, Bounce..)
 *
 * # Usage
 *
 * Picophysics is a single-file header library (in the spirit of stb). To use it
 * you must do this in a single .c/.cpp file:
 *
 * #define PHYSICS_IMPLEMENTATION
 * #include "physics.h"
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

typedef struct _PPCollision {
    PPVec3 p;
    PPVec3 n;
    void* obj1;
    void* obj2;

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

typedef enum _PPObjectType {
    PP_OBJECT_TYPE_SPHERE,
    PP_OBJECT_TYPE_BOX,
} PPObjectType;

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
bool pp_physics_ray_intersect(const PPVec3* origin, const PPVec3* direction, PPSphere** sphere_hit, PPTriangle** tri_hit, float* distance);
void pp_physics_clear();
void pp_physics_set_gravity(const PPVec3* v);
bool pp_physics_collision_map_add(BodyKind kind1, BodyKind kind2, bool (*callback)(const void*, const void*, BodyKind, BodyKind, const PPCollision* c));

PPTriangle* pp_physics_create_triangle(const PPVec3* v1, const PPVec3* v2, const PPVec3* v3, BodyKind kind);
size_t pp_physics_triangle_count();
const PPTriangle* pp_physics_triangle_at(size_t i);

PPSphere* pp_physics_create_sphere(float radius, const PPVec3* pos, float mass, BodyKind kind);
void pp_physics_destroy_sphere(PPSphere* s);
float pp_sphere_get_radius(const PPSphere* s);

PPBox* pp_physics_create_box(float width, float height, float depth, const PPVec3* pos, float mass, BodyKind kind);
void pp_physics_destroy_box(PPBox* s);
float pp_box_get_width(const PPBox* b);
float pp_box_get_height(const PPBox* b);
float pp_box_get_depth(const PPBox* b);

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

#ifdef PHYSICS_IMPLEMENTATION

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
    bool (*collision_callback)(const void*, const void*, BodyKind, BodyKind, const PPCollision* c);
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

bool pp_sphere_intersect(const PPSphere* sphere, const PPVec3* o, const PPVec3* d, PPVec3* out, float* distance);
bool pp_tri_intersect(const PPTriangle* tri, const PPVec3* o, const PPVec3* d, PPVec3* out, float* distance);

/**
 * Intersects the world with the specified ray. Returns true if something was hit.
 *
 * If a hit was detected, then either sphere_hit or tri_hit will be populated (depending on what was hit) and the distance from the
 * origin to the hit will be returned.
 */
bool pp_physics_ray_intersect(const PPVec3* origin, const PPVec3* direction, PPSphere** sphere_hit, PPTriangle** tri_hit, float* distance) {
    float closest_dist = FLT_MAX;
    const PPTriangle* closest_tri = NULL;
    const PPSphere* closest_sphere = NULL;

    for(int i = 0; i < pp_physics_triangle_count(); ++i) {
        const PPTriangle* t = pp_physics_triangle_at(i);

        PPVec3 hit;
        float dist;
        if(pp_tri_intersect(t, origin, direction, &hit, &dist)){
            if(dist < closest_dist) {
                closest_tri = t;
                closest_dist = dist;
            }
        }
    }

    for(int i = 0; i < pp_physics_body_total_count(); ++i) {
        const PPBody* body = pp_physics_body_at(i);
        const PPSphere* s = PP_SPHERE(body);

        if(!s || !body->is_alive) {
            continue;
        }

        PPVec3 hit;
        float dist;
        if(pp_sphere_intersect(s, origin, direction, &hit, &dist)) {
            if(dist < closest_dist) {
                closest_sphere = s;
                closest_tri = NULL;
                closest_dist = dist;
            }
        }
    }

    *sphere_hit = (PPSphere*) closest_sphere;
    *tri_hit = (PPTriangle*) closest_tri;
    *distance = closest_dist;
    return true;
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
    const float e = FLT_EPSILON;
    PPVec3 edge1, edge2, cross_e1, cross_e2, s;
    pp_vec3_sub(&tri->v[1], &tri->v[0], &edge1);
    pp_vec3_sub(&tri->v[2], &tri->v[0], &edge2);
    pp_vec3_cross(d, &edge2, &cross_e2);

    float det = pp_vec3_dot(&edge1, &cross_e2);

    if(det > -e && det < e) {
        return NULL;
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

    if(PP_SPHERE(s)) {
        // Simplified inertia for Spheres
        float I = s->inertia.m[0];

        if (I <= 0.0f) return;   // nothing to do for mass‑less or zero‑radius objects

        PPVec3 ang_acc;
        pp_vec3_scale(&torque, 1.0f / I, &ang_acc);
        pp_vec3_add(&s->a_acc, &ang_acc, &s->a_acc);
    }
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

bool pp_physics_collision_map_add(BodyKind kind1, BodyKind kind2, bool (*callback)(const void*, const void*, BodyKind, BodyKind, const PPCollision* c)) {
    if(!pp_physics_collision_map_search(kind1, kind2)) {
        struct _PPCollisionMapEntry* entry = &collision_map[collision_map_count++];
        entry->kind1 = kind1;
        entry->kind2 = kind2;
        entry->collision_callback = callback;
        return true;
    }

    return false;
}

void pp_fill_collision_info_sphere_box(const PPSphere* lhs, const PPBox* rhs, const PPVec3* contact_point, const PPVec3* n, PPCollision* c, float d) {
    c->dist = d;
    pp_vec3_assign(&c->p, contact_point);
    pp_vec3_assign(&c->n, n);
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
}

void pp_fill_collision_info_sphere_triangle(const PPSphere* lhs, const PPTriangle* tri, const PPVec3* p, float dist, PPCollision* c) {
    pp_vec3_scale(&tri->n, 1.0f, &c->n); // Copy
    pp_vec3_scale(p, 1.0f, &c->p); // Copy
    c->dist = dist;
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

void pp_physics_destroy_sphere(PPSphere* s) {
    s->body.is_alive = false;
    ++dead_object_count;
}

void pp_physics_set_gravity(const PPVec3* v) {
    pp_vec3_assign(&gravity, v);
    gravity_magnitude = pp_vec3_length(&gravity);
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

static void pp_sphere_triangle_response(PPSphere* lhs_sphere, const PPTriangle* tri, const PPCollision* c, float t) {
    PPBody* lhs_body = PP_BODY(lhs_sphere);
    float overlap = lhs_sphere->radius - c->dist;

    // Move the sphere out of overlap immediately
    PPVec3 adjustment;
    pp_vec3_scale(&c->n, overlap, &adjustment);
    pp_vec3_add(&lhs_body->pos, &adjustment, &lhs_body->pos);

    // Reflect the velocity based on the collision normal
    float vel_along_normal = pp_vec3_dot(&lhs_body->vel, &c->n);
    if (vel_along_normal < 0) {
        // Apply restitution
        PPVec3 vel_normal, vel_tangent;
        pp_vec3_scale(&c->n, vel_along_normal, &vel_normal);
        pp_vec3_sub(&lhs_body->vel, &vel_normal, &vel_tangent);

        float r = 1.0f + fmax(0 /*tri->bounce*/, lhs_body->bounce);
        float f = fmin(tri->friction, lhs_body->friction);

        PPVec3 impulse, friction_impulse;

        pp_vec3_scale(&vel_normal, -r, &impulse);
        pp_vec3_scale(&vel_tangent, -f, &friction_impulse);
        pp_vec3_add(&impulse, &friction_impulse, &impulse);
        pp_vec3_add(&lhs_body->vel, &impulse, &lhs_body->vel);

        PPVec3 contact_offset;
        pp_vec3_sub(&c->p, &lhs_body->pos, &contact_offset);

        // Calculate the contact impulse
        PPVec3 contact_impulse;
        pp_vec3_scale(&impulse, -1.0f, &contact_impulse);

        // Calculate torque due to the collision (Torque = r x F)
        PPVec3 torque;
        pp_vec3_cross(&contact_offset, &contact_impulse, &torque);

        // Assuming a simplified moment of inertia (I) as (2/5) * mass * radius^2 for the sphere
        float I = lhs_body->inertia.m[0];

        // Change in angular velocity due to torque = torque / moment of inertia
        PPVec3 angular_acceleration;
        pp_vec3_scale(&torque, 1.0f / I, &angular_acceleration);

        // Update the angular velocity
        pp_vec3_scale(&angular_acceleration, t, &angular_acceleration); // Scale by time step
        pp_vec3_add(&lhs_body->a_vel, &angular_acceleration, &lhs_body->a_vel);

        // float rolling_friction = 0.3f;
        // Vec3 rolling_friction_force;
        // vec3_scale(&lhs->body.a_vel, -rolling_friction * t, &rolling_friction_force);
        // vec3_add(&lhs->body.a_vel, &rolling_friction_force, &lhs->body.a_vel);
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

static inline bool flt_close(const float a, const float b) {
    return (a + FLT_EPSILON > b) && (a - FLT_EPSILON) < b;
}

bool pp_sphere_box_intersect(const PPSphere* lhs, const PPBox* rhs, PPVec3* contact_point, PPVec3* n, float* intersection) {
    PPVec3 dir;
    pp_vec3_sub(&lhs->body.pos, &rhs->body.pos, &dir);

    const PPVec3 bases[3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    PPVec3 axis[3];
    // Rotate the box axis into world space
    pp_quat_transform(&rhs->body.rot, &bases[0], &axis[0]);
    pp_quat_transform(&rhs->body.rot, &bases[1], &axis[1]);
    pp_quat_transform(&rhs->body.rot, &bases[2], &axis[2]);

    PPVec3 half_lengths;
    pp_vec3_set(&half_lengths,
        rhs->whd.x * 0.5f,
        rhs->whd.y * 0.5f,
        rhs->whd.z * 0.5f
    );

    PPVec3 distance;

    for(int i = 0; i < 3; ++i) {
        distance.xyz[i] = pp_vec3_dot(&dir, &axis[i]);
        if(distance.xyz[i] > half_lengths.xyz[i]) {
            distance.xyz[i] = half_lengths.xyz[i];
        } else if (distance.xyz[i] < -half_lengths.xyz[i]) {
            distance.xyz[i] = -half_lengths.xyz[i];
        }
    }

    pp_vec3_init(contact_point);

    for(int i = 0; i < 3; ++i) {
        PPVec3 t;
        pp_vec3_scale(&axis[i], distance.xyz[i], &t);
        pp_vec3_add(contact_point, &t, contact_point);
    }

    PPVec3 diff;
    pp_vec3_sub(contact_point, &lhs->body.pos, &diff);
    float dist_sq = pp_vec3_length_sq(&diff);
    if(dist_sq < (lhs->radius * lhs->radius)) {
        pp_vec3_assign(n, &diff);
        pp_vec3_normalize(n);
        *intersection = sqrtf(dist_sq) - lhs->radius;
        return true;
    }

    return false;

    // PPVec3 distance_vec, closest_point;
    // pp_vec3_sub(&lhs->body.pos, &rhs->body.pos, &distance_vec);
    // pp_vec3_assign(&closest_point, &rhs->body.pos);

    // for(int i = 0; i < 3; ++i) {
    //     float dist = pp_vec3_dot(&distance_vec, &axis[i]);
    //     if(dist >= half_lengths[i]) {
    //         dist = half_lengths[i];
    //     } else if(dist <= -half_lengths[i]) {
    //         dist = -half_lengths[i];
    //     }

    //     PPVec3 tmp;
    //     pp_vec3_scale(&axis[i], dist, &tmp);
    //     pp_vec3_add(&closest_point, &tmp, &closest_point);
    // }

    // PPVec3 diff;
    // pp_vec3_sub(&lhs->body.pos, &closest_point, &diff);
    // if(pp_vec3_length(&diff) <= lhs->radius) {
    //     pp_vec3_assign(contact_point, &closest_point);

    //     PPVec3 rel_point;
    //     pp_vec3_sub(contact_point, &lhs->body.pos, &rel_point);
    //     float d;
    //     for(int i = 0; i < 3; ++i) {
    //         d = pp_plane_distance(&axis[i], half_lengths[i], &rel_point);
    //         if(flt_close(d, 0.0f)) {
    //             pp_vec3_assign(n, &bases[i]);
    //             return true;
    //         }

    //         d = pp_plane_distance(&axis[i], -half_lengths[i], &rel_point);
    //         if(flt_close(d, 0.0f)) {
    //             pp_vec3_set(n, -bases[i].x, -bases[i].y, -bases[i].z);
    //             return true;
    //         }
    //     }

    //     assert(false && "Couldn't find contact normal");
    //     return true;
    // }

    // return false;
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
                        respond = cb->collision_callback(lhs_sphere, rhs_sphere, lhs_body->kind, rhs_body->kind, &c);
                    }

                    if (respond) {
                        pp_sphere_sphere_response(lhs_sphere, rhs_sphere, &c);
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
                        respond = cb->collision_callback(sphere, box, sphere->body.kind, box->body.kind, &c);
                    }

                    if (respond) {
                        pp_sphere_box_response(sphere, box, &c, t);
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
                        pp_fill_collision_info_sphere_triangle(lhs_sphere, tri, &p, dist, &c);

                        bool respond = true;
                        const struct _PPCollisionMapEntry* cb = pp_physics_collision_map_search(lhs_body->kind, tri->kind);

                        if(cb) {
                            respond = cb->collision_callback(lhs_sphere, tri, lhs_body->kind, tri->kind, &c);
                        }

                        if(respond) {
                            pp_sphere_triangle_response(lhs_sphere, tri, &c, t);
                        }
                    }
                }
            }
        }
    }
}

#endif
