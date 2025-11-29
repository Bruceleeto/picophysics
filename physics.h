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

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _PPVec3 {
    float xyz[3];
} PPVec3;

typedef struct _PPQuaternion {
    float xyzw[4];
} PPQuaternion;

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

    BodyKind kind;

    PPAxisLock lock;
} PPBody;

typedef struct _PPSphere {
    PPBody body;
    float radius;
    bool is_alive;
    void* user_data;
} PPSphere;

typedef struct _PPTriangle {
    PPVec3 v[3];
    PPVec3 n;
    PPPlane p;
    BodyKind kind;
    float friction;
} PPTriangle;

extern PPVec3* pp_vec3_init(PPVec3* v);
extern PPVec3* pp_vec3_set(PPVec3* v, float x, float y, float z);
extern PPVec3* pp_vec3_scale(const PPVec3* v1, float t, PPVec3* out);

extern void pp_quat_between(const PPVec3* v0, const PPVec3* q1, PPQuaternion* result);
extern void pp_quat_slerp(const PPQuaternion* q0, const PPQuaternion* q1, float t, PPQuaternion* result);

extern void pp_physics_step(float t);
extern bool pp_physics_ray_intersect(const PPVec3* origin, const PPVec3* direction, PPSphere** sphere_hit, PPTriangle** tri_hit, float* distance);
extern void pp_physics_clear();

extern PPTriangle* pp_physics_create_triangle(const PPVec3* v1, const PPVec3* v2, const PPVec3* v3, BodyKind kind);
extern size_t pp_physics_triangle_count();
extern const PPTriangle* pp_physics_triangle_at(size_t i);

extern size_t pp_physics_sphere_count();
extern size_t pp_physics_sphere_total_count();
extern const PPSphere* pp_physics_sphere_at(size_t i);
extern PPSphere* pp_physics_create_sphere(float radius, const PPVec3* pos, float mass, BodyKind kind);
extern void pp_physics_destroy_sphere(PPSphere* s);
extern void pp_physics_set_gravity(const PPVec3* v);

extern bool pp_collision_map_add(BodyKind kind1, BodyKind kind2, bool (*callback)(const void*, const void*, BodyKind, BodyKind));

extern PPSphere* pp_sphere_set_bounce(PPSphere* s, float b);
extern void pp_sphere_add_force(PPSphere* s, float x, float y, float z);
extern void pp_sphere_add_angular_force(PPSphere* s, float x, float y, float z);
extern void pp_sphere_lock_axis(PPSphere* s, PPAxisLock lock);
extern void pp_sphere_set_angular_damping(PPSphere* s, float d);
extern void pp_sphere_set_damping(PPSphere* s, float d);
extern void pp_sphere_get_forward(PPSphere* s, PPVec3* f);
extern void pp_sphere_set_position(PPSphere* s, float x, float y, float z);
extern void pp_sphere_get_position(PPSphere* s, PPVec3* pos);
extern float pp_sphere_get_radius(PPSphere* s);
extern void pp_sphere_set_user_data(PPSphere* s, void* data);
extern void* pp_sphere_get_user_data(const PPSphere* s);

extern void pp_set_angular_velocity(PPSphere* s, float x, float y, float z);
extern void pp_set_velocity(PPSphere* s, float x, float y, float z);
extern void pp_set_angular_acceleration(PPSphere* s, float x, float y, float z);
extern void pp_set_acceleration(PPSphere* s, float x, float y, float z);

// Given a direction vector, this will apply a force to attempt to look towards it
extern void pp_sphere_look_at(PPSphere* s, float x, float y, float z);

#ifdef __cplusplus
}
#endif

#endif

#ifdef PHYSICS_IMPLEMENTATION

#define PHYSICS_MAX_SPHERES 32
#define PHYSICS_MAX_TRIANGLES 128

static PPSphere spheres[PHYSICS_MAX_SPHERES];
static int sphere_count = 0;
static int dead_sphere_count = 0;

static PPTriangle tris[PHYSICS_MAX_TRIANGLES];
static int tri_count = 0;

static struct _PPCollisionMapEntry {
    BodyKind kind1;
    BodyKind kind2;
    bool (*collision_callback)(const void*, const void*, BodyKind, BodyKind);
} collision_map[32];

static int collision_map_count = 0;

static PPVec3 gravity = {.xyz = {0.0f, 0.0f, 0.0f}};
static float gravity_magnitude = 0.0f;

PPVec3* pp_vec3_init(PPVec3* v) {
    v->xyz[0] = 0.0f;
    v->xyz[1] = 0.0f;
    v->xyz[2] = 0.0f;
    return v;
}

PPVec3* pp_vec3_set(PPVec3* v, float x, float y, float z) {
    v->xyz[0] = x;
    v->xyz[1] = y;
    v->xyz[2] = z;
    return v;
}

PPVec3* pp_vec3_assign(PPVec3* target, const PPVec3* source) {
    pp_vec3_set(target, source->xyz[0], source->xyz[1], source->xyz[2]);
    return target;
}

PPVec3* pp_vec3_add(const PPVec3* v1, const PPVec3* v2, PPVec3* out) {
    out->xyz[0] = v1->xyz[0] + v2->xyz[0];
    out->xyz[1] = v1->xyz[1] + v2->xyz[1];
    out->xyz[2] = v1->xyz[2] + v2->xyz[2];
    return out;
}

PPVec3* pp_vec3_sub(const PPVec3* v1, const PPVec3* v2, PPVec3* out) {
    out->xyz[0] = v1->xyz[0] - v2->xyz[0];
    out->xyz[1] = v1->xyz[1] - v2->xyz[1];
    out->xyz[2] = v1->xyz[2] - v2->xyz[2];
    return out;
}

PPVec3* pp_vec3_scale(const PPVec3* v1, float t, PPVec3* out) {
    out->xyz[0] = v1->xyz[0] * t;
    out->xyz[1] = v1->xyz[1] * t;
    out->xyz[2] = v1->xyz[2] * t;
    return out;
}

float pp_vec3_length(const PPVec3* v1) {
    return sqrtf(v1->xyz[0] * v1->xyz[0] + v1->xyz[1] * v1->xyz[1] + v1->xyz[2] * v1->xyz[2]);
}

float pp_vec3_dist(const PPVec3* v1, const PPVec3* v2) {
    PPVec3 tmp;
    pp_vec3_sub(v2, v1, &tmp);
    return pp_vec3_length(&tmp);
}

PPVec3* pp_vec3_cross(const PPVec3 *v1, const PPVec3 *v2, PPVec3 *out) {
    out->xyz[0] = v1->xyz[1] * v2->xyz[2] - v1->xyz[2] * v2->xyz[1];
    out->xyz[1] = v1->xyz[2] * v2->xyz[0] - v1->xyz[0] * v2->xyz[2];
    out->xyz[2] = v1->xyz[0] * v2->xyz[1] - v1->xyz[1] * v2->xyz[0];
    return out;
}

float pp_vec3_dot(const PPVec3 *v1, const PPVec3 *v2) {
    return v1->xyz[0] * v2->xyz[0] +
           v1->xyz[1] * v2->xyz[1] +
           v1->xyz[2] * v2->xyz[2];
}

bool pp_vec3_normalize(PPVec3 *v) {
    float length = pp_vec3_length(v);

    // Check for zero-length vector to avoid division by zero
    if (length > 0.0f) {
        v->xyz[0] /= length;
        v->xyz[1] /= length;
        v->xyz[2] /= length;
        return true;
    } else {
        pp_vec3_init(v);
        return false;
    }
}

PPQuaternion* pp_quat_init(PPQuaternion* q) {
    q->xyzw[0] = 0.0f;
    q->xyzw[1] = 0.0f;
    q->xyzw[2] = 0.0f;
    q->xyzw[3] = 1.0f;
    return q;
}

PPQuaternion* pp_quat_set(PPQuaternion* q, float x, float y, float z, float w) {
    q->xyzw[0] = x;
    q->xyzw[1] = y;
    q->xyzw[2] = z;
    q->xyzw[3] = w;
    return q;
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
        q_rot->xyzw[0] = axis.xyz[0] * sin_half_angle;
        q_rot->xyzw[1] = axis.xyz[1] * sin_half_angle;
        q_rot->xyzw[2] = axis.xyz[2] * sin_half_angle;
        q_rot->xyzw[3] = cos_half_angle;
    } else {
        // If there is no rotation
        pp_quat_init(q_rot);
    }
}

void pp_quat_multiply(const PPQuaternion* q1, const PPQuaternion* q2, PPQuaternion* result) {
    PPQuaternion tmp;
    tmp.xyzw[0] = q1->xyzw[3] * q2->xyzw[0] + q1->xyzw[0] * q2->xyzw[3] + q1->xyzw[1] * q2->xyzw[2] - q1->xyzw[2] * q2->xyzw[1];
    tmp.xyzw[1] = q1->xyzw[3] * q2->xyzw[1] - q1->xyzw[0] * q2->xyzw[2] + q1->xyzw[1] * q2->xyzw[3] + q1->xyzw[2] * q2->xyzw[0];
    tmp.xyzw[2] = q1->xyzw[3] * q2->xyzw[2] + q1->xyzw[0] * q2->xyzw[1] - q1->xyzw[1] * q2->xyzw[0] + q1->xyzw[2] * q2->xyzw[3];
    tmp.xyzw[3] = q1->xyzw[3] * q2->xyzw[3] - q1->xyzw[0] * q2->xyzw[0] - q1->xyzw[1] * q2->xyzw[1] - q1->xyzw[2] * q2->xyzw[2];

    *result = tmp;
}

void pp_quat_normalize(PPQuaternion* q) {
    float norm = sqrtf(q->xyzw[0] * q->xyzw[0] + q->xyzw[1] * q->xyzw[1] + q->xyzw[2] * q->xyzw[2] + q->xyzw[3] * q->xyzw[3]);
    if (norm > 0) {
        q->xyzw[0] /= norm;
        q->xyzw[1] /= norm;
        q->xyzw[2] /= norm;
        q->xyzw[3] /= norm;
    }
}

static void pp_quat_forward(const PPQuaternion* q, PPVec3* out)
{
    float x = q->xyzw[0];
    float y = q->xyzw[1];
    float z = q->xyzw[2];
    float w = q->xyzw[3];

    out->xyz[0] = 2.0f * (x * z + w * y);
    out->xyz[1] = 2.0f * (y * z - w * x);
    out->xyz[2] = 1.0f - 2.0f * (x * x + y * y);
}

void pp_quat_between(const PPVec3* v0, const PPVec3* v1, PPQuaternion* result) {
    float dot = pp_vec3_dot(v0, v1);

    // If the vectors are exactly opposite, return a 180-degree rotation around an arbitrary axis
    if (dot < -1.0f + 1e-6f) {
        // Rotate around the Y axis
        result->xyzw[0] = 0.0f;
        result->xyzw[1] = 1.0f;
        result->xyzw[2] = 0.0f;
        result->xyzw[3] = 0.0f;
        return;
    }

    // If the vectors are exactly the same, return the identity quaternion
    if (dot > 1.0f - 1e-6f) {
        result->xyzw[0] = 0.0f;
        result->xyzw[1] = 0.0f;
        result->xyzw[2] = 0.0f;
        result->xyzw[3] = 1.0f;
        return;
    }

    // Calculate the axis of rotation
    PPVec3 axis;
    axis.xyz[0] = v0->xyz[1] * v1->xyz[2] - v0->xyz[2] * v1->xyz[1];
    axis.xyz[1] = v0->xyz[2] * v1->xyz[0] - v0->xyz[0] * v1->xyz[2];
    axis.xyz[2] = v0->xyz[0] * v1->xyz[1] - v0->xyz[1] * v1->xyz[0];

    pp_vec3_normalize(&axis);

    // Calculate the angle of rotation
    float angle = acosf(dot);

    // Calculate the quaternion
    float half_angle = angle * 0.5f;
    float sin_half_angle = sinf(half_angle);
    result->xyzw[0] = axis.xyz[0] * sin_half_angle;
    result->xyzw[1] = axis.xyz[1] * sin_half_angle;
    result->xyzw[2] = axis.xyz[2] * sin_half_angle;
    result->xyzw[3] = cosf(half_angle);
}

void pp_quat_slerp(const PPQuaternion* q0, const PPQuaternion* q1, float t, PPQuaternion* result) {
    float dot = q0->xyzw[0] * q1->xyzw[0] + q0->xyzw[1] * q1->xyzw[1] + q0->xyzw[2] * q1->xyzw[2] + q0->xyzw[3] * q1->xyzw[3];

    PPQuaternion q1_temp;
    if (dot < 0.0f) {
        q1_temp.xyzw[0] = -q1->xyzw[0];
        q1_temp.xyzw[1] = -q1->xyzw[1];
        q1_temp.xyzw[2] = -q1->xyzw[2];
        q1_temp.xyzw[3] = -q1->xyzw[3];
        dot = -dot;
    } else {
        q1_temp = *q1;
    }

    // If the quaternions are very close, use linear interpolation
    if (dot > 0.9995f) {
        result->xyzw[0] = q0->xyzw[0] + t * (q1_temp.xyzw[0] - q0->xyzw[0]);
        result->xyzw[1] = q0->xyzw[1] + t * (q1_temp.xyzw[1] - q0->xyzw[1]);
        result->xyzw[2] = q0->xyzw[2] + t * (q1_temp.xyzw[2] - q0->xyzw[2]);
        result->xyzw[3] = q0->xyzw[3] + t * (q1_temp.xyzw[3] - q0->xyzw[3]);
        return;
    }

    // Calculate the angle between the quaternions
    float theta = acosf(dot);

    // Calculate the coefficients for spherical linear interpolation
    float sin_theta = sinf(theta);
    float s0 = sinf((1.0f - t) * theta) / sin_theta;
    float s1 = sinf(t * theta) / sin_theta;

    // Perform the interpolation
    result->xyzw[0] = s0 * q0->xyzw[0] + s1 * q1_temp.xyzw[0];
    result->xyzw[1] = s0 * q0->xyzw[1] + s1 * q1_temp.xyzw[1];
    result->xyzw[2] = s0 * q0->xyzw[2] + s1 * q1_temp.xyzw[2];
    result->xyzw[3] = s0 * q0->xyzw[3] + s1 * q1_temp.xyzw[3];
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

    for(int i = 0; i < pp_physics_sphere_total_count(); ++i) {
        const PPSphere* s = pp_physics_sphere_at(i);
        if(!s->is_alive) {
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

PPSphere* pp_sphere_set_bounce(PPSphere* s, float b);

void pp_set_angular_velocity(PPSphere* s, float x, float y, float z) {
    pp_vec3_set(&s->body.a_vel, x, y, z);
}

void pp_set_velocity(PPSphere* s, float x, float y, float z) {
    pp_vec3_set(&s->body.vel, x, y, z);
}

void pp_set_angular_acceleration(PPSphere* s, float x, float y, float z) {
    pp_vec3_set(&s->body.a_acc, x, y, z);
}

void pp_set_acceleration(PPSphere* s, float x, float y, float z) {
    pp_vec3_set(&s->body.acc, x, y, z);
}

void pp_sphere_set_angular_damping(PPSphere* s, float d) {
    if(d < 0.0f || d > 1.0f) {
        return;
    }

    s->body.a_damping = d;
}

void pp_sphere_set_damping(PPSphere* s, float d) {
    if(d < 0.0f || d > 1.0f) {
        return;
    }

    s->body.damping = d;
}

void pp_sphere_get_forward(PPSphere* s, PPVec3* f) {
    pp_quat_forward(&s->body.rot, f);
}

float pp_sphere_get_radius(PPSphere* s) {
    return s->radius;
}

void pp_quat_from_axis_angle(PPQuaternion* q, const PPVec3* axis, float angle) {
    PPVec3 a;
    pp_vec3_assign(&a, axis);
    pp_vec3_normalize(&a);

    float half = angle * 0.5f;
    float s = sinf(half);   // sin(θ/2)
    float c = cosf(half);   // cos(θ/2)

    q->xyzw[0] = a.xyz[0] * s;   // axis.x * sin(θ/2)
    q->xyzw[1] = a.xyz[1] * s;   // axis.y * sin(θ/2)
    q->xyzw[2] = a.xyz[2] * s;   // axis.z * sin(θ/2)
    q->xyzw[3] = c;         // cos(θ/2)

    pp_quat_normalize(q);
}

void pp_sphere_look_at(PPSphere* s, float x, float y, float z) {
    PPVec3 t, f, c;
    pp_vec3_set(&t, x, y, z);
    pp_sphere_get_forward(s, &f);
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
    pp_vec3_scale(&s->body.a_vel, kd, &b);
    pp_vec3_sub(&a, &b, &torque);
    pp_sphere_add_angular_force(s, torque.xyz[0], torque.xyz[1], torque.xyz[2]);
}

PPSphere* pp_sphere_init(PPSphere* s, float radius, const PPVec3* pos, float mass, BodyKind kind) {
    s->radius = radius;
    s->is_alive = true;
    s->user_data = NULL;

    pp_vec3_init(&s->body.vel);
    pp_vec3_init(&s->body.acc);
    pp_vec3_init(&s->body.a_vel);
    pp_vec3_init(&s->body.a_acc);
    pp_quat_init(&s->body.rot);
    pp_vec3_set(&s->body.pos, pos->xyz[0], pos->xyz[1], pos->xyz[2]);
    s->body.kind = kind;
    s->body.mass = mass;
    s->body.friction = 0.3f;
    s->body.damping = 0.01f;
    s->body.a_damping = 0.02f;
    pp_sphere_set_bounce(s, 0.5f);
    return s;
}

void pp_sphere_set_user_data(PPSphere* s, void* data) {
    s->user_data = data;
}

void* pp_sphere_get_user_data(const PPSphere* s) {
    return s->user_data;
}

void pp_sphere_lock_axis(PPSphere* s, PPAxisLock lock) {
    s->body.lock = lock;
}

void pp_sphere_get_position(PPSphere*s, PPVec3* pos) {
    pp_vec3_assign(pos, &s->body.pos);
}

void pp_sphere_set_position(PPSphere*s, float x, float y, float z) {
    pp_vec3_set(&s->body.pos, x, y, z);
}

PPSphere* pp_sphere_set_bounce(PPSphere* s, float b) {
    if(b < 0.0f || b > 1.0f) {
        return NULL;
    }

    s->body.bounce = b;
    return s;
}

void pp_sphere_add_force(PPSphere* s, float x, float y, float z) {
    PPVec3 force;
    pp_vec3_set(&force, x, y, z);

    PPVec3 acceleration;

    // Ensure you do not divide by zero
    if (s->body.mass > 0) {
        // a = F / m
        pp_vec3_scale(&force, 1.0f / s->body.mass, &acceleration);

        // Add acceleration to the sphere's current acceleration
        pp_vec3_add(&s->body.acc, &acceleration, &s->body.acc);
    }
}

void pp_sphere_add_angular_force(PPSphere* s, float tx, float ty, float tz)
{
    PPVec3 torque;
    pp_vec3_set(&torque, tx, ty, tz);

    float I = (2.0f / 5.0f) * s->body.mass * s->radius * s->radius;

    if (I <= 0.0f) return;   // nothing to do for mass‑less or zero‑radius objects

    PPVec3 ang_acc;
    pp_vec3_scale(&torque, 1.0f / I, &ang_acc);
    pp_vec3_add(&s->body.a_acc, &ang_acc, &s->body.a_acc);
}

const struct _PPCollisionMapEntry* pp_collision_map_search(BodyKind kind1, BodyKind kind2) {
    for(int i = 0; i < collision_map_count; ++i) {
        struct _PPCollisionMapEntry* entry = &collision_map[i];
        if((entry->kind1 == kind1 && entry->kind2 == kind2) || (entry->kind2 == kind1 && entry->kind1 == kind2)) {
            return entry;
        }
    }

    return NULL;
}

bool pp_collision_map_add(BodyKind kind1, BodyKind kind2, bool (*callback)(const void*, const void*, BodyKind, BodyKind)) {
    if(!pp_collision_map_search(kind1, kind2)) {
        struct _PPCollisionMapEntry* entry = &collision_map[collision_map_count++];
        entry->kind1 = kind1;
        entry->kind2 = kind2;
        entry->collision_callback = callback;
        return true;
    }

    return false;
}

static void pp_fill_collision_info_sphere_sphere(const PPSphere* lhs, const PPSphere* rhs, PPCollision* c) {
    pp_vec3_sub(&lhs->body.pos, &rhs->body.pos, &c->n);
    float l = pp_vec3_length(&c->n);

    if(l > 0) {
        c->n.xyz[0] /= l;
        c->n.xyz[1] /= l;
        c->n.xyz[2] /= l;
    }

    float total_radius = lhs->radius + rhs->radius;
    float wr1 = lhs->radius / total_radius;
    float wr2 = rhs->radius / total_radius;

    for(int i = 0; i < 3; ++i) {
        c->p.xyz[i] = lhs->body.pos.xyz[i] * wr1 + rhs->body.pos.xyz[i] * wr2;
    }
}

static void pp_fill_collision_info_sphere_triangle(const PPSphere* lhs, const PPTriangle* tri, const PPVec3* p, PPCollision* c) {
    pp_vec3_scale(&tri->n, 1.0f, &c->n); // Copy
    pp_vec3_scale(p, 1.0f, &c->p); // Copy
}

PPSphere* pp_physics_create_sphere(float radius, const PPVec3* pos, float mass, BodyKind kind) {
    PPSphere* ret = NULL;

    if(dead_sphere_count) {
        for(int i = 0; i < sphere_count; ++i) {
            if(!spheres[i].is_alive) {
                dead_sphere_count--;
                ret = spheres + i;
                break;
            }
        }
    }

    if(!ret) {
        ret = &spheres[sphere_count++];
    }

    pp_sphere_init(ret, radius, pos, mass, kind);
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
    sphere_count = 0;
    dead_sphere_count = 0;
    memset(tris, 0, sizeof(tris));
    memset(spheres, 0, sizeof(spheres));
}

size_t pp_physics_triangle_count() {
    return tri_count;
}

size_t pp_physics_sphere_count() {
    return sphere_count - dead_sphere_count;
}

size_t pp_physics_sphere_total_count() {
    return sphere_count;
}

const PPSphere* pp_physics_sphere_at(size_t i) {
    return spheres + i;
}

const PPTriangle* pp_physics_triangle_at(size_t i) {
    return tris + i;
}

void pp_physics_destroy_sphere(PPSphere* s) {
    s->is_alive = false;
    ++dead_sphere_count;
}

void pp_physics_set_gravity(const PPVec3* v) {
    pp_vec3_assign(&gravity, v);
    gravity_magnitude = pp_vec3_length(&gravity);
}

void pp_physics_step(float t) {
    PPVec3 scaled_vel;

    // Apply acceleration to velocity
    for(int i = 0; i < sphere_count; ++i) {
        PPSphere* sp = &spheres[i];
        if(!sp->is_alive) {
            continue;
        }

        // Apply gravity to acceleration before applying acceleration
        // to velocity
        pp_vec3_add(&sp->body.acc, &gravity, &sp->body.acc);

        pp_vec3_scale(&sp->body.acc, t, &scaled_vel);
        pp_vec3_add(&sp->body.vel, &scaled_vel, &sp->body.vel);
        pp_vec3_init(&sp->body.acc);

        // Apply linear damping
        pp_vec3_scale(&sp->body.vel, 1.0f - sp->body.damping, &sp->body.vel);

        if(sp->body.lock) {
            if((sp->body.lock & PP_AXIS_LOCK_PITCH) == PP_AXIS_LOCK_PITCH) {
                sp->body.a_acc.xyz[0] = 0.0f;
                sp->body.a_vel.xyz[0] = 0.0f;
            }

            if((sp->body.lock & PP_AXIS_LOCK_YAW) == PP_AXIS_LOCK_YAW) {
                sp->body.a_acc.xyz[1] = 0.0f;
                sp->body.a_vel.xyz[1] = 0.0f;
            }

            if((sp->body.lock & PP_AXIS_LOCK_ROLL) == PP_AXIS_LOCK_ROLL) {
                sp->body.a_acc.xyz[2] = 0.0f;
                sp->body.a_vel.xyz[2] = 0.0f;
            }
        }

        PPVec3 scaled_ang_vel; // Temporary variable to store scaled angular velocity
        pp_vec3_scale(&sp->body.a_acc, t, &scaled_ang_vel);
        pp_vec3_add(&sp->body.a_vel, &scaled_ang_vel, &sp->body.a_vel);

        // Apply angular damping if desired
        pp_vec3_scale(&sp->body.a_vel, 1.0f - sp->body.a_damping, &sp->body.a_vel);

        // Reset the acceleration
        pp_vec3_init(&sp->body.a_acc);
    }

    // Move all spheres by their velocity
    for(int i = 0; i < sphere_count; ++i) {
        PPSphere* sp = &spheres[i];
        if(!sp->is_alive) {
            continue;
        }

        pp_vec3_scale(&sp->body.vel, t, &scaled_vel);
        pp_vec3_add(&sp->body.pos, &scaled_vel, &sp->body.pos);

        PPQuaternion q_rot;
        pp_quat_from_angular_velocity(&sp->body.a_vel, t, &q_rot); // Get rotation quaternion from angular velocity
        pp_quat_multiply(&sp->body.rot, &q_rot, &sp->body.rot); // Combine with current rotation
        pp_quat_normalize(&sp->body.rot); // Normalize the quaternion
    }


    for(int i = 0; i < sphere_count; ++i) {
        // Check collision between spheres
        PPSphere* lhs = &spheres[i];

        if(!lhs->is_alive) {
            continue;
        }

        for(int j = i + 1; j < sphere_count; ++j) {
            PPSphere* rhs = &spheres[j];

            if(!rhs->is_alive) {
                continue;
            }

            float dist = pp_vec3_dist(&lhs->body.pos, &rhs->body.pos);
            if (dist <= (lhs->radius + rhs->radius)) {
                PPCollision c;
                pp_fill_collision_info_sphere_sphere(lhs, rhs, &c);

                bool respond = true;
                const struct _PPCollisionMapEntry* cb = pp_collision_map_search(lhs->body.kind, rhs->body.kind);
                if (cb) {
                    respond = cb->collision_callback(lhs, rhs, lhs->body.kind, rhs->body.kind);
                }

                if (respond) {
                    float overlap = (lhs->radius + rhs->radius) - dist;

                    PPVec3 adjustment_lhs, adjustment_rhs;
                    pp_vec3_scale(&c.n, overlap * 0.5f, &adjustment_lhs);
                    pp_vec3_scale(&c.n, overlap * 0.5f, &adjustment_rhs);
                    pp_vec3_add(&lhs->body.pos, &adjustment_lhs, &lhs->body.pos);
                    pp_vec3_sub(&rhs->body.pos, &adjustment_rhs, &rhs->body.pos);

                    PPVec3 rel_vel;
                    pp_vec3_sub(&rhs->body.vel, &lhs->body.vel, &rel_vel);   // v_rhs – v_lhs
                    float vel_along_normal = pp_vec3_dot(&rel_vel, &c.n);
                    if (vel_along_normal < 0) {
                        PPVec3 penetration, tangent;
                        pp_vec3_scale(&c.n, vel_along_normal, &penetration);
                        pp_vec3_sub(&rel_vel, &penetration, &tangent);

                        // Moving towards each other
                        float r = fmax(lhs->body.bounce, rhs->body.bounce);
                        float f = fmin(lhs->body.friction, rhs->body.friction);

                        float inv_mass_sum = (1.0f / lhs->body.mass) + (1.0f / rhs->body.mass);
                        float j_n = -(1.0f + r) * vel_along_normal / inv_mass_sum;
                        PPVec3 impulse_n;
                        pp_vec3_scale(&c.n, j_n, &impulse_n);

                        float jt = -pp_vec3_dot(&rel_vel, &tangent) / inv_mass_sum;
                        jt = fmax(-j_n * f, fmin(jt, j_n * f));   // clamp to μ·|j_n|
                        PPVec3 impulse_t;
                        pp_vec3_normalize(&tangent);      // ensure unit tangent
                        pp_vec3_scale(&tangent, jt, &impulse_t);

                        PPVec3 impulse;
                        pp_vec3_add(&impulse_n, &impulse_t, &impulse);

                        PPVec3 dv_lhs, dv_rhs;
                        pp_vec3_scale(&impulse,  1.0f / lhs->body.mass, &dv_lhs);
                        pp_vec3_scale(&impulse,  1.0f / rhs->body.mass, &dv_rhs);

                        pp_vec3_add(&lhs->body.vel, &dv_lhs, &lhs->body.vel);   // v_lhs ← v_lhs + Δv
                        pp_vec3_sub(&rhs->body.vel, &dv_rhs, &rhs->body.vel);   // v_rhs ← v_rhs + Δv
                    }
                }
            }
        }

        for(int j = 0; j < tri_count; ++j) {
            const PPTriangle* tri = tris + j;

            PPVec3 p, d;
            pp_vec3_scale(&tri->n, -1.0f, &d);
            float dist;
            if(pp_tri_intersect(tri, &lhs->body.pos, &d, &p, &dist)) {
                if(dist <= lhs->radius) {
                    PPCollision c;
                    pp_fill_collision_info_sphere_triangle(lhs, tri, &p, &c);

                    bool respond = true;
                    const struct _PPCollisionMapEntry* cb = pp_collision_map_search(lhs->body.kind, tri->kind);

                    if(cb) {
                        respond = cb->collision_callback(lhs, tri, lhs->body.kind, tri->kind);
                    }

                    if(respond) {
                        float overlap = lhs->radius - dist;

                        // Move the sphere out of overlap immediately
                        PPVec3 adjustment;
                        pp_vec3_scale(&c.n, overlap, &adjustment);
                        pp_vec3_add(&lhs->body.pos, &adjustment, &lhs->body.pos);

                        // Reflect the velocity based on the collision normal
                        float vel_along_normal = pp_vec3_dot(&lhs->body.vel, &c.n);
                        if (vel_along_normal < 0) {
                            // Apply restitution
                            PPVec3 vel_normal, vel_tangent;
                            pp_vec3_scale(&c.n, vel_along_normal, &vel_normal);
                            pp_vec3_sub(&lhs->body.vel, &vel_normal, &vel_tangent);

                            float r = 1.0f + fmax(0 /*tri->bounce*/, lhs->body.bounce);
                            float f = fmin(tri->friction, lhs->body.friction);

                            PPVec3 impulse, friction_impulse;

                            pp_vec3_scale(&vel_normal, -r, &impulse);
                            pp_vec3_scale(&vel_tangent, -f, &friction_impulse);
                            pp_vec3_add(&impulse, &friction_impulse, &impulse);
                            pp_vec3_add(&lhs->body.vel, &impulse, &lhs->body.vel);

                            PPVec3 contact_offset;
                            pp_vec3_sub(&c.p, &lhs->body.pos, &contact_offset);

                            // Calculate the contact impulse
                            PPVec3 contact_impulse;
                            pp_vec3_scale(&impulse, -1.0f, &contact_impulse);

                            // Calculate torque due to the collision (Torque = r x F)
                            PPVec3 torque;
                            pp_vec3_cross(&contact_offset, &contact_impulse, &torque);

                            // Assuming a simplified moment of inertia (I) as (2/5) * mass * radius^2 for the sphere
                            float I = (2.0f / 5.0f) * lhs->body.mass * lhs->radius * lhs->radius;

                            // Change in angular velocity due to torque = torque / moment of inertia
                            PPVec3 angular_acceleration;
                            pp_vec3_scale(&torque, 1.0f / I, &angular_acceleration);

                            // Update the angular velocity
                            pp_vec3_scale(&angular_acceleration, t, &angular_acceleration); // Scale by time step
                            pp_vec3_add(&lhs->body.a_vel, &angular_acceleration, &lhs->body.a_vel);

                            // float rolling_friction = 0.3f;
                            // Vec3 rolling_friction_force;
                            // vec3_scale(&lhs->body.a_vel, -rolling_friction * t, &rolling_friction_force);
                            // vec3_add(&lhs->body.a_vel, &rolling_friction_force, &lhs->body.a_vel);
                        }
                    }
                }
            }
        }
    }
}

#endif
