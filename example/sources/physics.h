#ifndef PICOPHYSICS_H
#define PICOPHYSICS_H

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _Vec3 {
    float xyz[3];
} Vec3;

typedef struct _Quaternion {
    float xyzw[4];
} Quaternion;

typedef struct _Plane {
    Vec3 n;
    float d;
} Plane;

struct _Sphere;

typedef uint8_t BodyKind;

typedef struct _Collision {
    Vec3 p;
    Vec3 n;
    void* obj1;
    void* obj2;

    BodyKind kind1;
    BodyKind kind2;
} Collision;

enum AxisLock {
    AXIS_LOCK_NONE,
    AXIS_LOCK_PITCH = 0x1,
    AXIS_LOCK_YAW = 0x2,
    AXIS_LOCK_ROLL = 0x4,
    AXIS_LOCK_PITCH_AND_ROLL = AXIS_LOCK_PITCH | AXIS_LOCK_ROLL,
    AXIS_LOCK_PITCH_AND_YAW = AXIS_LOCK_PITCH | AXIS_LOCK_YAW,
    AXIS_LOCK_YAW_AND_ROLL = AXIS_LOCK_YAW | AXIS_LOCK_ROLL,
    AXIS_LOCK_ALL = AXIS_LOCK_PITCH | AXIS_LOCK_YAW | AXIS_LOCK_ROLL
};

typedef struct _Body {
    Vec3 pos;
    float bounce;
    Quaternion rot;

    Vec3 vel;
    Vec3 acc;
    float damping;

    Vec3 a_vel;
    Vec3 a_acc;
    float a_damping;

    float mass;
    float friction;

    BodyKind kind;

    AxisLock lock;
} Body;

typedef struct _Sphere {
    Body body;
    float radius;
    bool is_alive;
    void* user_data;
} Sphere;

typedef struct _Triangle {
    Vec3 v[3];
    Vec3 n;
    Plane p;
    BodyKind kind;
    float friction;
} Triangle;

extern Vec3* vec3_init(Vec3* v);
extern Vec3* vec3_set(Vec3* v, float x, float y, float z);
extern Vec3* vec3_scale(const Vec3* v1, float t, Vec3* out);

extern void physics_step(float t);

extern Triangle* physics_create_triangle(const Vec3* v1, const Vec3* v2, const Vec3* v3, BodyKind kind);
extern size_t physics_triangle_count();
extern const Triangle* physics_triangle_at(size_t i);

extern Sphere* physics_create_sphere(float radius, const Vec3* pos, float mass, BodyKind kind);
extern void physics_destroy_sphere(Sphere* s);
extern void physics_set_gravity(const Vec3* v);

extern bool collision_map_add(BodyKind kind1, BodyKind kind2, bool (*callback)(const void*, const void*, BodyKind, BodyKind));

extern Sphere* sphere_set_bounce(Sphere* s, float b);
extern void sphere_add_force(Sphere* s, float x, float y, float z);
extern void sphere_add_angular_force(Sphere* s, float x, float y, float z);
extern void sphere_lock_axis(Sphere* s, AxisLock lock);
extern void sphere_set_angular_damping(Sphere* s, float d);
extern void sphere_set_damping(Sphere* s, float d);
extern void sphere_get_forward(Sphere* s, Vec3* f);
extern void sphere_set_position(Sphere* s, float x, float y, float z);
extern void sphere_get_position(Sphere* s, Vec3* pos);
extern float sphere_get_radius(Sphere* s);
extern void sphere_set_user_data(Sphere* s, void* data);
extern void* sphere_get_user_data(const Sphere* s);

// Given a direction vector, this will apply a force to attempt to look towards it
extern void sphere_look_at(Sphere* s, float x, float y, float z);

#ifdef __cplusplus
}
#endif

#endif

#ifdef PHYSICS_IMPLEMENTATION

#define PHYSICS_MAX_SPHERES 32
#define PHYSICS_MAX_TRIANGLES 128

static Sphere spheres[PHYSICS_MAX_SPHERES];
static int sphere_count = 0;

static Triangle tris[PHYSICS_MAX_TRIANGLES];
static int tri_count = 0;

static struct _CollisionMapEntry {
    BodyKind kind1;
    BodyKind kind2;
    bool (*collision_callback)(const void*, const void*, BodyKind, BodyKind);
} collision_map[32];

static int collision_map_count = 0;

static Vec3 gravity = {.xyz = {0.0f, 0.0f, 0.0f}};
static float gravity_magnitude = 0.0f;

Vec3* vec3_init(Vec3* v) {
    v->xyz[0] = 0.0f;
    v->xyz[1] = 0.0f;
    v->xyz[2] = 0.0f;
    return v;
}

Vec3* vec3_set(Vec3* v, float x, float y, float z) {
    v->xyz[0] = x;
    v->xyz[1] = y;
    v->xyz[2] = z;
    return v;
}

Vec3* vec3_assign(Vec3* target, const Vec3* source) {
    vec3_set(target, source->xyz[0], source->xyz[1], source->xyz[2]);
    return target;
}

Vec3* vec3_add(const Vec3* v1, const Vec3* v2, Vec3* out) {
    out->xyz[0] = v1->xyz[0] + v2->xyz[0];
    out->xyz[1] = v1->xyz[1] + v2->xyz[1];
    out->xyz[2] = v1->xyz[2] + v2->xyz[2];
    return out;
}

Vec3* vec3_sub(const Vec3* v1, const Vec3* v2, Vec3* out) {
    out->xyz[0] = v1->xyz[0] - v2->xyz[0];
    out->xyz[1] = v1->xyz[1] - v2->xyz[1];
    out->xyz[2] = v1->xyz[2] - v2->xyz[2];
    return out;
}

Vec3* vec3_scale(const Vec3* v1, float t, Vec3* out) {
    out->xyz[0] = v1->xyz[0] * t;
    out->xyz[1] = v1->xyz[1] * t;
    out->xyz[2] = v1->xyz[2] * t;
    return out;
}

float vec3_length(const Vec3* v1) {
    return sqrtf(v1->xyz[0] * v1->xyz[0] + v1->xyz[1] * v1->xyz[1] + v1->xyz[2] * v1->xyz[2]);
}

float vec3_dist(const Vec3* v1, const Vec3* v2) {
    Vec3 tmp;
    vec3_sub(v2, v1, &tmp);
    return vec3_length(&tmp);
}

Vec3* vec3_cross(const Vec3 *v1, const Vec3 *v2, Vec3 *out) {
    out->xyz[0] = v1->xyz[1] * v2->xyz[2] - v1->xyz[2] * v2->xyz[1];
    out->xyz[1] = v1->xyz[2] * v2->xyz[0] - v1->xyz[0] * v2->xyz[2];
    out->xyz[2] = v1->xyz[0] * v2->xyz[1] - v1->xyz[1] * v2->xyz[0];
    return out;
}

float vec3_dot(const Vec3 *v1, const Vec3 *v2) {
    return v1->xyz[0] * v2->xyz[0] +
           v1->xyz[1] * v2->xyz[1] +
           v1->xyz[2] * v2->xyz[2];
}

bool vec3_normalize(Vec3 *v) {
    float length = vec3_length(v);

    // Check for zero-length vector to avoid division by zero
    if (length > 0.0f) {
        v->xyz[0] /= length;
        v->xyz[1] /= length;
        v->xyz[2] /= length;
        return true;
    } else {
        vec3_init(v);
        return false;
    }
}

Quaternion* quat_init(Quaternion* q) {
    q->xyzw[0] = 0.0f;
    q->xyzw[1] = 0.0f;
    q->xyzw[2] = 0.0f;
    q->xyzw[3] = 1.0f;
    return q;
}

Quaternion* quat_set(Quaternion* q, float x, float y, float z, float w) {
    q->xyzw[0] = x;
    q->xyzw[1] = y;
    q->xyzw[2] = z;
    q->xyzw[3] = w;
    return q;
}

void quat_from_angular_velocity(const Vec3* a_vel, float dt, Quaternion* q_rot) {
    float angle = vec3_length(a_vel) * dt; // Calculate the rotation angle
    if (angle > 0.0f) {
        Vec3 axis;
        vec3_assign(&axis, a_vel);
        vec3_normalize(&axis); // Normalize the angular velocity to get the axis of rotation

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
        quat_init(q_rot);
    }
}

void quat_multiply(const Quaternion* q1, const Quaternion* q2, Quaternion* result) {
    Quaternion tmp;
    tmp.xyzw[0] = q1->xyzw[3] * q2->xyzw[0] + q1->xyzw[0] * q2->xyzw[3] + q1->xyzw[1] * q2->xyzw[2] - q1->xyzw[2] * q2->xyzw[1];
    tmp.xyzw[1] = q1->xyzw[3] * q2->xyzw[1] - q1->xyzw[0] * q2->xyzw[2] + q1->xyzw[1] * q2->xyzw[3] + q1->xyzw[2] * q2->xyzw[0];
    tmp.xyzw[2] = q1->xyzw[3] * q2->xyzw[2] + q1->xyzw[0] * q2->xyzw[1] - q1->xyzw[1] * q2->xyzw[0] + q1->xyzw[2] * q2->xyzw[3];
    tmp.xyzw[3] = q1->xyzw[3] * q2->xyzw[3] - q1->xyzw[0] * q2->xyzw[0] - q1->xyzw[1] * q2->xyzw[1] - q1->xyzw[2] * q2->xyzw[2];

    *result = tmp;
}

void quat_normalize(Quaternion* q) {
    float norm = sqrtf(q->xyzw[0] * q->xyzw[0] + q->xyzw[1] * q->xyzw[1] + q->xyzw[2] * q->xyzw[2] + q->xyzw[3] * q->xyzw[3]);
    if (norm > 0) {
        q->xyzw[0] /= norm;
        q->xyzw[1] /= norm;
        q->xyzw[2] /= norm;
        q->xyzw[3] /= norm;
    }
}

static void quat_forward(const Quaternion* q, Vec3* out)
{
    float x = q->xyzw[0];
    float y = q->xyzw[1];
    float z = q->xyzw[2];
    float w = q->xyzw[3];

    out->xyz[0] = 2.0f * (x * z + w * y);
    out->xyz[1] = 2.0f * (y * z - w * x);
    out->xyz[2] = 1.0f - 2.0f * (x * x + y * y);
}

Vec3* tri_intersect(const Triangle* tri, const Vec3* o, const Vec3* d, Vec3* out) {

    const float e = FLT_EPSILON;
    Vec3 edge1, edge2, cross_e1, cross_e2, s;
    vec3_sub(&tri->v[1], &tri->v[0], &edge1);
    vec3_sub(&tri->v[2], &tri->v[0], &edge2);
    vec3_cross(d, &edge2, &cross_e2);

    float det = vec3_dot(&edge1, &cross_e2);

    if(det > -e && det < e) {
        return NULL;
    }

    float inv_det = 1.0f / det;
    vec3_sub(o, &tri->v[0], &s);
    float u = inv_det * vec3_dot(&s, &cross_e2);

    if ((u < 0 && fabsf(u) > e) || (u > 1 && fabsf(u-1) > e)) {
        return NULL;
    }

    vec3_cross(&s, &edge1, &cross_e1);
    float v = inv_det * vec3_dot(d, &cross_e1);

    if ((v < 0 && fabsf(v) > e) || (u + v > 1 && fabsf(u + v - 1) > e)) {
        return NULL;
    }

    float t = inv_det * vec3_dot(&edge2, &cross_e1);

    if(t <= e) {
        return NULL;
    }

    vec3_scale(d, t, out);
    vec3_add(out, o, out);
    return out;
}

Sphere* sphere_set_bounce(Sphere* s, float b);

void sphere_set_angular_damping(Sphere* s, float d) {
    if(d < 0.0f || d > 1.0f) {
        return;
    }

    s->body.a_damping = d;
}

void sphere_set_damping(Sphere* s, float d) {
    if(d < 0.0f || d > 1.0f) {
        return;
    }

    s->body.damping = d;
}

void sphere_get_forward(Sphere* s, Vec3* f) {
    quat_forward(&s->body.rot, f);
}

float sphere_get_radius(Sphere* s) {
    return s->radius;
}

void quat_from_axis_angle(Quaternion* q, const Vec3* axis, float angle) {
    Vec3 a;
    vec3_assign(&a, axis);
    vec3_normalize(&a);

    float half = angle * 0.5f;
    float s = sinf(half);   // sin(θ/2)
    float c = cosf(half);   // cos(θ/2)

    q->xyzw[0] = a.xyz[0] * s;   // axis.x * sin(θ/2)
    q->xyzw[1] = a.xyz[1] * s;   // axis.y * sin(θ/2)
    q->xyzw[2] = a.xyz[2] * s;   // axis.z * sin(θ/2)
    q->xyzw[3] = c;         // cos(θ/2)

    quat_normalize(q);
}

void sphere_look_at(Sphere* s, float x, float y, float z) {
    Vec3 t, f, c;
    vec3_set(&t, x, y, z);
    sphere_get_forward(s, &f);
    vec3_cross(&f, &t, &c);
    float d = vec3_dot(&f, &t);

    // Build error quaternion
    Quaternion q_err;
    if (d < -0.9999f) {                     // opposite direction
        Vec3 ortho, axis;
        if(fabsf(f.xyz[0]) < 0.9f) {
            vec3_set(&ortho, 1, 0, 0);
        } else {
            vec3_set(&ortho, 0, 1, 0);
        }

        vec3_cross(&f, &ortho, &axis);
        vec3_normalize(&axis);

        // 180° rotation
        quat_from_axis_angle(&q_err, &axis, M_PI);
    } else {
        Vec3 axis;
        float s = sqrtf((1.0f + d) * 2.0f);
        axis.xyz[0] = c.xyz[0] / s;
        axis.xyz[1] = c.xyz[1] / s;
        axis.xyz[2] = c.xyz[2] / s;

        quat_set(&q_err, axis.xyz[0], axis.xyz[1], axis.xyz[2], s * 0.5f);
    }

    quat_normalize(&q_err);

    // Angular error vector (approximation)
    Vec3 error;
    vec3_set(&error, 2.0f * q_err.xyzw[0], 2.0f * q_err.xyzw[1], 2.0f * q_err.xyzw[2]);

    Vec3 torque, a, b;

    // Configurable
    float kp = 10.0f;
    float kd = 1.0f;

    // Vector3 torque = -cfg.kp * error - cfg.kd * a_vel;
    vec3_scale(&error, -kp, &a);
    vec3_scale(&s->body.a_vel, kd, &b);
    vec3_sub(&a, &b, &torque);
    sphere_add_angular_force(s, torque.xyz[0], torque.xyz[1], torque.xyz[2]);
}

Sphere* sphere_init(Sphere* s, float radius, const Vec3* pos, float mass, BodyKind kind) {
    s->radius = radius;
    s->is_alive = true;
    s->user_data = NULL;

    vec3_init(&s->body.vel);
    vec3_init(&s->body.acc);
    vec3_init(&s->body.a_vel);
    vec3_init(&s->body.a_acc);
    quat_init(&s->body.rot);
    vec3_set(&s->body.pos, pos->xyz[0], pos->xyz[1], pos->xyz[2]);
    s->body.kind = kind;
    s->body.mass = mass;
    s->body.friction = 0.3f;
    s->body.damping = 0.01f;
    s->body.a_damping = 0.02f;
    sphere_set_bounce(s, 0.5f);
    return s;
}

void sphere_set_user_data(Sphere* s, void* data) {
    s->user_data = data;
}

void* sphere_get_user_data(const Sphere* s) {
    return s->user_data;
}

void sphere_lock_axis(Sphere* s, AxisLock lock) {
    s->body.lock = lock;
}

void sphere_get_position(Sphere*s, Vec3* pos) {
    vec3_assign(pos, &s->body.pos);
}

void sphere_set_position(Sphere*s, float x, float y, float z) {
    vec3_set(&s->body.pos, x, y, z);
}

Sphere* sphere_set_bounce(Sphere* s, float b) {
    if(b < 0.0f || b > 1.0f) {
        return NULL;
    }

    s->body.bounce = b;
    return s;
}

void sphere_add_force(Sphere* s, float x, float y, float z) {
    Vec3 force;
    vec3_set(&force, x, y, z);

    Vec3 acceleration;

    // Ensure you do not divide by zero
    if (s->body.mass > 0) {
        // a = F / m
        vec3_scale(&force, 1.0f / s->body.mass, &acceleration);

        // Add acceleration to the sphere's current acceleration
        vec3_add(&s->body.acc, &acceleration, &s->body.acc);
    }
}

void sphere_add_angular_force(Sphere* s, float tx, float ty, float tz)
{
    Vec3 torque;
    vec3_set(&torque, tx, ty, tz);

    float I = (2.0f / 5.0f) * s->body.mass * s->radius * s->radius;

    if (I <= 0.0f) return;   // nothing to do for mass‑less or zero‑radius objects

    Vec3 ang_acc;
    vec3_scale(&torque, 1.0f / I, &ang_acc);
    vec3_add(&s->body.a_acc, &ang_acc, &s->body.a_acc);
}

const struct _CollisionMapEntry* collision_map_search(BodyKind kind1, BodyKind kind2) {
    for(int i = 0; i < collision_map_count; ++i) {
        struct _CollisionMapEntry* entry = &collision_map[i];
        if((entry->kind1 == kind1 && entry->kind2 == kind2) || (entry->kind2 == kind1 && entry->kind1 == kind2)) {
            return entry;
        }
    }

    return NULL;
}

bool collision_map_add(BodyKind kind1, BodyKind kind2, bool (*callback)(const void*, const void*, BodyKind, BodyKind)) {
    if(!collision_map_search(kind1, kind2)) {
        struct _CollisionMapEntry* entry = &collision_map[collision_map_count++];
        entry->kind1 = kind1;
        entry->kind2 = kind2;
        entry->collision_callback = callback;
        return true;
    }

    return false;
}

static void fill_collision_info_sphere_sphere(const Sphere* lhs, const Sphere* rhs, Collision* c) {
    vec3_sub(&lhs->body.pos, &rhs->body.pos, &c->n);
    float l = vec3_length(&c->n);

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

static void fill_collision_info_sphere_triangle(const Sphere* lhs, const Triangle* tri, const Vec3* p, Collision* c) {
    vec3_scale(&tri->n, 1.0f, &c->n); // Copy
    vec3_scale(p, 1.0f, &c->p); // Copy
}

Sphere* physics_create_sphere(float radius, const Vec3* pos, float mass, BodyKind kind) {
    Sphere* ret = &spheres[sphere_count++];
    sphere_init(ret, radius, pos, mass, kind);
    return ret;
}

Triangle* physics_create_triangle(const Vec3* v1, const Vec3* v2, const Vec3* v3, BodyKind kind) {
    Triangle* tri = &tris[tri_count++];
    vec3_assign(&tri->v[0], v1);
    vec3_assign(&tri->v[1], v2);
    vec3_assign(&tri->v[2], v3);

    Vec3 e1, e2;
    vec3_sub(v2, v1, &e1);
    vec3_sub(v3, v1, &e2);

    vec3_cross(&e1, &e2, &tri->n);
    vec3_normalize(&tri->n);

    tri->friction = 0.1f;
    tri->kind = kind;

    return tri;
}

size_t physics_triangle_count() {
    return tri_count;
}

const Triangle* physics_triangle_at(size_t i) {
    return tris + i;
}

void physics_destroy_sphere(Sphere* s) {
    s->is_alive = false;
}

void physics_set_gravity(const Vec3* v) {
    vec3_assign(&gravity, v);
    gravity_magnitude = vec3_length(&gravity);
}

void physics_step(float t) {
    Vec3 scaled_vel;

    // Apply acceleration to velocity
    for(int i = 0; i < sphere_count; ++i) {
        Sphere* sp = &spheres[i];
        if(!sp->is_alive) {
            continue;
        }

        // Apply gravity to acceleration before applying acceleration
        // to velocity
        vec3_add(&sp->body.acc, &gravity, &sp->body.acc);

        vec3_scale(&sp->body.acc, t, &scaled_vel);
        vec3_add(&sp->body.vel, &scaled_vel, &sp->body.vel);
        vec3_init(&sp->body.acc);

        // Apply linear damping
        vec3_scale(&sp->body.vel, 1.0f - sp->body.damping, &sp->body.vel);

        if(sp->body.lock) {
            if((sp->body.lock & AXIS_LOCK_PITCH) == AXIS_LOCK_PITCH) {
                sp->body.a_acc.xyz[0] = 0.0f;
                sp->body.a_vel.xyz[0] = 0.0f;
            }

            if((sp->body.lock & AXIS_LOCK_YAW) == AXIS_LOCK_YAW) {
                sp->body.a_acc.xyz[1] = 0.0f;
                sp->body.a_vel.xyz[1] = 0.0f;
            }

            if((sp->body.lock & AXIS_LOCK_ROLL) == AXIS_LOCK_ROLL) {
                sp->body.a_acc.xyz[2] = 0.0f;
                sp->body.a_vel.xyz[2] = 0.0f;
            }
        }

        Vec3 scaled_ang_vel; // Temporary variable to store scaled angular velocity
        vec3_scale(&sp->body.a_acc, t, &scaled_ang_vel);
        vec3_add(&sp->body.a_vel, &scaled_ang_vel, &sp->body.a_vel);

        // Apply angular damping if desired
        vec3_scale(&sp->body.a_vel, 1.0f - sp->body.a_damping, &sp->body.a_vel);

        // Reset the acceleration
        vec3_init(&sp->body.a_acc);
    }

    // Move all spheres by their velocity
    for(int i = 0; i < sphere_count; ++i) {
        Sphere* sp = &spheres[i];
        if(!sp->is_alive) {
            continue;
        }

        vec3_scale(&sp->body.vel, t, &scaled_vel);
        vec3_add(&sp->body.pos, &scaled_vel, &sp->body.pos);

        Quaternion q_rot;
        quat_from_angular_velocity(&sp->body.a_vel, t, &q_rot); // Get rotation quaternion from angular velocity
        quat_multiply(&sp->body.rot, &q_rot, &sp->body.rot); // Combine with current rotation
        quat_normalize(&sp->body.rot); // Normalize the quaternion
    }


    for(int i = 0; i < sphere_count; ++i) {
        // Check collision between spheres
        Sphere* lhs = &spheres[i];

        if(!lhs->is_alive) {
            continue;
        }

        for(int j = i + 1; j < sphere_count; ++j) {
            Sphere* rhs = &spheres[j];

            if(!rhs->is_alive) {
                continue;
            }

            float dist = vec3_dist(&lhs->body.pos, &rhs->body.pos);
            if (dist <= (lhs->radius + rhs->radius)) {
                Collision c;
                fill_collision_info_sphere_sphere(lhs, rhs, &c);

                bool respond = true;
                const struct _CollisionMapEntry* cb = collision_map_search(lhs->body.kind, rhs->body.kind);
                if (cb) {
                    respond = cb->collision_callback(lhs, rhs, lhs->body.kind, rhs->body.kind);
                }

                if (respond) {
                    float overlap = (lhs->radius + rhs->radius) - dist;

                    Vec3 adjustment_lhs, adjustment_rhs;
                    vec3_scale(&c.n, overlap * 0.5f, &adjustment_lhs);
                    vec3_scale(&c.n, overlap * 0.5f, &adjustment_rhs);
                    vec3_add(&lhs->body.pos, &adjustment_lhs, &lhs->body.pos);
                    vec3_sub(&rhs->body.pos, &adjustment_rhs, &rhs->body.pos);

                    Vec3 rel_vel;
                    vec3_sub(&rhs->body.vel, &lhs->body.vel, &rel_vel);   // v_rhs – v_lhs
                    float vel_along_normal = vec3_dot(&rel_vel, &c.n);
                    if (vel_along_normal < 0) {
                        Vec3 penetration, tangent;
                        vec3_scale(&c.n, vel_along_normal, &penetration);
                        vec3_sub(&rel_vel, &penetration, &tangent);

                        // Moving towards each other
                        float r = fmax(lhs->body.bounce, rhs->body.bounce);
                        float f = fmin(lhs->body.friction, rhs->body.friction);

                        float inv_mass_sum = (1.0f / lhs->body.mass) + (1.0f / rhs->body.mass);
                        float j_n = -(1.0f + r) * vel_along_normal / inv_mass_sum;
                        Vec3 impulse_n;
                        vec3_scale(&c.n, j_n, &impulse_n);

                        float jt = -vec3_dot(&rel_vel, &tangent) / inv_mass_sum;
                        jt = fmax(-j_n * f, fmin(jt, j_n * f));   // clamp to μ·|j_n|
                        Vec3 impulse_t;
                        vec3_normalize(&tangent);      // ensure unit tangent
                        vec3_scale(&tangent, jt, &impulse_t);

                        Vec3 impulse;
                        vec3_add(&impulse_n, &impulse_t, &impulse);

                        Vec3 dv_lhs, dv_rhs;
                        vec3_scale(&impulse,  1.0f / lhs->body.mass, &dv_lhs);
                        vec3_scale(&impulse,  1.0f / rhs->body.mass, &dv_rhs);

                        vec3_add(&lhs->body.vel, &dv_lhs, &lhs->body.vel);   // v_lhs ← v_lhs + Δv
                        vec3_sub(&rhs->body.vel, &dv_rhs, &rhs->body.vel);   // v_rhs ← v_rhs + Δv
                    }
                }
            }
        }

        for(int j = 0; j < tri_count; ++j) {
            const Triangle* tri = tris + j;

            Vec3 p, d;
            vec3_scale(&tri->n, -1.0f, &d);
            if(tri_intersect(tri, &lhs->body.pos, &d, &p)) {
                float dist = vec3_dist(&lhs->body.pos, &p);

                if(dist <= lhs->radius) {
                    Collision c;
                    fill_collision_info_sphere_triangle(lhs, tri, &p, &c);

                    bool respond = true;
                    const struct _CollisionMapEntry* cb = collision_map_search(lhs->body.kind, tri->kind);

                    if(cb) {
                        respond = cb->collision_callback(lhs, tri, lhs->body.kind, tri->kind);
                    }

                    if(respond) {
                        float overlap = lhs->radius - dist;

                        // Move the sphere out of overlap immediately
                        Vec3 adjustment;
                        vec3_scale(&c.n, overlap, &adjustment);
                        vec3_add(&lhs->body.pos, &adjustment, &lhs->body.pos);

                        // Reflect the velocity based on the collision normal
                        float vel_along_normal = vec3_dot(&lhs->body.vel, &c.n);
                        if (vel_along_normal < 0) {
                            // Apply restitution
                            Vec3 vel_normal, vel_tangent;
                            vec3_scale(&c.n, vel_along_normal, &vel_normal);
                            vec3_sub(&lhs->body.vel, &vel_normal, &vel_tangent);

                            float r = 1.0f + fmax(0 /*tri->bounce*/, lhs->body.bounce);
                            float f = fmin(tri->friction, lhs->body.friction);

                            Vec3 impulse, friction_impulse;

                            vec3_scale(&vel_normal, -r, &impulse);
                            vec3_scale(&vel_tangent, -f, &friction_impulse);
                            vec3_add(&impulse, &friction_impulse, &impulse);
                            vec3_add(&lhs->body.vel, &impulse, &lhs->body.vel);

                            Vec3 contact_offset;
                            vec3_sub(&c.p, &lhs->body.pos, &contact_offset);

                            // Calculate the contact impulse
                            Vec3 contact_impulse;
                            vec3_scale(&impulse, -1.0f, &contact_impulse);

                            // Calculate torque due to the collision (Torque = r x F)
                            Vec3 torque;
                            vec3_cross(&contact_offset, &contact_impulse, &torque);

                            // Assuming a simplified moment of inertia (I) as (2/5) * mass * radius^2 for the sphere
                            float I = (2.0f / 5.0f) * lhs->body.mass * lhs->radius * lhs->radius;

                            // Change in angular velocity due to torque = torque / moment of inertia
                            Vec3 angular_acceleration;
                            vec3_scale(&torque, 1.0f / I, &angular_acceleration);

                            // Update the angular velocity
                            vec3_scale(&angular_acceleration, t, &angular_acceleration); // Scale by time step
                            vec3_add(&lhs->body.a_vel, &angular_acceleration, &lhs->body.a_vel);

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
