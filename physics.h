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

    BodyKind kind;
} Body;

typedef struct _Sphere {
    Body body;
    float radius;
    bool is_alive;
} Sphere;

typedef struct _OBB {
    Body body;
    Vec3 extents;
    Vec3 axes[3];
    bool is_alive;
} OBB;

typedef struct _Triangle {
    Vec3 v[3];
    Vec3 n;
    Plane p;
    BodyKind kind;
} Triangle;

extern Vec3* vec3_init(Vec3* v);
extern Vec3* vec3_set(Vec3* v, float x, float y, float z);

extern void physics_step(float t);

extern Triangle* physics_create_triangle(const Vec3* v1, const Vec3* v2, const Vec3* v3, BodyKind kind);
extern size_t physics_triangle_count();
extern const Triangle* physics_triangle_at(size_t i);

extern Sphere* physics_create_sphere(float radius, const Vec3* pos, float mass, BodyKind kind);
extern void physics_destroy_sphere(Sphere* s);
extern void physics_set_gravity(const Vec3* v);

extern bool collision_map_add(BodyKind kind1, BodyKind kind2, bool (*callback)(const void*, const void*, BodyKind, BodyKind));

extern Sphere* sphere_set_bounce(Sphere* s, float b);
extern void sphere_add_force(Sphere* s, const Vec3* force);

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

Sphere* sphere_init(Sphere* s, float radius, const Vec3* pos, float mass, BodyKind kind) {
    s->radius = radius;
    s->is_alive = true;

    vec3_init(&s->body.vel);
    vec3_init(&s->body.acc);
    vec3_init(&s->body.a_vel);
    vec3_init(&s->body.a_acc);
    quat_init(&s->body.rot);
    vec3_set(&s->body.pos, pos->xyz[0], pos->xyz[1], pos->xyz[2]);
    s->body.kind = kind;
    s->body.mass = mass;
    s->body.damping = 0.0f;
    sphere_set_bounce(s, 0.5f);
    return s;
}

Sphere* sphere_set_bounce(Sphere* s, float b) {
    if(b < 0.0f || b > 1.0f) {
        return NULL;
    }

    s->body.bounce = b;
    return s;
}

void sphere_add_force(Sphere* s, const Vec3* force) {
    Vec3 acceleration;

    // Ensure you do not divide by zero
    if (s->body.mass > 0) {
        // a = F / m
        vec3_scale(force, 1.0f / s->body.mass, &acceleration);

        // Add acceleration to the sphere's current acceleration
        vec3_add(&s->body.acc, &acceleration, &s->body.acc);
    }
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

        // Apply linear damping
        vec3_scale(&sp->body.vel, 1.0f - sp->body.damping, &sp->body.vel);

        Vec3 scaled_ang_vel; // Temporary variable to store scaled angular velocity
        vec3_scale(&sp->body.a_acc, t, &scaled_ang_vel);
        vec3_add(&sp->body.a_vel, &scaled_ang_vel, &sp->body.a_vel);

        // Apply angular damping if desired
        vec3_scale(&sp->body.a_vel, 1.0f - sp->body.a_damping, &sp->body.a_vel);

        // Reset the acceleration
        vec3_init(&sp->body.acc);
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
                    // Correctly calculate overlap
                    float overlap = (lhs->radius + rhs->radius) - dist;

                    // Calculate how much to separate each sphere based on their radii
                    float totalRadius = lhs->radius + rhs->radius;

                    // Calculate the ratio based on the spheres' radii
                    float lhs_correction = overlap * (lhs->radius / totalRadius);
                    float rhs_correction = overlap * (rhs->radius / totalRadius);

                    Vec3 add;

                    // Separate left-hand sphere
                    vec3_scale(&c.n, lhs_correction, &add);
                    vec3_add(&lhs->body.pos, &add, &lhs->body.pos);

                    // Separate right-hand sphere
                    vec3_scale(&c.n, -rhs_correction, &add);
                    vec3_add(&rhs->body.pos, &add, &rhs->body.pos);

                    // Directly calculate relative velocity
                    float rel_vel = vec3_dot(&rhs->body.vel, &c.n) - vec3_dot(&lhs->body.vel, &c.n);

                    if (rel_vel < 0) {
                        // Calculate new velocities based on the bounce values
                        float bounce_factor = (lhs->body.bounce + rhs->body.bounce) / 2; // Average bounce factor

                        // Calculate impulse
                        float impulse = (1 + bounce_factor) * rel_vel / (1 / lhs->body.mass + 1 / rhs->body.mass);

                        // Adjustments for the relative velocity
                        float lhs_impulse = impulse / lhs->body.mass;
                        float rhs_impulse = impulse / rhs->body.mass;

                        vec3_add(&lhs->body.vel, vec3_scale(&c.n, lhs_impulse, &add), &lhs->body.vel);
                        vec3_sub(&rhs->body.vel, vec3_scale(&c.n, rhs_impulse, &add), &rhs->body.vel);

                        Vec3 contact_offset_lhs, contact_offset_rhs;
                        vec3_sub(&c.p, &lhs->body.pos, &contact_offset_lhs);
                        vec3_sub(&c.p, &rhs->body.pos, &contact_offset_rhs);

                        Vec3 torque_lhs, torque_rhs;
                        vec3_cross(&contact_offset_lhs, &c.n, &torque_lhs);
                        vec3_cross(&contact_offset_rhs, &c.n, &torque_rhs);

                        // Assuming a simplified moment of inertia (I) as mass * radius^2 for spheres
                        float I_lhs = (2.0f / 5.0f) * lhs->body.mass * lhs->radius * lhs->radius;
                        float I_rhs = (2.0f / 5.0f) * rhs->body.mass * rhs->radius * rhs->radius;

                        // Change in angular velocity due to torque = torque / moment of inertia
                        float Il = 1.0f / I_lhs;
                        vec3_scale(&torque_lhs, Il, &lhs->body.a_vel);

                        float Ir = 1.0f / I_rhs;
                        vec3_scale(&torque_rhs, Ir, &rhs->body.a_vel);
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
                            Vec3 reflection;
                            vec3_scale(&c.n,
                                       2 * lhs->body.bounce * vel_along_normal,
                                       &reflection);
                            vec3_sub(&lhs->body.vel, &reflection, &lhs->body.vel);

                            Vec3 contact_offset;
                            vec3_sub(&c.p, &lhs->body.pos, &contact_offset);

                            // Calculate torque due to the collision (Torque = r x F)
                            Vec3 torque;
                            vec3_cross(&contact_offset, &c.n, &torque);

                            // Assuming a simplified moment of inertia (I) as mass * radius^2 for the sphere
                            float I = (2.0f / 5.0f) * lhs->body.mass * lhs->radius * lhs->radius;

                            // Change in angular velocity due to torque = torque / moment of inertia
                            lhs->body.a_vel.xyz[0] += torque.xyz[0] / I;
                            lhs->body.a_vel.xyz[1] += torque.xyz[1] / I;
                            lhs->body.a_vel.xyz[2] += torque.xyz[2] / I;
                        }
                    }
                }
            }
        }
    }
}

#endif
