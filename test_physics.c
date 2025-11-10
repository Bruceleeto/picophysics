#define PHYSICS_IMPLEMENTATION
#include "physics.h"

#define CHECK(expr, msg) \
    if(!(expr)) {        \
        fprintf(stderr, "CHECK FAILED (%d): %s", __LINE__, msg); \
        exit(1); \
    } \

#define CLOSE(a, b) \
    ((a) > (b) - 0.000001f) && ((a) < (b) + 0.000001f)

int main(int argc, char* argv[]) {
    Vec3 test;

    // Vector tests

    vec3_init(&test);
    CHECK(test.xyz[0] == 0.0f, "Unexpected value\n");
    CHECK(test.xyz[1] == 0.0f, "Unexpected value\n");
    CHECK(test.xyz[2] == 0.0f, "Unexpected value\n");

    vec3_set(&test, 1, 1, 1);
    CHECK(test.xyz[0] == 1.0f, "Unexpected value\n");
    CHECK(test.xyz[1] == 1.0f, "Unexpected value\n");
    CHECK(test.xyz[2] == 1.0f, "Unexpected value\n");

    Vec3 ret;
    vec3_add(&test, &test, &ret);
    CHECK(ret.xyz[0] == 2.0f, "Unexpected value\n");
    CHECK(ret.xyz[1] == 2.0f, "Unexpected value\n");
    CHECK(ret.xyz[2] == 2.0f, "Unexpected value\n");

    vec3_sub(&ret, &test, &ret);
    CHECK(ret.xyz[0] == 1.0f, "Unexpected value\n");
    CHECK(ret.xyz[1] == 1.0f, "Unexpected value\n");
    CHECK(ret.xyz[2] == 1.0f, "Unexpected value\n");

    vec3_scale(&ret, 2.0f, &ret);
    CHECK(ret.xyz[0] == 2.0f, "Unexpected value\n");
    CHECK(ret.xyz[1] == 2.0f, "Unexpected value\n");
    CHECK(ret.xyz[2] == 2.0f, "Unexpected value\n");

    CHECK(vec3_normalize(&ret), "Couldn't normalize\n");
    float l = vec3_length(&ret);
    CHECK(CLOSE(l, 1.0f), "Unexpected value\n");

    Vec3 v1, v2;
    vec3_set(&v1, 1.0f, 0.0f, 0.0f);
    vec3_set(&v2, 0.0f, 0.0f, 1.0f);

    vec3_cross(&v1, &v2, &ret);

    CHECK(ret.xyz[0] == 0.0f, "Unexpected value\n");
    CHECK(ret.xyz[1] == -1.0f, "Unexpected value\n");
    CHECK(ret.xyz[2] == 0.0f, "Unexpected value\n");

    float d = vec3_dot(&v1, &v2);
    CHECK(d == 0.0f, "Unexpected value\n");

    // Quaternion tests

    Quaternion q;
    quat_init(&q);

    CHECK(q.xyzw[0] == 0.0f, "Unexpected value\n");
    CHECK(q.xyzw[1] == 0.0f, "Unexpected value\n");
    CHECK(q.xyzw[2] == 0.0f, "Unexpected value\n");
    CHECK(q.xyzw[3] == 1.0f, "Unexpected value\n");

    // Tri intersection

    Triangle tri;
    Vec3 origin, dir;
    vec3_set(&origin, 0, 1, 0);
    vec3_set(&dir, 0, -1, 0);

    vec3_set(&tri.v[0], -1, 0, -1);
    vec3_set(&tri.v[1], 0, 0, 1);
    vec3_set(&tri.v[2], 1, 0, -1);

    CHECK(tri_intersect(&tri, &origin, &dir, &ret), "Didn't intersect");

    CHECK(ret.xyz[0] == 0.0f, "Unexpected value\n");
    CHECK(ret.xyz[1] == 0.0f, "Unexpected value\n");
    CHECK(ret.xyz[2] == 0.0f, "Unexpected value\n");

    vec3_set(&dir, 0, 1, 0);

    CHECK(tri_intersect(&tri, &origin, &dir, &ret) == NULL, "Unexpectedly intersected");

    Vec3 p1, p2;
    vec3_set(&p1, -0.5f, 0.0f, 0.0f);
    vec3_set(&p2, 0.5f, 0.0f, 0.0f);

    Sphere* lhs = physics_create_sphere(1.0f, &p1, 1, 0);
    Sphere* rhs = physics_create_sphere(1.0f, &p2, 1, 0);

    physics_step(1.0f / 60.0f);

    CHECK(lhs->body.pos.xyz[0] == -1.0f, "Unexpected position\n");
    CHECK(rhs->body.pos.xyz[0] == 1.0f, "Unexpected position\n");

    physics_destroy_sphere(rhs);
    CHECK(rhs->is_alive == false, "Sphere unexpectedly alive\n");

    Vec3 v3;

    vec3_set(&v1, -10, 0, -10);
    vec3_set(&v2, 0, 0, 10);
    vec3_set(&v3, 10, 0, -10);

    vec3_set(&lhs->body.pos, 0.0f, 0.5f, 0.0f);

    physics_create_triangle(&v1, &v2, &v3, 0);

    physics_step(1.0f / 60.0f);

    CHECK(lhs->body.pos.xyz[1] == 1.0f, "Body position didn't move");

    return 0;
}
