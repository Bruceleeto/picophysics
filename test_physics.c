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
    PPVec3 test;

    // Vector tests

    pp_vec3_init(&test);
    CHECK(test.xyz[0] == 0.0f, "Unexpected value\n");
    CHECK(test.xyz[1] == 0.0f, "Unexpected value\n");
    CHECK(test.xyz[2] == 0.0f, "Unexpected value\n");

    pp_vec3_set(&test, 1, 1, 1);
    CHECK(test.xyz[0] == 1.0f, "Unexpected value\n");
    CHECK(test.xyz[1] == 1.0f, "Unexpected value\n");
    CHECK(test.xyz[2] == 1.0f, "Unexpected value\n");

    PPVec3 ret;
    pp_vec3_add(&test, &test, &ret);
    CHECK(ret.xyz[0] == 2.0f, "Unexpected value\n");
    CHECK(ret.xyz[1] == 2.0f, "Unexpected value\n");
    CHECK(ret.xyz[2] == 2.0f, "Unexpected value\n");

    pp_vec3_sub(&ret, &test, &ret);
    CHECK(ret.xyz[0] == 1.0f, "Unexpected value\n");
    CHECK(ret.xyz[1] == 1.0f, "Unexpected value\n");
    CHECK(ret.xyz[2] == 1.0f, "Unexpected value\n");

    pp_vec3_scale(&ret, 2.0f, &ret);
    CHECK(ret.xyz[0] == 2.0f, "Unexpected value\n");
    CHECK(ret.xyz[1] == 2.0f, "Unexpected value\n");
    CHECK(ret.xyz[2] == 2.0f, "Unexpected value\n");

    CHECK(pp_vec3_normalize(&ret), "Couldn't normalize\n");
    float l = pp_vec3_length(&ret);
    CHECK(CLOSE(l, 1.0f), "Unexpected value\n");

    PPVec3 v1, v2;
    pp_vec3_set(&v1, 1.0f, 0.0f, 0.0f);
    pp_vec3_set(&v2, 0.0f, 0.0f, 1.0f);

    pp_vec3_cross(&v1, &v2, &ret);

    CHECK(ret.xyz[0] == 0.0f, "Unexpected value\n");
    CHECK(ret.xyz[1] == -1.0f, "Unexpected value\n");
    CHECK(ret.xyz[2] == 0.0f, "Unexpected value\n");

    float d = pp_vec3_dot(&v1, &v2);
    CHECK(d == 0.0f, "Unexpected value\n");

    // Quaternion tests

    PPQuaternion q;
    pp_quat_init(&q);

    CHECK(q.xyzw[0] == 0.0f, "Unexpected value\n");
    CHECK(q.xyzw[1] == 0.0f, "Unexpected value\n");
    CHECK(q.xyzw[2] == 0.0f, "Unexpected value\n");
    CHECK(q.xyzw[3] == 1.0f, "Unexpected value\n");

    // Tri intersection

    PPTriangle tri;
    PPVec3 origin, dir;
    pp_vec3_set(&origin, 0, 1, 0);
    pp_vec3_set(&dir, 0, -1, 0);

    pp_vec3_set(&tri.v[0], -1, 0, -1);
    pp_vec3_set(&tri.v[1], 0, 0, 1);
    pp_vec3_set(&tri.v[2], 1, 0, -1);

    CHECK(pp_tri_intersect(&tri, &origin, &dir, &ret, NULL), "Didn't intersect");

    CHECK(ret.xyz[0] == 0.0f, "Unexpected value\n");
    CHECK(ret.xyz[1] == 0.0f, "Unexpected value\n");
    CHECK(ret.xyz[2] == 0.0f, "Unexpected value\n");

    pp_vec3_set(&dir, 0, 1, 0);

    CHECK(pp_tri_intersect(&tri, &origin, &dir, &ret, NULL) == false, "Unexpectedly intersected");

    PPVec3 p1, p2;
    pp_vec3_set(&p1, -0.5f, 0.0f, 0.0f);
    pp_vec3_set(&p2, 0.5f, 0.0f, 0.0f);

    PPSphere* lhs = pp_physics_create_sphere(1.0f, &p1, 1, 0);
    PPSphere* rhs = pp_physics_create_sphere(1.0f, &p2, 1, 0);

    pp_physics_step(1.0f / 60.0f);

    CHECK(lhs->body.pos.xyz[0] == -1.0f, "Unexpected position\n");
    CHECK(rhs->body.pos.xyz[0] == 1.0f, "Unexpected position\n");

    pp_physics_destroy_body(PP_BODY(rhs));
    CHECK(rhs->body.is_alive == false, "Sphere unexpectedly alive\n");

    PPVec3 v3;

    pp_vec3_set(&v1, -10, 0, -10);
    pp_vec3_set(&v2, 0, 0, 10);
    pp_vec3_set(&v3, 10, 0, -10);

    pp_vec3_set(&lhs->body.pos, 0.0f, 0.5f, 0.0f);

    pp_physics_create_triangle(&v1, &v2, &v3, 0);

    pp_physics_step(1.0f / 60.0f);

    CHECK(lhs->body.pos.xyz[1] == 1.0f, "Body position didn't move");

    PPTriangle* ct;
    PPSphere* cs;
    PPVec3 ro = (PPVec3){.xyz = {0, 5, 0}};
    PPVec3 rd = (PPVec3){.xyz = {0, -1, 0}};
    float dist;
    CHECK(pp_physics_ray_intersect(&ro, &rd, &cs, &ct, &dist), "Ray didn't intersect");
    CHECK(CLOSE(dist, 5.0f), "Unexpected intersection distance");

    rhs = pp_physics_create_sphere(1.0f, &p2, 1, 0);

    CHECK(pp_physics_body_count() == 2, "Incorrect sphere count");
    CHECK(pp_physics_body_total_count() == 2, "Incorrect sphere count");
    pp_physics_destroy_body(PP_BODY(lhs));
    CHECK(pp_physics_body_count() == 1, "Incorrect sphere count");
    CHECK(pp_physics_body_total_count() == 2, "Incorrect sphere count");
    lhs = pp_physics_create_sphere(1.0f, &p1, 1, 0);
    CHECK(pp_physics_body_count() == 2, "Incorrect sphere count");
    CHECK(pp_physics_body_total_count() == 2, "Incorrect sphere count");

    // Reset
    pp_physics_clear();

    PPBox* b1 = pp_physics_create_box(1.0f, 1.0f, 1.0f, &p1, 1.0f, 0);
    PPSphere* s1 = pp_physics_create_sphere(1.0f, &p2, 1, 0);

    pp_physics_step(1.0f / 60.0f);

    fprintf(stderr, "Position of b1: %f, %f, %f\n", b1->body.pos.xyz[0], b1->body.pos.xyz[1], b1->body.pos.xyz[2]);
    fprintf(stderr, "Position of s1: %f, %f, %f\n", s1->body.pos.xyz[0], s1->body.pos.xyz[1], s1->body.pos.xyz[2]);

    CHECK(b1->body.pos.xyz[0] == -0.75f, "Unexpected position\n");  // Boxes don't move
    CHECK(s1->body.pos.xyz[0] == 0.75f, "Unexpected position\n");

    pp_body_set_position(PP_BODY(b1), p2.x, p2.y, p2.z);
    pp_body_set_position(PP_BODY(s1), p1.x, p1.y, p1.z);

    pp_physics_step(1.0f / 60.0f);

    CHECK(b1->body.pos.xyz[0] == 0.75f, "Unexpected position\n");  // Boxes don't move
    CHECK(s1->body.pos.xyz[0] == -0.75f, "Unexpected position\n");

    return 0;
}
