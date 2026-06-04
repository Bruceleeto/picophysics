#pragma once

/*
 * Tests for pp_physics_ray_intersect against spheres, triangles and boxes,
 * including nearest-hit selection and the ignore_kinds filter.
 *
 * Note: ignore_kinds is a 0-terminated array, so kind 0 can never be ignored
 * (the terminator and the default kind collide). These tests use non-zero kinds
 * wherever the filter is exercised.
 */

#include "../tools/test.h"

#include "../picophysics.h"

class RayTests : public test::TestCase {
public:
    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    void test_ray_hits_sphere_ahead() {
        PPVec3 pos = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* sphere = pp_physics_create_sphere(1.0f, &pos, 1.0f, 0);

        PPVec3 origin = {.xyz = {0.0f, 0.0f, -10.0f}};
        PPVec3 dir = {.xyz = {0.0f, 0.0f, 1.0f}};

        const PPBody* body_hit = nullptr;
        const PPTriangle* tri_hit = nullptr;
        float dist = 0.0f;
        PPVec3 hit;

        bool ok = pp_physics_ray_intersect(&origin, &dir, nullptr,
                                           &body_hit, &tri_hit, &dist, &hit);
        assert_true(ok);
        assert_equal((const PPBody*) sphere, body_hit);
        assert_is_null(tri_hit);
        // Front of the sphere sits at z = -1, so the ray travels 9 units.
        assert_close(9.0f, dist, 1e-3f);
        assert_close(-1.0f, hit.z, 1e-3f);
    }

    void test_ray_misses_sphere() {
        PPVec3 pos = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_create_sphere(1.0f, &pos, 1.0f, 0);

        // Parallel ray offset well outside the sphere.
        PPVec3 origin = {.xyz = {5.0f, 0.0f, -10.0f}};
        PPVec3 dir = {.xyz = {0.0f, 0.0f, 1.0f}};

        bool ok = pp_physics_ray_intersect(&origin, &dir, nullptr,
                                           nullptr, nullptr, nullptr, nullptr);
        assert_false(ok);
    }

    void test_ray_hits_triangle() {
        // Floor triangle in the XZ plane at y = 0.
        PPVec3 v1 = {.xyz = {-10.0f, 0.0f, -10.0f}};
        PPVec3 v2 = {.xyz = {  0.0f, 0.0f,  10.0f}};
        PPVec3 v3 = {.xyz = { 10.0f, 0.0f, -10.0f}};
        const PPTriangle* tri = pp_physics_create_triangle(&v1, &v2, &v3, 0);

        PPVec3 origin = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPVec3 dir = {.xyz = {0.0f, -1.0f, 0.0f}};

        const PPBody* body_hit = nullptr;
        const PPTriangle* tri_hit = nullptr;
        float dist = 0.0f;

        bool ok = pp_physics_ray_intersect(&origin, &dir, nullptr,
                                           &body_hit, &tri_hit, &dist, nullptr);
        assert_true(ok);
        assert_equal(tri, tri_hit);
        assert_is_null(body_hit);
        assert_close(5.0f, dist, 1e-3f);
    }

    void test_ray_hits_box() {
        PPVec3 pos = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* box = pp_physics_create_box(2.0f, 2.0f, 2.0f, &pos, 1.0f, 0);

        PPVec3 origin = {.xyz = {0.0f, 0.0f, -10.0f}};
        PPVec3 dir = {.xyz = {0.0f, 0.0f, 1.0f}};

        const PPBody* body_hit = nullptr;
        float dist = 0.0f;

        bool ok = pp_physics_ray_intersect(&origin, &dir, nullptr,
                                           &body_hit, nullptr, &dist, nullptr);
        assert_true(ok);
        assert_equal((const PPBody*) box, body_hit);
        // Front face at z = -1.
        assert_close(9.0f, dist, 1e-3f);
    }

    // With two spheres in line, the nearer one must be reported.
    void test_ray_returns_nearest_hit() {
        PPVec3 near_pos = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPVec3 far_pos = {.xyz = {0.0f, 0.0f, 5.0f}};
        PPBody* near_sphere = pp_physics_create_sphere(1.0f, &near_pos, 1.0f, 0);
        pp_physics_create_sphere(1.0f, &far_pos, 1.0f, 0);

        PPVec3 origin = {.xyz = {0.0f, 0.0f, -10.0f}};
        PPVec3 dir = {.xyz = {0.0f, 0.0f, 1.0f}};

        const PPBody* body_hit = nullptr;
        float dist = 0.0f;

        bool ok = pp_physics_ray_intersect(&origin, &dir, nullptr,
                                           &body_hit, nullptr, &dist, nullptr);
        assert_true(ok);
        assert_equal((const PPBody*) near_sphere, body_hit);
        assert_close(9.0f, dist, 1e-3f);
    }

    // Ignoring the only object's kind makes the ray report a miss.
    void test_ignore_kinds_skips_body() {
        const BodyKind kind = 7;
        PPVec3 pos = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_create_sphere(1.0f, &pos, 1.0f, kind);

        PPVec3 origin = {.xyz = {0.0f, 0.0f, -10.0f}};
        PPVec3 dir = {.xyz = {0.0f, 0.0f, 1.0f}};

        BodyKind ignore[] = {kind, 0};
        bool ok = pp_physics_ray_intersect(&origin, &dir, ignore,
                                           nullptr, nullptr, nullptr, nullptr);
        assert_false(ok);

        // A different ignore list leaves the hit intact.
        BodyKind ignore_other[] = {3, 0};
        ok = pp_physics_ray_intersect(&origin, &dir, ignore_other,
                                      nullptr, nullptr, nullptr, nullptr);
        assert_true(ok);
    }
};
