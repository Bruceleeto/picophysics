#pragma once

/*
 * Tests for the integrator: gravity, the per-body gravity multiplier, static
 * bodies and the velocity limiter.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class IntegrationTests : public test::TestCase {
public:
    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    // A free body accelerates under gravity. After one second of simulation at
    // g = -10 its downward speed is ~10 m/s (a hair less, due to the small
    // built-in damping) and it has dropped from where it started.
    void test_free_fall() {
        PPVec3 g = {.xyz = {0.0f, -10.0f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 start = {.xyz = {0.0f, 100.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &start, 1.0f, 0);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 60; ++i) {
            pp_physics_step(dt, 8, 4);
        }

        PPVec3 v, p;
        pp_body_get_velocity(b, &v);
        pp_body_get_position(b, &p);

        assert_close(-10.0f, v.y, 0.5f);
        assert_true(p.y < 100.0f);     // fell
        assert_true(p.y > 90.0f);      // ~5 m drop in 1 s, not absurd
    }

    // A zero-mass body is static: gravity does not move it.
    void test_static_body_ignores_gravity() {
        PPVec3 g = {.xyz = {0.0f, -10.0f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 start = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &start, 0.0f, 0);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 60; ++i) {
            pp_physics_step(dt, 8, 4);
        }

        PPVec3 p, v;
        pp_body_get_position(b, &p);
        pp_body_get_velocity(b, &v);
        assert_close(5.0f, p.y, 1e-5f);
        assert_close(0.0f, v.y, 1e-5f);
    }

    // A gravity multiplier of zero cancels gravity for that body only.
    void test_gravity_multiplier_zero() {
        PPVec3 g = {.xyz = {0.0f, -10.0f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 start = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &start, 1.0f, 0);
        pp_body_set_gravity_multiplier(b, 0.0f);
        assert_close(0.0f, pp_body_get_gravity_multiplier(b), 1e-6f);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 60; ++i) {
            pp_physics_step(dt, 8, 4);
        }

        PPVec3 v, p;
        pp_body_get_velocity(b, &v);
        pp_body_get_position(b, &p);
        assert_close(0.0f, v.y, 1e-4f);
        assert_close(5.0f, p.y, 1e-4f);
    }

    // The velocity limiter caps speed regardless of how hard gravity pulls.
    void test_velocity_limit() {
        PPVec3 g = {.xyz = {0.0f, -100.0f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 start = {.xyz = {0.0f, 100.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &start, 1.0f, 0);
        pp_body_limit_velocity(b, 3.0f);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 60; ++i) {
            pp_physics_step(dt, 8, 4);
        }

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        float speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        assert_true(speed <= 3.0f + 1e-3f);
        // It really is moving at the cap, not just slow.
        assert_close(3.0f, speed, 1e-2f);
    }
};
