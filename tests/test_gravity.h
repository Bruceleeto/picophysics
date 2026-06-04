#pragma once

/*
 * Tests for gravity application: the global gravity vector and the per-body
 * gravity multiplier. (Free-fall, the static-body case, a zero multiplier and
 * the velocity limiter are covered in test_integration.h; these focus on
 * direction, scaling and per-body isolation.)
 */

#include "../tools/test.h"

#include "../picophysics.h"

class GravityTests : public test::TestCase {
public:
    static constexpr float DT = 1.0f / 60.0f;

    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    // Gravity is applied as a full vector, not just along Y. After one step a
    // free body's velocity equals the gravity vector times dt.
    void test_gravity_is_a_vector() {
        PPVec3 g = {.xyz = {1.0f, -2.0f, 3.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_damping(b, 0.0f);

        pp_physics_step(DT, 8, 4);

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close(1.0f * DT, v.x, 1e-5f);
        assert_close(-2.0f * DT, v.y, 1e-5f);
        assert_close(3.0f * DT, v.z, 1e-5f);
    }

    // Gravity is mass-independent: two bodies of different mass gain the same
    // velocity under gravity.
    void test_gravity_is_mass_independent() {
        PPVec3 g = {.xyz = {0.0f, -10.0f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 p1 = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPVec3 p2 = {.xyz = {5.0f, 0.0f, 0.0f}};
        PPBody* light = pp_physics_create_sphere(1.0f, &p1, 1.0f, 0);
        PPBody* heavy = pp_physics_create_sphere(1.0f, &p2, 1000.0f, 0);
        pp_body_set_damping(light, 0.0f);
        pp_body_set_damping(heavy, 0.0f);

        for (int i = 0; i < 30; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 vl, vh;
        pp_body_get_velocity(light, &vl);
        pp_body_get_velocity(heavy, &vh);
        assert_close(vl.y, vh.y, 1e-4f);
    }

    // A multiplier of 2 doubles the effective gravity for that body.
    void test_gravity_multiplier_doubles() {
        PPVec3 g = {.xyz = {0.0f, -10.0f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_damping(b, 0.0f);
        pp_body_set_gravity_multiplier(b, 2.0f);

        pp_physics_step(DT, 8, 4);

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close(-10.0f * 2.0f * DT, v.y, 1e-5f);
    }

    // A negative multiplier reverses gravity for that body (it "falls" upward).
    void test_negative_multiplier_reverses() {
        PPVec3 g = {.xyz = {0.0f, -10.0f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_damping(b, 0.0f);
        pp_body_set_gravity_multiplier(b, -1.0f);

        pp_physics_step(DT, 8, 4);

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close(10.0f * DT, v.y, 1e-5f); // upward
    }

    // The multiplier is per-body: zeroing one body's gravity leaves another in
    // the same world unaffected.
    void test_multiplier_is_per_body() {
        PPVec3 g = {.xyz = {0.0f, -10.0f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 p1 = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPVec3 p2 = {.xyz = {5.0f, 0.0f, 0.0f}};
        PPBody* falling = pp_physics_create_sphere(1.0f, &p1, 1.0f, 0);
        PPBody* floating = pp_physics_create_sphere(1.0f, &p2, 1.0f, 0);
        pp_body_set_damping(falling, 0.0f);
        pp_body_set_gravity_multiplier(floating, 0.0f);

        for (int i = 0; i < 60; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 vf, vfl, pfl;
        pp_body_get_velocity(falling, &vf);
        pp_body_get_velocity(floating, &vfl);
        pp_body_get_position(floating, &pfl);

        assert_true(vf.y < -9.0f);          // falling accelerated
        assert_close(0.0f, vfl.y, 1e-5f);   // floating untouched
        assert_close(0.0f, pfl.y, 1e-5f);
    }
};
