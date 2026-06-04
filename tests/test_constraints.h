#pragma once

/*
 * Tests for the fixed-distance constraint.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class ConstraintTests : public test::TestCase {
public:
    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    static float dist_between(PPBody* a, PPBody* b) {
        PPVec3 pa, pb;
        pp_body_get_position(a, &pa);
        pp_body_get_position(b, &pb);
        float dx = pa.x - pb.x, dy = pa.y - pb.y, dz = pa.z - pb.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void test_constraint_count() {
        PPVec3 pa = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPVec3 pb = {.xyz = {10.0f, 0.0f, 0.0f}};
        PPBody* a = pp_physics_create_sphere(1.0f, &pa, 1.0f, 0);
        PPBody* b = pp_physics_create_sphere(1.0f, &pb, 1.0f, 0);

        assert_equal((size_t) 0, pp_body_get_constraint_count(a));
        pp_physics_create_fixed_distance_constraint(a, b, 4.0f);
        assert_equal((size_t) 1, pp_body_get_constraint_count(a));
        assert_equal((size_t) 1, pp_body_get_constraint_count(b));
    }

    // Two bodies further apart than the target distance are pulled together
    // until they sit at the target separation.
    void test_fixed_distance_pulls_together() {
        PPVec3 pa = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPVec3 pb = {.xyz = {10.0f, 0.0f, 0.0f}};
        PPBody* a = pp_physics_create_sphere(1.0f, &pa, 1.0f, 0);
        PPBody* b = pp_physics_create_sphere(1.0f, &pb, 1.0f, 0);
        pp_physics_create_fixed_distance_constraint(a, b, 4.0f);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 400; ++i) {
            pp_physics_step(dt, 8, 8);
        }

        assert_close(4.0f, dist_between(a, b), 0.2f);
    }

    // Two bodies closer than the target are pushed apart to the target.
    void test_fixed_distance_pushes_apart() {
        PPVec3 pa = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPVec3 pb = {.xyz = {1.0f, 0.0f, 0.0f}};
        PPBody* a = pp_physics_create_sphere(1.0f, &pa, 1.0f, 0);
        PPBody* b = pp_physics_create_sphere(1.0f, &pb, 1.0f, 0);
        pp_physics_create_fixed_distance_constraint(a, b, 5.0f);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 400; ++i) {
            pp_physics_step(dt, 8, 8);
        }

        assert_close(5.0f, dist_between(a, b), 0.2f);
    }

    // Anchoring one end to a static body keeps the static body fixed while the
    // dynamic body is reeled in to the target distance.
    void test_fixed_distance_with_static_anchor() {
        PPVec3 anchor_pos = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPVec3 dyn_pos = {.xyz = {10.0f, 0.0f, 0.0f}};
        PPBody* anchor = pp_physics_create_sphere(1.0f, &anchor_pos, 0.0f, 0); // static
        PPBody* dyn = pp_physics_create_sphere(1.0f, &dyn_pos, 1.0f, 0);
        pp_physics_create_fixed_distance_constraint(anchor, dyn, 3.0f);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 400; ++i) {
            pp_physics_step(dt, 8, 8);
        }

        PPVec3 ap;
        pp_body_get_position(anchor, &ap);
        assert_close(0.0f, ap.x, 1e-4f); // anchor never moved
        assert_close(3.0f, dist_between(anchor, dyn), 0.2f);
    }
};
