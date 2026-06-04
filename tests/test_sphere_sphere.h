#pragma once

/*
 * Tests for sphere vs. sphere collision response.
 *
 * Note: sphere/sphere contacts are resolved by the velocity solver only -- the
 * position solver early-outs for them (their stored "dist" is a signed gap that
 * is negative while overlapping). So these tests drive the spheres together
 * with velocity and assert momentum/separation behaviour rather than positional
 * push-apart of a resting overlap.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class SphereSphereTests : public test::TestCase {
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

    // A moving sphere strikes a stationary one head-on. Momentum must transfer:
    // the struck sphere ends up moving in the direction of impact, and the
    // bodies never swap order (no tunnelling).
    void test_head_on_transfers_momentum() {
        PPVec3 pa = {.xyz = {-1.5f, 0.0f, 0.0f}};
        PPVec3 pb = {.xyz = { 1.5f, 0.0f, 0.0f}};
        PPBody* a = pp_physics_create_sphere(1.0f, &pa, 1.0f, 0);
        PPBody* b = pp_physics_create_sphere(1.0f, &pb, 1.0f, 0);
        pp_body_set_velocity(a, 5.0f, 0.0f, 0.0f);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 120; ++i) {
            pp_physics_step(dt, 8, 4);
        }

        PPVec3 va, posa, posb;
        pp_body_get_velocity(a, &va);
        pp_body_get_position(a, &posa);
        pp_body_get_position(b, &posb);

        PPVec3 vb;
        pp_body_get_velocity(b, &vb);

        // The struck sphere was pushed in +x...
        assert_true(vb.x > 0.1f);
        // ...the striker slowed down (lost momentum to b)...
        assert_true(va.x < 5.0f);
        // ...and a stayed to the left of b.
        assert_true(posa.x < posb.x);
    }

    // Two spheres approaching each other must not pass through one another;
    // they end the simulation no closer than (almost) touching.
    void test_approaching_spheres_do_not_overlap_badly() {
        PPVec3 pa = {.xyz = {-3.0f, 0.0f, 0.0f}};
        PPVec3 pb = {.xyz = { 3.0f, 0.0f, 0.0f}};
        PPBody* a = pp_physics_create_sphere(1.0f, &pa, 1.0f, 0);
        PPBody* b = pp_physics_create_sphere(1.0f, &pb, 1.0f, 0);
        pp_body_set_velocity(a, 4.0f, 0.0f, 0.0f);
        pp_body_set_velocity(b, -4.0f, 0.0f, 0.0f);

        const float dt = 1.0f / 60.0f;
        float min_dist = 1e30f;
        for (int i = 0; i < 200; ++i) {
            pp_physics_step(dt, 8, 4);
            float d = dist_between(a, b);
            if (d < min_dist) min_dist = d;
        }

        // Sum of radii is 2. They are allowed to dip slightly inside (the
        // contact is caught a step late and there is no positional push-out),
        // but must never deeply interpenetrate or pass through.
        assert_true(min_dist > 1.5f);

        PPVec3 posa, posb;
        pp_body_get_position(a, &posa);
        pp_body_get_position(b, &posb);
        assert_true(posa.x < posb.x);
    }

    // A dynamic sphere bouncing off a static (zero-mass) sphere should rebound:
    // its velocity reverses sign along the approach axis.
    void test_bounce_off_static_sphere() {
        PPVec3 dyn_pos = {.xyz = {-3.0f, 0.0f, 0.0f}};
        PPVec3 stat_pos = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* dyn = pp_physics_create_sphere(1.0f, &dyn_pos, 1.0f, 0);
        PPBody* stat = pp_physics_create_sphere(1.0f, &stat_pos, 0.0f, 0);
        pp_body_set_bounce(dyn, 1.0f);
        pp_body_set_bounce(stat, 1.0f);
        pp_body_set_velocity(dyn, 6.0f, 0.0f, 0.0f);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 200; ++i) {
            pp_physics_step(dt, 8, 4);
        }

        PPVec3 v, sp;
        pp_body_get_velocity(dyn, &v);
        pp_body_get_position(stat, &sp);

        // The static sphere never moved...
        assert_close(0.0f, sp.x, 1e-4f);
        // ...and the dynamic sphere bounced back (now travelling -x).
        assert_true(v.x < 0.0f);
    }
};
