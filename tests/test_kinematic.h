#pragma once

/*
 * Tests for kinematic bodies: zero-mass bodies that have been given a
 * velocity. They move exactly as told -- nothing in the simulation can deflect
 * them -- while pushing dynamic bodies and carrying whatever rests on them.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class KinematicTests : public test::TestCase {
public:
    static constexpr float DT = 1.0f / 60.0f;

    void set_up() {
        pp_physics_clear();
        pp_physics_set_sleeping_enabled(true);
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);
    }

    void make_floor() {
        PPVec3 v1 = {.xyz = {-500.0f, 0.0f, -500.0f}};
        PPVec3 v2 = {.xyz = {   0.0f, 0.0f,  500.0f}};
        PPVec3 v3 = {.xyz = { 500.0f, 0.0f, -500.0f}};
        pp_physics_create_triangle(&v1, &v2, &v3, 0);
    }

    // A 4 x 0.5 x 4 slab with its top face at `top_y`.
    PPBody* make_platform(float x, float top_y) {
        PPVec3 p = {.xyz = {x, top_y - 0.25f, 0.0f}};
        return pp_physics_create_box(4.0f, 0.5f, 4.0f, &p, 0.0f, 0);
    }

    PPBody* make_crate(float x, float y) {
        PPVec3 p = {.xyz = {x, y, 0.0f}};
        PPBody* b = pp_physics_create_box(1.0f, 1.0f, 1.0f, &p, 1.0f, 0);
        pp_body_set_bounce(b, 0.0f);
        return b;
    }

    void run(int steps) {
        for (int i = 0; i < steps; ++i) {
            pp_physics_step(DT, 8, 4);
        }
    }

    // A massless body with no velocity is static, as it always was.
    void test_static_body_stays_put() {
        PPBody* plat = make_platform(0.0f, 5.0f);
        PPBody* crate = make_crate(0.0f, 5.5f);
        run(120);

        PPVec3 p, c;
        pp_body_get_position(plat, &p);
        pp_body_get_position(crate, &c);
        assert_close(0.0f, p.x, 1e-7f);
        assert_close(4.75f, p.y, 1e-7f);
        assert_close(5.5f, c.y, 0.02f); // held up by it
    }

    // It moves at exactly the velocity given: gravity does not pull it and a
    // crate landing on it does not slow it.
    void test_kinematic_moves_at_set_velocity() {
        PPBody* plat = make_platform(0.0f, 5.0f);
        pp_body_set_velocity(plat, 2.0f, 0.0f, 0.0f);
        make_crate(0.0f, 7.0f); // drops onto it part way through

        run(120); // 2 seconds

        PPVec3 p, v;
        pp_body_get_position(plat, &p);
        pp_body_get_velocity(plat, &v);
        assert_close(4.0f, p.x, 1e-3f);
        assert_close(4.75f, p.y, 1e-5f);
        assert_close(2.0f, v.x, 1e-6f);
        assert_close(0.0f, v.y, 1e-6f);
    }

    // The point of a moving platform: what stands on it goes with it.
    void test_platform_carries_rider() {
        PPBody* plat = make_platform(0.0f, 5.0f);
        PPBody* crate = make_crate(0.0f, 5.5f);
        run(30); // let it settle first

        pp_body_set_velocity(plat, 2.0f, 0.0f, 0.0f);
        run(180); // 3 seconds -> platform at x=6

        PPVec3 p, c;
        pp_body_get_position(plat, &p);
        pp_body_get_position(crate, &c);
        assert_close(6.0f, p.x, 1e-2f);
        // Still on board (platform is 4 wide), near where it started on it.
        assert_true(std::fabs(c.x - p.x) < 0.5f);
        assert_close(5.5f, c.y, 0.05f);
    }

    // A lift going up takes its rider up without the rider sinking through.
    void test_lift_raises_rider() {
        PPBody* lift = make_platform(0.0f, 1.0f);
        PPBody* crate = make_crate(0.0f, 1.5f);
        run(30);

        pp_body_set_velocity(lift, 0.0f, 1.5f, 0.0f);
        run(240); // 4 seconds -> top face at 7

        PPVec3 c;
        pp_body_get_position(crate, &c);
        assert_close(7.5f, c.y, 0.05f);

        // Stop the lift: the rider settles on it rather than flying off far.
        pp_body_set_velocity(lift, 0.0f, 0.0f, 0.0f);
        run(120);
        pp_body_get_position(crate, &c);
        assert_close(7.5f, c.y, 0.05f);
    }

    // A lift going down: the rider follows it down (no hovering).
    void test_lift_lowers_rider() {
        PPBody* lift = make_platform(0.0f, 10.0f);
        PPBody* crate = make_crate(0.0f, 10.5f);
        run(30);

        pp_body_set_velocity(lift, 0.0f, -1.0f, 0.0f);
        run(180); // top face at 7

        PPVec3 c;
        pp_body_get_position(crate, &c);
        assert_close(7.5f, c.y, 0.05f);
    }

    // A kinematic pusher shoves a crate along the floor and is not slowed by it.
    void test_kinematic_pushes_dynamic_body() {
        make_floor();
        PPVec3 pp = {.xyz = {-3.0f, 0.5f, 0.0f}};
        PPBody* pusher = pp_physics_create_box(1.0f, 1.0f, 1.0f, &pp, 0.0f, 0);
        PPBody* crate = make_crate(0.0f, 0.5f);
        run(30);

        pp_body_set_velocity(pusher, 1.0f, 0.0f, 0.0f);
        run(300); // 5 seconds -> pusher centre at x=2, its front face at 2.5

        PPVec3 p, c;
        pp_body_get_position(pusher, &p);
        pp_body_get_position(crate, &c);
        assert_close(2.0f, p.x, 1e-2f);
        // The crate is ahead of the pusher's front face, not overlapping it.
        assert_true(c.x > 2.9f);
        assert_true(c.x < 3.5f);
        assert_close(0.5f, c.y, 0.05f);
    }

    // Angular velocity works too: a turntable drags a crate round with it.
    void test_turntable_turns_rider() {
        PPBody* table = make_platform(0.0f, 5.0f);
        PPBody* crate = make_crate(1.0f, 5.5f);
        run(30);

        pp_body_set_angular_velocity(table, 0.0f, 0.5f, 0.0f);
        run(120);

        PPVec3 c;
        pp_body_get_position(crate, &c);
        // Carried round the Y axis: it has left the X axis it started on.
        assert_true(std::fabs(c.z) > 0.3f);
        assert_close(5.5f, c.y, 0.05f);

        PPQuaternion q;
        pp_body_get_rotation(table, &q);
        assert_true(std::fabs(q.y) > 0.2f); // the table itself turned
    }

    // A rider that has gone to sleep on a parked platform wakes and travels
    // when the platform sets off.
    void test_moving_platform_wakes_sleeping_rider() {
        PPBody* plat = make_platform(0.0f, 5.0f);
        PPBody* crate = make_crate(0.0f, 5.5f);
        run(180);
        assert_true(pp_body_is_asleep(crate));

        pp_body_set_velocity(plat, 2.0f, 0.0f, 0.0f);
        run(120);

        PPVec3 p, c;
        pp_body_get_position(plat, &p);
        pp_body_get_position(crate, &c);
        assert_false(pp_body_is_asleep(crate));
        assert_true(std::fabs(c.x - p.x) < 0.5f);
    }

    // pp_body_move_kinematic() reaches its target in one step, and carries a
    // rider when driven every frame.
    void test_move_kinematic_reaches_target_and_carries() {
        PPBody* plat = make_platform(0.0f, 5.0f);
        PPBody* crate = make_crate(0.0f, 5.5f);
        run(30);

        for (int i = 1; i <= 120; ++i) {
            PPVec3 target = {.xyz = {i * 0.02f, 4.75f, 0.0f}}; // 1.2 units/s
            pp_body_move_kinematic(plat, &target, DT);
            pp_physics_step(DT, 8, 4);

            PPVec3 p;
            pp_body_get_position(plat, &p);
            assert_close(target.x, p.x, 1e-4f);
        }

        PPVec3 p, c;
        pp_body_get_position(plat, &p);
        pp_body_get_position(crate, &c);
        assert_true(std::fabs(c.x - p.x) < 0.5f);
    }
};
