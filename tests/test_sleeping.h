#pragma once

/*
 * Tests for sleeping: bodies that come to rest stop being simulated until
 * something disturbs them, and everything resting together sleeps and wakes as
 * one group. Also covers rolling resistance, which is what lets a sphere come
 * to rest (and so sleep) at all.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class SleepingTests : public test::TestCase {
public:
    static constexpr float DT = 1.0f / 60.0f;

    void set_up() {
        pp_physics_clear();
        pp_physics_set_sleeping_enabled(true);
        pp_physics_set_sleep_thresholds(0.05f, 0.05f, 0.5f);
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);
    }

    void make_floor() {
        PPVec3 v1 = {.xyz = {-500.0f, 0.0f, -500.0f}};
        PPVec3 v2 = {.xyz = {   0.0f, 0.0f,  500.0f}};
        PPVec3 v3 = {.xyz = { 500.0f, 0.0f, -500.0f}};
        pp_physics_create_triangle(&v1, &v2, &v3, 0);
    }

    PPBody* resting_box(float x, float y) {
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

    static float speed(PPBody* b) {
        PPVec3 v;
        pp_body_get_velocity(b, &v);
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    void test_resting_body_falls_asleep() {
        make_floor();
        PPBody* b = resting_box(0.0f, 0.5f);

        assert_false(pp_body_is_asleep(b));
        run(120);
        assert_true(pp_body_is_asleep(b));
        assert_close(0.0f, speed(b), 1e-8f);
    }

    // Asleep means not simulated: gravity must not creep it into the floor.
    void test_sleeping_body_does_not_move() {
        make_floor();
        PPBody* b = resting_box(0.0f, 0.5f);
        run(120);
        assert_true(pp_body_is_asleep(b));

        PPVec3 before, after;
        pp_body_get_position(b, &before);
        run(600);
        pp_body_get_position(b, &after);

        assert_close(before.x, after.x, 1e-7f);
        assert_close(before.y, after.y, 1e-7f);
        assert_close(before.z, after.z, 1e-7f);
    }

    // A body in free fall is slow at the top of its arc but never still for
    // long enough; it must not freeze in mid-air.
    void test_falling_body_stays_awake() {
        PPVec3 p = {.xyz = {0.0f, 100.0f, 0.0f}};
        PPBody* b = pp_physics_create_box(1.0f, 1.0f, 1.0f, &p, 1.0f, 0);
        pp_body_set_velocity(b, 0.0f, 1.0f, 0.0f); // passes through v=0

        for (int i = 0; i < 120; ++i) {
            pp_physics_step(DT, 8, 4);
            assert_false(pp_body_is_asleep(b));
        }

        PPVec3 q;
        pp_body_get_position(b, &q);
        assert_true(q.y < 95.0f);
    }

    // Something moving wakes a sleeper in time for the impact to count: the
    // sleeper must take its share of the momentum, not act like a wall.
    void test_impact_wakes_sleeper_and_transfers_momentum() {
        make_floor();
        PPBody* box = resting_box(0.0f, 0.5f);
        run(120);
        assert_true(pp_body_is_asleep(box));

        PPVec3 sp = {.xyz = {-5.0f, 0.5f, 0.0f}};
        PPBody* ball = pp_physics_create_sphere(0.5f, &sp, 2.0f, 0);
        pp_body_set_bounce(ball, 0.0f);
        pp_body_set_velocity(ball, 10.0f, 0.0f, 0.0f);

        run(60);

        PPVec3 q;
        pp_body_get_position(box, &q);
        assert_false(pp_body_is_asleep(box));
        assert_true(q.x > 0.5f); // shoved along, not left behind
    }

    // A pile sleeps as a group and wakes as a group: disturb the bottom box
    // and the top one must be awake too.
    void test_stack_sleeps_and_wakes_together() {
        make_floor();
        PPBody* bottom = resting_box(0.0f, 0.5f);
        PPBody* middle = resting_box(0.0f, 1.52f);
        PPBody* top = resting_box(0.0f, 2.54f);
        PPBody* elsewhere = resting_box(20.0f, 0.5f);

        run(180);
        assert_true(pp_body_is_asleep(bottom));
        assert_true(pp_body_is_asleep(middle));
        assert_true(pp_body_is_asleep(top));
        assert_true(pp_body_is_asleep(elsewhere));

        pp_body_wake(bottom);
        assert_false(pp_body_is_asleep(bottom));
        assert_false(pp_body_is_asleep(middle));
        assert_false(pp_body_is_asleep(top));
        // A separate pile is left alone.
        assert_true(pp_body_is_asleep(elsewhere));

        // Woken with nothing actually pushing it, the stack stays standing and
        // goes back to sleep.
        run(180);
        PPVec3 q;
        pp_body_get_position(top, &q);
        assert_close(2.5f, q.y, 0.1f);
        assert_true(std::fabs(q.x) < 0.05f);
        assert_true(pp_body_is_asleep(top));
    }

    void test_api_calls_wake_body() {
        make_floor();
        PPBody* b = resting_box(0.0f, 0.5f);

        run(120);
        assert_true(pp_body_is_asleep(b));
        pp_body_add_force(b, 0.0f, 1.0f, 0.0f);
        assert_false(pp_body_is_asleep(b));

        run(120);
        assert_true(pp_body_is_asleep(b));
        pp_body_set_velocity(b, 0.0f, 0.0f, 0.0f);
        assert_false(pp_body_is_asleep(b));

        run(120);
        assert_true(pp_body_is_asleep(b));
        pp_body_set_position(b, 0.0f, 0.5f, 0.0f);
        assert_false(pp_body_is_asleep(b));
    }

    // A woken body really is simulated again: lift it and it falls.
    void test_woken_body_responds() {
        make_floor();
        PPBody* b = resting_box(0.0f, 0.5f);
        run(120);
        assert_true(pp_body_is_asleep(b));

        pp_body_set_position(b, 0.0f, 5.0f, 0.0f);
        run(30);

        PPVec3 q;
        pp_body_get_position(b, &q);
        assert_true(q.y < 4.5f);
    }

    // Destroying what a sleeper rests on must wake it, or it would hang in
    // the air.
    void test_destroying_support_wakes_body() {
        make_floor();
        PPVec3 sp = {.xyz = {0.0f, 1.0f, 0.0f}};
        PPBody* support = pp_physics_create_box(2.0f, 2.0f, 2.0f, &sp, 0.0f, 0); // static
        PPBody* b = resting_box(0.0f, 2.5f);

        run(120);
        assert_true(pp_body_is_asleep(b));

        pp_physics_destroy_body(support);
        assert_false(pp_body_is_asleep(b));

        run(120);
        PPVec3 q;
        pp_body_get_position(b, &q);
        assert_close(0.5f, q.y, 0.05f); // fell to the floor
    }

    void test_gravity_change_wakes_everything() {
        make_floor();
        PPBody* b = resting_box(0.0f, 0.5f);
        run(120);
        assert_true(pp_body_is_asleep(b));

        PPVec3 up = {.xyz = {0.0f, 9.8f, 0.0f}};
        pp_physics_set_gravity(&up);
        assert_false(pp_body_is_asleep(b));

        run(60);
        PPVec3 q;
        pp_body_get_position(b, &q);
        assert_true(q.y > 1.0f);
    }

    void test_sleeping_can_be_disabled() {
        make_floor();
        pp_physics_set_sleeping_enabled(false);
        PPBody* b = resting_box(0.0f, 0.5f);
        run(300);
        assert_false(pp_body_is_asleep(b));

        // Turning it off wakes anything already asleep.
        pp_physics_set_sleeping_enabled(true);
        run(120);
        assert_true(pp_body_is_asleep(b));
        pp_physics_set_sleeping_enabled(false);
        assert_false(pp_body_is_asleep(b));

        pp_physics_set_sleeping_enabled(true);
    }

    void test_never_sleeps_flag() {
        make_floor();
        PPBody* b = resting_box(0.0f, 0.5f);
        pp_body_set_never_sleeps(b, true);
        run(300);
        assert_false(pp_body_is_asleep(b));

        pp_body_set_never_sleeps(b, false);
        run(120);
        assert_true(pp_body_is_asleep(b));
    }

    // An awake body resting NEAR a sleeper, not touching, must not keep waking
    // it (and be woken back in turn): both have to end up asleep.
    void test_quiet_neighbours_both_sleep() {
        make_floor();
        PPBody* a = resting_box(0.0f, 0.5f);
        run(120);
        assert_true(pp_body_is_asleep(a));

        PPBody* b = resting_box(1.2f, 0.5f); // 0.2 gap: bounding spheres overlap
        run(180);

        assert_true(pp_body_is_asleep(a));
        assert_true(pp_body_is_asleep(b));
    }

    void test_rolling_ball_comes_to_rest_and_sleeps() {
        make_floor();
        PPVec3 p = {.xyz = {0.0f, 0.5f, 0.0f}};
        PPBody* ball = pp_physics_create_sphere(0.5f, &p, 1.0f, 0);
        pp_body_set_bounce(ball, 0.0f);
        pp_body_set_velocity(ball, 5.0f, 0.0f, 0.0f);

        run(1200);

        PPVec3 q;
        pp_body_get_position(ball, &q);
        assert_true(pp_body_is_asleep(ball));
        assert_true(q.x > 5.0f);   // it did roll
        assert_true(q.x < 60.0f);  // and it did stop
    }

    // With rolling resistance off, a ball on the flat keeps most of its speed:
    // the resistance above is what stopped it, not something else.
    void test_zero_rolling_resistance_keeps_rolling() {
        make_floor();
        PPVec3 p = {.xyz = {0.0f, 0.5f, 0.0f}};
        PPBody* ball = pp_physics_create_sphere(0.5f, &p, 1.0f, 0);
        pp_body_set_bounce(ball, 0.0f);
        pp_body_set_rolling_resistance(ball, 0.0f);
        pp_body_set_damping(ball, 0.0f);
        pp_body_set_angular_damping(ball, 0.0f);
        pp_body_set_velocity(ball, 5.0f, 0.0f, 0.0f);

        run(600);

        assert_false(pp_body_is_asleep(ball));
        assert_true(speed(ball) > 3.0f); // 5 -> ~3.57 once sliding becomes rolling
    }

    // Rolling resistance must not hold a ball on a real slope.
    void test_ball_still_rolls_down_a_slope() {
        // Plane tilted 15 degrees about Z, downhill towards +x.
        float t = std::tan(15.0f * 3.14159265f / 180.0f), s = 200.0f;
        PPVec3 a = {.xyz = {-s,  s * t,  s}}, b = {.xyz = {s, -s * t,  s}};
        PPVec3 c = {.xyz = { s, -s * t, -s}}, d = {.xyz = {-s,  s * t, -s}};
        pp_physics_create_triangle(&a, &b, &c, 0);
        pp_physics_create_triangle(&a, &c, &d, 0);

        PPVec3 p = {.xyz = {0.0f, 0.55f, 0.0f}};
        PPBody* ball = pp_physics_create_sphere(0.5f, &p, 1.0f, 0);
        pp_body_set_bounce(ball, 0.0f);

        run(300);

        PPVec3 q;
        pp_body_get_position(ball, &q);
        assert_false(pp_body_is_asleep(ball));
        assert_true(q.x > 10.0f);
    }
};
