#pragma once

/*
 * Tests for ball and hinge joints.
 *
 * A ball joint pins a point of one body to a point of another (or to the
 * world) and leaves rotation free. A hinge additionally keeps an axis of each
 * body aligned, so the only freedom left is turning about that axis; it can
 * carry angle limits and a motor.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class JointTests : public test::TestCase {
public:
    static constexpr float DT = 1.0f / 60.0f;

    void set_up() {
        pp_physics_clear();
        pp_physics_set_sleeping_enabled(true);
        set_gravity(-9.8f);
    }

    void set_gravity(float y) {
        PPVec3 g = {.xyz = {0.0f, y, 0.0f}};
        pp_physics_set_gravity(&g);
    }

    static PPVec3 vec(float x, float y, float z) {
        PPVec3 v = {.xyz = {x, y, z}};
        return v;
    }

    static float dist(const PPVec3& a, const PPVec3& b) {
        float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
        return std::sqrt(x * x + y * y + z * z);
    }

    PPBody* make_ball(float x, float y, float z, float mass = 1.0f) {
        PPVec3 p = vec(x, y, z);
        return pp_physics_create_sphere(0.25f, &p, mass, 0);
    }

    // A 2 x 0.2 x 0.2 bar lying along X with its left end at the origin.
    PPBody* make_bar(float mass = 1.0f) {
        PPVec3 p = vec(1.0f, 0.0f, 0.0f);
        return pp_physics_create_box(2.0f, 0.2f, 0.2f, &p, mass, 0);
    }

    void run(int steps) {
        for (int i = 0; i < steps; ++i) {
            pp_physics_step(DT, 8, 4);
        }
    }

    // ---- ball joint --------------------------------------------------------

    // A ball on a 2 m arm pinned to the world swings like a pendulum: it never
    // leaves the sphere of that radius and it passes through to the far side.
    void test_ball_joint_pendulum_keeps_its_length() {
        PPVec3 anchor = vec(0.0f, 5.0f, 0.0f);
        PPBody* bob = make_ball(2.0f, 5.0f, 0.0f);
        assert_is_not_null(pp_physics_create_ball_joint(NULL, bob, &anchor));

        float worst = 0.0f, min_x = 10.0f, min_y = 10.0f;
        for (int i = 0; i < 300; ++i) {
            pp_physics_step(DT, 8, 4);
            PPVec3 p;
            pp_body_get_position(bob, &p);
            worst = std::fmax(worst, std::fabs(dist(p, anchor) - 2.0f));
            min_x = std::fmin(min_x, p.x);
            min_y = std::fmin(min_y, p.y);
        }

        assert_true(worst < 0.01f);
        assert_close(3.0f, min_y, 0.02f);   // bottom of the swing
        assert_true(min_x < -1.8f);         // and up the other side
    }

    // The world can be given as either body.
    void test_world_can_be_either_body() {
        PPVec3 anchor = vec(0.0f, 5.0f, 0.0f);
        PPBody* bob = make_ball(0.0f, 3.0f, 0.0f);
        pp_physics_create_ball_joint(bob, NULL, &anchor);
        run(120);

        PPVec3 p;
        pp_body_get_position(bob, &p);
        assert_close(3.0f, p.y, 0.01f);
        assert_close(0.0f, p.x, 0.01f);
    }

    // Two free bodies pinned together stay pinned when one is kicked, and the
    // joint only passes momentum between them -- it does not create any.
    void test_ball_joint_between_two_bodies_conserves_momentum() {
        set_gravity(0.0f);
        PPBody* a = make_ball(-1.0f, 0.0f, 0.0f, 1.0f);
        PPBody* b = make_ball( 1.0f, 0.0f, 0.0f, 3.0f);
        pp_body_set_damping(a, 0.0f);
        pp_body_set_damping(b, 0.0f);
        PPVec3 anchor = vec(0.0f, 0.0f, 0.0f);
        pp_physics_create_ball_joint(a, b, &anchor);

        pp_body_set_velocity(a, 0.0f, 4.0f, 2.0f);
        run(240);

        PPVec3 va, vb, pa, pb;
        pp_body_get_velocity(a, &va);
        pp_body_get_velocity(b, &vb);
        assert_close(0.0f, va.x * 1.0f + vb.x * 3.0f, 0.02f);
        assert_close(4.0f, va.y * 1.0f + vb.y * 3.0f, 0.02f);
        assert_close(2.0f, va.z * 1.0f + vb.z * 3.0f, 0.02f);

        // Each centre is still 1 m from the shared anchor, so 2 m apart at most
        // and -- the anchor being a point on both -- never closer than that
        // unless they fold, which a kick square to the line cannot do alone.
        pp_body_get_position(a, &pa);
        pp_body_get_position(b, &pb);
        assert_true(dist(pa, pb) < 2.01f);
    }

    // A chain of links hung from the world settles hanging straight down with
    // every link still attached to the next.
    void test_chain_hangs_together() {
        const int N = 5;
        PPBody* links[N];
        PPBody* prev = NULL;
        for (int i = 0; i < N; ++i) {
            PPVec3 p = vec(0.5f + i, 10.0f, 0.0f);
            links[i] = pp_physics_create_box(0.9f, 0.2f, 0.2f, &p, 1.0f, 0);
            pp_body_set_damping(links[i], 0.8f);
            pp_body_set_angular_damping(links[i], 0.8f);
            PPVec3 a = vec((float) i, 10.0f, 0.0f);
            pp_physics_create_ball_joint(prev, links[i], &a);
            prev = links[i];
        }

        run(1500);

        for (int i = 0; i < N; ++i) {
            PPVec3 p;
            pp_body_get_position(links[i], &p);
            assert_close(0.0f, p.x, 0.1f);
            assert_close(0.0f, p.z, 0.1f);
            assert_close(10.0f - 0.5f - i, p.y, 0.05f);
        }
    }

    // The anchor is a point on the body, not its centre of mass: a lopsided
    // compound body pinned at its origin keeps its origin where it was.
    void test_ball_joint_on_lopsided_body_holds_the_anchor_point() {
        PPVec3 c = vec(0.0f, 5.0f, 0.0f);
        PPBody* body = pp_physics_create_body(&c);
        pp_body_add_sphere(body, 0.25f, 1.0f, 0, 1.0f, 0.0f, 0.0f);
        pp_body_add_sphere(body, 0.25f, 3.0f, 0, 3.0f, 0.0f, 0.0f);
        pp_physics_create_ball_joint(NULL, body, &c);

        float worst = 0.0f;
        for (int i = 0; i < 300; ++i) {
            pp_physics_step(DT, 8, 4);
            PPVec3 p;
            pp_body_get_position(body, &p);
            worst = std::fmax(worst, dist(p, c));
        }

        PPVec3 fwd;
        assert_true(worst < 0.01f);
        pp_body_get_forward(body, &fwd);
        assert_close(1.0f, fwd.z, 0.01f);   // swung in the XY plane only
    }

    // ---- hinge --------------------------------------------------------------

    // A hinge about Y throws away whatever spin is not about Y.
    void test_hinge_only_turns_about_its_axis() {
        set_gravity(0.0f);
        PPBody* door = make_bar();
        PPVec3 anchor = vec(0.0f, 0.0f, 0.0f), axis = vec(0.0f, 1.0f, 0.0f);
        PPConstraint* h = pp_physics_create_hinge_joint(NULL, door, &anchor, &axis);
        assert_is_not_null(h);

        pp_body_set_angular_velocity(door, 3.0f, 2.0f, -1.0f);
        run(120);

        PPVec3 w, p;
        w = door->a_vel;
        pp_body_get_position(door, &p);
        assert_close(0.0f, w.x, 0.01f);
        assert_close(0.0f, w.z, 0.01f);
        assert_true(std::fabs(w.y) > 0.2f);

        // Centre stays in the hinge plane, 1 m from the hinge line.
        assert_close(0.0f, p.y, 0.01f);
        assert_close(1.0f, std::sqrt(p.x * p.x + p.z * p.z), 0.01f);
    }

    // The reported angle is the turn about the axis since the joint was made.
    void test_hinge_angle_matches_the_pose() {
        set_gravity(0.0f);
        PPBody* door = make_bar();
        PPVec3 anchor = vec(0.0f, 0.0f, 0.0f), axis = vec(0.0f, 1.0f, 0.0f);
        PPConstraint* h = pp_physics_create_hinge_joint(NULL, door, &anchor, &axis);
        assert_close(0.0f, pp_constraint_get_hinge_angle(h), 0.0001f);

        pp_body_set_angular_velocity(door, 0.0f, 2.0f, 0.0f);
        run(60);

        PPVec3 p;
        pp_body_get_position(door, &p);
        float angle = pp_constraint_get_hinge_angle(h);
        assert_true(angle > 0.2f);
        // Positive turn about +Y swings +X towards -Z.
        assert_close(std::atan2(-p.z, p.x), angle, 0.01f);
    }

    // Limits stop the swing at both ends.
    void test_hinge_limits_stop_the_swing() {
        set_gravity(0.0f);
        PPBody* door = make_bar();
        pp_body_set_angular_damping(door, 0.0f);
        pp_body_set_damping(door, 0.0f);
        PPVec3 anchor = vec(0.0f, 0.0f, 0.0f), axis = vec(0.0f, 1.0f, 0.0f);
        PPConstraint* h = pp_physics_create_hinge_joint(NULL, door, &anchor, &axis);
        pp_constraint_set_hinge_limits(h, -0.5f, 1.0f);

        float lo = 0.0f, hi = 0.0f;
        pp_body_set_angular_velocity(door, 0.0f, 8.0f, 0.0f);
        for (int i = 0; i < 200; ++i) {
            pp_physics_step(DT, 8, 4);
            hi = std::fmax(hi, pp_constraint_get_hinge_angle(h));
        }
        pp_body_set_angular_velocity(door, 0.0f, -8.0f, 0.0f);
        for (int i = 0; i < 200; ++i) {
            pp_physics_step(DT, 8, 4);
            lo = std::fmin(lo, pp_constraint_get_hinge_angle(h));
        }

        assert_true(hi > 0.9f);
        assert_true(hi < 1.05f);
        assert_true(lo < -0.4f);
        assert_true(lo > -0.55f);
    }

    // A bar hinged at one end falls until the lower limit catches it, then
    // rests there.
    void test_hinge_limit_holds_a_load_under_gravity() {
        PPBody* bar = make_bar();
        PPVec3 anchor = vec(0.0f, 0.0f, 0.0f), axis = vec(0.0f, 0.0f, 1.0f);
        PPConstraint* h = pp_physics_create_hinge_joint(NULL, bar, &anchor, &axis);
        pp_constraint_set_hinge_limits(h, -0.6f, 0.6f);
        run(300);

        assert_close(-0.6f, pp_constraint_get_hinge_angle(h), 0.03f);

        PPVec3 w;
        w = bar->a_vel;
        assert_close(0.0f, w.z, 0.05f);

        pp_constraint_disable_hinge_limits(h);
        run(40);
        assert_true(pp_constraint_get_hinge_angle(h) < -0.8f);
    }

    // A strong motor drives the hinge at the asked-for speed, gravity or not.
    void test_hinge_motor_reaches_target_speed() {
        PPBody* bar = make_bar();
        PPVec3 anchor = vec(0.0f, 0.0f, 0.0f), axis = vec(0.0f, 0.0f, 1.0f);
        PPConstraint* h = pp_physics_create_hinge_joint(NULL, bar, &anchor, &axis);
        pp_constraint_set_hinge_motor(h, 2.0f, 200.0f);
        run(90);

        PPVec3 w;
        w = bar->a_vel;
        assert_close(2.0f, w.z, 0.02f);
        assert_close(0.0f, w.x, 0.02f);
        assert_close(0.0f, w.y, 0.02f);
    }

    // The bar's weight is 9.8 N at 1 m: a 2 N.m motor cannot lift it, a
    // 50 N.m one can.
    void test_hinge_motor_is_limited_by_max_torque() {
        PPBody* bar = make_bar();
        pp_body_set_angular_damping(bar, 0.9f);
        PPVec3 anchor = vec(0.0f, 0.0f, 0.0f), axis = vec(0.0f, 0.0f, 1.0f);
        PPConstraint* h = pp_physics_create_hinge_joint(NULL, bar, &anchor, &axis);

        pp_constraint_set_hinge_motor(h, 1.0f, 2.0f);
        run(900);
        PPVec3 p;
        pp_body_get_position(bar, &p);
        assert_true(p.y < -0.9f);           // hanging, a little off plumb
        assert_true(p.x > 0.0f);

        pp_constraint_set_hinge_motor(h, 1.0f, 50.0f);
        float top = -10.0f;
        for (int i = 0; i < 400; ++i) {
            pp_physics_step(DT, 8, 4);
            pp_body_get_position(bar, &p);
            top = std::fmax(top, p.y);
        }
        assert_true(top > 0.9f);            // carried over the top
    }

    // Speed zero is a brake: the bar is held out level against gravity.
    void test_hinge_motor_at_zero_speed_is_a_brake() {
        PPBody* bar = make_bar();
        PPVec3 anchor = vec(0.0f, 0.0f, 0.0f), axis = vec(0.0f, 0.0f, 1.0f);
        PPConstraint* h = pp_physics_create_hinge_joint(NULL, bar, &anchor, &axis);
        pp_constraint_set_hinge_motor(h, 0.0f, 100.0f);
        run(300);
        assert_close(0.0f, pp_constraint_get_hinge_angle(h), 0.02f);

        pp_constraint_set_hinge_motor(h, 0.0f, 0.0f);   // off
        run(30);
        assert_true(pp_constraint_get_hinge_angle(h) < -0.3f);
    }

    // A hinge between two free bodies: the pair can tumble as a whole, but
    // relative to each other they only turn about the shared axis.
    void test_hinge_between_two_dynamic_bodies() {
        set_gravity(0.0f);
        PPVec3 pa = vec(-1.0f, 0.0f, 0.0f), pb = vec(1.0f, 0.0f, 0.0f);
        PPBody* a = pp_physics_create_box(2.0f, 0.2f, 1.0f, &pa, 1.0f, 0);
        PPBody* b = pp_physics_create_box(2.0f, 0.2f, 1.0f, &pb, 1.0f, 0);
        PPVec3 anchor = vec(0.0f, 0.0f, 0.0f), axis = vec(0.0f, 0.0f, 1.0f);
        PPConstraint* h = pp_physics_create_hinge_joint(a, b, &anchor, &axis);

        pp_body_set_angular_velocity(b, 1.0f, 2.0f, 3.0f);
        run(180);

        // Both bodies' Z axes (their "forward") still agree.
        PPVec3 fa, fb;
        pp_body_get_forward(a, &fa);
        pp_body_get_forward(b, &fb);
        assert_true(fa.x * fb.x + fa.y * fb.y + fa.z * fb.z > 0.999f);
        assert_true(std::fabs(pp_constraint_get_hinge_angle(h)) > 0.1f);
    }

    // ---- housekeeping -------------------------------------------------------

    // Jointed bodies overlap freely unless told otherwise. The pin sits off
    // to one side, so two overlapping balls that do collide can scissor apart
    // about it.
    void test_jointed_bodies_do_not_collide() {
        set_gravity(0.0f);
        PPVec3 pa = vec(0.0f, 0.0f, 0.0f), pb = vec(0.3f, 0.0f, 0.0f);
        PPBody* a = pp_physics_create_sphere(0.5f, &pa, 1.0f, 0);
        PPBody* b = pp_physics_create_sphere(0.5f, &pb, 1.0f, 0);
        PPVec3 anchor = vec(0.15f, 1.0f, 0.0f);
        PPConstraint* j = pp_physics_create_ball_joint(a, b, &anchor);
        run(60);

        PPVec3 qa, qb;
        pp_body_get_position(a, &qa);
        pp_body_get_position(b, &qb);
        assert_close(0.3f, dist(qa, qb), 0.001f);

        pp_constraint_set_collide_connected(j, true);
        pp_body_wake(a);
        run(120);

        pp_body_get_position(a, &qa);
        pp_body_get_position(b, &qb);
        assert_true(dist(qa, qb) > 0.95f);
        // Still pinned: the gap between the two bodies' anchor points.
        PPVec3 gap = j->joint.separation, zero = vec(0.0f, 0.0f, 0.0f);
        assert_true(dist(gap, zero) < 0.01f);
    }

    void test_destroying_the_joint_releases_the_body() {
        PPVec3 anchor = vec(0.0f, 5.0f, 0.0f);
        PPBody* bob = make_ball(0.0f, 3.0f, 0.0f);
        PPConstraint* j = pp_physics_create_ball_joint(NULL, bob, &anchor);
        assert_equal((size_t) 1, pp_body_get_constraint_count(bob));
        run(60);

        PPVec3 p;
        pp_body_get_position(bob, &p);
        assert_close(3.0f, p.y, 0.01f);

        pp_physics_destroy_constraint(j);
        assert_equal((size_t) 0, pp_body_get_constraint_count(bob));
        run(60);
        pp_body_get_position(bob, &p);
        assert_true(p.y < 0.0f);
    }

    void test_destroying_a_body_destroys_its_joints() {
        PPBody* a = make_ball(0.0f, 5.0f, 0.0f);
        PPBody* b = make_ball(1.0f, 5.0f, 0.0f);
        PPVec3 anchor = vec(0.5f, 5.0f, 0.0f);
        pp_physics_create_ball_joint(a, b, &anchor);
        pp_physics_destroy_body(a);
        assert_equal((size_t) 0, pp_body_get_constraint_count(b));
        run(10);    // must not touch the dead body

        // The slot is reused.
        PPConstraint* j = pp_physics_create_ball_joint(NULL, b, &anchor);
        assert_is_not_null(j);
        assert_equal((size_t) 1, pp_body_get_constraint_count(b));
    }

    void test_running_out_of_constraints_returns_null() {
        PPBody* a = make_ball(0.0f, 5.0f, 0.0f);
        PPVec3 anchor = vec(0.0f, 5.0f, 0.0f);
        // The limit is a build-time setting the tests cannot see; whatever it
        // is, creation has to start failing cleanly rather than overrun.
        int made = 0;
        while (made < 100000 && pp_physics_create_ball_joint(NULL, a, &anchor)) {
            ++made;
        }
        assert_true(made > 0);
        assert_true(made < 100000);
        assert_equal((size_t) made, pp_body_get_constraint_count(a));
        run(2);
    }

    // ---- sleeping -----------------------------------------------------------

    // A pendulum hanging at rest goes to sleep and stays put.
    void test_resting_jointed_body_sleeps() {
        PPVec3 anchor = vec(0.0f, 5.0f, 0.0f);
        PPBody* bob = make_ball(0.0f, 3.0f, 0.0f);
        pp_physics_create_ball_joint(NULL, bob, &anchor);
        run(120);
        assert_true(pp_body_is_asleep(bob));

        run(60);
        PPVec3 p;
        pp_body_get_position(bob, &p);
        assert_close(3.0f, p.y, 0.01f);
    }

    // Two jointed bodies sleep as one group: kicking either wakes both.
    void test_jointed_bodies_wake_together() {
        set_gravity(0.0f);
        PPBody* a = make_ball(-1.0f, 0.0f, 0.0f);
        PPBody* b = make_ball( 1.0f, 0.0f, 0.0f);
        PPVec3 anchor = vec(0.0f, 0.0f, 0.0f);
        pp_physics_create_ball_joint(a, b, &anchor);
        run(90);
        assert_true(pp_body_is_asleep(a));
        assert_true(pp_body_is_asleep(b));

        pp_body_set_velocity(a, 0.0f, 1.0f, 0.0f);
        assert_false(pp_body_is_asleep(b));
        run(30);

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_true(std::fabs(v.y) > 0.01f);   // dragged along
    }

    // A sleeper hung from a kinematic body wakes when that body moves off, and
    // is towed along.
    void test_kinematic_partner_wakes_and_tows_a_sleeper() {
        PPVec3 top = vec(0.0f, 5.0f, 0.0f);
        PPBody* crane = pp_physics_create_sphere(0.25f, &top, 0.0f, 0);
        PPBody* load = make_ball(0.0f, 3.0f, 0.0f);
        pp_physics_create_ball_joint(crane, load, &top);
        run(120);
        assert_true(pp_body_is_asleep(load));

        PPVec3 target = top;
        for (int i = 0; i < 240; ++i) {
            target.x += 2.0f * DT;
            pp_body_move_kinematic(crane, &target, DT);
            pp_physics_step(DT, 8, 4);
        }

        assert_false(pp_body_is_asleep(load));
        PPVec3 c, l;
        pp_body_get_position(crane, &c);
        pp_body_get_position(load, &l);
        assert_close(8.0f, c.x, 0.05f);
        assert_close(2.0f, dist(c, l), 0.02f);
        assert_true(l.x > 6.0f);
    }
};
