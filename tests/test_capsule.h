#pragma once

/*
 * Tests for the capsule shape: a sphere swept along the body's local Y axis.
 * Covers its mass properties, resting on triangles (upright and lying down),
 * contact with every other shape, ray hits, and the reason to have it at all:
 * a character-sized capsule rides up a low kerb that stops a box dead.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class CapsuleTests : public test::TestCase {
public:
    static constexpr float DT = 1.0f / 60.0f;
    static constexpr float S = 0.70710678f; // sin/cos of 45 degrees

    void set_up() {
        pp_physics_clear();
        pp_physics_set_sleeping_enabled(true);
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);
    }

    static PPVec3 V(float x, float y, float z) {
        PPVec3 v = {.xyz = {x, y, z}};
        return v;
    }

    // Two triangles, wound so the normal follows a -> b -> c -> d.
    void quad(PPVec3 a, PPVec3 b, PPVec3 c, PPVec3 d) {
        pp_physics_create_triangle(&a, &b, &c, 0);
        pp_physics_create_triangle(&a, &c, &d, 0);
    }

    void make_floor() {
        quad(V(-100, 0, -100), V(-100, 0, 100), V(100, 0, 100), V(100, 0, -100));
    }

    // Low floor for x < 5, a floor raised by `h` beyond it, and the riser
    // between them facing back towards -x.
    void make_kerb(float h) {
        quad(V(-20, 0, -5), V(-20, 0, 5), V(5, 0, 5), V(5, 0, -5));
        quad(V(5, h, -5), V(5, h, 5), V(40, h, 5), V(40, h, -5));
        quad(V(5, 0, -5), V(5, 0, 5), V(5, h, 5), V(5, h, -5));
    }

    PPBody* make_capsule(float x, float y, float z, float mass) {
        PPVec3 p = V(x, y, z);
        PPBody* c = pp_physics_create_capsule(0.4f, 1.8f, &p, mass, 0);
        pp_body_set_bounce(c, 0.0f);
        return c;
    }

    // Rotate a capsule to lie along the X axis.
    static void lay_along_x(PPBody* c) {
        pp_body_set_rotation(c, 0.0f, 0.0f, S, S);
    }

    static void keep_upright(PPBody* c) {
        pp_body_lock_axis(c, (PPAxisLock)(PP_AXIS_LOCK_PITCH | PP_AXIS_LOCK_ROLL));
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

    // `height` is the overall height: radius 0.4 and height 1.8 leave an inner
    // segment of half-length 0.5, and a bounding radius of half the height.
    void test_dimensions() {
        PPBody* c = make_capsule(0, 0, 0, 1.0f);
        const PPShape* s = pp_body_get_shape(c, 0);

        assert_equal((int)PP_OBJECT_TYPE_CAPSULE, (int)s->type);
        assert_close(0.4f, s->capsule.radius, 1e-6f);
        assert_close(0.5f, s->capsule.half_height, 1e-6f);
        assert_close(0.9f, pp_body_get_radius(c), 1e-6f);
        assert_close(1.0f, pp_body_get_mass(c), 1e-6f);
    }

    // No taller than its own diameter, it is a sphere -- inertia included.
    void test_short_capsule_is_a_sphere() {
        PPVec3 p = V(0, 0, 0);
        PPBody* c = pp_physics_create_capsule(0.5f, 0.5f, &p, 2.0f, 0);
        const PPShape* s = pp_body_get_shape(c, 0);

        assert_close(0.0f, s->capsule.half_height, 1e-6f);
        float sphere_i = 0.4f * 2.0f * 0.25f;
        assert_close(sphere_i, c->inertia.m[0], 1e-5f);
        assert_close(sphere_i, c->inertia.m[4], 1e-5f);
        assert_close(sphere_i, c->inertia.m[8], 1e-5f);
    }

    // Long and thin: easy to spin about its own axis, hard to tumble end over
    // end, and the same about both sideways axes.
    void test_inertia_shape() {
        PPBody* c = make_capsule(0, 0, 0, 10.0f);
        float ixx = c->inertia.m[0], iyy = c->inertia.m[4], izz = c->inertia.m[8];

        assert_true(iyy > 0.0f);
        assert_true(ixx > 2.0f * iyy);
        assert_close(ixx, izz, 1e-5f);

        // Bracketed by a cylinder of the inner length and one of the full height.
        float r2 = 0.16f;
        float lo = 10.0f * (1.0f * 1.0f / 12.0f + r2 / 4.0f);
        float hi = 10.0f * (1.8f * 1.8f / 12.0f + r2 / 4.0f);
        assert_true(ixx > lo);
        assert_true(ixx < hi);
    }

    void test_upright_capsule_rests_on_floor() {
        make_floor();
        PPBody* c = make_capsule(0, 3, 0, 70.0f);
        keep_upright(c);
        run(240);

        PPVec3 p;
        pp_body_get_position(c, &p);
        assert_close(0.9f, p.y, 0.02f); // half its height
        assert_true(speed(c) < 0.01f);
        assert_true(std::fabs(p.x) < 0.01f);
    }

    // Lying down it needs a contact towards each end, or it would see-saw.
    void test_lying_capsule_rests_level() {
        make_floor();
        PPBody* c = make_capsule(0, 2, 0, 70.0f);
        lay_along_x(c);
        run(300);

        PPVec3 p, up;
        pp_body_get_position(c, &p);
        pp_body_get_up(c, &up);
        assert_close(0.4f, p.y, 0.02f);        // its radius
        assert_true(std::fabs(up.y) < 0.02f);  // axis still horizontal
        assert_true(speed(c) < 0.01f);
        assert_true(pp_body_is_asleep(c));
    }

    // Free to rotate and knocked, it falls over and ends up lying down.
    void test_unlocked_capsule_topples() {
        make_floor();
        PPBody* c = make_capsule(0, 0.9f, 0, 70.0f);
        run(30);
        pp_body_set_angular_velocity(c, 0.0f, 0.0f, -1.5f);
        run(600);

        PPVec3 p, up;
        pp_body_get_position(c, &p);
        pp_body_get_up(c, &up);
        assert_close(0.4f, p.y, 0.03f);
        assert_true(std::fabs(up.y) < 0.05f);
    }

    // Dropped fast, it must not pass through the floor.
    void test_capsule_does_not_tunnel() {
        make_floor();
        PPBody* c = make_capsule(0, 30, 0, 70.0f);
        keep_upright(c);
        pp_body_set_velocity(c, 0.0f, -300.0f, 0.0f);
        run(120);

        PPVec3 p;
        pp_body_get_position(c, &p);
        assert_close(0.9f, p.y, 0.05f);
    }

    // The reason to use one for a character: it rides up a kerb lower than its
    // radius, where a box of the same size stops dead.
    void test_capsule_steps_over_low_kerb_box_does_not() {
        for (int use_capsule = 0; use_capsule < 2; ++use_capsule) {
            pp_physics_clear();
            make_kerb(0.15f);

            PPVec3 p = V(0, 1.0f, 0);
            PPBody* c = use_capsule ? pp_physics_create_capsule(0.4f, 1.8f, &p, 70.0f, 0)
                                    : pp_physics_create_box(0.8f, 1.8f, 0.8f, &p, 70.0f, 0);
            pp_body_set_bounce(c, 0.0f);
            pp_body_set_friction(c, 0.0f);
            pp_body_lock_axis(c, (PPAxisLock)(PP_AXIS_LOCK_PITCH | PP_AXIS_LOCK_ROLL | PP_AXIS_LOCK_YAW));
            run(60);

            for (int i = 0; i < 300; ++i) { // walk at 3 units/s for 5 seconds
                PPVec3 v;
                pp_body_get_velocity(c, &v);
                pp_body_set_velocity(c, 3.0f, v.y, 0.0f);
                pp_physics_step(DT, 8, 4);
            }

            PPVec3 q;
            pp_body_get_position(c, &q);
            if (use_capsule) {
                assert_true(q.x > 10.0f);        // up and over
                assert_close(1.05f, q.y, 0.05f); // standing on the raised floor
            } else {
                assert_true(q.x < 5.0f);         // stuck against the riser
            }
        }
    }

    // A wall taller than its radius stops it like anything else.
    void test_capsule_blocked_by_wall() {
        make_kerb(1.0f);
        PPBody* c = make_capsule(0, 1.0f, 0, 70.0f);
        pp_body_set_friction(c, 0.0f);
        keep_upright(c);
        run(60);

        for (int i = 0; i < 300; ++i) {
            PPVec3 v;
            pp_body_get_velocity(c, &v);
            pp_body_set_velocity(c, 3.0f, v.y, 0.0f);
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 q;
        pp_body_get_position(c, &q);
        assert_close(4.6f, q.x, 0.05f); // wall at 5, radius 0.4
        assert_close(0.9f, q.y, 0.05f);
    }

    void test_capsule_rests_on_box() {
        PPVec3 bp = V(0, 1, 0);
        pp_physics_create_box(2.0f, 2.0f, 2.0f, &bp, 0.0f, 0); // static, top at 2
        PPBody* c = make_capsule(0, 4, 0, 70.0f);
        keep_upright(c);
        run(240);

        PPVec3 p;
        pp_body_get_position(c, &p);
        assert_close(2.9f, p.y, 0.02f);
    }

    // Longer than the box it lies across: it must balance on the box's top
    // face, both ends in the air, not tip off one side.
    void test_long_capsule_balances_across_box() {
        PPVec3 bp = V(0, 0.5f, 0);
        pp_physics_create_box(1.0f, 1.0f, 1.0f, &bp, 0.0f, 0); // top at 1

        PPVec3 p = V(0, 2, 0);
        PPBody* c = pp_physics_create_capsule(0.3f, 3.0f, &p, 10.0f, 0);
        pp_body_set_bounce(c, 0.0f);
        lay_along_x(c);
        run(300);

        PPVec3 q, up;
        pp_body_get_position(c, &q);
        pp_body_get_up(c, &up);
        assert_close(1.3f, q.y, 0.02f);
        assert_true(std::fabs(up.y) < 0.03f);
        assert_true(std::fabs(q.x) < 0.05f);
    }

    // A box dropped on a lying capsule lands on top of it (capsule as obj1 of
    // a pair where it was the second body created, too).
    void test_box_rests_on_lying_capsule() {
        make_floor();
        PPVec3 bp = V(0, 3, 0);
        PPBody* box = pp_physics_create_box(1.0f, 0.4f, 1.0f, &bp, 1.0f, 0);
        pp_body_set_bounce(box, 0.0f);
        pp_body_lock_axis(box, (PPAxisLock)(PP_AXIS_LOCK_PITCH | PP_AXIS_LOCK_ROLL | PP_AXIS_LOCK_YAW));

        PPBody* c = make_capsule(0, 0.4f, 0, 0.0f); // static, top surface at 0.8
        lay_along_x(c);
        run(240);

        PPVec3 q;
        pp_body_get_position(box, &q);
        assert_close(1.0f, q.y, 0.03f);
    }

    void test_capsules_stack_crossed() {
        make_floor();
        PPBody* lower = make_capsule(0, 0.4f, 0, 10.0f);
        lay_along_x(lower);
        PPBody* upper = make_capsule(0, 1.3f, 0, 10.0f);
        pp_body_set_rotation(upper, 0.5f, 0.5f, 0.5f, 0.5f); // lying along Z
        run(300);

        PPVec3 q;
        pp_body_get_position(upper, &q);
        assert_close(1.2f, q.y, 0.03f); // two radii on top of 0.4
        assert_true(speed(upper) < 0.01f);
    }

    // Side by side and parallel, two capsules pressed together must not
    // interpenetrate or twist round each other.
    void test_parallel_capsules_push_apart_squarely() {
        PPVec3 zero = V(0, 0, 0);
        pp_physics_set_gravity(&zero);

        PPBody* a = make_capsule(0, 0, 0, 0.0f); // static
        PPBody* b = make_capsule(2.0f, 0, 0, 5.0f);
        pp_body_set_velocity(b, -2.0f, 0.0f, 0.0f);
        run(120);

        PPVec3 q, up;
        pp_body_get_position(b, &q);
        pp_body_get_up(b, &up);
        assert_true(q.x > 0.78f);       // two radii apart, no overlap
        assert_true(up.y > 0.999f);     // still parallel: the push was square-on
        (void)a;
    }

    void test_sphere_bounces_off_capsule() {
        PPVec3 zero = V(0, 0, 0);
        pp_physics_set_gravity(&zero);

        make_capsule(0, 0, 0, 0.0f); // static, upright
        PPVec3 sp = V(-5, 0.3f, 0);
        PPBody* ball = pp_physics_create_sphere(0.3f, &sp, 1.0f, 0);
        pp_body_set_bounce(ball, 1.0f);
        pp_body_set_damping(ball, 0.0f);
        pp_body_set_velocity(ball, 8.0f, 0.0f, 0.0f);
        run(90);

        PPVec3 q, v;
        pp_body_get_position(ball, &q);
        pp_body_get_velocity(ball, &v);
        assert_true(q.x < -0.7f);  // never got past the capsule's side
        assert_true(v.x < -1.0f);  // and is heading back
    }

    // A sphere landing on the rounded top of an upright capsule.
    void test_sphere_hits_capsule_cap() {
        PPVec3 zero = V(0, 0, 0);
        pp_physics_set_gravity(&zero);

        make_capsule(0, 0, 0, 0.0f); // static: top of the cap at y = 0.9
        PPVec3 sp = V(0, 4, 0);
        PPBody* ball = pp_physics_create_sphere(0.3f, &sp, 1.0f, 0);
        pp_body_set_bounce(ball, 0.0f);
        pp_body_set_velocity(ball, 0.0f, -3.0f, 0.0f);
        run(120);

        PPVec3 q;
        pp_body_get_position(ball, &q);
        assert_close(1.2f, q.y, 0.03f); // 0.9 + its radius
    }

    void test_ray_hits_capsule() {
        PPBody* c = make_capsule(0, 0, 0, 0.0f);
        const PPBody* hit_body = NULL;
        const PPTriangle* hit_tri = NULL;
        float dist = 0.0f;
        PPVec3 at;

        // Into the side.
        PPVec3 o = V(-5, 0.2f, 0), d = V(1, 0, 0);
        assert_true(pp_physics_ray_intersect(&o, &d, NULL, &hit_body, &hit_tri, &dist, &at));
        assert_true(hit_body == c);
        assert_close(4.6f, dist, 1e-3f);

        // Down onto the top cap.
        o = V(0, 5, 0); d = V(0, -1, 0);
        assert_true(pp_physics_ray_intersect(&o, &d, NULL, &hit_body, &hit_tri, &dist, &at));
        assert_close(4.1f, dist, 1e-3f);

        // Past the rounded shoulder: inside the bounding box, outside the shape.
        o = V(-5, 0.85f, 0.35f); d = V(1, 0, 0);
        assert_false(pp_physics_ray_intersect(&o, &d, NULL, &hit_body, &hit_tri, &dist, &at));

        // Clean miss.
        o = V(-5, 3, 0); d = V(1, 0, 0);
        assert_false(pp_physics_ray_intersect(&o, &d, NULL, &hit_body, &hit_tri, &dist, &at));
    }

    // Capsules work as one shape of a compound body.
    void test_capsule_in_compound_body() {
        make_floor();
        PPVec3 p = V(0, 3, 0);
        PPBody* b = pp_physics_create_body(&p);
        pp_body_add_capsule(b, 0.3f, 1.2f, 1.0f, 0, -1.0f, 0.0f, 0.0f);
        pp_body_add_capsule(b, 0.3f, 1.2f, 1.0f, 0,  1.0f, 0.0f, 0.0f);
        pp_body_set_bounce(b, 0.0f);
        run(300);

        PPVec3 q, up;
        pp_body_get_position(b, &q);
        pp_body_get_up(b, &up);
        assert_close(0.6f, q.y, 0.03f);   // two legs, standing
        assert_true(up.y > 0.99f);
        assert_true(speed(b) < 0.01f);
    }
};
