#pragma once

/*
 * Tests for bodies carrying multiple collider shapes, including mixed shape
 * types, exercising collision response across all of a body's shapes.
 *
 * Each shape on a body is tested independently in pp_physics_step: the
 * sphere/triangle pass iterates every sphere shape, and the body-body pass
 * iterates every shape pair and dispatches to the right handler (sphere-sphere,
 * sphere-box, ...). A shape's world position is body.pos + rot * shape.offset,
 * so an off-centre shape's contact carries a lever arm and feeds the body's
 * angular response.
 *
 * These also serve as a regression guard for the solver normal-impulse angular
 * sign fix: before it, any off-centre (lever-arm) contact injected energy and a
 * symmetric two-sphere body would spin up and fly off the floor.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class MultiShapeTests : public test::TestCase {
public:
    static constexpr float DT = 1.0f / 60.0f;

    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    void make_floor() {
        PPVec3 v1 = {.xyz = {-100.0f, 0.0f, -100.0f}};
        PPVec3 v2 = {.xyz = {   0.0f, 0.0f,  100.0f}};
        PPVec3 v3 = {.xyz = { 100.0f, 0.0f, -100.0f}};
        pp_physics_create_triangle(&v1, &v2, &v3, 0);
    }

    static float a_vel_mag(PPBody* b) {
        return std::sqrt(b->a_vel.x*b->a_vel.x + b->a_vel.y*b->a_vel.y + b->a_vel.z*b->a_vel.z);
    }

    // A dumbbell (two spheres offset symmetrically on either side of the body
    // origin) dropped flat onto a triangle floor must come to rest LEVEL: both
    // sphere contacts support it, their lever-arm torques cancel, and it neither
    // tips nor spins. (Before the angular-sign fix this flew off the floor.)
    void test_dumbbell_rests_level_on_floor() {
        make_floor();
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        const float radius = 0.5f;
        PPVec3 c = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* db = pp_physics_create_body(&c);
        pp_body_add_sphere(db, radius, 1.0f, 0, -2.0f, 0.0f, 0.0f);
        pp_body_add_sphere(db, radius, 1.0f, 0,  2.0f, 0.0f, 0.0f);
        pp_body_set_bounce(db, 0.0f);

        for (int i = 0; i < 400; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p, fwd;
        pp_body_get_position(db, &p);
        pp_body_get_forward(db, &fwd);

        // Both spheres rest on the floor -> body centre one radius up.
        assert_close(radius, p.y, 0.05f);
        // Stayed put laterally and upright (no tip, no spin).
        assert_true(std::fabs(p.x) < 0.3f);
        assert_true(fwd.z > 0.99f);
        assert_true(a_vel_mag(db) < 0.2f);
    }

    // A body whose two shapes are stacked vertically rests on the lower one; the
    // upper, non-contacting shape neither holds it up nor interferes.
    void test_noncontacting_shape_is_ignored() {
        make_floor();
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        const float radius = 0.5f;
        PPVec3 c = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* b = pp_physics_create_body(&c);
        pp_body_add_sphere(b, radius, 1.0f, 0, 0.0f, 0.0f, 0.0f); // lands on floor
        pp_body_add_sphere(b, radius, 1.0f, 0, 0.0f, 3.0f, 0.0f); // 3 up, never touches
        pp_body_set_bounce(b, 0.0f);

        for (int i = 0; i < 400; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p;
        pp_body_get_position(b, &p);
        assert_close(radius, p.y, 0.05f);
    }

    // Multi-shape body-body contact: a two-sphere "bar" rests on two separate
    // static spheres, one under each shape. Both shape pairs resolve and the bar
    // stays level.
    void test_multi_shape_body_rests_on_two_bodies() {
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 s1 = {.xyz = {-2.0f, 0.0f, 0.0f}};
        PPVec3 s2 = {.xyz = { 2.0f, 0.0f, 0.0f}};
        pp_physics_create_sphere(1.0f, &s1, 0.0f, 0); // static support, top at y=1
        pp_physics_create_sphere(1.0f, &s2, 0.0f, 0);

        PPVec3 c = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* bar = pp_physics_create_body(&c);
        pp_body_add_sphere(bar, 0.5f, 1.0f, 0, -2.0f, 0.0f, 0.0f);
        pp_body_add_sphere(bar, 0.5f, 1.0f, 0,  2.0f, 0.0f, 0.0f);
        pp_body_set_bounce(bar, 0.0f);

        for (int i = 0; i < 400; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p, fwd;
        pp_body_get_position(bar, &p);
        pp_body_get_forward(bar, &fwd);

        // Support top (1) + bar-sphere radius (0.5) = 1.5.
        assert_close(1.5f, p.y, 0.1f);
        assert_true(std::fabs(p.x) < 0.4f);
        assert_true(fwd.z > 0.99f);
    }

    // Mixed shape types on one (static) body: a moving sphere aimed at the BOX
    // shape is stopped by it via the sphere-box handler.
    void test_moving_sphere_stopped_by_box_shape() {
        PPVec3 cc = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* comp = pp_physics_create_body(&cc);
        pp_body_add_sphere(comp, 1.0f, 0.0f, 0, -3.0f, 0.0f, 0.0f); // sphere shape, off to the left
        pp_body_add_box(comp, 2.0f, 2.0f, 2.0f, 0.0f, 0, 3.0f, 0.0f, 0.0f); // box shape, right face at x=4
        pp_body_set_bounce(comp, 0.0f);

        PPVec3 mp = {.xyz = {8.0f, 0.0f, 0.0f}};
        PPBody* mov = pp_physics_create_sphere(0.5f, &mp, 1.0f, 0);
        pp_body_set_bounce(mov, 0.0f);
        pp_body_set_velocity(mov, -5.0f, 0.0f, 0.0f);

        for (int i = 0; i < 200; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p;
        pp_body_get_position(mov, &p);
        // Box right face at x=4, sphere radius 0.5 -> rests at ~4.5, on the box's
        // side (never tunnels through to x<4).
        assert_true(p.x > 4.0f);
        assert_close(4.5f, p.x, 0.3f);
    }

    // The other shape of the same mixed body: a moving sphere aimed at the
    // SPHERE shape is stopped by it via the sphere-sphere handler. This confirms
    // per-shape dispatch picks the correct handler for each shape.
    void test_moving_sphere_stopped_by_sphere_shape() {
        PPVec3 cc = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* comp = pp_physics_create_body(&cc);
        pp_body_add_sphere(comp, 1.0f, 0.0f, 0, -3.0f, 0.0f, 0.0f); // sphere shape, left face at x=-4
        pp_body_add_box(comp, 2.0f, 2.0f, 2.0f, 0.0f, 0, 3.0f, 0.0f, 0.0f);
        pp_body_set_bounce(comp, 0.0f);

        PPVec3 mp = {.xyz = {-8.0f, 0.0f, 0.0f}};
        PPBody* mov = pp_physics_create_sphere(0.5f, &mp, 1.0f, 0);
        pp_body_set_bounce(mov, 0.0f);
        pp_body_set_velocity(mov, 5.0f, 0.0f, 0.0f);

        for (int i = 0; i < 200; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p;
        pp_body_get_position(mov, &p);
        // Sphere shape centre x=-3, radius 1 -> left surface x=-4; moving sphere
        // (r 0.5) rests at ~-4.5 and never tunnels through to x>-4.
        assert_true(p.x < -4.0f);
        assert_close(-4.5f, p.x, 0.3f);
    }

    // Direct regression guard for the lever-arm energy-injection bug: an
    // off-centre contact must not spin the body up or fling it away. A dumbbell
    // dropped onto the floor stays near the origin with a small final speed,
    // rather than rocketing off (which it did with the angular sign inverted).
    void test_offcentre_contact_does_not_inject_energy() {
        make_floor();
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 c = {.xyz = {0.0f, 3.0f, 0.0f}};
        PPBody* db = pp_physics_create_body(&c);
        pp_body_add_sphere(db, 0.5f, 1.0f, 0, -2.0f, 0.0f, 0.0f);
        pp_body_add_sphere(db, 0.5f, 1.0f, 0,  2.0f, 0.0f, 0.0f);
        pp_body_set_bounce(db, 0.0f);

        for (int i = 0; i < 400; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p, v;
        pp_body_get_position(db, &p);
        pp_body_get_velocity(db, &v);
        float speed = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);

        assert_true(std::fabs(p.x) < 1.0f);  // stayed put, didn't fly sideways
        assert_true(p.y < 2.0f);             // didn't get launched upward
        assert_true(speed < 0.5f);           // settled, no runaway energy
        assert_true(a_vel_mag(db) < 0.5f);   // no runaway spin
    }
};
