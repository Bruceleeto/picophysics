#pragma once

/*
 * Tests for sphere vs. triangle collision detection and response.
 *
 * The headline concern (and the reason this file exists) is *tunneling*: a
 * sphere moving fast enough to cross a triangle within a single physics step
 * can pass straight through it without ever being detected. The sphere/triangle
 * test in pp_physics_step is purely discrete -- it raycasts from the sphere's
 * *current* centre along the triangle normal and only registers a hit when the
 * centre is already within `radius` of the plane. Nothing accounts for the
 * distance the sphere travelled during the step, so a high velocity defeats it.
 *
 * The tunneling tests below assert the *correct* behaviour: a sphere that starts
 * on one side of a triangle must never end up on the far side. Several of them
 * are expected to FAIL against the current engine -- that failure is the bug
 * the author is chasing, captured as an executable, regression-ready spec.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cfloat>

class SphereTriangleTests : public test::TestCase {
public:
    void set_up() {
        // Each test starts from a clean world. pp_physics_clear() wipes bodies
        // and triangles but NOT gravity, so reset that too.
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    // ---- helpers -----------------------------------------------------------

    // A large floor triangle lying in the XZ plane at y = 0, with its normal
    // pointing up (+Y). The origin (0,0,0) projects comfortably inside it, so a
    // sphere dropped anywhere near x=z=0 will land on it.
    PPTriangle* make_floor(BodyKind kind = 0) {
        PPVec3 v1 = {.xyz = {-100.0f, 0.0f, -100.0f}};
        PPVec3 v2 = {.xyz = {   0.0f, 0.0f,  100.0f}};
        PPVec3 v3 = {.xyz = { 100.0f, 0.0f, -100.0f}};
        return pp_physics_create_triangle(&v1, &v2, &v3, kind);
    }

    // A large vertical wall in the XY plane at z = 0, normal pointing toward +Z.
    // A sphere approaching from the +Z side moving in -Z should be stopped.
    PPTriangle* make_wall(BodyKind kind = 0) {
        PPVec3 v1 = {.xyz = {-100.0f, -100.0f, 0.0f}};
        PPVec3 v2 = {.xyz = { 100.0f, -100.0f, 0.0f}};
        PPVec3 v3 = {.xyz = {   0.0f,  100.0f, 0.0f}};
        return pp_physics_create_triangle(&v1, &v2, &v3, kind);
    }

    // Step the world a number of times and return the minimum value of the
    // sphere centre along the given axis (0=x, 1=y, 2=z) seen at any point.
    float min_axis_over_steps(PPBody* sphere, int axis, int steps, float dt) {
        float result = FLT_MAX;
        for (int i = 0; i < steps; ++i) {
            pp_physics_step(dt, 8, 4);
            PPVec3 p;
            pp_body_get_position(sphere, &p);
            if (p.xyz[axis] < result) {
                result = p.xyz[axis];
            }
        }
        return result;
    }

    // ---- baseline coverage (expected to pass) ------------------------------

    // Sanity: creating a triangle registers it and the computed normal points
    // the way the winding implies.
    void test_floor_triangle_has_upward_normal() {
        make_floor();
        assert_equal((size_t) 1, pp_physics_triangle_count());

        const PPTriangle* t = pp_physics_triangle_at(0);
        assert_close(0.0f, t->n.x, 1e-5f);
        assert_close(1.0f, t->n.y, 1e-5f);
        assert_close(0.0f, t->n.z, 1e-5f);
    }

    // A slow sphere falling under gravity must come to rest on the floor with
    // its centre roughly one radius above the plane, and must never dip below
    // it. This is the "happy path" the engine is built to handle.
    void test_slow_sphere_rests_on_floor() {
        make_floor();

        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        const float radius = 1.0f;
        PPVec3 start = {.xyz = {0.0f, 3.0f, 0.0f}};
        PPBody* sphere = pp_physics_create_sphere(radius, &start, 1.0f, 0);
        pp_body_set_bounce(sphere, 0.0f); // settle rather than bounce

        const float dt = 1.0f / 60.0f;
        // Plenty of steps to fall ~2 units and settle.
        float min_y = min_axis_over_steps(sphere, 1, 240, dt);

        PPVec3 final;
        pp_body_get_position(sphere, &final);

        // Never penetrated past the surface by more than the solver's slop, and
        // ended up resting near one radius above the plane.
        assert_true(min_y > -0.05f);
        assert_close(radius, final.y, 0.1f);
    }

    // Control case for the tunneling tests: a sphere moving toward the floor
    // slowly enough that it can never cross in a single step is reliably caught.
    void test_slow_sphere_does_not_pass_through_floor() {
        make_floor();

        const float radius = 1.0f;
        PPVec3 start = {.xyz = {0.0f, 3.0f, 0.0f}};
        PPBody* sphere = pp_physics_create_sphere(radius, &start, 1.0f, 0);
        pp_body_set_bounce(sphere, 0.0f);
        pp_body_set_velocity(sphere, 0.0f, -2.0f, 0.0f);

        const float dt = 1.0f / 60.0f;
        float min_y = min_axis_over_steps(sphere, 1, 200, dt);

        // The sphere's centre must never fully cross to the underside.
        assert_true(min_y > -radius);
    }

    // ---- tunneling tests (the focus) ---------------------------------------
    //
    // These assert the correct, no-tunnel behaviour. Against the current
    // discrete-only collision code they are expected to FAIL; once continuous /
    // swept collision is added they should pass.

    // A sphere fired hard at the floor from above must end up resting on top of
    // it, not several units below it. At this speed the per-step displacement
    // dwarfs the sphere's diameter.
    void test_fast_sphere_does_not_tunnel_through_floor() {
        make_floor();

        const float radius = 1.0f;
        // Start inside the "skip window": high enough to be outside the
        // detection band (centre > radius above the plane) yet close enough
        // that a single ~10-unit step carries the centre to roughly y = -5,
        // well past the band on the far side. A discrete test sees the sphere
        // far above the plane, then far below it, and never in between.
        PPVec3 start = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* sphere = pp_physics_create_sphere(radius, &start, 1.0f, 0);
        pp_body_set_bounce(sphere, 0.0f);

        // ~10 units of travel per step at dt=1/60 -- far more than the diameter.
        pp_body_set_velocity(sphere, 0.0f, -600.0f, 0.0f);

        const float dt = 1.0f / 60.0f;
        float min_y = min_axis_over_steps(sphere, 1, 60, dt);

        PPVec3 final;
        pp_body_get_position(sphere, &final);

        // It must never have fully crossed the plane...
        assert_true(min_y > -radius);
        // ...and should be sitting on top of the floor at the end.
        assert_true(final.y > 0.0f);
    }

    // Same idea against a vertical wall, ruling out an axis-specific quirk.
    void test_fast_sphere_does_not_tunnel_through_wall() {
        make_wall();

        const float radius = 1.0f;
        PPVec3 start = {.xyz = {0.0f, 0.0f, 5.0f}};
        PPBody* sphere = pp_physics_create_sphere(radius, &start, 1.0f, 0);
        pp_body_set_bounce(sphere, 0.0f);
        pp_body_set_velocity(sphere, 0.0f, 0.0f, -600.0f);

        const float dt = 1.0f / 60.0f;
        float min_z = min_axis_over_steps(sphere, 2, 60, dt);

        PPVec3 final;
        pp_body_get_position(sphere, &final);

        assert_true(min_z > -radius);
        assert_true(final.z > 0.0f);
    }

    // A small, fast sphere is the worst case for tunneling: the smaller the
    // radius the larger the speed the discrete test can be defeated by, but the
    // principle is unchanged -- it should still not pass through.
    void test_small_fast_sphere_does_not_tunnel() {
        make_floor();

        const float radius = 0.1f;
        PPVec3 start = {.xyz = {0.0f, 2.5f, 0.0f}};
        PPBody* sphere = pp_physics_create_sphere(radius, &start, 1.0f, 0);
        pp_body_set_bounce(sphere, 0.0f);
        pp_body_set_velocity(sphere, 0.0f, -300.0f, 0.0f);

        const float dt = 1.0f / 60.0f;
        float min_y = min_axis_over_steps(sphere, 1, 60, dt);

        assert_true(min_y > -radius);
    }

    // Sweep a range of impact speeds. None of them should let the sphere escape
    // through the floor. This pins down the speed at which tunneling currently
    // begins (the first speed that trips the assertion when run against the
    // unfixed engine).
    void test_no_tunnel_across_speed_sweep() {
        const float radius = 1.0f;
        const float dt = 1.0f / 60.0f;
        const float speeds[] = {10.0f, 50.0f, 100.0f, 200.0f, 400.0f, 800.0f};

        for (size_t s = 0; s < sizeof(speeds) / sizeof(speeds[0]); ++s) {
            set_up(); // fresh world per speed
            make_floor();

            PPVec3 start = {.xyz = {0.0f, 20.0f, 0.0f}};
            PPBody* sphere = pp_physics_create_sphere(radius, &start, 1.0f, 0);
            pp_body_set_bounce(sphere, 0.0f);
            pp_body_set_velocity(sphere, 0.0f, -speeds[s], 0.0f);

            float min_y = min_axis_over_steps(sphere, 1, 80, dt);
            assert_true(min_y > -radius);
        }
    }
};
