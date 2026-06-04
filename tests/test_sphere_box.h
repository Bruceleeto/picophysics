#pragma once

/*
 * Tests for sphere vs. box collision and response. Unlike sphere/sphere,
 * sphere/box contacts do get positional correction, so a sphere can come to
 * rest on a static box.
 */

#include "../tools/test.h"

#include "../picophysics.h"

class SphereBoxTests : public test::TestCase {
public:
    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    // A static box with its top face at y = 1; a sphere dropped onto it should
    // settle resting on top with its centre at top + radius, and never sink in.
    void test_sphere_rests_on_static_box() {
        PPVec3 box_pos = {.xyz = {0.0f, 0.0f, 0.0f}};
        // 4 x 2 x 4 box -> half height 1, top face at y = 1.
        pp_physics_create_box(4.0f, 2.0f, 4.0f, &box_pos, 0.0f, 0);

        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        const float radius = 0.5f;
        PPVec3 sphere_pos = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* sphere = pp_physics_create_sphere(radius, &sphere_pos, 1.0f, 0);
        pp_body_set_bounce(sphere, 0.0f);

        const float dt = 1.0f / 60.0f;
        float min_y = 1e30f;
        for (int i = 0; i < 300; ++i) {
            pp_physics_step(dt, 8, 4);
            PPVec3 p;
            pp_body_get_position(sphere, &p);
            if (p.y < min_y) min_y = p.y;
        }

        PPVec3 final;
        pp_body_get_position(sphere, &final);

        // Rest height is box-top (1) + radius (0.5) = 1.5.
        assert_close(1.5f, final.y, 0.1f);
        // Never penetrated meaningfully past the rest height.
        assert_true(min_y > 1.5f - 0.1f);
    }

    // The static box must not be shoved around by the sphere landing on it.
    void test_static_box_does_not_move() {
        PPVec3 box_pos = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* box = pp_physics_create_box(4.0f, 2.0f, 4.0f, &box_pos, 0.0f, 0);

        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 sphere_pos = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* sphere = pp_physics_create_sphere(0.5f, &sphere_pos, 1.0f, 0);
        pp_body_set_bounce(sphere, 0.0f);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 200; ++i) {
            pp_physics_step(dt, 8, 4);
        }

        PPVec3 bp;
        pp_body_get_position(box, &bp);
        assert_close(0.0f, bp.x, 1e-4f);
        assert_close(0.0f, bp.y, 1e-4f);
        assert_close(0.0f, bp.z, 1e-4f);
    }

    // A sphere overlapping a static box is pushed back out along the nearest
    // face rather than being left embedded.
    void test_overlapping_sphere_pushed_out() {
        PPVec3 box_pos = {.xyz = {0.0f, 0.0f, 0.0f}};
        // 2 x 2 x 2 box, half extents 1.
        pp_physics_create_box(2.0f, 2.0f, 2.0f, &box_pos, 0.0f, 0);

        const float radius = 1.0f;
        // Centre just above the top face, so the sphere is overlapping.
        PPVec3 sphere_pos = {.xyz = {0.0f, 1.2f, 0.0f}};
        PPBody* sphere = pp_physics_create_sphere(radius, &sphere_pos, 1.0f, 0);
        pp_body_set_bounce(sphere, 0.0f);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 200; ++i) {
            pp_physics_step(dt, 8, 4);
        }

        PPVec3 p;
        pp_body_get_position(sphere, &p);
        // Pushed up to rest on the top face: centre at box-top (1) + radius (1).
        assert_true(p.y > 1.5f);
        assert_close(2.0f, p.y, 0.15f);
    }
};
