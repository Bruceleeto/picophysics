#pragma once

/*
 * Tests for box vs. triangle collision (SAT, one-sided like sphere/triangle).
 * A box rests on a triangle floor/ramp, never tunnels through at speed
 * (speculative contacts), and is ignored when approached from behind the
 * one-sided face. The floor is built from TWO triangles so the box straddles
 * their shared edge -- this guards the contact deduplication that stops the
 * doubled shared-edge contacts from injecting energy.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class BoxTriangleTests : public test::TestCase {
public:
    static constexpr float DT = 1.0f / 60.0f;

    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    // Floor in the XZ plane at y=0 (normal +Y), split into two triangles along
    // the x=z diagonal that runs through the origin.
    void make_floor() {
        PPVec3 a = {.xyz = {-50.0f, 0.0f, -50.0f}};
        PPVec3 b = {.xyz = {-50.0f, 0.0f,  50.0f}};
        PPVec3 c = {.xyz = { 50.0f, 0.0f,  50.0f}};
        PPVec3 d = {.xyz = { 50.0f, 0.0f, -50.0f}};
        pp_physics_create_triangle(&a, &b, &c, 0);
        pp_physics_create_triangle(&a, &c, &d, 0);
    }

    // A box dropped onto the triangle floor rests on top, upright, at the right
    // height -- straddling the two triangles' shared diagonal without the
    // doubled-contact energy injection.
    void test_box_rests_on_triangle_floor() {
        make_floor();
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 dp = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* box = pp_physics_create_box(2.0f, 2.0f, 2.0f, &dp, 1.0f, 0); // half height 1
        pp_body_set_bounce(box, 0.0f);

        for (int i = 0; i < 400; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p, fwd;
        pp_body_get_position(box, &p);
        pp_body_get_forward(box, &fwd);

        assert_close(1.0f, p.y, 0.1f);          // rests one half-height above y=0
        assert_true(std::fabs(p.x) < 0.3f);
        assert_true(fwd.z > 0.95f);             // upright
    }

    // A box driven hard into the floor must never pass through it, at any speed.
    // (Speculative contacts catch the box before it crosses; the contact dedup
    // keeps the multi-triangle solve stable so it doesn't get launched either.)
    void test_box_does_not_tunnel_through_floor() {
        const float radius = 0.5f; // half extent of a 1x1x1 box
        const float speeds[] = {50.0f, 200.0f, 600.0f, 1500.0f};

        for (size_t s = 0; s < sizeof(speeds) / sizeof(speeds[0]); ++s) {
            set_up();
            make_floor();
            PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
            pp_physics_set_gravity(&g);

            PPVec3 dp = {.xyz = {0.0f, 5.0f, 0.0f}};
            PPBody* box = pp_physics_create_box(1.0f, 1.0f, 1.0f, &dp, 1.0f, 0);
            pp_body_set_bounce(box, 0.0f);
            pp_body_set_velocity(box, 0.0f, -speeds[s], 0.0f);

            float min_y = 1e30f;
            for (int i = 0; i < 300; ++i) {
                pp_physics_step(DT, 8, 4);
                PPVec3 q;
                pp_body_get_position(box, &q);
                if (q.y < min_y) min_y = q.y;
            }

            PPVec3 p;
            pp_body_get_position(box, &p);
            // Never crossed to the underside (a tunnel would be hugely negative)...
            assert_true(min_y > -radius);
            // ...and ended resting on top of the floor.
            assert_true(p.y > 0.0f);
        }
    }

    // One-sided: a box behind the triangle (centre below the plane) is ignored,
    // exactly like a sphere approaching from behind. It must not be ejected.
    void test_box_below_plane_is_ignored() {
        make_floor(); // normal +Y
        // No gravity (set_up). Box centre just below the plane.
        PPVec3 dp = {.xyz = {0.0f, -0.3f, 0.0f}};
        PPBody* box = pp_physics_create_box(1.0f, 1.0f, 1.0f, &dp, 1.0f, 0);
        pp_body_set_bounce(box, 0.0f);

        for (int i = 0; i < 60; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p, v;
        pp_body_get_position(box, &p);
        pp_body_get_velocity(box, &v);
        // Untouched: the one-sided face does not push a body that is behind it.
        assert_close(-0.3f, p.y, 1e-3f);
        assert_close(0.0f, v.y, 1e-3f);
    }

    // A box on a tilted triangle (ramp) rests on the surface rather than
    // falling through it. Checked via the box centre's signed distance to the
    // ramp plane, which is robust to the box sliding along the ramp.
    void test_box_rests_on_ramp() {
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        // Ramp rising along +x: y goes 0 -> 5 across x in [-10, 10].
        PPVec3 r1 = {.xyz = {-10.0f, 0.0f, -10.0f}};
        PPVec3 r2 = {.xyz = {-10.0f, 0.0f,  10.0f}};
        PPVec3 r3 = {.xyz = { 10.0f, 5.0f,  10.0f}};
        PPVec3 r4 = {.xyz = { 10.0f, 5.0f, -10.0f}};
        pp_physics_create_triangle(&r1, &r2, &r3, 0);
        pp_physics_create_triangle(&r1, &r3, &r4, 0);

        PPVec3 bp = {.xyz = {0.0f, 6.0f, 0.0f}};
        PPBody* box = pp_physics_create_box(1.0f, 1.0f, 1.0f, &bp, 1.0f, 0);
        pp_body_set_bounce(box, 0.0f);

        for (int i = 0; i < 300; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        // Signed distance of the box centre to the ramp plane.
        const PPTriangle* tri = pp_physics_triangle_at(0);
        PPVec3 p;
        pp_body_get_position(box, &p);
        PPVec3 rel;
        pp_vec3_set(&rel, p.x - tri->v[0].x, p.y - tri->v[0].y, p.z - tri->v[0].z);
        // (the test TU can't call the static-inline dot; do it inline)
        float signed_dist = rel.x * tri->n.x + rel.y * tri->n.y + rel.z * tri->n.z;

        // Resting on the surface: centre sits above the plane by roughly a half
        // extent, and certainly hasn't sunk through it.
        assert_true(signed_dist > 0.2f);
        assert_true(signed_dist < 1.5f);
    }

    // Sanity: a single triangle (no shared edge) also supports a resting box.
    void test_box_on_single_triangle() {
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 a = {.xyz = {-30.0f, 0.0f, -30.0f}};
        PPVec3 b = {.xyz = {  0.0f, 0.0f,  40.0f}};
        PPVec3 c = {.xyz = { 30.0f, 0.0f, -30.0f}};
        pp_physics_create_triangle(&a, &b, &c, 0);

        PPVec3 dp = {.xyz = {0.0f, 4.0f, 0.0f}};
        PPBody* box = pp_physics_create_box(1.0f, 1.0f, 1.0f, &dp, 1.0f, 0);
        pp_body_set_bounce(box, 0.0f);

        for (int i = 0; i < 300; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p;
        pp_body_get_position(box, &p);
        assert_close(0.5f, p.y, 0.1f); // half extent above the floor
    }
};
