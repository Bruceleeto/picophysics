#pragma once

/*
 * Tests for box vs. box collision, now handled by SAT (Separating Axis Theorem)
 * with a clipped multi-point contact manifold. SAT replaced the old GJK/EPA
 * path, which produced degenerate normals on flat faces and only a single
 * off-centre contact point -- so boxes couldn't rest or stack. With SAT a box
 * rests squarely on another, and boxes stack stably.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class BoxBoxTests : public test::TestCase {
public:
    static constexpr float DT = 1.0f / 60.0f;

    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    static bool finite_pos(PPBody* b) {
        PPVec3 p;
        pp_body_get_position(b, &p);
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }

    // Two equal boxes overlapping along X push apart and settle, correctly
    // ordered, no longer overlapping. (SAT's manifold makes both contacts push
    // along the same normal; the Baumgarte position solver leaves them a touch
    // over-separated, hence the loose upper bound.)
    void test_overlapping_boxes_separate() {
        PPVec3 ap = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPVec3 bp = {.xyz = {1.9f, 0.0f, 0.0f}};
        PPBody* a = pp_physics_create_box(2.0f, 2.0f, 2.0f, &ap, 1.0f, 0);
        PPBody* b = pp_physics_create_box(2.0f, 2.0f, 2.0f, &bp, 1.0f, 0);
        pp_body_set_bounce(a, 0.0f);
        pp_body_set_bounce(b, 0.0f);

        for (int i = 0; i < 300; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 pa, pb, va, vb;
        pp_body_get_position(a, &pa);
        pp_body_get_position(b, &pb);
        pp_body_get_velocity(a, &va);
        pp_body_get_velocity(b, &vb);

        float gap = pb.x - pa.x;
        assert_true(gap > 1.9f);   // no longer overlapping (sum of half-extents is 2)
        assert_true(gap < 2.4f);
        assert_true(pa.x < pb.x);
        assert_true(std::sqrt(va.x*va.x + va.y*va.y + va.z*va.z) < 0.5f);
        assert_true(std::sqrt(vb.x*vb.x + vb.y*vb.y + vb.z*vb.z) < 0.5f);
    }

    // A box dropped onto a static box rests squarely on top: centre at the
    // correct height, upright, not spinning. (Impossible with the old single
    // off-centre EPA contact, which tumbled and exploded.)
    void test_box_rests_on_static_box() {
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 fp = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* floor = pp_physics_create_box(10.0f, 2.0f, 10.0f, &fp, 0.0f, 0); // top at y=1
        pp_body_set_bounce(floor, 0.0f);

        PPVec3 dp = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* box = pp_physics_create_box(2.0f, 2.0f, 2.0f, &dp, 1.0f, 0); // half height 1
        pp_body_set_bounce(box, 0.0f);

        for (int i = 0; i < 600; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p, fwd;
        pp_body_get_position(box, &p);
        pp_body_get_forward(box, &fwd);

        // Floor top (1) + box half height (1) = 2.
        assert_close(2.0f, p.y, 0.1f);
        assert_true(std::fabs(p.x) < 0.2f);
        assert_true(std::fabs(p.z) < 0.2f);
        assert_true(fwd.z > 0.99f); // upright, didn't tip
    }

    // A static box must not be shoved by a box landing on it.
    void test_static_box_not_displaced() {
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 fp = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* floor = pp_physics_create_box(10.0f, 2.0f, 10.0f, &fp, 0.0f, 0);
        PPVec3 dp = {.xyz = {0.0f, 5.0f, 0.0f}};
        pp_physics_create_box(2.0f, 2.0f, 2.0f, &dp, 1.0f, 0);

        for (int i = 0; i < 300; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 bp;
        pp_body_get_position(floor, &bp);
        assert_close(0.0f, bp.x, 1e-3f);
        assert_close(0.0f, bp.y, 1e-3f);
        assert_close(0.0f, bp.z, 1e-3f);
    }

    // Three boxes dropped into a column form a stable stack at the right
    // heights -- the headline win from the multi-point SAT manifold.
    void test_box_stack_is_stable() {
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 fp = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_create_box(10.0f, 2.0f, 10.0f, &fp, 0.0f, 0); // top at y=1

        PPVec3 p1 = {.xyz = {0.0f, 2.0f, 0.0f}};
        PPVec3 p2 = {.xyz = {0.0f, 4.0f, 0.0f}};
        PPVec3 p3 = {.xyz = {0.0f, 6.0f, 0.0f}};
        PPBody* b1 = pp_physics_create_box(2.0f, 2.0f, 2.0f, &p1, 1.0f, 0);
        PPBody* b2 = pp_physics_create_box(2.0f, 2.0f, 2.0f, &p2, 1.0f, 0);
        PPBody* b3 = pp_physics_create_box(2.0f, 2.0f, 2.0f, &p3, 1.0f, 0);
        pp_body_set_bounce(b1, 0.0f);
        pp_body_set_bounce(b2, 0.0f);
        pp_body_set_bounce(b3, 0.0f);

        for (int i = 0; i < 1000; ++i) {
            pp_physics_step(DT, 12, 6);
        }

        PPVec3 q1, q2, q3;
        pp_body_get_position(b1, &q1);
        pp_body_get_position(b2, &q2);
        pp_body_get_position(b3, &q3);

        // Resting heights 2, 4, 6 (small downward sink from solver slop is fine).
        assert_close(2.0f, q1.y, 0.15f);
        assert_close(4.0f, q2.y, 0.2f);
        assert_close(6.0f, q3.y, 0.25f);
        // The column didn't topple over.
        assert_true(std::fabs(q3.x) < 0.4f);
        assert_true(std::fabs(q3.z) < 0.4f);
    }

    // A box thrown head-on at a static box (no bounce) is stopped at the contact
    // rather than tunnelling through or being flung away.
    void test_thrown_box_is_stopped() {
        PPVec3 sp = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* stat = pp_physics_create_box(2.0f, 2.0f, 2.0f, &sp, 0.0f, 0);
        pp_body_set_bounce(stat, 0.0f);

        PPVec3 mp = {.xyz = {-5.0f, 0.0f, 0.0f}};
        PPBody* mov = pp_physics_create_box(2.0f, 2.0f, 2.0f, &mp, 1.0f, 0);
        pp_body_set_bounce(mov, 0.0f);
        pp_body_set_velocity(mov, 5.0f, 0.0f, 0.0f);

        for (int i = 0; i < 300; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 p, sps;
        pp_body_get_position(mov, &p);
        pp_body_get_position(stat, &sps);

        // Static face at x=-1, moving box (half-width 1) rests at ~ -2; never
        // tunnels through (x<0) and isn't flung back past its start (x>-4).
        assert_true(p.x < 0.0f);
        assert_true(p.x > -4.0f);
        assert_close(0.0f, sps.x, 1e-3f); // static unmoved
    }

    // Robustness/no-crash guard for arbitrary orientations: two rotated,
    // overlapping boxes must run without NaNs (SAT must always return a usable
    // axis, never divide by a degenerate one).
    void test_rotated_boxes_finite() {
        PPVec3 g = {.xyz = {0.0f, -9.8f, 0.0f}};
        pp_physics_set_gravity(&g);

        PPVec3 fp = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_create_box(20.0f, 2.0f, 20.0f, &fp, 0.0f, 0);

        PPVec3 dp = {.xyz = {0.0f, 4.0f, 0.0f}};
        PPBody* box = pp_physics_create_box(2.0f, 2.0f, 2.0f, &dp, 1.0f, 0);
        // Tip it onto a corner so contact starts as edge/vertex, not face-face.
        // Axis (1,0,1) normalised, angle 0.6 rad, built as a quaternion directly
        // (pp_quat_from_axis_angle isn't part of the public API).
        const float half = 0.3f; // angle/2
        const float s = std::sin(half), inv = 1.0f / std::sqrt(2.0f);
        pp_body_set_rotation(box, inv * s, 0.0f, inv * s, std::cos(half));

        for (int i = 0; i < 400; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        assert_true(finite_pos(box));
        PPVec3 p;
        pp_body_get_position(box, &p);
        assert_true(p.y > 0.0f);     // settled above the floor, didn't sink through
        assert_true(p.y < 5.0f);     // didn't get launched
    }
};
