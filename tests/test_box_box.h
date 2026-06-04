#pragma once

/*
 * Tests for box vs. box collision (GJK + EPA).
 *
 * IMPORTANT SCOPE NOTE: box/box is only partially reliable. The EPA fixes in
 * this library corrected three real bugs (a crash from a stale closest-face
 * index, a far-face depth overshoot, and an inverted contact normal), so a
 * symmetric resting overlap now separates correctly and nothing aborts. But EPA
 * still produces degenerate normals on flat face-to-face contact and yields only
 * a single, off-centre contact point, so dynamic impacts and stacking remain
 * unstable (they inject energy). Those cases need a SAT-based collider with a
 * clipped multi-point manifold and are intentionally NOT asserted here.
 *
 * These tests pin down (a) the behaviour that is now correct and stable, and
 * (b) that the previously-crashing path no longer aborts or produces NaNs.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class BoxBoxTests : public test::TestCase {
public:
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

    // Two equal boxes overlapping along X push apart to (almost) touching and
    // come to rest in the correct order. This is the case the EPA sign/index
    // fixes made correct -- before, the boxes were pulled *together* and the
    // sim diverged.
    void test_overlapping_boxes_separate() {
        PPVec3 ap = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPVec3 bp = {.xyz = {1.9f, 0.0f, 0.0f}}; // overlap of 0.1 (each box is 2 wide)
        PPBody* a = pp_physics_create_box(2.0f, 2.0f, 2.0f, &ap, 1.0f, 0);
        PPBody* b = pp_physics_create_box(2.0f, 2.0f, 2.0f, &bp, 1.0f, 0);
        pp_body_set_bounce(a, 0.0f);
        pp_body_set_bounce(b, 0.0f);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 300; ++i) {
            pp_physics_step(dt, 8, 4);
        }

        PPVec3 pa, pb, va, vb;
        pp_body_get_position(a, &pa);
        pp_body_get_position(b, &pb);
        pp_body_get_velocity(a, &va);
        pp_body_get_velocity(b, &vb);

        float gap = pb.x - pa.x;
        // Separated to roughly the sum of half-extents (2), still ordered...
        assert_true(gap > 1.8f);
        assert_true(gap < 2.1f);
        assert_true(pa.x < pb.x);
        // ...and settled (not drifting or exploding).
        assert_true(std::sqrt(va.x*va.x + va.y*va.y + va.z*va.z) < 0.5f);
        assert_true(std::sqrt(vb.x*vb.x + vb.y*vb.y + vb.z*vb.z) < 0.5f);
    }

    // Regression guard for the EPA crash: a box overlapping a static box used to
    // trip an assertion in pp_polytope_push_face and abort the process. It must
    // now run to completion with finite state (correctness of the response is a
    // separate, still-open problem -- see the scope note above).
    void test_epa_contact_does_not_crash_or_nan() {
        PPVec3 sp = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* stat = pp_physics_create_box(4.0f, 2.0f, 4.0f, &sp, 0.0f, 0);

        PPVec3 dp = {.xyz = {0.3f, 1.6f, 0.2f}}; // overlapping the static box
        PPBody* dyn = pp_physics_create_box(2.0f, 2.0f, 2.0f, &dp, 1.0f, 0);

        const float dt = 1.0f / 60.0f;
        for (int i = 0; i < 120; ++i) {
            pp_physics_step(dt, 8, 4);
        }

        assert_true(finite_pos(stat));
        assert_true(finite_pos(dyn));
    }
};
