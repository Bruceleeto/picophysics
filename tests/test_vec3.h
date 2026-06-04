#pragma once

/*
 * Tests for the public vector helpers (pp_vec3_set / assign / normalize).
 * Most of the vec3 maths is static-inline and lives inside the implementation
 * block, so only these three are reachable from a test translation unit.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class Vec3Tests : public test::TestCase {
public:
    void test_set_assigns_components() {
        PPVec3 v;
        pp_vec3_set(&v, 1.0f, 2.0f, 3.0f);
        assert_close(1.0f, v.x, 1e-6f);
        assert_close(2.0f, v.y, 1e-6f);
        assert_close(3.0f, v.z, 1e-6f);
        // The anonymous-union array view must agree with the named fields.
        assert_close(1.0f, v.xyz[0], 1e-6f);
        assert_close(2.0f, v.xyz[1], 1e-6f);
        assert_close(3.0f, v.xyz[2], 1e-6f);
    }

    void test_assign_copies() {
        PPVec3 src, dst;
        pp_vec3_set(&src, -4.0f, 5.5f, 6.0f);
        pp_vec3_assign(&dst, &src);
        assert_close(-4.0f, dst.x, 1e-6f);
        assert_close(5.5f, dst.y, 1e-6f);
        assert_close(6.0f, dst.z, 1e-6f);
    }

    void test_normalize_unit_length() {
        PPVec3 v;
        pp_vec3_set(&v, 3.0f, 0.0f, 4.0f); // length 5
        bool ok = pp_vec3_normalize(&v);
        assert_true(ok);

        float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        assert_close(1.0f, len, 1e-5f);

        // Direction preserved: 3-4-5 triangle.
        assert_close(0.6f, v.x, 1e-5f);
        assert_close(0.0f, v.y, 1e-5f);
        assert_close(0.8f, v.z, 1e-5f);
    }

    void test_normalize_already_unit() {
        PPVec3 v;
        pp_vec3_set(&v, 0.0f, 1.0f, 0.0f);
        assert_true(pp_vec3_normalize(&v));
        assert_close(0.0f, v.x, 1e-6f);
        assert_close(1.0f, v.y, 1e-6f);
        assert_close(0.0f, v.z, 1e-6f);
    }

    // A zero-length vector cannot be normalised: the function reports failure
    // and leaves a zero vector behind rather than producing NaNs.
    void test_normalize_zero_vector_fails() {
        PPVec3 v;
        pp_vec3_set(&v, 0.0f, 0.0f, 0.0f);
        bool ok = pp_vec3_normalize(&v);
        assert_false(ok);
        assert_close(0.0f, v.x, 1e-6f);
        assert_close(0.0f, v.y, 1e-6f);
        assert_close(0.0f, v.z, 1e-6f);
    }
};
