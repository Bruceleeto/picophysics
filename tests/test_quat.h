#pragma once

/*
 * Tests for the public quaternion helpers and the body basis vectors derived
 * from a rotation (forward / up / right). The library's convention, taken from
 * the identity quaternion, is forward = +Z, up = +Y, right = +X.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class QuatTests : public test::TestCase {
public:
    void set_up() {
        pp_physics_clear();
    }

    void test_init_is_identity() {
        PPQuaternion q;
        pp_quat_init(&q);
        assert_close(0.0f, q.x, 1e-6f);
        assert_close(0.0f, q.y, 1e-6f);
        assert_close(0.0f, q.z, 1e-6f);
        assert_close(1.0f, q.w, 1e-6f);
    }

    void test_assign_copies() {
        PPQuaternion src, dst;
        src.x = 0.1f; src.y = 0.2f; src.z = 0.3f; src.w = 0.4f;
        pp_quat_assign(&dst, &src);
        assert_close(0.1f, dst.x, 1e-6f);
        assert_close(0.2f, dst.y, 1e-6f);
        assert_close(0.3f, dst.z, 1e-6f);
        assert_close(0.4f, dst.w, 1e-6f);
    }

    void test_angle_between_identity_is_zero() {
        PPQuaternion a, b;
        pp_quat_init(&a);
        pp_quat_init(&b);
        assert_close(0.0f, pp_quat_angle_between(&a, &b), 1e-5f);
    }

    // A quaternion representing a 90-degree rotation about Y, compared with the
    // identity, should report an angle of pi/2.
    void test_angle_between_quarter_turn() {
        PPQuaternion identity;
        pp_quat_init(&identity);

        PPQuaternion rot;
        float half = (float) (M_PI / 4.0); // half of 90 degrees
        rot.x = 0.0f;
        rot.y = std::sin(half);
        rot.z = 0.0f;
        rot.w = std::cos(half);

        assert_close((float) (M_PI / 2.0), pp_quat_angle_between(&identity, &rot), 1e-4f);
    }

    // The shortest rotation taking +Z onto +X is a quarter turn; feeding those
    // two vectors to pp_quat_between and applying it (via the body basis) should
    // map forward from +Z to +X.
    void test_between_maps_forward() {
        PPVec3 from = {.xyz = {0.0f, 0.0f, 1.0f}};
        PPVec3 to = {.xyz = {1.0f, 0.0f, 0.0f}};
        PPQuaternion q;
        pp_quat_between(&from, &to, &q);

        // Drive a body with this rotation and check its forward vector.
        PPVec3 origin = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &origin, 1.0f, 0);
        pp_body_set_rotation(b, q.x, q.y, q.z, q.w);

        PPVec3 f;
        pp_body_get_forward(b, &f);
        assert_close(1.0f, f.x, 1e-4f);
        assert_close(0.0f, f.y, 1e-4f);
        assert_close(0.0f, f.z, 1e-4f);
    }

    void test_between_identical_vectors_is_identity() {
        PPVec3 a = {.xyz = {0.0f, 1.0f, 0.0f}};
        PPQuaternion q;
        pp_quat_between(&a, &a, &q);
        assert_close(0.0f, q.x, 1e-5f);
        assert_close(0.0f, q.y, 1e-5f);
        assert_close(0.0f, q.z, 1e-5f);
        assert_close(1.0f, q.w, 1e-5f);
    }

    void test_basis_vectors_of_identity() {
        PPVec3 origin = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &origin, 1.0f, 0);
        pp_body_set_rotation(b, 0.0f, 0.0f, 0.0f, 1.0f);

        PPVec3 f, u, r;
        pp_body_get_forward(b, &f);
        pp_body_get_up(b, &u);
        pp_body_get_right(b, &r);

        assert_close(0.0f, f.x, 1e-6f); assert_close(0.0f, f.y, 1e-6f); assert_close(1.0f, f.z, 1e-6f);
        assert_close(0.0f, u.x, 1e-6f); assert_close(1.0f, u.y, 1e-6f); assert_close(0.0f, u.z, 1e-6f);
        assert_close(1.0f, r.x, 1e-6f); assert_close(0.0f, r.y, 1e-6f); assert_close(0.0f, r.z, 1e-6f);
    }

    // slerp at the endpoints returns the endpoints; the half-way point of a
    // 90-degree turn about Y is a 45-degree turn.
    void test_slerp_endpoints_and_midpoint() {
        PPQuaternion a;
        pp_quat_init(&a);

        float half = (float) (M_PI / 4.0);
        PPQuaternion b;
        b.x = 0.0f; b.y = std::sin(half); b.z = 0.0f; b.w = std::cos(half);

        PPQuaternion r;
        // acos() is ill-conditioned near a zero angle, so the endpoint checks
        // use a looser tolerance than the well-conditioned midpoint check.
        pp_quat_slerp(&a, &b, 0.0f, &r);
        assert_close(0.0f, pp_quat_angle_between(&a, &r), 2e-3f);

        pp_quat_slerp(&a, &b, 1.0f, &r);
        assert_close(0.0f, pp_quat_angle_between(&b, &r), 2e-3f);

        pp_quat_slerp(&a, &b, 0.5f, &r);
        // Half of a 90-degree rotation is 45 degrees from the start.
        assert_close((float) (M_PI / 4.0), pp_quat_angle_between(&a, &r), 1e-4f);
    }
};
