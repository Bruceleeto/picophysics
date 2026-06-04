#pragma once

/*
 * Tests for body dynamics: how linear/angular velocity, forces, acceleration
 * and damping evolve a free body through pp_physics_step.
 *
 * The full PPBody struct is public, so where there is no getter (e.g. angular
 * velocity, acceleration) these tests read the fields directly.
 *
 * Step ordering inside pp_physics_step that matters here: integrate_forces
 * applies (acc + gravity)*t to velocity, then linear damping, then clears acc;
 * integrate_velocities then advances position and rotation. acc/a_acc are
 * cleared every step, so forces are per-step impulses and must be reapplied.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class BodyDynamicsTests : public test::TestCase {
public:
    static constexpr float DT = 1.0f / 60.0f;

    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    // A free body with no damping/gravity drifts at constant velocity.
    void test_constant_velocity_drift() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_damping(b, 0.0f);
        pp_body_set_velocity(b, 3.0f, 0.0f, 0.0f);

        for (int i = 0; i < 60; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPVec3 pos;
        pp_body_get_position(b, &pos);
        assert_close(3.0f, pos.x, 1e-3f); // 3 u/s for 1 s
        assert_close(0.0f, pos.y, 1e-4f);
    }

    void test_angular_velocity_set() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_angular_velocity(b, 1.0f, 2.0f, 3.0f);
        assert_close(1.0f, b->a_vel.x, 1e-6f);
        assert_close(2.0f, b->a_vel.y, 1e-6f);
        assert_close(3.0f, b->a_vel.z, 1e-6f);
    }

    // F = ma: one step of a constant force changes velocity by (F/m)*dt.
    void test_force_produces_acceleration() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 2.0f, 0); // mass 2
        pp_body_set_damping(b, 0.0f);

        pp_body_add_force(b, 10.0f, 0.0f, 0.0f);
        pp_physics_step(DT, 8, 4);

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close((10.0f / 2.0f) * DT, v.x, 1e-5f);
    }

    // Forces accumulate within a single step.
    void test_forces_accumulate_in_one_step() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 2.0f, 0);
        pp_body_set_damping(b, 0.0f);

        pp_body_add_force(b, 10.0f, 0.0f, 0.0f);
        pp_body_add_force(b, 10.0f, 0.0f, 0.0f);
        pp_physics_step(DT, 8, 4);

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close((20.0f / 2.0f) * DT, v.x, 1e-5f);
    }

    // Forces are cleared each step, so a single add_force only acts for one step.
    void test_force_is_per_step() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 2.0f, 0);
        pp_body_set_damping(b, 0.0f);

        pp_body_add_force(b, 10.0f, 0.0f, 0.0f);
        pp_physics_step(DT, 8, 4); // force applied here...
        pp_physics_step(DT, 8, 4); // ...and gone now

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close((10.0f / 2.0f) * DT, v.x, 1e-5f);
    }

    // A massless body ignores forces (a = F/m is undefined, so skipped).
    void test_force_on_static_body_ignored() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 0.0f, 0);
        pp_body_add_force(b, 100.0f, 0.0f, 0.0f);
        pp_physics_step(DT, 8, 4);

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close(0.0f, v.x, 1e-6f);
    }

    void test_set_acceleration() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_damping(b, 0.0f);
        pp_body_set_acceleration(b, 0.0f, 5.0f, 0.0f);
        pp_physics_step(DT, 8, 4);

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close(5.0f * DT, v.y, 1e-5f);
    }

    // Linear damping bleeds off velocity geometrically: v_n = v0 * (1 - d*t)^n.
    void test_linear_damping_decays_velocity() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_damping(b, 0.5f);
        pp_body_set_velocity(b, 10.0f, 0.0f, 0.0f);

        const int n = 120;
        for (int i = 0; i < n; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        float expected = 10.0f * std::pow(1.0f - 0.5f * DT, (float) n);
        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close(expected, v.x, 1e-3f);
        assert_true(v.x < 10.0f);
    }

    // A constant angular velocity rotates the body; with damping off the total
    // turn after one second equals the angular speed (rad).
    void test_angular_velocity_rotates_body() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_angular_damping(b, 0.0f);
        pp_body_set_rotation(b, 0.0f, 0.0f, 0.0f, 1.0f);
        pp_body_set_angular_velocity(b, 0.0f, 1.0f, 0.0f); // 1 rad/s about Y

        for (int i = 0; i < 60; ++i) {
            pp_physics_step(DT, 8, 4);
        }

        PPQuaternion identity, rot;
        pp_quat_init(&identity);
        pp_body_get_rotation(b, &rot);
        assert_close(1.0f, pp_quat_angle_between(&identity, &rot), 5e-3f);
    }

    // Torque about a sphere's axis: ang_acc = torque / I, with I = (2/5) m r^2.
    void test_angular_force_produces_angular_velocity() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0); // I = 0.4
        pp_body_set_angular_damping(b, 0.0f);

        // torque 0.4 about x -> ang_acc = 0.4 / 0.4 = 1 rad/s^2
        pp_body_add_angular_force(b, 0.4f, 0.0f, 0.0f);
        pp_physics_step(DT, 8, 4);

        assert_close(1.0f * DT, b->a_vel.x, 1e-5f);
    }

    void test_velocity_at_position_linear_only() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_velocity(b, 5.0f, 0.0f, 0.0f);

        PPVec3 query = {.xyz = {2.0f, 3.0f, 4.0f}};
        PPVec3 v;
        pp_body_get_velocity_at_position(b, &query, &v);
        // No spin: every point shares the linear velocity.
        assert_close(5.0f, v.x, 1e-6f);
        assert_close(0.0f, v.y, 1e-6f);
        assert_close(0.0f, v.z, 1e-6f);
    }

    void test_velocity_at_position_with_spin() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_angular_velocity(b, 0.0f, 0.0f, 1.0f); // spin about +Z

        // Point one unit out along +X: v = w x r = (0,0,1) x (1,0,0) = (0,1,0).
        PPVec3 query = {.xyz = {1.0f, 0.0f, 0.0f}};
        PPVec3 v;
        pp_body_get_velocity_at_position(b, &query, &v);
        assert_close(0.0f, v.x, 1e-6f);
        assert_close(1.0f, v.y, 1e-6f);
        assert_close(0.0f, v.z, 1e-6f);
    }

    // A force applied off the centre of mass produces both linear acceleration
    // and torque.
    void test_force_at_position_creates_torque() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0); // mass 1, I = 0.4
        pp_body_set_damping(b, 0.0f);
        pp_body_set_angular_damping(b, 0.0f);

        // Force +Y applied at (1,0,0): torque = (1,0,0) x (0,1,0) = (0,0,1).
        PPVec3 world_pos = {.xyz = {1.0f, 0.0f, 0.0f}};
        PPVec3 force = {.xyz = {0.0f, 1.0f, 0.0f}};
        pp_body_add_force_at_position(b, &world_pos, &force);
        pp_physics_step(DT, 8, 4);

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close(1.0f * DT, v.y, 1e-5f);          // linear: (F/m)*dt
        assert_close((1.0f / 0.4f) * DT, b->a_vel.z, 1e-5f); // angular: (T/I)*dt
    }

    void test_angular_velocity_limit() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_angular_damping(b, 0.0f);
        pp_body_limit_angular_velocity(b, 2.0f);
        pp_body_set_angular_velocity(b, 0.0f, 10.0f, 0.0f);

        pp_physics_step(DT, 8, 4);

        float speed = std::sqrt(b->a_vel.x * b->a_vel.x +
                                b->a_vel.y * b->a_vel.y +
                                b->a_vel.z * b->a_vel.z);
        assert_true(speed <= 2.0f + 1e-3f);
        assert_close(2.0f, speed, 1e-2f);
    }
};
