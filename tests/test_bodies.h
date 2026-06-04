#pragma once

/*
 * Tests for body lifecycle and accessors: creation, counts, destruction,
 * mass, kind, user data, position/velocity round-trips and multi-shape bodies.
 */

#include "../tools/test.h"

#include "../picophysics.h"

class BodyTests : public test::TestCase {
public:
    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    void test_create_sphere_increments_count() {
        assert_equal((size_t) 0, pp_physics_body_count());
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        assert_equal((size_t) 1, pp_physics_body_count());
        pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        assert_equal((size_t) 2, pp_physics_body_count());
    }

    void test_sphere_has_one_shape() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(2.0f, &p, 1.0f, 0);
        assert_equal((size_t) 1, pp_body_get_shape_count(b));

        const PPShape* s = pp_body_get_shape(b, 0);
        assert_is_not_null((void*) s);
        assert_equal((int) PP_OBJECT_TYPE_SPHERE, (int) s->type);
        assert_close(2.0f, s->sphere.radius, 1e-6f);
    }

    void test_create_box() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_box(2.0f, 4.0f, 6.0f, &p, 1.0f, 0);
        const PPShape* s = pp_body_get_shape(b, 0);
        assert_equal((int) PP_OBJECT_TYPE_BOX, (int) s->type);
        assert_close(1.0f, s->box.half_extents.x, 1e-6f);
        assert_close(2.0f, s->box.half_extents.y, 1e-6f);
        assert_close(3.0f, s->box.half_extents.z, 1e-6f);
    }

    // Destroying a body drops the live count but keeps a slot in the total
    // count (slots are recycled, not removed).
    void test_destroy_body() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* a = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        assert_equal((size_t) 2, pp_physics_body_count());

        pp_physics_destroy_body(a);
        assert_equal((size_t) 1, pp_physics_body_count());
        assert_equal((size_t) 2, pp_physics_body_total_count());
    }

    // A freed slot is reused by the next creation rather than growing the pool.
    void test_destroyed_slot_is_recycled() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* a = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_physics_destroy_body(a);
        assert_equal((size_t) 1, pp_physics_body_total_count());

        pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        assert_equal((size_t) 1, pp_physics_body_count());
        assert_equal((size_t) 1, pp_physics_body_total_count());
    }

    void test_mass_round_trip() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 5.0f, 0);
        assert_close(5.0f, pp_body_get_mass(b), 1e-5f);

        pp_body_set_mass(b, 10.0f);
        assert_close(10.0f, pp_body_get_mass(b), 1e-5f);
    }

    // A zero-mass body is the engine's "static" marker.
    void test_zero_mass_is_static() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 0.0f, 0);
        assert_close(0.0f, pp_body_get_mass(b), 1e-6f);
    }

    void test_kind_round_trip() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 3);
        assert_equal((BodyKind) 3, pp_body_get_kind(b));

        pp_body_set_kind(b, 9);
        assert_equal((BodyKind) 9, pp_body_get_kind(b));
    }

    void test_user_data_round_trip() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        assert_is_null(pp_body_get_user_data(b));

        int marker = 42;
        pp_body_set_user_data(b, &marker);
        assert_equal((void*) &marker, pp_body_get_user_data(b));
    }

    void test_position_round_trip() {
        PPVec3 p = {.xyz = {1.0f, 2.0f, 3.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);

        PPVec3 got;
        pp_body_get_position(b, &got);
        assert_close(1.0f, got.x, 1e-6f);
        assert_close(2.0f, got.y, 1e-6f);
        assert_close(3.0f, got.z, 1e-6f);

        pp_body_set_position(b, -4.0f, -5.0f, -6.0f);
        pp_body_get_position(b, &got);
        assert_close(-4.0f, got.x, 1e-6f);
        assert_close(-5.0f, got.y, 1e-6f);
        assert_close(-6.0f, got.z, 1e-6f);
    }

    void test_velocity_round_trip() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        pp_body_set_velocity(b, 7.0f, 8.0f, 9.0f);

        PPVec3 v;
        pp_body_get_velocity(b, &v);
        assert_close(7.0f, v.x, 1e-6f);
        assert_close(8.0f, v.y, 1e-6f);
        assert_close(9.0f, v.z, 1e-6f);
    }

    // A body can hold several shapes; the bounding radius reflects the most
    // distant one (radius + offset).
    void test_multiple_shapes() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_body(&p);
        assert_equal((size_t) 0, pp_body_get_shape_count(b));

        pp_body_add_sphere(b, 1.0f, 1.0f, 0, 0.0f, 0.0f, 0.0f);
        pp_body_add_sphere(b, 0.5f, 1.0f, 0, 3.0f, 0.0f, 0.0f); // offset 3
        assert_equal((size_t) 2, pp_body_get_shape_count(b));

        // Far shape dominates: radius 0.5 + offset 3 = 3.5.
        assert_close(3.5f, pp_body_get_radius(b), 1e-5f);
    }
};
