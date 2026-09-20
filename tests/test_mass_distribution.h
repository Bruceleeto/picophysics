#pragma once

/*
 * Tests for mass and inertia computation (pp_body_recompute_mass_inertia).
 *
 * The inertia tensor is a 3x3 stored row-major in PPMat3::m, with the diagonal
 * at indices 0 (Ixx), 4 (Iyy), 8 (Izz) and the symmetric products of inertia at
 * 1/3 (xy), 2/6 (xz), 5/7 (yz). Offsets contribute via the parallel-axis
 * theorem about the body origin. The struct is public, so tests read it directly.
 *
 * Reference formulae:
 *   solid sphere: Ixx = Iyy = Izz = (2/5) m r^2
 *   solid box   : Ixx = (1/12) m (h^2 + d^2), and cyclically for Iyy, Izz
 */

#include "../tools/test.h"

#include "../picophysics.h"

class MassDistributionTests : public test::TestCase {
public:
    void set_up() {
        pp_physics_clear();
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);
    }

    void test_sphere_inertia_tensor() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        // mass 2, radius 3 -> I = 2/5 * 2 * 9 = 7.2 on each diagonal.
        PPBody* b = pp_physics_create_sphere(3.0f, &p, 2.0f, 0);

        assert_close(7.2f, b->inertia.m[0], 1e-4f);
        assert_close(7.2f, b->inertia.m[4], 1e-4f);
        assert_close(7.2f, b->inertia.m[8], 1e-4f);
        // Centred single shape -> no products of inertia.
        assert_close(0.0f, b->inertia.m[1], 1e-6f);
        assert_close(0.0f, b->inertia.m[2], 1e-6f);
        assert_close(0.0f, b->inertia.m[5], 1e-6f);
        // inv_inertia is the elementwise reciprocal on the diagonal.
        assert_close(1.0f / 7.2f, b->inv_inertia.m[0], 1e-4f);
    }

    void test_box_inertia_tensor() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        // w=2, h=4, d=6, mass 12.
        // Ixx = 1/12*12*(16+36) = 52
        // Iyy = 1/12*12*(4+36)  = 40
        // Izz = 1/12*12*(4+16)  = 20
        PPBody* b = pp_physics_create_box(2.0f, 4.0f, 6.0f, &p, 12.0f, 0);

        assert_close(52.0f, b->inertia.m[0], 1e-3f);
        assert_close(40.0f, b->inertia.m[4], 1e-3f);
        assert_close(20.0f, b->inertia.m[8], 1e-3f);
    }

    void test_total_mass_sums_shapes() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_body(&p);
        pp_body_add_sphere(b, 1.0f, 3.0f, 0, 0.0f, 0.0f, 0.0f);
        pp_body_add_sphere(b, 1.0f, 5.0f, 0, 0.0f, 0.0f, 0.0f);

        assert_close(8.0f, pp_body_get_mass(b), 1e-5f);
        assert_close(1.0f / 8.0f, b->inv_mass, 1e-6f);
    }

    // Inertia is taken about the body's centre of mass, not its origin. A lone
    // shape offset from the origin IS the centre of mass, so the offset adds
    // nothing; the parallel-axis term only appears between shapes (next test).
    void test_offset_shape_parallel_axis() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_body(&p);
        // sphere mass 1, radius 1 (local I = 0.4), offset 2 along +X.
        pp_body_add_sphere(b, 1.0f, 1.0f, 0, 2.0f, 0.0f, 0.0f);

        assert_close(2.0f, b->com_local.x, 1e-5f);
        assert_close(0.4f, b->inertia.m[0], 1e-4f);
        assert_close(0.4f, b->inertia.m[4], 1e-4f);
        assert_close(0.4f, b->inertia.m[8], 1e-4f);
        assert_close(0.0f, b->inertia.m[1], 1e-6f);
        assert_close(0.0f, b->inertia.m[2], 1e-6f);
    }

    // Unequal shapes: the centre of mass sits between them, weighted by mass,
    // and each contributes m*d^2 measured from there.
    void test_lopsided_body_inertia_about_centre_of_mass() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_body(&p);
        pp_body_add_sphere(b, 1.0f, 1.0f, 0, 0.0f, 0.0f, 0.0f);
        pp_body_add_sphere(b, 1.0f, 3.0f, 0, 2.0f, 0.0f, 0.0f);

        // COM at (0*1 + 2*3) / 4 = 1.5.
        assert_close(1.5f, b->com_local.x, 1e-5f);
        // Ixx: 0.4*1 + 0.4*3. Iyy/Izz add 1*1.5^2 + 3*0.5^2 = 3.
        assert_close(1.6f, b->inertia.m[0], 1e-4f);
        assert_close(4.6f, b->inertia.m[4], 1e-4f);
        assert_close(4.6f, b->inertia.m[8], 1e-4f);
    }

    // A lopsided body spinning freely must turn about its centre of mass: with
    // no forces acting, that point cannot move.
    void test_lopsided_body_spins_about_centre_of_mass() {
        PPVec3 zero = {.xyz = {0.0f, 0.0f, 0.0f}};
        pp_physics_set_gravity(&zero);

        PPVec3 p = {.xyz = {0.0f, 5.0f, 0.0f}};
        PPBody* b = pp_physics_create_body(&p);
        pp_body_add_sphere(b, 0.5f, 1.0f, 0, 0.0f, 0.0f, 0.0f);
        pp_body_add_sphere(b, 0.5f, 3.0f, 0, 2.0f, 0.0f, 0.0f);
        pp_body_set_damping(b, 0.0f);
        pp_body_set_angular_damping(b, 0.0f);
        pp_body_set_angular_velocity(b, 0.0f, 0.0f, 2.0f);

        for (int i = 0; i < 600; ++i) {
            pp_physics_step(1.0f / 60.0f, 8, 4);

            // Centre of mass = origin + rot * (1.5, 0, 0): the rotated X axis.
            PPQuaternion q;
            PPVec3 o;
            pp_body_get_rotation(b, &q);
            pp_body_get_position(b, &o);
            float com_x = o.x + 1.5f * (1.0f - 2.0f * (q.y * q.y + q.z * q.z));
            float com_y = o.y + 1.5f * (2.0f * (q.x * q.y + q.w * q.z));
            assert_close(1.5f, com_x, 1e-3f);
            assert_close(5.0f, com_y, 1e-3f);
        }
    }

    // Two equal shapes placed symmetrically about the origin: products of
    // inertia cancel, and the perpendicular-axis terms add up.
    void test_symmetric_offsets_cancel_products() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_body(&p);
        pp_body_add_sphere(b, 1.0f, 1.0f, 0,  2.0f, 0.0f, 0.0f);
        pp_body_add_sphere(b, 1.0f, 1.0f, 0, -2.0f, 0.0f, 0.0f);

        assert_close(0.8f, b->inertia.m[0], 1e-4f);   // 2 * 0.4
        assert_close(8.8f, b->inertia.m[4], 1e-4f);   // 2 * (0.4 + 4)
        assert_close(8.8f, b->inertia.m[8], 1e-4f);
        assert_close(0.0f, b->inertia.m[1], 1e-5f);   // products cancel
        assert_close(2.0f, pp_body_get_mass(b), 1e-5f); // two unit-mass spheres
    }

    // A massless (static) body has zero inverse mass and a zero inverse inertia
    // tensor, so impulses can never move or spin it.
    void test_static_body_has_zero_inverses() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 0.0f, 0);

        assert_close(0.0f, b->inv_mass, 1e-8f);
        for (int i = 0; i < 9; ++i) {
            assert_close(0.0f, b->inv_inertia.m[i], 1e-8f);
        }
    }

    // Changing a body's mass scales the existing shape masses proportionally,
    // so the inertia tensor scales by the same factor.
    void test_set_mass_scales_inertia() {
        PPVec3 p = {.xyz = {0.0f, 0.0f, 0.0f}};
        // sphere mass 2, radius 1 -> I = 0.4 * 2 = 0.8.
        PPBody* b = pp_physics_create_sphere(1.0f, &p, 2.0f, 0);
        assert_close(0.8f, b->inertia.m[0], 1e-4f);

        pp_body_set_mass(b, 4.0f); // doubles
        assert_close(4.0f, pp_body_get_mass(b), 1e-5f);
        assert_close(0.25f, b->inv_mass, 1e-6f);
        assert_close(1.6f, b->inertia.m[0], 1e-4f); // doubled too
    }
};
