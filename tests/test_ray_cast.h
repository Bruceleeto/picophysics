#pragma once

/*
 * Tests for pp_physics_ray_cast: the bounded ray that goes through the
 * triangle query, reports a surface normal and kind, and is cheap enough to
 * fire from every wheel every frame.
 */

#include "../tools/test.h"

#include "../picophysics.h"

#include <cmath>

class RayCastTests : public test::TestCase {
public:
    // State for the host-side triangle query used below.
    struct QueryWorld {
        PPTriangle floor;
        int calls;
        float largest_reach;
        float smallest_reach;
        float flood_above_reach;   // return `max` junk triangles above this
    };

    void set_up() {
        pp_physics_clear();
        pp_physics_set_triangle_query(NULL, NULL);
        PPVec3 zero = vec(0.0f, 0.0f, 0.0f);
        pp_physics_set_gravity(&zero);
    }

    void tear_down() {
        pp_physics_set_triangle_query(NULL, NULL);
    }

    static PPVec3 vec(float x, float y, float z) {
        PPVec3 v = {.xyz = {x, y, z}};
        return v;
    }

    void make_floor(BodyKind kind) {
        PPVec3 v1 = vec(-100.0f, 0.0f, -100.0f);
        PPVec3 v2 = vec(   0.0f, 0.0f,  100.0f);
        PPVec3 v3 = vec( 100.0f, 0.0f, -100.0f);
        pp_physics_create_triangle(&v1, &v2, &v3, kind);
    }

    static int query(const PPVec3* centre, float reach, PPTriangle* out, int max, void* user) {
        QueryWorld* w = (QueryWorld*) user;
        w->calls++;
        if (reach > w->largest_reach) w->largest_reach = reach;
        if (reach < w->smallest_reach) w->smallest_reach = reach;

        if (reach > w->flood_above_reach) {
            // A crowded region: more triangles than fit, none of them the floor.
            PPVec3 a = vec(500.0f, 500.0f, 500.0f), b = vec(501.0f, 500.0f, 500.0f), c = vec(500.0f, 500.0f, 501.0f);
            for (int i = 0; i < max; ++i) {
                pp_triangle_init(&out[i], &a, &b, &c, 0);
            }
            return max;
        }

        // The floor is at y = 0: hand it over only if the sphere reaches it.
        if (std::fabs(centre->y) > reach || max < 1) {
            return 0;
        }
        out[0] = w->floor;
        return 1;
    }

    void init_query_world(QueryWorld* w, float flood_above_reach) {
        PPVec3 v1 = vec(-100.0f, 0.0f, -100.0f);
        PPVec3 v2 = vec(   0.0f, 0.0f,  100.0f);
        PPVec3 v3 = vec( 100.0f, 0.0f, -100.0f);
        pp_triangle_init(&w->floor, &v1, &v2, &v3, 7);
        w->calls = 0;
        w->largest_reach = 0.0f;
        w->smallest_reach = 1e9f;
        w->flood_above_reach = flood_above_reach;
        pp_physics_set_triangle_query(query, w);
    }

    // ---- static world -------------------------------------------------------

    void test_hits_floor_with_distance_normal_and_kind() {
        make_floor(3);
        PPVec3 o = vec(1.0f, 2.0f, 1.0f), d = vec(0.0f, -1.0f, 0.0f);
        PPRayHit hit;
        assert_true(pp_physics_ray_cast(&o, &d, 5.0f, NULL, NULL, &hit));

        assert_is_null(hit.body);
        assert_equal(3, (int) hit.kind);
        assert_close(2.0f, hit.distance, 0.0001f);
        assert_close(1.0f, hit.point.x, 0.0001f);
        assert_close(0.0f, hit.point.y, 0.0001f);
        assert_close(1.0f, hit.normal.y, 0.0001f);
    }

    void test_respects_max_distance() {
        make_floor(0);
        PPVec3 o = vec(0.0f, 2.0f, 0.0f), d = vec(0.0f, -1.0f, 0.0f);
        PPRayHit hit;
        assert_false(pp_physics_ray_cast(&o, &d, 1.9f, NULL, NULL, &hit));
        assert_true(pp_physics_ray_cast(&o, &d, 2.1f, NULL, NULL, &hit));
    }

    // The distance is in world units whatever the length of `direction`.
    void test_direction_need_not_be_unit() {
        make_floor(0);
        PPVec3 o = vec(0.0f, 2.0f, 0.0f), d = vec(0.0f, -40.0f, 0.0f);
        PPRayHit hit;
        assert_true(pp_physics_ray_cast(&o, &d, 5.0f, NULL, NULL, &hit));
        assert_close(2.0f, hit.distance, 0.0001f);
    }

    // Seen from underneath, the normal still faces the ray.
    void test_normal_faces_the_ray() {
        make_floor(0);
        PPVec3 o = vec(0.0f, -2.0f, 0.0f), d = vec(0.0f, 1.0f, 0.0f);
        PPRayHit hit;
        assert_true(pp_physics_ray_cast(&o, &d, 5.0f, NULL, NULL, &hit));
        assert_close(-1.0f, hit.normal.y, 0.0001f);
    }

    void test_ignore_kinds_skips_triangle() {
        make_floor(4);
        PPVec3 o = vec(0.0f, 2.0f, 0.0f), d = vec(0.0f, -1.0f, 0.0f);
        BodyKind ignore[] = {4, 0};
        assert_false(pp_physics_ray_cast(&o, &d, 5.0f, ignore, NULL, NULL));
        assert_true(pp_physics_ray_cast(&o, &d, 5.0f, NULL, NULL, NULL));
    }

    // ---- through the triangle query ----------------------------------------

    // With a query set the ray sees the host's world, and asks only for a
    // sphere around itself.
    void test_uses_the_triangle_query() {
        QueryWorld w;
        init_query_world(&w, 1e9f);

        PPVec3 o = vec(0.0f, 0.5f, 0.0f), d = vec(0.0f, -1.0f, 0.0f);
        PPRayHit hit;
        assert_true(pp_physics_ray_cast(&o, &d, 1.0f, NULL, NULL, &hit));
        assert_close(0.5f, hit.distance, 0.0001f);
        assert_equal(7, (int) hit.kind);
        assert_equal(1, w.calls);
        assert_true(w.largest_reach < 0.6f);   // half the ray, plus a margin
        assert_true(w.largest_reach >= 0.5f);

        // Too short to reach: nothing, and still just one small query.
        w.calls = 0;
        assert_false(pp_physics_ray_cast(&o, &d, 0.4f, NULL, NULL, &hit));
        assert_equal(1, w.calls);
    }

    // A query that comes back full is split and retried until it is not.
    void test_full_query_is_split_until_it_fits() {
        QueryWorld w;
        init_query_world(&w, 2.0f);

        PPVec3 o = vec(0.0f, 9.0f, 0.0f), d = vec(0.0f, -1.0f, 0.0f);
        PPRayHit hit;
        assert_true(pp_physics_ray_cast(&o, &d, 20.0f, NULL, NULL, &hit));
        assert_is_null(hit.body);
        assert_close(9.0f, hit.distance, 0.001f);
        assert_true(w.calls > 1);
        assert_true(w.smallest_reach <= 2.0f);
    }

    // ---- bodies -------------------------------------------------------------

    void test_sphere_hit_reports_outward_normal() {
        PPVec3 p = vec(0.0f, 0.0f, 0.0f);
        PPBody* s = pp_physics_create_sphere(1.0f, &p, 1.0f, 2);
        PPVec3 o = vec(0.0f, 0.0f, -5.0f), d = vec(0.0f, 0.0f, 1.0f);
        PPRayHit hit;
        assert_true(pp_physics_ray_cast(&o, &d, 10.0f, NULL, NULL, &hit));
        assert_equal((const PPBody*) s, hit.body);
        assert_equal(2, (int) hit.kind);
        assert_close(4.0f, hit.distance, 0.0001f);
        assert_close(-1.0f, hit.normal.z, 0.0001f);
    }

    // A long thin box turned 90 degrees about Y: a ray that would have hit it
    // unturned now misses, and one that would have missed now hits its side.
    void test_box_rotation_is_respected() {
        PPVec3 p = vec(0.0f, 0.0f, 0.0f);
        PPBody* box = pp_physics_create_box(6.0f, 1.0f, 1.0f, &p, 1.0f, 0);
        pp_body_set_rotation(box, 0.0f, std::sin(0.785398f), 0.0f, std::cos(0.785398f));

        PPVec3 d = vec(0.0f, -1.0f, 0.0f);
        PPVec3 along_old_length = vec(2.5f, 5.0f, 0.0f);
        PPVec3 along_new_length = vec(0.0f, 5.0f, 2.5f);

        PPRayHit hit;
        assert_false(pp_physics_ray_cast(&along_old_length, &d, 10.0f, NULL, NULL, &hit));
        assert_true(pp_physics_ray_cast(&along_new_length, &d, 10.0f, NULL, NULL, &hit));
        assert_equal((const PPBody*) box, hit.body);
        assert_close(4.5f, hit.distance, 0.001f);
        assert_close(1.0f, hit.normal.y, 0.001f);
    }

    // A box tipped 45 degrees about Z presents an edge-on roof: the normal is
    // the tilted face's, not the world's up.
    void test_tilted_box_reports_face_normal() {
        PPVec3 p = vec(0.0f, 0.0f, 0.0f);
        PPBody* box = pp_physics_create_box(2.0f, 2.0f, 2.0f, &p, 1.0f, 0);
        pp_body_set_rotation(box, 0.0f, 0.0f, std::sin(0.392699f), std::cos(0.392699f));

        PPVec3 o = vec(0.5f, 5.0f, 0.0f), d = vec(0.0f, -1.0f, 0.0f);
        PPRayHit hit;
        assert_true(pp_physics_ray_cast(&o, &d, 10.0f, NULL, NULL, &hit));
        assert_close(0.7071f, std::fabs(hit.normal.x), 0.001f);
        assert_close(0.7071f, hit.normal.y, 0.001f);
        assert_close(0.0f, hit.normal.z, 0.001f);
    }

    void test_capsule_side_normal() {
        PPVec3 p = vec(0.0f, 0.0f, 0.0f);
        PPBody* c = pp_physics_create_capsule(0.5f, 3.0f, &p, 1.0f, 0);
        PPVec3 o = vec(-5.0f, 0.4f, 0.0f), d = vec(1.0f, 0.0f, 0.0f);
        PPRayHit hit;
        assert_true(pp_physics_ray_cast(&o, &d, 10.0f, NULL, NULL, &hit));
        assert_equal((const PPBody*) c, hit.body);
        assert_close(4.5f, hit.distance, 0.001f);
        assert_close(-1.0f, hit.normal.x, 0.001f);
        assert_close(0.0f, hit.normal.y, 0.001f);
    }

    // The nearest thing wins, whichever kind of thing it is.
    void test_nearest_of_body_and_world() {
        make_floor(0);
        PPVec3 p = vec(0.0f, 1.0f, 0.0f);
        PPBody* crate = pp_physics_create_box(1.0f, 1.0f, 1.0f, &p, 1.0f, 0);

        PPVec3 d = vec(0.0f, -1.0f, 0.0f);
        PPVec3 over_crate = vec(0.0f, 5.0f, 0.0f), beside_crate = vec(3.0f, 5.0f, 0.0f);
        PPRayHit hit;

        assert_true(pp_physics_ray_cast(&over_crate, &d, 10.0f, NULL, NULL, &hit));
        assert_equal((const PPBody*) crate, hit.body);
        assert_close(3.5f, hit.distance, 0.001f);

        assert_true(pp_physics_ray_cast(&beside_crate, &d, 10.0f, NULL, NULL, &hit));
        assert_is_null(hit.body);
        assert_close(5.0f, hit.distance, 0.001f);
    }

    // A wheel ray starts inside its own chassis. ignore_body skips just that
    // body; another of the same kind is still hit.
    void test_ignore_body_skips_only_that_body() {
        PPVec3 pa = vec(0.0f, 2.0f, 0.0f), pb = vec(0.0f, 0.0f, 0.0f);
        PPBody* chassis = pp_physics_create_box(2.0f, 1.0f, 4.0f, &pa, 1.0f, 5);
        PPBody* other = pp_physics_create_box(2.0f, 1.0f, 4.0f, &pb, 1.0f, 5);

        PPVec3 o = vec(0.0f, 3.0f, 0.0f), d = vec(0.0f, -1.0f, 0.0f);
        PPRayHit hit;
        assert_true(pp_physics_ray_cast(&o, &d, 10.0f, NULL, NULL, &hit));
        assert_equal((const PPBody*) chassis, hit.body);

        assert_true(pp_physics_ray_cast(&o, &d, 10.0f, NULL, chassis, &hit));
        assert_equal((const PPBody*) other, hit.body);
        assert_close(2.5f, hit.distance, 0.001f);
    }

    // Starting inside a body is not a hit on that body.
    void test_ray_from_inside_a_body_passes_out() {
        make_floor(0);
        PPVec3 p = vec(0.0f, 2.0f, 0.0f);
        pp_physics_create_box(2.0f, 2.0f, 2.0f, &p, 1.0f, 0);

        PPVec3 o = vec(0.0f, 2.0f, 0.0f), d = vec(0.0f, -1.0f, 0.0f);
        PPRayHit hit;
        assert_true(pp_physics_ray_cast(&o, &d, 10.0f, NULL, NULL, &hit));
        assert_is_null(hit.body);
        assert_close(2.0f, hit.distance, 0.001f);
    }

    void test_bodies_beyond_max_distance_are_missed() {
        PPVec3 p = vec(0.0f, 0.0f, 10.0f);
        pp_physics_create_sphere(1.0f, &p, 1.0f, 0);
        PPVec3 o = vec(0.0f, 0.0f, 0.0f), d = vec(0.0f, 0.0f, 1.0f);
        assert_false(pp_physics_ray_cast(&o, &d, 8.9f, NULL, NULL, NULL));
        assert_true(pp_physics_ray_cast(&o, &d, 9.1f, NULL, NULL, NULL));
    }
};
