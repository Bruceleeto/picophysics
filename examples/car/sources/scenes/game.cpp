
#include "game.h"

#define PICOPHYSICS_IMPLEMENTATION
#include <picophysics.h>

#define CAR_BODY_KIND 1
#define CAR_INNER_KIND 2
#define BALL_KIND 3

/* These are different things because we want the bigger
 * car sphere (body) to collide with walls, but if it collides
 * with the floor the car will float in the air. The smaller sphere
 * (inner) collides with the floor only */
#define ENV_FLOOR_KIND 4
#define ENV_WALL_KIND 5

void define_stadium()
{
    /* This is a recreation of the DS64 SDF
     * field using triangles. Later this can be
     * extended to improve the curve */

    // Floor
    PPVec3 v1, v2, v3, v4;
    pp_vec3_set(&v1, -50, 0, -50);
    pp_vec3_set(&v2, -50, 0, 50);
    pp_vec3_set(&v3, 50, 0, 50);
    pp_vec3_set(&v4, 50, 0, -50);
    auto f1 = pp_physics_create_triangle(&v1, &v2, &v3, ENV_FLOOR_KIND);
    auto f2 = pp_physics_create_triangle(&v1, &v3, &v4, ENV_FLOOR_KIND);

    assert(f1->n.xyz[1] > 0.0f);
    assert(f2->n.xyz[1] > 0.0f);

    const float w = 40.0f;
    const float d = 26.0f;
    const float hw = w * 0.5f;
    const float hd = d * 0.5f;
    const float h = 40.0f;

    // Left wall
    pp_vec3_set(&v1, -hw, 0, hd);
    pp_vec3_set(&v2, -hw, 0, -hd);
    pp_vec3_set(&v3, -hw, h, -hd);
    pp_vec3_set(&v4, -hw, h, hd);

    auto lw1 = pp_physics_create_triangle(&v1, &v2, &v3, ENV_WALL_KIND);
    auto lw2 = pp_physics_create_triangle(&v1, &v3, &v4, ENV_WALL_KIND);
    assert(lw1->n.xyz[0] > 0.0f);
    assert(lw2->n.xyz[0] > 0.0f);

    pp_vec3_set(&v1, hw, 0, -hd);
    pp_vec3_set(&v2, hw, 0, hd);
    pp_vec3_set(&v3, hw, h, hd);
    pp_vec3_set(&v4, hw, h, -hd);

    auto rw1 = pp_physics_create_triangle(&v1, &v2, &v3, ENV_WALL_KIND);
    auto rw2 = pp_physics_create_triangle(&v1, &v3, &v4, ENV_WALL_KIND);
    assert(rw1->n.xyz[0] < 0.0f);
    assert(rw2->n.xyz[0] < 0.0f);

    float qw = hw * 0.5f;

    // Back wall
    pp_vec3_set(&v1, -qw, 0, -hd);
    pp_vec3_set(&v2, qw, 0, -hd);
    pp_vec3_set(&v3, qw, h, -hd);
    pp_vec3_set(&v4, -qw, h, -hd);

    auto bw1 = pp_physics_create_triangle(&v1, &v2, &v3, ENV_WALL_KIND);
    auto bw2 = pp_physics_create_triangle(&v1, &v3, &v4, ENV_WALL_KIND);
    assert(bw1->n.xyz[2] > 0.0f);
    assert(bw2->n.xyz[2] > 0.0f);

    pp_vec3_set(&v1, qw, 0, hd);
    pp_vec3_set(&v2, -qw, 0, hd);
    pp_vec3_set(&v3, -qw, h, hd);
    pp_vec3_set(&v4, qw, h, hd);

    auto fw1 = pp_physics_create_triangle(&v1, &v2, &v3, ENV_WALL_KIND);
    auto fw2 = pp_physics_create_triangle(&v1, &v3, &v4, ENV_WALL_KIND);
    assert(fw1->n.xyz[2] < 0.0f);
    assert(fw2->n.xyz[2] < 0.0f);

    float qd = hd * 0.4f;

    // Back left angled
    pp_vec3_set(&v1, -qw, 0, -hd);
    pp_vec3_set(&v2, -qw, h, -hd);
    pp_vec3_set(&v3, -hw, h, -qd);
    pp_vec3_set(&v4, -hw, 0, -qd);

    pp_physics_create_triangle(&v1, &v2, &v3, ENV_WALL_KIND);
    pp_physics_create_triangle(&v1, &v3, &v4, ENV_WALL_KIND);

    // Back right angled
    pp_vec3_set(&v1, qw, 0, -hd);
    pp_vec3_set(&v2, hw, 0, -qd);
    pp_vec3_set(&v3, hw, h, -qd);
    pp_vec3_set(&v4, qw, h, -hd);

    pp_physics_create_triangle(&v1, &v2, &v3, ENV_WALL_KIND);
    pp_physics_create_triangle(&v1, &v3, &v4, ENV_WALL_KIND);

    // Front left angled
    pp_vec3_set(&v1, -qw, 0, hd);
    pp_vec3_set(&v2, -hw, 0, qd);
    pp_vec3_set(&v3, -hw, h, qd);
    pp_vec3_set(&v4, -qw, h, hd);

    pp_physics_create_triangle(&v1, &v2, &v3, ENV_WALL_KIND);
    pp_physics_create_triangle(&v1, &v3, &v4, ENV_WALL_KIND);

    // Front right angled
    pp_vec3_set(&v1, qw, 0, hd);
    pp_vec3_set(&v2, qw, h, hd);
    pp_vec3_set(&v3, hw, h, qd);
    pp_vec3_set(&v4, hw, 0, qd);

    pp_physics_create_triangle(&v1, &v2, &v3, ENV_WALL_KIND);
    pp_physics_create_triangle(&v1, &v3, &v4, ENV_WALL_KIND);
}

bool dont_collide(const void *, const void *, BodyKind, BodyKind, const PPCollision *, const void *)
{
    return false;
}

bool ground_check(
    const void *lhs, const void *rhs, BodyKind k0, BodyKind, const PPCollision *, const void *)
{
    // Abuse the user data pointer to store the grounded flag
    if (k0 == CAR_INNER_KIND) {
        pp_body_set_user_data((PPBody *) lhs, (void *) 1);
    } else {
        pp_body_set_user_data((PPBody *) rhs, (void *) 1);
    }
    return true;
}

bool ball_car_collision(
    const void *lhs, const void *rhs, BodyKind k0, BodyKind k1, const PPCollision *c, const void *)
{
    PPSphere *ball = (k0 == BALL_KIND) ? (PPSphere *) lhs : (PPSphere *) rhs;
    PPBox *car = (k0 == CAR_BODY_KIND) ? (PPBox *) lhs : (PPBox *) rhs;
    PPSphere *car_ball_body = (PPSphere *) pp_body_get_user_data(PP_BODY(car));

    PPVec3 vel;
    pp_body_get_velocity_at_position(PP_BODY(car_ball_body), &c->p, &vel);
    pp_body_add_force(PP_BODY(ball), vel.x, vel.y, vel.z);
    return false;
}

void rotate_to_direction(PPBox *sphere, const PPVec3 *target_forward, float dt)
{
    // 720 degrees a second
    const float rot_rate_in_radians = (M_PI * 2.0f) * 2.0f;

    PPVec3 forward;
    pp_vec3_set(&forward, 0, 0, -1);

    PPQuaternion rotation;
    pp_quat_between(&forward, target_forward, &rotation);
    pp_quat_slerp(&sphere->body.rot, &rotation, dt, &sphere->body.rot);
}

void GameScene::on_load() {
    pp_physics_collision_map_add(CAR_BODY_KIND, CAR_INNER_KIND, NULL, &dont_collide);
    pp_physics_collision_map_add(CAR_INNER_KIND, BALL_KIND, NULL, &dont_collide);
    pp_physics_collision_map_add(CAR_BODY_KIND, ENV_FLOOR_KIND, NULL, &dont_collide);
    pp_physics_collision_map_add(CAR_INNER_KIND, ENV_FLOOR_KIND, NULL, &ground_check);
    pp_physics_collision_map_add(CAR_BODY_KIND, BALL_KIND, NULL, &ball_car_collision);

    auto car_mesh2 = assets->load_mesh("assets/car/sedan-sports.obj");
    float cs = 1.0f / car_mesh2->aabb().max_dimension();
    car_mesh2->transform_vertices(smlt::Mat4::as_scale(smlt::Vec3(cs, cs, cs)));

    PPVec3 pos;
    pp_vec3_set(&pos, 0, 2, 0);
    ball_.body = pp_physics_create_sphere(0.5f, &pos, 0.1f, BALL_KIND);
    // pp_body_set_friction(PP_BODY(ball_.body), 0.9f);

    pp_vec3_set(&pos, 1.0f, 4, 0);
    cars_[0].body = pp_physics_create_box(0.5f, 0.5f, 1.0f, &pos, 1.0, CAR_BODY_KIND);
    cars_[0].roll_body = pp_physics_create_sphere(0.5f,
                                                  &pos,
                                                  car_mesh2->aabb().height() / 2,
                                                  CAR_INNER_KIND);
    pp_body_set_user_data(PP_BODY(cars_[0].body), cars_[0].roll_body);

    pp_body_lock_axis(PP_BODY(cars_[0].body), PP_AXIS_LOCK_PITCH_AND_ROLL);
    pp_body_set_angular_damping(PP_BODY(cars_[0].body), 0.95f);

    auto tex = assets->load_texture("assets/sand.png");
    auto floor_mat = assets->load_material(smlt::Material::BuiltIns::TEXTURE_ONLY);
    floor_mat->set_cull_mode(smlt::CULL_MODE_NONE);
    floor_mat->set_blend_func(smlt::BLEND_ALPHA);
    floor_mat->set_base_color_map(tex);

    auto car_mesh1 = assets->create_mesh(smlt::VertexSpecification::POSITION_AND_DIFFUSE);
    car_mesh1->create_submesh_as_sphere("roll_body",
                                        floor_mat,
                                        car_mesh2->aabb().height() / 2,
                                        10,
                                        10);

    cars_[0].roll_body_actor = create_child<smlt::Actor>(car_mesh1);
    cars_[0].body_actor = create_child<smlt::Actor>(car_mesh2);

    // pp_body_set_bounce(PP_BODY(ball_.body), 0.1f);
    // pp_body_set_damping(PP_BODY(ball_.body), 0.001f);
    // pp_body_set_bounce(PP_BODY(cars_[0].body), 0.1f);
    // pp_body_set_bounce(PP_BODY(cars_[0].roll_body), 0.1f);

    define_stadium();

    PPVec3 grv;
    pp_vec3_init(&grv);
    grv.xyz[1] = -9.8f;
    pp_physics_set_gravity(&grv);

    camera_ = create_child<smlt::Camera3D>();
    camera_->set_perspective_projection(smlt::Degrees(60.0f), window->aspect_ratio());
    camera_->transform->set_position(smlt::Vec3(0, 10, 8));
    camera_->transform->look_at(smlt::Vec3());

    auto mesh = assets->load_mesh("assets/ball/mesh.obj");
    auto s = 1.0f / mesh->aabb().max_dimension();
    mesh->transform_vertices(smlt::Mat4::as_scale(smlt::Vec3(s, s, s)));

    ball_.actor = create_child<smlt::Actor>(mesh);

    smlt::MeshPtr floor_mesh = assets->create_mesh(smlt::VertexSpecification::POSITION_AND_DIFFUSE);

    auto submesh = floor_mesh->create_submesh("floor", floor_mat);
    for (std::size_t i = 0; i < pp_physics_triangle_count(); ++i) {
        auto j = floor_mesh->vertex_data->count();

        const PPTriangle *t = pp_physics_triangle_at(i);
        for(int k = 0; k < 3; ++k) {
            floor_mesh->vertex_data->position(t->v[k].xyz[0], t->v[k].xyz[1], t->v[k].xyz[2]);
            floor_mesh->vertex_data->color(smlt::Color(1.0f, 0.0f, 0.0f, 0.1f));
            floor_mesh->vertex_data->move_next();
        }

        submesh->add_vertex_range(j, 3);
    }

    floor_mesh->vertex_data->done();

    floor_ = create_child<smlt::Actor>(floor_mesh);

    auto layer = compositor->create_layer(this, camera_);
    layer->viewport->set_color(smlt::Color::gray());
    layer->set_clear_flags(smlt::BUFFER_CLEAR_ALL);
}

void GameScene::on_fixed_update(float step)
{
    // Set that we're not grounded
    pp_body_set_user_data(PP_BODY(cars_[0].roll_body), (void *) 0);
    pp_physics_step(step, 8, 3);

    // Try to zero out sideways forces to prevent the car drifting

    PPVec3 forward, right;
    pp_body_get_forward(PP_BODY(cars_[0].body), &forward);
    pp_body_get_right(PP_BODY(cars_[0].body), &right);

    PPVec3 roll_velocity;
    pp_body_get_velocity(PP_BODY(cars_[0].roll_body), &roll_velocity);

    float side_vel = pp_vec3_dot(&roll_velocity, &right);

    PPVec3 force;
    pp_vec3_scale(&right, -side_vel * pp_body_get_mass(PP_BODY(cars_[0].roll_body)), &force);
    pp_body_add_force(PP_BODY(cars_[0].roll_body), force.x, force.y, force.z);

    // The inner ball rolls on the floor
    // The body collides with walls, balls, and cars
    // The body needs to be positioned above the inner ball (which is smaller)

    PPVec3 inner_pos;
    pp_body_get_position(PP_BODY(cars_[0].roll_body), &inner_pos);
    float br = pp_box_get_height(cars_[0].body);
    float ir = pp_sphere_get_radius(cars_[0].roll_body);

    // Align the car body with the inner roll_body
    pp_body_set_position(PP_BODY(cars_[0].body),
                         inner_pos.xyz[0],
                         inner_pos.xyz[1] + (br - ir),
                         inner_pos.xyz[2]);
}

void GameScene::on_update(float dt) {
    auto p = cars_[0].body->body.pos;
    auto q = cars_[0].body->body.rot;
    cars_[0].body_actor->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));
    cars_[0].body_actor->transform->set_orientation(
        smlt::Quaternion(q.xyzw[0], q.xyzw[1], q.xyzw[2], q.xyzw[3]));

    p = cars_[0].roll_body->body.pos;
    q = cars_[0].roll_body->body.rot;
    cars_[0].roll_body_actor->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));
    cars_[0].roll_body_actor->transform->set_orientation(
        smlt::Quaternion(q.xyzw[0], q.xyzw[1], q.xyzw[2], q.xyzw[3]));

    p = ball_.body->body.pos;
    q = ball_.body->body.rot;
    ball_.actor->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));
    ball_.actor->transform->set_orientation(
        smlt::Quaternion(q.xyzw[0], q.xyzw[1], q.xyzw[2], q.xyzw[3]));

    PPVec3 drive_force;
    pp_vec3_set(&drive_force, -input->axis_value("Horizontal"), 0.0f, input->axis_value("Vertical"));
    pp_vec3_normalize(&drive_force);

    float thrust = pp_vec3_length(&drive_force);

    if (fabs(thrust) > 0.0001f) {
        rotate_to_direction(cars_[0].body, &drive_force, smlt::clamp(5.0f * dt, 0.0f, 1.0f));
    }

    PPVec3 f;
    pp_body_get_forward(PP_BODY(cars_[0].body), &f);

    bool grounded = (bool) pp_body_get_user_data(PP_BODY(cars_[0].roll_body));
    if (grounded) {
        pp_vec3_scale(&f, thrust * 5.0f, &f);
        pp_body_add_force(PP_BODY(cars_[0].roll_body), f.xyz[0], f.xyz[1], f.xyz[2]);

        if (input->axis_was_pressed("Fire1") && grounded) {
            pp_body_add_force(PP_BODY(cars_[0].roll_body), 0, 100.0f, 0);
        }
    } else {
        pp_vec3_scale(&f, thrust * 0.5f, &f);
        pp_body_add_force(PP_BODY(cars_[0].roll_body), f.xyz[0], f.xyz[1], f.xyz[2]);
    }

    camera_->transform->look_at(cars_[0].body_actor->transform->position(), smlt::Vec3::up());
    camera_->transform->set_position(cars_[0].body_actor->transform->position()
                                     + smlt::Vec3(0, 15, 10));
}

void GameScene::on_activate() {
    // Called when the scene is made active (after load)
}

void GameScene::on_deactivate() {
    // Called before unloading
}

void GameScene::on_unload() {
    // Cleanup your scene here
}
