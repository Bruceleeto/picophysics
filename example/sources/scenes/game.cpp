
#include "game.h"

#define PHYSICS_IMPLEMENTATION
#include "../physics.h"

#define CAR_BODY_KIND 1
#define CAR_SHELL_KIND 2
#define BALL_KIND 3
#define ENV_KIND 0

void define_stadium()
{
    /* This is a recreation of the DS64 SDF
     * field using triangles. Later this can be
     * extended to improve the curve */

    // Floor
    Vec3 v1, v2, v3, v4;
    vec3_set(&v1, -50, 0, -50);
    vec3_set(&v2, -50, 0, 50);
    vec3_set(&v3, 50, 0, 50);
    vec3_set(&v4, 50, 0, -50);
    auto f1 = physics_create_triangle(&v1, &v2, &v3, ENV_KIND);
    auto f2 = physics_create_triangle(&v1, &v3, &v4, ENV_KIND);

    assert(f1->n.xyz[1] > 0.0f);
    assert(f2->n.xyz[1] > 0.0f);

    const float w = 40.0f;
    const float d = 26.0f;
    const float hw = w * 0.5f;
    const float hd = d * 0.5f;
    const float h = 40.0f;

    // Left wall
    vec3_set(&v1, -hw, 0, hd);
    vec3_set(&v2, -hw, 0, -hd);
    vec3_set(&v3, -hw, h, -hd);
    vec3_set(&v4, -hw, h, hd);

    auto lw1 = physics_create_triangle(&v1, &v2, &v3, ENV_KIND);
    auto lw2 = physics_create_triangle(&v1, &v3, &v4, ENV_KIND);
    assert(lw1->n.xyz[0] > 0.0f);
    assert(lw2->n.xyz[0] > 0.0f);

    vec3_set(&v1, hw, 0, -hd);
    vec3_set(&v2, hw, 0, hd);
    vec3_set(&v3, hw, h, hd);
    vec3_set(&v4, hw, h, -hd);

    auto rw1 = physics_create_triangle(&v1, &v2, &v3, ENV_KIND);
    auto rw2 = physics_create_triangle(&v1, &v3, &v4, ENV_KIND);
    assert(rw1->n.xyz[0] < 0.0f);
    assert(rw2->n.xyz[0] < 0.0f);
}

/* Collision between a body and a shell should be ignored */
bool car_body_car_shell(const void *, const void *, BodyKind, BodyKind)
{
    return false;
}

/* Car shell should only collide with the environment, not other cars
 * or balls */
bool car_shell_ball(const void *, const void *, BodyKind, BodyKind)
{
    return false;
}

bool car_body_env(const void *, const void *, BodyKind, BodyKind)
{
    return false;
}

void GameScene::on_load() {
    collision_map_add(CAR_BODY_KIND, CAR_SHELL_KIND, &car_body_car_shell);
    collision_map_add(CAR_SHELL_KIND, BALL_KIND, &car_shell_ball);
    collision_map_add(CAR_BODY_KIND, ENV_KIND, &car_body_env);

    auto car_mesh2 = assets->load_mesh("assets/car/sedan-sports.obj");
    float cs = 1.0f / car_mesh2->aabb().max_dimension();
    car_mesh2->transform_vertices(smlt::Mat4::as_scale(smlt::Vec3(cs, cs, cs)));

    Vec3 pos;
    vec3_set(&pos, 0, 2, 0);
    ball_.body = physics_create_sphere(0.5f, &pos, 1.0, BALL_KIND);

    vec3_set(&pos, 1.0f, 4, 0);
    cars_[0].body = physics_create_sphere(0.5f, &pos, 1.0, CAR_BODY_KIND);
    cars_[0].shell = physics_create_sphere(0.5f,
                                           &pos,
                                           car_mesh2->aabb().height() / 2,
                                           CAR_SHELL_KIND);
    sphere_lock_axis(cars_[0].body, AXIS_LOCK_PITCH_AND_ROLL);
    sphere_set_angular_damping(cars_[0].body, 0.3f);

    auto tex = assets->load_texture("assets/sand.png");
    auto floor_mat = assets->load_material(smlt::Material::BuiltIns::TEXTURE_ONLY);
    floor_mat->set_cull_mode(smlt::CULL_MODE_NONE);
    floor_mat->set_blend_func(smlt::BLEND_ALPHA);
    floor_mat->set_base_color_map(tex);

    auto car_mesh1 = assets->create_mesh(smlt::VertexSpecification::POSITION_AND_DIFFUSE);
    car_mesh1->create_submesh_as_sphere("shell", floor_mat, car_mesh2->aabb().height() / 2, 10, 10);

    cars_[0].shell_actor = create_child<smlt::Actor>(car_mesh1);
    cars_[0].body_actor = create_child<smlt::Actor>(car_mesh2);

    sphere_set_bounce(ball_.body, 0.3f);
    sphere_set_bounce(cars_[0].body, 0.1f);
    sphere_set_bounce(cars_[0].shell, 0.1f);

    define_stadium();

    Vec3 grv;
    vec3_init(&grv);
    grv.xyz[1] = -9.8f;
    physics_set_gravity(&grv);

    camera_ = create_child<smlt::Camera3D>();
    camera_->transform->set_position(smlt::Vec3(0, 5, 10));
    camera_->transform->look_at(smlt::Vec3());
    camera_->set_perspective_projection(smlt::Degrees(60.0f), window->aspect_ratio());

    auto mesh = assets->load_mesh("assets/ball/mesh.obj");
    auto s = 1.0f / mesh->aabb().max_dimension();
    mesh->transform_vertices(smlt::Mat4::as_scale(smlt::Vec3(s, s, s)));

    ball_.actor = create_child<smlt::Actor>(mesh);

    smlt::MeshPtr floor_mesh = assets->create_mesh(smlt::VertexSpecification::POSITION_AND_DIFFUSE);

    auto submesh = floor_mesh->create_submesh("floor", floor_mat);
    for(std::size_t i = 0; i < physics_triangle_count(); ++i) {
        auto j = floor_mesh->vertex_data->count();

        const Triangle* t = physics_triangle_at(i);
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
    physics_step(step);

    Vec3 pos;
    sphere_get_position(cars_[0].shell, &pos);
    sphere_set_position(cars_[0].body, pos.xyz[0], pos.xyz[1], pos.xyz[2]);
}

void GameScene::on_update(float dt) {
    auto p = cars_[0].body->body.pos;
    auto q = cars_[0].body->body.rot;
    cars_[0].body_actor->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));
    cars_[0].body_actor->transform->set_orientation(
        smlt::Quaternion(q.xyzw[0], q.xyzw[1], q.xyzw[2], q.xyzw[3]));

    p = cars_[0].shell->body.pos;
    q = cars_[0].shell->body.rot;
    cars_[0].shell_actor->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));
    cars_[0].shell_actor->transform->set_orientation(
        smlt::Quaternion(q.xyzw[0], q.xyzw[1], q.xyzw[2], q.xyzw[3]));

    p = ball_.body->body.pos;
    q = ball_.body->body.rot;
    ball_.actor->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));
    ball_.actor->transform->set_orientation(
        smlt::Quaternion(q.xyzw[0], q.xyzw[1], q.xyzw[2], q.xyzw[3]));

    sphere_add_angular_force(cars_[0].body, 0, -10.0f * input->axis_value("Horizontal"), 0);
    Vec3 f;
    sphere_get_forward(cars_[0].body, &f);
    vec3_scale(&f, 20.0f * input->axis_value("Vertical"), &f);
    sphere_add_force(cars_[0].shell, f.xyz[0], f.xyz[1], f.xyz[2]);
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
