
#include "game.h"

#define PHYSICS_IMPLEMENTATION
#include "../physics.h"

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
    auto f1 = physics_create_triangle(&v1, &v2, &v3, 0);
    auto f2 = physics_create_triangle(&v1, &v3, &v4, 0);

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

    auto lw1 = physics_create_triangle(&v1, &v2, &v3, 0);
    auto lw2 = physics_create_triangle(&v1, &v3, &v4, 0);
    assert(lw1->n.xyz[0] > 0.0f);
    assert(lw2->n.xyz[0] > 0.0f);

    vec3_set(&v1, hw, 0, -hd);
    vec3_set(&v2, hw, 0, hd);
    vec3_set(&v3, hw, h, hd);
    vec3_set(&v4, hw, h, -hd);

    auto rw1 = physics_create_triangle(&v1, &v2, &v3, 0);
    auto rw2 = physics_create_triangle(&v1, &v3, &v4, 0);
    assert(rw1->n.xyz[0] < 0.0f);
    assert(rw2->n.xyz[0] < 0.0f);
}

void GameScene::on_load() {
    Vec3 pos;
    vec3_set(&pos, 0, 2, 0);

    cars_[0].body = physics_create_sphere(1.0f, &pos, 1.0, 0);
    sphere_set_bounce(cars_[0].body, 1.0f);

    define_stadium();

    Vec3 grv;
    grv.xyz[1] = -9.8f;
    physics_set_gravity(&grv);

    camera_ = create_child<smlt::Camera3D>();
    camera_->transform->set_position(smlt::Vec3(0, 0.5f, 5));
    camera_->set_perspective_projection(smlt::Degrees(60.0f), window->aspect_ratio());

    auto mesh = assets->load_mesh("assets/ball/mesh.obj");
    auto s = 1.0f / mesh->aabb().max_dimension();
    mesh->transform_vertices(smlt::Mat4::as_scale(smlt::Vec3(s, s, s)));

    actor_ = create_child<smlt::Actor>(mesh);

    smlt::MeshPtr floor_mesh = assets->create_mesh(smlt::VertexSpecification::POSITION_AND_DIFFUSE);
    auto floor_mat = assets->clone_default_material();
    floor_mat->set_cull_mode(smlt::CULL_MODE_NONE);

    auto submesh = floor_mesh->create_submesh("floor", floor_mat);
    for(std::size_t i = 0; i < physics_triangle_count(); ++i) {
        auto j = floor_mesh->vertex_data->count();

        const Triangle* t = physics_triangle_at(i);
        for(int k = 0; k < 3; ++k) {
            floor_mesh->vertex_data->position(t->v[k].xyz[0], t->v[k].xyz[1], t->v[k].xyz[2]);
            floor_mesh->vertex_data->color(smlt::Color::white());
            floor_mesh->vertex_data->move_next();
        }

        submesh->add_vertex_range(j, 3);
    }

    floor_mesh->vertex_data->done();

    compositor->create_layer(this, camera_);
}

void GameScene::on_fixed_update(float step)
{
    physics_step(step);
}

void GameScene::on_update(float dt) {
    auto p = cars_[0].body->body.pos;
    actor_->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));

    if (input->axis_was_pressed("Fire1")) {
        cars_[0].body->body.pos.xyz[1] = 5.0f;
    }
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
