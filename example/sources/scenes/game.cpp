
#include "game.h"

#define PHYSICS_IMPLEMENTATION
#include "../physics.h"

void GameScene::on_load() {
    Vec3 pos;
    vec3_set(&pos, 0, 2, 0);

    cars_[0].body = physics_create_sphere(1.0f, &pos, 1.0, 0);
    sphere_set_bounce(cars_[0].body, 1.0f);

    Vec3 v1, v2, v3;
    vec3_set(&v1, -10, 0, -10);
    vec3_set(&v2, 0, 0, 10);
    vec3_set(&v3, 10, 0, -10);

    physics_create_triangle(&v1, &v2, &v3, 0);

    Vec3 grv;
    grv.xyz[1] = -0.1f;
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

void GameScene::on_update(float dt) {
    physics_step(dt);

    auto p = cars_[0].body->body.pos;
    actor_->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));

    if(input->axis_was_pressed("Fire1")) {
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
