
#include "game.h"

#define PHYSICS_IMPLEMENTATION
#include "../physics.h"

void GameScene::on_load() {
    Vec3 pos;
    vec3_init(&pos);

    cars_[0].body = physics_create_sphere(1.0f, &pos, 1.0, 0);

    Vec3 v1, v2, v3;
    vec3_set(&v1, -10, 0, -10);
    vec3_set(&v2, 0, 0, 10);
    vec3_set(&v3, 10, 0, -10);

    physics_create_triangle(&v1, &v2, &v3, 0);

    auto camera = create_child<smlt::Camera3D>();
    actor_ = create_child<smlt::Actor>();

    compositor->create_layer(this, camera);
}

void GameScene::on_update(float dt) {
    physics_step(dt);

    auto p = cars_[0].body->body.pos;
    actor_->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));
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
