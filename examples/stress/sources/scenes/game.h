#pragma once

#include <simulant/simulant.h>

#include "../physics.h"

typedef struct _Shape
{
    PPBody *body;
    smlt::ActorPtr actor;
} Shape;

class GameScene : public smlt::Scene {
public:
    // Boilerplate
    GameScene(smlt::Window* window):
        smlt::Scene(window) {}

    void on_load();
    void on_update(float dt);
    void on_fixed_update(float step);
    void on_activate();
    void on_deactivate();
    void on_unload();

    int shape_count_ = 0;
    Shape shapes_[100];

    smlt::Debug *debug_;
    smlt::Camera3D *camera_;
};
