#pragma once

#include <simulant/simulant.h>

#include "../physics.h"


typedef struct _Car {
    Sphere* body;
    Sphere *shell;

    smlt::ActorPtr body_actor;
    smlt::ActorPtr shell_actor;
} Car;

typedef struct _Ball
{
    Sphere *body;
    smlt::ActorPtr actor;
} Ball;

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

    Car cars_[4];
    Ball ball_;

    smlt::ActorPtr actor_;
    smlt::ActorPtr floor_;
    smlt::Camera3D *camera_;
};
