#pragma once

#include <simulant/simulant.h>

#include <picophysics.h>

typedef struct _Box
{
    PPBox *body;
    smlt::ActorPtr actor;
} Box;

typedef struct _Ball
{
    PPSphere *body;
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

    Box box_;
    Ball ball_;
    int current_object_ = 0;
    bool response_enabled_ = false;

    smlt::Debug *debug_;

    smlt::ActorPtr floor_;
    smlt::Camera3D *camera_;
};
