
#include "game.h"

#define PICOPHYSICS_IMPLEMENTATION
#include "../physics.h"

#define BOX_KIND 1
#define BALL_KIND 2

/* These are different things because we want the bigger
 * car sphere (body) to collide with walls, but if it collides
 * with the floor the car will float in the air. The smaller sphere
 * (inner) collides with the floor only */
#define ENV_FLOOR_KIND 4
#define ENV_WALL_KIND 5

void define_stadium()
{
    // Create a box at the default position with no mass
    pp_physics_create_box(100f, 1.0f, 100.0f, NULL, 0.0f, ENV_FLOOR_KIND);
}

bool visualise(const void *, const void *, BodyKind, BodyKind, const PPCollision *c, const void *self)
{
    GameScene *scene = (GameScene *) self;

    smlt::Vec3 point(c->p.x, c->p.y, c->p.z);
    smlt::Vec3 normal(c->n.x, c->n.y, c->n.z);
    scene->debug_->draw_point(point, smlt::Color::red());
    scene->debug_->draw_line(point, point + normal, smlt::Color::green());
    return true;
}

void GameScene::on_load() {
    pp_physics_collision_map_add(BOX_KIND, BALL_KIND, (void *) this, &visualise);

    auto axis = input->new_axis("Tab");
    axis->set_positive_keyboard_key(smlt::KEYBOARD_CODE_TAB);

    camera_ = create_child<smlt::Camera3D>();
    camera_->set_perspective_projection(smlt::Degrees(60.0f), window->aspect_ratio());
    camera_->transform->set_position(smlt::Vec3(0, 0, -10));

    auto tex = assets->load_texture("assets/sand.png");
    auto floor_mat = assets->load_material(smlt::Material::BuiltIns::TEXTURE_ONLY);
    floor_mat->set_cull_mode(smlt::CULL_MODE_NONE);
    floor_mat->set_blend_func(smlt::BLEND_ALPHA);
    floor_mat->set_base_color_map(tex);

    define_stadium();

    auto mesh = assets->load_mesh("assets/ball/mesh.obj");
    auto s = 1.0f / mesh->aabb().max_dimension();
    mesh->transform_vertices(smlt::Mat4::as_scale(smlt::Vec3(s, s, s)));
    ball_.actor = create_child<smlt::Actor>(mesh);

    auto box_mesh = assets->load_mesh("assets/box/cube.obj");
    box_.actor = create_child<smlt::Actor>(box_mesh);
    s = 1.0f / box_mesh->aabb().max_dimension();
    box_mesh->transform_vertices(smlt::Mat4::as_scale(smlt::Vec3(s, s, s)));

    PPVec3 pos;
    pp_vec3_set(&pos, 5.0f, 4, 0);
    ball_.body = pp_physics_create_sphere(mesh->aabb().max_dimension() * 0.5f,
                                          &pos,
                                          0.01f,
                                          BALL_KIND);

    pp_vec3_set(&pos, -5.0f, 4, 0);
    box_.body = pp_physics_create_box(mesh->aabb().width(),
                                      mesh->aabb().height(),
                                      mesh->aabb().depth(),
                                      &pos,
                                      1.0,
                                      BOX_KIND);


    floor_ = create_child<smlt::Actor>(floor_mesh);

    auto layer = compositor->create_layer(this, camera_);
    layer->viewport->set_color(smlt::Color::gray());
    layer->set_clear_flags(smlt::BUFFER_CLEAR_ALL);

    debug_ = create_child<smlt::Debug>();
    debug_->set_line_width(0.05f);
    debug_->set_point_size(0.25f);
}

void GameScene::on_fixed_update(float step)
{
    pp_physics_step(step);
}

void GameScene::on_update(float dt) {
    auto &rgen = smlt::RandomGenerator::instance();
    static float time_to_next_shape = 0.0f;
    time_to_next_shape -= dt;
    if (time_to_next_shape_ <= 0.0f) {
        PPVec3 pos;
        pp_vec3_set(&pos, rgen.float_in_range(-8.0f, 8.0f), 10.0f, rgen.float_in_range(-8.0f, 8.0f));

        int which = rgen.int_in_range(0, 1);
        if (which == 0) {
            pp_physics_create_sphere(0.5f, &pos, 1.0f, BALL_KIND);
        } else {
            pp_physics_create_box(1.0f, 1.0f, 1.0f, &pos, 1.0f, BOX_KIND);
        }

        time_to_next_shape_ = 1.0f;
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
