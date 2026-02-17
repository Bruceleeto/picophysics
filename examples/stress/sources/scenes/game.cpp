
#include "game.h"

#define PICOPHYSICS_IMPLEMENTATION
#include <picophysics.h>

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
    float height = -2.0f;
    PPVec3 p0 = {-100.0f, height, -100.0f};
    PPVec3 p1 = {0.0f, height, 100.0f};
    PPVec3 p2 = {100.0f, height, -100.0f};
    pp_physics_create_triangle(&p0, &p1, &p2, ENV_FLOOR_KIND);
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
    PPVec3 grv = {0.0f, -9.8f, 0.0f};
    pp_physics_set_gravity(&grv);

    auto axis = input->new_axis("Tab");
    axis->set_positive_keyboard_key(smlt::KEYBOARD_CODE_TAB);

    camera_ = create_child<smlt::Camera3D>();
    camera_->set_perspective_projection(smlt::Degrees(60.0f), window->aspect_ratio());
    camera_->transform->set_position(smlt::Vec3(0, 0, -10));
    camera_->transform->look_at(smlt::Vec3());

    auto tex = assets->load_texture("assets/sand.png");
    auto floor_mat = assets->load_material(smlt::Material::BuiltIns::TEXTURE_ONLY);
    floor_mat->set_cull_mode(smlt::CULL_MODE_NONE);
    floor_mat->set_blend_func(smlt::BLEND_ALPHA);
    floor_mat->set_base_color_map(tex);

    define_stadium();

    auto mesh = assets->load_mesh("assets/ball/mesh.obj",
                                  smlt::VertexSpecification::DEFAULT,
                                  smlt::MeshLoadOptions(),
                                  smlt::GARBAGE_COLLECT_NEVER);
    mesh->set_name("Sphere");

    auto s = 1.0f / mesh->aabb().max_dimension();
    mesh->transform_vertices(smlt::Mat4::as_scale(smlt::Vec3(s, s, s)));

    auto box_mesh = assets->load_mesh("assets/box/cube.obj",
                                      smlt::VertexSpecification::DEFAULT,
                                      smlt::MeshLoadOptions(),
                                      smlt::GARBAGE_COLLECT_NEVER);
    box_mesh->set_name("Box");
    s = 1.0f / box_mesh->aabb().max_dimension();
    box_mesh->transform_vertices(smlt::Mat4::as_scale(smlt::Vec3(s, s, s)));

    auto layer = compositor->create_layer(this, camera_);
    layer->viewport->set_color(smlt::Color::gray());
    layer->set_clear_flags(smlt::BUFFER_CLEAR_ALL);

    debug_ = create_child<smlt::Debug>();
    debug_->set_line_width(0.05f);
    debug_->set_point_size(0.25f);

    auto sphere_axis = input->new_axis("Sphere");
    sphere_axis->set_positive_keyboard_key(smlt::KEYBOARD_CODE_S);

    auto box_axis = input->new_axis("Box");
    box_axis->set_positive_keyboard_key(smlt::KEYBOARD_CODE_B);
}

void GameScene::on_fixed_update(float step)
{
    pp_physics_step(step, 8, 3);
}

void GameScene::on_update(float dt) {
    if (input->axis_was_pressed("Sphere")) {
        Shape *s = &shapes_[shape_count_++];

        PPVec3 pos = {
            smlt::RandomGenerator::instance().float_in_range(-2.0f, 2.0f),
            1.0f,
            smlt::RandomGenerator::instance().float_in_range(-2.0f, 2.0f),
        };

        s->body = PP_BODY(pp_physics_create_sphere(0.5f, &pos, 1.0f, BALL_KIND));

        auto m = assets->find_mesh("Sphere");
        assert(m);
        s->actor = create_child<smlt::Actor>(m);
    }

    if (input->axis_was_pressed("Box")) {
        Shape *s = &shapes_[shape_count_++];
        s->body = PP_BODY(pp_physics_create_box(1.0f, 1.0f, 1.0f, NULL, 1.0f, BOX_KIND));

        auto m = assets->find_mesh("Box");
        assert(m);
        s->actor = create_child<smlt::Actor>(m);
    }

    for (int i = 0; i < shape_count_; ++i) {
        Shape *s = &shapes_[i];
        PPVec3 pos;
        PPQuaternion rot;
        pp_body_get_position(s->body, &pos);
        pp_body_get_rotation(s->body, &rot);

        s->actor->transform->set_position(smlt::Vec3(pos.x, pos.y, pos.z));
        s->actor->transform->set_orientation(smlt::Quaternion(rot.x, rot.y, rot.z, rot.w));
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
