
#include "game.h"

#define PHYSICS_IMPLEMENTATION
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

bool dont_collide(
    const void *, const void *, BodyKind, BodyKind, const PPCollision *c, const void *self)
{
    GameScene *scene = (GameScene *) self;

    smlt::Vec3 point(c->p.x, c->p.y, c->p.z);
    smlt::Vec3 normal(c->n.x, c->n.y, c->n.z);
    scene->debug_->draw_point(point, smlt::Color::red());
    scene->debug_->draw_line(point, point + normal, smlt::Color::green());
    return true;
}

void GameScene::on_load() {
    pp_physics_collision_map_add(BOX_KIND, BALL_KIND, (void *) this, &dont_collide);

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

    debug_ = create_child<smlt::Debug>();
    debug_->set_line_width(0.05f);
    debug_->set_point_size(0.25f);
}

void GameScene::on_fixed_update(float step)
{
    pp_physics_step(step);
}

void GameScene::on_update(float dt) {
    auto p = box_.body->body.pos;
    auto q = box_.body->body.rot;
    box_.actor->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));
    box_.actor->transform->set_orientation(
        smlt::Quaternion(q.xyzw[0], q.xyzw[1], q.xyzw[2], q.xyzw[3]));

    p = ball_.body->body.pos;
    q = ball_.body->body.rot;
    ball_.actor->transform->set_position(smlt::Vec3(p.xyz[0], p.xyz[1], p.xyz[2]));
    ball_.actor->transform->set_orientation(
        smlt::Quaternion(q.xyzw[0], q.xyzw[1], q.xyzw[2], q.xyzw[3]));

    auto target = (this->current_object_ == 0) ? ball_.actor : box_.actor;
    camera_->transform->look_at(target->transform->position(), smlt::Vec3::up());
    // camera_->transform->set_position(target->transform->position() + smlt::Vec3(0, 0, 10));

    PPVec3 force = {.x = -input->axis_value("Horizontal"),
                    .y = input->axis_value("Vertical"),
                    .z = 0.0f};

    pp_body_add_force(current_object_ == 0 ? PP_BODY(ball_.body) : PP_BODY(box_.body),
                      force.x,
                      force.y,
                      force.z);

    if (input->axis_was_pressed("Tab")) {
        current_object_ = (current_object_ + 1) % 2;
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
