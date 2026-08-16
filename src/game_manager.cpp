#include <utility>
#include <vector>

#include "game.hpp"
#include "platform.hpp"

////////////////////////////////////////////////////////////////////////////////
// Assets

// Serialized font blob, defined in the generated font_data.cpp.
std::span<const uint8_t> font_bytes();

Font Font::load() {
    Font font;
    auto data = font_bytes();
    const uint8_t n = data[0];
    font.width = data[1];
    font.height = data[2];
    const size_t n_pixels = font.width * font.height;
    gassert(n_pixels % 8 == 0);
    const size_t cell_bytes = n_pixels / 8;
    gassert(data.size() == 3 + n * (1 + cell_bytes));
    font.glyphs.reserve(n);
    for (uint8_t i = 0; i < n; ++i) {
        const size_t off = 3 + size_t(i) * (1 + cell_bytes);
        std::vector<uint8_t> px(n_pixels);
        for (size_t p = 0; p < n_pixels; ++p)
            px[p] = (data[off + 1 + p / 8] >> (7 - p % 8)) & 1 ? 255 : 0;
        font.glyphs.push_back({data[off], std::move(px)});
    }
    return font;
}

////////////////////////////////////////////////////////////////////////////////
// GameManager

namespace {
constexpr float respawn_delay = 2.0f;
constexpr float enemy_spawn_interval = 10.0f;
constexpr size_t max_enemy_drones = 3;
constexpr float enemy_spawn_radius = 3.0f * float(City::block_size) * float(City::cell_size);
constexpr Vec<3> cam_offset_vec{-1.2f, 0.0f, 0.5f};  // chase (behind, slightly up)
}  // namespace

void GameManager::update(GameState& state) {
    // Enemy cleanup
    std::erase_if(state.drones, [](const Drone& d) {
        return d.kind == Drone::Kind::Enemy && d.status != Drone::Status::Alive;
    });

    // Player respawn
    auto& player = player_drone(state);
    if (player.status != Drone::Status::Alive) {
        if (state.respawn_time == 0)
            state.respawn_time = state.tick + uint64(respawn_delay / Physics::dt);
        if (state.tick >= state.respawn_time) {
            state.respawn_time = 0;
            spawn_drone(state, Drone::Kind::Player, state.spawn);
        }
    } else {
        state.respawn_time = 0;
    }

    // Enemy respawn
    if (state.next_enemy_spawn_tick == 0) {
        state.next_enemy_spawn_tick = state.tick + uint64(enemy_spawn_interval / Physics::dt);
    } else if (state.tick >= state.next_enemy_spawn_tick) {
        size_t enemy_count = 0;
        foreach_drone(state, [&](size_t, const Drone& d) {
            if (d.kind == Drone::Kind::Enemy) ++enemy_count;
        });
        if (enemy_count < max_enemy_drones) {
            uint64 h = hash(state.city.seed, state.tick, state.next_drone_id);
            float angle = (h & 0xffffff) * (2.0f * pi / float(1 << 24));
            Vec<2> target_pos = {
                state.objective[0] + enemy_spawn_radius * std::cos(angle),
                state.objective[1] + enemy_spawn_radius * std::sin(angle),
            };
            Vec<2> spawn_xy =
                City::intersection_center(City::nearest_intersection_block(target_pos));
            spawn_drone(state, Drone::Kind::Enemy, {spawn_xy[0], spawn_xy[1], 20.0f});
            state.next_enemy_spawn_tick = 0;
        }
    }

    // Player camera update
    const auto& p = player_drone(state);
    state.camera_pos = p.position + rotate(p.orientation, cam_offset_vec);
    state.camera_orientation = p.orientation;
}

////////////////////////////////////////////////////////////////////////////////
// game_loop

namespace {
void copy_control(const Window& w, Vec<4>& c) {
    auto f = [](bool b) { return b ? 1.0f : 0.0f; };
    c = {};
    // Arrows
    c[0] += f(w.held(Key::right)) - f(w.held(Key::left));
    c[1] += f(w.held(Key::up)) - f(w.held(Key::down));
    // ZXC
    c[2] += 2 * f(w.held(Key::x)) - 1;
    c[3] += f(w.held(Key::c)) - f(w.held(Key::z));
}
}  // namespace

void game_loop(Window& window) {
    GameState state{};
    auto C = float(City::cell_size);
    state.objective = City::intersection_center(City::nearest_intersection_block({C, C}));
    state.objective_radius = C;
    state.lives = 5;
    state.spawn = {state.objective[0], state.objective[1], 5.0f};
    spawn_drone(state, Drone::Kind::Player, state.spawn);
    auto initial_state = state;
    GameManager game_manager;
    AI ai;
    Renderer renderer;
    AudioEngine audio_engine;
    window.set_audio_callback([&audio_engine](int16_t* stream, int num_frames) {
        audio_engine.fill_buffer(stream, num_frames);
    });
    Stopwatch stopwatch;
    float physics_time = 0.0f;
    while (window.frame()) {
        const float dt = stopwatch.lap();
        if (window.pressed(Key::hash)) {
            const uint next_drone_id = state.next_drone_id;
            state = initial_state;
            state.next_drone_id = next_drone_id;
            for (auto& d : state.drones)
                d.id = state.next_drone_id++;
            physics_time = 0.0f;
        }
        copy_control(window, player_drone(state).control_m);
        ai.update(state);
        physics_time = std::min(0.25f, physics_time + dt);
        while (physics_time >= Physics::dt) {
            Physics::tick(state);
            physics_time -= Physics::dt;
        }
        game_manager.update(state);
        renderer.draw(state, dt);
        audio_engine.update(state);
    }
}
