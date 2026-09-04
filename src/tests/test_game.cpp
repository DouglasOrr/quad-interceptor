#include <doctest/doctest.h>

#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>

#include "game.hpp"
#include "platform.hpp"

namespace {
std::string temp_dir() {
    static uint counter = 0;
    std::string dir =
        (std::filesystem::temp_directory_path() /
         ("quadinterceptor_test" + std::to_string(::getpid()) + "_" + std::to_string(++counter)))
            .string();
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}
}  // namespace

TEST_CASE("game loop runs for a few frames") {
    Window w({
        .title = "Quad Interceptor (Test)",
        .mode = WindowMode::Windowed,
        .width = 1000,
        .height = 600,
        .msaa = 1,
        .vsync = true,
        .profile_path = "",
    });
    Renderer renderer;
    Settings settings = Settings::defaults();
    for (auto& mission : Mission::all()) {
        mission.test_player_ai = true;
        mission.test_time_limit = 0.25f;
        game_loop(w, renderer, settings, mission);
    }
    for (auto& drone_type : DroneType::all()) {
        auto mission = Mission::all().front();
        mission.player_type = drone_type;
        mission.test_player_ai = true;
        mission.test_time_limit = 0.25f;
        game_loop(w, renderer, settings, mission);
    }
}

TEST_CASE("AI mean time between crashes on city terrain") {
    constexpr float sim_seconds = 5 * 60.0f;
    const uint64 tick_limit = uint64(sim_seconds / Physics::dt);
    for (auto mission : Mission::all()) {
        if (mission.terrain_type.kind != TerrainType::Kind::City) continue;

        GameState s{};
        s.mission = mission;
        s.mission.test_enemy_crashes = true;
        s.pregame = false;
        s.lives = mission.starting_lives;
        s.terrain.type = mission.terrain_type;
        s.terrain.seed = static_cast<int64>(hash(mission.seed, uint64_t{0xf738a04c}));
        GameManager game_manager;
        AI ai;
        Physics physics;
        game_manager.update(s);

        // Count player + enemy AI crashes (status transitions into Crashed), across respawns.
        std::unordered_map<uint, Drone::Status> prev_status;
        uint64 crash_count = 0;
        while (s.tick < tick_limit) {
            ai.update(s);
            physics.tick(s);
            for (const auto& d : s.drones) {
                auto it = prev_status.find(d.id);
                Drone::Status prev = (it == prev_status.end()) ? Drone::Status::Alive : it->second;
                if (d.kind == Drone::Kind::Enemy && d.status == Drone::Status::Crashed &&
                    prev != Drone::Status::Crashed) {
                    ++crash_count;
                }
                prev_status[d.id] = d.status;
            }
            game_manager.update(s);
        }

        const float sim_time = float(s.tick) * Physics::dt;
        const float mtb = crash_count > 0 ? sim_time / float(crash_count) : sim_time;
        print("#", mission.name, "( tilt", mission.enemy_tilt, ")");
        print("  crashes:", crash_count);
        print("  mean time between crashes:", mtb, "s (", sim_time, "s )");
    }
}

TEST_CASE("shell_loop runs") {
    Window w({
        .title = "Test (Quad Interceptor)",
        .mode = WindowMode::Windowed,
        .width = 1000,
        .height = 600,
        .msaa = 1,
        .vsync = true,
        .profile_path = "",
    });
    const std::string dir = temp_dir();
    setenv("QUAD_INTERCEPTOR_CONFIG_DIR", dir.c_str(), 1);
    Settings settings = Settings::defaults();
    w.test_press_after(Key::esc, 0.25f);

    shell_loop(w, settings);
    unsetenv("QUAD_INTERCEPTOR_CONFIG_DIR");
}

TEST_CASE("settings round-trip") {
    const std::string dir = temp_dir();
    Settings s = Settings::defaults();
    s.fullscreen = true;
    s.msaa = 4;
    s.vsync = false;
    s.show_fps = true;
    s.window_width = 1600;
    s.window_height = 900;
    s.master_volume = 0.33f;
    s.music_volume = 0.75f;
    s.effects_volume = 0.55f;
    s.bindings[size_t(Action::LiftUp)] = SDL_SCANCODE_SPACE;
    s.save(dir);

    Settings t = Settings::defaults();
    CHECK(t.load(dir) == Settings::LoadResult::OK);
    CHECK(t.fullscreen == s.fullscreen);
    CHECK(t.msaa == s.msaa);
    CHECK(t.vsync == s.vsync);
    CHECK(t.show_fps == s.show_fps);
    CHECK(t.window_width == s.window_width);
    CHECK(t.window_height == s.window_height);
    CHECK(t.master_volume == doctest::Approx(s.master_volume));
    CHECK(t.music_volume == doctest::Approx(s.music_volume));
    CHECK(t.effects_volume == doctest::Approx(s.effects_volume));
    CHECK(t.bindings == s.bindings);
}

TEST_CASE("settings defaults on missing or corrupt file") {
    const std::string dir = temp_dir();
    Settings defaults = Settings::defaults();
    Settings missing = defaults;
    CHECK(missing.load(dir) == Settings::LoadResult::NoFile);

    std::ofstream out(dir + "/settings.cfg");
    out << "fullscreen=maybe\n"
           "msaa=abc\n"
           "vsync=banana\n"
           "show_fps=banana\n"
           "master_volume=3\n"
           "bind.roll_left=-5\n"
           "bind.unknown=1\n"
           "unexpected=1\n"
           "bind.yaw_left=12\n";  // a valid line
    out.close();
    Settings corrupt = defaults;
    CHECK(corrupt.load(dir) == Settings::LoadResult::Error);
    CHECK(corrupt.fullscreen == defaults.fullscreen);
    CHECK(corrupt.msaa == defaults.msaa);
    CHECK(corrupt.vsync == defaults.vsync);
    CHECK(corrupt.master_volume == doctest::Approx(1.0f));  // clamped
    CHECK(corrupt.bindings[size_t(Action::RollLeft)] ==
          defaults.bindings[size_t(Action::RollLeft)]);      // invalid scancode
    CHECK(corrupt.bindings[size_t(Action::YawLeft)] == 12);  // valid line read
}
