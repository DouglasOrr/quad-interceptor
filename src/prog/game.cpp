#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>

#include "game.hpp"
#include "platform.hpp"

int main(int argc, char** argv) {
    Window::init();

    const auto missions = Mission::all();
    std::optional<Mission> launch;
    std::optional<DroneType> player_type;
    std::optional<bool> player_ai;
    std::optional<float> time_limit;
    std::optional<uint64> seed;
    std::optional<WindowMode> window_mode;
    std::optional<bool> vsync;
    std::optional<uint> msaa;
    std::string profile_path;
    std::optional<std::string> config_dir_;

    for (int i = 1; i < argc; ++i) {
        // Parse (a leading -- is tolerated e.g. `vsync=true` or `--vsync=true`)
        std::string_view arg = argv[i];
        if (arg.starts_with("--")) arg.remove_prefix(2);
        const size_t eq = arg.find('=');
        std::string_view key = arg.substr(0, eq);
        std::string_view value =
            eq == std::string_view::npos ? std::string_view{} : arg.substr(eq + 1);

        // Process
        if (key == "mission") {
            auto found = std::find_if(missions.begin(), missions.end(),
                                      [value](auto& m) { return m.name == value; });
            if (found == missions.end()) {
                std::cerr << "Error! Unknown mission=" << value << std::endl;
                return 1;
            }
            launch = *found;
        } else if (key == "drone") {
            const auto drone_types = DroneType::all();
            auto found = std::find_if(drone_types.begin(), drone_types.end(),
                                      [value](auto& t) { return t.name == value; });
            if (found == drone_types.end()) {
                std::cerr << "Error! Unknown drone=" << value << std::endl;
                return 1;
            }
            player_type = *found;
        } else if (key == "window") {
            if (value == "full") {
                window_mode = WindowMode::Fullscreen;
            } else if (value == "medium") {
                window_mode = WindowMode::Windowed;
            } else {
                std::cerr << "Error! Unknown window=" << value << std::endl;
                return 1;
            }
        } else if (key == "player") {
            if (value == "ai") {
                player_ai = true;
            } else if (value == "user") {
                player_ai = false;
            } else {
                std::cerr << "Error! Unknown player=" << value << std::endl;
                return 1;
            }
        } else if (key == "timeout") {
            time_limit = std::atof(std::string(value).c_str());
        } else if (key == "profile") {
            if (value == "true") {
                profile_path = std::string(argv[0]) + ".profile";
            } else if (value == "false") {
                profile_path = "";
            } else {
                std::cerr << "Error! Unknown profile=" << value << std::endl;
                return 1;
            }
        } else if (key == "vsync") {
            if (value == "true") {
                vsync = true;
            } else if (value == "false") {
                vsync = false;
            } else {
                std::cerr << "Error! Unknown vsync=" << value << std::endl;
                return 1;
            }
        } else if (key == "msaa") {
            msaa = static_cast<uint>(std::atoi(std::string(value).c_str()));
        } else if (key == "seed") {
            seed = static_cast<uint64>(std::strtoull(std::string(value).c_str(), nullptr, 0));
        } else if (key == "config-dir") {
            if (value.empty()) {
                std::cerr << "Error! config-dir needs a path" << std::endl;
                return 1;
            }
            config_dir_ = std::string(value);
        } else {
            std::cerr << "Error! Unknown arg " << argv[i] << std::endl;
            return 1;
        }
    }

    if (config_dir_) {
        if (setenv("SMALLGAME_CONFIG_DIR", config_dir_->c_str(), 1) != 0) {
            std::cerr << "Error! setenv failed" << std::endl;
            return 1;
        }
    }
    // Load persisted settings, overlay CLI overrides, and write back (so settings.cfg
    // appears on first run and every change survives a restart).
    Settings settings = Settings::defaults();
    auto settings_load = settings.load(get_config_dir());
    if (window_mode) settings.fullscreen = *window_mode == WindowMode::Fullscreen;
    if (vsync) settings.vsync = *vsync;
    if (msaa) settings.msaa = *msaa;
    if (settings_load == Settings::LoadResult::NoFile) {  // create on first run
        settings.save(get_config_dir());
    }

    Window window({
        .title = "Small Game",
        .mode = settings.fullscreen ? WindowMode::Fullscreen : WindowMode::Windowed,
        .width = settings.window_width,
        .height = settings.window_height,
        .msaa = settings.msaa,
        .vsync = settings.vsync,
        .profile_path = profile_path,
    });
    if (launch) {
        Mission mission = *launch;
        if (player_type) mission.player_type = *player_type;
        if (player_ai) mission.player_ai = *player_ai;
        if (time_limit) mission.time_limit = *time_limit;
        if (seed) mission.seed = *seed;
        Renderer renderer;
        game_loop(window, renderer, settings, mission);
    } else {
        shell_loop(window, settings);
    }
}
