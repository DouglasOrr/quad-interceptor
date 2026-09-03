#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#include "game.hpp"
#include "platform.hpp"

////////////////////////////////////////////////////////////////////////////////
// Settings

namespace {

constexpr const char* settings_file = "settings.cfg";
constexpr const char* progress_file = "progress.cfg";

std::string trim(std::string s) {
    auto is_ws = [](char c) { return c == ' ' || c == '\t' || c == '\r'; };
    size_t b = 0, e = s.size();
    while (b < e && is_ws(s[b]))
        ++b;
    while (e > b && is_ws(s[e - 1]))
        --e;
    return s.substr(b, e - b);
}

void parse_bool(const std::string& v, bool& out, bool& failed, const char* file) {
    if (v == "true") {
        out = true;
    } else if (v == "false") {
        out = false;
    } else {
        std::fprintf(stderr, "%s: invalid bool '%s'\n", file, v.c_str());
        failed = true;
    }
}

void parse_float(const std::string& v, float& out, bool& failed, const char* file) {
    char* end = nullptr;
    const float f = std::strtof(v.c_str(), &end);
    if (end == v.c_str() || *end != '\0') {
        std::fprintf(stderr, "%s: invalid float '%s'\n", file, v.c_str());
        failed = true;
    } else {
        out = f;
    }
}

void parse_int(const std::string& v, int& out, bool& failed, const char* file) {
    char* end = nullptr;
    const long n = std::strtol(v.c_str(), &end, 10);
    if (end == v.c_str() || *end != '\0') {
        std::fprintf(stderr, "%s: invalid int '%s'\n", file, v.c_str());
        failed = true;
    } else {
        out = int(n);
    }
}

void parse_binding(const std::string& key, const std::string& value, Settings& s, bool& failed) {
    gassert(key.starts_with("bind."));
    const std::string name = key.substr(5);
    for (uint i = 0; i < size_t(Action::count); ++i) {
        if (name == Action::key(Action::Type(i))) {
            int sc = 0;
            bool parse_failed = false;
            parse_int(value, sc, parse_failed, settings_file);
            if (parse_failed) {
                failed = true;
            } else if (sc >= 0 && sc < SDL_NUM_SCANCODES) {
                s.bindings[i] = sc;
            } else {
                std::fprintf(stderr, "%s: binding '%s' scancode %d out of range\n", settings_file,
                             name.c_str(), sc);
                failed = true;
            }
            return;
        }
    }
    std::fprintf(stderr, "%s: unexpected binding '%s'\n", settings_file, key.c_str());
}

std::string fmt_float(float f) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.2f", f);
    return buf;
}

}  // namespace

const char* Action::key(Type a) {
    switch (a) {
        case Action::RollLeft:
            return "roll_left";
        case Action::RollRight:
            return "roll_right";
        case Action::PitchFwd:
            return "pitch_fwd";
        case Action::PitchBack:
            return "pitch_back";
        case Action::LiftUp:
            return "lift_up";
        case Action::LiftDown:
            return "lift_down";
        case Action::YawLeft:
            return "yaw_left";
        case Action::YawRight:
            return "yaw_right";
        default:
            return "?";
    }
}

const char* Action::name(Type a) {
    switch (a) {
        case Action::RollLeft:
            return "left (roll)";
        case Action::RollRight:
            return "right (roll)";
        case Action::PitchFwd:
            return "forward (pitch)";
        case Action::PitchBack:
            return "back (pitch)";
        case Action::LiftUp:
            return "up (lift)";
        case Action::LiftDown:
            return "down (lift)";
        case Action::YawLeft:
            return "rotate left (yaw)";
        case Action::YawRight:
            return "rotate right (yaw)";
        default:
            return "?";
    }
}

std::string get_config_dir() {
    if (const char* env = std::getenv("QUAD_INTERCEPTOR_CONFIG_DIR"); env && *env) return env;
    if (std::string pref = Window::pref_path("QuadInterceptor"); !pref.empty()) return pref;
    return ".";
}

Settings Settings::defaults() {
    Settings s;
    s.fullscreen = false;
    s.msaa = 1;
    s.vsync = true;
    s.window_width = 1920;
    s.window_height = 1080;
    s.master_volume = 1.0f;
    s.music_volume = 0.5f;
    s.effects_volume = 0.5f;
    s.bindings = std::get<1>(Settings::control_presets().front());
    return s;
}

std::vector<std::tuple<std::string, Settings::Bindings>> Settings::control_presets() {
    return {
        {"Preset - WASD/IJKL",
         {
             SDL_SCANCODE_J,
             SDL_SCANCODE_L,
             SDL_SCANCODE_I,
             SDL_SCANCODE_K,
             SDL_SCANCODE_W,
             SDL_SCANCODE_S,
             SDL_SCANCODE_A,
             SDL_SCANCODE_D,
         }},
        {"Preset - WASD/IJKL (rev)",
         {
             SDL_SCANCODE_A,
             SDL_SCANCODE_D,
             SDL_SCANCODE_W,
             SDL_SCANCODE_S,
             SDL_SCANCODE_I,
             SDL_SCANCODE_K,
             SDL_SCANCODE_J,
             SDL_SCANCODE_L,
         }},
        {"Preset - Arrows",
         {
             SDL_SCANCODE_LEFT,
             SDL_SCANCODE_RIGHT,
             SDL_SCANCODE_UP,
             SDL_SCANCODE_DOWN,
             SDL_SCANCODE_X,
             SDL_SCANCODE_LALT,
             SDL_SCANCODE_Z,
             SDL_SCANCODE_C,
         }},
    };
}

std::array<std::tuple<int, int>, 4> Settings::resolutions() {
    return {{
        {1280, 720},
        {1600, 900},
        {1920, 1080},
        {2560, 1440},
    }};
}

Settings::LoadResult Settings::load(const std::string& dir) {
    auto path = dir + "/" + settings_file;
    std::ifstream in(path);
    fprintf(stderr, "Loading settings from %s\n", path.c_str());
    if (!in) return LoadResult::NoFile;
    bool any_failed = false;
    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));
        bool failed = false;
        if (key == "fullscreen") {
            parse_bool(value, fullscreen, failed, settings_file);
        } else if (key == "msaa") {
            int n = 0;
            parse_int(value, n, failed, settings_file);
            if (!failed && n >= 1) msaa = uint(n);
        } else if (key == "vsync") {
            parse_bool(value, vsync, failed, settings_file);
        } else if (key == "window_width") {
            int n = 0;
            parse_int(value, n, failed, settings_file);
            if (!failed && n > 0) window_width = n;
        } else if (key == "window_height") {
            int n = 0;
            parse_int(value, n, failed, settings_file);
            if (!failed && n > 0) window_height = n;
        } else if (key == "master_volume") {
            float f = 0.0f;
            parse_float(value, f, failed, settings_file);
            if (!failed) master_volume = std::clamp(f, 0.0f, 1.0f);
        } else if (key == "music_volume") {
            float f = 0.0f;
            parse_float(value, f, failed, settings_file);
            if (!failed) music_volume = std::clamp(f, 0.0f, 1.0f);
        } else if (key == "effects_volume") {
            float f = 0.0f;
            parse_float(value, f, failed, settings_file);
            if (!failed) effects_volume = std::clamp(f, 0.0f, 1.0f);
        } else if (key.starts_with("bind.")) {
            parse_binding(key, value, *this, failed);
        } else {
            fprintf(stderr, "%s: unexpected setting '%s'\n", settings_file, key.c_str());
            failed = true;
        }
        any_failed |= failed;
    }
    return any_failed ? LoadResult::Error : LoadResult::OK;
}

void Settings::save(const std::string& dir) const {
    std::filesystem::create_directories(dir);
    std::ofstream out(dir + "/" + settings_file);
    if (!out) return;
    out << "# settings.cfg\n";
    out << "fullscreen=" << (fullscreen ? "true" : "false") << '\n';
    out << "msaa=" << msaa << '\n';
    out << "vsync=" << (vsync ? "true" : "false") << '\n';
    out << "window_width=" << window_width << '\n';
    out << "window_height=" << window_height << '\n';
    out << "master_volume=" << fmt_float(master_volume) << '\n';
    out << "music_volume=" << fmt_float(music_volume) << '\n';
    out << "effects_volume=" << fmt_float(effects_volume) << '\n';
    for (uint i = 0; i < size_t(Action::count); ++i)
        out << "bind." << Action::key(Action::Type(i)) << '=' << bindings[i] << '\n';
}

////////////////////////////////////////////////////////////////////////////////
// Shell

namespace {

ShellState load_shell(const std::string& dir) {
    ShellState shell{
        .persistent = true,
        .screen = ShellState::Screen::Main,
        .navigation = ShellState::Navigation::None,
        .chosen{},
        .missions{},
    };
    const auto all_missions = Mission::all();
    for (auto& m : all_missions) {
        shell.missions.push_back(
            {.name = m.name, .best = {MissionResult::None, 0.0f, 0u}, .unlocked = false});
    }
    shell.missions.front().unlocked = true;  // first mission always unlocked
    for (size_t i = 0; i < all_missions.size(); ++i)
        if (all_missions[i].type == Mission::Sandbox) shell.missions[i].unlocked = true;
    auto current_mission = shell.missions.end();
    std::string line;
    std::ifstream in(dir + "/" + progress_file);
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        if (line[0] == '[') {
            const size_t close = line.find(']');
            auto section =
                trim(close == std::string::npos ? line.substr(1) : line.substr(1, close - 1));
            current_mission = std::find_if(shell.missions.begin(), shell.missions.end(),
                                           [&section](auto& m) { return m.name == section; });
            if (current_mission == shell.missions.end()) {
                fprintf(stderr, "%s: unexpected misssion [%s]\n", progress_file, section.c_str());
            }
            continue;
        }
        if (current_mission == shell.missions.end()) {
            fprintf(stderr, "%s: mission data outside [mission]: %s\n", progress_file,
                    line.c_str());
            continue;
        }
        const size_t eq = line.find('=');
        if (eq == std::string::npos) {
            fprintf(stderr, "%s: couldn't parse line '%s'\n", progress_file, line.c_str());
            continue;
        }
        auto key = trim(line.substr(0, eq));
        auto value = trim(line.substr(eq + 1));
        bool bad_line = false;
        if (key == "completed") {
            bool b = false;
            parse_bool(value, b, bad_line, progress_file);
            if (!bad_line)
                current_mission->best.type = b ? MissionResult::Completed : MissionResult::None;
        } else if (key == "best_time") {
            float t = 0.0f;
            parse_float(value, t, bad_line, progress_file);
            if (!bad_line) current_mission->best.time = std::max(0.0f, t);
        } else if (key == "best_score") {
            int sc = 0;
            parse_int(value, sc, bad_line, progress_file);
            if (!bad_line) current_mission->best.score = uint(std::max(0, sc));
        } else if (key == "unlocked") {
            bool b = false;
            parse_bool(value, b, bad_line, progress_file);
            if (!bad_line) current_mission->unlocked = current_mission->unlocked || b;
        }
    }
    return shell;
}

void save_shell(ShellState& shell, const std::string& dir) {
    std::filesystem::create_directories(dir);
    auto path = dir + "/" + progress_file;
    std::ofstream out(path);
    if (!out) {
        fprintf(stderr, "failed to write progress: %s\n", progress_file);
        shell.persistent = false;
        return;
    }
    shell.persistent = true;
    out << "# progress.cfg\n";
    for (auto& m : shell.missions) {  // always complete and ordered
        out << '[' << m.name << "]\n";
        out << "completed=" << (m.best.type == MissionResult::Completed ? "true" : "false") << '\n';
        out << "best_time=" << fmt_float(m.best.time) << '\n';
        out << "best_score=" << m.best.score << '\n';
        out << "unlocked=" << (m.unlocked ? "true" : "false") << '\n';
    }
}

void record_result(ShellState& shell, const Mission& mission, const MissionResult& result) {
    auto it = std::find_if(shell.missions.begin(), shell.missions.end(),
                           [&](auto& m) { return m.name == mission.name; });
    if (it == shell.missions.end()) return;
    if (result.type == MissionResult::Completed) it->best.type = MissionResult::Completed;
    if (result.time > 0.0f && (it->best.time == 0.0f || result.time < it->best.time))
        it->best.time = result.time;
    if (result.score > it->best.score) it->best.score = result.score;
    if (result.type == MissionResult::Completed) {
        const size_t idx = size_t(it - shell.missions.begin());
        if (idx + 1 < shell.missions.size()) shell.missions[idx + 1].unlocked = true;
    }
}

}  // namespace

void shell_unlock_all(ShellState& shell) {
    for (auto& m : shell.missions)
        m.unlocked = true;
    save_shell(shell, get_config_dir());
}

void shell_loop(Window& window, Settings& settings) {
    auto config_dir = get_config_dir();
    auto shell = load_shell(config_dir);
    save_shell(shell, config_dir);
    Renderer renderer;
    while (window.frame()) {
        renderer.menu_update_draw(window, shell, settings, /*overlay*/ false);
        switch (shell.navigation) {
            case ShellState::Navigation::Play: {
                auto result = game_loop(window, renderer, settings, shell.chosen);
                record_result(shell, shell.chosen, result);
                save_shell(shell, config_dir);
                shell.screen = ShellState::Screen::Missions;
                shell.navigation = ShellState::Navigation::None;
                break;
            }
            case ShellState::Navigation::Quit:
                save_shell(shell, config_dir);
                return;
            default:
                break;
        }
    }
    save_shell(shell, config_dir);
}
