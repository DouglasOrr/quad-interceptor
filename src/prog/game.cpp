#include <cstring>
#include <string>
#include <string_view>

#include "game.hpp"
#include "platform.hpp"

int main(int argc, char** argv) {
    const auto missions = Mission::all();
    auto mission = missions.begin();
    WindowMode window_mode = WindowMode::Windowed;

    for (int i = 1; i < argc; ++i) {
        // Parse
        std::string_view arg = argv[i];
        const size_t eq = arg.find('=');
        std::string_view key = arg.substr(0, eq);
        std::string_view value =
            eq == std::string_view::npos ? std::string_view{} : arg.substr(eq + 1);

        // Process
        if (key == "mission") {
            auto mission = std::find_if(missions.begin(), missions.end(),
                                        [value](auto& m) { return m.name == value; });
            if (mission == missions.end()) {
                std::cerr << "Error! Unknown mission=" << value << std::endl;
                return 1;
            }
        } else if (key == "window") {
            if (value == "full") {
                window_mode = WindowMode::Fullscreen;
            } else if (value == "medium") {
                window_mode = WindowMode::Windowed;
            } else {
                std::cerr << "Error! Unknown window=" << value << std::endl;
                return 1;
            }
        } else {
            std::cerr << "Error! Unknown arg " << arg << std::endl;
            return 1;
        }
    }

    Window window("Small Game", 1920, 1080, window_mode);
    game_loop(window, *mission);
}
