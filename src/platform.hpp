#pragma once

#include <memory>

class Window {
public:
    Window(const char* title, int width, int height);
    ~Window();

    // Swap buffers and process input events,
    // returns false if the window should close
    bool frame();

private:
    struct State;
    std::unique_ptr<State> state_;
};
