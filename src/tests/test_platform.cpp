#include "doctest/doctest.h"

#include "platform.hpp"

TEST_CASE("Window creates and destroys") {
    Window w("Test", 640, 480);
    CHECK(w.frame());
}
