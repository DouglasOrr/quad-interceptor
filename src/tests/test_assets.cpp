#include <doctest/doctest.h>
#include <set>

#include "game.hpp"

TEST_CASE("every glyph payload is width*height bytes of 0 or 255") {
    const Font f = Font::load();
    std::set<uint8_t> chars;
    for (const Font::Glyph& g : f.glyphs) {
        chars.insert(g.ch);
        CHECK(g.data.size() == f.width * f.height);
        for (auto byte : g.data)
            if (byte != 0) CHECK(byte == 255);
    }
    std::string expected = "abcdefghijklmnopqrstuvwxyz0123456789";
    for (char ch : expected)
        CHECK(chars.count(ch) == 1);
}
