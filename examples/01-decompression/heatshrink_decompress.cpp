#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#define HEATSHRINK_DYNAMIC_ALLOC 1

extern "C" {
#include "third-party/heatshrink/heatshrink_decoder.h"
}

static std::vector<uint8_t> read_file(const char* path) {
    FILE* f = fopen(path, "rb");
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    std::vector<uint8_t> buf(len);
    fread(buf.data(), 1, len, f);
    fclose(f);
    return buf;
}

int main() {
    auto original = read_file("/usr/share/dict/words");
    auto compressed = read_file("build/heatshrink_words.compressed");
    printf("original:   %zu bytes\n", original.size());
    printf("compressed: %zu bytes (%.1f%%)\n", compressed.size(),
           100.0 * compressed.size() / original.size());

    uint8_t window_sz2 = 11;
    uint8_t lookahead_sz2 = 4;
    heatshrink_decoder* hsd = heatshrink_decoder_alloc(256, window_sz2, lookahead_sz2);

    std::vector<uint8_t> buf(4096);
    std::vector<uint8_t> decompressed;
    size_t sunk = 0;
    while (1) {
        if (sunk < compressed.size()) {
            size_t sink_sz = 0;
            heatshrink_decoder_sink(hsd, compressed.data() + sunk, compressed.size() - sunk, &sink_sz);
            sunk += sink_sz;
        }
        size_t poll_sz = 0;
        auto pres = heatshrink_decoder_poll(hsd, buf.data(), buf.size(), &poll_sz);
        if (poll_sz > 0) decompressed.insert(decompressed.end(), buf.data(), buf.data() + poll_sz);
        if (sunk >= compressed.size() && pres == HSDR_POLL_EMPTY) {
            auto fres = heatshrink_decoder_finish(hsd);
            if (fres == HSDR_FINISH_MORE) continue;
            if (fres == HSDR_FINISH_DONE) {
                size_t poll_sz2 = 0;
                heatshrink_decoder_poll(hsd, buf.data(), buf.size(), &poll_sz2);
                if (poll_sz2 > 0)
                    decompressed.insert(decompressed.end(), buf.data(), buf.data() + poll_sz2);
                break;
            }
        }
    }
    heatshrink_decoder_free(hsd);

    bool ok = (decompressed.size() == original.size() &&
               memcmp(decompressed.data(), original.data(), original.size()) == 0);
    printf("verify: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}