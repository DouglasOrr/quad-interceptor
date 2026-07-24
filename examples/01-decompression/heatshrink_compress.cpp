#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#define HEATSHRINK_DYNAMIC_ALLOC 1

extern "C" {
#include "third-party/heatshrink/heatshrink_encoder.h"
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

int main(int argc, char** argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <input> <output>\n", argv[0]);
        return 1;
    }
    auto input = read_file(argv[1]);

    uint8_t window_sz2 = 11;
    uint8_t lookahead_sz2 = 4;
    heatshrink_encoder* hse = heatshrink_encoder_alloc(window_sz2, lookahead_sz2);

    std::vector<uint8_t> buf(4096);
    std::vector<uint8_t> compressed;
    size_t sunk = 0;
    while (1) {
        if (sunk < input.size()) {
            size_t sink_sz = 0;
            heatshrink_encoder_sink(hse, input.data() + sunk, input.size() - sunk, &sink_sz);
            sunk += sink_sz;
        }
        size_t poll_sz = 0;
        auto pres = heatshrink_encoder_poll(hse, buf.data(), buf.size(), &poll_sz);
        if (poll_sz > 0) compressed.insert(compressed.end(), buf.data(), buf.data() + poll_sz);
        if (sunk >= input.size() && pres == HSER_POLL_EMPTY) {
            auto fres = heatshrink_encoder_finish(hse);
            if (fres == HSER_FINISH_MORE) continue;
            if (fres == HSER_FINISH_DONE) {
                size_t poll_sz2 = 0;
                heatshrink_encoder_poll(hse, buf.data(), buf.size(), &poll_sz2);
                if (poll_sz2 > 0) compressed.insert(compressed.end(), buf.data(), buf.data() + poll_sz2);
                break;
            }
        }
    }
    heatshrink_encoder_free(hse);

    FILE* f = fopen(argv[2], "wb");
    fwrite(compressed.data(), 1, compressed.size(), f);
    fclose(f);
    return 0;
}