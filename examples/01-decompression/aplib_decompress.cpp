#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

extern "C" {
#include "libapultra.h"
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
    auto compressed = read_file("build/aplib_words.compressed");
    printf("original:   %zu bytes\n", original.size());
    printf("compressed: %zu bytes (%.1f%%)\n", compressed.size(),
           100.0 * compressed.size() / original.size());

    size_t max_dec = apultra_get_max_decompressed_size(compressed.data(), compressed.size(), 0);
    std::vector<uint8_t> decompressed(max_dec);
    size_t dec_size = apultra_decompress(compressed.data(), decompressed.data(), compressed.size(), max_dec, 0, 0);
    if (dec_size == (size_t)-1) {
        fprintf(stderr, "apultra decompress failed\n");
        return 1;
    }
    decompressed.resize(dec_size);

    bool ok = (decompressed.size() == original.size() &&
               memcmp(decompressed.data(), original.data(), original.size()) == 0);
    printf("verify: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
