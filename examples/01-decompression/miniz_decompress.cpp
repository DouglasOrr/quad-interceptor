#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#define MINIZ_NO_ARCHIVE_APIS
#define MINIZ_NO_TIME
#define MINIZ_NO_ZLIB_COMPATIBLE_NAMES
#include "third-party/miniz/miniz.c"

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
    auto compressed = read_file("build/words.compressed");
    printf("original:   %zu bytes\n", original.size());
    printf("compressed: %zu bytes (%.1f%%)\n", compressed.size(),
           100.0 * compressed.size() / original.size());

    std::vector<uint8_t> decompressed(original.size());
    mz_ulong dec_size = decompressed.size();
    int ret = mz_uncompress(decompressed.data(), &dec_size, compressed.data(), compressed.size());
    if (ret != MZ_OK) {
        fprintf(stderr, "uncompress failed: %d\n", ret);
        return 1;
    }
    decompressed.resize(dec_size);

    bool ok = (decompressed.size() == original.size() &&
               memcmp(decompressed.data(), original.data(), original.size()) == 0);
    printf("verify: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}