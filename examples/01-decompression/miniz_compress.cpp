#include <cstdint>
#include <cstdio>
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

int main(int argc, char** argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <input> <output>\n", argv[0]);
        return 1;
    }
    auto original = read_file(argv[1]);
    mz_ulong bound = mz_compressBound(original.size());
    std::vector<uint8_t> comp(bound);
    mz_ulong comp_size = bound;
    int ret = mz_compress(comp.data(), &comp_size, original.data(), original.size());
    if (ret != MZ_OK) {
        fprintf(stderr, "compress failed: %d\n", ret);
        return 1;
    }
    comp.resize(comp_size);
    FILE* f = fopen(argv[2], "wb");
    fwrite(comp.data(), 1, comp.size(), f);
    fclose(f);
    return 0;
}