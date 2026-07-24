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
    size_t comp_size = 0;
    void* comp_data = tdefl_compress_mem_to_heap(original.data(), original.size(), &comp_size, 128);
    if (!comp_data) {
        fprintf(stderr, "compress failed\n");
        return 1;
    }
    FILE* f = fopen(argv[2], "wb");
    fwrite(comp_data, 1, comp_size, f);
    fclose(f);
    free(comp_data);
    return 0;
}
