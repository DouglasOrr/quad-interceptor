#include <cstdint>
#include <cstdio>
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

int main(int argc, char** argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s <input> <output>\n", argv[0]);
        return 1;
    }
    auto original = read_file(argv[1]);
    size_t max_comp = apultra_get_max_compressed_size(original.size());
    std::vector<uint8_t> comp(max_comp);
    size_t comp_size = apultra_compress(original.data(), comp.data(), original.size(), max_comp, 0, 0, 0, nullptr, nullptr);
    if (comp_size == (size_t)-1) {
        fprintf(stderr, "apultra compress failed\n");
        return 1;
    }
    comp.resize(comp_size);
    FILE* f = fopen(argv[2], "wb");
    fwrite(comp.data(), 1, comp.size(), f);
    fclose(f);
    return 0;
}
