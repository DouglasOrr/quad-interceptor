set -e
set -o xtrace

CFLAGS="-Wall -Wextra -Werror -Os -ffunction-sections -fdata-sections -DHEATSHRINK_DYNAMIC_ALLOC=1"
CXXFLAGS="-Wall -Wextra -Werror -Os -ffunction-sections -fdata-sections"
LINKFLAGS="-Wl,--gc-sections"
INC="-Ithird-party/heatshrink"

mkdir -p build

# Compile heatshrink C objects with dynamic alloc enabled
clang third-party/heatshrink/heatshrink_decoder.c ${CFLAGS} ${INC} -c -o build/hs_decoder.o
clang third-party/heatshrink/heatshrink_encoder.c ${CFLAGS} ${INC} -c -o build/hs_encoder.o

# Pre-compress the dictionary
clang++ heatshrink_compress.cpp ${CXXFLAGS} ${INC} -o build/heatshrink_compress build/hs_encoder.o ${LINKFLAGS}
./build/heatshrink_compress /usr/share/dict/words build/heatshrink_words.compressed

# Test the decompressor
clang++ heatshrink_decompress.cpp ${CXXFLAGS} ${INC} -o build/heatshrink_decompress build/hs_decoder.o ${LINKFLAGS}
strip build/heatshrink_decompress
stat -c %s build/heatshrink_decompress
./build/heatshrink_decompress
