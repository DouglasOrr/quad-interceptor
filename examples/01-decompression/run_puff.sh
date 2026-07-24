set -e
set -o xtrace

CXXFLAGS="-Wall -Wextra -Werror -Os -ffunction-sections -fdata-sections"
LINKFLAGS="-Wl,--gc-sections"

mkdir -p build

# Pre-compress the dictionary
clang++ puff_compress.cpp ${CXXFLAGS} -Ithird-party/miniz -o build/puff_compress ${LINKFLAGS}
./build/puff_compress /usr/share/dict/words build/puff_words.compressed

# Test the decompressor
clang++ puff_decompress.cpp ${CXXFLAGS} -Ithird-party/puff -o build/puff_decompress ${LINKFLAGS}
strip build/puff_decompress
stat -c %s build/puff_decompress
./build/puff_decompress
