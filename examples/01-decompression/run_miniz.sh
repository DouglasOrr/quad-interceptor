set -e
set -o xtrace

CXXFLAGS="-Wall -Wextra -Werror -Os -ffunction-sections -fdata-sections"
LINKFLAGS="-Wl,--gc-sections"

mkdir -p build

# Pre-compress the dictionary
clang++ miniz_compress.cpp ${CXXFLAGS} -o build/miniz_compress ${LINKFLAGS}
./build/miniz_compress /usr/share/dict/words build/words.compressed

# Test the decompressor
clang++ miniz_decompress.cpp ${CXXFLAGS} -o build/miniz_decompress ${LINKFLAGS}
strip build/miniz_decompress
stat -c %s build/miniz_decompress
./build/miniz_decompress
