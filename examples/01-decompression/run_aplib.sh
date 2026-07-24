set -e
set -o xtrace

CXXFLAGS="-Wall -Wextra -Werror -Os -ffunction-sections -fdata-sections"
LINKFLAGS="-Wl,--gc-sections"
INC="-Ithird-party/apultra/src -Ithird-party/apultra/src/libdivsufsort/include"

mkdir -p build

make -C third-party/apultra CC=clang CFLAGS="-Os -Isrc/libdivsufsort/include -Isrc" libs

clang++ aplib_compress.cpp ${CXXFLAGS} ${INC} third-party/apultra/libapultra.a -o build/aplib_compress ${LINKFLAGS}
./build/aplib_compress /usr/share/dict/words build/aplib_words.compressed

clang++ aplib_decompress.cpp ${CXXFLAGS} ${INC} third-party/apultra/libapultra.a -o build/aplib_decompress ${LINKFLAGS}
strip build/aplib_decompress
stat -c %s build/aplib_decompress
./build/aplib_decompress
