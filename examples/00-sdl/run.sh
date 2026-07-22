set -e
set -o xtrace

CXXFLAGS="-Wall -Wextra -Werror -Os -ffunction-sections -fdata-sections"
LINKFLAGS="-Wl,--gc-sections"

mkdir -p build
clang++ main.cpp ${CXXFLAGS} -isystem /usr/include/SDL2 -o build/main -lSDL2 -lGL ${LINKFLAGS}
strip build/main
ls -lh build/main
./build/main
