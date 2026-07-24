set -e
set -o xtrace

CXXFLAGS="-Wall -Wextra -Werror -Os -ffunction-sections -fdata-sections"
LINKFLAGS="-Wl,--gc-sections"

mkdir -p build

clang++ baseline.cpp ${CXXFLAGS} -o build/baseline ${LINKFLAGS}
strip build/baseline
stat -c %s build/baseline
./build/baseline
