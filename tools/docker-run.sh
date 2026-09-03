#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"

"$root/dev" -v release package
docker build -t quad-interceptor -f "$root/tools/Dockerfile" "$root/build"

if xhost +local:docker >/dev/null 2>&1; then
  trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT
fi

docker run --rm -it \
  -e DISPLAY="$DISPLAY" \
  -e SDL_VIDEODRIVER=x11 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e XDG_RUNTIME_DIR=/tmp/xdg \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  quad-interceptor "$@"
