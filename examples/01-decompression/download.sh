set -e

DIR="third-party"

# miniz
MINIZ_URL="https://github.com/richgel999/miniz/releases/download/3.1.2/miniz-3.1.2.zip"
curl -L "$MINIZ_URL" -o /tmp/miniz.zip
unzip -o -j /tmp/miniz.zip "miniz.c" "miniz.h" -d "$DIR/miniz"
rm /tmp/miniz.zip

# heatshrink
HEATSHRINK_URL="https://github.com/atomicobject/heatshrink/archive/refs/tags/v0.4.1.tar.gz"
HEAT_FILES="heatshrink_common.h heatshrink_config.h heatshrink_decoder.h heatshrink_encoder.h heatshrink_decoder.c heatshrink_encoder.c heatshrink.c"
curl -L "$HEATSHRINK_URL" -o /tmp/heatshrink.tar.gz
for f in $HEAT_FILES; do
    tar xzf /tmp/heatshrink.tar.gz -C "$DIR/heatshrink" --strip-components=1 "heatshrink-0.4.1/$f"
done
rm /tmp/heatshrink.tar.gz

# puff
PUFF_URL="https://raw.githubusercontent.com/madler/zlib/master/contrib/puff/puff.c"
PUFF_H_URL="https://raw.githubusercontent.com/madler/zlib/master/contrib/puff/puff.h"
mkdir -p "$DIR/puff"
curl -L "$PUFF_URL" -o "$DIR/puff/puff.c"
curl -L "$PUFF_H_URL" -o "$DIR/puff/puff.h"

# apultra (aPLib compatible)
APULTRA_URL="https://github.com/emmanuel-marty/apultra/archive/refs/heads/master.tar.gz"
mkdir -p "$DIR/apultra"
curl -L "$APULTRA_URL" -o /tmp/apultra.tar.gz
tar xzf /tmp/apultra.tar.gz -C "$DIR/apultra" --strip-components=1
rm /tmp/apultra.tar.gz
