FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends \
    libsdl2-2.0-0 libgl1 libegl1 libgl1-mesa-dri unzip \
    && rm -rf /var/lib/apt/lists/*
COPY quad-interceptor-release.zip /tmp/
RUN unzip -q /tmp/quad-interceptor-release.zip -d /opt \
    && rm /tmp/quad-interceptor-release.zip
ENTRYPOINT ["/opt/quad-interceptor"]
