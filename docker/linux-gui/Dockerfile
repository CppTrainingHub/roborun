# syntax=docker/dockerfile:1

FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG COPPELIASIM_URL=https://downloads.coppeliarobotics.com/V4_10_0_rev0/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz

RUN sed -i 's|http://archive.ubuntu.com/ubuntu/|http://mirror.sjtu.edu.cn/ubuntu/|g' /etc/apt/sources.list

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && apt-get install --yes --no-install-recommends \
         build-essential \
         ca-certificates \
         cmake \
         curl \
         fluxbox \
         imagemagick \
         libboost-dev \
         libboost-regex-dev \
         libdbus-1-3 \
         libfontconfig1 \
         libfreetype6 \
         libgl1 \
         libglu1-mesa \
         libice6 \
         libopengl0 \
         libsm6 \
         libusb-1.0-0 \
         libvpx7 \
         libx11-6 \
         libx11-xcb1 \
         libxcb-cursor0 \
         libxcb-icccm4 \
         libxcb-image0 \
         libxcb-keysyms1 \
         libxcb-randr0 \
         libxcb-render-util0 \
         libxcb-shape0 \
         libxcb-util1 \
         libxcb-xfixes0 \
         libxcb-xinerama0 \
         libxcb-xkb1 \
         libxext6 \
         libxi6 \
         libxkbcommon-x11-0 \
         libxrender1 \
         libzmq3-dev \
         lsof \
         mesa-utils \
         netcat-openbsd \
         novnc \
         procps \
         python3 \
         python3-pip \
         websockify \
         x11-utils \
         x11vnc \
         xdotool \
         xz-utils \
         xvfb \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir "cmake>=3.24,<4" cbor2 pyzmq

RUN mkdir -p /opt/coppeliasim-download \
    && total_bytes=229776116 \
    && chunk_bytes=14361008 \
    && pids='' \
    && part=0 \
    && while [ "$part" -lt 16 ]; do \
         start=$((part * chunk_bytes)); \
         end=$((start + chunk_bytes - 1)); \
         if [ "$end" -ge "$total_bytes" ]; then end=$((total_bytes - 1)); fi; \
         name=$(printf '%02d' "$part"); \
         (curl --http1.1 --fail --location --silent --show-error \
              --retry 8 --retry-delay 1 --retry-all-errors \
              --range "$start-$end" "$COPPELIASIM_URL" \
              --output "/opt/coppeliasim-download/part.$name" \
          && test "$(wc -c < "/opt/coppeliasim-download/part.$name")" \
                  -eq "$((end - start + 1))") & \
         pids="$pids $!"; \
         part=$((part + 1)); \
       done \
    && failed=0 \
    && for pid in $pids; do wait "$pid" || failed=1; done \
    && test "$failed" -eq 0 \
    && cat /opt/coppeliasim-download/part.* > /opt/coppeliasim-download/coppeliasim.tar.xz \
    && test "$(wc -c < /opt/coppeliasim-download/coppeliasim.tar.xz)" -eq "$total_bytes" \
    && xz --test /opt/coppeliasim-download/coppeliasim.tar.xz \
    && sha256sum /opt/coppeliasim-download/coppeliasim.tar.xz \
         | awk '{print $1}' > /opt/CoppeliaSim.archive.sha256 \
    && printf '%s\n' "$COPPELIASIM_URL" > /opt/CoppeliaSim.archive.url \
    && tar -xJf /opt/coppeliasim-download/coppeliasim.tar.xz -C /opt \
    && mv /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04 /opt/CoppeliaSim \
    && rm -rf /opt/coppeliasim-download

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    apt-get update \
    && apt-get install --yes --no-install-recommends git \
    && rm -rf /var/lib/apt/lists/*

LABEL org.opencontainers.image.title="RoboRun Linux GUI capture"
LABEL org.opencontainers.image.version="CoppeliaSim-Edu-4.10.0-rev0-ubuntu22.04-amd64"

ENV DISPLAY=:99
ENV QT_QPA_PLATFORM=xcb
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-roborun
ENV COPPELIASIM_ROOT_DIR=/opt/CoppeliaSim
ENV COPPELIASIM_PYTHON=/usr/bin/python3

WORKDIR /workspace

ENTRYPOINT ["python3", "/workspace/scripts/linux_gui_evidence.py", "container-run", "--case-dir", "/evidence"]
