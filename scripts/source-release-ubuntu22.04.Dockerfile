FROM ubuntu:22.04

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install --yes --no-install-recommends \
         build-essential \
         ca-certificates \
         clang-format \
         clang-tidy \
         gcc-11 \
         g++-11 \
         git \
         python3 \
         python3-pip \
    && python3 -m pip install --no-cache-dir "cmake>=3.24,<4" \
    && rm -rf /var/lib/apt/lists/*

ENV CC=gcc-11
ENV CXX=g++-11
