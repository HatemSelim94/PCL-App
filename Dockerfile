# Builds the core, runs its tests, installs the Python package and starts the
# MCP server. Linux only (the desktop application is not part of the image).
#
#   docker build -t pclapp .
#   docker run -i --rm -v /path/to/clouds:/data pclapp
#
# The server talks MCP over stdin/stdout, hence "-i". /data is the only
# directory it can read and write.

# ---- stage 1: build and test ------------------------------------------------
FROM ubuntu:24.04 AS build

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates \
        libpcl-dev python3-dev python3-venv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY . .

# C++ core and its tests
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPCLAPP_BUILD_APP=OFF \
    && cmake --build build --parallel \
    && ctest --test-dir build --output-on-failure

# Python package with the MCP server, plus its tests
RUN python3 -m venv /opt/pclapp \
    && /opt/pclapp/bin/pip install --no-cache-dir ".[test]" \
    && /opt/pclapp/bin/python -m pytest -q

# ---- stage 2: runtime ---------------------------------------------------------
# Only the shared libraries the extension module needs, not the -dev packages.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 libpcl-io1.14 libpcl-filters1.14 libpcl-search1.14 libpcl-segmentation1.14 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /opt/pclapp /opt/pclapp

RUN mkdir /data
VOLUME /data
ENV PCLAPP_DATA_DIR=/data

ENTRYPOINT ["/opt/pclapp/bin/pclapp-mcp"]
