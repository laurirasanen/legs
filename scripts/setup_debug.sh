#!/bin/bash

set -euo pipefail

export CXX=clang++

# TODO: ubsan doesn't link with jolt
# TODO: gcc busted
meson setup --reconfigure \
    -D examples=true \
    -D buildtype=debug \
    -D b_sanitize=address \
    -D b_lundef=false \
    build

