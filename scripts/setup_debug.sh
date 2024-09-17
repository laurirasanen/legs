#!/bin/bash

set -euo pipefail

meson setup --reconfigure \
    -D examples=true \
    -D buildtype=debug \
    -D b_sanitize=address,undefined \
    build

