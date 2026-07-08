#!/usr/bin/env python
# PlatformIO pre-script: injects the current build number as -DBUILD_NUMBER.
# The counter in .build_number is incremented by the Makefile (`make`/`make deploy`)
# so IDE/intellisense builds don't bump it. Reads (does not increment) here.
import os
Import("env")  # noqa: F821

bn_file = os.path.join(env["PROJECT_DIR"], ".build_number")  # noqa: F821
n = 0
try:
    with open(bn_file) as f:
        n = int(f.read().strip() or "0")
except Exception:
    n = 0

env.Append(CPPDEFINES=[("BUILD_NUMBER", n)])  # noqa: F821
print("== firmware build #%d ==" % n)
