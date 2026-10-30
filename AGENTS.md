# eBUS Library Development Guide

This repository contains the eBUS communication library. Use
[README.md](README.md) for the library overview and build features, and
[CONTRIBUTING.md](CONTRIBUTING.md) for coding standards, architecture, and
development practices.

## Build and Test

Build the library and run its Catch2 test suite from the repository root:

```bash
cmake -S . -B build -DEBUS_SIMULATION=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Tests only build with `EBUS_SIMULATION=ON`; CI (`.github/workflows/build.yml`)
runs the same sequence. Requires CMake ≥3.16. Embedding consumers set
`EBUS_BUILD_TOOLS=OFF` to skip host utilities.

## Implementation Constraints

- Use C++17.
- Avoid heap allocation in the protocol-processing hot path; prefer fixed
  capacity storage and existing small-buffer types.
- Preserve the public API and protocol behavior unless the task explicitly
  requires changing them.
- Keep library changes within this repository and run the relevant tests after
  modifying code.
