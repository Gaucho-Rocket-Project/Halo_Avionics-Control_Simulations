# C port

## C library and native tests

Requires CMake 3.23+, Ninja, and a C11 compiler.

```bash
cd controls/c-port
cmake --preset default
cmake --build --preset default
ctest --test-dir build --output-on-failure
```

## Python comparisons

Install [uv](https://docs.astral.sh/uv/) and sync the local Python environment:

```bash
cd controls/c-port
uv sync
uv run tests/compare_python.py
uv run tests/compare_halo_controls.py
```
