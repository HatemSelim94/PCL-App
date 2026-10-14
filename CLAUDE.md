# Notes for Claude Code

Point cloud loading, voxel downsampling and Euclidean clustering: a header-only
C++17 core on PCL (`core/`), pybind11 bindings and an MCP server (`python/`), a
Qt desktop application (`app/`). The README has the overview and the status
table.

## Build and test

Linux (PCL from the distribution: `libpcl-dev`, plus `python3-dev python3-venv`):

```sh
# C++ core and tests
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPCLAPP_BUILD_APP=OFF
cmake --build build --parallel
ctest --test-dir build --output-on-failure

# Python package and tests
python3 -m venv .venv
.venv/bin/pip install ".[test]"
.venv/bin/python -m pytest
```

Windows (Visual Studio 2022, PCL from vcpkg, `cmd` syntax):

```bat
set VCPKG_ROOT=C:\vcpkg
set CMAKE_TOOLCHAIN_FILE=C:\vcpkg\scripts\buildsystems\vcpkg.cmake

cmake --preset core-vcpkg
cmake --build --preset core-vcpkg
ctest --preset core-vcpkg

python -m venv .venv
.venv\Scripts\python -m pip install ".[test]"
.venv\Scripts\python -m pytest
```

After changing `python/src/bindings.cpp` or anything in `core/`, run the
`pip install` line again before `pytest`: the tests import the installed
package, not the source tree.

Desktop application (needs VTK 9 with Qt support; Qt 5 on Ubuntu, Qt 6 with
vcpkg):

```sh
cmake --preset app            # Linux; app-vcpkg on Windows
cmake --build --preset app
```

The application is translated (English, German, French, Spanish). After
changing or adding user-visible strings, refresh the translation sources and
fill in the new entries by hand:

```sh
lupdate -no-obsolete app/main.cpp app/mainwindow.cpp app/mainwindow.h app/mainwindow.ui \
    app/language.cpp -ts app/translations/pclapp_de.ts app/translations/pclapp_fr.ts \
    app/translations/pclapp_es.ts
lupdate -pluralonly -no-obsolete app/mainwindow.cpp -ts app/translations/pclapp_en.ts
```

The build compiles them with lrelease and copies the `.qm` files next to the
executable. Language names in the menu stay in their own language.

## Conventions

- C++17, formatted with the repository's `.clang-format`.
- `core/` is header-only, lives in namespace `pclapp`, and must not include Qt
  or VTK headers. Geometry belongs there and nowhere else.
- Processing steps are pure functions that take a parameter struct. Bad
  parameters throw `std::invalid_argument`, file problems throw
  `pclapp::IoError`.
- Functions that take a cloud pointer use `CloudPtr<PointT>` so that `PointT`
  can be deduced; create clouds with `pcl::make_shared`.
- The bindings only convert types. The MCP server only tracks clouds and checks
  paths; every path a client supplies goes through `resolve_in_data_dir`.
- The MCP server must never write to stdout, because stdout is the protocol
  channel.
- Every user-visible string in `app/` goes through `tr()` (or the `.ui`
  file), and the translation files are updated in the same change.
- New behaviour gets a test: Catch2 in `tests/core_tests.cpp`, pytest in
  `tests/python/`.
- Commit messages follow `type(scope): summary` (feat, fix, refactor, test,
  build, ci, docs, chore).

## Rules for this repository

- The README may only claim what the code does, and may only list a platform
  after the code was built and run there. Update its status table when that
  changes.
- Say which parts are PCL's algorithms and which are this project's design.
- Prefer standard, boring tooling over clever solutions.
- Ask before anything hard to undo: rewriting history, renaming the
  repository, moving the application from Qt 5 to Qt 6.
