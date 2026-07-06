# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **CUI//REL TO USA ONLY** — Advanced Framework for Simulation, Integration, and Modeling (AFSIM).
> Controlled by: Air Force Research Laboratory, Aerospace Systems Directorate.
> CUI Categories: CTI, EXPT. DIST-F, REL TO USA ONLY.
> See [README.md](README.md), [LICENSE.md](LICENSE.md), and [CONTRIBUTING.md](CONTRIBUTING.md).

## Repository Layout

This is a **distribution install** — source lives under `swdev/src/`, not at the repo root. This differs from a git clone where source is at the root. The `swdev/src/` directory is what matters for building.

| Path | Purpose |
|---|---|
| `swdev/src/` | Source root (this is `-S .`) |
| `swdev/src/BUILDS/` | Pre-configured CMake build directories (ninja, vs2019) |
| `bin/` | Pre-built executables and libraries |
| `demos/` | Demo scenarios and regression test lists |
| `tools/` | Development tools (at distribution level, not `swdev/src/tools/`) |

**Prerequisites** (see [doc/developer/build_instructions.rst](doc/developer/build_instructions.rst)):
- CMake ≥3.7 (≥3.14 for VS2019)
- Visual Studio 2015/2017/2019 (x64), or GCC 4.8.5+ (Linux)
- Python 3.x (for auto-tests and docs)
- 3rd party packages: GTEST 1.10, Qt 5.12.11, OSG 3.6.3, OSGEarth 2.10.1, SDL 2.0.16, GDAL 3.3.2, etc. — configured via `SWDEV_THIRD_PARTY_ROOT` or auto-detected at `../dependencies/3rd_party`

## Build System

AFSIM uses **CMake** (≥3.7, C++11) with an out-of-source build. This is **not** a git repository — cmake `find_package(Git)` will skip git-based versioning.

### Build presets

Configure with a preset from [cmake/Presets/](cmake/Presets/), then build via the generated project files.

| Preset | What it builds |
|---|---|
| `base` | mission + grammar_check + parser only |
| `exportable` | base + sensor_plot, weapon_tools, wsf_cyber, wsf_mil, wsf_mil_parser, wsf_space, wsf_p6dof, wsf_six_dof, wsf_coverage, wsf_alternate_locations, wsf_annotation, wsf_multiresolution, wsf_ripr, post_processor, wsf_scenario_analyzer |
| `exportable_gui` | exportable + warlock, wizard, mystic, evt_reader + their exportable plugins |
| `restricted` | engage, wsf_air_combat, wsf_brawler, wsf_fires, wsf_l16, wsf_mtt, wsf_nx, wsf_sosm, wsf_weapon_server, wsf_argo8, wsf_iads_c2, wsf_oms_uci, wsf_simdis, wsf_scenario_analyzer_iads_c2 |
| `restricted_gui` | restricted + mover_creator + restricted plugins (HeadDownView, HeadUpView, AcesDisplay, SAD, etc.) |
| `default` | All of the above combined |
| `experimental` | Non-distributed modules (wsf_ibsm_sensor, wsf_jse, wsf_protobuf, wsf_script_io, wsf_zmq) |

### Configure & Build

From this directory (`swdev/src/`):

```bash
# Configure (distribution layout: -S . is swdev/src/)
cmake --preset default-vs2019 -S . -B BUILDS/default-vs2019

# Configure with a limited build (base only)
cmake --preset default-vs2019 -S . -B BUILDS/minimal \
  -DCMAKE_PROJECT_INCLUDE_BEFORE=cmake/Presets/base.cmake

# Build (release)
cmake --build BUILDS/default-vs2019 --config Release

# Build (debug)
cmake --build BUILDS/default-vs2019 --config Debug

# Build a specific target
cmake --build BUILDS/default-vs2019 --target mission --config Release

# Install
cmake --install BUILDS/default-vs2019 --config Release --prefix BUILDS/install
```

**Unity builds** (CMake 3.16+): add `-DCMAKE_UNITY_BUILD=ON` for ~3x faster clean builds. Enables batch compilation of multiple sources within each target.

**VS Code / CMakeSettings.json**: Open `swdev/src/` as a folder in Visual Studio — `CMakeSettings.json` provides pre-configured x64/x86, Debug/Release, and Clang/Ninja profiles for CMake's built-in VS integration.

### CMake generators (from `CMakePresets.json`)

| Configure preset | Generator |
|---|---|
| `default-vs2019` | Visual Studio 16 2019 (x64) |
| `default-vs2017` | Visual Studio 15 2017 (x64) |
| `default-vs2015` | Visual Studio 14 2015 (x64) |
| `default-ninja` | Ninja Multi-Config (x64) |
| append `-clang` | Uses clang-cl instead of MSVC |
| append `-clang-linux` | Linux cross-compile with clang/gcc |

Key CMake variables:
- `BUILD_WITH_<module>` — enable/disable individual modules
- `WSF_PLUGIN_BUILD=TRUE` (default) — build plugins as DLLs vs static libs
- `WSF_ADD_EXTENSION_PATH` — additional search paths for custom extensions/plugins
- `SWDEV_THIRD_PARTY_ROOT` — 3rd party package location
- `PROMOTE_HARDWARE_EXCEPTIONS` — promote divide-by-zero, access-violation to C++ exceptions

## Module System & Discoverability

AFSIM is a **modular framework** of ~41 modules discovered at CMake time. Each module is a directory with:

- `wsf_module` — empty marker file (identifies the directory as a module; CMake globs for these markers)
- `wsf_cmake_extension.cmake` — defines:
  - `WSF_EXT_NAME` — module name
  - `WSF_EXT_TYPE` — `exe`, `plugin`, or `lib` (or `plugin` if `WSF_PLUGIN_BUILD`)
  - `WSF_EXT_SOURCE_PATH` — relative source path
- `CMakeLists.txt` — uses `wsf_project_template()` macro

**Expected project directory layout** (from [cmake/Modules/wsf_project_template.cmake](cmake/Modules/wsf_project_template.cmake)):
```
/module/
  /doc/              # Sphinx documentation
    /_conditional_section.txt  # one-liner: '* <feature name>' for RegisterFeature()
  /grammar/          # .ag grammar files
  /source/           # .hpp and .cpp source files
  /test/             # GTest unit tests
  /test_mission/     # test_*.txt auto-test scenarios
  /CMakeLists.txt
  /wsf_cmake_extension.cmake
  /wsf_module
```

**To add a new module**: create the above files in a new directory. The top-level [CMakeLists.txt](CMakeLists.txt) globs for `wsf_module` markers automatically. Gate it behind a `BUILD_WITH_<name>` CMake option in a preset file.

## Architecture

### Simulation Framework: Core vs Application vs Plugin

The architecture has three tiers:

1. **Core Libraries** (`core/`) — The simulation framework itself:
   - **wsf** (~1132 files, 204 top-level headers) — world simulation framework. Sub-modules in `source/`:
     - `mover/` — air/ground/surface/subsurface/space motion models
     - `sensor/` — radar, IR, EO sensing models
     - `comm/` — datalinks, networks, communication devices
     - `processor/` — onboard processors (behavior, FSM, behavior trees)
     - `script/` — scenario scripting language engine
     - `dis/` — Distributed Interactive Simulation protocol (IEEE 1278)
     - `xio/` / `xio_sim/` — External I/O (distributed simulation interconnect)
     - `event_pipe/` — internal event management
     - `observer/` — publish/subscribe data extraction
     - `traffic/` — air traffic management
     - `ext/` — extension interfaces
   - **wsf_util** — math, file I/O, strings, unit conversion, RNG
   - **wsf_parser** — scenario file parser (text → internal representation)
   - **wsf_cyber** — cyber warfare effects
   - **wsf_mil** / **wsf_l16** — MIL-STD / Link 16 datalink modeling
   - **wsf_space** — orbital mechanics, space sensors/platforms
   - **wsf_mtt** — multi-target tracking (track files, association, filtering)
   - **wsf_ripr** — tanker/refueling modeling
   - **wsf_weapon_server** — weapon engagement server
   - **wsf_nx** — network extension (HLA/DIS bridge)

2. **Applications** — Executables consuming the framework:
   - **mission** — mission-level simulation runner (simplest entry point)
   - **engage** — engagement-level with finer-grained physics
   - **warlock** — primary 3D visualization & simulation control (Qt + OSG, ~40 plugins)
   - **wizard** — scenario editing GUI (~1061 files, 30+ plugins)
   - **mystic** — after-action review / post-run analysis (~291 files, 45+ plugins)
   - **sensor_plot** — sensor coverage plotting tool
   - **weapon_tools** — weapons analysis & launch computer generation
   - **evt_reader** — event log viewer
   - **mover_creator** — aerodynamic model & propulsion editor GUI
   - **post_processor** — post-simulation data analysis

3. **Plugins** (`wsf_plugins/`) — Dynamically-loaded optional capabilities:
   - **wsf_oms_uci** (~8642 files) — Open Mission Systems / UCI interfaces (largest)
   - **wsf_air_combat** / **wsf_brawler** — air combat maneuvering behaviors
   - **wsf_six_dof** / **wsf_p6dof** — 6-DOF / partial 6-DOF motion
   - **wsf_iads_c2_lib** — Integrated Air Defense System C2
   - **wsf_fires** — artillery/missile coordination
   - **wsf_sosm** — System of Systems Model
   - **wsf_coverage** — sensor/weapon coverage analysis
   - **wsf_multiresolution** — multi-resolution modeling
   - **wsf_argo8** — Argo8 integration
   - **wsf_scenario_analyzer** — scenario analysis tools
   - **wsf_simdis** — SIMDIS integration
   - **wsf_annotation** — scenario annotation tools

### Tool Libraries

Support libraries in [tools/](tools/):
- **util** — core utilities: math, linear algebra, coordinate transforms, time, file I/O
- **wkf** — Widget/Kernel Framework (Qt widget library used by all GUI apps)
- **utilosg** — OpenSceneGraph utility wrappers (3D rendering)
- **dis** — IEEE 1278 DIS PDU encoding/decoding
- **genio** — Generic I/O for various file formats
- **packetio** — UDP/TCP socket communication
- **geodata** — geospatial data, map projections, coordinate systems
- **util_script** — Lua scripting bindings
- **utilqt** — Qt utility widgets
- **vespatk** — digital terrain & elevation data
- **artificer** — simulation controller/launcher (also parses WPA performance traces)
- **tracking_filters** — Kalman & tracking filters
- **profiling** — performance profiling infrastructure

## Key Framework Patterns

### Component-Based Architecture (CBA)

The core design pattern: **Platforms are containers of components**. Every platform (aircraft, ship, missile, satellite) is a `WsfPlatform` that owns a `WsfComponentList` holding its sensors, communication devices, movers, processors, weapons, etc.

- `WsfComponent` — base class for all components. Key virtual methods: `CloneComponent`, `ProcessInput`, `PreInitialize`, `Initialize`, `Initialize2`, `QueryInterface`, `GetComponentRoles`
- `WsfComponentList` — manages components by name or role, supports iteration, add/delete, and callbacks
- `QueryInterface(int aRole)` — type-discovery mechanism. Components advertise their type via `WSF_DECLARE_COMPONENT_ROLE_TYPE` and `GetComponentRoles()`
- `FindComponentByRole()` / `FindComponentByName()` — primary lookup mechanisms

New component types register via:
```cpp
WSF_DECLARE_COMPONENT_ROLE_TYPE(WsfMySensor, cWSF_COMPONENT_SENSOR)
```

### Script Extensibility (No-Compile Behavior)

AFSIM's **script system** allows defining platform behaviors without C++ recompilation:
- **Processors** (`WsfProcessor`) — script-defined behaviors using the AFSIM scripting language
- **Finite State Machines** — `WsfFsmComponent` for state-based behavior with transition rules
- **Behavior Trees** — `WsfBehaviorTree` / `WsfAdvancedBehaviorTree` for hierarchical behavior modeling
- **Script Observers** — extract data via `script` type output without modifying C++

### Plugin System

Plugins are DLLs loaded at runtime, registered via `wsf_project_template(PLUGIN)`:
- Feature registration: `aApp.RegisterFeature()` in conditional-section files
- Each plugin has `doc/_conditional_section.txt` with feature names
- IDE folder organization: plugins→`wsf_plugins`, libs→`core`, exes→`applications`

### Grammar System

- **Grammar files** (`.ag`): Define the AFSIM scenario command grammar
- Grammars are compiled at build time and installed to `bin/grammar/`
- Grammar files define the text commands users write in `.txt` scenario files
- See [`doc/developer/wsf_grammar_format.rst`](core/wsf/doc/developer/wsf_grammar_format.rst) for format details

### Event System

AFSIM is event-based (abstracted from analysts):
- Each simulation owns an `EventManager` — responsible for time-ordered processing
- Supports real-time and constructive (as-fast-as-possible) modes
- Events are the backbone of sensor updates, mover updates, comms, and task execution

### Tasking & Tracking

- **Tasking**: A service for sending track-associated task assignments. Uses finite state machines — tracks maintain per-track state with user-defined transition rules.
- **Tracking**: Native track association, filtering (Kalman, alpha-beta, alpha-beta-gamma), and fusion. Supports perfect and imperfect correlation.

### Observer Service

Publish-subscribe data extraction service:
- Creates `WsfObserver` instances that monitor simulation events
- Script-type observers allow custom output without C++ changes
- Used for standard output, replay files, and distributed interfaces

### Simulation Interfaces

- **DIS** (IEEE 1278) — Distributed Interactive Simulation PDU encoding/decoding in `tools/dis/`
- **HLA** (IEEE 1516) — High-Level Architecture via `wsf_nx`
- **XIO** — AFSIM's special-purpose distributed simulation interconnect

## Testing

### Auto Tests (Scenario-Based Integration Tests)

Run AFSIM scenarios against an application. Test files are `test_*.txt` in `test_mission/` directories.

```bash
# Run all auto tests for mission
cmake --build BUILDS/default-vs2019 --target mission_AUTO_TEST --config Release

# Run auto tests for a specific module (e.g., wsf sensor tests)
cmake --build BUILDS/default-vs2019 --target wsf_mission_AUTO_TEST --config Release
```

The Python test runner is `tools/misc/auto_test.py`.

### Unit Tests (GTest)

Unit test C++ source files live in `test/` subdirectories using Google Test 1.8.0.

```bash
# Build and run all unit tests
cmake --build BUILDS/default-vs2019 --target RUN_ALL_UNIT_TESTS --config Release

# Run a specific module's unit tests
cmake --build BUILDS/default-vs2019 --target wsf_UNIT_TEST --config Release
```

Unit test locations across the codebase:
- `core/wsf/test/`, `core/wsf_cyber/test/`, `core/wsf_mil/test/`, `core/wsf_space/test/`, `core/wsf_parser/test/`
- `tools/util/test/`, `tools/util_script/test/`, `tools/geodata/test/`, `tools/artificer/test/`
- `wizard/test/`, `wizard/lib/source/test/`, `wizard/plugins/EventOutput/test/`
- `wsf_plugins/wsf_air_combat/test/`, `wsf_plugins/wsf_coverage/test/`, `wsf_plugins/wsf_multiresolution/test/`, `wsf_plugins/wsf_p6dof/p6dof/test/`, `wsf_plugins/wsf_six_dof/test/`
- `post_processor/lib/test/`

### Regression Tests

Compare simulation outputs against baselines (requires demos repo with gold-standard output).

```bash
cmake --build BUILDS/default-vs2019 --target ${PROJECT_NAME}_REGRESSION_TEST --config Release
```

### Valgrind (Linux only)

```bash
cmake --build BUILDS/ninja --target mission_AUTO_TEST_VALGRIND --config Release
```

## Documentation

Documentation uses **Sphinx** (v4.4.0) with **myst-parser** (v0.16.1) for Markdown support. See `requirements.txt`.

```bash
cmake --build BUILDS/default-vs2019 --target DOCUMENTATION --config Release
cmake --build BUILDS/default-vs2019 --target DOXYGEN --config Release
```

Key developer docs in [doc/developer/](doc/developer/):
- [coding_standards.rst](doc/developer/coding_standards.rst) — extensive C++ coding standard
- [build_instructions.rst](doc/developer/build_instructions.rst) — detailed build guide with CMake options
- [integration_guide.rst](doc/developer/integration_guide.rst) — component architecture and new module integration
- [documentation_generation.rst](doc/developer/documentation_generation.rst)
- [contribution_process.md](doc/developer/contribution_process.md) — PR/commit requirements
- [performance_tools.rst](doc/developer/performance_tools.rst) — WPA, VTune, perf profiling

## Code Style & Formatting

- C++11 standard; `.hpp` for headers, `.cpp` for implementation
- Include guards: `#ifndef FILENAME_HPP` / `#define FILENAME_HPP` / `#endif` (never `#pragma once`)
- **Clang-format 9.0+** with `.clang-format` at the repo root
  - Auto-runs as a git pre-commit hook (generates `.patch` files for non-conformant formatting)
  - VS2019 ships clang-format 11.0 built-in; VS2017 requires manual binary override to 9.0+
- Include ordering: corresponding header first, then system headers (alphabetical), then local headers (alphabetical)
- Follow conventions of adjacent code; see [Coding Standards](doc/developer/coding_standards.rst) for detailed rules
- Doxygen comments for new public APIs
- **Cppcheck** static analysis: `cmake --build <build-dir> --target CPPCHECK`
- **REVIEWERS** files in each directory — assign reviewers from changed directories' REVIEWERS files for PRs

## Performance Analysis

- **WPA (Windows Performance Analyzer)** — recommended on Windows for profiling mission app execution
- **artificer** (`tools/artificer/`) — parses `afperf_*.txt` trace output files into summaries and visualizations
- **Linux**: `perf`, `callgrind`, `gprof`
- See [doc/developer/performance_tools.rst](doc/developer/performance_tools.rst) for detailed profiling guidance

## External Model Examples

Toy/demonstration plugins in [externModel/](externModel/):
- `sensor/` (sensor_exercise) — example sensor plugin
- `weapon/` (weapon_exercise) — example weapon plugin
- `ReconSensor/` (recon_sensor_exercise) — example reconnaissance sensor plugin

## Versioning

Set in [CMakeLists.txt](CMakeLists.txt) — currently v2.9.0 (release 2022-02-25).
