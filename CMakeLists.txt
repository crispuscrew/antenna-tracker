cmake_minimum_required(VERSION 3.16)
project(antenna-tracker CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# ── Asio (standalone, header-only) ───────────────────────────────────────────
FetchContent_Declare(asio
  GIT_REPOSITORY https://github.com/chriskohlhoff/asio
  GIT_TAG        asio-1-30-2
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(asio)

# ── libpredict v2.0.0 (SGP4) ─────────────────────────────────────────────────
# We use the manual populate→patch→add_subdirectory pattern instead of
# FetchContent_MakeAvailable because:
#   1. predict.h is generated (from predict.h.in) into the BINARY dir,
#      so the include path is ${libpredict_BINARY_DIR}/include.
#   2. libpredict unconditionally adds its tests/ subdir with -Werror, and
#      those tests fail on GCC 14 (deprecated std::ptr_fun / std::not1).
#      Patching CMakeLists.txt before add_subdirectory disables them.
FetchContent_Declare(libpredict
  GIT_REPOSITORY https://github.com/la1k/libpredict
  GIT_TAG        v2.0.0
  GIT_SHALLOW    TRUE)

FetchContent_GetProperties(libpredict)
if(NOT libpredict_POPULATED)
    FetchContent_Populate(libpredict)

    # Patch: comment out tests (deprecated C++ APIs break -Werror on GCC 14)
    file(READ "${libpredict_SOURCE_DIR}/CMakeLists.txt" _lp_cmake)
    string(REPLACE "add_subdirectory(tests)" "#add_subdirectory(tests)" _lp_cmake "${_lp_cmake}")
    string(REPLACE "enable_testing()"        "#enable_testing()"        _lp_cmake "${_lp_cmake}")
    string(REPLACE "add_subdirectory(travis)" "#add_subdirectory(travis)" _lp_cmake "${_lp_cmake}")
    file(WRITE "${libpredict_SOURCE_DIR}/CMakeLists.txt" "${_lp_cmake}")

    add_subdirectory(${libpredict_SOURCE_DIR} ${libpredict_BINARY_DIR})
endif()

# ── antenna-tracker executable ────────────────────────────────────────────────
set(SOURCES
    src/main.cpp
    src/protocol/vkaprotocol.cpp
    src/tracking/tle_tracker.cpp
    src/tracking/trajectory_planner.cpp
    src/controller/antenna_controller.cpp
)

add_executable(antenna-tracker ${SOURCES})

# predict.h is generated into the build dir (not source dir).
target_include_directories(antenna-tracker PRIVATE
    src
    ${asio_SOURCE_DIR}/asio/include
    ${libpredict_BINARY_DIR}/include
)

target_link_libraries(antenna-tracker PRIVATE predict)

target_compile_definitions(antenna-tracker PRIVATE
    ASIO_STANDALONE
    _GNU_SOURCE
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(antenna-tracker PRIVATE -Wall -Wextra -O2)
endif()
