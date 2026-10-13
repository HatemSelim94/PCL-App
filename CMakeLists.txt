cmake_minimum_required(VERSION 3.16)

# C is enabled next to C++ on purpose: on Ubuntu, PCL's package config pulls
# in VTK, and VTK's MPI dependency (MPI::MPI_C) only exists when C is enabled.
project(PclApp VERSION 2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(PCLAPP_BUILD_APP   "Build the Qt desktop application" ON)
option(PCLAPP_BUILD_TESTS "Build the unit tests (fetches Catch2)" ON)

# ---------------------------------------------------------------------------
# Processing core: header-only, Qt-free, so it builds and tests on a plain
# CI runner with nothing but PCL installed.
# ---------------------------------------------------------------------------
set(PCLAPP_PCL_COMPONENTS common io filters search segmentation)
if(PCLAPP_BUILD_APP)
  list(APPEND PCLAPP_PCL_COMPONENTS visualization)
endif()
find_package(PCL 1.11 REQUIRED COMPONENTS ${PCLAPP_PCL_COMPONENTS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(pclapp_core INTERFACE)
target_include_directories(pclapp_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
target_link_libraries(pclapp_core INTERFACE ${PCL_LIBRARIES})
target_compile_definitions(pclapp_core INTERFACE ${PCL_DEFINITIONS})

# Warning level for our own translation units (not forced on consumers).
add_library(pclapp_warnings INTERFACE)
target_compile_options(pclapp_warnings INTERFACE
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>)

# ---------------------------------------------------------------------------
# Desktop application (Qt5 Widgets + VTK 8 QVTKWidget)
# ---------------------------------------------------------------------------
if(PCLAPP_BUILD_APP)
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  find_package(VTK REQUIRED)
  if(VTK_VERSION VERSION_LESS 9 AND VTK_USE_FILE)
    include(${VTK_USE_FILE})
  endif()

  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTOUIC ON)

  add_executable(PclApp
    app/main.cpp
    app/mainwindow.cpp
    app/mainwindow.h
    app/mainwindow.ui)
  target_link_libraries(PclApp PRIVATE pclapp_core pclapp_warnings Qt5::Widgets ${VTK_LIBRARIES})
endif()

# ---------------------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------------------
if(PCLAPP_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.8.0)
  FetchContent_MakeAvailable(Catch2)

  add_executable(pclapp_tests tests/core_tests.cpp)
  target_link_libraries(pclapp_tests PRIVATE pclapp_core pclapp_warnings Catch2::Catch2WithMain)

  enable_testing()
  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
  include(Catch)
  catch_discover_tests(pclapp_tests)
endif()
