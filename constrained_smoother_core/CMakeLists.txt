cmake_minimum_required(VERSION 3.20)
project(constrained_smoother_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
include(CMakePackageConfigHelpers)

# Ceres 2.2 requests Eigen 3.3 through find_package(), which rejects newer
# Homebrew Eigen releases such as 5.x. Pin Eigen to a known Ceres-compatible
# 3.x release and expose a minimal package config for Ceres to consume.
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  SOURCE_SUBDIR cmake-no-op
)
FetchContent_MakeAvailable(eigen)
FetchContent_GetProperties(eigen)

set(_csc_eigen3_config_dir "${CMAKE_BINARY_DIR}/eigen3-config")
file(MAKE_DIRECTORY "${_csc_eigen3_config_dir}")
file(WRITE "${_csc_eigen3_config_dir}/Eigen3Config.cmake"
  "set(Eigen3_VERSION 3.4.0)\n"
  "set(EIGEN3_VERSION_STRING 3.4.0)\n"
  "if(NOT TARGET Eigen3::Eigen)\n"
  "  add_library(Eigen3::Eigen INTERFACE IMPORTED)\n"
  "  set_target_properties(Eigen3::Eigen PROPERTIES\n"
  "    INTERFACE_INCLUDE_DIRECTORIES \"${eigen_SOURCE_DIR}\"\n"
  "  )\n"
  "endif()\n"
  "set(Eigen3_FOUND TRUE)\n"
)
write_basic_package_version_file(
  "${_csc_eigen3_config_dir}/Eigen3ConfigVersion.cmake"
  VERSION 3.4.0
  COMPATIBILITY AnyNewerVersion
)
set(Eigen3_DIR "${_csc_eigen3_config_dir}" CACHE PATH "Eigen3 package config directory" FORCE)

# Ceres (and transitive dependencies) via FetchContent.
FetchContent_Declare(
  ceres
  GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
  GIT_TAG 2.2.0
)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(CERES_THREADING_MODEL CXX_THREADS CACHE STRING "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(ceres)

add_library(${PROJECT_NAME}
  src/smoother.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ceres
)

add_executable(constrained_smoother_example src/example_main.cpp)
target_link_libraries(constrained_smoother_example PRIVATE ${PROJECT_NAME})

option(CSC_BUILD_TESTS "Build tests" ON)
if(CSC_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
  )
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(test_geometry tests/test_geometry.cpp)
  target_link_libraries(test_geometry PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  add_test(NAME test_geometry COMMAND test_geometry)

  add_executable(test_smoother tests/test_smoother.cpp)
  target_link_libraries(test_smoother PRIVATE ${PROJECT_NAME} GTest::gtest_main)
  add_test(NAME test_smoother COMMAND test_smoother)
endif()
