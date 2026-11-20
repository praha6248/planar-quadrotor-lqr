cmake_minimum_required(VERSION 3.15)
project(PlanarQuadrotorLQR CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(PQ_BUILD_APP   "Build the SDL2 simulator (needs the git submodules)" ON)
option(PQ_BUILD_TESTS "Build unit tests (GoogleTest)"                       OFF)

include(cmake/CPM.cmake)

CPMAddPackage(
  NAME Eigen
  VERSION 3.4.0
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  # Eigen's CMakelists are not intended for library use
  DOWNLOAD_ONLY YES
)

if(Eigen_ADDED)
  add_library(Eigen INTERFACE IMPORTED)
  target_include_directories(Eigen INTERFACE ${Eigen_SOURCE_DIR})
endif()

# --- Model + controller (no graphics), shared by the simulator and the tests ---
add_library(quadrotor_core STATIC PlanarQuadrotor/planar_quadrotor.cpp PlanarQuadrotor/lqr.cpp)
target_include_directories(quadrotor_core PUBLIC ${PROJECT_SOURCE_DIR}/PlanarQuadrotor)
target_link_libraries(quadrotor_core PUBLIC Eigen)

# --- Interactive simulator ---
if(PQ_BUILD_APP)
  add_subdirectory(matplotplusplus)
  add_subdirectory(SDL)
  add_subdirectory(SDL2_gfx)

  set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")

  add_executable(planar_quadrotor PlanarQuadrotor/simulate.cpp PlanarQuadrotor/planar_quadrotor_visualizer.cpp)
  if(TARGET SDL2::SDL2main)
      # It has an implicit dependency on SDL2 functions, so it MUST be added before SDL2::SDL2 (or SDL2::SDL2-static)
      target_link_libraries(planar_quadrotor SDL2::SDL2main)
  endif()
  target_link_libraries(planar_quadrotor quadrotor_core SDL2::SDL2 SDL2_gfx matplot)
  target_include_directories(planar_quadrotor PRIVATE ${PROJECT_SOURCE_DIR})
endif()

# --- Unit tests: cmake -DPQ_BUILD_APP=OFF -DPQ_BUILD_TESTS=ON, then ctest ---
if(PQ_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
    GIT_SHALLOW TRUE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(quadrotor_tests tests/test_quadrotor.cpp)
  target_link_libraries(quadrotor_tests PRIVATE quadrotor_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(quadrotor_tests)
endif()
