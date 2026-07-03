cmake_minimum_required(VERSION 3.16)
project(physics_sandbox LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# ---- Locate Bullet ----------------------------------------------------------
# Prefer CMake's find_package; fall back to pkg-config (Debian/Ubuntu). We link
# the single-precision libraries, so BT_USE_DOUBLE_PRECISION must NOT be defined.
set(BULLET_TARGET_INCLUDES "")
set(BULLET_TARGET_LIBS "")

find_package(Bullet QUIET)
if(BULLET_FOUND)
  message(STATUS "Found Bullet via find_package")
  set(BULLET_TARGET_INCLUDES ${BULLET_INCLUDE_DIRS})
  set(BULLET_TARGET_LIBS ${BULLET_LIBRARIES})
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(BULLET REQUIRED bullet)
  message(STATUS "Found Bullet via pkg-config: ${BULLET_LIBRARIES}")
  set(BULLET_TARGET_INCLUDES ${BULLET_INCLUDE_DIRS})
  set(BULLET_TARGET_LIBS ${BULLET_LIBRARIES})
endif()

# ---- Engine library ---------------------------------------------------------
add_library(sandbox_core STATIC
  src/RigidBodyFactory.cpp
  src/PhysicsWorld.cpp
  src/RayCaster.cpp
  src/DebugDrawer.cpp
  src/TrajectoryLogger.cpp
  src/Metrics.cpp
  src/Scenarios.cpp
)
target_include_directories(sandbox_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${BULLET_TARGET_INCLUDES}
)
target_link_libraries(sandbox_core PUBLIC ${BULLET_TARGET_LIBS})
target_link_directories(sandbox_core PUBLIC ${BULLET_LIBRARY_DIRS})

# ---- CLI driver -------------------------------------------------------------
add_executable(sandbox src/main.cpp)
target_link_libraries(sandbox PRIVATE sandbox_core)

# ---- Tests ------------------------------------------------------------------
enable_testing()
add_executable(sandbox_tests tests/physics_tests.cpp)
target_link_libraries(sandbox_tests PRIVATE sandbox_core)
add_test(NAME physics_tests COMMAND sandbox_tests)
