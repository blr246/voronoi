cmake_minimum_required(VERSION 2.8)

file(GLOB HEADERS "*.h")

# Enable OpenMP in UNIX.
# Enable multicore builds in MSVC.
find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
if(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
else(UNIX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif(UNIX)

# Library targets:
#   voronoi_core - shared objects for voronoi
project(voronoi_core)
set(SRCS
    "voronoi_diagram_generator.cpp"
    "voronoi_core.cpp")
add_library(voronoi_core STATIC ${SRCS} ${HEADERS})

# Executable targets:
#   voronoi - the main solution
#   voronoi_gtest - all tests

project(voronoi)
set(SRCS
    "voronoi.cpp")
add_executable(voronoi ${SRCS} ${HEADERS})
target_link_libraries(voronoi voronoi_core)
if(WIN32)
	target_link_libraries(voronoi Ws2_32)
endif(WIN32)

if(HPS_GTEST_ENABLED)
  project(voronoi_gtest)
  set(SRCS
      "voronoi_gtest.cpp")
  include_directories(${GTEST_INCLUDE_DIRS})
  add_executable(voronoi_gtest ${SRCS} ${HEADERS})
  target_link_libraries(voronoi_gtest voronoi_core gtest)
  add_test(voronoi_gtest voronoi_gtest)
endif(HPS_GTEST_ENABLED)

project(voronoi_all)
