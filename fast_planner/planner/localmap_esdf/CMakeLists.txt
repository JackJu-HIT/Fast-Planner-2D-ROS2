
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -std=c++17")
include_directories( 
    include 
)

find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

add_library( 
        local_map STATIC
        src/sdf_map.cpp 
    )


