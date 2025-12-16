project (Multi_objective_search)


cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET (CMAKE_LINKER  "/usr/bin/ld")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall")

include_directories("inc")

file(GLOB SOURCES "src/BiCriteria/*.cpp" "src/Utils/*.cpp")

# Find Boost
find_package( Boost REQUIRED COMPONENTS program_options log thread filesystem system)
include_directories( ${Boost_INCLUDE_DIRS} )


message(${CMAKE_BINARY_DIR})

add_executable(multiobj src/driver.cpp ${SOURCES})
target_link_libraries(multiobj ${Boost_LIBRARIES})

# Preprocessing tool for obstacle uncertainty problems
add_executable(preprocess_graph src/preprocess_graph.cpp)
set_target_properties(preprocess_graph PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    COMPILE_FLAGS "-O3"
)