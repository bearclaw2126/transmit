cmake_minimum_required(VERSION 3.5.1)
project(transmitter)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Find UHD package
find_package(UHD 4.5.0 REQUIRED)

# Include UHD and Boost directories
include_directories(${UHD_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
LINK_DIRECTORIES(/usr/lib/x86_64-linux-gnu)
SET(CMAKE_CXX_FLAGS "-lfftw3_threads -lfftw3 -lm")
SET(CMAKE_C_FLAGS "-lfftw3_threads -lfftw3 -lm")
include_directories(${UHD_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${FFTW_INCLUDE_DIRS} src) 
link_directories(${Boost_LIBRARY_DIRS})
# Add executable
add_executable(receptor src/transmit.cpp)
target_link_libraries(receptor fftw3)

# Link against UHD and Boost libraries
target_link_libraries(receptor ${UHD_LIBRARIES} ${Boost_LIBRARIES} ${FFTW_LIBRARIES})