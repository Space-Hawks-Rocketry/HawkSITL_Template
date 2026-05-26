cmake_minimum_required(VERSION 3.31)
project(FlightComputer)

set(CMAKE_CXX_STANDARD 20)        # Options: 11, 14, 17, 20, 23, 26
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Forces error if the compiler doesn't support it

# Recursive globbing (includes subfolders)
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Add the found files to your executable or library
add_executable(FlightComputer ${SOURCES})

# Add necessary libraries
target_link_libraries(FlightComputer PRIVATE zmq)

# Add the include directory
target_include_directories(FlightComputer PUBLIC include)