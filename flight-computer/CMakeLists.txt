cmake_minimum_required(VERSION 3.31)
project(FlightComputer)

# Recursive globbing (includes subfolders)
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Add the found files to your executable or library
add_executable(FlightComputer ${SOURCES})