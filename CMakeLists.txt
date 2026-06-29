cmake_minimum_required(VERSION 3.16)
project(rxs-cc-plugins LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(RXS_BUILD_MESH2CAD "Build mesh2cad utility" ON)
option(RXS_BUILD_MESHSLICE "Build meshSlice utility" ON)

if(RXS_BUILD_MESH2CAD)
    add_subdirectory(mesh2cad)
endif()
if(RXS_BUILD_MESHSLICE)
    add_subdirectory(meshSlice)
endif()
