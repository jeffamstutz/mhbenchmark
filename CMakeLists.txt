
###############################################################################
# Copyright (c) 2014, SURVICE Engineering Company
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in
#     the documentation and/or other materials provided with the
#     distribution.
#
#   * Neither the name of SURVICE Engineering Company nor the names of
#     its contributors may be used to endorse or promote products
#     derived from this software without specific prior written
#     permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

project(mhbenchmark)


###############################################################################
# mhbenchmark prerequisites

cmake_minimum_required(VERSION 3.0)

find_package(Qt5Gui    REQUIRED)

set(OSPRAY_BUILD_DIRECTORY ${CMAKE_SOURCE_DIR}/../ospray/build CACHE PATH "Path to OSPRay build directory")
set(OSPRAY_SOURCE_DIRECTORY ${CMAKE_SOURCE_DIR}/../ospray CACHE PATH "Path to OSPRay source directory")


###############################################################################
# mhbenchmark-specific build options

set(CMAKE_AUTOMOC             ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_FLAGS "-std=c++11 -fopenmp ${CMAKE_CXX_FLAGS}")


###############################################################################
# mhbenchmark executable

set(EXEC_NAME benchmark)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

add_definitions(
  ${Qt5Gui_DEFINITIONS}
  -DOSPRAY_MODULES_DIR=${OSPRAY_BUILD_DIRECTORY}
)

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_LIST_DIR}
  ${OSPRAY_SOURCE_DIRECTORY}
  ${OSPRAY_SOURCE_DIRECTORY}/ospray
  ${OSPRAY_SOURCE_DIRECTORY}/ospray/embree/common
  ${OSPRAY_SOURCE_DIRECTORY}/ospray/include
)

add_subdirectory(core)
add_subdirectory(math)

add_executable(${EXEC_NAME}
  main.cc
  save_image.cc
)

target_link_libraries(${EXEC_NAME}
  Qt5::Gui
  ${OSPRAY_BUILD_DIRECTORY}/libospray.so
  ${OSPRAY_BUILD_DIRECTORY}/libospray_embree.so
  ato_core
  ato_math
)


###############################################################################
# 'make install' target

set(HEADER_DIR  include/libato)
set(RUNTIME_DIR bin)
set(LIBRARY_DIR lib)
set(ARCHIVE_DIR lib)

install(TARGETS ${EXEC_NAME}
  RUNTIME DESTINATION ${RUNTIME_DIR}
  LIBRARY DESTINATION ${LIBRARY_DIR}
  ARCHIVE DESTINATION ${ARCHIVE_DIR}
)

