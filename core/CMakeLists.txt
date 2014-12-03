
###############################################################################
# Copyright (c) 2014, SURVICE Engineering Company
# Copyright (c) 2014, Christiaan Gribble <cgribble[]rtvtk org>
# Copyright (c) 2012, Grove City College
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
#   * Neither the name of Grove City College nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
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


###############################################################################
# libato core library

set(LIBRARY_NAME ato_core)

set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)


###############################################################################
# namespace core::io

# directory 'core/io'
set(COREIO_HEADERS
  io/BufferedReader.t
  io/BufferedWriter.t
  io/Parser.t
  io/StreamIO.h
  io/StreamManip.h
  )

set(COREIO_SOURCES
  io/StreamIO.cc
  )

  
###############################################################################
# namespace core

set(CORE_HEADERS
  ato_coreExports.h
  BBox.h
  Configuration.h
  ColorMap.t
  Exception.h
  LogStreambuf.t
  Mesh.h
  MinMax.t
  RGB.t
  TexCoord.h
  Texture.h
  TileQueue.h
  Timer.h
  Types.h
  UniformGrid.t
  Utility.h
  Volume3D.t
  )

set(CORE_SOURCES
  BBox.cc
  ColorMap.cc
  Configuration.cc
  Mesh.cc
  RGB.cc
  Texture.cc
  Timer.cc
  )


###############################################################################
# Finish core library

source_group("core::io Header files" FILES ${COREIO_HEADERS})
source_group("core::io Source files" FILES ${COREIO_SOURCES})

source_group("core Header files" FILES ${CORE_HEADERS})
source_group("core Source files" FILES ${CORE_SOURCES})

include_directories(
  )

add_library(${LIBRARY_NAME}
  ${COREIO_HEADERS}
  ${COREIO_SOURCES}
  ${CORE_HEADERS}
  ${CORE_SOURCES}
  )

