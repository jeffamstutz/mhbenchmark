
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of SURVICE Engineering Company nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef core_ato_coreExports_h
#define core_ato_coreExports_h

#ifdef WIN32
#  ifdef ATO_CORE_STATIC_DEFINE
#    define ATO_CORE_EXPORT
#    define ATO_CORE_NO_EXPORT
#  else
#    ifndef ATO_CORE_EXPORT
#      ifdef ato_core_EXPORTS
         /* We are building this library */
#        define ATO_CORE_EXPORT __declspec(dllexport)
#      else
         /* We are using this library */
#        define ATO_CORE_EXPORT __declspec(dllimport)
#      endif
#    endif

#    ifndef ATO_CORE_NO_EXPORT
#      define ATO_CORE_NO_EXPORT
#    endif
#  endif

#  ifndef ATO_CORE_DEPRECATED
#    define ATO_CORE_DEPRECATED __declspec(deprecated)
#    define ATO_CORE_DEPRECATED_EXPORT ATO_CORE_EXPORT __declspec(deprecated)
#    define ATO_CORE_DEPRECATED_NO_EXPORT ATO_CORE_NO_EXPORT __declspec(deprecated)
#  endif

#  define DEFINE_NO_DEPRECATED 0
#  if DEFINE_NO_DEPRECATED
#    define ATO_CORE_NO_DEPRECATED
#  endif
#else

#  define ATO_CORE_EXPORT
#  define ATO_CORE_NO_EXPORT
#  define ATO_CORE_DEPRECATED
#  define ATO_CORE_DEPRECATED_EXPORT
#  define ATO_CORE_DEPRECATED_NO_EXPORT

#endif // WIN32

#endif // core_ato_coreExports_h
