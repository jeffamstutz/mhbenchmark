
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2013, Christiaan Gribble <cgribble[]rtvtk org>
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
 *  * Neither the name of the author nor the names of its contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
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


#ifndef core_Utility_h
#define core_Utility_h

#include <sstream>
#include <string>

#ifdef WIN32
#  include <Shlwapi.h>
#  define absolutePath(x) (!PathIsRelative(x.c_str()))
#else
#  define absolutePath(x) ((x[0] == '/'))
#endif // WIN32


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::ostringstream;
    using std::string;


    ///////////////////////////////////////////////////////////////////////////
    // Helper function definitions

#ifdef WIN32
    inline float drand48()
    {
      return static_cast<float>(rand())/RAND_MAX;
    }

    inline float srand48(uint seed)
    {
      srand(seed);
    }
#else
    inline float drand48()
    {
      return ::drand48();
    }

    inline float srand48(long int seed)
    {
      ::srand48(seed);
    }
#endif // WIN32

    inline void matchDirectory(string& fname, const string& full)
    {
      string result;
#if defined(WIN32)
      // XXX(cpg) - some Windows paths use '\' while others use '/'
      /*
      size_t lastSlash = full.rfind('\\');
      if (lastSlash == string::npos)
        result = ".\\";
      */
      size_t lastSlash = full.rfind('/');
      if (lastSlash == string::npos)
        result = "./";
#else
      size_t lastSlash = full.rfind('/');
      if (lastSlash == string::npos)
        result = "./";
#endif // defined(WIN32)
      else
        result = full.substr(0, lastSlash + 1);

      result += fname;
      fname   = result;
    }

    template<typename T>
    string to_string(const T& value)
    {
      static ostringstream oss;
      oss.str("");
      oss << value;
      return oss.str();
    }

  } // namespace core

} // namespace ato

#endif // core_Utility_h
