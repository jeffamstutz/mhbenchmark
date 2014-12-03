
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


#ifndef coreio_StreamManip_h
#define coreio_StreamManip_h

#include <iostream>

// #include <core/ato_coreExports.h>
#include <core/Types.h>


namespace ato
{

  namespace core
  {

    namespace io
    {

      /////////////////////////////////////////////////////////////////////////
      // Class definitions

      // NOTE(cpg) - export unnecessary for header-only implementation...
      class /*ATO_CORE_EXPORT*/ pad
      {
      public:
        pad(uint, char = ' ');

        std::ostream& operator()(std::ostream&) const;

      private:
        uint m_n;
        char m_c;
      };


      /////////////////////////////////////////////////////////////////////////
      // Inline member fuction definitions

      inline pad::pad(uint n, char c) :
        m_n(n),
        m_c(c)
      {
        // no-op
      }

      inline std::ostream& pad::operator()(std::ostream& out) const
      {
        for (uint i = 0; i < m_n; ++i)
          out << m_c;
        return out;
      }

      inline std::ostream& operator<<(std::ostream& out, const pad& p)
      {
        return p(out);
      }

    } // namespace io

  } // namespace core

} // namespace ato

#endif // coreio_StreamManip_h
