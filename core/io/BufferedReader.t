
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2012, Grove City College
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
 *  * Neither the name of Grove City College nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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


#ifndef coreio_BufferedReader_t
#define coreio_BufferedReader_t

#include <iostream>

#include <core/Types.h>

namespace ato
{

  namespace core
  {

    namespace io
    {

      /////////////////////////////////////////////////////////////////////////
      // Using declarations

      using std::istream;
      using std::streampos;


      /////////////////////////////////////////////////////////////////////////
      // Class template definition

      /*! @class BufferedReader
          @brief Read in information to a given type.
      */
      template<typename T, uint MaxSize, bool ReadSelf>
      class BufferedReader
      {
      public:
        BufferedReader(istream& in) :
          m_in(in),
          m_binary(false),
          m_nelts(0),
          m_idx(0),
          m_good(true)
        {
          // no-op
        }

        ~BufferedReader()
        {
          // no-op
        }

        void init(bool binary)
        {
          m_binary = binary;
        }

        operator bool()
        {
          return m_good;
        }

        void clear()
        {
          m_good = true;
        }

        BufferedReader& operator>>(T& value)
        {
          read(value);
          return (*this);
        }

        bool read(T& value)
        {
          if (m_idx == m_nelts || m_in.tellg() != m_pos)
          {
            if (slurp())
            {
              value = m_buffer[m_idx++];
              return true;
            }

            return false;
          }

          value = m_buffer[m_idx++];
          return true;
        }

        bool slurp()
        {
          m_good  = false;
          m_nelts = 0;
          m_idx   = 0;
          if (m_binary)
          {
            m_in.read(reinterpret_cast<char*>(m_buffer), MaxSize*sizeof(T));
            m_nelts = m_in.gcount()/sizeof(T);
          }
          else
          {
            while (m_nelts < MaxSize && m_in >> m_buffer[m_nelts])
              ++m_nelts;
          }

          m_in.clear();
          m_pos = m_in.tellg();

          m_good = (m_nelts > 0);
          return m_good;
        }

      private:
        istream&  m_in;
        bool      m_binary;

        T    m_buffer[MaxSize];
        uint m_nelts;
        uint m_idx;

        streampos m_pos;

        bool m_good;
      };

    } // namespace io

  } // namespace core

} // namespace ato

#endif // coreio_BufferedReader_t
