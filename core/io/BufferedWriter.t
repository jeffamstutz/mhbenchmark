
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


#ifndef coreio_BufferedWriter_t
#define coreio_BufferedWriter_t

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

      using std::endl;
      using std::ostream;


      /////////////////////////////////////////////////////////////////////////
      // Class template definition

      /*! @class BufferedWriter
          @brief Write out information on a given type.
      */
      template<typename T, uint MaxSize, bool WriteSelf>
      class BufferedWriter
      {
      public:
        BufferedWriter(ostream& out) :
          m_out(out),
          m_size(0)
        {
          // no-op
        }

        ~BufferedWriter()
        {
          flush();
        }

        void init(bool binary)
        {
          m_binary = binary;
        }

        operator bool()
        {
          return m_out;
        }

        BufferedWriter& operator<<(const T& value)
        {
          write(value);
          return (*this);
        }

        BufferedWriter& operator<<(std::ostream& (*manip)(std::ostream&))
        {
          manip(m_out);
          return (*this);
        }

        void write(const T& value)
        {
          if (m_size < MaxSize)
            m_buffer[m_size++] = value;

          if (m_size == MaxSize)
            flush();
        }

        void flush()
        {
          if (m_binary && m_size*sizeof(T) != 0)
            m_out.write(reinterpret_cast<const char*>(m_buffer), m_size*sizeof(T));
          else
          {
            for (uint i = 0; i < m_size; ++i)
              m_out << m_buffer[i] << endl;
          }

          m_size = 0;
        }

      private:
        ostream& m_out;
        bool     m_binary;

        T    m_buffer[MaxSize];
        uint m_size;
      };

    } // namespace io

  } // namespace core

} // namespace ato

#endif // coreio_BufferedWriter_t
