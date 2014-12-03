
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


#ifndef core_LogStreambuf_t
#define core_LogStreambuf_t

#include <algorithm>
#include <mutex>
#include <sstream>
#include <vector>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Template class definition

    template<typename ReceiverT>
    class LogStreambuf : public std::streambuf
    {
    public:
      LogStreambuf(ReceiverT&);
      ~LogStreambuf();

      virtual std::streamsize xsputn(const char* p, std::streamsize n);
      virtual int sync();
      virtual int overflow(int);

    private:
      char* begin();
      char* end();

      std::mutex m_mutex;

      std::vector<char> m_buffer;

      ReceiverT& m_rcvr;
    };


    ///////////////////////////////////////////////////////////////////////////
    // Member function definitions

    template<typename ReceiverT>
    LogStreambuf<ReceiverT>::LogStreambuf(ReceiverT& rcvr) :
      std::basic_streambuf<char, std::char_traits<char> >(),
      m_buffer(4),
      m_rcvr(rcvr)
    {
      /*
      std::cout << "ctor()" << std::endl;
      */
      // XXX(cpg) - use of the mutex here was causing deadlock during program
      //            initialization (after DebugStream class was moved to
      //            ato::core namespace); not using the lock here seems okay,
      //            but beware!
      /*
      std::lock_guard<std::mutex> lock(m_mutex);
      */

      setp(begin(), end());
    }

    template<typename ReceiverT>
    LogStreambuf<ReceiverT>::~LogStreambuf()
    {
      /*
      std::cout << "dtor()" << std::endl;
      */
      // XXX(cpg) - use of the mutex here was causing the program to crash on
      //            exit; not using the lock here seems okay, but beware!
      /*
      std::lock_guard<std::mutex> lock(m_mutex);
      */

      if (pptr() != begin())
      {
        sync();
      }
    }

    template<typename ReceiverT>
    std::streamsize LogStreambuf<ReceiverT>::xsputn(const char* p, std::streamsize n)
    {
      /*
      std::cout << "xsputn()" << std::endl;
      */
      std::lock_guard<std::mutex> lock(m_mutex);

      if (pptr() + n >= epptr())
      {
        unsigned offset = pptr() - pbase();
        m_buffer.resize(m_buffer.size() + n - (epptr() - pptr()));
        setp(begin(), end());
        pbump(offset);
      }

      std::copy(p, p+n, pptr());
      pbump(n);

      return n;
    }

    template<typename ReceiverT>
    int LogStreambuf<ReceiverT>::sync()
    {
      /*
      std::cout << "sync()" << std::endl;
      */
      std::lock_guard<std::mutex> lock(m_mutex);

      if (pptr() + 1 >= epptr())
      {
        m_buffer.push_back(0);
        setp(begin(), end());
      }
      else
      {
        *pptr() = 0;
      }

      std::streamsize n = pptr() - pbase();
      pbump(-n);

      // clog << begin() << flush;
      if (m_rcvr)
        m_rcvr.output(begin());

      return n;
    }

    template<typename ReceiverT>
    int LogStreambuf<ReceiverT>::overflow(int i)
    {
      /*
      std::cout << "overflow()" << std::endl;
      */
      std::lock_guard<std::mutex> lock(m_mutex);

      unsigned offset = pptr() - pbase();
      m_buffer.push_back(i);
      setp(begin(), end());
      pbump(offset + 1);

      return i;
    }

    template<typename ReceiverT>
    char* LogStreambuf<ReceiverT>::begin()
    {
      return &(m_buffer.front());
    }

    template<typename ReceiverT>
    char* LogStreambuf<ReceiverT>::end()
    {
      return ((&m_buffer.back()) + 1);
    }

  } // namespace core

} // namespace ato

#endif // core_LogStreambuf_t
