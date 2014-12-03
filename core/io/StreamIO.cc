
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


#include <iostream>

#include <core/io/StreamIO.h>


namespace ato
{

  namespace core
  {

    namespace io
    {

      /////////////////////////////////////////////////////////////////////////
      // Stream I/O object definitions

      std::ostream cout(std::cout.rdbuf());
      std::ostream cerr(std::cerr.rdbuf());
      std::ostream clog(std::clog.rdbuf());


      /////////////////////////////////////////////////////////////////////////
      // Stream I/O configuration function definitions

      void setDefaultOut(std::ostream& out)
      {
        cout.rdbuf(out.rdbuf());
      }

      void setDefaultOut(std::streambuf& outbuf)
      {
        cout.rdbuf(&outbuf);
      }

      void setDefaultErr(std::ostream& err)
      {
        cerr.rdbuf(err.rdbuf());
      }

      void setDefaultErr(std::streambuf& errbuf)
      {
        cerr.rdbuf(&errbuf);
      }

      void setDefaultLog(std::ostream& log)
      {
        clog.rdbuf(log.rdbuf());
      }

      void setDefaultLog(std::streambuf& logbuf)
      {
        clog.rdbuf(&logbuf);
      }

    } // namespace io

  } // namespace core

} // namespace ato
