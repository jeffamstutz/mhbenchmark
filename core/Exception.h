
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


#ifndef core_Exception_h
#define core_Exception_h

#include <exception>
#include <string>

#include <core/ato_coreExports.h>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::exception;
    using std::string;


    ///////////////////////////////////////////////////////////////////////////
    // Class definitions

    class ATO_CORE_EXPORT Exception : public exception
    {
    public:
      Exception(const string&);
      virtual ~Exception() throw();

      virtual const char* what() const throw();

    protected:
      const string m_msg;
    };

    class ATO_CORE_EXPORT FunctionNotImplemented : public Exception
    {
    public:
      FunctionNotImplemented(const string&);
      virtual ~FunctionNotImplemented() throw();
    };


    ///////////////////////////////////////////////////////////////////////////
    // Inline member function definitions

    inline Exception::Exception(const string& msg) :
      m_msg(msg)
    {
      // no-op
    }

    inline Exception::~Exception()
    {
      // no-op
    }

    inline const char* Exception::what() const throw()
    {
      return m_msg.c_str();
    }

    inline FunctionNotImplemented::FunctionNotImplemented(const string& name) :
      Exception("Function \"" + name + "\" not implemented")
    {
      // no-op
    }

    inline FunctionNotImplemented::~FunctionNotImplemented()
    {
      // no-op
    }

  } // namespace core

} // namespace ato

#endif // core_Exception_h
