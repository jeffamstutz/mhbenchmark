
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2012, 2013, Christiaan Gribble <cgribble[]rtvtk org>
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


#ifndef coreio_Parser_t
#define coreio_Parser_t

#include <iostream>
#include <map>
#include <string>


namespace ato
{

  namespace core
  {

    namespace io
    {

      /////////////////////////////////////////////////////////////////////////
      // Using declarations

      using std::istream;
      using std::map;
      using std::string;


      /////////////////////////////////////////////////////////////////////////
      // Helper function definition

      template<typename T>
      void ignore(istream&, string&, string&, T&)
      {
        // no-op
      }


      /////////////////////////////////////////////////////////////////////////
      // Class template definition

      template<typename T>
      class Parser
      {
      public:
        typedef void (*ParseFcn)(istream&, string&, string&, T&);

        Parser(const string&  ignore  = "",
               const string&  token   = "",
               const string&  comment = "",
                     ParseFcn dflt    = &ato::core::io::ignore) :
          m_ignore(ignore),
          m_token(token),
          m_comment(comment)
        {
          m_fcns["default"] = dflt;
        }

        void ignore (const string& ignore ) { m_ignore  = ignore;  }
        void token  (const string& token  ) { m_token   = token;   }
        void comment(const string& comment) { m_comment = comment; }

        void handler(const string& key, ParseFcn fcn)
        {
          m_fcns[key] = fcn;
        }

        void parse(istream& in, string& line, T& rcvr)
        {
          // Tokenize 'line' based on first occurrence of 'm_token' characters
          const size_t pos   = line.find_first_of(m_token);
                string key   = line.substr(0, pos);
                string value = line.substr(pos+1);

          trim(key);
          trim(value);

          const string& idx = (m_fcns.find(key) == m_fcns.end() ? "default" :
                               key);

          m_fcns[idx](in, key, value, rcvr);
        }

      private:
        void trim(string& line) const
        {
          try
          {
            // Trim from start up to first occurrence of 'm_comment'
            //   characters
            line = line.substr(0, line.find(m_comment));

            // Eat leading and trailing 'm_ignore' characters
            const size_t left  = line.find_first_not_of(m_ignore);
            const size_t right = line.find_last_not_of(m_ignore);
            line = line.substr(left, right-left+1);
          }
          catch (...)
          {
            line = "";
          }
        }

        string m_ignore;
        string m_token;
        string m_comment;

        map<string, ParseFcn> m_fcns;
      };

    } // namespace io

  } // namespace core

} // namespace ato

#endif // coreio_Parser_t
