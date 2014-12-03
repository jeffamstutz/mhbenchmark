
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2012, 2014 Christiaan Gribble <cgribble[]rtvtk org>
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


#ifndef core_Configuration_h
#define core_Configuration_h

#include <iosfwd>
#include <map>
#include <string>

#include <core/ato_coreExports.h>
#include <core/Exception.h>
#include <core/Types.h>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::ifstream;
    using std::istream;
    using std::map;
    using std::ofstream;
    using std::ostream;
    using std::string;


    ///////////////////////////////////////////////////////////////////////////
    // Class definition

    class ATO_CORE_EXPORT Configuration
    {
    public:

      /////////////////////////////////////////////////////////////////////////
      // Static data member declarations

      static const string s_FileNotFound;


      /////////////////////////////////////////////////////////////////////////
      // Exception classes

      class FileNotFound : public Exception
      {
      public:
        FileNotFound(const string& file, const string& mode) :
          Exception("Failed to open \"" + file + "\" for " + mode)
        {
          // no-op
        }

        virtual ~FileNotFound() throw()
        {
          // no-op
        }

      private:
        // none
      };

      class ValueNotFound : public Exception
      {
      public:
        ValueNotFound(const string& key) :
          Exception("Value associated with \"" + key + "\" not found")
        {
          // no-op
        }

        virtual ~ValueNotFound() throw()
        {
          // no-op
        }

      private:
        // none
      };


      /////////////////////////////////////////////////////////////////////////
      // Constructor

      Configuration(const string&, const string&, const string& = "");


      /////////////////////////////////////////////////////////////////////////
      // Mutators

      virtual void flag(uint flag);
      virtual void option(const string&, const string&);


      /////////////////////////////////////////////////////////////////////////
      // Accessors

      virtual uint flags() const;

      template<typename T>
      T option(const string& key) const;


      /////////////////////////////////////////////////////////////////////////
      // Core functionality

      virtual void load(bool = false);
      virtual void save(const string&, const string&, const string&);


      /////////////////////////////////////////////////////////////////////////
      // Friend function declarations

      friend ATO_CORE_EXPORT ostream& operator<<(ostream&, const Configuration&);

      template<typename T>
      friend void parseOption(istream&, string&, string&, T&);

    protected:
      virtual void parse(ifstream&)                = 0;
      virtual void write(ofstream&, const string&) = 0;

      string m_directory;
      string m_filename;
      string m_default;

      uint m_flags;

      map<string, string> m_opts;

    private:
      string makeConfigPath() const;
    };


    ///////////////////////////////////////////////////////////////////////////
    // Inline member function definitions

    inline void Configuration::flag(uint flag)
    {
      m_flags |= flag;
    }

    inline void Configuration::option(const string& key, const string& value)
    {
      m_opts[key] = value;
    }

    inline uint Configuration::flags() const
    {
      return m_flags;
    }


    ///////////////////////////////////////////////////////////////////////////
    // Template member definitions

    template<typename T>
    T Configuration::option(const string& key) const
    {
      map<string, string>::const_iterator iter = m_opts.find(key);
      if (iter != m_opts.end())
        return T(iter->second);

      throw ValueNotFound(key);
    }


    ///////////////////////////////////////////////////////////////////////////
    // Template specializations

    template<>
    inline double Configuration::option(const string& key) const
    {
      return atof(option<string>(key).c_str());
    }


    ///////////////////////////////////////////////////////////////////////////
    // Explicit instantiations

    template string Configuration::option(const string&) const;


    ///////////////////////////////////////////////////////////////////////////
    // Template definitions

    template<typename T>
    void parseOption(istream& /* in */,
                     string&  key,
                     string&  value,
                     T&       config)
    {
      config.m_opts[key] = value;
    }

  } // namespace core

} // namespace ato

#endif // core_Configuration_h
