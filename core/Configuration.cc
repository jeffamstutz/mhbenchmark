
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

#include <iostream>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

#ifdef WIN32
#  include <shlobj.h>
#  include <windows.h>
#else
#  include <cstdlib>
#  include <sys/stat.h>
#endif // WIN32

#include <core/io/Parser.t>
#include <core/io/StreamIO.h>

#include <core/Configuration.h>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::endl;
    using std::getline;
    using std::ifstream;
    using std::istringstream;
    using std::ostream;
    using std::ofstream;
    using std::string;

    using ato::core::io::cerr;
    using ato::core::io::Parser;


    ///////////////////////////////////////////////////////////////////////////
    // Static data member definitions

    const string Configuration::s_FileNotFound = "File not found";


    ///////////////////////////////////////////////////////////////////////////
    // Member function definitions

    Configuration::Configuration(const string& directory,
                                 const string& filename,
                                 const string& dflt) :
      m_directory(directory),
      m_filename(filename),
      m_default(dflt),
      m_flags(0)
    {
      m_opts["config"] = makeConfigPath();
    }

    void Configuration::load(bool verbose)
    {
      ifstream fin(m_opts["config"].c_str());
      if (!fin.is_open())
      {
        if (m_opts["config"] == m_default)
        {
          m_opts["config"] = s_FileNotFound;
          throw FileNotFound(m_opts["config"], "reading");
        }

        // Try default configuration file
        if (verbose)
        {
          cerr << "Warning:  Failed to open \"" << m_opts["config"]
               << "\" for reading; attempting to load \""
               << m_default << '\"' << endl;
        }

        fin.open(m_default);
        if (!fin.is_open())
        {
          m_opts["config"] = s_FileNotFound;
          throw FileNotFound(m_default, "reading");
        }

        m_opts["config"] = m_default;
      }


      /////////////////////////////////////////////////////////////////////////
      // Invoke derived class's parse

      parse(fin);
    }

    void Configuration::save(const string& directory,
                             const string& filename,
                             const string& title)
    {
      m_directory = directory;
      m_filename  = filename;

      string   full = makeConfigPath();
      ofstream fout(full.c_str());
      if (!fout.is_open())
        throw FileNotFound(full, "writing");

      m_opts["config"] = full;


      /////////////////////////////////////////////////////////////////////////
      // Invoke derived class's write

      write(fout, title);

      fout.close();
    }

    string Configuration::makeConfigPath() const
    {
#ifdef WIN32
      // XXX(cpg) - hard-coded maximum path length!
      char home[1024];
      if (SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, home) != S_OK)
      {
        // Try home directory...
        if (SHGetFolderPathA(NULL, CSIDL_PROFILE, NULL, 0, home) != S_OK)
          return "";
      }

      string path(home);
      path += "\\" + m_directory;
      DWORD attr;

      int rc = SHCreateDirectoryEx(NULL, path.c_str(), NULL);
      switch (rc)
      {
      case ERROR_FILE_EXISTS:
      case ERROR_ALREADY_EXISTS:
        attr = GetFileAttributes(path.c_str());
        if (!(attr & FILE_ATTRIBUTE_DIRECTORY))
        {
          // "${HOME}/.libato" is not a directory!
          return "";
        }

        break;

      default:
        return "";

        break;
      }

      path = path + "\\" + m_filename;
#else
      string path(getenv("HOME"));
      path += "/" + m_directory;

      struct stat sb;
      if (stat(path.c_str(), &sb) != 0)
      {
        // Path "${HOME}/.'m_directory'/" not found
        if (mkdir(path.c_str(), S_IRWXU) != 0)
          return "";
      }
      else if (!S_ISDIR(sb.st_mode))
      {
        // "${HOME}/.'m_director'" is not a directory!
        return "";
      }

      path = path + "/" + m_filename;
#endif // WIN32

      return path;
    }


    ///////////////////////////////////////////////////////////////////////////
    // Stream I/O

    ostream& operator<<(ostream& out, const Configuration& c)
    {
      out << "flags = " << c.m_flags << endl;
      out << "options" << endl;

      map<string, string>::const_iterator iter = c.m_opts.begin();
      while (iter != c.m_opts.end())
      {
        out << "  " << iter->first << " = \"" << iter->second << '\"' << endl;
        ++iter;
      }
      out << endl;

      return out;
    }

  } // namespace core

} // namespace ato
