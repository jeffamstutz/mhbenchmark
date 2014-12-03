
/******************************************************************************
 * Copyright (c) 2014 SURVICE Engineering Company
 * Copyright (c) 2012, 2013, Christiaan Gribble <cgribble[]rtvtk org>
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


#include <cstdio>
#include <cstring>

#include <core/Texture.h>
#include <core/Utility.h>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::istream;
    using std::ostream;


    ///////////////////////////////////////////////////////////////////////////
    // Member function definitions

    Texture::Texture(const string& filename,
                           float   uscale,
                           float   vscale) :
      m_width(0),
      m_height(0),
      m_uscale(uscale),
      m_vscale(vscale),
      m_values(0)
    {
      load(filename.c_str());
    }

    Texture::Texture(const core::Float::RGB& rgb,
                           float             uscale,
                           float             vscale) :
      m_width(1),
      m_height(1),
      m_uscale(uscale),
      m_vscale(vscale)
    {
      m_values = new core::Float::RGB[1*1];
      if (!m_values)
      {
        // Invalidate texture
        invalidate();

        return;
      }

      m_values[0] = rgb;
    }

    Texture::Texture(const Texture& rhs)
    {
      if (this == &rhs)
        return;

      // Copy members
      m_width  = rhs.m_width;
      m_height = rhs.m_height;
      m_uscale = rhs.m_uscale;
      m_vscale = rhs.m_vscale;

      // Allocate pixels
      m_values = new core::Float::RGB[m_width*m_height];
      if (!m_values)
      {
        // Invalidate texture
        invalidate();

        // Throw exception
        string msg = "Failed to allocate " +
          to_string(m_width*m_height*sizeof(core::Float::RGB)) + "bytes";

        throw InvalidTexture(msg);
      }

      // Copy pixels
      for (uint i = 0; i < m_width*m_height; ++i)
        m_values[i] = rhs.m_values[i];
    }

    Texture::Texture() :
      m_width(0),
      m_height(0),
      m_uscale(1.f),
      m_vscale(1.f),
      m_values(0)
    {
      // no-op
    }

    Texture::~Texture()
    {
      if (m_values)
        delete [] m_values;
    }

    Texture& Texture::operator=(const Texture& rhs)
    {
      if (this == &rhs)
        return *this;

      // Copy members
      m_width  = rhs.m_width;
      m_height = rhs.m_height;
      m_uscale = rhs.m_uscale;
      m_vscale = rhs.m_vscale;

      // Allocate pixels
      if (m_values)
        delete [] m_values;

      m_values = new core::Float::RGB[m_width*m_height];
      if (!m_values)
      {
        // Invalidate texture
        invalidate();

        // Throw exception
        string msg = "Failed to allocate " +
          to_string(m_width*m_height*sizeof(core::Float::RGB)) + "bytes";

        throw InvalidTexture(msg);
      }

      // Copy pixels
      for (uint i = 0; i < m_width*m_height; ++i)
        m_values[i] = rhs.m_values[i];

      return *this;
    }

    void Texture::load(const char* filename)
    {
      /*! @todo TODO(cpg) - cplusplus-ify me! */
      FILE* fp = 0;
      if (strcmp(filename + strlen(filename) - 4, ".ppm"))
      {
#ifdef WIN32
        /*! @bug XXX(cpg) - not yet implemented */

        // Invalidate texture
        invalidate();

        // Throw exception
        string msg = "PPM conversion not yet implemented!";

        throw InvalidTexture(msg);
#else
        // Convert non-PPM file
        char cmd[1024];
        sprintf(cmd, "convert '%s' PPM:-", filename);
        fp = popen(cmd, "r");
#endif // WIN32
      }
      else
      {
        fp = fopen(filename, "rb");
        if (!fp)
        {
          // Invalidate texture
          invalidate();

          // Throw exception
          string msg = "Failed to open \"" + to_string(filename) +
            "\" for reading";
          throw InvalidTexture(msg);
        }
      }

      // Read PPM magic
      char magic[32];
      if (fscanf(fp, "%s", magic) == 0)
      {
        // Invalidate texture
        invalidate();

        // Throw exception
        string msg = "Failed to read from \"" + to_string(filename) + "\"";
        throw InvalidTexture(msg);
      }

      // Eat comments and whitespace
      char line[1024];
      int c = fgetc(fp);
      while (c == '#' || isspace(c))
      {
        if (c == '#' && fgets(line, 1024, fp) == 0)
        {
          // Invalidate texture
          invalidate();

          // Throw exception
          string msg = "Failed to read from \"" + to_string(filename) + "\"";
          throw InvalidTexture(msg);
        }

        c = fgetc(fp);
      }

      // Back up and read dimensions
      ungetc(c, fp);

      uint junk;
      if (fscanf(fp, "%d %d %d", &m_width, &m_height, &junk) != 3)
      {
        // Invalidate texture
        invalidate();

        // Throw exception
        throw InvalidTexture("Failed to read PPM header");
      }

      // Eat newline/whitespace
      fgetc(fp);

      // Read image data
      readData(fp, magic, m_width);

      fclose(fp);
    }

    /*! @todo TODO(cpg) - cplusplus-ify me! */
    void Texture::readData(      FILE* fp,
                           const char* magic,
                                 uint  stride)
    {
      // Allocate pixels
      if (m_values)
        delete [] m_values;

      m_values = new core::Float::RGB[m_width*m_height];
      if (!m_values)
      {
        // Invalidate texture
        invalidate();

        // Throw exception
        string msg = "Failed to allocate " +
          to_string(m_width*m_height*sizeof(core::Float::RGB)) + " bytes";

        throw InvalidTexture(msg);
      }

      if (!strcmp(magic, "P3"))
      {
        uint r, g, b;
        for (int y = m_height-1; y >= 0; --y)
        {
          for (uint x = 0; x < m_width; ++x)
          {
            uint pixel = y*stride + x;
            if (fscanf(fp, "%d %d %d", &r, &g, &b) == 0)
            {
              // Invalidate texture
              invalidate();

              // Throw exception
              throw InvalidTexture("Failed to read texture values");
            }

            m_values[pixel][0] = ((float)r + 0.5f)/256.f;
            m_values[pixel][1] = ((float)g + 0.5f)/256.f;
            m_values[pixel][2] = ((float)b + 0.5f)/256.f;
          }
        }
      }
      else if (!strcmp(magic, "P6"))
      {
        for (int y = m_height-1; y >= 0; --y)
        {
          for (uint x = 0; x < m_width; ++x)
          {
            uint pixel = y*stride + x;
            m_values[pixel][0] = ((float)fgetc(fp) + 0.5f)/256.f;
            m_values[pixel][1] = ((float)fgetc(fp) + 0.5f)/256.f;
            m_values[pixel][2] = ((float)fgetc(fp) + 0.5f)/256.f;
          }
        }
      }
      else
      {
        // Invalidate texture
        invalidate();

        // Throw exception
        string msg = "Unrecognized PPM magic:  \"" + to_string(magic) + "\"";

        throw InvalidTexture(msg);
      }
    }

    core::Float::RGB Texture::getRGB(const TexCoord& uvw) const
    {
      const float u_in = uvw[0]*m_uscale;
      const float v_in = uvw[1]*m_vscale;

      const float u = u_in - (int)(u_in);
      const float v = v_in - (int)(v_in);

      const int uscaled = (int)(u*m_width);
      const int vscaled = (int)(v*m_height);

      const uint pindex = (uint)(vscaled*m_width + uscaled);

      return m_values[pindex];
    }

    bool Texture::read(istream& in)
    {
      // Read width, height
      in.read(reinterpret_cast<char*>(&m_width),  sizeof(uint));
      in.read(reinterpret_cast<char*>(&m_height), sizeof(uint));

      // Read uscale, vscale
      in.read(reinterpret_cast<char*>(&m_uscale), sizeof(float));
      in.read(reinterpret_cast<char*>(&m_vscale), sizeof(float));

      // Allocate pixels
      if (m_values)
        delete [] m_values;

      m_values = new core::Float::RGB[m_width*m_height];
      if (!m_values)
      {
        // Invalidate texture
        invalidate();

        // Throw exception
        string msg = "Failed to allocate " +
          to_string(m_width*m_height*sizeof(core::Float::RGB)) + "bytes";

        throw InvalidTexture(msg);
      }

      // Read pixel values
      in.read(reinterpret_cast<char*>(m_values),
              m_width*m_height*sizeof(core::Float::RGB));

      return in.good();
    }

    bool Texture::write(ostream& out) const
    {
      // Write width, height
      out.write(reinterpret_cast<const char*>(&m_width),  sizeof(uint));
      out.write(reinterpret_cast<const char*>(&m_height), sizeof(uint));

      // Write uscale, vscale
      out.write(reinterpret_cast<const char*>(&m_uscale), sizeof(float));
      out.write(reinterpret_cast<const char*>(&m_vscale), sizeof(float));

      // Write pixel values
      out.write(reinterpret_cast<const char*>(m_values),
                m_width*m_height*sizeof(core::Float::RGB));

      return out.good();
    }

  } // namespace core

} // namespace ato
