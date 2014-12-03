
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2012, 2014, Christiaan Gribble <cgribble[]rtvtk org>
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


#ifndef core_Texture_h
#define core_Texture_h

#include <iostream>
#include <string>

#include <core/ato_coreExports.h>
#include <core/Exception.h>
#include <core/RGB.t>
#include <core/TexCoord.h>

#include <math/Math.h>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::string;

    using core::Float::TexCoord;

    using math::Float::Vector;


    ///////////////////////////////////////////////////////////////////////////
    // Class definition

    /*! @class Texture
        @brief Information about object color and scaling.
    */
    class ATO_CORE_EXPORT Texture
    {
    public:

      /////////////////////////////////////////////////////////////////////////
      // Exception class

      class ATO_CORE_EXPORT InvalidTexture : public Exception
      {
      public:
        InvalidTexture(const string& msg) :
          Exception(msg)
        {
          // no-op
        }

        virtual ~InvalidTexture() throw()
        {
          // no-op
        }

      private:
        // none
      };


      /////////////////////////////////////////////////////////////////////////
      // Constructors, destructor, & assignment operator

      Texture(const string&, float, float);
      Texture(const core::Float::RGB&, float, float);
      Texture(const Texture&);
      Texture();

      ~Texture();

      Texture& operator=(const Texture&);


      /////////////////////////////////////////////////////////////////////////
      // Core functionality

      size_t size() const;

      core::Float::RGB getRGB(const TexCoord&) const;

      void load(const char*);

      bool read(std::istream&);
      bool write(std::ostream&) const;


      /////////////////////////////////////////////////////////////////////////
      // Helper functions

      void invalidate();
      void readData(FILE*, const char*, uint);


      /////////////////////////////////////////////////////////////////////////
      // Data members

      /*! @bug XXX(cpg) - why are these public?  and then why not use a struct?
       *                  if a class is the right construct, make these private
       *                  and make load() and readData() private as well
       */

      uint  m_width;
      uint  m_height;
      float m_uscale;
      float m_vscale;

      core::Float::RGB* m_values;
    };


    ///////////////////////////////////////////////////////////////////////////
    // Inline member function definitions

    inline void Texture::invalidate()
    {
      m_width  = 0;
      m_height = 0;
      m_uscale = 1.f;
      m_vscale = 1.f;

      if (m_values)
        delete [] m_values;
      m_values = 0;
    }

    inline size_t Texture::size() const
    {
      return (2*sizeof(uint) +
              2*sizeof(float) +
              m_width*m_height*sizeof(core::Float::RGB));
    }

  } // namespace core

} // namespace ato

#endif // core_Texture_h
