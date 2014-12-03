
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2013, 2014, Christiaan Gribble <cgribble[]rtvtk org>
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


#ifndef core_RGB_t
#define core_RGB_t

/*! @todo TODO(cpg) - consider including shift operators only in RGB<int>
 *                    might require a specialized RGB<int> class
 *                    definition...)
 */

#include <cstdlib>
#include <iostream>

#include <core/ato_coreExports.h>
#include <core/Types.h>

#include <math/Math.h>

#ifdef WIN32
#  undef RGB
#endif // WIN32


namespace ato
{

  /////////////////////////////////////////////////////////////////////////////
  // Forward declarations

  namespace mathT
  {

    template<typename T>
    class Vector;

  } // namespace mathT


  namespace coreT
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::istream;
    using std::ostream;

    using math::Exp;
    using math::Log;
    using math::Pow;


    ///////////////////////////////////////////////////////////////////////////
    // Forward declarations

    template<typename T>
    class RGB;

    template<typename T>
    RGB<T> operator*(const T&, const RGB<T>&);

    template<typename T>
    RGB<T> Exp(const RGB<T>&);

    template<typename T>
    RGB<T> Log(const RGB<T>&);

    template<typename T>
    RGB<T> Pow(const RGB<T>&, const RGB<T>&);

    template<typename T>
    RGB<T> Pow(const RGB<T>&, const T&);


    ///////////////////////////////////////////////////////////////////////////
    // Class template definition

    /*! @class RGB
        @brief Storage and manipulation of color values.
    */
    template<typename T>
    class RGB
    {
    public:

      /////////////////////////////////////////////////////////////////////////
      // Static members

      static const RGB One;
      static const RGB Zero;
      static const RGB Invalid;


      /////////////////////////////////////////////////////////////////////////
      // Constructors

      inline RGB(const T& r, const T& g, const T& b)
      {
        e[0] = r;
        e[1] = g;
        e[2] = b;
      }

      inline RGB()
      {
        *this = RGB::Zero;
      }

      template<typename U>
      inline RGB(const RGB<U>& rgb)
      {
        e[0] = T(rgb.e[0]);
        e[1] = T(rgb.e[1]);
        e[2] = T(rgb.e[2]);
      }


      /////////////////////////////////////////////////////////////////////////
      // Destructor

      inline ~RGB()
      {
        // no-op
      }


      /////////////////////////////////////////////////////////////////////////
      // Member operators

      inline T& operator[](uint i)
      {
        return e[i];
      }

      inline const T& operator[](uint i) const
      {
        return e[i];
      }

      inline bool operator==(const RGB<T>& rgb) const
      {
        return (e[0] == rgb.e[0] &&
                e[1] == rgb.e[1] &&
                e[2] == rgb.e[2]);
      }

      inline bool operator!=(const RGB<T>& rgb) const
      {
        return (e[0] != rgb.e[0] ||
                e[1] != rgb.e[1] ||
                e[2] != rgb.e[2]);
      }

      inline RGB operator<<(uint i) const
      {
        // NOTE(cpg) - effectively a no-op for anything but RGB<int>; see
        //             template specializations below

        // To quiet warnings...
        return *this;
      }

      inline RGB operator>>(uint i) const
      {
        // NOTE(cpg) - effectively a no-op for anything but RGB<int>; see
        //             template specializations below

        // To quiet warnings...
        return *this;
      }


      /////////////////////////////////////////////////////////////////////////
      // RGB<T>-RGB<T> operators

      inline RGB operator+(const RGB& rgb) const
      {
        return RGB(e[0] + rgb.e[0],
                   e[1] + rgb.e[1],
                   e[2] + rgb.e[2]);
      }

      inline RGB operator-(const RGB& rgb) const
      {
        return RGB(e[0] - rgb.e[0],
                   e[1] - rgb.e[1],
                   e[2] - rgb.e[2]);
      }

      inline RGB operator*(const RGB& rgb) const
      {
        return RGB(e[0]*rgb.e[0],
                   e[1]*rgb.e[1],
                   e[2]*rgb.e[2]);
      }

      inline RGB& operator+=(const RGB& rgb)
      {
        e[0] += rgb.e[0];
        e[1] += rgb.e[1];
        e[2] += rgb.e[2];

        return *this;
      }

      inline RGB& operator*=(const RGB& rgb)
      {
        e[0] *= rgb.e[0];
        e[1] *= rgb.e[1];
        e[2] *= rgb.e[2];

        return *this;
      }

      inline void clamp(const RGB& rgb0, const RGB& rgb1)
      {
        e[0] = (e[0] < rgb0.e[0] ? rgb0.e[0] : (e[0] > rgb1.e[0] ? rgb1.e[0] : e[0]));
        e[1] = (e[1] < rgb0.e[1] ? rgb0.e[1] : (e[1] > rgb1.e[1] ? rgb1.e[1] : e[1]));
        e[2] = (e[2] < rgb0.e[2] ? rgb0.e[2] : (e[2] > rgb1.e[2] ? rgb1.e[2] : e[2]));
      }


      /////////////////////////////////////////////////////////////////////////
      // RGB<T>-T interoperability

      inline RGB operator*(const T& s) const
      {
        return RGB(s*e[0], s*e[1], s*e[2]);
      }

      inline RGB operator/(const T& s) const
      {
        const T inv = 1/s;
        return RGB(e[0]*inv, e[1]*inv, e[2]*inv);
      }

      inline RGB operator*=(const T& s)
      {
        *this = *this*s;
        return *this;
      }

      inline RGB operator/=(const T& s)
      {
        *this = *this/s;
        return *this;
      }

      inline void clamp(const T& s0, const T& s1)
      {
        e[0] = (e[0] < s0 ? s0 : (e[0] > s1 ? s1 : e[0]));
        e[1] = (e[1] < s0 ? s0 : (e[1] > s1 ? s1 : e[1]));
        e[2] = (e[2] < s0 ? s0 : (e[2] > s1 ? s1 : e[2]));
      }


      /////////////////////////////////////////////////////////////////////////
      // Friend classes, functions

      template<typename U>
      friend class RGB;

      friend class mathT::Vector<T>;

      template<typename U>
      friend class mathT::Vector;

#ifndef __CUDACC__
#ifndef WIN32
      // XXX(cpg) - causes VS2013 compiler to crash!
      friend RGB<T> coreT::operator* <> (const T&, const RGB<T>&);

      friend RGB<T> coreT::Exp <> (const RGB<T>&);
      friend RGB<T> coreT::Log <> (const RGB<T>&);
      friend RGB<T> coreT::Pow <> (const RGB<T>&, const RGB<T>&);
      friend RGB<T> coreT::Pow <> (const RGB<T>&, const T&);
#endif // WIN32
#endif // __CUDACC__


      /////////////////////////////////////////////////////////////////////////
      // Stream I/O

      inline friend istream& operator>>(istream& in, RGB& rgb)
      {
        char junk;
        in >> junk;
        in >> rgb.e[0] >> rgb.e[1] >> rgb.e[2];
        in >> junk;

        return in;
      }

      inline friend ostream& operator<<(ostream& out, const RGB& rgb)
      {
        out << '(' << rgb.e[0] << ' ' << rgb.e[1] << ' ' << rgb.e[2] << ')';

        return out;
      }

    private:
      T e[3];
    };


    ///////////////////////////////////////////////////////////////////////////
    // Static members

#ifdef ATO_CORE_EXPORT
    template<typename T>
    const RGB<T> RGB<T>::One(1, 1, 1);

    template<typename T>
    const RGB<T> RGB<T>::Zero(0, 0, 0);

    template<typename T>
    const RGB<T> RGB<T>::Invalid(-1, -1, -1);
#endif // ATO_CORE_EXPORT


    ///////////////////////////////////////////////////////////////////////////
    // Friend function definitions

    template<typename T>
    inline RGB<T> operator*(const T& s, const RGB<T>& rgb)
    {
#ifdef WIN32
      // NOTE(cpg) - force use of operator[] because operator* is not a
      //             friend...  see XXX(cpg) above
      return RGB<T>(s*rgb[0], s*rgb[1], s*rgb[2]);
#else
      return RGB<T>(s*rgb.e[0], s*rgb.e[1], s*rgb.e[2]);
#endif // WIN32
    }

    template<typename T>
    inline RGB<T> Exp(const RGB<T>& rgb)
    {
      return RGB<T>(Exp(rgb[0]), Exp(rgb[1]), Exp(rgb[2]));
    }

    template<typename T>
    inline RGB<T> Log(const RGB<T>& rgb)
    {
      return RGB<T>(Log(rgb[0]), Log(rgb[1]), Log(rgb[2]));
    }

    template<typename T>
    inline RGB<T> Pow(const RGB<T>& rgb0, const RGB<T>& rgb1)
    {
      return RGB<T>(Pow(rgb0[0], rgb1[0]),
                    Pow(rgb0[1], rgb1[1]),
                    Pow(rgb0[2], rgb1[2]));
    }

    template<typename T>
    inline RGB<T> Pow(const RGB<T>& rgb, const T& s)
    {
      return RGB<T>(Pow(rgb[0], s),
                    Pow(rgb[1], s),
                    Pow(rgb[2], s));
    }

  } // namespace coreT


  namespace core
  {

    namespace Float
    {

      /////////////////////////////////////////////////////////////////////////
      // Type definitions

      typedef coreT::RGB<float> RGB;


      /////////////////////////////////////////////////////////////////////////
      // Useful colors

      extern ATO_CORE_EXPORT const RGB Black;
      extern ATO_CORE_EXPORT const RGB Blue;
      extern ATO_CORE_EXPORT const RGB Cyan;
      extern ATO_CORE_EXPORT const RGB Green;
      extern ATO_CORE_EXPORT const RGB Magenta;
      extern ATO_CORE_EXPORT const RGB Red;
      extern ATO_CORE_EXPORT const RGB White;
      extern ATO_CORE_EXPORT const RGB Yellow;

    } // namespace Float

  } // namespace core

} // namespace ato

#endif // core_RGB_t
