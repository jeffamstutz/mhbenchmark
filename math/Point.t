
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2012-2014, Christiaan Gribble <cgribble[]rtvtk org>
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


#ifndef math_Point_t
#define math_Point_t

#include <cstdlib>

#include <iostream>
#include <limits>

#include <math/ato_mathExports.h>
#include <math/Helpers.t>


// NOTE(cpg) - an explanation of the need for TEMPLATE_FRIEND_MAGIC, even if
//             it still doesn't fully explain the '<>' syntax:
//
//               http://www.parashift.com/c++-faq/template-friends.html
#ifndef WIN32
#  define TEMPLATE_FRIEND_NAME(x) mathT::x
#  define TEMPLATE_FRIEND_MAGIC   <>
#else
#  define TEMPLATE_FRIEND_NAME(x) (x)
#  define TEMPLATE_FRIEND_MAGIC
#endif // WIN32



namespace ato
{

  namespace mathT
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::istream;
    using std::ostream;
    using std::numeric_limits;


    ///////////////////////////////////////////////////////////////////////////
    // Forward declarations

    template<typename T>
    class Point;

    template<typename T>
    class Vector;

    template<typename T>
    Point<T> operator*(const T&, const Point<T>&);


    ///////////////////////////////////////////////////////////////////////////
    // Class template definition

    template<typename T>
    class Point
    {
    public:

      /////////////////////////////////////////////////////////////////////////
      // Static members

      static const Point Epsilon;
      static const Point One;
      static const Point Max;
      static const Point Min;
      static const Point Zero;


      /////////////////////////////////////////////////////////////////////////
      // Constructors

      inline Point(const T& x, const T& y, const T& z)
      {
        e[0] = x;
        e[1] = y;
        e[2] = z;
      }

      inline Point(const Point& p)
      {
        e[0] = p.e[0];
        e[1] = p.e[1];
        e[2] = p.e[2];
      }

      inline Point()
      {
        e[0] = 0;
        e[1] = 0;
        e[2] = 0;
      }


      /////////////////////////////////////////////////////////////////////////
      // Destructor

      inline ~Point()
      {
        // no-op
      }


      /////////////////////////////////////////////////////////////////////////
      // Member operators

      inline T& operator[](size_t i)
      {
        return e[i];
      }

      inline const T& operator[](size_t i) const
      {
        return e[i];
      }

      inline Point operator-() const
      {
        return Point(-e[0], -e[1], -e[2]);
      }


      /////////////////////////////////////////////////////////////////////////
      // Point<T>-Point<T> operators

      inline Point operator+(const Point& p) const
      {
        return Point(e[0] + p.e[0], e[1] + p.e[1], e[2] + p.e[2]);
      }

      inline Vector<T> operator-(const Point& p) const
      {
        return Vector<T>(e[0] - p.e[0], e[1] - p.e[1], e[2] - p.e[2]);
      }


      /////////////////////////////////////////////////////////////////////////
      // Point<T>-Vector<T> interoperability

      inline Point operator+(const Vector<T>& v) const
      {
        return Point(e[0] + v.e[0], e[1] + v.e[1], e[2] + v.e[2]);
      }

      inline Point operator-(const Vector<T>& v) const
      {
        return Point(e[0] - v.e[0], e[1] - v.e[1], e[2] - v.e[2]);
      }

      inline Point& operator-=(const Vector<T>& v)
      {
        e[0] -= v[0];
        e[1] -= v[1];
        e[2] -= v[2];

        return *this;
      }


      /////////////////////////////////////////////////////////////////////////
      // Point<T>-T interoperability

      inline Point operator/(const T& s) const
      {
        const T inv = 1/s;
        return Point(e[0]*inv, e[1]*inv, e[2]*inv);
      }


      /////////////////////////////////////////////////////////////////////////
      // Friend classes, functions

      friend class Vector<T>;

#ifndef __CUDACC__
#  define USE_INTERNAL_FRIENDS
#  ifdef USE_INTERNAL_FRIENDS
      inline friend Point operator*(const T& s, const Point& p)
      {
        return Point(s*p.e[0], s*p.e[1], s*p.e[2]);
      }
#  else
      friend ATO_MATH_EXPORT Point<T> TEMPLATE_FRIEND_NAME(operator*) TEMPLATE_FRIEND_MAGIC (const T&, const Point<T>&);
#  endif // USE_INTERNAL_FRIENDS
#endif // __CUDACC__

      inline friend Point Min(const Point& p0, const Point& p1)
      {
        return Point(math::Min(p0.e[0], p1.e[0]),
                     math::Min(p0.e[1], p1.e[1]),
                     math::Min(p0.e[2], p1.e[2]));
      }

      inline friend Point Max(const Point& p0, const Point& p1)
      {
        return Point(math::Max(p0.e[0], p1.e[0]),
                     math::Max(p0.e[1], p1.e[1]),
                     math::Max(p0.e[2], p1.e[2]));
      }


      /////////////////////////////////////////////////////////////////////////
      // Stream I/O

      inline friend istream& operator>>(istream& in, Point& p)
      {
        char junk;
        in >> junk;
        in >> p.e[0] >> p.e[1] >> p.e[2];
        in >> junk;

        return in;
      }

      inline friend ostream& operator<<(ostream& out, const Point& p)
      {
        out << '(' << p.e[0] << ' ' << p.e[1] << ' ' << p.e[2] << ')';
        return out;
      }

    private:
      T e[3];
    };


    ///////////////////////////////////////////////////////////////////////////
    // Static members

#ifdef ATO_MATH_EXPORT
    template<typename T>
    const Point<T> Point<T>::Epsilon(numeric_limits<T>::epsilon(),
                                     numeric_limits<T>::epsilon(),
                                     numeric_limits<T>::epsilon());

    template<typename T>
    const Point<T> Point<T>::Max(numeric_limits<T>::max(),
                                 numeric_limits<T>::max(),
                                 numeric_limits<T>::max());

    template<typename T>
    const Point<T> Point<T>::Min(-numeric_limits<T>::max(),
                                 -numeric_limits<T>::max(),
                                 -numeric_limits<T>::max());

    template<typename T>
    const Point<T> Point<T>::One(1, 1, 1);

    template<typename T>
    const Point<T> Point<T>::Zero(0, 0, 0);
#endif // ATO_MATH_EXPORT


    ///////////////////////////////////////////////////////////////////////////
    // Friend functions

#ifndef USE_INTERNAL_FRIENDS
    template<typename T>
    inline Point<T> operator*(const T& s, const Point<T>& p)
    {
      return Point<T>(s*p.e[0], s*p.e[1], s*p.e[2]);
    }
#endif // USE_INTERNAL_FRIENDS

  } // namespace mathT

} // namespace ato

#endif // math_Point_t
