
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


#ifndef math_Vector_t
#define math_Vector_t

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

    using std::numeric_limits;


    ///////////////////////////////////////////////////////////////////////////
    // Forward declarations

    template<typename T>
    class Point;

    template<typename T>
    class Vector;

    template<typename T>
    Vector<T> operator*(const T&, const Vector<T>&);

    template<typename T>
    Vector<T> operator/(const T&, const Vector<T>&);

    template<typename T>
    T Dot(const Vector<T>&, const Vector<T>&);

    template<typename T>
    Vector<T> Cross(const Vector<T>&, const Vector<T>&);


    ///////////////////////////////////////////////////////////////////////////
    // Class template definition

    template<typename T>
    class Vector
    {
    public:

      /////////////////////////////////////////////////////////////////////////
      // Static members

      static const Vector Epsilon;
      static const Vector One;
      static const Vector Max;
      static const Vector Min;
      static const Vector Zero;


      /////////////////////////////////////////////////////////////////////////
      // Constructors

      inline Vector(const T& x, const T& y, const T& z)
      {
        e[0] = x;
        e[1] = y;
        e[2] = z;
      }

      inline Vector(const Vector& v)
      {
        e[0] = v.e[0];
        e[1] = v.e[1];
        e[2] = v.e[2];
      }

      inline Vector()
      {
        e[0] = 0;
        e[1] = 0;
        e[2] = 0;
      }

      // Point<T> --> Vector<T>
      inline explicit Vector(const Point<T>& p)
      {
        e[0] = p.e[0];
        e[1] = p.e[1];
        e[2] = p.e[2];
      }

      // Vector<U> --> Vector<T>
      template<typename U>
      inline explicit Vector(const Vector<U>& v)
      {
        e[0] = T(v.e[0]);
        e[1] = T(v.e[1]);
        e[2] = T(v.e[2]);
      }


      /////////////////////////////////////////////////////////////////////////
      // Destructor

      inline ~Vector()
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

      inline bool operator==(const Vector& v) const
      {
        return (e[0] == v.e[0] && e[1] == v.e[1] && e[2] == v.e[2]);
      }

      inline Vector operator-() const
      {
        return Vector(-e[0], -e[1], -e[2]);
      }

      inline T length2() const
      {
        return (e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
      }

      inline T length() const
      {
        return math::Sqrt(length2());
      }

      inline T normalize()
      {
        const T len = length();
        const T inv = (len > 0 ? 1/len : 0);

        e[0] *= inv;
        e[1] *= inv;
        e[2] *= inv;

        return len;
      }

      inline Vector<T> normal() const
      {
        const T len = math::Sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
        const T inv = (len > 0 ? 1/len : 0);

        return Vector<T>(inv*e[0], inv*e[1], inv*e[2]);
      }

      inline Vector<T> orthogonal() const
      {
        // if |z| < |x| or |z| < |y|
        //   <x, y, z> = <y, -x, 0>
        // else
        //   <x, y, z> = <0, z, -y>
        const T& abs_z = math::Abs(e[2]);
        if (abs_z < math::Abs(e[0]) || abs_z < math::Abs(e[1]))
          return Vector<T>(e[1], -e[0], 0).normal();

        return Vector<T>(0, e[2], -e[1]).normal();
      }


      /////////////////////////////////////////////////////////////////////////
      // Vector<T>-Vector<T> operators

      inline Vector operator+(const Vector& v) const
      {
        return Vector(e[0] + v.e[0], e[1] + v.e[1], e[2] + v.e[2]);
      }

      inline Vector operator-(const Vector& v) const
      {
        return Vector(e[0] - v.e[0], e[1] - v.e[1], e[2] - v.e[2]);
      }

      inline Vector& operator+=(const Vector& v)
      {
        e[0] += v.e[0];
        e[1] += v.e[1];
        e[2] += v.e[2];

        return *this;
      }


      /////////////////////////////////////////////////////////////////////////
      // Vector<T>-T interoperability

      inline Vector operator*(const T& s) const
      {
        return Vector(s*e[0], s*e[1], s*e[2]);
      }

      inline Vector operator/(const T& s) const
      {
        const T inv_s = 1/s;

        return Vector(e[0]*inv_s, e[1]*inv_s, e[2]*inv_s);
      }

      inline Vector& operator*=(const T& s)
      {
        e[0] *= s;
        e[1] *= s;
        e[2] *= s;

        return *this;
      }


      /////////////////////////////////////////////////////////////////////////
      // Friend classes, functions

      friend class Point<T>;

      template<typename U>
      friend class Vector;

#ifndef __CUDACC__
#  define USE_INTERNAL_FRIENDS
#  ifdef USE_INTERNAL_FRIENDS
      inline friend Vector operator*(const T& s, const Vector& v)
      {
        return Vector(s*v.e[0], s*v.e[1], s*v.e[2]);
      }

      inline friend Vector operator/(const T& s, const Vector& v)
      {
        return Vector(s/v.e[0], s/v.e[1], s/v.e[2]);
      }

      inline friend T Dot(const Vector& v0, const Vector& v1)
      {
        return (v0.e[0]*v1.e[0] + v0.e[1]*v1.e[1] + v0.e[2]*v1.e[2]);
      }

      inline friend Vector Cross(const Vector& v0, const Vector& v1)
      {
        return Vector(v0.e[1]*v1.e[2] - v0.e[2]*v1.e[1],
                      v0.e[2]*v1.e[0] - v0.e[0]*v1.e[2],
                      v0.e[0]*v1.e[1] - v0.e[1]*v1.e[0]);
      }
#  else
      friend ATO_MATH_EXPORT Vector<T> TEMPLATE_FRIEND_NAME(operator*) TEMPLATE_FRIEND_MAGIC (const T&,         const Vector<T>&);
      friend ATO_MATH_EXPORT Vector<T> TEMPLATE_FRIEND_NAME(operator/) TEMPLATE_FRIEND_MAGIC (const T&,         const Vector<T>&);
      friend ATO_MATH_EXPORT T         TEMPLATE_FRIEND_NAME(Dot)       TEMPLATE_FRIEND_MAGIC (const Vector<T>&, const Vector<T>&);
      friend ATO_MATH_EXPORT Vector<T> TEMPLATE_FRIEND_NAME(Cross)     TEMPLATE_FRIEND_MAGIC (const Vector<T>&, const Vector<T>&);
#  endif // USE_INTERNAL_FRIENDS
#endif // __CUDACC__

      inline friend Vector Min(const Vector& v0, const Vector& v1)
      {
        return Vector(math::Min(v0.e[0], v1.e[0]),
                      math::Min(v0.e[1], v1.e[1]),
                      math::Min(v0.e[2], v1.e[2]));
      }

      inline friend Vector Max(const Vector& v0, const Vector& v1)
      {
        return Vector(math::Max(v0.e[0], v1.e[0]),
                      math::Max(v0.e[1], v1.e[1]),
                      math::Max(v0.e[2], v1.e[2]));
      }


      /////////////////////////////////////////////////////////////////////////
      // Stream I/O

      inline friend istream& operator>>(istream& in, Vector& v)
      {
        char junk;
        in >> junk;
        in >> v.e[0] >> v.e[1] >> v.e[2];
        in >> junk;

        return in;
      }

      inline friend ostream& operator<<(ostream& out, const Vector& v)
      {
        out << '(' << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2] << ')';

        return out;
      }

    private:
      T e[3];
    };


    ///////////////////////////////////////////////////////////////////////////
    // Static members

#ifdef ATO_MATH_EXPORT
    template<typename T>
    const Vector<T> Vector<T>::Epsilon(numeric_limits<T>::epsilon(),
                                       numeric_limits<T>::epsilon(),
                                       numeric_limits<T>::epsilon());

    template<typename T>
    const Vector<T> Vector<T>::Max(numeric_limits<T>::max(),
                                   numeric_limits<T>::max(),
                                   numeric_limits<T>::max());

    template<typename T>
    const Vector<T> Vector<T>::Min(-numeric_limits<T>::max(),
                                   -numeric_limits<T>::max(),
                                   -numeric_limits<T>::max());

    template<typename T>
    const Vector<T> Vector<T>::One(1, 1, 1);

    template<typename T>
    const Vector<T> Vector<T>::Zero(0, 0, 0);
#endif // ATO_MATH_EXPORT


    ///////////////////////////////////////////////////////////////////////////
    // Friend functions

#ifndef USE_INTERNAL_FRIENDS
    template<typename T>
    inline Vector<T> operator*(const T& s, const Vector<T>& v)
    {
      return Vector<T>(s*v.e[0], s*v.e[1], s*v.e[2]);
    }

    template<typename T>
    inline Vector<T> operator/(const T& s, const Vector<T>& v)
    {
      return Vector<T>(s/v.e[0], s/v.e[1], s/v.e[2]);
    }

    template<typename T>
    inline T Dot(const Vector<T>& v0, const Vector<T>& v1)
    {
      return (v0.e[0]*v1.e[0] + v0.e[1]*v1.e[1] + v0.e[2]*v1.e[2]);
    }

    template<typename T>
    inline Vector<T> Cross(const Vector<T>& v0, const Vector<T>& v1)
    {
      return Vector<T>(v0.e[1]*v1.e[2] - v0.e[2]*v1.e[1],
                       v0.e[2]*v1.e[0] - v0.e[0]*v1.e[2],
                       v0.e[0]*v1.e[1] - v0.e[1]*v1.e[0]);
    }
#endif  // USE_INTERNAL_FRIENDS

  } // namespace mathT

} // namespace ato

#endif // math_Vector_t
