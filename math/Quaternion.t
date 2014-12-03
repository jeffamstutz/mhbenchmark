
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2014, Christiaan Gribble <cgribble[]rtvtk org>
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


#ifndef math_Quaternion_t
#define math_Quaternion_t

#include <iostream>

#include <math/Vector.t>


namespace ato
{

  namespace mathT
  {

    //////////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::ostream;

    using math::Cos;
    using math::Sin;


    //////////////////////////////////////////////////////////////////////////////
    // Class template definition

    /*! @class Quaternion
        @brief Mathematical structure for encoding three-dimensional rotation
               and orientation.
    */
    template<typename T>
    class Quaternion
    {
    public:
      Quaternion(const Vector<T>& v)
      {
        e[0] = v[0];
        e[1] = v[1];
        e[2] = v[2];
        e[3] = 0.f;
      }

      Quaternion(float x, float y, float z)
      {
        e[0] = x;
        e[1] = y;
        e[2] = z;
        e[3] = 0.f;
      }

      Quaternion(float s)
      {
        e[0] = 0.f;
        e[1] = 0.f;
        e[2] = 0.f;
        e[3] = s;
      }

      Quaternion(float s, Vector<T>& v)
      {
        e[0] = v[0];
        e[1] = v[1];
        e[2] = v[2];
        e[3] = s;
      }

      Quaternion(float s, float x, float y, float z)
      {
        e[0] = x;
        e[1] = y;
        e[2] = z;
        e[3] = s;
      }

      Quaternion()
      {
        e[0] = 0.f;
        e[1] = 0.f;
        e[2] = 0.f;
        e[3] = 0.f;
      }

      T s() const { return e[3]; }
      T x() const { return e[0]; }
      T y() const { return e[1]; }
      T z() const { return e[2]; }

      T  operator[](int i) const { return e[i]; }
      T& operator[](int i)       { return e[i]; }

      Quaternion conjugate() const
      {
        return Quaternion(e[3], -e[0], -e[1], -e[2]);
      }

      Quaternion inverse() const
      {
        return conjugate()*Quaternion(1/norm());
      }

      T norm() const
      {
        return (e[0]*e[0] + e[1]*e[1] + e[2]*e[2] + e[3]*e[3]);
      }

      Quaternion operator+(const Quaternion& q) const
      {
        return Quaternion(e[3] + q.e[3],
                          e[0] + q.e[0],
                          e[1] + q.e[1],
                          e[2] + q.e[2]);
      }

      Quaternion operator*(const Quaternion& q) const
      {
        // NOTE(dje) - to concatenate quaternion rotations, multiply
        //             the original rotation by the increment
        const Vector<T> v1(  e[0],   e[1],   e[2]);
        const Vector<T> v2(q.e[0], q.e[1], q.e[2]);

        return Quaternion(e[3]*q.e[3] - Dot(v1, v2),
                          e[3]*v2 + q.e[3]*v1 + Cross(v1, v2));
      }

      void encodeR(float theta, const Vector<T>& axis)
      {
        theta /= 2;

        const Vector<T> n    = axis.normal();
        const float     sine = Sin(theta);

        e[0] = sine*n[0];
        e[1] = sine*n[1];
        e[2] = sine*n[2];
        e[3] = Cos(theta);
      }

      Vector<T> rotateV(const Vector<T>& v) const
      {
        Quaternion q = inverse()*v*(*this);
        return Vector<T>(q.x(), q.y(), q.z());
      }

      inline friend ostream& operator<<(ostream& out, const Quaternion& q)
      {
        out << "[ " << q.e[3] << ", " << q.e[0] << ", " << q.e[1] << ", "
            << q.e[2] << " ]";

        return out;
      }

    private:
      T e[4];
    };

  } // namespace mathT

} // namespace ato

#endif // math_Quaternion_t
