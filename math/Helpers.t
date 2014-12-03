
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


#ifndef math_Helpers_t
#define math_Helpers_t

#include <cmath>


namespace ato
{

  namespace math
  {

    ///////////////////////////////////////////////////////////////////////////
    // Helper functions

    template<typename T>
    inline T Abs(const T& x)
    {
      return (x < 0 ? -x : x);
    }

    template<typename T>
    inline T ACos(const T& x)
    {
      return acos(x);
    }

    template<typename T>
    inline T ASin(const T& x)
    {
      return asin(x);
    }

    template<typename T>
    inline T ATan(const T& x)
    {
      return atan(x);
    }

    template<typename T>
    inline T ATan2(const T& x, const T& y)
    {
      return atan2(x, y);
    }

    template<typename T>
    inline T Ceil(const T& x)
    {
      return ceil(x);
    }

    template<typename T>
    inline T Clamp(const T&x, const T& lo, const T& hi)
    {
      return (x < lo ? lo : (x > hi ? hi : x));
    }

    template<typename T>
    inline T Cos(const T& x)
    {
      return cos(x);
    }

    template<typename T>
    inline T Exp(const T& x)
    {
      return exp(x);
    }

    template<typename T>
    inline T Floor(const T& x)
    {
      return floor(x);
    }

    template<typename T>
    inline T Log(const T& x)
    {
      return T(log(static_cast<double>(x)));
    }

    template<typename T>
    inline T Log2(const T& x)
    {
#ifdef WIN32
      return T(log(static_cast<double>(x)/log(2.)));
#else
      return T(log2(static_cast<double>(x)));
#endif // WIN32
    }

    template<typename T>
    inline T Log10(const T& x)
    {
      return T(log10(static_cast<double>(x)));
    }

    template<typename T>
    inline T Max(const T& x0, const T& x1)
    {
      return (x0 > x1 ? x0 : x1);
    }

    template<typename T>
    inline T Max(const T& x0, const T& x1, const T& x2)
    {
      return Max(x0, Max(x1, x2));
    }

    template<typename T>
    inline T Max(const T& x0, const T& x1, const T& x2, const T& x3)
    {
      return Max(Max(x0, x1), Max(x2, x3));
    }

    template<typename T>
    inline T Min(const T& x0, const T& x1)
    {
      return (x0 < x1 ? x0 : x1);
    }

    template<typename T>
    inline T Min(const T& x0, const T& x1, const T& x2)
    {
      return Min(x0, Min(x1, x2));
    }

    template<typename T>
    inline T Min(const T& x0, const T& x1, const T& x2, const T& x3)
    {
      return Min(Min(x0, x1), Min(x2, x3));
    }

    template<typename T>
    inline T Pow(const T& x, const T& y)
    {
      return T(pow(x, y));
    }

    template<typename T>
    inline T Sin(const T& x)
    {
      return sin(x);
    }

    template<typename T>
    inline T Sqrt(const T& x)
    {
      return T(sqrt(x));
    }

    template<typename T>
    inline T Tan(const T& x)
    {
      return tan(x);
    }


    ///////////////////////////////////////////////////////////////////////////
    // Template specializations

    template<>
    inline float Abs(const float& x)
    {
      return fabsf(x);
    }

    template<>
    inline float ACos(const float& x)
    {
      return acosf(x);
    }

    template<>
    inline float ASin(const float& x)
    {
      return asinf(x);
    }

    template<>
    inline float ATan(const float& x)
    {
      return atanf(x);
    }

    template<>
    inline float ATan2(const float& x, const float& y)
    {
      return atan2f(x, y);
    }

    template<>
    inline float Ceil(const float& x)
    {
      return ceilf(x);
    }

    template<>
    inline float Cos(const float& x)
    {
      return cosf(x);
    }

    template<>
    inline float Exp(const float& x)
    {
      return expf(x);
    }

    template<>
    inline float Floor(const float& x)
    {
      return floorf(x);
    }

    template<>
    inline float Log(const float& x)
    {
      return logf(x);
    }

    template<>
    inline float Pow(const float& x, const float& y)
    {
      return powf(x, y);
    }

    template<>
    inline float Sin(const float& x)
    {
      return sinf(x);
    }

    template<>
    inline float Sqrt(const float& x)
    {
      return sqrtf(x);
    }

    template<>
    inline float Tan(const float& x)
    {
      return tanf(x);
    }

  } // namespace math

} // namespace ato

#endif // math_Helpers_t
