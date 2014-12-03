
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
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
 *  * Neither the name of SURVICE Engineering Company nor the names of
 *    its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
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


#include <cmath>

#include <math/Constants.h>


#ifdef WIN32
#  ifndef M_PI
#    define M_PI 3.14159265358979323846
#  endif // M_PI
#endif // WIN32


namespace ato
{

  namespace math
  {

    const float Pi               = M_PI;
    const float TwoPi            = 2.f*Pi;
    const float FourPi           = 4.f*Pi;
    const float EightPi          = 8.f*Pi;
    const float PiOverTwo        = 0.5f*Pi;
    const float PiOverFour       = 0.25f*Pi;
    const float OneOverPi        = 1.f/Pi;
    const float OneOverTwoPi     = 1.f/TwoPi;
    const float OneOverFourPi    = 1.f/FourPi;
    const float OneOverEightPi   = 1.f/EightPi;
    const float DegreesToRadians = Pi/180.f;
    const float RadiansToDegrees = 180.f/Pi;
    const float OneOverSqrtTwo   = 0.70710678118f;

  } // namespace math

} // namespace ato
