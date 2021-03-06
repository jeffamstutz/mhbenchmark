
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


#ifndef core_MinMax_t
#define core_MinMax_t


namespace ato
{

  namespace core
  {

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

  } // namespace core

} // namespace ato

#endif // core_MinMax_t
