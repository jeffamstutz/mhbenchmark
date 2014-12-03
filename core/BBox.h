
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2012, Christiaan Gribble <cgribble[]rtvtk org>
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


#ifndef core_BBox_h
#define core_BBox_h

#include <iostream>

#include <core/ato_coreExports.h>
#include <core/MinMax.t>

#include <math/Math.h>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::istream;
    using std::ostream;

    using math::Float::Point;
    using math::Float::Vector;


    ///////////////////////////////////////////////////////////////////////////
    // Class definition

    class ATO_CORE_EXPORT BBox
    {
    public:
      BBox(const Point& = Point::Max, const Point& = Point::Min);

      ~BBox();

            Point& operator[](uint);
      const Point& operator[](uint) const;

      void reset(const Point& = Point::Max, const Point& = Point::Min);

      void extend(const BBox&);
      void extend(const Point&);

      Point  center()    const;
      Vector diagonal()  const;
      float  computeSA() const;

      bool contains(const Point&) const;

      bool read(istream&);
      bool write(ostream&) const;

      friend ATO_CORE_EXPORT istream& operator>>(istream&,       BBox&);
      friend ATO_CORE_EXPORT ostream& operator<<(ostream&, const BBox&);

    protected:
      Point m_bounds[2];
    };


    ///////////////////////////////////////////////////////////////////////////
    // Inline member definitions

    inline BBox::BBox(const Point& min, const Point& max)
    {
      reset(min, max);
    }

    inline BBox::~BBox()
    {
      // no-op
    }

    inline Point& BBox::operator[](uint i)
    {
      return m_bounds[i];
    }

    inline const Point& BBox::operator[](uint i) const
    {
      return m_bounds[i];
    }

    inline void BBox::reset(const Point& min, const Point& max)
    {
      m_bounds[0] = min;
      m_bounds[1] = max;
    }

    inline void BBox::extend(const BBox& b)
    {
      m_bounds[0] = Min(m_bounds[0], b.m_bounds[0]);
      m_bounds[1] = Max(m_bounds[1], b.m_bounds[1]);
    }

    inline void BBox::extend(const Point& p)
    {
      m_bounds[0] = Min(m_bounds[0], p);
      m_bounds[1] = Max(m_bounds[1], p);
    }

    inline Point BBox::center() const
    {
      return 0.5f*(m_bounds[0] + m_bounds[1]);
    }

    inline Vector BBox::diagonal() const
    {
      return (m_bounds[1] - m_bounds[0]);
    }

    inline float BBox::computeSA() const
    {
      const Vector d = diagonal();
      return 2.f*(d[0]*d[1] + d[1]*d[2] + d[2]*d[0]);
    }

    inline bool BBox::contains(const Point& p) const
    {
      return ((p[0] >= m_bounds[0][0] && p[0] <= m_bounds[1][0]) &&
              (p[1] >= m_bounds[0][1] && p[0] <= m_bounds[1][1]) &&
              (p[2] >= m_bounds[0][2] && p[0] <= m_bounds[1][2]));
    }

  } // namespace core

} // namespace ato

#endif // core_BBox_h
