
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


#ifndef core_Volume3D_t
#define core_Volume3D_t

#include <core/BBox.h>
#include <core/Types.h>

#include <math/Helpers.t>
#include <math/Math.h>


namespace ato
{

  namespace coreT
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using ato::core::BBox;

    using ato::math::Float::Point;
    using ato::math::Float::Vector;


    ///////////////////////////////////////////////////////////////////////////
    // Template class definitions

    template<typename DataT>
    class Volume3D
    {
    public:
      Volume3D(const BBox& bounds, uint nx, uint ny, uint nz) :
        m_bounds(bounds),
        m_nx(nx),
        m_ny(ny),
        m_nz(nz)
      {
        const Vector& d = bounds.diagonal();
        m_csz  = Vector(d[0]/m_nx, d[1]/m_ny, d[2]/m_nz);
        m_invw = 1.f/m_csz;

        // Allocate memory
        m_data   = new DataT[m_nx*m_ny*m_nz]();
        m_slices = new DataT**[m_nz];
        for (auto z = 0; z < m_nz; ++z)
        {
          m_slices[z] = new DataT*[m_ny];
          for (auto y = 0; y < m_ny; ++y)
            m_slices[z][y] = &(m_data[z*m_ny*m_nx + y*m_nx]);
        }
      }

      ~Volume3D()
      {
        // XXX(cpg) - causing segfaults!
        /*
        for (auto z = 0; z < m_nz; ++z)
          delete [] m_slices[z];

        delete [] m_slices;
        delete [] m_data;
        */
      }


      /////////////////////////////////////////////////////////////////////////
      // Slice, row accessors

      DataT** operator[](size_t z)
      {
        return m_slices[z];
      }

      const DataT** operator[](size_t z) const
      {
        return m_slices[z];
      }

      DataT* row(size_t y, size_t z)
      {
        return m_slices[z][y];
      }

      const DataT* row(size_t y, size_t z) const
      {
        return m_slices[z][y];
      }


      /////////////////////////////////////////////////////////////////////////
      // Cell accessors

      DataT& cell(size_t x, size_t y, size_t z)
      {
        return m_data[z*m_ny*m_nx + y*m_nx + x];
      }

      const DataT& cell(size_t x, size_t y, size_t z) const
      {
        return m_data[z*m_ny*m_nx + y*m_nx + x];
      }


      /////////////////////////////////////////////////////////////////////////
      // Other functionality

      DataT* ptr()
      {
        return m_data;
      }

      const DataT* ptr() const
      {
        return m_data;
      }

      const BBox& bounds() const
      {
        return m_bounds;
      }

      ato::mathT::Vector<size_t> dimensions() const
      {
        return ato::mathT::Vector<size_t>(m_nx, m_ny, m_nz);
      }

    protected:
      DataT*   m_data;
      DataT*** m_slices;

      BBox m_bounds;

      size_t m_nx;
      size_t m_ny;
      size_t m_nz;

      Vector m_csz;
      Vector m_invw;
    };

  } // namespace coreT

} // namespace ato

#endif // core_Volume3D_t
