
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


#ifndef core_UniformGrid_t
#define core_UniformGrid_t

#include <cassert>
#include <vector>

#include <core/BBox.h>
#include <core/Types.h>

#include <math/Helpers.t>
#include <math/Math_Float.h>

// XXX(cpg) - temporary
/*
#include <iostream>
using std::cerr;
using std::endl;
*/


namespace ato
{

  namespace coreT
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using ato::core::BBox;

    using ato::math::Float::Point;
    using ato::math::Float::Vector;

    using ato::math::Ceil;

    using ato::math::Max;
    using ato::math::Pow;


    ///////////////////////////////////////////////////////////////////////////
    // Forward declarations

    template<typename T>
    class ExtentComputer;


    ///////////////////////////////////////////////////////////////////////////
    // Template class definitions

    template<typename PrimitiveT,
             typename ExtentComputerT = ExtentComputer<PrimitiveT> >
    class UniformGrid
    {
    public:
      typedef std::vector<PrimitiveT*> PrimitiveList;

      static const float Epsilon;

      UniformGrid(const BBox& bounds,
                        bool  est   = false,
                        float res   = 64.f,
                        float scale =  1.f) :
        m_bounds(bounds)
      {
        if (est)
        {
          /////////////////////////////////////////////////////////////////////
          // Compute grid dimensions based on number of primitives and
          //   scale factor

          Vector adjust = Epsilon*bounds.diagonal();

          m_bounds[0] = m_bounds[0] - adjust;
          m_bounds[1] = m_bounds[1] + adjust;

          Vector length = m_bounds.diagonal();
          float  nprims = res;

          float maxlen  = Max(Max(length[0], length[1]), length[2]);
          float cubeRt  = scale*Pow(nprims, 1.f/3.f);
          float incells = cubeRt/maxlen;

          for (uint i = 0; i < 3; ++i)
          {
            m_ncells[i] = length[i]*incells;
            m_invw[i]   = m_ncells[i]/length[i];
          }
        }
        else
        {
          /////////////////////////////////////////////////////////////////////
          // Compute grid dimensions based on bounds and specified resolution

          float inv_r = 1.f/res;

          Point& m = m_bounds[0];
          Point& M = m_bounds[1];

          for (uint i = 0; i < 3; ++i)
          {
            m[i] = res*static_cast<int>(inv_r*m[i] - 1.f);
            M[i] = res*static_cast<int>(inv_r*M[i] + 1.f);
          }

          Vector length = m_bounds.diagonal();
          for (uint i = 0; i < 3; ++i)
          {
            m_ncells[i] = Ceil(length[i]*inv_r);
            m_invw[i]   = inv_r;
          }
        }

        // Allocate cells
        m_cells.resize(m_ncells[2]);
        for (uint z = 0; z < m_ncells[2]; ++z)
        {
          m_cells[z].resize(m_ncells[1]);
          for (uint y = 0; y < m_ncells[1]; ++y)
            m_cells[z][y].resize(m_ncells[0]);
        }
      }

      virtual ~UniformGrid()
      {
        // Deallocate primitives
        for (size_t z = 0; z < m_ncells[2]; ++z)
        {
          for (size_t y = 0; y < m_ncells[1]; ++y)
          {
            for (size_t x = 0; x < m_ncells[0]; ++x)
            {
              PrimitiveList& prims = m_cells[z][y][x];
              for (size_t i = 0; i < prims.size(); ++i)
                delete prims[i];
            }
          }
        }
      }

      const BBox& bounds() const
      {
        return m_bounds;
      }

      Vector dimensions() const
      {
        return Vector(m_ncells[0], m_ncells[1], m_ncells[2]);
      }

      Vector cellsize() const
      {
        return 1.f/m_invw;
      }

      size_t nx() const
      {
        return m_ncells[0];
      }

      size_t ny() const
      {
        return m_ncells[1];
      }

      size_t nz() const
      {
        return m_ncells[2];
      }

      const std::vector<std::vector<PrimitiveList> >& operator[](size_t i) const
      {
        assert(i >= 0 && i < m_ncells[2]);

        return m_cells[i];
      }

      void add(const PrimitiveT& p)
      {
        insert(new PrimitiveT(p));
      }

      void add(PrimitiveT* p)
      {
        insert(new PrimitiveT(*p));
      }

      void insert(PrimitiveT* p)
      {
        size_t extent[6];
        computeExtent(extent, p);

        /*
        cerr << "Primitive p = " << *p << endl;
        cerr << "  extent[x] = [" << extent[0] << ", " << extent[1] << ')' << endl;
        cerr << "  extent[y] = [" << extent[2] << ", " << extent[3] << ')' << endl;
        cerr << "  extent[z] = [" << extent[4] << ", " << extent[5] << ')' << endl;
        cerr << endl;
        */

        assert(extent[0] >= 0 && extent[0] <  m_ncells[0]);
        assert(extent[1] >= 0 && extent[1] <= m_ncells[0]);

        assert(extent[2] >= 0 && extent[2] <  m_ncells[1]);
        assert(extent[3] >= 0 && extent[3] <= m_ncells[1]);

        assert(extent[4] >= 0 && extent[4] <  m_ncells[2]);
        assert(extent[5] >= 0 && extent[5] <= m_ncells[2]);

        for (size_t z = extent[4]; z < extent[5]; ++z)
          for (size_t y = extent[2]; y < extent[3]; ++y)
            for (size_t x = extent[0]; x < extent[1]; ++x)
              m_cells[z][y][x].push_back(p);
      }

    private:
      void computeExtent(size_t extent[6], PrimitiveT* p)
      {
        ExtentComputerT::computeExtent(extent, m_bounds[0], m_invw, p);
      }

      friend ExtentComputerT;

      BBox   m_bounds;
      uint   m_ncells[3];
      Vector m_invw;

      std::vector<std::vector<std::vector<PrimitiveList> > > m_cells;
    };

    template<typename PrimitiveT, class ExtentComputerT>
    const float UniformGrid<PrimitiveT, ExtentComputerT>::Epsilon = 1.e-6f;

    template<typename T>
    class ExtentComputer
    {
    public:
      static void computeExtent(      size_t  e[6],
                                const Point&  min,
                                const Vector& inv,
                                const T&      p)
      {
        const BBox&  bounds = p->bounds();
        const Vector vmin   = bounds[0] - min;
        const Vector vmax   = bounds[1] - min;

        for (uint i = 0; i < 3; ++i)
        {
          e[2*i]   = vmin[i]*inv[i];
          e[2*i+1] = vmax[i]*inv[i] + 1;
        }
      }
    };

    template<>
    class ExtentComputer<Point>
    {
    public:
      static void computeExtent(      size_t  e[6],
                                const Point&  min,
                                const Vector& inv,
                                const Point*  p)
      {
        const Vector v = *p - min;

        for (uint i = 0; i < 3; ++i)
        {
          e[2*i]   = v[i]*inv[i];
          e[2*i+1] = e[2*i] + 1;
        }
      }
    };

  } // namespace coreT

} // namespace ato

#endif // core_UniformGrid_t
