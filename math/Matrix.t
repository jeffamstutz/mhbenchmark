
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


#ifndef math_Matrix_t
#define math_Matrix_t

#include <iostream>

#include <core/Types.h>

#include <math/Constants.h>
#include <math/Helpers.t>
#include <math/Point.t>
#include <math/Vector.t>


namespace ato
{

  namespace mathT
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::endl;
    using std::ostream;


    ///////////////////////////////////////////////////////////////////////////
    // Enumerated types

    enum MatrixMode { RowMajor, ColumnMajor, numModes };


    ///////////////////////////////////////////////////////////////////////////
    // Class template definition

    /*! @class Matrix
        @brief Storage and manipulation of matrices.
    */
    template<typename   T,
             uint       Major,
             uint       Minor,
             MatrixMode Mode>
    class Matrix
    {
    public:

      /////////////////////////////////////////////////////////////////////////
      // Constructors

      Matrix(const T* m)
      {
        for (uint i = 0; i < Major; ++i)
          for (uint j = 0; j < Minor; ++j)
            e[i][j] = m[Minor*i+j];
      }

      Matrix()
      {
        for (uint i = 0; i < Major; ++i)
          for (uint j = 0; j < Minor; ++j)
            e[i][j] = 0.f;
      }


      /////////////////////////////////////////////////////////////////////////
      // Mutator, accessor

      T* operator[](const int i)
      {
        return e[i];
      }

      const T* operator[](const int i) const
      {
        return e[i];
      }


      /////////////////////////////////////////////////////////////////////////
      // Matrix operations

      void identity()
      {
        for (uint i = 0; i < Major; ++i)
          for (uint j = 0; j < Minor; ++j)
            e[i][j] = (i == j ? 1.f : 0.f);
      }

      Matrix<T, Minor, Major, Mode> transpose() const
      {
        Matrix<T, Minor, Major, Mode> retval;
        for (uint i = 0; i < Major; ++i)
          for (uint j = 0; j < Minor; ++j)
            retval[j][i] = e[i][j];

        return retval;
      }

    protected:
      T e[Major][Minor];
    };


    ////////////////////////////////////////////////////////////////////////////
    // Stream I/O

    template<typename T, uint Major, uint Minor, MatrixMode Mode>
    ostream& operator<<(      ostream&                       out,
                        const Matrix<T, Major, Minor, Mode>& matrix)
    {
      int rows = (Mode == RowMajor ? Major : Minor);
      int cols = (Mode == RowMajor ? Minor : Major);

      out << endl;
      for (uint i = 0; i < rows; ++i)
      {
        for (uint j = 0; j < cols; ++j)
          out << (Mode == RowMajor ? matrix[i][j] : matrix[j][i]) << ' ';
        out << endl;
      }

      return out;
    }


    ///////////////////////////////////////////////////////////////////////////
    // Matrix-Matrix multiplication specializations

    // RowMajor by RowMajor
    template<typename T, uint Major, uint inner, uint Minor>
    Matrix<T, Major, Minor, RowMajor>
    operator*(const Matrix<T, Major, inner, RowMajor>& lhs,
              const Matrix<T, inner, Minor, RowMajor>& rhs)
    {
      Matrix<T, Major, Minor, RowMajor> retval;
      for (uint i = 0; i < Major; ++i)
      {
        for (uint j = 0; j < Minor; ++j)
        {
          for (uint k = 0; k < inner; ++k)
            retval[i][j] += lhs[i][k] * rhs[k][j];
        }
      }

      return retval;
    }

    // RowMajor by ColumnMajor
    template<typename T, uint Major, uint inner, uint Minor>
    Matrix<T, Major, Minor, RowMajor>
    operator*(const Matrix<T, Major, inner, RowMajor>&    lhs,
              const Matrix<T, Minor, inner, ColumnMajor>& rhs)
    {
      Matrix<T, Major, Minor, RowMajor> retval;
      for (uint i = 0; i < Major; ++i)
      {
        for (uint j = 0; j < Minor; ++j)
        {
          for (uint k = 0; k < inner; ++k)
            retval[i][j] += lhs[i][k] * rhs[j][k];
        }
      }

      return retval;
    }

    // ColumnMajor by RowMajor
    template<typename T, uint Major, uint inner, uint Minor>
    Matrix<T, Major, Minor, ColumnMajor>
    operator*(const Matrix<T, inner, Minor, ColumnMajor>& lhs,
              const Matrix<T, inner, Major, RowMajor>&  rhs)
    {
      Matrix<T, Major, Minor, ColumnMajor> retval;
      for (uint i = 0; i < Major; ++i)
      {
        for (uint j = 0; j < Minor; ++j)
        {
          for (uint k = 0; k < inner; ++k)
            retval[i][j] += lhs[k][j] * rhs[k][i];
        }
      }

      return retval;
    }

    // ColumnMajor by ColumnMajor
    template<typename T, uint Major, uint inner, uint Minor>
    Matrix<T, Major, Minor, ColumnMajor>
    operator*(const Matrix<T, inner, Minor, ColumnMajor>& lhs,
              const Matrix<T, Major, inner, ColumnMajor>& rhs)
    {
      Matrix<T, Major, Minor, ColumnMajor> retval;
      for (uint i = 0; i < Major; ++i)
      {
        for (uint j = 0; j < Minor; ++j)
        {
          for (uint k = 0; k < inner; ++k)
            retval[i][j] += lhs[k][j] * rhs[i][k];
        }
      }

      return retval;
    }

  } // namespace mathT

} // namespace ato

#endif // math_Matrix_t
