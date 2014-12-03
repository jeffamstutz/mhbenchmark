
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


#ifndef core_TileQueue_h
#define core_TileQueue_h

#include <vector>
#include <mutex>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Class definitions

    struct Tile
    {
      Tile(uint = -1, uint = -1, uint = -1, uint = -1);

      uint sx, ex, sy, ey;
    };

    class TileQueue
    {
    public:
      TileQueue();
      ~TileQueue();

      void populate(uint, uint, uint, uint);
      bool pop(Tile&);
      void reset();

    private:
      std::vector<Tile> m_tiles;

      std::mutex m_mutex;
      uint       m_idx;
    };


    ///////////////////////////////////////////////////////////////////////////
    // Inline member function definitions

    inline Tile::Tile(uint sx_, uint ex_, uint sy_, uint ey_) :
      sx(sx_),
      ex(ex_),
      sy(sy_),
      ey(ey_)
    {
      // no-op
    }

    inline TileQueue::TileQueue() :
      m_idx(0)
    {
      // no-op
    }

    inline TileQueue::~TileQueue()
    {
      // no-op
    }

    inline void TileQueue::populate(uint width, uint height, uint tx, uint ty)
    {
      m_tiles.clear();

      for (uint y = 0; y < height; y += ty)
        for (uint x = 0; x < width; x += tx)
          m_tiles.push_back(Tile(x, x+tx, y, y+ty));
    }

    inline bool TileQueue::pop(Tile& tile)
    {
      std::unique_lock<std::mutex> lock(m_mutex);

      if (m_idx >= m_tiles.size())
        return false;

      tile = m_tiles[m_idx++];

      return true;
    }

    inline void TileQueue::reset()
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      m_idx = 0;
    }

  } // namespace core

} // namespace ato

#endif // core_TileQueue_h
