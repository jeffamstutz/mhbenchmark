
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2013, Christiaan Gribble <cgribble[]rtvtk org>
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
 *  * Neither the name of the author nor the names of its contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
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


#include <iomanip>
#include <iostream>

#include <core/Timer.h>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::ios;
    using std::setprecision;
    using std::setw;
    using std::streamsize;

#if defined(USING_MACOSX)
    mach_timebase_info_data_t Timer::tbaseInfo;
#endif // defined(USING_MACOSX)

    ostream& operator<<(ostream& out, Timer& timer)
    {
      float seconds = timer.getElapsed();
      int   minutes = 0;
      int   hours   = 0;
      if (seconds > 60)
      {
        minutes  = int(seconds/60.f);
        seconds -= 60.f*minutes;

        hours   = minutes/60;
        minutes = minutes%60;
      }

      char fill = out.fill('0');
      out << setw(2) << hours << ":";
      out << setw(2) << minutes << ":";

      out.setf(ios::fixed);
      streamsize precision = out.precision(2);

      out << setw(5) << seconds;

      out.precision(precision);
      out.unsetf(ios::fixed);
      out.fill(fill);

      return out;
    }

  } // namespace core

} // namespace ato
