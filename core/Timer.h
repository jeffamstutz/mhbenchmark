
/******************************************************************************
 * Copyright (c) 2014, SURVICE Engineering Company
 * Copyright (c) 2013, Ethan Kerzner <ethan-kerzner[]uiowa edu>
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

// NOTE(ebk) - Parts of this file adapted, with permission, from
//             Chris Wyman (http://homepage.cs.uiowa.edu/~cwyman/)


#ifndef core_Timer_h
#define core_Timer_h

#include <iosfwd>

#if defined(WIN32) && defined(_MSC_VER) && !defined(USING_MSVC)
#  define USING_MSVC
#endif

#if defined(__APPLE__) && !defined(USING_MACOSX) && !defined(USING_LINUX)
#  define USING_MACOSX
#endif

#if defined(__GNUC__) && !defined(USING_MACOSX) && !defined(USING_LINUX)
#  define USING_LINUX
#endif

#if defined(USING_MSVC)

#include <windows.h>
#pragma comment(lib, "kernel32.lib")
  typedef LARGE_INTEGER TimerStruct;

#elif defined(USING_LINUX)

#include <time.h>
  typedef struct timespec TimerStruct;

#elif defined(USING_MACOSX)

#include <CoreServices/CoreServices.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
  typedef uint64_t TimerStruct;

#endif // USING_MSVC

#include <core/ato_coreExports.h>


namespace ato
{

  namespace core
  {

    ///////////////////////////////////////////////////////////////////////////
    // Using declarations

    using std::ostream;


    ///////////////////////////////////////////////////////////////////////////
    // Class definition

    class ATO_CORE_EXPORT Timer
    {
    public:
      Timer();

      void reset();

      double getElapsed();
      double getReset();

    private:
      void   getHighResolutionTime(TimerStruct*);
      double convertTimeDifferenceToSec(TimerStruct*, TimerStruct*);

      TimerStruct lastTime;
      TimerStruct curTime;

#if defined(USING_MACOSX)
      static mach_timebase_info_data_t tbaseInfo;
#endif // defined(USING_MACOSX)
    };


    ///////////////////////////////////////////////////////////////////////////
    // Inline member function definitions

    inline Timer::Timer()
    {
#if defined(USING_MACOSX)
      mach_timebase_info(&tbaseInfo);
#endif // defined(USING_MACOSX)

      getHighResolutionTime(&lastTime);
    }

    inline void Timer::reset()
    {
      getHighResolutionTime(&lastTime);
    }

    inline double Timer::getElapsed()
    {
      getHighResolutionTime(&curTime);
      return convertTimeDifferenceToSec(&curTime, &lastTime);
    }

    inline double Timer::getReset()
    {
      curTime = lastTime;
      getHighResolutionTime(&lastTime);
      return convertTimeDifferenceToSec(&lastTime, &curTime);
    }

#if defined(USING_MSVC)

    inline void Timer::getHighResolutionTime(TimerStruct* t)
    {
      QueryPerformanceCounter(t);
    }

    inline double Timer::convertTimeDifferenceToSec(TimerStruct* end,
                                                    TimerStruct* begin)
    {
      TimerStruct freq;
      QueryPerformanceFrequency(&freq);
      return (end->QuadPart - begin->QuadPart)/(double)freq.QuadPart;
    }

#elif defined(USING_LINUX)

    inline void Timer::getHighResolutionTime(TimerStruct* t)
    {
      clock_gettime(CLOCK_REALTIME, t);
    }

    inline double Timer::convertTimeDifferenceToSec(TimerStruct* end,
                                                    TimerStruct* begin)
    {
      return ((end->tv_sec - begin->tv_sec) +
              (1e-9)*(end->tv_nsec - begin->tv_nsec));
    }

#elif defined(USING_MACOSX)

    inline void Timer::getHighResolutionTime(TimerStruct* t)
    {
      *t = mach_absolute_time();
    }

    inline double Timer::convertTimeDifferenceToSec(TimerStruct* end,
                                                    TimerStruct* begin)
    {
      uint64_t elapsed = *end - *begin;
      uint64_t enano   = (elapsed*tbaseInfo.numer/tbaseInfo.denom);

      return (1e-9)*((double)(*(uint64_t*)&enano));
    }

#endif

    ostream& operator<<(ostream& os, Timer& timer);

  } // namespace core

} // namespace ato

#endif // Core_Timer_h
