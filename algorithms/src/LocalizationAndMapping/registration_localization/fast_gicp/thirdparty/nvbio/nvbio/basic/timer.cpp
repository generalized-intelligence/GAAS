/*
 * nvbio
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the NVIDIA CORPORATION nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <nvbio/basic/timer.h>

#ifdef WIN32

#include <windows.h>
#include <winbase.h>

namespace nvbio {

Timer::Timer()
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);

	m_freq = freq.QuadPart;
}

void Timer::start()
{
	LARGE_INTEGER tick;
	QueryPerformanceCounter(&tick);

	m_start = tick.QuadPart;
}
void Timer::stop()
{
	LARGE_INTEGER tick;
	QueryPerformanceCounter(&tick);

	m_stop = tick.QuadPart;
}

float Timer::seconds() const
{
	return float(double(m_stop - m_start) / double(m_freq));
}

} // namespace nvbio

#else

#include <time.h>
#include <sys/time.h>

namespace nvbio {

#if 0

void Timer::start()
{
    m_start = clock();
}
void Timer::stop()
{
    m_stop = clock();
}

float Timer::seconds() const
{
	return float(m_stop - m_start) / float(CLOCKS_PER_SEC);
}

#elif 0

void Timer::start()
{
    timespec _time;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&_time);
    m_start    = _time.tv_sec;
    m_start_ns = _time.tv_nsec;
}
void Timer::stop()
{
    timespec _time;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&_time);
    m_stop    = _time.tv_sec;
    m_stop_ns = _time.tv_nsec;
}

float Timer::seconds() const
{
    if (m_stop_ns < m_start_ns)
    	return float( double(m_stop - m_start - 1) + double(1000000000 + m_stop_ns - m_start_ns)*1.0e-9 );
    else
    	return float( double(m_stop - m_start) + double(m_stop_ns - m_start_ns)*1.0e-9 );
}

#else

void Timer::start()
{
    timeval _time;
    gettimeofday(&_time,NULL);
    m_start    = _time.tv_sec;
    m_start_ns = _time.tv_usec;
}
void Timer::stop()
{
    timeval _time;
    gettimeofday(&_time,NULL);
    m_stop    = _time.tv_sec;
    m_stop_ns = _time.tv_usec;
}

float Timer::seconds() const
{
    if (m_stop_ns < m_start_ns)
    	return float( double(m_stop - m_start - 1) + double(1000000 + m_stop_ns - m_start_ns)*1.0e-6 );
    else
    	return float( double(m_stop - m_start) + double(m_stop_ns - m_start_ns)*1.0e-6 );
}

#endif

} // namespace nvbio

#endif
