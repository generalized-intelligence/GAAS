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

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/numbers.h>

#include <string>
#include <deque>

namespace nvbio {

/// \page timing_page Timing
///
/// This module implements basic some timers and timing-related functionality:
///
/// - Timer
/// - FakeTimer
/// - TimeSeries
///

#ifdef WIN32

///
/// A simple timer class
///
struct Timer
{
	/// constructor
	Timer();

	/// start timing
	void start();

	/// stop timing
	void stop();

	float seconds() const;

	uint64			m_freq;
	uint64			m_start;
	uint64			m_stop;
};

#else

///
/// A simple timer class
///
struct Timer
{
	/// constructor
    Timer() {}

	/// start timing
	void start();

	/// stop timing
    void stop();

    float seconds() const;

private:
    int64 m_start;
    int64 m_stop;
    int64 m_start_ns;
    int64 m_stop_ns;
};

#endif

///
/// A helper timer which measures the time from its instantiation
/// to the moment it goes out of scope
///
template <typename T>
struct ScopedTimer
{
	 ScopedTimer(T* time) : m_time( time ), m_timer() { m_timer.start(); }
	~ScopedTimer() { m_timer.stop(); *m_time += m_timer.seconds(); }

	T*		m_time;
	Timer	m_timer;
};

///
/// A zero-overhead timer which doesn't do any timing...
///
struct FakeTimer
{
    void start();
    void stop();

    float seconds() const { return 0.0f; }
};

///
/// A class used to keep track of several timing statistics for repeating kernel or function calls
///
struct TimeSeries
{
    /// constructor
    ///
    TimeSeries() : num(0), calls(0), time(0.0f), device_time(0.0f), max_speed(0.0f)
    {
        for (uint32 i = 0; i < 32; ++i)
        {
            bin_calls[i] = 0;
            bin_items[i] = 0;
            bin_time[i]  = 0.0f;
            bin_speed[i] = 0.0f;

            user[i] = 0.0f;
            user_names[i] = NULL;
            user_units[i] = "";
        }
    }

    /// add a sample
    ///
    /// \param c        the number of calls / items processed
    /// \param t        the amount of time spent
    /// \param dt       the amount of device-time spent
    void add(const uint32 c, const float t, const float dt = 0.0f)
    {
        num++;
        calls += c;
        time  += t;
        device_time += dt;
        max_speed = std::max( max_speed, float(c) / t );
        if (info.size() == 10000)
            info.pop_front();
        info.push_back( std::make_pair( c, t ) );

        const uint32 bin = c ? nvbio::log2( c ) : 0u;
        bin_calls[bin]++;
        bin_time[bin]  += t;
        bin_speed[bin] += float(c) / t;
        bin_items[bin] += c;
    }

    // return the average speed
    float avg_speed() const { return float(calls) / time; }

    std::string                             name;
    std::string                             units;

    uint32                                  num;
    uint64                                  calls;
    float                                   time;
    float                                   device_time;
    float                                   max_speed;
    uint32                                  bin_calls[32];
    uint64                                  bin_items[32];
    float                                   bin_time[32];
    float                                   bin_speed[32];
    std::deque< std::pair<uint32,float> >   info;

    float                                   user[32];
    const char*                             user_names[32];
    const char*                             user_units[32];
    bool                                    user_avg[32];
};

} // namespace nvbio
