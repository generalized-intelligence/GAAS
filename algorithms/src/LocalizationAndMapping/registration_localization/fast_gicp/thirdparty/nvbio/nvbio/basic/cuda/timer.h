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
#include <cuda_runtime.h>

namespace nvbio {
namespace cuda {

///
/// A CUDA timer
///
struct Timer
{
    /// constructor
    ///
    inline Timer();

    /// destructor
    ///
    inline ~Timer();

	/// start timing
    ///
	inline void start();

	/// stop timing
    ///
	inline void stop();

    /// elapsed seconds
    ///
	inline float seconds() const;

    cudaEvent_t m_start, m_stop;
};

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

// constructor
//
Timer::Timer()
{
    cudaEventCreate(&m_start);
    cudaEventCreate(&m_stop);
}

// destructor
//
Timer::~Timer()
{
    cudaEventDestroy(m_start);
    cudaEventDestroy(m_stop);
}

// start timing
//
void Timer::start()
{
    cudaEventRecord(m_start, 0);
}

// stop timing
//
void Timer::stop()
{
    cudaEventRecord(m_stop, 0);
    cudaEventSynchronize(m_stop);
}

// elapsed seconds
//
float Timer::seconds() const
{
    float elapsedTime;
    cudaEventElapsedTime(&elapsedTime, m_start, m_stop);
    return elapsedTime * 1.0e-3f;
}

} // namespace cuda
} // namespace nvbio
