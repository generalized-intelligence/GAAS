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

#include <nvbio/basic/atomics.h>
#include <nvbio/basic/threads.h>

#ifdef WIN32

#include <windows.h>

namespace nvbio {

int32 atomic_increment(int32 volatile *value) { return InterlockedIncrement(reinterpret_cast<LONG volatile*>(value)); }
int64 atomic_increment(int64 volatile *value) { return InterlockedIncrement64(value); }

int32 atomic_decrement(int32 volatile *value) { return InterlockedDecrement(reinterpret_cast<LONG volatile*>(value)); }
int64 atomic_decrement(int64 volatile *value) { return InterlockedDecrement64(value); }

} // namespace nvbio

#endif

namespace nvbio {

void host_release_fence()
{
    #if defined(__GNUC__)
    // make sure the other threads see the reference count before the output is set
    __atomic_thread_fence( __ATOMIC_RELEASE );
    #endif
}

void host_acquire_fence()
{
    #if defined(__GNUC__)
    // make sure the other threads see the reference count before the output is set
    __atomic_thread_fence( __ATOMIC_ACQUIRE );
    #endif
}

int32 host_atomic_add(int32* value, const int32 op)
{
#if defined(__GNUC__)
    return __atomic_fetch_add( value, op, __ATOMIC_RELAXED );
#else
    Mutex mutex;
    ScopedLock lock( &mutex );

    const int32 old = *value;
    *value += op;
    return old;
#endif
}
uint32 host_atomic_add(uint32* value, const uint32 op)
{
#if defined(__GNUC__)
    return __atomic_fetch_add( value, op, __ATOMIC_RELAXED );
#else
    Mutex mutex;
    ScopedLock lock( &mutex );

    const uint32 old = *value;
    *value += op;
    return old;
#endif
}
int64 host_atomic_add(int64* value, const int64 op)
{
#if defined(__GNUC__)
    return __atomic_fetch_add( value, op, __ATOMIC_RELAXED );
#else
    Mutex mutex;
    ScopedLock lock( &mutex );

    const int64 old = *value;
    *value += op;
    return old;
#endif
}
uint64 host_atomic_add(uint64* value, const uint64 op)
{
#if defined(__GNUC__)
    return __atomic_fetch_add( value, op, __ATOMIC_RELAXED );
#else
    Mutex mutex;
    ScopedLock lock( &mutex );

    const uint64 old = *value;
    *value += op;
    return old;
#endif
}
int32 host_atomic_sub(int32* value, const int32 op)
{
#if defined(__GNUC__)
    return __atomic_fetch_sub( value, op, __ATOMIC_RELAXED );
#else
    Mutex mutex;
    ScopedLock lock( &mutex );

    const int32 old = *value;
    *value -= op;
    return old;
#endif
}
uint32 host_atomic_sub(uint32* value, const uint32 op)
{
#if defined(__GNUC__)
    return __atomic_fetch_sub( value, op, __ATOMIC_RELAXED );
#else
    Mutex mutex;
    ScopedLock lock( &mutex );

    const uint32 old = *value;
    *value -= op;
    return old;
#endif
}

int64 host_atomic_sub(int64* value, const int64 op)
{
#if defined(__GNUC__)
    return __atomic_fetch_sub( value, op, __ATOMIC_RELAXED );
#else
    Mutex mutex;
    ScopedLock lock( &mutex );

    const int64 old = *value;
    *value -= op;
    return old;
#endif
}
uint64 host_atomic_sub(uint64* value, const uint64 op)
{
#if defined(__GNUC__)
    return __atomic_fetch_sub( value, op, __ATOMIC_RELAXED );
#else
    Mutex mutex;
    ScopedLock lock( &mutex );

    const uint64 old = *value;
    *value -= op;
    return old;
#endif
}

uint32 host_atomic_or(uint32* value, const uint32 op)
{
#if defined(__GNUC__)
    return __atomic_fetch_or( value, op, __ATOMIC_RELAXED );
#else
    Mutex mutex;
    ScopedLock lock( &mutex );

    const uint32 old = *value;
    *value |= op;
    return old;
#endif
}
uint64 host_atomic_or(uint64* value, const uint64 op)
{
#if defined(__GNUC__)
    return __atomic_fetch_or( value, op, __ATOMIC_RELAXED );
#else
    Mutex mutex;
    ScopedLock lock( &mutex );

    const uint64 old = *value;
    *value |= op;
    return old;
#endif
}

} // namespace nvbio
