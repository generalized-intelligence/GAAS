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

//#define USE_WARP_VOTE


namespace nvbio {

NVBIO_FORCEINLINE NVBIO_DEVICE void warp_incff(uint32* count)
{
#ifdef USE_WARP_VOTE
    *(volatile uint32*)count += __popc( __ballot( true ) );
#else
    atomicInc( count, uint32(-1) );
#endif
}
NVBIO_FORCEINLINE NVBIO_DEVICE uint32 warp_inc(uint32* count)
{
#ifdef USE_WARP_VOTE
    const volatile uint32 val = *(volatile uint32*)count;
    const volatile uint32 mask = __ballot( true );
    const uint32   warp_count  = __popc( mask );
    const uint32   warp_scan   = __popc( mask >> warp_tid() ) - 1u;
    if (warp_scan == 0)
        *(volatile uint32*)count = val + warp_count;
    return val + warp_scan;
#else
    return atomicInc( count, uint32(-1) );
#endif
}
NVBIO_FORCEINLINE NVBIO_DEVICE void warp_decff(uint32* count)
{
#ifdef USE_WARP_VOTE
    *(volatile uint32*)count -= __popc( __ballot( true ) );
#else
    atomicDec( count, uint32(-1) );
#endif
}
NVBIO_FORCEINLINE NVBIO_DEVICE uint32 warp_dec(uint32* count)
{
#ifdef USE_WARP_VOTE
    const volatile uint32 val  = *(volatile uint32*)count;
    const volatile uint32 mask = __ballot( true );
    const uint32   warp_count  = __popc( mask );
    const uint32   warp_scan   = __popc( mask >> warp_tid() ) - 1u;
    if (warp_scan == 0)
        *(volatile uint32*)count = val - warp_count;
    return val - warp_scan;
#else
    return atomicDec( count, uint32(-1) );
#endif
}

} // namespace nvbio
