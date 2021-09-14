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

#include <nvbio/basic/timer.h>
#include <nvbio/basic/cuda/timer.h>
#include <cuda_runtime.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

inline void optional_device_synchronize()
{
#if DO_OPTIONAL_SYNCHRONIZE
    cudaDeviceSynchronize();
#endif
}

#if DO_DEVICE_TIMING
typedef nvbio::cuda::Timer DeviceTimer;
#else
typedef nvbio::FakeTimer   DeviceTimer;
#endif

// convert a range from a [begin,last] representation to a [begin,end) one
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint2 inclusive_to_exclusive(const uint2 range) { return make_uint2( range.x, range.y + 1u ); }

#ifdef __CUDACC__

// alloc some slots from a counter using a warp-synchronous reduction
NVBIO_FORCEINLINE NVBIO_DEVICE uint32 alloc(uint32* counter, volatile uint32 *warp_broadcast)
{
#if USE_WARP_SYNCHRONOUS_QUEUES
    const uint32 mask = __ballot( true );
    const uint32 pop_scan  = __popc( mask << (32u - warp_tid()) );
    const uint32 pop_count = __popc( mask );
    if (pop_scan == 0)
        *warp_broadcast = atomicAdd( counter, pop_count );

    return *warp_broadcast + pop_scan;
#else
    return atomicAdd( counter, 1u );
#endif
}

#endif // __CUDACC__

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
