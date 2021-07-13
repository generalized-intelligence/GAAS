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

namespace nvbio {

#if defined(NVBIO_ENABLE_PROFILING)
#if defined(__CUDA_ARCH__)
#define NVBIO_INC_UTILIZATION(v1,v2) \
do { \
    atomicAdd( (uint32*)&v1, 1 ); \
    const uint32 mask = __ballot(true); \
    if (__popc( mask >> warp_tid() ) == 1) \
        v2 += 32; \
} while (0)
#define NVBIO_STATS_SET(x,v)          x = v
#define NVBIO_STATS_ADD(x,v)          x += v
#else
#define NVBIO_INC_UTILIZATION(v1,v2)
#define NVBIO_STATS_SET(x,v)          x = v
#define NVBIO_STATS_ADD(x,v)          x += v
#define NVBIO_STATS(stmnt)            stmnt
#endif
#else
#define NVBIO_INC_UTILIZATION(v1,v2)
#define NVBIO_STATS_SET(x,v)
#define NVBIO_STATS_ADD(x,v)
#define NVBIO_STATS(stmnt)
#endif

} // namespace nvbio
