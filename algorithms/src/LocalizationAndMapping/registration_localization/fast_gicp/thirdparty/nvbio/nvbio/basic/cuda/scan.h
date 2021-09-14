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

/*! \file scan.h
 *   \brief Define CUDA based scan primitives.
 */

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/numbers.h>

namespace nvbio {
namespace cuda {

/// intra-warp inclusive scan
///
/// \param val      per-threrad input value
template <typename T> __device__ __forceinline__ T bit_scan(bool p)
{
    const uint32 mask = __ballot( p );
    const uint32 pop_scan  = __popc( mask << (warpSize - warp_tid() - 1u) );
	return pop_scan;
}

/// intra-warp inclusive scan
///
/// \param val      per-threrad input value
/// \param tidx     warp thread index
/// \param red      scan result storage (2*WARP_SIZE elements)
template <typename T> __device__ __forceinline__ T scan_warp(T val, const int32 tidx, volatile T *red)
{
    // pad initial segment with zeros
    red[tidx] = 0;
    red += 32;

    // Hillis-Steele scan
    red[tidx] = val;
    val += red[tidx-1];  red[tidx] = val;
    val += red[tidx-2];  red[tidx] = val;
    val += red[tidx-4];  red[tidx] = val;
    val += red[tidx-8];  red[tidx] = val;
    val += red[tidx-16]; red[tidx] = val;
	return val;
}
/// return the total from a scan_warp
///
/// \param red      scan result storage
template <typename T> __device__ __forceinline__ T scan_warp_total(volatile T *red) { return red[63]; }

// intra-warp inclusive scan
//
// \param val      per-threrad input value
// \param tidx     warp thread index
// \param red      scan result storage (2*WARP_SIZE elements)
template <typename T, typename Op, uint32 COUNT>
struct scan_dispatch {};

// intra-warp inclusive scan
//
// \param val      per-threrad input value
// \param tidx     warp thread index
// \param red      scan result storage (2*WARP_SIZE elements)
template <typename T, typename Op>
struct scan_dispatch<T,Op,32>
{
    static __device__ __forceinline__ T scan(T val, const Op op, const int32 tidx, volatile T *red, const T init)
    {
        // pad initial segment with zeros
        red[tidx] = init;
        red += 32;

        // Hillis-Steele scan
        red[tidx] = val;
        val = op( val, red[tidx-1] );  red[tidx] = val;
        val = op( val, red[tidx-2] );  red[tidx] = val;
        val = op( val, red[tidx-4] );  red[tidx] = val;
        val = op( val, red[tidx-8] );  red[tidx] = val;
        val = op( val, red[tidx-16] ); red[tidx] = val;
	    return val;
    }
    static __device__ __forceinline__ T scan_total(volatile T *red) { return red[63]; }
};
// intra-warp inclusive scan
//
// \param val      per-threrad input value
// \param tidx     warp thread index
// \param red      scan result storage (2*WARP_SIZE elements)
template <typename T, typename Op>
struct scan_dispatch<T,Op,16>
{
    static __device__ __forceinline__ T scan(T val, const Op op, const int32 tidx, volatile T *red, const T init)
    {
        // pad initial segment with zeros
        red[tidx] = init;
        red += 32;

        // Hillis-Steele scan
        if (tidx < 16)
        {
            red[tidx] = val;
            val = op( val, red[tidx-1] );  red[tidx] = val;
            val = op( val, red[tidx-2] );  red[tidx] = val;
            val = op( val, red[tidx-4] );  red[tidx] = val;
            val = op( val, red[tidx-8] );  red[tidx] = val;
        }
	    return val;
    }
    static __device__ __forceinline__ T scan_total(volatile T *red) { return red[47]; }
};
// intra-warp inclusive scan
//
// \param val      per-threrad input value
// \param tidx     warp thread index
// \param red      scan result storage (2*WARP_SIZE elements)
template <typename T, typename Op>
struct scan_dispatch<T,Op,8>
{
    static __device__ __forceinline__ T scan(T val, const Op op, const int32 tidx, volatile T *red, const T init)
    {
        // pad initial segment with zeros
        red[tidx] = init;
        red += 32;

        // Hillis-Steele scan
        if (tidx < 8)
        {
            red[tidx] = val;
            val = op( val, red[tidx-1] );  red[tidx] = val;
            val = op( val, red[tidx-2] );  red[tidx] = val;
            val = op( val, red[tidx-4] );  red[tidx] = val;
        }
	    return val;
    }
    static __device__ __forceinline__ T scan_total(volatile T *red) { return red[39]; }
};
// intra-warp inclusive scan
//
// \param val      per-threrad input value
// \param tidx     warp thread index
// \param red      scan result storage (2*WARP_SIZE elements)
template <typename T, typename Op>
struct scan_dispatch<T, Op,4>
{
    static __device__ __forceinline__ T scan(T val, const Op op, const int32 tidx, volatile T *red, const T init)
    {
        // pad initial segment with zeros
        red[tidx] = init;
        red += 32;

        // Hillis-Steele scan
        if (tidx < 4)
        {
            red[tidx] = val;
            val = op( val, red[tidx-1] );  red[tidx] = val;
            val = op( val, red[tidx-2] );  red[tidx] = val;
        }
	    return val;
    }
    static __device__ __forceinline__ T scan_total(volatile T *red) { return red[35]; }
};
// intra-warp inclusive scan
//
// \param val      per-threrad input value
// \param tidx     warp thread index
// \param red      scan result storage (2*WARP_SIZE elements)
template <typename T, typename Op>
struct scan_dispatch<T, Op, 2>
{
    static __device__ __forceinline__ T scan(T val, const Op op, const int32 tidx, volatile T *red, const T init)
    {
        // pad initial segment with zeros
        red[tidx] = init;
        red += 32;

        // Hillis-Steele scan
        if (tidx < 2)
        {
            red[tidx] = val;
            val = op( val, red[tidx-1] );  red[tidx] = val;
        }
	    return val;
    }
    static __device__ __forceinline__ T scan_total(volatile T *red) { return red[33]; }
};

/// intra-warp inclusive scan
///
/// \param val      per-threrad input value
/// \param tidx     warp thread index
/// \param red      scan result storage (2*WARP_SIZE elements)
template <uint32 COUNT,typename T, typename Op> __device__ __forceinline__ T scan(T val, const Op op, const T init, volatile T *red)
{
    return scan_dispatch<T,Op,COUNT>::scan( val, op, warp_tid(), red, init );
}
/// intra-warp inclusive scan
///
/// \param val      per-threrad input value
/// \param tidx     warp thread index
/// \param red      scan result storage (2*WARP_SIZE elements)
template <uint32 COUNT,typename T> __device__ __forceinline__ T scan(T val, volatile T *red)
{
    return scan_dispatch<T,add_functor,COUNT>::scan( val, add_functor(), warp_tid(), red, T(0) );
}
/// return the total from a scan_warp
///
/// \param red      scan result storage
template <uint32 COUNT,typename T> __device__ __forceinline__ T scan_total(volatile T *red)
{
    return scan_dispatch<T,add_functor,COUNT>::scan_total( red );
}


/// alloc n elements per thread from a common pool, using a synchronous warp scan
///
/// \param n                number of elements to alloc
/// \param warp_tid         warp thread index
/// \param warp_red         temporary warp scan storage (2*WARP_SIZE elements)
/// \param warp_broadcast   temporary warp broadcasting storage
__device__ __forceinline__
uint32 alloc(uint32 n, uint32* pool, const int32 warp_tid, volatile uint32* warp_red, volatile uint32* warp_broadcast)
{
    uint32 warp_scan  = scan_warp( n, warp_tid, warp_red ) - n;
    uint32 warp_count = scan_warp_total( warp_red );
    if (warp_tid == 0)
        *warp_broadcast = atomicAdd( pool, warp_count );

    return *warp_broadcast + warp_scan;
}

/// alloc zero or exactly N elements per thread from a common pool
///
/// \param p                allocation predicate
/// \param warp_tid         warp thread id
/// \param warp_broadcast   temporary warp broadcasting storage
template <uint32 N>
__device__ __forceinline__
uint32 alloc(bool pred, uint32* pool, const int32 warp_tid, volatile uint32* warp_broadcast)
{
    const uint32 warp_mask  = __ballot( pred );
    const uint32 warp_count = __popc( warp_mask );
    const uint32 warp_scan  = __popc( warp_mask << (warpSize - warp_tid) );

    // acquire an offset for this warp
    if (warp_scan == 0 && pred)
        *warp_broadcast = atomicAdd( pool, warp_count * N );

    // find offset
    return *warp_broadcast + warp_scan * N;
}

// generalized all primitive
template <uint32 COUNT>
struct all_dispatch {};

// generalized all primitive
template <>
struct all_dispatch<32>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        return __all(p);
    }
};

// generalized all primitive
template <>
struct all_dispatch<2>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        const uint32 mask  = __ballot(p);
        const uint32 tid   = (threadIdx.x & 31) >> 1;
        const uint32 tmask = 3u << (tid*2);
        return (mask & tmask) == tmask;
    }
};

// generalized all primitive
template <>
struct all_dispatch<4>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        const uint32 mask  = __ballot(p);
        const uint32 tid   = (threadIdx.x & 31) >> 2;
        const uint32 tmask = 15u << (tid*4);
        return (mask & tmask) == tmask;
    }
};

// generalized all primitive
template <>
struct all_dispatch<8>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        const uint32 mask  = __ballot(p);
        const uint32 tid   = (threadIdx.x & 31) >> 3;
        const uint32 tmask = 255u << (tid*8);
        return (mask & tmask) == tmask;
    }
};

// generalized all primitive
template <>
struct all_dispatch<16>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        const uint32 mask  = __ballot(p);
        const uint32 tid   = (threadIdx.x & 31) >> 4;
        const uint32 tmask = 65535u << (tid*16);
        return (mask & tmask) == tmask;
    }
};

// generalized all primitive
template <>
struct all_dispatch<64>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        sm[ warp_id() ] = __all(p);
        __syncthreads();

        // compute the block id
        const uint32 tid = warp_id() >> 1;
        return sm[ tid*2 ] & sm[ tid*2 + 1 ];
    }
};
// generalized all primitive
template <>
struct all_dispatch<128>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        sm[ warp_id() ] = __all(p);
        __syncthreads();

        // check whether all warps in this block are set
        const uint32 bid = warp_id() >> 2;
        return __all( sm[ bid * 4 + warp_tid() & 3 ] );
    }
};
// generalized all primitive
template <>
struct all_dispatch<256>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        sm[ warp_id() ] = __all(p);
        __syncthreads();

        // check whether all warps in this block are set
        const uint32 bid = warp_id() >> 3;
        return __all( sm[ bid * 8 + warp_tid() & 7 ] );
    }
};

/// generalized all primitive for arbitrarily (power of 2) sized thread groups
///
/// \tparam COUNT   thread-group size
/// \param p        per-thread predicate
/// \param sm       shared-memory array needed when COUNT is larger than 32
///
/// \return         true iff the predicate is true for all threads in the thread-group
///
template <uint32 COUNT>
__device__ __forceinline__ bool all(const bool p, volatile uint8* sm = NULL)
{
    return all_dispatch<COUNT>::enact( p, sm );
}

// generalized any primitive
template <uint32 COUNT>
struct any_dispatch {};

// generalized any primitive
template <>
struct any_dispatch<32>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        return __any(p);
    }
};

// generalized any primitive
template <>
struct any_dispatch<2>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        const uint32 mask  = __ballot(p);
        const uint32 tid   = (threadIdx.x & 31) >> 1;
        const uint32 tmask = 3u << (tid*2);
        return (mask & tmask);
    }
};

// generalized any primitive
template <>
struct any_dispatch<4>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        const uint32 mask  = __ballot(p);
        const uint32 tid   = (threadIdx.x & 31) >> 2;
        const uint32 tmask = 15u << (tid*4);
        return (mask & tmask);
    }
};

// generalized any primitive
template <>
struct any_dispatch<8>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        const uint32 mask  = __ballot(p);
        const uint32 tid   = (threadIdx.x & 31) >> 3;
        const uint32 tmask = 255u << (tid*8);
        return (mask & tmask);
    }
};

// generalized any primitive
template <>
struct any_dispatch<16>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        const uint32 mask  = __ballot(p);
        const uint32 tid   = (threadIdx.x & 31) >> 4;
        const uint32 tmask = 65535u << (tid*16);
        return (mask & tmask);
    }
};

// generalized any primitive
template <>
struct any_dispatch<64>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        sm[ warp_id() ] = __any(p);
        __syncthreads();

        // compute the block id
        const uint32 tid = warp_id() >> 1;
        return sm[ tid*2 ] | sm[ tid*2 + 1 ];
    }
};
// generalized any primitive
template <>
struct any_dispatch<128>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        sm[ warp_id() ] = __any(p);
        __syncthreads();

        // check whether any warps in this block are set
        const uint32 bid = warp_id() >> 2;
        return __any( sm[ bid * 4 + warp_tid() & 3 ] );
    }
};
// generalized any primitive
template <>
struct any_dispatch<256>
{
    static __device__ __forceinline__ bool enact(const bool p, volatile uint8* sm)
    {
        sm[ warp_id() ] = __any(p);
        __syncthreads();

        // check whether any warps in this block are set
        const uint32 bid = warp_id() >> 3;
        return __any( sm[ bid * 8 + warp_tid() & 7 ] );
    }
};

/// generalized any primitive for arbitrarily (power of 2) sized thread groups
///
/// \tparam COUNT   thread-group size
/// \param p        per-thread predicate
/// \param sm       shared-memory array needed when COUNT is larger than 32
///
/// \return         true iff the predicate is true for any threads in the thread-group
///
template <uint32 COUNT>
__device__ __forceinline__ bool any(const bool p, volatile uint8* sm = NULL)
{
    return any_dispatch<COUNT>::enact( p, sm );
}

} // namespace cuda
} // namespace nvbio
