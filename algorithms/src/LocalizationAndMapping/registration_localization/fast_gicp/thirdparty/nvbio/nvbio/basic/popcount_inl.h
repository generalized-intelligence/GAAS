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

namespace nvbio {

#ifdef __CUDACC__
NVBIO_FORCEINLINE NVBIO_DEVICE uint32 device_popc(const uint32 i) { return __popc(i); }
#endif

#ifdef __CUDACC__
NVBIO_FORCEINLINE NVBIO_DEVICE uint32 device_popc(const uint64 i) { return __popcll(i); }
#endif

// int32 popcount
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc(const int32 i)
{
    return popc(uint32(i));
}
// uint32 popcount
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc(const uint32 i)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return device_popc( i );
#elif defined(__GNUC__)
    return __builtin_popcount( i );
#else
    uint32 v = i;
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    v = (v + (v >> 4)) & 0x0F0F0F0F;
    return (v * 0x01010101) >> 24;
#endif
}

// uint64 popcount
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc(const uint64 i)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return device_popc( i );
#elif defined(__GNUC__)
    return __builtin_popcountll( i );
#else
    //return popc( uint32(i & 0xFFFFFFFFU) ) + popc( uint32(i >> 32) );
    uint64 v = i;
    v = v - ((v >> 1) & 0x5555555555555555U);
    v = (v & 0x3333333333333333U) + ((v >> 2) & 0x3333333333333333U);
    v = (v + (v >> 4)) & 0x0F0F0F0F0F0F0F0FU;
    return (v * 0x0101010101010101U) >> 56;
#endif
}

// uint8 popcount
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc(const uint8 i)
{
    const uint32 lut[16] = { 0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4 };
    return lut[ i & 0x0F ] + lut[ i >> 4 ];
}

// find the n-th bit set in a 4-bit mask (n in [1,4])
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 find_nthbit4(const uint32 mask, const uint32 n)
{
    const uint32 popc0 = (mask & 1u);
    const uint32 popc1 = popc0 + ((mask >> 1u) & 1u);
    const uint32 popc2 = popc1 + ((mask >> 2u) & 1u);
    return (n <= popc1) ?
           (n == popc0) ? 0u : 1u :
           (n == popc2) ? 2u : 3u;
}

// compute the pop-count of 4-bit mask
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc4(const uint32 mask)
{
    return
         (mask        & 1u) +
        ((mask >> 1u) & 1u) +
        ((mask >> 2u) & 1u) +
        ((mask >> 3u) & 1u);
}

// find the n-th bit set in a 8-bit mask (n in [1,8])
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 find_nthbit8(const uint32 mask, const uint32 n)
{
    const uint32 popc_half = popc4( mask );
    uint32 _mask;
    uint32 _n;
    uint32 _r;

    if (n <= popc_half)
    {
        _mask = mask;
        _n    = n;
        _r    = 0u;
    }
    else
    {
        _mask = mask >> 4u;
        _n    = n - popc_half;
        _r    = 4u;
    }
    return find_nthbit4( _mask, _n ) + _r;
}

// find the n-th bit set in a 16-bit mask (n in [1,16])
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 find_nthbit16(const uint32 mask, const uint32 n)
{
    const uint32 popc_half = popc( mask & 0xFu );

    uint32 _mask;
    uint32 _n;
    uint32 _r;

    if (n <= popc_half)
    {
        _mask = mask;
        _n    = n;
        _r    = 0u;
    }
    else
    {
        _mask = mask >> 8u;
        _n    = n - popc_half;
        _r    = 8u;
    }
    return find_nthbit8( _mask, _n ) + _r;
}

// find the n-th bit set in a 32-bit mask (n in [1,32])
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 find_nthbit(const uint32 mask, const uint32 n)
{
    const uint32 popc_half = popc( mask & 0xFFu );

    uint32 _mask;
    uint32 _n;
    uint32 _r;

    if (n <= popc_half)
    {
        _mask = mask;
        _n    = n;
        _r    = 0u;
    }
    else
    {
        _mask = mask >> 16u;
        _n    = n - popc_half;
        _r    = 16u;
    }
    return find_nthbit16( _mask, _n ) + _r;
}

// find the n-th bit set in a 8-bit mask (n in [1,8])
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 find_nthbit(const uint8 mask, const uint32 n)
{
    return find_nthbit8( mask, n );
}

// find the n-th bit set in a 16-bit mask (n in [1,16])
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 find_nthbit(const uint16 mask, const uint32 n)
{
    return find_nthbit16( mask, n );
}

#if defined(NVBIO_DEVICE_COMPILATION)
NVBIO_FORCEINLINE NVBIO_DEVICE uint32 ffs_device(const int32 x) { return __ffs(x); }
NVBIO_FORCEINLINE NVBIO_DEVICE uint32 lzc_device(const int32 x) { return __clz(x); }
#endif

// find the least significant bit set
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 ffs(const int32 x)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return ffs_device(x);
#elif defined(__GNUC__)
    return __builtin_ffs(x);
#else
    return x ? popc(x ^ (~(-x))) : 0u;
#endif
}

// count the number of leading zeros
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 lzc(const uint32 x)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return lzc_device(x);
#elif defined(__GNUC__)
    return __builtin_clz(x);
#else
    uint32 y = x;
    y |= (y >> 1);
    y |= (y >> 2);
    y |= (y >> 4);
    y |= (y >> 8);
    y |= (y >> 16);
    return (32u - popc(y));
#endif
}

// count the number of occurrences of a given 2-bit pattern in a given word
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit(const uint32 x, int c)
{
    const uint32 odd  = ((c&2)? x : ~x) >> 1;
    const uint32 even = ((c&1)? x : ~x);
    const uint32 mask = odd & even & 0x55555555;
    return popc(mask);
}

// count the number of occurrences of a given 2-bit pattern in a given word
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit(const uint64 x, int c)
{
    const uint64 odd  = ((c&2)? x : ~x) >> 1;
    const uint64 even = ((c&1)? x : ~x);
    const uint64 mask = odd & even & 0x5555555555555555U;
    return popc(mask);
}

// count the number of occurrences of all 2-bit patterns in a given word,
// using an auxiliary table.
//
// \param count_table      auxiliary table to perform the parallel bit-counting
//                         for all integers in the range [0,255].
//
// \return                 the 4 pop counts shifted and OR'ed together
//
template <typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit_all(
    const uint32 b,
    const CountTable count_table)
{
#if 1
    return
        count_table[(b)&0xff]     + count_table[(b)>>8&0xff] +
        count_table[(b)>>16&0xff] + count_table[(b)>>24];
#elif 0
    return (1u << ((b&3)     << 3)) +
           (1u << ((b>>2&3)  << 3)) + 
           (1u << ((b>>4&3)  << 3)) +
           (1u << ((b>>6&3)  << 3)) +
           (1u << ((b>>8&3)  << 3)) +
           (1u << ((b>>10&3) << 3)) +
           (1u << ((b>>12&3) << 3)) +
           (1u << ((b>>14&3) << 3)) +
           (1u << ((b>>16&3) << 3)) +
           (1u << ((b>>18&3) << 3)) +
           (1u << ((b>>20&3) << 3)) +
           (1u << ((b>>22&3) << 3)) +
           (1u << ((b>>24&3) << 3)) +
           (1u << ((b>>26&3) << 3)) +
           (1u << ((b>>28&3) << 3)) +
           (1u << ((b>>30)   << 3));
#else
    const uint32 n1 = popc( ~b >> 1 & ~b & 0x55555555 );
    const uint32 n2 = popc( ~b >> 1 &  b & 0x55555555 );
    const uint32 n3 = popc(  b >> 1 & ~b & 0x55555555 );
    const uint32 n4 = 16u - n1 - n2 - n3;

    return  n1 +
           (n2 << 8) +
           (n3 << 16) +
           (n4 << 24);
#endif
}

// count the number of occurrences of all 2-bit patterns in a given word,
// using an auxiliary table.
//
// \param count_table      auxiliary table to perform the parallel bit-counting
//                         for all integers in the range [0,255].
//
// \return                 the 4 pop counts shifted and OR'ed together
//
template <typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit_all(
    const uint64 b,
    const CountTable count_table)
{
    return
        count_table[(b)&0xff]     + count_table[(b)>>8&0xff] +
        count_table[(b)>>16&0xff] + count_table[(b)>>24&0xff] +
        count_table[(b)>>32&0xff] + count_table[(b)>>40&0xff] +
        count_table[(b)>>48&0xff] + count_table[(b)>>56];
}

// given a 32-bit word encoding a set of 2-bit symbols, return a submask containing
// all but the first 'i' entries.
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 hibits_2bit(const uint32 mask, const uint32 i)
{
    return mask & ~((1u<<(i<<1)) - 1u);
}

// given a 64-bit word encoding a set of 2-bit symbols, return a submask containing
// all but the first 'i' entries.
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint64 hibits_2bit(const uint64 mask, const uint32 i)
{
    return mask & ~((uint64(1u)<<(i<<1)) - 1u);
}

// count the number of occurrences of a given 2-bit pattern in all but the first 'i' symbols
// of a 32-bit word mask.
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit(const uint32 mask, int c, const uint32 i)
{
    const uint32 r = popc_2bit( hibits_2bit( mask, i ), c );

    // if the 2-bit pattern we're looking for is 0, we have to subtract
    // the amount of symbols we added by masking
    return (c == 0) ? r - i : r;
}

// count the number of occurrences of a given 2-bit pattern in all but the first 'i' symbols
// of a 32-bit word mask.
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit(const uint64 mask, int c, const uint32 i)
{
    const uint32 r = popc_2bit( hibits_2bit( mask, i ), c );

    // if the 2-bit pattern we're looking for is 0, we have to subtract
    // the amount of symbols we added by masking
    return (c == 0) ? r - i : r;
}

// count the number of occurrences of a given n-bit pattern in a given word
//
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_nbit(const uint32 x, int c)
{
    if (N == 1)
    {
        const uint32 r = popc(x);
        return c ? r : 32u - r;
    }
    else if (N == 2)
        return popc_2bit(x,c);
    else if (N == 4)
    {
        // use brute-force - TODO: find a clever algorithm
        return (((x >>  0) & 15u) == c) +
               (((x >>  4) & 15u) == c) +
               (((x >>  8) & 15u) == c) +
               (((x >> 12) & 15u) == c) +
               (((x >> 16) & 15u) == c) +
               (((x >> 20) & 15u) == c) +
               (((x >> 24) & 15u) == c) +
               (((x >> 28) & 15u) == c);
    }
    else if (N == 8)
    {
        // use brute-force - TODO: find a clever algorithm
        return (((x >>  0) & 255u) == c) +
               (((x >>  8) & 255u) == c) +
               (((x >> 16) & 255u) == c) +
               (((x >> 24) & 255u) == c);
    }
    return 0u;
}

// count the number of occurrences of a given n-bit pattern in a given word
//
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_nbit(const uint64 x, int c)
{
    if (N == 1)
    {
        const uint32 r = popc(x);
        return c ? r : 64u - r;
    }
    else if (N == 2)
        return popc_2bit(x,c);
    else
    {
        return popc_nbit<N>( uint32(x),       c ) +
               popc_nbit<N>( uint32(x >> 32), c );
    }
}

// given a 32-bit word encoding a set of n-bit symbols, return a submask containing
// all but the first 'i' entries.
//
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 hibits_nbit(const uint32 mask, const uint32 i)
{
    const uint32 LOG_N = (N == 1) ? 0u :
                         (N == 2) ? 1u :
                         (N == 4) ? 2u :
                                    3u;

    return mask & ~((1u<<(i<<LOG_N)) - 1u);
}

// given a 64-bit word encoding a set of n-bit symbols, return a submask containing
// all but the first 'i' entries.
//
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint64 hibits_nbit(const uint64 mask, const uint32 i)
{
    const uint32 LOG_N = (N == 1) ? 1u :
                         (N == 2) ? 1u :
                         (N == 4) ? 2u :
                                    3u;

    return mask & ~((uint64(1u)<<(i<<LOG_N)) - 1u);
}

// count the number of occurrences of a given n-bit pattern in all but the first 'i' symbols
// of a 32-bit word mask.
//
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_nbit(const uint32 mask, int c, const uint32 i)
{
    const uint32 r = popc_nbit<N>( hibits_nbit<N>( mask, i ), c );

    // if the n-bit pattern we're looking for is 0, we have to subtract
    // the amount of symbols we added by masking
    return (c == 0) ? r - i : r;
}

// count the number of occurrences of a given n-bit pattern in all but the first 'i' symbols
// of a 32-bit word mask.
//
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_nbit(const uint64 mask, int c, const uint32 i)
{
    const uint32 r = popc_nbit<N>( hibits_nbit<N>( mask, i ), c );

    // if the n-bit pattern we're looking for is 0, we have to subtract
    // the amount of symbols we added by masking
    return (c == 0) ? r - i : r;
}

// count the number of occurrences of all 2-bit patterns in all but the first 'i' symbols
// of a given word, using an auxiliary table.
//
// \param count_table      auxiliary table to perform the parallel bit-counting
//                         for all integers in the range [0,255].
//
// \return                 the 4 pop counts shifted and OR'ed together
//
template <typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit_all(
    const uint32     mask,
    const CountTable count_table,
    const uint32     i)
{
    return popc_2bit_all( hibits_2bit( mask, i ), count_table ) - i;
}

// count the number of occurrences of all 2-bit patterns in all but the first 'i' symbols
// of a given word, using an auxiliary table.
//
// \param count_table      auxiliary table to perform the parallel bit-counting
//                         for all integers in the range [0,255].
//
// \return                 the 4 pop counts shifted and OR'ed together
//
template <typename CountTable>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 popc_2bit_all(
    const uint64     mask,
    const CountTable count_table,
    const uint32     i)
{
    return popc_2bit_all( hibits_2bit( mask, i ), count_table ) - i;
}

} // namespace nvbio
