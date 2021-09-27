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

#include <nvbio/fmindex/paged_text.h>
#include <nvbio/basic/popcount.h>
#include <nvbio/basic/algorithms.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/basic/transform_iterator.h>
#include <stdio.h>
#include <stdlib.h>

#if defined(PLATFORM_X86)
//#define SSE_LOADS
//#define SSE_MATH
#endif

#if defined(SSE_LOADS)
#include <emmintrin.h>
#include <smmintrin.h>
#endif

namespace nvbio {

// build a set of buckets pointing to the lower/upper bounds of a sequence of keys
//
void build_buckets(const uint64 key_range, const uint32 n_keys, const uint64* keys, const uint32 bucket_size, nvbio::vector<host_tag,uint32>& buckets, const bool upper)
{
    const uint32 n_buckets = (uint32)util::divide_ri( key_range, bucket_size ) + 1u;
    buckets.resize( n_buckets );

    #pragma omp parallel for
    for (int32 i = 0; i < int32( n_buckets ); ++i)
    {
        buckets[i] = upper ?
            upper_bound_index( uint64(i) * bucket_size, keys, n_keys ) :
            lower_bound_index( uint64(i) * bucket_size, keys, n_keys );
    }
}

// count the number of occurrences of a given 2-bit pattern in a given word
//
template <uint8 c>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 popc_2bit(const uint64 x)
{
    const uint64 odd  = ((c&2)? x : ~x) >> 1;
    const uint64 even = ((c&1)? x : ~x);
    const uint64 mask = odd & even & 0x5555555555555555U;
    return popc(mask);
}

// count the number of occurrences of a given 2-bit pattern in all but the first 'i' symbols
// of a 32-bit word mask.
//
template <uint8 c>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 popc_2bit(const uint64 mask, const uint32 i)
{
    const uint32 r = popc_2bit<c>( hibits_2bit( mask, i ) );

    // if the 2-bit pattern we're looking for is 0, we have to subtract
    // the amount of symbols we added by masking
    return (c == 0) ? r - i : r;
}

#if defined(SSE_LOADS)

// count the number of occurrences of a given 2-bit pattern in a given word
//
template <uint8 c>
NVBIO_FORCEINLINE
uint32 popc_2bit(const __m128i x)
{
#if defined(SSE_MATH)
    const __m128i ones  = _mm_set_epi64x( (int64)0xFFFFFFFFFFFFFFFFull, (int64)0xFFFFFFFFFFFFFFFFull );
    const __m128i fives = _mm_set_epi64x( (int64)0x5555555555555555ull, (int64)0x5555555555555555ull );
    const __m128i odd   = _mm_srli_epi64( ((c&2)? x : _mm_subs_epu8( ones, x )), 1 );
    const __m128i even  =                 ((c&1)? x : _mm_subs_epu8( ones, x ));
    const __m128i mask  = _mm_and_si128( _mm_and_si128( odd, even ), fives );
    return popc( (uint64)_mm_extract_epi64( mask, 0 ) ) +
           popc( (uint64)_mm_extract_epi64( mask, 1 ) );
#else
    return popc_2bit<c>( (uint64)_mm_extract_epi64(x,0) ) +
           popc_2bit<c>( (uint64)_mm_extract_epi64(x,1) );
#endif
}

// given a 64-bit word encoding a set of 2-bit symbols, return a submask containing
// all but the first 'i' entries.
//
NVBIO_FORCEINLINE __m128i hibits_2bit(const __m128i mask, const uint32 w, const uint32 i)
{
    const uint64 u = ~((uint64(1u) << (i<<1)) - 1u);
    const __m128i m = _mm_set_epi64x(
        (int64)(w == 0 ? u : 0xFFFFFFFFFFFFFFFFull),
        (int64)(w == 1 ? u : 0x0000000000000000ull) );

    return _mm_and_si128( mask, m );
}

// count the number of occurrences of a given 2-bit pattern in a given word
//
template <uint8 c>
NVBIO_FORCEINLINE
uint32 popc_2bit(const __m128i mask, const uint32 w, const uint32 i)
{
#if 0 && defined(SSE_MATH)
    const uint32 r = popc_2bit<c>( hibits_2bit( mask, w, i ) );

    // if the 2-bit pattern we're looking for is 0, we have to subtract
    // the amount of symbols we added by masking
    return (c == 0) ? r - i - w*32u : r;
#else
    if (w == 0)
        return popc_2bit<c>( (uint64)_mm_extract_epi64(mask,0), i );
    else
        return popc_2bit<c>( (uint64)_mm_extract_epi64(mask,0) ) +
               popc_2bit<c>( (uint64)_mm_extract_epi64(mask,1), i );
#endif
}

#endif

uint32 popc_2bit(const uint64* page, const uint32 n, const uint32 mod, const uint32 c)
{
#if !defined(SSE_LOADS)
    // sum up all the pop-counts of the relevant masks
    uint32 out = 0u;

    for (uint32 j = 0; j < n; ++j)
        out += popc_2bit( page[j], c );

    out += popc_2bit( page[n], c, mod );
    return out;
#else
    const __m128i* page_mm = reinterpret_cast<const __m128i*>( page );

    uint32 out = 0u;

    if (c == 0)
    {
        // sum up all the pop-counts of the relevant masks
        for (uint32 j = 0; j < n/2; ++j)
            out += popc_2bit<0>( _mm_load_si128( page_mm + j ) );

        out += popc_2bit<0>( _mm_load_si128( page_mm + n/2 ), n & 1, mod );
    }
    else if (c == 1)
    {
        // sum up all the pop-counts of the relevant masks
        for (uint32 j = 0; j < n/2; ++j)
            out += popc_2bit<1>( _mm_load_si128( page_mm + j ) );

        out += popc_2bit<1>( _mm_load_si128( page_mm + n/2 ), n & 1, mod );
    }
    else if (c == 2)
    {
        // sum up all the pop-counts of the relevant masks
        for (uint32 j = 0; j < n/2; ++j)
            out += popc_2bit<2>( _mm_load_si128( page_mm + j ) );

        out += popc_2bit<2>( _mm_load_si128( page_mm + n/2 ), n & 1, mod );
    }
    else
    {
        // sum up all the pop-counts of the relevant masks
        for (uint32 j = 0; j < n/2; ++j)
            out += popc_2bit<3>( _mm_load_si128( page_mm + j ) );

        out += popc_2bit<3>( _mm_load_si128( page_mm + n/2 ), n & 1, mod );
    }
    return out;
#endif
}

void SparseSymbolSet::reserve(const uint64 n, const uint32 n_special)
{
    m_pos.reserve( n_special );
    m_new_pos.reserve( n_special );

    m_id.reserve( n_special );
    m_new_id.reserve( n_special );

    m_buckets.reserve( (n / BUCKET_SIZE) + 1u );
}

void SparseSymbolSet::set(const uint64 range, const uint32 n_special, const uint32* p, const uint32* id)
{
    nvbio::vector<host_tag,uint8> h_temp_storage;

    m_pos.resize( n_special );
    m_id.resize( n_special );

    thrust::transform(
        p,
        p + n_special,
        m_pos.begin(),
        cast_functor<uint32,uint64>() );

    thrust::copy(
        id,
        id + n_special,
        m_id.begin() );

    m_n_special = n_special;

    set_range( range );
}

void SparseSymbolSet::insert(
    const uint64    range,
    const uint32    n_block,
    const uint64*   g,
    const uint32    n_special,
    const uint32*   p_special,
    const uint64*   g_special,
    const uint32*   id_special)
{
    nvbio::vector<host_tag,uint8> h_temp_storage;

    // make room for the output dollars
    m_new_pos.resize( m_n_special + n_special );
    m_new_id.resize(  m_n_special + n_special );

    // for each ext dollar, find how many insertions preceed it
    #pragma omp parallel for
    for (int64 i = 0; i < int64( m_n_special ); ++i)
    {
        const uint32 n = upper_bound_index( m_pos[i], &g[0], n_block );
        m_pos[i] += n;
    }

    // do a parallel merge
    merge_by_key<host_tag>(
        m_n_special,
        n_special,
        m_pos.begin(),
        make_binary_transform_iterator( g_special, p_special, thrust::plus<uint64>() ),
        m_id.begin(),
        id_special,
        m_new_pos.begin(),
        m_new_id.begin(),
        h_temp_storage );

    m_pos.swap( m_new_pos );
    m_id.swap( m_new_id );

    m_n_special += n_special;

    set_range( range );
}

// extend range
//
void SparseSymbolSet::set_range(const uint64 n)
{
    m_n = n;

    // build the buckets
    build_buckets( m_n, m_n_special, &m_pos[0], BUCKET_SIZE, m_buckets, false );
}

} // namespace nvbio
