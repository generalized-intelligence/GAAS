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

#include <nvbio/sufsort/sufsort_priv.h>
#include <nvbio/basic/primitives.h>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace nvbio {
namespace priv {

struct pack_flags_functor
{
    // constructor
    pack_flags_functor(
        const uint32    _n,
        const uint8*    _flags,
              uint32*   _comp_flags)
    :   n( _n ), flags( _flags ), comp_flags( _comp_flags ) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 thread_id) const
    {
        const uint32 idx = 32 * thread_id;
        if (idx >= n)
            return;

        // initialize the output word
        uint32 f = 0u;

        #pragma unroll
        for (uint32 i = 0; i < 2; ++i)
        {
            // fetch and process 16-bytes in one go
            const uint4 flag = ((const uint4*)flags)[idx/16u + i];

            if (flag.x & (255u <<  0)) f |= 1u << (i*16 + 0);
            if (flag.x & (255u <<  8)) f |= 1u << (i*16 + 1);
            if (flag.x & (255u << 16)) f |= 1u << (i*16 + 2);
            if (flag.x & (255u << 24)) f |= 1u << (i*16 + 3);
            if (flag.y & (255u <<  0)) f |= 1u << (i*16 + 4);
            if (flag.y & (255u <<  8)) f |= 1u << (i*16 + 5);
            if (flag.y & (255u << 16)) f |= 1u << (i*16 + 6);
            if (flag.y & (255u << 24)) f |= 1u << (i*16 + 7);
            if (flag.z & (255u <<  0)) f |= 1u << (i*16 + 8);
            if (flag.z & (255u <<  8)) f |= 1u << (i*16 + 9);
            if (flag.z & (255u << 16)) f |= 1u << (i*16 + 10);
            if (flag.z & (255u << 24)) f |= 1u << (i*16 + 11);
            if (flag.w & (255u <<  0)) f |= 1u << (i*16 + 12);
            if (flag.w & (255u <<  8)) f |= 1u << (i*16 + 13);
            if (flag.w & (255u << 16)) f |= 1u << (i*16 + 14);
            if (flag.w & (255u << 24)) f |= 1u << (i*16 + 15);
        }

        // write the output word
        comp_flags[thread_id] = f;
    }

    const uint32    n;
    const uint8*    flags;
          uint32*   comp_flags;
};

template <typename T>
struct build_head_flags_functor;

template <>
struct build_head_flags_functor<uint32>
{
    // constructor
    build_head_flags_functor(
        const uint32    _n,
        const uint32*   _keys,
              uint8*    _flags)
    :   n( _n ), keys( _keys ), flags( _flags ) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 thread_id) const
    {
        const uint32 idx = 4 * thread_id;
        if (idx >= n)
            return;

        // load the previous key
        const uint32 key_p = idx ? keys[idx-1] : 0xFFFFFFFF;

        // load the next 4 keys
        const uint4  key  = ((const uint4*)keys)[thread_id];
        const uchar4 flag = ((const uchar4*)flags)[thread_id];

        // and write the corresponding 4 flags
        ((uchar4*)flags)[thread_id] = make_uchar4(
            (key.x != key_p) ? 1u : flag.x,
            (key.y != key.x) ? 1u : flag.y,
            (key.z != key.y) ? 1u : flag.z,
            (key.w != key.z) ? 1u : flag.w );
    }

    const uint32    n;
    const uint32*   keys;
          uint8*    flags;
};

template <>
struct build_head_flags_functor<uint64>
{
    // constructor
    build_head_flags_functor(
        const uint32    _n,
        const uint64*   _keys,
              uint8*    _flags)
    :   n( _n ), keys( _keys ), flags( _flags ) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint32 thread_id) const
    {
        const uint32 idx = thread_id;
        if (idx >= n)
            return;

        // load the previous key
        const uint64 key_p = idx ? keys[idx-1] : 0xFFFFFFFF;

        // load the next key
        const uint64 key = keys[thread_id];
        const uint8 flag = flags[thread_id];

        // and write the corresponding flag out
        flags[thread_id] = (key != key_p) ? 1u : flag;
    }

    const uint32    n;
    const uint64*   keys;
          uint8*    flags;
};

// pack a set of head flags into a bit-packed array
//
void pack_flags(
    const uint32            n,
    const uint8*            flags,
          uint32*           comp_flags)
{
    const uint32 n_words = util::divide_ri( n, 32u );

    // use a for_each to automate support for older compute capabilities with limited grid sizes
    nvbio::for_each<device_tag>(
        n_words,
        thrust::make_counting_iterator<uint32>(0u),
        pack_flags_functor( n, flags, comp_flags ) );
}

// build a set of head flags looking at adjacent keys
//
void build_head_flags(
    const uint32            n,
    const uint32*           keys,
          uint8*            flags)
{
    const uint32 n_quads = util::divide_ri( n, 4u );

    // use a for_each to automate support for older compute capabilities with limited grid sizes
    nvbio::for_each<device_tag>(
        n_quads,
        thrust::make_counting_iterator<uint32>(0u),
        build_head_flags_functor<uint32>( n, keys, flags ) );
}

// build a set of head flags looking at adjacent keys
//
void build_head_flags(
    const uint32            n,
    const uint64*           keys,
          uint8*            flags)
{
    // use a for_each to automate support older compute capabilities with limited grid sizes
    nvbio::for_each<device_tag>(
        n,
        thrust::make_counting_iterator<uint32>(0u),
        build_head_flags_functor<uint64>( n, keys, flags ) );
}

uint32 extract_radix_16(
    const priv::string_set_2bit_be& string_set,
    const uint2                  suffix,
    const uint32                 word_idx)
{
    priv::local_set_suffix_word_functor<2u,16u,4u,priv::string_set_2bit_be,uint32> word_functor( string_set, word_idx );
    return word_functor( suffix );
}

uint32 extract_radix_32(
    const priv::string_set_2bit_be& string_set,
    const uint2                  suffix,
    const uint32                 word_idx)
{
    priv::local_set_suffix_word_functor<2u,32u,4u,priv::string_set_2bit_be,uint32> word_functor( string_set, word_idx );
    return word_functor( suffix );
}

uint32 extract_radix_64(
    const priv::string_set_2bit_u64_be& string_set,
    const uint2                         suffix,
    const uint32                        word_idx)
{
    priv::local_set_suffix_word_functor<2u,64u,5u,priv::string_set_2bit_u64_be,uint32> word_functor( string_set, word_idx );
    return word_functor( suffix );
}

void extract_radices_16(
    const priv::string_set_2bit_be  string_set,
    const uint32                    n_suffixes,
    const uint32                    word_begin,
    const uint32                    word_end,
    const uint2*                    h_suffixes,
          uint32*                   h_radices,
          uint8*                    h_symbols)
{
    if (word_begin+1 == word_end)
    {
        #pragma omp parallel for
        for (int32 i = 0; i < int32( n_suffixes ); ++i)
        {
            const uint2 suffix = h_suffixes[ i ];
            h_radices[i] = extract_radix_16( string_set, suffix, word_begin );
        }
    }
    else
    {
        #pragma omp parallel for
        for (int32 i = 0; i < int32( n_suffixes ); ++i)
        {
            const uint2 local_suffix_idx = h_suffixes[ i ];

            const uint32 string_idx = local_suffix_idx.y;
            const uint32 suffix_idx = local_suffix_idx.x;

            const uint64 string_off = string_set.offsets()[ string_idx ];
            const uint64 string_end = string_set.offsets()[ string_idx+1u ];
            const uint64 string_len = uint32( string_end - string_off );

            const uint32* base_words = string_set.base_string().stream();

            if (h_symbols != NULL)
                h_symbols[i] = suffix_idx ? string_set.base_string()[ string_off + suffix_idx-1u ] : 255u;

            extract_word_packed<16u,4u,2u>(
                base_words,
                string_len,
                string_off,
                suffix_idx,
                word_begin,
                word_end,
                strided_iterator<uint32*>( h_radices + i, n_suffixes ) );
        }
    }
}
void extract_radices_32(
    const priv::string_set_2bit_be  string_set,
    const uint32                    n_suffixes,
    const uint32                    word_begin,
    const uint32                    word_end,
    const uint2*                    h_suffixes,
          uint32*                   h_radices,
          uint8*                    h_symbols)
{
    if (word_begin+1 == word_end)
    {
        #pragma omp parallel for
        for (int32 i = 0; i < int32( n_suffixes ); ++i)
        {
            const uint2 suffix = h_suffixes[ i ];
            h_radices[i] = extract_radix_32( string_set, suffix, word_begin );
        }
    }
    else
    {
        #pragma omp parallel for
        for (int32 i = 0; i < int32( n_suffixes ); ++i)
        {
            const uint2 local_suffix_idx = h_suffixes[ i ];

            const uint32 string_idx = local_suffix_idx.y;
            const uint32 suffix_idx = local_suffix_idx.x;

            const uint64 string_off = string_set.offsets()[ string_idx ];
            const uint64 string_end = string_set.offsets()[ string_idx+1u ];
            const uint64 string_len = uint32( string_end - string_off );

            const uint32* base_words = string_set.base_string().stream();

            if (h_symbols != NULL)
                h_symbols[i] = suffix_idx ? string_set.base_string()[ string_off + suffix_idx-1u ] : 255u;

            extract_word_packed<32u,4u,2u>(
                base_words,
                string_len,
                string_off,
                suffix_idx,
                word_begin,
                word_end,
                strided_iterator<uint32*>( h_radices + i, n_suffixes ) );
        }
    }
}

void extract_radices_64(
    const priv::string_set_2bit_u64_be  string_set,
    const uint32                        n_suffixes,
    const uint32                        word_begin,
    const uint32                        word_end,
    const uint2*                        h_suffixes,
          uint64*                       h_radices,
          uint8*                        h_symbols)
{
    if (word_begin+1 == word_end)
    {
        #pragma omp parallel for
        for (int32 i = 0; i < int32( n_suffixes ); ++i)
        {
            const uint2 suffix = h_suffixes[ i ];
            h_radices[i] = extract_radix_64( string_set, suffix, word_begin );
        }
    }
    else
    {
        #pragma omp parallel for
        for (int32 i = 0; i < int32( n_suffixes ); ++i)
        {
            const uint2 local_suffix_idx = h_suffixes[ i ];

            const uint32 string_idx = local_suffix_idx.y;
            const uint32 suffix_idx = local_suffix_idx.x;

            const uint64 string_off = string_set.offsets()[ string_idx ];
            const uint64 string_end = string_set.offsets()[ string_idx+1u ];
            const uint64 string_len = uint32( string_end - string_off );

            const uint64* base_words = string_set.base_string().stream();

            if (h_symbols != NULL)
                h_symbols[i] = suffix_idx ? string_set.base_string()[ string_off + suffix_idx - 1u ] : 255u;

            extract_word_packed<64u,5u,2u>(
                base_words,
                string_len,
                string_off,
                suffix_idx,
                word_begin,
                word_end,
                strided_iterator<uint64*>( h_radices + i, n_suffixes ) );
        }
    }
}

void extract_radices(
    const priv::string_set_2bit_be  string_set,
    const uint32                    n_suffixes,
    const uint32                    word_begin,
    const uint32                    word_end,
    const uint32                    word_bits,
    const uint2*                    h_suffixes,
          uint32*                   h_radices,
          uint8*                    h_symbols)
{
    if (word_bits == 16)
    {
        extract_radices_16(
            string_set,
            n_suffixes,
            word_begin,
            word_end,
            h_suffixes,
            h_radices,
            h_symbols );
    }
    else if (word_bits == 32)
    {
        extract_radices_32(
            string_set,
            n_suffixes,
            word_begin,
            word_end,
            h_suffixes,
            h_radices,
            h_symbols );
    }
    else
    {
        log_error(stderr,"extract_radices(): unsupported number of bits\n");
        exit(1);
    }
}

void extract_radices(
    const priv::string_set_2bit_u64_be  string_set,
    const uint32                        n_suffixes,
    const uint32                        word_begin,
    const uint32                        word_end,
    const uint32                        word_bits,
    const uint2*                        h_suffixes,
          uint64*                       h_radices,
          uint8*                        h_symbols)
{
    if (word_bits == 64)
    {
        extract_radices_64(
            string_set,
            n_suffixes,
            word_begin,
            word_end,
            h_suffixes,
            h_radices,
            h_symbols );
    }
    else
    {
        log_error(stderr,"extract_radices(): unsupported number of bits\n");
        exit(1);
    }
}

/// process a batch of BWT symbols
///
uint32 DollarExtractor::extract(
    const uint32  n_suffixes,
    const uint8*  h_bwt,
    const uint8*  d_bwt,
    const uint2*  h_suffixes,
    const uint2*  d_suffixes,
    const uint32* d_indices)
{
    if (h_suffixes != NULL &&   // these are NULL for the empty suffixes
        d_suffixes != NULL)
    {
        priv::alloc_storage( d_dollar_ranks,   n_suffixes );
        priv::alloc_storage( d_dollars,        n_suffixes );
        priv::alloc_storage( h_dollar_ranks,   n_suffixes );
        priv::alloc_storage( h_dollars,        n_suffixes );

        uint32 n_found_dollars = 0;

        if (d_indices != NULL)
        {
            priv::alloc_storage( d_dollar_indices, n_suffixes );

            // find the dollar signs
            n_found_dollars = cuda::copy_flagged(
                n_suffixes,
                thrust::make_zip_iterator(
                    thrust::make_tuple(
                        thrust::make_counting_iterator<uint64>(0) + offset,
                        thrust::device_ptr<const uint32>( d_indices ) ) ),
                thrust::make_transform_iterator( thrust::device_ptr<const uint8>( d_bwt ), equal_to_functor<uint8>(255u) ),
                thrust::make_zip_iterator(
                    thrust::make_tuple(
                        d_dollar_ranks.begin(),
                        d_dollar_indices.begin() ) ),
                d_temp_storage );

            // gather their indices
            thrust::gather(
                d_dollar_indices.begin(),
                d_dollar_indices.begin() + n_found_dollars,
                thrust::make_transform_iterator( thrust::device_ptr<const uint2>( d_suffixes ), priv::suffix_component_functor<priv::STRING_ID>() ),
                d_dollars.begin() );
        }
        else
        {
            // find the dollar signs
            n_found_dollars = cuda::copy_flagged(
                n_suffixes,
                thrust::make_zip_iterator(
                    thrust::make_tuple(
                        thrust::make_counting_iterator<uint64>(0) + offset,
                        thrust::make_transform_iterator( thrust::device_ptr<const uint2>( d_suffixes ), priv::suffix_component_functor<priv::STRING_ID>() ) ) ),
                thrust::make_transform_iterator( thrust::device_ptr<const uint8>( d_bwt ), equal_to_functor<uint8>(255u) ),
                thrust::make_zip_iterator(
                    thrust::make_tuple(
                        d_dollar_ranks.begin(),
                        d_dollars.begin() ) ),
                d_temp_storage );
        }

        // and copy them back to the host
        thrust::copy(
            d_dollar_ranks.begin(),
            d_dollar_ranks.begin() + n_found_dollars,
            h_dollar_ranks.begin() );

        // and copy them back to the host
        thrust::copy(
            d_dollars.begin(),
            d_dollars.begin() + n_found_dollars,
            h_dollars.begin() );

        offset    += n_suffixes;
        n_dollars += n_found_dollars;
        return n_found_dollars;
    }
    else
    {
        offset += n_suffixes;
        return 0;
    }
}

} // namespace priv
} // namespace nvbio
