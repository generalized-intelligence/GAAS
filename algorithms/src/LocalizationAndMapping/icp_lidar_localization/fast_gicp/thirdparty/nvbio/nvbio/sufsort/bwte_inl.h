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

#include <cub/cub.cuh>
#include <mgpuhost.cuh>
#include <moderngpu.cuh>
#include <nvbio/sufsort/compression_sort.h>
#include <nvbio/sufsort/sufsort_priv.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/strings/suffix.h>
#include <thrust/merge.h>
#include <algorithm>

namespace nvbio {

inline
void BWTEBlock::reserve(const uint32 _max_block_strings, const uint32 _max_block_suffixes)
{
    priv::alloc_storage( h_dollar_off,  _max_block_strings );
    priv::alloc_storage( h_dollar_id,   _max_block_strings );
    priv::alloc_storage( h_dollar_pos,  _max_block_strings );

    priv::alloc_storage( h_SA,          _max_block_suffixes );
    priv::alloc_storage( h_BWT,         _max_block_suffixes );
    priv::alloc_storage( h_cum_lengths, _max_block_strings );

    // pin all the host memory used in device <-> host copies
    if (max_block_suffixes < _max_block_suffixes) cudaHostRegister( &h_SA[0],          _max_block_suffixes * sizeof(uint32), cudaHostRegisterPortable );
    if (max_block_strings  < _max_block_strings)  cudaHostRegister( &h_cum_lengths[0], _max_block_strings  * sizeof(uint32), cudaHostRegisterPortable );
    if (max_block_strings  < _max_block_strings)  cudaHostRegister( &h_dollar_off[0],  _max_block_strings  * sizeof(uint32), cudaHostRegisterPortable );
    if (max_block_strings  < _max_block_strings)  cudaHostRegister( &h_dollar_id[0],   _max_block_strings  * sizeof(uint32), cudaHostRegisterPortable );

  #if defined(QUICK_CHECK_REPORT) || defined(CHECK_SORTING)
    priv::alloc_storage( h_string_ids, _max_block_suffixes );
  #endif

    max_block_suffixes = _max_block_suffixes;
    max_block_strings  = _max_block_strings;
}

/// constructor
///
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
BWTEContext<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator>::BWTEContext(const int device) :
    mgpu_ctxt( mgpu::CreateCudaDevice( device ) ),
    string_sorter( mgpu_ctxt ),
    suffixes( mgpu_ctxt )
{
    load_time           = 0.0f;
    sort_time           = 0.0f;
    copy_time           = 0.0f;
    rank_time           = 0.0f;
    insert_time         = 0.0f;
    insert_dollars_time = 0.0f;

    n_strings_ext  = 0u;
    n_suffixes_ext = 0u;

    n_processed_strings  = 0u;
    n_processed_suffixes = 0u;
}

// needed device memory
//
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
uint64 BWTEContext<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator>::needed_device_memory(const uint32 _max_block_strings, const uint32 _max_block_suffixes) const
{
    const size_t d_bytes =
        string_sorter.needed_device_memory( _max_block_suffixes )
      + string_set_handler.needed_device_memory( max_block_suffixes )
      + suffixes.needed_device_memory( _max_block_strings, _max_block_suffixes )
      + chunk_loader.needed_device_memory( _max_block_strings, _max_block_suffixes )
      + _max_block_suffixes * sizeof(uint2)          // d_suffixes
      + _max_block_suffixes * sizeof(uint8)          // d_temp_storage
      + _max_block_suffixes * sizeof(uint8)          // d_BWT_block
      + _max_block_strings  * sizeof(uint32)         // d_dollar_off
      + _max_block_strings  * sizeof(uint32);        // d_dollar_id

    return d_bytes;
}

// reserve space for a maximum block size
//
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
void BWTEContext<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator>::reserve(const uint32 _max_block_strings, const uint32 _max_block_suffixes)
{
    max_block_suffixes = _max_block_suffixes;
    max_block_strings  = _max_block_strings;

    const size_t d_bytes =
        string_sorter.needed_device_memory( max_block_suffixes )
      + string_set_handler.needed_device_memory( max_block_suffixes )
      + suffixes.needed_device_memory( max_block_strings, max_block_suffixes )
      + chunk_loader.needed_device_memory( _max_block_strings, _max_block_suffixes )
      + max_block_suffixes * sizeof(uint2)          // d_suffixes
      + max_block_suffixes * sizeof(uint8)          // d_temp_storage
      + max_block_suffixes * sizeof(uint8)          // d_BWT_block
      + max_block_strings  * sizeof(uint32)         // d_dollar_off
      + max_block_strings  * sizeof(uint32);        // d_dollar_id

    log_verbose(stderr, "  allocating device sorting storage (%.1f GB)\n",
        float( d_bytes ) / float(1024*1024*1024) );

    priv::alloc_storage( d_suffixes,     max_block_suffixes );
    priv::alloc_storage( d_temp_storage, max_block_suffixes );

    priv::alloc_storage( d_BWT_block,  max_block_suffixes );

    priv::alloc_storage( d_dollar_off, max_block_strings );
    priv::alloc_storage( d_dollar_id,  max_block_strings );

    chunk_loader.reserve( max_block_strings, max_block_suffixes );

    suffixes.reserve( max_block_strings, max_block_suffixes );

    string_set_handler.reserve( max_block_suffixes, SORTING_SLICE_SIZE );
    string_sorter.reserve( max_block_suffixes );

    const size_t h_bytes =
        max_block_suffixes * sizeof(uint64) * 2 +   // g, g_sorted
        max_block_suffixes * sizeof(uint32)     +   // h_SA
        max_block_strings  * sizeof(uint32)     +   // h_cum_lengths
        max_block_suffixes * sizeof(uint8)      +   // h_BWT
        max_block_strings  * sizeof(uint32)     +   // h_dollar_off
        max_block_strings  * sizeof(uint32)     +   // h_dollar_id
        max_block_strings  * sizeof(uint64);        // h_dollar_pos

    log_verbose(stderr, "  allocating host sorting storage (%.1f GB)\n",
        float( h_bytes ) / float(1024*1024*1024) );

    priv::alloc_storage( g,             max_block_suffixes );
    priv::alloc_storage( g_sorted,      max_block_suffixes );

    block.reserve( max_block_strings, max_block_suffixes );
}

// append a new block of strings
//
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
void BWTEContext<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator>::append_block(
    const uint32                            block_begin,
    const uint32                            block_end,
    const string_set_type                   string_set,
        PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  BWT_ext,
        SparseSymbolSet&                    BWT_ext_dollars,
    const bool                              forward)
{
    sort_block( block_begin, block_end, string_set, block );

    merge_block(
        block_begin,
        block_end,
        string_set,
        block,
        BWT_ext,
        BWT_ext_dollars,
        forward );
}

// merge the given sorted block
//
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
void BWTEContext<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator>::merge_block(
    const uint32                        block_begin,
    const uint32                        block_end,
    const string_set_type               string_set,
    BWTEBlock&                          block,
    PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  BWT_ext,
    SparseSymbolSet&                    BWT_ext_dollars,
    const bool                          forward)
{
    n_suffixes_ext = BWT_ext.size();
    n_strings_ext  = BWT_ext_dollars.size();

    // merge the new BWT with the old one
    if (n_suffixes_ext)
    {
        //
        // Rank all the suffixes in the sorted block wrt BWT_ext.
        //

        rank_block(
            block_begin,
            block_end,
            string_set,
            block,
            BWT_ext,
            BWT_ext_dollars,
            forward );

        insert_block(
            block,
            BWT_ext,
            BWT_ext_dollars );
    }
    else
    {
        // store the BWT block in BWT_ext
        BWT_ext.resize( block.n_suffixes, &block.h_BWT[0] );

        // save the initial set of dollars
        BWT_ext_dollars.set(
            block.n_suffixes,
            block.n_strings,
            raw_pointer( block.h_dollar_off ),
            raw_pointer( block.h_dollar_id ) );

     #if defined(CHECK_COPY)
        log_visible(stderr, "  check copy...      ");
        uint64 occ[SYMBOL_COUNT] = { 0 };
        for (uint32 i = 0; i < block.n_suffixes; ++i)
        {
            if ((i % 1000) == 0)
                log_visible(stderr, "\r  check copy... %6.2f%%     ", 100.0f * float(i)/float(block.n_suffixes));

            const uint32 cb = block.h_BWT[i] & (SYMBOL_COUNT-1);
            const uint32 ce = BWT_ext[i];
            if (ce != cb)
            {
                log_error(stderr, "at %u, expected %u, got %u!\n", i, cb, ce);
                exit(1);
            }

            ++occ[ ce ];
            for (uint8 q = 0; q < SYMBOL_COUNT; ++q)
            {
                const uint64 r = BWT_ext.rank( i, q );
                if (r != occ[q])
                {
                    log_error(stderr, "ranks mismatch at c[%u], i[%llu]:p[%llu]: expected %llu, got %llu!\n",
                        uint32(q), i, occ[q], r );
                    exit(1);
                }
            }
        }
        log_visible(stderr, "\r  check copy...            \n");
     #endif

      #if defined(CHECK_SORTING)
        // build a suffix localizer
        const priv::localize_suffix_functor suffix_localizer(
            nvbio::raw_pointer( block.h_cum_lengths ),
            nvbio::raw_pointer( block.h_string_ids ),
            block_begin );

        for (uint32 i = 0; i < n_block_suffixes; ++i)
            TSA[i] = suffix_localizer( block.h_SA[i] );
      #endif
    }

    // update counters
    n_processed_strings  += block.n_strings;
    n_processed_suffixes += block.n_suffixes;
}

// sort the given device block
//
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
void BWTEContext<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator>::sort_block(
    const uint32            block_begin,
    const uint32            block_end,
    const string_set_type   string_set,
    BWTEBlock&              block)
{
    block.reserve( max_block_strings, max_block_suffixes );

    const uint32 n_block_strings = block_end - block_begin;

    uint32 n_block_suffixes = 0u;
    for (uint32 i = block_begin; i < block_end; ++i)
        n_block_suffixes += nvbio::length( string_set[i] ) + 1u;

    block.n_strings  = n_block_strings;
    block.n_suffixes = n_block_suffixes;

    // load this block
    log_debug(stderr, "  load device chunk (%u)\n", n_block_suffixes);

    Timer timer;
    timer.start();

    const chunk_set_type chunk_set = chunk_loader.load( string_set, block_begin, block_end );

    NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
    timer.stop();
    load_time += timer.seconds();

    // compute its SA and BWT
    log_debug(stderr, "  sort\n");
    {
        const uint32 SYMBOLS_PER_RADIXWORD = priv::symbols_per_word<SYMBOL_SIZE,RADIX_BITS,DOLLAR_BITS>();

        log_debug(stderr, "    set suffix flattener\n");
        {
            cuda::ScopedTimer<float> timer( &sort_time );

            // prepare the suffix flattener
            suffixes.set( chunk_set );

            // make sure we get the proper number of suffixes
            if (suffixes.n_suffixes != n_block_suffixes)
            {
                log_error(stderr, "  unexpected number of suffixes! (%u)\n", suffixes.n_suffixes);
                exit(1);
            }
        }

        // copy the suffix flattener to the host
        {
            cuda::ScopedTimer<float> timer( &copy_time );

            #if defined(HOST_STRING_IDS)
            thrust::copy(
                suffixes.string_ids.begin(),
                suffixes.string_ids.begin() + n_block_suffixes,
                block.h_string_ids.begin() );
            #endif

            thrust::copy(
                suffixes.cum_lengths.begin(),
                suffixes.cum_lengths.begin() + n_block_strings,
                block.h_cum_lengths.begin() );
        }

        log_debug(stderr, "    localize suffixes\n");
        {
            cuda::ScopedTimer<float> timer( &sort_time );

            // build a suffix localizer
            const priv::localize_suffix_functor suffix_localizer(
                nvbio::raw_pointer( suffixes.cum_lengths ),
                nvbio::raw_pointer( suffixes.string_ids ) );

            // build the list of suffixes
            nvbio::transform<device_tag>(
                n_block_suffixes,
                thrust::make_counting_iterator<uint32>(0),
                d_suffixes.begin(),
                suffix_localizer );
        }

        // reuse suffixes.string_ids to store the final indices
        thrust::device_ptr<uint32> d_indices( nvbio::raw_pointer( suffixes.string_ids ) );

        // temporary BWT storage
        thrust::device_ptr<uint8> d_unsorted_bwt = thrust::device_ptr<uint8>( raw_pointer( d_temp_storage ) );
        thrust::device_ptr<uint8> d_bwt          = thrust::device_ptr<uint8>( raw_pointer( d_BWT_block ) );
        
        log_debug(stderr, "    compression sort\n");
        {
            cuda::ScopedTimer<float> timer( &sort_time );

            // compute the maximum number of words needed to represent a suffix
            const uint32 n_words = util::divide_ri( suffixes.max_length( chunk_set ), SYMBOLS_PER_RADIXWORD );

            //
            // sort the suffixes, building the SA
            //

            // initialize the set radices
            string_set_handler.set( chunk_set );
            string_set_handler.init( n_block_suffixes, NULL, nvbio::raw_pointer( d_suffixes ) );

            cuda::DiscardDelayList delay_list;

            // sort the suffix strings!
            string_sorter.sort(
                string_set_handler,
                n_block_suffixes,
                n_words,
                thrust::make_counting_iterator<uint32>(0u),
                d_indices,
                uint32(-1),
                delay_list,
                SORTING_SLICE_SIZE );

            //
            // extract the BWT from the SA
            //

            // copy the unsorted symbols corresponding to each suffix
            nvbio::transform<device_tag>(
                n_block_suffixes,
                d_suffixes.begin(),
                d_unsorted_bwt,
                priv::string_set_bwt_functor<chunk_set_type>( chunk_set ) );

            // gather the symbols in sorted order
            thrust::gather(
                d_indices,
                d_indices + n_block_suffixes,
                d_unsorted_bwt,
                d_bwt );
        }

        // copy back results to the host
        log_debug(stderr, "    copy to host\n");
        {
            cuda::ScopedTimer<float> timer( &copy_time );

            // copy the SA
            thrust::copy(
                d_indices,
                d_indices + n_block_suffixes,
                block.h_SA.begin() );

            // copy the BWT
            thrust::copy(
                d_bwt,
                d_bwt + n_block_suffixes,
                block.h_BWT.begin() );

            // copy the dollar offsets and their string ids
            const uint32 n_dollars = nvbio::copy_flagged(
                n_block_suffixes,
                thrust::make_zip_iterator( thrust::make_tuple(
                    thrust::make_counting_iterator<uint32>(0),
                    thrust::make_transform_iterator( d_suffixes.begin(), priv::suffix_component_functor<priv::STRING_ID>() ) ) ),
                thrust::make_transform_iterator( d_bwt, equal_to_functor<uint8>( 255u ) ),
                thrust::make_zip_iterator( thrust::make_tuple(
                    d_dollar_off.begin(),
                    d_dollar_id.begin() ) ),
                d_temp_storage );

            if (n_dollars != n_block_strings)
            {
                log_error(stderr, "mismatching number of dollars! expected %u, got %u\n", n_block_strings, n_dollars);
                exit(1);
            }

            // copy the dollar offsets
            thrust::copy(
                d_dollar_off.begin(),
                d_dollar_off.begin() + n_block_strings,
                block.h_dollar_off.begin() );

            // copy the dollar ids
            thrust::copy(
                d_dollar_id.begin(),
                d_dollar_id.begin() + n_block_strings,
                block.h_dollar_id.begin() );
        }
    }

    log_verbose(stderr, "  sort   : %.1f M suffixes/s (sort: %.1f%%, copy: %.1f%%)\n",
        (1.0e-6f * (n_processed_suffixes + n_block_suffixes)) / (sort_time + copy_time),
        100.0f * sort_time / (sort_time + copy_time),
        100.0f * copy_time / (sort_time + copy_time));
}

// rank the block suffixes wrt BWT_ext
//
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
void BWTEContext<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator>::rank_block(
    const uint32                        block_begin,
    const uint32                        block_end,
    const string_set_type               string_set,
    const BWTEBlock&                    block,
    PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  BWT_ext,
    SparseSymbolSet&                    BWT_ext_dollars,
    const bool                          forward)
{
    typedef typename string_set_type::string_type   string_type;

    const uint32 n_block_strings   = block.n_strings;
    const uint32 n_block_suffixes  = block.n_suffixes;

    //
    // Rank all the suffixes in the sorted block wrt BWT_ext.
    // Currently, we just do a parallel loop through all the strings in the block,
    // and for each of them iterate through their suffixes starting from the shortest.
    //
    // An alternative would be to break the *sorted* suffixes in batches of equal
    // length (i.e. first all the sorted 1-suffixes, then all the sorted 2-suffixes, etc),
    // and do a series of parallel loops through them: this would ensure that the
    // ranking operations perform coherent memory accesses.
    //
    // How do we select all the j-suffixes in sorted order from the SA?
    // Is there any better way then doing a separate parallel select across all of them for each j?
    // In a way, it's a multi-select, where a single pass could essentially do N-scans at
    // the same time...
    //

    Timer timer;
    timer.start();

    // initialize the insertion array
    //#pragma omp parallel for
    //for (int i = 0; i < int( n_block_suffixes ); ++i)
    //    g[i] = 0u;

    log_debug(stderr, "  rank\n");
    log_debug(stderr, "  C:\n");
    uint64 C[SYMBOL_COUNT+1];
    {
        const uint64* freqs = BWT_ext.symbol_frequencies();
        C[0] = n_strings_ext; // number of dollars
        for (uint32 i = 1; i <= SYMBOL_COUNT; ++i)
        {
            C[i] = C[i-1] + freqs[i-1];

            // if this is the symbol used to represent dollars we have to remove them
            if ((255u % SYMBOL_COUNT) == i-1)
                C[i] -= n_strings_ext;

            log_debug(stderr, "    %llu\n", C[i] );
        }
    }

    const uint32 DOLLAR_SYMBOL = 255u & (SYMBOL_COUNT-1);

    // loop through all the strings in this block
    #pragma omp parallel for
    for (int32 q = 0; q < int32( n_block_strings ); ++q)
    {
        const string_type string = string_set[block_begin + q];

        const uint32 len = nvbio::length( string );

        // start with the dollar sign: C[$_q] + rank($_q,i) reduces to C[$_q] = #dollars(block) = (backwards ? 0 : block_begin)
        uint64 i = forward ? n_strings_ext : 0u;

        const uint32 suffix_off = q ? block.h_cum_lengths[ q-1 ] : 0u;

        g[suffix_off + len] = i;

        // loop backwards through all the suffixes of the current string
        for (int32 k = len-1u; k >= 0; --k)
        {
            const uint8 c = string[k];

            const uint64 i_dollars = (c == DOLLAR_SYMBOL) ?
                BWT_ext_dollars.rank(i-1) : 0u;

            // compute the number of external suffixes 'j' preceding this one
            const uint64 r = C[c] + BWT_ext.rank( i-1, c );
            const uint64 j = r - i_dollars;
            assert( i_dollars <= r );
            assert( j <= n_suffixes_ext );

            // save the insertion slot
            g[suffix_off + k] = j;

          #if 0
            if ((q == 243303  && k >= 56) ||
                (q == 2163961 && k >= 56))
            {
                if (k == 99) log_verbose(stderr, "\n");
                log_verbose(stderr, "  q[%8u:%2u] : LF[%9llu,%u] = %9llu (r[%9llu], $[%9llu]\n",
                    q, k, i, uint32(c), j, r, i_dollars,
                    i_dollars,
                    i_dollars );
            }
          #endif

            i = j;
        }
    }

    // reorder g based on SA
    #pragma omp parallel for
    for (int32 i = 0; i < int32( n_block_suffixes ); ++i)
        g_sorted[i] = g[block.h_SA[i]];

    timer.stop();
    rank_time += timer.seconds();
    log_verbose(stderr, "  rank   : %.1f M suffixes/s\n",
        (1.0e-6f * (n_processed_suffixes + n_block_suffixes)) / rank_time);

  #if defined(CHECK_SORTING)
    log_visible(stderr, "  check sorting...  ");

    // build a suffix localizer
    const priv::localize_suffix_functor suffix_localizer(
        nvbio::raw_pointer( block.h_cum_lengths ),
        nvbio::raw_pointer( block.h_string_ids ),
        block_begin );

    typedef Suffix<input_string_type,uint32> suffix_type;

    for (uint32 i = 0; i < n_block_suffixes; ++i)
    {
        if ((i % 1000) == 0)
            log_visible(stderr, "\r  check sorting... %6.2f%%     ", 100.0f * float(i)/float(n_block_suffixes));

        const uint32 sa = block.h_SA[i];

        const uint2 suffix_i = suffix_localizer( sa );

        if (g_sorted[i] < n_suffixes_ext)
        {
            const uint2 suffix_n = TSA[ g_sorted[i] ];

            // make sure s_i <= s_n
            const int32 cmp = compare_suffixes( string_set, suffix_i, suffix_n );

            if (cmp == 1)
            {
                log_error(stderr, "\ninsertion[%u -> %llu] : (%u,%u) > (%u,%u)\n",
                    i, g_sorted[i],
                    suffix_i.y, suffix_i.x,
                    suffix_n.y, suffix_n.x );

                const suffix_type string_i = make_suffix( string_set[suffix_i.y], suffix_i.x );
                const suffix_type string_n = make_suffix( string_set[suffix_n.y], suffix_n.x );

                log_error(stderr, "  I: \"");
                for (uint32 j = 0; j < nvbio::length( string_i ); ++j)
                {
                    const uint8 c_i = string_i[j];
                    log_error_cont(stderr, "%u", uint32(c_i) );
                }
                log_error_cont(stderr, "\"\n" );

                log_error(stderr, "  N: \"");
                for (uint32 j = 0; j < nvbio::length( string_n ); ++j)
                {
                    const uint8 c_n = string_n[j];
                    log_error_cont(stderr, "%u", uint32(c_n) );
                }
                log_error_cont(stderr, "\"\n" );

                exit(1);
            }
        }
        if (g_sorted[i] > 0)
        {
            const uint2 suffix_p = TSA[ g_sorted[i]-1 ];

            // make sure s_p < s_i
            const int32 cmp = compare_suffixes( string_set, suffix_i, suffix_p );

            if (cmp == -1)
            {
                log_error(stderr, "\ninsertion[%u-1 -> %llu] : (%u,%u) < (%u, %u)\n",
                    i,
                    g_sorted[i],
                    suffix_i.y, suffix_i.x,
                    suffix_p.y, suffix_p.x );

                const suffix_type string_i = make_suffix( string_set[suffix_i.y], suffix_i.x );
                const suffix_type string_p = make_suffix( string_set[suffix_p.y], suffix_p.x );

                log_error(stderr, "  I: \"");
                for (uint32 j = 0; j < nvbio::length( string_i ); ++j)
                {
                    const uint8 c_i = string_i[j];
                    log_error_cont(stderr, "%u", uint32(c_i) );
                }
                log_error_cont(stderr, "\"\n" );

                log_error(stderr, "  P: \"");
                for (uint32 j = 0; j < nvbio::length( string_p ); ++j)
                {
                    const uint8 c_p = string_p[j];
                    log_error_cont(stderr, "%u", uint32(c_p) );
                }
                log_error_cont(stderr, "\"\n" );

                exit(1);
            }
        }
    }
    log_visible(stderr, "\r  check sorting... done      \n");
    exit(1);
  #endif

  #if defined(QUICK_CHECK)
    log_verbose(stderr,  "  check sorting\n");
    // check that the suffix ranks are truly sorted
    if (is_sorted<host_tag>( n_block_suffixes, &g_sorted[0] ) == false)
    {
        log_error(stderr, "  BWTE: suffix ranks not in sorted order!\n");
      #if defined(QUICK_CHECK_REPORT)
        for (uint32 i = 0; i < n_block_suffixes-1; ++i)
        {
            if (g_sorted[i] > g_sorted[i+1])
            {
                log_error(stderr, "  g_s[%u->%u] = %llu > g_s[%u->%u] = %llu\n",
                    i,   block.h_SA[i],   g_sorted[i],
                    i+1, block.h_SA[i+1], g_sorted[i+1] );

                // build a suffix localizer
                const priv::localize_suffix_functor suffix_localizer(
                    nvbio::raw_pointer( block.h_cum_lengths ),
                    nvbio::raw_pointer( block.h_string_ids ) );

                const uint2 suffix1 = suffix_localizer( block.h_SA[i] );
                const uint2 suffix2 = suffix_localizer( block.h_SA[i+1] );


                log_error(stderr, "  (%02u,%8u): ", suffix1.x, suffix1.y);
                {
                    const string_type string = string_set[block_begin + suffix1.y];
                    for (uint32 j = suffix1.x; j < nvbio::length( string ); ++j)
                        log_error_cont(stderr, "%u", uint32( string[j] ) );
                    log_error_cont(stderr, "\n");
                }
                log_error(stderr, "  (%02u,%8u): ", suffix2.x, suffix2.y);
                {
                    const string_type string = string_set[block_begin + suffix2.y];
                    for (uint32 j = suffix2.x; j < nvbio::length( string ); ++j)
                        log_error_cont(stderr, "%u", uint32( string[j] ) );
                    log_error_cont(stderr, "\n");
                }
                break;
            }
        }
      #endif
        exit(1);
    }
  #endif
}

// insert the block
//
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
void BWTEContext<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator>::insert_block(
    BWTEBlock&                          block,
    PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  BWT_ext,
    SparseSymbolSet&                    BWT_ext_dollars)
{
    const uint32 n_block_strings   = block.n_strings;
    const uint32 n_block_suffixes  = block.n_suffixes;

  #if defined(CHECK_INSERTION)
    // make a copy of BWT_ext
    nvbio::vector<host_tag,uint8> BWT_ext_copy( n_suffixes_ext );
    #pragma omp parallel for
    for (int32 i = 0; i < int32( n_suffixes_ext ); ++i)
        BWT_ext_copy[i] = BWT_ext[i];
  #endif

    log_debug(stderr,  "  insert\n");
    {
        ScopedTimer<float> timer( &insert_time );

        // insert BWT[i] at g_sorted[i]
        BWT_ext.insert( n_block_suffixes, &g_sorted[0], &block.h_BWT[0] );
    }
    log_debug(stderr,  "  insert dollars\n");
    {
        ScopedTimer<float> timer( &insert_dollars_time );

        // build h_dollar_pos
        thrust::gather(
            block.h_dollar_off.begin(),
            block.h_dollar_off.begin() + n_block_strings,
            g_sorted.begin(),
            block.h_dollar_pos.begin() );

        // insert the block dollars in BWT_ext_dollars
        BWT_ext_dollars.insert(
            BWT_ext.size(),
            n_block_suffixes,
            raw_pointer( g_sorted ),
            n_block_strings,
            raw_pointer( block.h_dollar_off ),
            raw_pointer( block.h_dollar_pos ),
            raw_pointer( block.h_dollar_id ) );
    }
    log_verbose(stderr, "  insert : %.1f M suffixes/s (symbols: %.1f%%, dollars: %.1f%%)\n",
        (1.0e-6f * (n_processed_suffixes + n_block_suffixes)) / (insert_time + insert_dollars_time),
        100.0f * insert_time         / (insert_time + insert_dollars_time),
        100.0f * insert_dollars_time / (insert_time + insert_dollars_time));

#if defined(CHECK_INSERTION)
    log_visible(stderr,  "  check insertions");
    uint64 occ[SYMBOL_COUNT] = { 0 };
    {
        uint64 p = 0;
        uint64 o = 0;
        uint32 n_dollars = 0;
        for (uint32 j = 0; j < n_block_suffixes; ++j)
        {
            if ((j % 1000) == 0)
                log_visible(stderr, "\r  check insertions... %6.2f%%     ", 100.0f * float(j)/float(n_block_suffixes));

            const uint64 g_j = g_sorted[j];

            while (p < nvbio::min( g_j, n_suffixes_ext ))
            {
                if (BWT_ext[o] != BWT_ext_copy[p])
                {
                    log_error(stderr, "insertion mismatch at o[%llu]:p[%llu]: expected %u, got %u\n", o, p,
                        uint32( BWT_ext_copy[p] ), uint32( BWT_ext[o] ));
                    exit(1);
                }

                ++occ[ BWT_ext[o] ];
                for (uint8 q = 0; q < SYMBOL_COUNT; ++q)
                {
                    const uint64 r = BWT_ext.rank( o, q );
                    if (r != occ[q])
                    {
                        log_error(stderr, "ranks mismatch at c[%u], o[%llu]:p[%llu]: expected %llu, got %llu\n",
                            uint32(q), o, p, occ[q], r );
                        exit(1);
                    }
                }

                ++o;
                ++p;
            }

            const uint32 cc = block.h_BWT[j] % SYMBOL_COUNT;
            n_dollars += (block.h_BWT[j] == 255u) ? 1u : 0u;

            if (BWT_ext[o] != cc)
            {
                const uint32 ce = BWT_ext[o]; 
                log_error(stderr, "insertion mismatch at o[%llu]:j[%u]:g[%llu]: expected %u, got %u (page[%u:%llu:%p])\n",
                    o, j, g_j,
                    cc, ce, BWT_ext.find_page(o), BWT_ext.m_offsets[ BWT_ext.find_page(o) ], BWT_ext.get_page( BWT_ext.find_page(o) ));
                exit(1);
            }
            ++occ[ cc ];
            for (uint8 q = 0; q < SYMBOL_COUNT; ++q)
            {
                const uint64 r = BWT_ext.rank( o, q );
                if (r != occ[q])
                {
                    log_error(stderr, "ranks mismatch at c[%u], o[%llu]:j[%llu]: expected %llu, got %llu\n",
                        uint32(q), o, p, occ[q], r );
                    exit(1);
                }
            }
            ++o;
        }
        while (p < n_suffixes_ext)
        {
            if (BWT_ext[o] != BWT_ext_copy[p])
            {
                log_error(stderr, "insertion mismatch at o[%llu]:p[%llu]: expected %u, got %u\n", o, p,
                    uint32( BWT_ext_copy[p] ), uint32( BWT_ext[o] ));
                exit(1);
            }
            ++occ[ BWT_ext[o] ];
            for (uint8 q = 0; q < SYMBOL_COUNT; ++q)
            {
                const uint64 r = BWT_ext.rank( o, q );
                if (r != occ[q])
                {
                    log_error(stderr, "ranks mismatch at c[%u], o[%llu]:p[%llu]: expected %llu, got %llu\n",
                        uint32(q), o, p, occ[q], r );
                    exit(1);
                }
            }
            ++o;
            ++p;
        }
    }
    log_visible(stderr,  "\r  check insertions           \n");
#endif
}

///
/// Parallel BWTE algorithm for computing the BWT of a string-set.
///
/// \param string_set           the input set of strings
/// \param output               the output handler for the resulting BWT
/// \param params               the BWT construction parameters
///
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
void bwte(
    const ConcatenatedStringSet<
        PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,typename std::iterator_traits<offsets_iterator>::value_type>,
        offsets_iterator>                   string_set,
        PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  BWT_ext,
        SparseSymbolSet&                    BWT_ext_dollars,
        BWTParams*                          params)
{
    typedef PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,uint64>                  input_packed_stream_type;
    typedef ConcatenatedStringSet<input_packed_stream_type,uint64*>                         input_string_set_type;
    typedef typename ConcatenatedStringSet<input_packed_stream_type,uint64*>::string_type   input_string_type;

    // fetch the number of strings
    const uint32 N = string_set.size();

    // compute the size of the final BWT
    uint64 bwt_size = 0;
    for (uint32 i = 0; i < N; ++i)
    {
        const input_string_type string = string_set[i];
        const uint32 len = nvbio::length( string );

        bwt_size += len + 1u;
    }

    // alloc enough storage for the BWT vectors
    log_verbose(stderr, "  allocating host BWT storage (%.1f GB)\n",
        float( BWT_ext.needed_host_memory( bwt_size ) + N * sizeof(uint64) ) / float(1024*1024*1024) );

    BWT_ext.reserve( bwt_size );
    BWT_ext_dollars.reserve( bwt_size, N );

    // prepare a BWTEContext
    int current_device;
    cudaGetDevice( &current_device );
    BWTEContext<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator> bwte_context( current_device );

    const uint32 max_block_suffixes = 256*1024*1024;
    const uint32 max_block_strings  =  16*1024*1024;

    bwte_context.reserve( max_block_strings, max_block_suffixes );

    const bool forward = true;

    // BWT merging loop
    for (uint32 block_begin = forward ? 0 : N,
                block_end   = forward ? 0 : N;
         forward ? (block_begin < N) :
                   (block_end   > 0);
        )
    {
        uint32 n_block_suffixes = 0;

        if (forward)
        {
            // go forward finding block_end so as to collect a given number of suffixes
            for (uint32 i = block_begin; block_begin < N; ++i)
            {
                const input_string_type string = string_set[i];

                const uint32 string_len = nvbio::length( string );

                // check whether we need to stop growing the block
                if (n_block_suffixes + string_len + 1u > max_block_suffixes ||
                    i - block_begin                    > max_block_strings)
                    break;

                n_block_suffixes += string_len + 1u;
                block_end         = i+1;
            }
        }
        else
        {
            // go backwards finding block_begin so as to collect a given number of suffixes
            for (int32 i = block_end-1; i >= 0; --i)
            {
                const input_string_type string = string_set[i];

                const uint32 string_len = nvbio::length( string );

                // check whether we need to stop growing the block
                if (n_block_suffixes + string_len + 1u > max_block_suffixes ||
                    block_end - i                      > max_block_strings)
                    break;

                n_block_suffixes += string_len + 1u;
                block_begin       = i;
            }
        }

        log_info(stderr, "  block [%u, %u] (%u suffixes)\n", block_begin, block_end, n_block_suffixes);

        bwte_context.append_block(
            block_begin,
            block_end,
            string_set,
            BWT_ext,
            BWT_ext_dollars,
            forward );

        if (forward)
            block_begin = block_end;
        else
            block_end = block_begin;
    }
}

} // namespace nvbio
