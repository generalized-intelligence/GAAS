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
#include <nvbio/sufsort/sufsort_priv.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/cuda/timer.h>
#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/cuda/primitives.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/adjacent_difference.h>
#include <nvbio/basic/omp.h>

namespace nvbio {
namespace priv {

/// A context class to perform suffix bucketing for all suffixes of a string-set
///
///\tparam SYMBOL_SIZE          the size of the symbols, in bits
///\tparam N_BITS               the number of bits used for bucketing
///\tparam DOLLAR_BITS          the number of bits used for encoding whether and where a dollar occurrs in the first N_BITS
///\tparam bucket_type          the word type used to encode buckets, i.e the first N_BITS of each suffix
///
template <uint32 SYMBOL_SIZE, uint32 N_BITS, uint32 DOLLAR_BITS, typename bucket_type = uint32>
struct DeviceCoreSetSuffixBucketer
{
    /// constructor
    ///
    DeviceCoreSetSuffixBucketer(mgpu::ContextPtr _mgpu) :
        suffixes( _mgpu ),
        d_setup_time(0.0f),
        d_flatten_time(0.0f),
        d_count_sort_time(0.0f),
        d_collect_sort_time(0.0f),
        d_filter_time(0.0f),
        d_remap_time(0.0f),
        d_max_time(0.0f),
        d_search_time(0.0f),
        d_copy_time(0.0f),
        m_mgpu( _mgpu ) {}

    /// clear internal timers
    ///
    void clear_timers()
    {
        suffixes.clear_timers();

        d_setup_time        = 0.0f;
        d_flatten_time      = 0.0f;
        d_count_sort_time   = 0.0f;
        d_collect_sort_time = 0.0f;
        d_filter_time       = 0.0f;
        d_remap_time        = 0.0f;
        d_max_time          = 0.0f;
        d_copy_time         = 0.0f;
    }

    /// count the number of suffixes falling in each bucket, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    template <typename string_set_type>
    uint64 count(const string_set_type& string_set)
    {
        cuda::Timer timer;

        // initialize the flattener
        suffixes.set( string_set, false ); // skip empty suffixes, whose position is trivial

        const uint32 n_suffixes = suffixes.n_suffixes;
        const uint32 n_buckets  = 1u << N_BITS;

        // initialize the temporary and output vectors
        alloc_storage( d_indices, n_suffixes * 2u );
        alloc_storage( d_radices, n_suffixes * 2u );
        alloc_storage( d_buckets, n_buckets );

        timer.start();

        // extract the first radix word from each of the suffixes
        suffixes.flatten(
            string_set,
            0u,                             // load the first word
            Bits<N_BITS, DOLLAR_BITS>(),    // number of bits per word
            thrust::make_counting_iterator<uint32>(0u),
            d_radices.begin() );

        timer.stop();
        d_flatten_time += timer.seconds();

        timer.start();

        // sort the radices so as to make binning easy
        cuda::SortBuffers<bucket_type*> sort_buffers;
        cuda::SortEnactor               sort_enactor;

        sort_buffers.selector = 0;
        sort_buffers.keys[0]  = nvbio::device_view( d_radices );
        sort_buffers.keys[1]  = nvbio::device_view( d_radices ) + n_suffixes;
        sort_enactor.sort( n_suffixes, sort_buffers, 0u, N_BITS );

        timer.stop();
        d_count_sort_time += timer.seconds();

        // initialize the bucket counters
        thrust::fill( d_buckets.begin(), d_buckets.end(), bucket_type(0u) );

        // compute the number of effectively used buckets looking at the last non-empty one
        const uint32 n_used_buckets = d_radices[ sort_buffers.selector * n_suffixes + n_suffixes-1 ] + 1u;

        timer.start();

        // find the end of each bin of values
        mgpu::SortedSearch<mgpu::MgpuBoundsUpper>(
            thrust::make_counting_iterator<uint32>(0u),
            n_used_buckets,
            d_radices.begin() + sort_buffers.selector * n_suffixes,
            n_suffixes,
            d_buckets.begin(),
            *m_mgpu );

        // compute the histogram by taking differences of the cumulative histogram
        thrust::adjacent_difference(
            d_buckets.begin(), d_buckets.begin() + n_used_buckets,
            d_buckets.begin());

        timer.stop();
        d_search_time += timer.seconds();

        return suffixes.n_suffixes;
    }

    /// collect the suffixes falling in a given set of buckets, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    template <typename string_set_type, typename bucketmap_iterator>
    uint32 collect(
        const string_set_type&      string_set,
        const uint32                bucket_begin,
        const uint32                bucket_end,
        const uint32                string_offset,
        const bucketmap_iterator    bucketmap)
    {
        cuda::Timer timer;

        timer.start();

        // initialize the flattener
        suffixes.set( string_set, false ); // skip empty suffixes, whose position is trivial

        timer.stop();
        d_setup_time += timer.seconds();

        const uint32 n_suffixes = suffixes.n_suffixes;
        const uint32 n_buckets  = 1u << N_BITS;

        // initialize the temporary and output vectors
        alloc_storage( d_indices, n_suffixes * 2u );
        alloc_storage( d_radices, n_suffixes * 2u );
        alloc_storage( d_buckets, n_buckets );

        timer.start();

        // extract the first radix word from each of the suffixes
        suffixes.flatten(
            string_set,
            0u,                             // load the first word
            Bits<N_BITS, DOLLAR_BITS>(),    // number of bits per word
            thrust::make_counting_iterator<uint32>(0u),
            d_radices.begin() );

        timer.stop();
        d_flatten_time += timer.seconds();

        timer.start();

        // determine if a radix is in the given bucket range
        const priv::in_range_functor in_range = priv::in_range_functor( bucket_begin, bucket_end );

        // retain only suffixes whose radix is between the specified buckets
        n_collected = cuda::copy_flagged(
            n_suffixes,
            thrust::make_zip_iterator( thrust::make_tuple( thrust::make_counting_iterator<uint32>(0u), d_radices.begin() ) ),
            thrust::make_transform_iterator( d_radices.begin(), in_range ),
            thrust::make_zip_iterator( thrust::make_tuple( d_indices.begin(), d_radices.begin() ) ) + n_suffixes,
            suffixes.temp_storage );

        timer.stop();
        d_filter_time += timer.seconds();

        timer.start();

        // remap the collected radices
        thrust::gather(
            d_radices.begin() + n_suffixes,
            d_radices.begin() + n_suffixes + n_collected,
            bucketmap,
            d_radices.begin() + n_suffixes );

        timer.stop();
        d_remap_time += timer.seconds();

        timer.start();

        // compute the maximum suffix length inside the range
        max_suffix_len = suffixes.max_length(
            string_set,
            d_indices.begin() + n_suffixes,
            d_indices.begin() + n_suffixes + n_collected );

        timer.stop();
        d_max_time += timer.seconds();

        timer.start();

        // use a device-staging buffer to localize the indices
        const uint32 buffer_stride = util::round_i( n_collected, 32u );
        alloc_storage( d_output, buffer_stride * 2 );

        localize_suffix_functor localizer(
                nvbio::device_view( suffixes.cum_lengths ),
                nvbio::device_view( suffixes.string_ids ),
                string_offset );

        thrust::transform(
            d_indices.begin() + n_suffixes,
            d_indices.begin() + n_suffixes + n_collected,
            d_output.begin() + buffer_stride,
            localizer );

        timer.stop();
        d_copy_time += timer.seconds();

        timer.start();

        // sort the radices so as to make binning easy
        cuda::SortBuffers<bucket_type*,uint64*> sort_buffers;
        cuda::SortEnactor                       sort_enactor;

        sort_buffers.selector  = 0;
      //#define SORT_BY_BUCKETS
      #if defined(SORT_BY_BUCKETS)
        sort_buffers.keys[0]   = nvbio::device_view( d_radices ) + n_suffixes;
        sort_buffers.keys[1]   = nvbio::device_view( d_radices );
        sort_buffers.values[0] = (uint64*)nvbio::device_view( d_output ) + buffer_stride;
        sort_buffers.values[1] = (uint64*)nvbio::device_view( d_output );
        sort_enactor.sort( n_collected, sort_buffers, 0u, N_BITS );
      #endif

        timer.stop();
        d_collect_sort_time += timer.seconds();

        //
        // copy all the indices inside the range to the output
        //

        alloc_storage( h_suffixes, n_suffixes );
        alloc_storage( h_radices,  n_suffixes );

        timer.start();

        // the buffer selector had inverted semantics
        sort_buffers.selector = 1 - sort_buffers.selector;

        // and copy everything to the output
        thrust::copy(
            d_output.begin() + sort_buffers.selector * buffer_stride,
            d_output.begin() + sort_buffers.selector * buffer_stride + n_collected,
            h_suffixes.begin() );

        // and copy everything to the output
        thrust::copy(
            d_radices.begin() + sort_buffers.selector * n_suffixes,
            d_radices.begin() + sort_buffers.selector * n_suffixes + n_collected,
            h_radices.begin() );

        timer.stop();
        d_copy_time += timer.seconds();

        return n_collected;
    }

    /// return the amount of used device memory
    ///
    uint64 allocated_device_memory() const
    {
        return
            suffixes.allocated_device_memory() +
            d_indices.size() * sizeof(uint32) +
            d_radices.size() * sizeof(bucket_type) +
            d_buckets.size() * sizeof(uint32) +
            d_output.size()  * sizeof(uint32);
    }

    /// return the amount of used device memory
    ///
    uint64 allocated_host_memory() const
    {
        return
            //suffixes.allocated_host_memory() +
            h_radices.size()  * sizeof(bucket_type) +
            h_suffixes.size() * sizeof(uint2);
    }

public:
    SetSuffixFlattener<SYMBOL_SIZE>     suffixes;
    thrust::device_vector<uint32>       d_indices;
    thrust::device_vector<bucket_type>  d_radices;
    thrust::device_vector<uint32>       d_buckets;
    thrust::device_vector<uint2>        d_output;
    thrust::host_vector<bucket_type>    h_radices;
    thrust::host_vector<uint2>          h_suffixes;
    uint32                              n_collected;
    uint32                              max_suffix_len;

    float                               d_setup_time;
    float                               d_flatten_time;
    float                               d_count_sort_time;
    float                               d_collect_sort_time;
    float                               d_filter_time;
    float                               d_remap_time;
    float                               d_max_time;
    float                               d_search_time;
    float                               d_copy_time;
    mgpu::ContextPtr                    m_mgpu;
};

/// A context class to perform suffix bucketing for all suffixes of a string-set
///
///\tparam SYMBOL_SIZE          the size of the symbols, in bits
///\tparam BIG_ENDIAN           the endianness of the symbols in each word of the packed string set
///\tparam storage_type         the storage iterator type of the input packed string
///\tparam N_BITS               the number of bits used for bucketing
///\tparam DOLLAR_BITS          the number of bits used for encoding whether and where a dollar occurrs in the first N_BITS
///\tparam bucket_type          the word type used to encode buckets, i.e the first N_BITS of each suffix
///\tparam system_tag           the system tag identifying the memory-space where the input string-set resides in
///
template <
    uint32   SYMBOL_SIZE,
    bool     BIG_ENDIAN,
    typename storage_type,
    uint32   N_BITS,
    uint32   DOLLAR_BITS,
    typename bucket_type,
    typename system_tag>
struct DeviceSetSuffixBucketer
{
    typedef priv::ChunkLoader<SYMBOL_SIZE,BIG_ENDIAN,storage_type,uint64*,system_tag,device_tag> chunk_loader_type;
    typedef typename chunk_loader_type::chunk_set_type                                           chunk_set_type;

    /// constructor
    ///
    DeviceSetSuffixBucketer(mgpu::ContextPtr _mgpu) : m_bucketer( _mgpu ) {}

    /// clear internal timers
    ///
    void clear_timers()
    {
        count_time   = 0.0f;
        load_time    = 0.0f;
        merge_time   = 0.0f;

        collect_time = 0.0f;
        bin_time     = 0.0f;
    }

    /// count the number of suffixes falling in each bucket, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    template <typename string_set_type>
    uint64 count(
        const string_set_type&             string_set,
              thrust::host_vector<uint32>& h_buckets)
    {
        Timer count_timer;
        count_timer.start();

        const uint32 chunk_size = 128*1024;
        const uint32 n_strings  = string_set.size();

        uint64 n_suffixes = 0u;

        const uint32 n_buckets = 1u << N_BITS;

        // initialize temporary vectors
        alloc_storage( d_buckets, n_buckets );

        // initialize the bucket counters
        thrust::fill( d_buckets.begin(), d_buckets.end(), bucket_type(0u) );

        // declare a chunk loader
        chunk_loader_type chunk_loader;

        // split the string-set in batches
        for (uint32 chunk_begin = 0; chunk_begin < n_strings; chunk_begin += chunk_size)
        {
            // determine the end of this batch
            const uint32 chunk_end = nvbio::min( chunk_begin + chunk_size, n_strings );

            Timer timer;
            timer.start();

            const chunk_set_type chunk_set = chunk_loader.load( string_set, chunk_begin, chunk_end );

            NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
            timer.stop();
            load_time += timer.seconds();

            n_suffixes += m_bucketer.count( chunk_set );

            timer.start();

            // and merge them in with the global buckets
            thrust::transform(
                m_bucketer.d_buckets.begin(),
                m_bucketer.d_buckets.end(),
                d_buckets.begin(),
                d_buckets.begin(),
                thrust::plus<uint32>() );

            NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
            timer.stop();
            merge_time += timer.seconds();
        }

        // copy the results
        thrust::copy( d_buckets.begin(), d_buckets.end(), h_buckets.begin() );

        //
        // initialize internal bucketing helpers
        //

        m_global_offset = 0u;

        // scan the bucket offsets so as to have global positions
        alloc_storage( h_bucket_offsets, n_buckets );
        thrust::exclusive_scan(
            thrust::make_transform_iterator( h_buckets.begin(), cast_functor<uint32,uint64>() ),
            thrust::make_transform_iterator( h_buckets.end(),   cast_functor<uint32,uint64>() ),
            h_bucket_offsets.begin() );

        count_timer.stop();
        count_time += count_timer.seconds();

        return n_suffixes;
    }

    /// collect the suffixes falling in a given set of buckets, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    template <typename string_set_type>
    uint64 collect(
        const string_set_type&              string_set,
        const uint32                        bucket_begin,
        const uint32                        bucket_end,
              uint32&                       max_suffix_len,
        const thrust::host_vector<uint32>&  h_subbuckets,
        thrust::host_vector<uint2>&         h_suffixes)
    {
        const uint32 chunk_size = 128*1024;
        const uint32 n_strings  = string_set.size();

        // copy the sub-buckets on the device
        alloc_storage( d_subbuckets, uint32( h_subbuckets.size() ) );
        thrust::copy( h_subbuckets.begin(), h_subbuckets.end(), d_subbuckets.begin() );

        // declare a chunk loader
        chunk_loader_type chunk_loader;

        uint64 n_collected = 0u;

        // split the string-set in batches
        for (uint32 chunk_begin = 0; chunk_begin < n_strings; chunk_begin += chunk_size)
        {
            // determine the end of this batch
            const uint32 chunk_end = nvbio::min( chunk_begin + chunk_size, n_strings );

            Timer timer;
            timer.start();

            const chunk_set_type chunk_set = chunk_loader.load( string_set, chunk_begin, chunk_end );

            NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
            timer.stop();
            load_time += timer.seconds();

            timer.start();

            m_bucketer.collect(
                chunk_set,
                bucket_begin,
                bucket_end,
                chunk_begin,
                d_subbuckets.begin() );

            timer.stop();
            collect_time += timer.seconds();

            // merge the collected suffixes
            {
                max_suffix_len = nvbio::max( max_suffix_len, m_bucketer.max_suffix_len );

                // check whether we are overflowing our output buffer
                if (n_collected + m_bucketer.n_collected > h_suffixes.size())
                {
                    log_error(stderr,"buffer size exceeded! (%llu/%llu)\n", n_collected + m_bucketer.n_collected, uint64( h_suffixes.size() ));
                    exit(1);
                }

                Timer timer;
                timer.start();

                // dispatch each suffix to their respective bucket
                for (uint32 i = 0; i < m_bucketer.n_collected; ++i)
                {
                    const uint2  loc    = m_bucketer.h_suffixes[i];
                    const uint32 bucket = m_bucketer.h_radices[i];
                    const uint64 slot   = h_bucket_offsets[bucket]++; // this could be done in parallel using atomics

                    NVBIO_CUDA_DEBUG_ASSERT(
                        slot >= m_global_offset,
                        slot <  m_global_offset + h_suffixes.size(),
                        "[%u] = (%u,%u) placed at %llu - %llu (%u)\n", i, loc.x, loc.y, slot, m_global_offset, bucket );

                    h_suffixes[ slot - m_global_offset ] = loc;
                }

                timer.stop();
                bin_time += timer.seconds();
            }

            n_collected += m_bucketer.n_collected;
        }

        // keep track of how many suffixes we have collected globally
        m_global_offset += n_collected;

        return n_collected;
    }

    /// return the amount of used device memory
    ///
    uint64 allocated_device_memory() const { return m_bucketer.allocated_device_memory(); }

    /// return the amount of used device memory
    ///
    uint64 allocated_host_memory() const { return m_bucketer.allocated_host_memory(); }

    /// print some stats
    ///
    void log_count_stats()
    {
        log_verbose(stderr,"    counting : %.1fs\n", count_time );
        log_verbose(stderr,"      load   : %.1fs\n", load_time);
        log_verbose(stderr,"      merge  : %.1fs\n", merge_time);
        log_verbose(stderr,"      count  : %.1fs\n", count_time - load_time - merge_time);
        log_verbose(stderr,"        flatten : %.1fs\n", m_bucketer.d_flatten_time);
        log_verbose(stderr,"        sort    : %.1fs\n", m_bucketer.d_count_sort_time);
        log_verbose(stderr,"        search  : %.1fs\n", m_bucketer.d_search_time);
        log_verbose(stderr,"      setup  : %.1fs\n", m_bucketer.d_setup_time);
        log_verbose(stderr,"        scan   : %.1fs\n", m_bucketer.suffixes.d_scan_time);
        log_verbose(stderr,"        search : %.1fs\n", m_bucketer.suffixes.d_search_time);
    }

    /// print some stats
    ///
    void log_collect_stats()
    {
        log_verbose(stderr,"    load     : %.1fs\n", load_time);
        log_verbose(stderr,"    collect  : %.1fs\n", collect_time);
        log_verbose(stderr,"      setup    : %.1fs\n", m_bucketer.d_setup_time);
        log_verbose(stderr,"        scan   : %.1fs\n", m_bucketer.suffixes.d_scan_time);
        log_verbose(stderr,"        search : %.1fs\n", m_bucketer.suffixes.d_search_time);
        log_verbose(stderr,"      flatten  : %.1fs\n", m_bucketer.d_flatten_time);
        log_verbose(stderr,"      filter   : %.1fs\n", m_bucketer.d_filter_time);
        log_verbose(stderr,"      remap    : %.1fs\n", m_bucketer.d_remap_time);
        log_verbose(stderr,"      max      : %.1fs\n", m_bucketer.d_max_time);
        log_verbose(stderr,"      sort     : %.1fs\n", m_bucketer.d_collect_sort_time);
        log_verbose(stderr,"      copy     : %.1fs\n", m_bucketer.d_copy_time);
        log_verbose(stderr,"    bin      : %.1fs\n", bin_time);
    }

public:
    DeviceCoreSetSuffixBucketer<SYMBOL_SIZE,N_BITS,DOLLAR_BITS,bucket_type> m_bucketer;
    thrust::device_vector<uint32>                                           d_buckets;
    thrust::device_vector<uint32>                                           d_subbuckets;
    thrust::host_vector<uint64>                                             h_bucket_offsets;
    uint64                                                                  m_global_offset;

    float load_time;
    float count_time;
    float merge_time;

    float collect_time;
    float bin_time;
};

/// A context class to perform suffix bucketing for all suffixes of a string-set, using a host core
///
template <uint32 SYMBOL_SIZE, uint32 N_BITS, uint32 DOLLAR_BITS, typename bucket_type = uint32>
struct HostCoreSetSuffixBucketer
{
    /// constructor
    ///
    HostCoreSetSuffixBucketer() {}

    /// clear internal timers
    ///
    void clear_timers() {}

    /// count the number of suffixes falling in each bucket, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    void count_init()
    {
        const uint32 n_buckets = 1u << N_BITS;

        // initialize the temporary and output vectors
        alloc_storage( h_buckets, n_buckets );

        // initialize the bucket counters
        for (uint32 i = 0; i < n_buckets; ++i)
            h_buckets[i] = 0u;

        // initialize the global suffix counter
        n_suffixes = 0u;
    }

    /// count the number of suffixes falling in each bucket, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    template <typename string_set_type>
    uint64 count(const string_set_type& string_set)
    {
        typedef typename string_set_type::string_type string_type;

        // extract the first word
        const local_set_suffix_word_functor<SYMBOL_SIZE,N_BITS,DOLLAR_BITS,string_set_type,bucket_type> word_functor( string_set, 0u );

        const uint32 n_strings = string_set.size();

        uint64 _n_suffixes = 0u;

        // loop through all the strings in the set
        for (uint32 i = 0; i < n_strings; ++i)
        {
            const string_type string = string_set[i];
            const uint32      string_len = nvbio::length( string );

            // loop through all suffixes
            for (uint32 j = 0; j < string_len; ++j)
            {
                // extract the first word of the j-th suffix
                const bucket_type radix = word_functor( make_uint2( j, i ) );

                // and count it
                ++h_buckets[ radix ];
            }

            // increase total number of suffixes
            _n_suffixes += string_len;
        }

        // update the global counter
        n_suffixes += _n_suffixes;

        return _n_suffixes;
    }

    /// collect the suffixes falling in a given set of buckets, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    template <typename string_set_type, typename bucketmap_iterator>
    uint32 collect(
        const string_set_type&      string_set,
        const uint32                bucket_begin,
        const uint32                bucket_end,
        const uint32                string_offset,
        const bucketmap_iterator    bucketmap)
    {
        typedef typename string_set_type::string_type string_type;

        // extract the first word
        const local_set_suffix_word_functor<SYMBOL_SIZE,N_BITS,DOLLAR_BITS,string_set_type,bucket_type> word_functor( string_set, 0u );

        const uint32 n_strings = string_set.size();

        // keep track of the maximum suffix length
        max_suffix_len = 0u;

        // keep track of the number of collected suffixes
        n_collected = 0u;

        // loop through all the strings in the set
        for (uint32 i = 0; i < n_strings; ++i)
        {
            const string_type string = string_set[i];
            const uint32      string_len = nvbio::length( string );

            // loop through all suffixes
            for (uint32 j = 0; j < string_len; ++j)
            {
                // extract the first word of the j-th suffix
                const bucket_type radix = word_functor( make_uint2( j, i ) );

                // check whether the extracted radix is in the given bucket
                if (radix >= bucket_begin && radix < bucket_end)
                {
                    if (n_collected + 1u > h_radices.size())
                    {
                        h_radices.resize( (n_collected + 1u) * 2u );
                        h_suffixes.resize( (n_collected + 1u) * 2u );
                    }

                    // remap the radix
                    h_radices[ n_collected ]  = bucketmap[ radix ];
                    h_suffixes[ n_collected ] = make_uint2( j, i + string_offset );

                    // advance the output index
                    n_collected++;
                }
            }

            // update the maximum suffix length
            max_suffix_len = nvbio::max( max_suffix_len, string_len );
        }
        return n_collected;
    }

    /// return the amount of used device memory
    ///
    uint64 allocated_device_memory() const { return 0u; }

    /// return the amount of used device memory
    ///
    uint64 allocated_host_memory() const
    {
        return
            h_buckets.size()  * sizeof(uint32)       +
            h_radices.size()  * sizeof(bucket_type)  +
            h_suffixes.size() * sizeof(uint2);
    }

public:
    thrust::host_vector<uint32>      h_buckets;
    thrust::host_vector<bucket_type> h_radices;
    thrust::host_vector<uint2>       h_suffixes;
    uint32                           n_suffixes;
    uint32                           n_collected;
    uint32                           max_suffix_len;
};

/// A context class to perform suffix bucketing for all suffixes of a string-set on the host
///
///\tparam SYMBOL_SIZE          the size of the symbols, in bits
///\tparam BIG_ENDIAN           the endianness of the symbols in each word of the packed string set
///\tparam storage_type         the storage iterator type of the input packed string
///\tparam N_BITS               the number of bits used for bucketing
///\tparam DOLLAR_BITS          the number of bits used for encoding whether and where a dollar occurrs in the first N_BITS
///\tparam bucket_type          the word type used to encode buckets, i.e the first N_BITS of each suffix
///
template <
    uint32   SYMBOL_SIZE,
    bool     BIG_ENDIAN,
    typename storage_type,
    uint32   N_BITS,
    uint32   DOLLAR_BITS,
    typename bucket_type>
struct HostSetSuffixBucketer
{
    typedef priv::ChunkLoader<SYMBOL_SIZE,BIG_ENDIAN,storage_type,uint64*,host_tag,host_tag> chunk_loader_type;
    typedef typename chunk_loader_type::chunk_set_type                                       chunk_set_type;

    /// constructor
    ///
    HostSetSuffixBucketer(mgpu::ContextPtr _mgpu)
    {
        m_bucketers.resize( omp_get_num_procs() );
    }

    /// clear internal timers
    ///
    void clear_timers()
    {
        collect_time = 0.0f;
        merge_time   = 0.0f;
        bin_time     = 0.0f;
    }

    /// count the number of suffixes falling in each bucket, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    template <typename string_set_type>
    uint64 count(
        const string_set_type&             string_set,
              thrust::host_vector<uint32>& h_buckets)
    {
        ScopedTimer<float> count_timer( &count_time );

        const uint32 n_threads = omp_get_max_threads();

        const uint32 n_buckets  = 1u << N_BITS;
        const uint32 chunk_size = 128*1024;
        const uint32 batch_size = n_threads * chunk_size;
        const uint32 n_strings  = string_set.size();

        // initialize bucket counters
        #pragma omp parallel for
        for (int b = 0; b < int(n_buckets); ++b)
            h_buckets[b] = 0u;

        // declare a chunk loader
        chunk_loader_type chunk_loader;

        // keep track of the total number of suffixes
        uint64 n_suffixes = 0u;

        // initialize bucket counters
        for (uint32 i = 0; i < n_threads; ++i)
            m_bucketers[i].count_init();

        // split the string-set in batches
        for (uint32 batch_begin = 0; batch_begin < n_strings; batch_begin += batch_size)
        {
            // determine the end of this batch
            const uint32 batch_end = nvbio::min( batch_begin + batch_size, n_strings );

            // split the batch in chunks, one per core
            #pragma omp parallel
            {
                const uint32 tid         = omp_get_thread_num();
                const uint32 chunk_begin = batch_begin + tid * chunk_size;

                const uint32 chunk_end = nvbio::min( chunk_begin + chunk_size, batch_end );

                // make sure this thread is active
                if (chunk_begin < batch_end)
                {
                    const chunk_set_type chunk_set = chunk_loader.load( string_set, chunk_begin, chunk_end );

                    m_bucketers[tid].count( chunk_set );
                }
            }
        }

        // merge all buckets
        {
            ScopedTimer<float> timer( &merge_time );

            for (uint32 i = 0; i < n_threads; ++i)
            {
                // sum the number of suffixes
                n_suffixes += m_bucketers[i].n_suffixes;

                #pragma omp parallel for
                for (int b = 0; b < int(n_buckets); ++b)
                    h_buckets[b] += m_bucketers[i].h_buckets[b];
            }
        }

        //
        // initialize internal bucketing helpers
        //

        m_global_offset = 0u;

        // scan the bucket offsets so as to have global positions
        alloc_storage( h_bucket_offsets, n_buckets );
        thrust::exclusive_scan(
            thrust::make_transform_iterator( h_buckets.begin(), cast_functor<uint32,uint64>() ),
            thrust::make_transform_iterator( h_buckets.end(),   cast_functor<uint32,uint64>() ),
            h_bucket_offsets.begin() );

        return n_suffixes;
    }

    /// collect the suffixes falling in a given set of buckets, where the buckets
    /// are defined by the first n_bits of the suffix
    ///
    template <typename string_set_type>
    uint64 collect(
        const string_set_type&              string_set,
        const uint32                        bucket_begin,
        const uint32                        bucket_end,
              uint32&                       max_suffix_len,
        const thrust::host_vector<uint32>&  h_subbuckets,
        thrust::host_vector<uint2>&         h_suffixes)
    {
        const uint32 batch_size = 1024*1024;
        const uint32 n_strings  = string_set.size();

        const uint32 n_threads = omp_get_max_threads();

        const uint32 chunk_size = util::divide_ri( batch_size, n_threads );

        // declare a chunk loader
        chunk_loader_type chunk_loader;

        uint64 n_collected = 0u;

        // split the string-set in batches
        for (uint32 batch_begin = 0; batch_begin < n_strings; batch_begin += batch_size)
        {
            // determine the end of this batch
            const uint32 batch_end = nvbio::min( batch_begin + batch_size, n_strings );

            // perform bucketing for all the suffixes in the current batch
            {
                // time collection
                ScopedTimer<float> timer( &collect_time );

                // split the batch in chunks, one per core
                #pragma omp parallel
                {
                    const uint32 tid         = omp_get_thread_num();
                    const uint32 chunk_begin = batch_begin + tid * chunk_size;

                    const uint32 chunk_end = nvbio::min( chunk_begin + chunk_size, batch_end );

                    // make sure this thread is active
                    if (chunk_begin < batch_end)
                    {
                        const chunk_set_type chunk_set = chunk_loader.load( string_set, chunk_begin, chunk_end );

                        m_bucketers[tid].collect(
                            chunk_set,
                            bucket_begin,
                            bucket_end,
                            chunk_begin,
                            h_subbuckets.begin() );
                    }
                }
            }

            max_suffix_len = 0u;

            // merge all output vectors, binning the output suffixes
            for (uint32 i = 0; i < n_threads; ++i)
            {
                const uint32 chunk_begin = batch_begin + i * chunk_size;

                // make sure this thread is active
                if (chunk_begin < batch_end)
                {
                    // time binning
                    ScopedTimer<float> timer( &bin_time );

                    // keep stats on the maximum suffix length
                    max_suffix_len = nvbio::max( max_suffix_len, m_bucketers[i].max_suffix_len );

                    // check whether we are overflowing our output buffer
                    if (n_collected + m_bucketers[i].n_collected > h_suffixes.size())
                    {
                        log_error(stderr,"buffer size exceeded! (%llu/%llu)\n", n_collected + m_bucketers[i].n_collected, uint64( h_suffixes.size() ));
                        exit(1);
                    }

                    // dispatch each suffix to their respective bucket
                    for (uint32 j = 0; j < m_bucketers[i].n_collected; ++j)
                    {
                        const uint2  loc    = m_bucketers[i].h_suffixes[j];
                        const uint32 bucket = m_bucketers[i].h_radices[j];
                        const uint64 slot   = h_bucket_offsets[bucket]++; // this could be done in parallel using atomics

                        NVBIO_CUDA_DEBUG_ASSERT(
                            slot >= m_global_offset,
                            slot <  m_global_offset + h_suffixes.size(),
                            "[%u:%u] = (%u,%u) placed at %llu - %llu (%u)\n", i, j, loc.x, loc.y, slot, m_global_offset, bucket );

                        h_suffixes[ slot - m_global_offset ] = loc;
                    }

                    // keep stats
                    n_collected += m_bucketers[i].n_collected;
                }
            }
        }

        // keep track of how many suffixes we have collected globally
        m_global_offset += n_collected;

        return n_collected;
    }

    /// return the amount of used device memory
    ///
    uint64 allocated_device_memory() const { return 0u; }

    /// return the amount of used device memory
    ///
    uint64 allocated_host_memory() const
    {
        uint64 r = 0u;
        for (uint32 i = 0; i < m_bucketers.size(); ++i)
            r += m_bucketers[i].allocated_host_memory();
        return r;
    }

    /// print some stats
    ///
    void log_count_stats()
    {
        log_verbose(stderr,"    count : %.1fs\n", count_time);
        log_verbose(stderr,"      count : %.1fs\n", count_time - merge_time);
        log_verbose(stderr,"      merge : %.1fs\n", merge_time);
    }

    /// print some stats
    ///
    void log_collect_stats()
    {
        log_verbose(stderr,"    collect  : %.1fs\n", collect_time);
        log_verbose(stderr,"    bin      : %.1fs\n", bin_time);
    }

public:
    std::vector< HostCoreSetSuffixBucketer<SYMBOL_SIZE,N_BITS,DOLLAR_BITS,bucket_type> > m_bucketers;
    uint64                                                                               m_global_offset;
    thrust::host_vector<uint64>                                                          h_bucket_offsets;

    float count_time;
    float merge_time;

    float collect_time;
    float bin_time;
};

} // namespace priv
} // namespace nvbio
