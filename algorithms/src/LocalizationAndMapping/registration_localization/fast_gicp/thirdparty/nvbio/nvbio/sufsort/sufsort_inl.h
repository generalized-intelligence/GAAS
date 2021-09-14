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

#include <nvbio/sufsort/sufsort_priv.h>
#include <nvbio/sufsort/sufsort_bucketing.h>
#include <nvbio/sufsort/sufsort_utils.h>
#include <nvbio/sufsort/compression_sort.h>
#include <nvbio/sufsort/prefix_doubling_sufsort.h>
#include <nvbio/sufsort/blockwise_sufsort.h>
#include <nvbio/sufsort/dcs.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/basic/omp.h>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/timer.h>
#include <thrust/device_vector.h>
#include <thrust/transform_scan.h>
#include <thrust/binary_search.h>
#include <thrust/iterator/constant_iterator.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/sort.h>
#include <mgpuhost.cuh>
#include <moderngpu.cuh>


namespace nvbio {

namespace cuda {

// return the position of the primary suffix of a string
//
template <typename string_type>
typename string_type::index_type find_primary(
    const typename string_type::index_type  string_len,
    const string_type                       string)
{
    NVBIO_VAR_UNUSED const uint32 SYMBOL_SIZE = string_type::SYMBOL_SIZE;

    // compute the primary by simply counting how many of the suffixes between 1 and N
    // are lexicographically less than the primary suffix
    return thrust::transform_reduce(
        thrust::make_counting_iterator<uint32>(1u),
        thrust::make_counting_iterator<uint32>(0u) + string_len,
        bind_second_functor< priv::string_suffix_less<SYMBOL_SIZE,string_type> >(
            priv::string_suffix_less<SYMBOL_SIZE,string_type>( string_len, string ),
            0u ),
        0u,
        thrust::plus<uint32>() ) + 1u;
}

// Sort the suffixes of all the strings in the given string_set
//
template <typename string_set_type, typename output_handler>
void suffix_sort(
    const string_set_type&   string_set,
          output_handler&    output,
          BWTParams*         params)
{
    typedef uint32 word_type;
    NVBIO_VAR_UNUSED const uint32 WORD_BITS   = uint32( 8u * sizeof(word_type) );
    NVBIO_VAR_UNUSED const uint32 DOLLAR_BITS = 4;

    NVBIO_VAR_UNUSED const uint32 SYMBOL_SIZE      = 2u;
    NVBIO_VAR_UNUSED const uint32 SYMBOLS_PER_WORD = priv::symbols_per_word<SYMBOL_SIZE,WORD_BITS,DOLLAR_BITS>();

    int current_device;
    cudaGetDevice( &current_device );
    mgpu::ContextPtr mgpu_ctxt = mgpu::CreateCudaDevice( current_device ); 

    // instantiate a suffix flattener on the string set
    priv::SetSuffixFlattener<SYMBOL_SIZE> suffixes( mgpu_ctxt );
    suffixes.set( string_set );

    // compute the maximum number of words needed to represent a suffix
    const uint32 m = (suffixes.max_length( string_set ) + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

    // compute the number of suffixes
    const uint32 n_suffixes = suffixes.n_suffixes;

    thrust::device_vector<word_type> radices( n_suffixes*2 );
    thrust::device_vector<uint32>    indices( n_suffixes*2 );

    // initialize the list of suffix indices
    thrust::copy(
        thrust::make_counting_iterator<uint32>(0u),
        thrust::make_counting_iterator<uint32>(n_suffixes),
        indices.begin() );

    cuda::SortBuffers<word_type*,uint32*> sort_buffers;
    cuda::SortEnactor                     sort_enactor;

    sort_buffers.selector  = 0;
    sort_buffers.keys[0]   = nvbio::device_view( radices );
    sort_buffers.keys[1]   = nvbio::device_view( radices ) + n_suffixes;
    sort_buffers.values[0] = nvbio::device_view( indices );
    sort_buffers.values[1] = nvbio::device_view( indices ) + n_suffixes;

    // do what is essentially an LSD radix-sort on the suffixes, word by word
    for (int32 word_idx = m-1; word_idx >= 0; --word_idx)
    {
        // extract the given radix word from each of the partially sorted suffixes
        suffixes.flatten(
            string_set,
            word_idx,
            priv::Bits<WORD_BITS,DOLLAR_BITS>(),
            indices.begin() + sort_buffers.selector * n_suffixes,
            radices.begin() + sort_buffers.selector * n_suffixes );

        // and sort them
        sort_enactor.sort( n_suffixes, sort_buffers );
    }

    output.process(
        n_suffixes,
        nvbio::device_view( indices ) + sort_buffers.selector * n_suffixes,
        nvbio::device_view( suffixes.string_ids ),
        nvbio::device_view( suffixes.cum_lengths ));
}

// Sort all the suffixes of a given string
//
template <typename string_type, typename output_iterator>
void suffix_sort(
    const typename stream_traits<string_type>::index_type   string_len,
    const string_type                                       string,
    output_iterator                                         output,
    BWTParams*                                              params)
{
    PrefixDoublingSufSort sufsort;
    sufsort.sort(
        string_len,
        string,
        output + 1u );

    // assign the zero'th suffix
    output[0] = string_len;

    NVBIO_CUDA_DEBUG_STATEMENT( log_verbose(stderr,"    extract  : %5.1f ms\n", 1.0e3f * sufsort.extract_time) );
    NVBIO_CUDA_DEBUG_STATEMENT( log_verbose(stderr,"    gather   : %5.1f ms\n", 1.0e3f * sufsort.gather_time) );
    NVBIO_CUDA_DEBUG_STATEMENT( log_verbose(stderr,"    r-sort   : %5.1f ms\n", 1.0e3f * sufsort.radixsort_time) );
    NVBIO_CUDA_DEBUG_STATEMENT( log_verbose(stderr,"    segment  : %5.1f ms\n", 1.0e3f * sufsort.segment_time) );
    NVBIO_CUDA_DEBUG_STATEMENT( log_verbose(stderr,"    invert   : %5.1f ms\n", 1.0e3f * sufsort.inverse_time) );
    NVBIO_CUDA_DEBUG_STATEMENT( log_verbose(stderr,"    compact  : %5.1f ms\n", 1.0e3f * sufsort.compact_time) );
}

// Sort all the suffixes of a given string
//
template <typename string_type, typename output_handler>
void blockwise_suffix_sort(
    const typename string_type::index_type  string_len,
    string_type                             string,
    output_handler&                         output,
    BWTParams*                              params)
{
    typedef typename string_type::index_type index_type;

    // find a suitable Difference Cover...
    const size_t needed_bytes_64   = size_t( DCS::estimated_sample_size<64>( string_len ) ) * 8u;
    const size_t needed_bytes_128  = size_t( DCS::estimated_sample_size<128>( string_len ) ) * 8u;
    const size_t needed_bytes_256  = size_t( DCS::estimated_sample_size<256>( string_len ) ) * 8u;
    const size_t needed_bytes_512  = size_t( DCS::estimated_sample_size<512>( string_len ) ) * 8u;
    const size_t needed_bytes_1024 = size_t( DCS::estimated_sample_size<1024>( string_len ) ) * 8u;

    size_t free, total;
    cudaMemGetInfo(&free, &total);

    DCS dcs;

    if (free >= 2*needed_bytes_64)
        dcs.init<64>();
    else if (free >= 2*needed_bytes_128)
        dcs.init<128>();
    else if (free >= 2*needed_bytes_256)
        dcs.init<256>();
    else if (free >= 2*needed_bytes_512)
        dcs.init<512>();
    else if (free >= 2*needed_bytes_1024)
        dcs.init<1024>();
    else
        dcs.init<2048>();

    // build a table for our Difference Cover
    log_verbose(stderr, "  building DCS-%u... started\n", dcs.Q);

    blockwise_build(
        dcs,
        string_len,
        string,
        params );

    log_verbose(stderr, "  building DCS-%u... done\n", dcs.Q);

    // and do the Difference Cover based sorting
    log_verbose(stderr, "  DCS-based sorting... started\n");

    blockwise_suffix_sort(
        string_len,
        string,
        string_len,
        thrust::make_counting_iterator<uint32>(0u),
        output,
        &dcs,
        params );

    log_verbose(stderr, "  DCS-based sorting... done\n");
}

// Compute the bwt of a device-side string
//
// \return         position of the primary suffix / $ symbol
//
template <typename string_type, typename output_iterator>
typename string_type::index_type bwt(
    const typename string_type::index_type  string_len,
    string_type                             string,
    output_iterator                         output,
    BWTParams*                              params)
{
    typedef typename string_type::index_type index_type;

    // build a BWT handler
    StringBWTHandler<string_type,output_iterator> bwt_handler(
        string_len,
        string,
        output );

    // and pass it to the blockwise suffix sorter
    blockwise_suffix_sort(
        string_len,
        string,
        bwt_handler,
        params );

    log_verbose(stderr,"\n    primary at %llu\n", bwt_handler.primary);

    // shift back all symbols following the primary
    bwt_handler.remove_dollar();

    return bwt_handler.primary;
}

template <uint32 BUCKETING_BITS_T, uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type>
struct HostBWTConfigGPUBucketer
{
    typedef typename std::iterator_traits<storage_type>::value_type word_type;

    static const uint32 WORD_BITS       = uint32( 8u * sizeof(word_type) );
    static const uint32 DOLLAR_BITS     = WORD_BITS <= 32 ? 4 : 5;
    static const uint32 BUCKETING_BITS  = BUCKETING_BITS_T;

    typedef ConcatenatedStringSet<
            PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,uint64>,
            uint64*>    string_set_type;

    typedef priv::HostStringSetRadices<string_set_type,SYMBOL_SIZE,DOLLAR_BITS,WORD_BITS>   string_set_handler;

    typedef typename priv::word_selector<BUCKETING_BITS>::type  bucket_type;
    typedef priv::DeviceSetSuffixBucketer<
        SYMBOL_SIZE,BIG_ENDIAN,storage_type,
        BUCKETING_BITS,DOLLAR_BITS,bucket_type,
        host_tag>                                               suffix_bucketer;
};

template <uint32 BUCKETING_BITS_T, uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type>
struct HostBWTConfigCPUBucketer
{
    typedef typename std::iterator_traits<storage_type>::value_type word_type;

    static const uint32 WORD_BITS       = uint32( 8u * sizeof(word_type) );
    static const uint32 DOLLAR_BITS     = WORD_BITS <= 32 ? 4 : 5;
    static const uint32 BUCKETING_BITS  = BUCKETING_BITS_T;

    typedef ConcatenatedStringSet<
            PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,uint64>,
            uint64*>    string_set_type;

    typedef priv::HostStringSetRadices<string_set_type,SYMBOL_SIZE,DOLLAR_BITS,WORD_BITS>   string_set_handler;

    typedef typename priv::word_selector<BUCKETING_BITS>::type  bucket_type;
    typedef priv::HostSetSuffixBucketer<
        SYMBOL_SIZE,BIG_ENDIAN,storage_type,
        BUCKETING_BITS,DOLLAR_BITS,bucket_type>                 suffix_bucketer;
};

template <uint32 BUCKETING_BITS_T, uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type>
struct DeviceBWTConfig
{
    typedef typename std::iterator_traits<storage_type>::value_type word_type;

    static const uint32 WORD_BITS   = uint32( 8u * sizeof(word_type) );
    static const uint32 DOLLAR_BITS = WORD_BITS <= 32 ? 4 : 5;
    static const uint32 BUCKETING_BITS  = BUCKETING_BITS_T;

    typedef ConcatenatedStringSet<
            PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,uint64>,
            uint64*>    string_set_type;

    typedef priv::DeviceStringSetRadices<string_set_type,SYMBOL_SIZE,DOLLAR_BITS,WORD_BITS> string_set_handler;

    typedef typename priv::word_selector<BUCKETING_BITS>::type  bucket_type;
    typedef priv::DeviceSetSuffixBucketer<
        SYMBOL_SIZE,BIG_ENDIAN,storage_type,
        BUCKETING_BITS,DOLLAR_BITS,bucket_type,
        device_tag>                                             suffix_bucketer;
};

// simple status class
struct LargeBWTStatus
{
    enum {
        OK          = 0,
        LargeBucket = 1,
    };

    // default constructor
    LargeBWTStatus() : code(OK) {}

    // return whether the status is OK
    operator bool() const { return code == OK ? true : false; }

    uint32 code;
    uint32 bucket_size;
    uint32 bucket_index;
};

template <typename ConfigType, uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type>
struct LargeBWTSkeleton
{
    typedef typename std::iterator_traits<storage_type>::value_type word_type;
    typedef typename ConfigType::string_set_handler                 string_set_handler_type;
    typedef typename ConfigType::bucket_type                        bucket_type;
    typedef typename ConfigType::suffix_bucketer                    suffix_bucketer_type;

    typedef ConcatenatedStringSet<
            typename PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,uint64>::iterator,
            uint64*>    string_set_type;

    // compute the maximum sub-bucket size
    //
    static uint32 max_subbucket_size(
        const thrust::host_vector<uint32>&  h_buckets,
        const uint64                        max_super_block_size,
        const uint32                        limit,
        LargeBWTStatus*                     status)
    {
        NVBIO_VAR_UNUSED const uint32 DOLLAR_BITS = ConfigType::DOLLAR_BITS;
        NVBIO_VAR_UNUSED const uint32 DOLLAR_MASK = (1u << DOLLAR_BITS) - 1u;

        uint32 max_size  = 0u;
        uint32 max_index = 0u;

        // build the subbucket pointers
        for (uint32 bucket_begin = 0, bucket_end = 0; bucket_begin < h_buckets.size(); bucket_begin = bucket_end)
        {
            // grow the block of buckets until we can
            uint64 bucket_size;
            for (bucket_size = 0; (bucket_end < h_buckets.size()) && (bucket_size + h_buckets[bucket_end] <= max_super_block_size); ++bucket_end)
                bucket_size += h_buckets[bucket_end];

            // check whether a single bucket exceeds our host buffer capacity
            // TODO: if this is a short-string bucket, we could handle it with special care,
            // but it requires modifying the collecting loop to output everything directly.
            if (bucket_end == bucket_begin)
                throw nvbio::runtime_error("bucket %u contains %u strings: buffer overflow!", bucket_begin, h_buckets[bucket_begin]);

            // loop through the sub-buckets
            for (uint32 subbucket = bucket_begin; subbucket < bucket_end; ++subbucket)
            {
                // only keep track of buckets that are NOT short-string buckets
                if ((subbucket & DOLLAR_MASK) == DOLLAR_MASK)
                {
                    if (max_size < h_buckets[subbucket])
                    {
                        max_size  = h_buckets[subbucket];
                        max_index = subbucket;
                    }
                }
            }
        }

        if (max_size > limit)
        {
            status->code = LargeBWTStatus::LargeBucket;
            status->bucket_size  = max_size;
            status->bucket_index = max_index;
        }

        return max_size;
    }

    // construct the sub-bucket lists
    //
    static void build_subbuckets(
        const thrust::host_vector<uint32>&  h_buckets,
        thrust::host_vector<uint32>&        h_subbuckets,
        const uint64                        max_super_block_size,
        const uint32                        max_block_size)
    {
        NVBIO_VAR_UNUSED const uint32 DOLLAR_BITS = ConfigType::DOLLAR_BITS;
        NVBIO_VAR_UNUSED const uint32 DOLLAR_MASK = (1u << DOLLAR_BITS) - 1u;

        // build the subbucket pointers
        for (uint32 bucket_begin = 0, bucket_end = 0; bucket_begin < h_buckets.size(); bucket_begin = bucket_end)
        {
            // grow the block of buckets until we can
            uint64 bucket_size;
            for (bucket_size = 0; (bucket_end < h_buckets.size()) && (bucket_size + h_buckets[bucket_end] <= max_super_block_size); ++bucket_end)
                bucket_size += h_buckets[bucket_end];

            // check whether a single bucket exceeds our host buffer capacity
            // TODO: if this is a short-string bucket, we could handle it with special care,
            // but it requires modifying the collecting loop to output everything directly.
            if (bucket_end == bucket_begin)
                throw nvbio::runtime_error("bucket %u contains %u strings: buffer overflow!", bucket_begin, h_buckets[bucket_begin]);

            // build the sub-buckets
            for (uint32 subbucket_begin = bucket_begin, subbucket_end = bucket_begin; subbucket_begin < bucket_end; subbucket_begin = subbucket_end)
            {
                if (h_buckets[subbucket_begin] > max_block_size)
                {
                    // if this is NOT a short-string bucket, we can't cope with it
                    if ((subbucket_begin & DOLLAR_MASK) == DOLLAR_MASK)
                        throw nvbio::runtime_error("bucket %u contains %u strings: buffer overflow!", subbucket_begin, h_buckets[subbucket_begin]);

                    // this is a short-string bucket: we can handle it with special care
                    h_subbuckets[ subbucket_end++ ] = subbucket_begin; // point to the beginning of this sub-bucket
                }
                else
                {
                    // grow the block of sub-buckets until we can
                    uint32 subbucket_size;
                    for (subbucket_size = 0; (subbucket_end < bucket_end) && (subbucket_size + h_buckets[subbucket_end] <= max_block_size); ++subbucket_end)
                    {
                        subbucket_size += h_buckets[subbucket_end];

                        h_subbuckets[ subbucket_end ] = subbucket_begin; // point to the beginning of this sub-bucket
                    }
                }
            }
        }
    }

    template <typename output_handler>
    static LargeBWTStatus enact(
        const string_set_type       string_set,
        output_handler&             output,
        BWTParams*                  params)
    {
        NVBIO_VAR_UNUSED const uint32 BUCKETING_BITS = ConfigType::BUCKETING_BITS;
        NVBIO_VAR_UNUSED const uint32 DOLLAR_BITS    = ConfigType::DOLLAR_BITS;
        NVBIO_VAR_UNUSED const uint32 DOLLAR_MASK    = (1u << DOLLAR_BITS) - 1u;
        NVBIO_VAR_UNUSED const uint32 SLICE_SIZE     = params ? params->radix_slice : 4u;

        const uint32 N = string_set.size();

        LargeBWTStatus          status;

        // allocate an MGPU context
        int current_device;
        cudaGetDevice( &current_device );
        mgpu::ContextPtr        mgpu_ctxt = mgpu::CreateCudaDevice( current_device ); 

        suffix_bucketer_type    bucketer( mgpu_ctxt );
        string_set_handler_type string_set_handler( string_set );
        cuda::CompressionSort   string_sorter( mgpu_ctxt );

        priv::DollarExtractor   dollars;

        const uint64 max_super_block_size = params ?                    // requires max_super_block_size*8 host memory bytes
            (params->host_memory - uint64(128u*1024u*1024u)) / 8u :     // leave 128MB for the bucket counters
            512*1024*1024;
        uint32 max_block_size = params ?
            params->device_memory / 32 :                          // requires max_block_size*32 device memory bytes
            32*1024*1024;                                         // default: 1GB

        log_verbose(stderr,"  super-block-size: %.1f M\n", float(max_super_block_size)/float(1024*1024));
        log_verbose(stderr,"        block-size: %.1f M\n", float(max_block_size)/float(1024*1024));
        thrust::host_vector<uint2>       h_suffixes( max_super_block_size );
        thrust::host_vector<uint8>       h_block_bwt;

        // reuse some buffers
        thrust::device_vector<uint32>    d_indices;
        thrust::device_vector<uint2>     d_bucket_suffixes;
        thrust::device_vector<uint8>     d_block_bwt;
        //thrust::device_vector<uint8>     d_temp_storage;

        //
        // split the suffixes in buckets, and count them
        //

        const uint32 n_buckets = 1u << BUCKETING_BITS;

        thrust::host_vector<uint32> h_buckets( n_buckets );
        thrust::host_vector<uint32> h_subbuckets( n_buckets );

        // count how many suffixes fall in each bucket
        const uint64 total_suffixes = bucketer.count( string_set, h_buckets );

        // find the maximum bucket size
        const uint32 max_bucket_size = thrust::reduce(
            h_buckets.begin(),
            h_buckets.end(),
            0u,
            thrust::maximum<uint32>() );

        // compute the largest non-elementary bucket
        const uint32 largest_subbucket = max_subbucket_size( h_buckets, max_super_block_size, max_block_size, &status );
        if (!status)
        {
            log_verbose(stderr,"    exceeded maximum bucket size\n");
            return status;
        }

        log_verbose(stderr,"    max bucket size: %u (%u)\n", largest_subbucket, max_bucket_size);
        bucketer.log_count_stats();

        float bwt_time    = 0.0f;
        float output_time = 0.0f;

        // output the last character of each string (i.e. the symbols preceding all the dollar signs)
        const uint32 block_size = max_block_size / 4u; // this can be done in relatively small blocks
        for (uint32 block_begin = 0; block_begin < N; block_begin += block_size)
        {
            const uint32 block_end = nvbio::min( block_begin + block_size, N );

            // consume subbucket_size suffixes
            const uint32 n_suffixes = block_end - block_begin;

            Timer timer;
            timer.start();

            priv::alloc_storage( h_block_bwt, n_suffixes );
            priv::alloc_storage( d_block_bwt, n_suffixes );

            // load the BWT symbols
            string_set_handler.dollar_bwt(
                block_begin,
                block_end,
                plain_view( h_block_bwt ) );

            // copy them to the device
            thrust::copy(
                h_block_bwt.begin(),
                h_block_bwt.begin() + n_suffixes,
                d_block_bwt.begin() );

            timer.stop();
            bwt_time += timer.seconds();

            timer.start();

            // extract the dollars
            const uint32 n_dollars = dollars.extract(
                n_suffixes,
                raw_pointer( h_block_bwt ),
                raw_pointer( d_block_bwt ),
                NULL,
                NULL,
                NULL );

            // invoke the output handler
            output.process(
                n_suffixes,
                raw_pointer( h_block_bwt ),
                n_dollars,
                raw_pointer( dollars.h_dollar_ranks ),
                raw_pointer( dollars.h_dollars ) );

            timer.stop();
            output_time += timer.seconds();
        }

        bucketer.clear_timers();

        //
        // at this point, we have to do multiple passes through the input string set,
        // collecting in each pass as many buckets as we can fit in memory at once
        //

        float sufsort_time = 0.0f;
        float collect_time = 0.0f;

        // reduce the scratchpads size if possible
        const uint32 optimal_block_size = 32*1024*1024;
        if (largest_subbucket <= optimal_block_size)
            max_block_size     = optimal_block_size;

        // reserve memory for scratchpads
        {
            log_verbose(stderr,"  allocating scratchpads\n" );

            string_set_handler.reserve( max_block_size, SLICE_SIZE );
            string_sorter.reserve( max_block_size );

            priv::alloc_storage( h_block_bwt,       max_block_size );
            priv::alloc_storage( d_block_bwt,       max_block_size );
            priv::alloc_storage( d_indices,         max_block_size );
            priv::alloc_storage( d_bucket_suffixes, max_block_size );

            log_verbose(stderr,"  allocated device memory: %.1f MB\n",
                float( bucketer.allocated_device_memory()           +
                       string_set_handler.allocated_device_memory() +
                       string_sorter.allocated_device_memory()      +
                       d_block_bwt.size()       * sizeof(uint8)     +
                       d_indices.size()         * sizeof(uint32)    +
                       d_bucket_suffixes.size() * sizeof(uint2)
                ) / float(1024*1024) );
            log_verbose(stderr,"    bucketer : %.1f MB\n", float( bucketer.allocated_device_memory() ) / float(1024*1024) );
            log_verbose(stderr,"    handler  : %.1f MB\n", float( string_set_handler.allocated_device_memory() ) / float(1024*1024) );
            log_verbose(stderr,"    sorter   : %.1f MB\n", float( string_sorter.allocated_device_memory() ) / float(1024*1024) );

            log_verbose(stderr,"  allocated host memory: %.1f MB\n",
                float( bucketer.allocated_host_memory() +
                       string_set_handler.allocated_host_memory() +
                       h_block_bwt.size()       * sizeof(uint8)  +
                       h_suffixes.size()        * sizeof(uint2)  +
                       h_buckets.size()         * sizeof(uint32) +
                       h_subbuckets.size()      * sizeof(uint32)
                ) / float(1024*1024) );
        }

        // now build the sub-bucket lists
        build_subbuckets(
            h_buckets,
            h_subbuckets,
            max_super_block_size,
            max_block_size );

        uint64 global_suffix_offset = 0;

        for (uint32 bucket_begin = 0, bucket_end = 0; bucket_begin < h_buckets.size(); bucket_begin = bucket_end)
        {
            // grow the block of buckets until we can
            uint64 bucket_size;
            for (bucket_size = 0; (bucket_end < h_buckets.size()) && (bucket_size + h_buckets[bucket_end] <= max_super_block_size); ++bucket_end)
                bucket_size += h_buckets[bucket_end];

            //
            // do a global scan to find all suffixes falling in the current super-bucket
            //

            uint64 suffix_count   = 0;
            uint32 max_suffix_len = 0;

            log_verbose(stderr,"  collect buckets[%u:%u] (%llu suffixes)\n", bucket_begin, bucket_end, bucket_size);
            Timer collect_timer;
            collect_timer.start();

            suffix_count = bucketer.collect(
                string_set,
                bucket_begin,
                bucket_end,
                max_suffix_len,
                h_subbuckets,
                h_suffixes );

            collect_timer.stop();
            collect_time += collect_timer.seconds();
            log_verbose(stderr,"  collect : %.1fs (%.1f M suffixes/s - %.1f M scans/s)\n", collect_time, 1.0e-6f*float(global_suffix_offset + suffix_count)/collect_time, 1.0e-6f*float(total_suffixes)/collect_time);
            bucketer.log_collect_stats();

            //
            // at this point we have a large collection of localized suffixes to sort in h_suffixes;
            // we'll do it looping on multiple sub-buckets, on the GPU
            //

            suffix_count = 0u;

            const uint32 n_words = string_set_handler.num_words( max_suffix_len );

            for (uint32 subbucket_begin = bucket_begin, subbucket_end = bucket_begin; subbucket_begin < bucket_end; subbucket_begin = subbucket_end)
            {
                if (h_buckets[subbucket_begin] > max_block_size)
                {
                    // check if this is not a short-string bucket - it should never actually happen as we already tested for it
                    if ((subbucket_begin & DOLLAR_MASK) == DOLLAR_MASK)
                        throw nvbio::runtime_error("bucket %u contains %u strings: overflow!", subbucket_begin, h_buckets[subbucket_begin]);

                    // advance by one
                    ++subbucket_end;

                    const uint32 subbucket_size = h_buckets[subbucket_begin];

                    Timer suf_timer;
                    suf_timer.start();

                    // chop the bucket in multiple blocks
                    for (uint32 block_begin = 0; block_begin < subbucket_size; block_begin += max_block_size)
                    {
                        const uint32 block_end = nvbio::min( block_begin + max_block_size, subbucket_size );

                        // consume subbucket_size suffixes
                        const uint32 n_suffixes = block_end - block_begin;

                        // copy the host suffixes to the device
                        const uint2* h_bucket_suffixes = &h_suffixes[0] + suffix_count + block_begin;

                        // copy the suffix list to the device
                        priv::alloc_storage( d_bucket_suffixes, n_suffixes );
                        thrust::copy(
                            h_bucket_suffixes,
                            h_bucket_suffixes + n_suffixes,
                            d_bucket_suffixes.begin() );

                        // initialize the set radices
                        string_set_handler.init( n_suffixes, h_bucket_suffixes, nvbio::plain_view( d_bucket_suffixes ) );

                        Timer timer;
                        timer.start();

                        priv::alloc_storage( h_block_bwt, n_suffixes );
                        priv::alloc_storage( d_block_bwt, n_suffixes );

                        // load the BWT symbols
                        string_set_handler.bwt(
                            n_suffixes,
                            (const uint32*)NULL,
                            plain_view( h_block_bwt ),
                            plain_view( d_block_bwt ) );

                        timer.stop();
                        bwt_time += timer.seconds();

                        timer.start();

                        // extract the dollars
                        const uint32 n_dollars = dollars.extract(
                            n_suffixes,
                            raw_pointer( h_block_bwt ),
                            raw_pointer( d_block_bwt ),
                            h_bucket_suffixes,
                            raw_pointer( d_bucket_suffixes ),
                            NULL );

                        // invoke the output handler
                        output.process(
                            n_suffixes,
                            raw_pointer( h_block_bwt ),
                            n_dollars,
                            raw_pointer( dollars.h_dollar_ranks ),
                            raw_pointer( dollars.h_dollars ) );

                        timer.stop();
                        output_time += timer.seconds();
                    }
                    
                    suffix_count += subbucket_size;

                    suf_timer.stop();
                    sufsort_time += suf_timer.seconds();
                }
                else
                {
                    // grow the block of sub-buckets until we can
                    uint32 subbucket_size;
                    for (subbucket_size = 0; (subbucket_end < bucket_end) && (subbucket_size + h_buckets[subbucket_end] <= max_block_size); ++subbucket_end)
                        subbucket_size += h_buckets[subbucket_end];

                    log_verbose(stderr,"\r  sufsort buckets[%u:%u] (%.1f M suffixes/s)    ", subbucket_begin, subbucket_end, 1.0e-6f*float(global_suffix_offset + suffix_count)/sufsort_time);
                    if (subbucket_size == 0)
                        continue;

                    // consume subbucket_size suffixes
                    const uint32 n_suffixes = subbucket_size;

                    try
                    {
                        // reserve enough space
                        priv::alloc_storage( d_indices, max_block_size );
                    }
                    catch (...)
                    {
                        log_error(stderr, "LargeBWTSkeleton: d_indices allocation failed!\n");
                        throw;
                    }

                    Timer suf_timer;
                    suf_timer.start();

                    // copy the host suffixes to the device
                    const uint2* h_bucket_suffixes = &h_suffixes[0] + suffix_count;

                    priv::alloc_storage( d_bucket_suffixes, n_suffixes );

                    // copy the suffix list to the device
                    thrust::copy(
                        h_bucket_suffixes,
                        h_bucket_suffixes + n_suffixes,
                        d_bucket_suffixes.begin() );

                    NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "\n    initialize radices\n" ) );

                    // initialize the set radices
                    string_set_handler.init( n_suffixes, h_bucket_suffixes, nvbio::plain_view( d_bucket_suffixes ) );

                    NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "    sort strings\n" ) );

                    cuda::DiscardDelayList delay_list;

                    string_sorter.sort(
                        string_set_handler,
                        n_suffixes,
                        n_words,
                        thrust::make_counting_iterator<uint32>(0u),
                        d_indices.begin(),
                        uint32(-1),
                        delay_list,
                        SLICE_SIZE );

                    NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "    BWT transform\n" ) );

                    Timer timer;
                    timer.start();

                    priv::alloc_storage( h_block_bwt, n_suffixes );
                    priv::alloc_storage( d_block_bwt, n_suffixes );

                    // load the BWT symbols
                    string_set_handler.bwt(
                        n_suffixes,
                        plain_view( d_indices ),
                        plain_view( h_block_bwt ),
                        plain_view( d_block_bwt ) );

                    timer.stop();
                    bwt_time += timer.seconds();

                    NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "    BWT output\n" ) );

                    timer.start();

                    // invoke the output handler
                    const uint32 n_dollars = dollars.extract(
                        n_suffixes,
                        raw_pointer( h_block_bwt ),
                        raw_pointer( d_block_bwt ),
                        h_bucket_suffixes,
                        raw_pointer( d_bucket_suffixes ),
                        raw_pointer( d_indices ) );

                    // invoke the output handler
                    output.process(
                        n_suffixes,
                        raw_pointer( h_block_bwt ),
                        n_dollars,
                        raw_pointer( dollars.h_dollar_ranks ),
                        raw_pointer( dollars.h_dollars ) );

                    timer.stop();
                    output_time += timer.seconds();
                    
                    suffix_count += subbucket_size;

                    suf_timer.stop();
                    sufsort_time += suf_timer.seconds();
                }
            }
            log_verbose(stderr,"\r  sufsort : %.1fs (%.1f M suffixes/s)                     \n", sufsort_time, 1.0e-6f*float(global_suffix_offset + suffix_count)/sufsort_time);
            log_verbose(stderr,"    copy     : %.1fs\n", string_sorter.copy_time);
            log_verbose(stderr,"    extract  : %.1fs\n", string_sorter.extract_time);
            log_verbose(stderr,"    r-sort   : %.1fs\n", string_sorter.radixsort_time);
            log_verbose(stderr,"    compress : %.1fs\n", string_sorter.compress_time);
            log_verbose(stderr,"    compact  : %.1fs\n", string_sorter.compact_time);
            log_verbose(stderr,"    scatter  : %.1fs\n", string_sorter.scatter_time);
            log_verbose(stderr,"    bwt      : %.1fs\n", bwt_time);
            log_verbose(stderr,"    output   : %.1fs\n", output_time);

            global_suffix_offset += suffix_count;
        }
        return status;
    }
};

// Compute the bwt of a device-side string set
//
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename output_handler>
void bwt(
    const ConcatenatedStringSet<
        PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,uint64>,
        uint64*>                    string_set,
        output_handler&             output,
        BWTParams*                  params)
{
    typedef cuda::DeviceBWTConfig<16,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_16; // 16-bits bucketing
    typedef cuda::DeviceBWTConfig<20,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_20; // 20-bits bucketing
    typedef cuda::DeviceBWTConfig<24,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_24; // 24-bits bucketing

    cuda::LargeBWTStatus status;

    // try 16-bit bucketing
    if (status = cuda::LargeBWTSkeleton<config_type_16,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
        string_set,
        output,
        params ))
        return;

    // try 20-bit bucketing
    if (status = cuda::LargeBWTSkeleton<config_type_20,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
        string_set,
        output,
        params ))
        return;

    // try 24-bit bucketing
    if (status = cuda::LargeBWTSkeleton<config_type_24,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
        string_set,
        output,
        params ))
        return;

    if (status.code == LargeBWTStatus::LargeBucket)
        throw nvbio::runtime_error("subbucket %u contains %u strings: buffer overflow!\n  please try increasing the device memory limit to at least %u MB\n", status.bucket_index, status.bucket_size, util::divide_ri( status.bucket_size, 1024u*1024u )*32u);
}

} // namespace cuda

// Compute the bwt of a host-side string set
//
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename output_handler>
void large_bwt(
    const ConcatenatedStringSet<
        PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,uint64>,
        uint64*>                    string_set,
        output_handler&             output,
        BWTParams*                  params)
{
    cuda::LargeBWTStatus status;

    const uint32 bucketing_bits = params ? params->bucketing_bits : 16u;

    if (params && params->cpu_bucketing)
    {
        typedef cuda::HostBWTConfigCPUBucketer<16,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_16; // 16-bits bucketing
        typedef cuda::HostBWTConfigCPUBucketer<20,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_20; // 20-bits bucketing
        typedef cuda::HostBWTConfigCPUBucketer<24,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_24; // 24-bits bucketing
        typedef cuda::HostBWTConfigCPUBucketer<26,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_26; // 26-bits bucketing

        // try 16-bit bucketing
        if (bucketing_bits <= 16u)
        {
            if (status = cuda::LargeBWTSkeleton<config_type_16,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
                string_set,
                output,
                params ))
                return;
        }

        // try 20-bit bucketing
        if (bucketing_bits <= 20u)
        {
            if (status = cuda::LargeBWTSkeleton<config_type_20,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
                string_set,
                output,
                params ))
                return;
        }

        // try 24-bit bucketing
        if (bucketing_bits <= 24u)
        {
            if (status = cuda::LargeBWTSkeleton<config_type_24,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
                string_set,
                output,
                params ))
                return;
        }

        // try 26-bit bucketing
        if (status = cuda::LargeBWTSkeleton<config_type_26,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
            string_set,
            output,
            params ))
            return;
    }
    else
    {
        typedef cuda::HostBWTConfigGPUBucketer<16,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_16; // 16-bits bucketing
        typedef cuda::HostBWTConfigGPUBucketer<20,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_20; // 20-bits bucketing
        typedef cuda::HostBWTConfigGPUBucketer<24,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_24; // 24-bits bucketing
        typedef cuda::HostBWTConfigGPUBucketer<26,SYMBOL_SIZE,BIG_ENDIAN,storage_type> config_type_26; // 26-bits bucketing

        // try 16-bit bucketing
        if (bucketing_bits <= 16u)
        {
            if (status = cuda::LargeBWTSkeleton<config_type_16,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
                string_set,
                output,
                params ))
                return;
        }

        // try 20-bit bucketing
        if (bucketing_bits <= 20u)
        {
            if (status = cuda::LargeBWTSkeleton<config_type_20,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
                string_set,
                output,
                params ))
                return;
        }

        // try 24-bit bucketing
        if (bucketing_bits <= 24u)
        {
            if (status = cuda::LargeBWTSkeleton<config_type_24,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
                string_set,
                output,
                params ))
                return;
        }

        // try 26-bit bucketing
        if (status = cuda::LargeBWTSkeleton<config_type_26,SYMBOL_SIZE,BIG_ENDIAN,storage_type>::enact(
            string_set,
            output,
            params ))
            return;
    }

    if (status.code == cuda::LargeBWTStatus::LargeBucket)
        throw nvbio::runtime_error("subbucket %u contains %u strings: buffer overflow!\n  please try increasing the device memory limit to at least %u MB\n", status.bucket_index, status.bucket_size, util::divide_ri( status.bucket_size, 1024u*1024u )*32u);
}

} // namespace nvbio
