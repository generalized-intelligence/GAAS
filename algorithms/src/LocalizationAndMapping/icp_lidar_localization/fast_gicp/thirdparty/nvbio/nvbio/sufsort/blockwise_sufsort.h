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
#include <nvbio/sufsort/compression_sort.h>
#include <nvbio/sufsort/prefix_doubling_sufsort.h>
#include <nvbio/sufsort/dcs.h>
#include <nvbio/strings/string_set.h>
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

///@addtogroup Sufsort
///@{

/// Sort a list of suffixes of a given string
///
/// \return         position of the primary suffix / $ symbol
///
template <typename string_type, typename suffix_iterator, typename output_handler>
void blockwise_suffix_sort(
    const typename string_type::index_type  string_len,
    string_type                             string,
    const typename string_type::index_type  n_suffixes,
    suffix_iterator                         suffixes,
    output_handler&                         output,
    const DCS*                              dcs,
    BWTParams*                              params)
{
    typedef typename string_type::index_type index_type;
    typedef uint32 word_type;

    NVBIO_VAR_UNUSED const uint32 SYMBOL_SIZE    = string_type::SYMBOL_SIZE;
    NVBIO_VAR_UNUSED const uint32 BUCKETING_BITS = 20;
    NVBIO_VAR_UNUSED const uint32 DOLLAR_BITS    = 4;
    NVBIO_VAR_UNUSED const uint32 WORD_BITS      = uint32( 8u * sizeof(uint32) );

    const uint32     M = 1024*1024;     // requires M * 16 device memory bytes
    const index_type N = n_suffixes;

    const uint32 n_chunks = (n_suffixes + M-1) / M;

    // check the amount of available device memory
    size_t free, total;
    cudaMemGetInfo(&free, &total);

    //const size_t max_super_block_mem  = free - max_block_size*16u - 512u*1024u*1024u;
    //const uint32 max_super_block_size = uint32( max_super_block_mem / 4u );
    const uint32 max_super_block_size = nvbio::min(             // requires max_super_block_size*4 host memory bytes
        index_type( params ?
            (params->host_memory - (128u*1024u*1024u)) / 4u :   // leave 128MB for the bucket counters
            512*1024*1024 ),
        string_len );

    // determine the amount of available device memory
    const uint64 max_device_memory = params ?
        nvbio::min( uint64( free ), uint64( params->device_memory ) ) :
                    uint64( free );

    // determine the maximum block size
    uint32 max_block_size    = 0;
    uint32 needed_device_mem = 0;

    // start from a work-optimal block size and progressively halve it until we fit our memory budget
    for (max_block_size = 32*1024*1024; max_block_size >= 1024*1024; max_block_size /= 2)
    {
        const uint32 n_buckets = 1u << (BUCKETING_BITS);

        // compute the amount of needed device memory
        needed_device_mem = CompressionSort::needed_device_memory( max_block_size ) +   // CompressionSort requirements
                            max_block_size*(sizeof(uint32)*3+sizeof(uint8))         +   // d_subbucket_suffixes + d_delayed_suffixes + d_delayed_slots + d_block_bwt
                            n_buckets * sizeof(uint32) * 3                          +   // d_buckets + d_subbuckets + bucketer.d_buckets
                            128*1024*1024;                                              // scraps

        // check whether we fit in our budget
        if (needed_device_mem <= max_device_memory)
            break;
    }

    const uint32 DELAY_BUFFER = 1024*1024;

    log_verbose(stderr,"  super-block-size: %.1f M\n", float(max_super_block_size)/float(1024*1024));
    log_verbose(stderr,"        block-size: %.1f M\n", float(max_block_size)/float(1024*1024));
    thrust::host_vector<uint32> h_super_suffixes( max_super_block_size, 0u );
    thrust::host_vector<uint32> h_block_suffixes( max_block_size );
    thrust::host_vector<uint32> h_block_radices( max_block_size );

    log_verbose(stderr,"  device-alloc(%.1f GB)... started\n", float(needed_device_mem)/float(1024*1024*1024));
    log_verbose(stderr,"    free: %.1f GB\n", float(free)/float(1024*1024*1024));

    thrust::device_vector<uint32> d_subbucket_suffixes( max_block_size );
    thrust::device_vector<uint32> d_delayed_suffixes( max_block_size );
    thrust::device_vector<uint32> d_delayed_slots( max_block_size );
    thrust::device_vector<uint8>  d_block_bwt( max_block_size );

    DelayList<thrust::device_vector<uint32>::iterator> delay_list(
        d_delayed_suffixes.begin(),
        d_delayed_slots.begin() );

    int current_device;
    cudaGetDevice( &current_device );
    mgpu::ContextPtr mgpu_ctxt = mgpu::CreateCudaDevice( current_device ); 

  #define COMPRESSION_SORTING
  #if defined(COMPRESSION_SORTING)
    CompressionSort compression_sort( mgpu_ctxt );
    compression_sort.reserve( max_block_size );
  #endif

    priv::StringSuffixBucketer<SYMBOL_SIZE,BUCKETING_BITS,DOLLAR_BITS> bucketer;

    log_verbose(stderr,"  device-alloc(%.1f GB)... done\n", float(needed_device_mem)/float(1024*1024*1024));
    log_verbose(stderr,"  bucket counting\n");

    // global bucket sizes
    thrust::device_vector<uint32> d_buckets( 1u << (BUCKETING_BITS), 0u );

    for (uint32 chunk_id = 0; chunk_id < n_chunks; ++chunk_id)
    {
        const index_type chunk_begin = chunk_id * M;
        const index_type chunk_end   = nvbio::min( chunk_begin + M, N );
        const uint32     chunk_size  = uint32( chunk_end - chunk_begin );

        // count the chunk's buckets
        bucketer.count( chunk_size, suffixes + chunk_begin, string_len, string );

        // and merge them in with the global buckets
        thrust::transform(
            bucketer.d_buckets.begin(),
            bucketer.d_buckets.end(),
            d_buckets.begin(),
            d_buckets.begin(),
            thrust::plus<uint32>() );
    }

    thrust::host_vector<uint32> h_buckets( d_buckets );
    thrust::host_vector<uint32> h_bucket_offsets( d_buckets.size() );
    thrust::host_vector<uint32> h_subbuckets( d_buckets.size() );

    NVBIO_VAR_UNUSED const uint32 max_bucket_size = thrust::reduce(
        d_buckets.begin(),
        d_buckets.end(),
        0u,
        thrust::maximum<uint32>() );

    log_verbose(stderr,"    max bucket size: %u\n", max_bucket_size);
    log_verbose(stderr,"      c-sort  : %.1fs\n", bucketer.d_count_sort_time);

    //
    // at this point, we have to do multiple passes through the input string set,
    // collecting in each pass as many buckets as we can fit in memory at once
    //

    // scan the bucket offsets so as to have global positions
    thrust::exclusive_scan(
        h_buckets.begin(),
        h_buckets.end(),
        h_bucket_offsets.begin() );

    // build the subbucket pointers
    for (uint32 bucket_begin = 0, bucket_end = 0; bucket_begin < h_buckets.size(); bucket_begin = bucket_end)
    {
        // grow the block of buckets until we can
        uint32 bucket_size;
        for (bucket_size = 0; (bucket_end < h_buckets.size()) && (bucket_size + h_buckets[bucket_end] < max_super_block_size); ++bucket_end)
            bucket_size += h_buckets[bucket_end];

        // build the sub-buckets
        for (uint32 subbucket_begin = bucket_begin, subbucket_end = bucket_begin; subbucket_begin < bucket_end; subbucket_begin = subbucket_end)
        {
            // check if this bucket is too large
            if (h_buckets[subbucket_begin] > max_block_size)
                throw nvbio::runtime_error("bucket %u contains %u strings: buffer overflow!", subbucket_begin, h_buckets[subbucket_begin]);

            // grow the block of sub-buckets until we can
            uint32 subbucket_size;
            for (subbucket_size = 0; (subbucket_end < bucket_end) && (subbucket_size + h_buckets[subbucket_end] < max_block_size); ++subbucket_end)
            {
                subbucket_size += h_buckets[subbucket_end];

                h_subbuckets[ subbucket_end ] = subbucket_begin; // point to the beginning of this sub-bucket
            }
        }
    }

    // build the subbucket pointers
    thrust::device_vector<uint32> d_subbuckets( h_subbuckets );

    NVBIO_VAR_UNUSED float sufsort_time     = 0.0f;
    NVBIO_VAR_UNUSED float collect_time     = 0.0f;
    NVBIO_VAR_UNUSED float bwt_copy_time    = 0.0f;
    NVBIO_VAR_UNUSED float bwt_scatter_time = 0.0f;

    index_type global_suffix_offset = 0;

    for (uint32 bucket_begin = 0, bucket_end = 0; bucket_begin < h_buckets.size(); bucket_begin = bucket_end)
    {
        // grow the block of buckets until we can
        uint32 bucket_size;
        for (bucket_size = 0; (bucket_end < h_buckets.size()) && (bucket_size + h_buckets[bucket_end] < max_super_block_size); ++bucket_end)
            bucket_size += h_buckets[bucket_end];

        uint32 suffix_count = 0;

        log_verbose(stderr,"  collect buckets[%u:%u] (%u suffixes)\n", bucket_begin, bucket_end, bucket_size);
        Timer collect_timer;
        collect_timer.start();

        for (uint32 chunk_id = 0; chunk_id < n_chunks; ++chunk_id)
        {
            const index_type chunk_begin = chunk_id * M;
            const index_type chunk_end   = nvbio::min( chunk_begin + M, N );
            const uint32     chunk_size  = uint32( chunk_end - chunk_begin );

            // collect the chunk's suffixes within the bucket range
            const uint32 n_collected = bucketer.collect(
                chunk_size,
                suffixes + chunk_begin,
                string_len,
                string,
                bucket_begin,
                bucket_end,
                d_subbuckets.begin(),
                h_block_radices.begin(),
                h_block_suffixes.begin() );

            // dispatch each suffix to their respective bucket
            for (uint32 i = 0; i < n_collected; ++i)
            {
                const uint32 loc    = h_block_suffixes[i];
                const uint32 bucket = h_block_radices[i];
                const uint64 slot   = h_bucket_offsets[bucket]++; // this could be done in parallel using atomics

                NVBIO_CUDA_DEBUG_ASSERT(
                    slot >= global_suffix_offset,
                    slot <  global_suffix_offset + max_super_block_size,
                    "[%u] = %u placed at %llu - %llu (%u)\n", i, loc, slot, global_suffix_offset, bucket );

                h_super_suffixes[ slot - global_suffix_offset ] = loc;
            }

            suffix_count += n_collected;

            if (suffix_count > max_super_block_size)
            {
                log_error(stderr,"buffer size exceeded! (%u/%u)\n", suffix_count, max_block_size);
                exit(1);
            }
        }
        NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
        collect_timer.stop();
        collect_time += collect_timer.seconds();

        log_verbose(stderr,"  collect : %.1fs, %.1f M suffixes/s\n", collect_time, 1.0e-6f*float(global_suffix_offset + suffix_count)/collect_time);
        log_verbose(stderr,"    setup   : %.1fs\n", bucketer.d_setup_time);
        log_verbose(stderr,"    flatten : %.1fs\n", bucketer.d_flatten_time);
        log_verbose(stderr,"    b-sort  : %.1fs\n", bucketer.d_collect_sort_time);
        log_verbose(stderr,"    search  : %.1fs\n", bucketer.d_remap_time);
        log_verbose(stderr,"    copy    : %.1fs\n", bucketer.d_copy_time);

        //
        // at this point we have a large collection of localized suffixes to sort in d_subbucket_suffixes;
        // we'll do it looping on multiple sub-buckets, on the GPU
        //

        NVBIO_VAR_UNUSED const uint32 SYMBOLS_PER_WORD = priv::symbols_per_word<SYMBOL_SIZE, WORD_BITS,DOLLAR_BITS>();

        suffix_count = 0u;

        for (uint32 subbucket_begin = bucket_begin, subbucket_end = bucket_begin; subbucket_begin < bucket_end; subbucket_begin = subbucket_end)
        {
            // grow the block of sub-buckets until we can
            uint32 subbucket_size;
            for (subbucket_size = 0; (subbucket_end < bucket_end) && (subbucket_size + h_buckets[subbucket_end] < max_block_size); ++subbucket_end)
                subbucket_size += h_buckets[subbucket_end];

            log_verbose(stderr,"\r  sufsort buckets[%u:%u] (%u suffixes, %.1f M suffixes/s)        ", subbucket_begin, subbucket_end, subbucket_size, 1.0e-6f*float(global_suffix_offset + suffix_count)/sufsort_time );
            log_debug(stderr, "    compression-sort\n");

            // consume subbucket_size suffixes
            const uint32 n_suffixes = subbucket_size;

            Timer timer;
            timer.start();

            // initialize the device sorting indices
            thrust::copy(
                h_super_suffixes.begin() + suffix_count,
                h_super_suffixes.begin() + suffix_count + n_suffixes,
                d_subbucket_suffixes.begin() );

        #if defined(COMPRESSION_SORTING)
            delay_list.set_offset( global_suffix_offset + suffix_count );

            const uint32 min_delay_pass = dcs == NULL ? 16u   : nvbio::max( util::divide_ri( dcs->Q, SYMBOLS_PER_WORD ), 8u );
            const uint32 max_delay_pass = dcs == NULL ? 1000u : nvbio::max( util::divide_ri( dcs->Q, SYMBOLS_PER_WORD ), 24u );

            compression_sort.sort(
                string_len,                     // the main string length
                string,                         // the main string
                n_suffixes,                     // number of suffixes to sort
                d_subbucket_suffixes.begin(),   // the suffixes to sort
                min_delay_pass,                 // minimum number of words to sort before possibly delaying
                max_delay_pass,                 // maximum number of words to sort before delaying
                delay_list );                   // the delay list
        #else // if defined(COMPRESSION_SORTING)
            if (dcs)
            {
                // and sort the corresponding suffixes
                thrust::stable_sort(
                    d_subbucket_suffixes.begin(),
                    d_subbucket_suffixes.begin() + n_suffixes,
                    priv::DCS_string_suffix_less<SYMBOL_SIZE,string_type>(
                        string_len,
                        string,
                        nvbio::plain_view( *dcs ) ) );
            }
            else
            {
                // and sort the corresponding suffixes
                thrust::stable_sort(
                    d_subbucket_suffixes.begin(),
                    d_subbucket_suffixes.begin() + n_suffixes,
                    priv::string_suffix_less<SYMBOL_SIZE,string_type>( string_len, string ) );
            }
        #endif
            NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
            timer.stop();
            sufsort_time += timer.seconds();

            // process the (partially) sorted block
            output.process_batch(
                n_suffixes,
                nvbio::plain_view( d_subbucket_suffixes ) );

            #if defined(COMPRESSION_SORTING)
            // process delayed suffixes
            if (delay_list.count &&
                (delay_list.count >= DELAY_BUFFER ||
                 subbucket_end == bucket_end))
            {
                log_debug(stderr, "    sort delayed-suffixes (%u)\n", delay_list.count);
                timer.start();

                // and sort the corresponding suffixes
                if (dcs)
                {
                    thrust::stable_sort(
                        delay_list.indices,
                        delay_list.indices + delay_list.count,
                        priv::DCS_string_suffix_less<SYMBOL_SIZE,string_type>(
                            string_len,
                            string,
                            nvbio::plain_view( *dcs ) ) );
/*
                    mgpu::MergesortKeys(
                        nvbio::plain_view( d_delayed_suffixes ),
                        delay_list.count,
                        priv::DCS_string_suffix_less<SYMBOL_SIZE,string_type>(
                            string_len,
                            string,
                            nvbio::plain_view( *dcs ) ),
                        *mgpu_ctxt );*/
                }
                else
                {
                    DiscardDelayList discard_list;
                    compression_sort.sort(
                        string_len,
                        string,
                        delay_list.count,
                        delay_list.indices,
                        uint32(-1),
                        uint32(-1),
                        discard_list );
                }

                NVBIO_CUDA_DEBUG_STATEMENT( cudaDeviceSynchronize() );
                timer.stop();
                sufsort_time += timer.seconds();
                compression_sort.stablesort_time += timer.seconds();
                //fprintf(stderr,"    %.1f s\n", timer.seconds());

                // scatter the delayed suffixes
                output.process_scattered(
                    delay_list.count,
                    nvbio::plain_view( d_delayed_suffixes ),
                    nvbio::plain_view( d_delayed_slots ) );

                // clear the list
                delay_list.clear();
            }
            #endif

            suffix_count += subbucket_size;
        }

        log_verbose(stderr,"\r  sufsort : %.1fs (%.1f M suffixes/s)                     \n", sufsort_time, 1.0e-6f*float(global_suffix_offset + suffix_count)/sufsort_time);

      #if defined(COMPRESSION_SORTING)
        log_verbose(stderr,"    extract  : %.1fs\n", compression_sort.extract_time);
        log_verbose(stderr,"    r-sort   : %.1fs\n", compression_sort.radixsort_time);
        log_verbose(stderr,"    s-sort   : %.1fs\n", compression_sort.stablesort_time);
        log_verbose(stderr,"    compress : %.1fs\n", compression_sort.compress_time);
        log_verbose(stderr,"    compact  : %.1fs\n", compression_sort.compact_time);
        log_verbose(stderr,"    bwt-copy : %.1fs\n", bwt_copy_time);
        log_verbose(stderr,"    bwt-scat : %.1fs\n", bwt_scatter_time);
      #endif

        global_suffix_offset += suffix_count;
    }
}

/// build the difference cover sample of a given string
///
template <typename string_type>
void blockwise_build(
    DCS&                                    dcs,
    const typename string_type::index_type  string_len,
    string_type                             string,
    BWTParams*                              params)
{
    typedef typename string_type::index_type index_type;

    const index_type block_size = 16*1024*1024;

    //
    // build the list of DC sample suffixes
    //

    // compute the size of the DC sample
    //const index_type estimated_sample_size = index_type( dcs.estimate_sample_size( string_len ) );

    thrust::device_vector<uint8> d_temp_storage;

    index_type estimated_sample_size = 0u;

    for (index_type block_begin = 0; block_begin < string_len; block_begin += block_size)
    {
        const index_type block_end = nvbio::min(
            block_begin + block_size,
            string_len );

        const priv::DCS_predicate in_dcs( dcs.Q, nvbio::plain_view( dcs.d_bitmask ) );

        estimated_sample_size += cuda::reduce(
            uint32( block_end - block_begin ),
            thrust::make_transform_iterator(
                thrust::make_transform_iterator(
                    thrust::make_counting_iterator<uint32>(0u) + block_begin, in_dcs ),
                    cast_functor<bool,uint32>() ),
            thrust::plus<uint32>(),
            d_temp_storage );
    }
    log_verbose(stderr,"  allocating DCS: %.1f MB\n", float(size_t( estimated_sample_size )*8u)/float(1024*1024));

    thrust::device_vector<uint32> d_sample( estimated_sample_size );

    index_type sample_size = 0;

    for (index_type block_begin = 0; block_begin < string_len; block_begin += block_size)
    {
        const index_type block_end = nvbio::min(
            block_begin + block_size,
            string_len );

        const priv::DCS_predicate in_dcs( dcs.Q, nvbio::plain_view( dcs.d_bitmask ) );

        sample_size += cuda::copy_if(
            uint32( block_end - block_begin ),
            thrust::make_counting_iterator<uint32>(0u) + block_begin,
            d_sample.begin() + sample_size,
            in_dcs,
            d_temp_storage );

        if (sample_size > estimated_sample_size)
        {
            log_error(stderr,"  DCS buffer overflow (%llu / %llu)\n", uint64( sample_size ), uint64( estimated_sample_size ));
            throw runtime_error( "DCS buffer overflow" );
        }
    }

    // alloc enough space for the DC ranks
    dcs.d_ranks.resize( util::round_i( sample_size, dcs.N ), 0u );

    // and sort the corresponding suffixes
    DCSSuffixRanker ranker( nvbio::plain_view( dcs ) );
    blockwise_suffix_sort(
        string_len,
        string,
        sample_size,
        d_sample.begin(),
        ranker,
        NULL,
        params );
}

} // namespace cuda
} // namespace nvbio
