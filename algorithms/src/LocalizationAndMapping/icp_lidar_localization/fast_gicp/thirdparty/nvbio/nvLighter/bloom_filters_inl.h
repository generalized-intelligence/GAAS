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

// bloom_filters_inl.h
//

#pragma once

// setup the internal storage
//
template <typename system_tag>
bool BloomFilters<system_tag>::setup(const int _device, const uint64 sampled_words, const uint64 trusted_words)
{
    device = _device;

    log_info(stderr, "  setup device %d\n", device);
    set_device();

    // gather device memory stats
    size_t free_device, total_device;
    device_memory( &free_device, &total_device );
    log_stats(stderr, "  device has %ld of %ld MB free\n", free_device/1024/1024, total_device/1024/1024);

    const uint32 bytes_per_word = 4u;
    const uint64 needed_bytes = (sampled_words + trusted_words)*bytes_per_word + 256*1024*1024;

    if (needed_bytes > free_device)
    {
        log_warning(stderr, "  insufficient memory: %.2f GB required\n", float( needed_bytes ) / float(1024*1024*1024));
        return false;
    }

    log_info(stderr, "  allocating sampled kmer filter (%.2f GB)\n", float( sampled_words*bytes_per_word ) / float(1024*1024*1024));
    sampled_kmers_storage.resize( sampled_words, 0u );

    log_info(stderr, "  allocating trusted kmer filter (%.2f GB)\n", float( trusted_words*bytes_per_word ) / float(1024*1024*1024));
    trusted_kmers_storage.resize( trusted_words, 0u );

    stats.resize( 10 );
    return true;
}

template <typename system_tag>
const nvbio::vector<system_tag,uint32>& BloomFilters<system_tag>::get_kmers(const KmersType type) const
{
    return (type == SAMPLED_KMERS) ? sampled_kmers_storage :
                                     trusted_kmers_storage;
}
template <typename system_tag>
nvbio::vector<system_tag,uint32>& BloomFilters<system_tag>::get_kmers(const KmersType type)
{
    return (type == SAMPLED_KMERS) ? sampled_kmers_storage :
                                     trusted_kmers_storage;
}
template <typename system_tag>
void BloomFilters<system_tag>::get_kmers(const KmersType type, nvbio::vector<nvbio::host_tag,uint32>& bf)
{
    set_device();

    if (type == SAMPLED_KMERS) bf = sampled_kmers_storage;
    else                       bf = trusted_kmers_storage;
}
template <typename system_tag>
void BloomFilters<system_tag>::set_kmers(const KmersType type, const nvbio::vector<nvbio::host_tag,uint32>& bf)
{
    set_device();

    if (type == SAMPLED_KMERS) sampled_kmers_storage = bf;
    else                       trusted_kmers_storage = bf;
}
template <typename system_tag>
void BloomFilters<system_tag>::set_threshold(const nvbio::vector<nvbio::host_tag,uint32>& _threshold)
{
    set_device();

    threshold = _threshold;
}

template <typename system_tag>
void BloomFilters<system_tag>::set_device() const
{
    if (nvbio::equal<system_tag,nvbio::device_tag>())
        cudaSetDevice( device );
}
template <typename system_tag>
void BloomFilters<system_tag>::device_memory(size_t* free_device, size_t* total_device) const
{
    if (nvbio::equal<system_tag,nvbio::device_tag>())
        cudaMemGetInfo( free_device, total_device );
    else
        *free_device = *total_device = 1024llu * 1024llu * 1024llu * 1024llu; // TODO!
}

// merge several Bloom filters
//
inline
void merge(
    BloomFilters<nvbio::host_tag>*      h_bloom_filters,
    const uint32                        device_count,
    BloomFilters<nvbio::device_tag>*    d_bloom_filters,
    const KmersType                     type)
{
    // merge the Bloom filters on the host
    if (h_bloom_filters && device_count)
    {
        log_info(stderr,"  merge filters\n");

        // get the host Bloom filter
        nvbio::vector<nvbio::host_tag,uint32>& bf = h_bloom_filters->get_kmers( type );
        nvbio::vector<nvbio::host_tag,uint32>  bf2;

        // merge with all the device ones
        for (uint32 d = 0; d < device_count; ++d)
        {
            d_bloom_filters[d].get_kmers( type, bf2 );

            #pragma omp parallel for
            for (int64 i = 0; i < (int64)bf.size(); ++i)
                bf[i] |= bf2[i];
        }

        // broadcast the merged Bloom filter to all devices
        for (uint32 d = 0; d < device_count; ++d)
            d_bloom_filters[d].set_kmers( type, bf );
    }
    else if (device_count > 1)
    {
        log_info(stderr,"  merge filters\n");
        nvbio::vector<nvbio::host_tag,uint32>  bf;
        nvbio::vector<nvbio::host_tag,uint32>  bf2;

        // get the first device Bloom filter
        d_bloom_filters[0].get_kmers( type, bf );

        // merge with all the rest
        for (uint32 d = 1; d < device_count; ++d)
        {
            d_bloom_filters[d].get_kmers( type, bf2 );

            #pragma omp parallel for
            for (int64 i = 0; i < (int64)bf.size(); ++i)
                bf[i] |= bf2[i];
        }

        // broadcast the merged Bloom filter to all devices
        for (uint32 d = 0; d < device_count; ++d)
            d_bloom_filters[d].set_kmers( type, bf );
    }
}


// merge several stats
//
inline
void merged_stats(
    const BloomFilters<nvbio::host_tag>*    h_bloom_filters,
    const uint32                            device_count,
    const BloomFilters<nvbio::device_tag>*  d_bloom_filters,
    nvbio::vector<nvbio::host_tag,uint64>&  stats)
{
    // merge the stats on the host
    nvbio::vector<nvbio::host_tag,uint64> stats2;

    // copy the first
    if (h_bloom_filters)
        stats = h_bloom_filters->stats;
    else
        stats.resize( 10, uint64(0) );

    // and merge with all the rest
    for (uint32 d = 0; d < device_count; ++d)
    {
        d_bloom_filters[d].set_device();

        stats2 = d_bloom_filters[d].stats;

        for (size_t i = 0; i < stats.size(); ++i)
            stats[i] += stats2[i];
    }
}

// helper functor to compute a blocked Bloom filter's occupancy
//
struct block_occupancy_functor
{
    typedef uint4  argument_type;
    typedef double result_type;

    block_occupancy_functor(const uint32 k) : K(float(k)) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    double operator() (const uint4 block) const
    {
        const uint32 used =
            nvbio::popc( block.x ) +
            nvbio::popc( block.y ) +
            nvbio::popc( block.z ) +
            nvbio::popc( block.w );
        return powf( float(used)/128.0f, K );
    }

    const float K;
};

// compute Bloom filter usage statistics
//
template <typename system_tag>
void compute_bloom_filter_stats(
    const BloomFilters<system_tag>& bloom_filters,
    const KmersType                 type,
    const uint32                    K,
    float&                          occupancy,
    float&                          approx_size,
    float&                          fp)
{
    typedef typename nvbio::if_equal<system_tag,nvbio::device_tag,thrust::device_ptr<const uint32>, const uint32*>::type uint_pointer_type;
    typedef typename nvbio::if_equal<system_tag,nvbio::device_tag,thrust::device_ptr<const uint4>,  const uint4*>::type  uint4_pointer_type;

    bloom_filters.set_device();

    const nvbio::vector<system_tag,uint32>& bf = bloom_filters.get_kmers( type );

    const uint32  n_words = uint32( bf.size() );
    const uint32* words   = raw_pointer( bf );

    // compute the number of bits set
    nvbio::vector<system_tag,uint8> temp_storage;

    const uint64 bits_per_word = 32;

    const uint64 bits_set = nvbio::reduce(
        n_words,
        thrust::make_transform_iterator(
            thrust::make_transform_iterator(
                uint_pointer_type( words ), nvbio::popc_functor<uint32>() ),
                nvbio::cast_functor<uint32,uint64>() ),
        thrust::plus<uint64>(),
        temp_storage );

    occupancy   = float( double( bits_set ) / double( n_words * bits_per_word ) );
    approx_size = float( -(double( n_words * bits_per_word ) * std::log( 1.0 - occupancy )) / double(K) );

  #if 0
    // compute the actual false positive rate
    fp = std::pow( occupancy, K );
  #else
    // compute the actual false positive rate - using the block-occupancy
    fp = float( nvbio::reduce(
        n_words / 4,
        thrust::make_transform_iterator(
            uint4_pointer_type( (const uint4*)words ),
            block_occupancy_functor( K ) ),
        thrust::plus<double>(),
        temp_storage ) / double(n_words / 4) );
  #endif
}
