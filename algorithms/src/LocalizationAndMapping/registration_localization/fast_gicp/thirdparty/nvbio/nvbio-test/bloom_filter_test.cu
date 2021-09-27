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

// wavelet_test.cu
//

#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/bloom_filter.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/basic/omp.h>
#include <stdio.h>
#include <stdlib.h>

namespace nvbio {

static const uint32 FILTER_K = 7;
static const uint32 ITEMS_PER_THREAD = 100;

struct hash_functor1
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint64 operator() (const uint64 kmer) const { return hash( kmer ); }
};
struct hash_functor2
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint64 operator() (const uint64 kmer) const { return hash2( kmer ); }
};


struct bloom_filter_inserter
{
    typedef uint64 argument_type;
    typedef uint8  return_type;

    bloom_filter_inserter(const uint64 _size, uint32* _words) : filter( FILTER_K, _size, (uint64_2*)_words ) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void operator() (const uint64 i)
    {
        for (uint64 j = i * ITEMS_PER_THREAD; j < (i+1)*ITEMS_PER_THREAD; ++j)
            filter.insert(j);
    }

    mutable blocked_bloom_filter< hash_functor1, hash_functor2, uint64_2*> filter;
};

struct bloom_filter_lookup
{
    typedef uint64 argument_type;
    typedef uint8  return_type;

    bloom_filter_lookup(const uint64 _size, const uint32* _words) : filter( FILTER_K, _size, (const uint4*)_words ) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint8 operator() (const uint64 i) const
    {
        uint8 r = 0;

        for (uint64 j = i * ITEMS_PER_THREAD; j < (i+1)*ITEMS_PER_THREAD; ++j)
            r |= filter[j];

        return r;
    }

    blocked_bloom_filter< hash_functor1, hash_functor2, cuda::load_pointer<uint4,cuda::LOAD_LDG> > filter;
};

template <typename system_tag>
void do_bloom_filter_test(const uint64 size, const uint32 N)
{
    log_info(stderr, "  %s test\n", equal<system_tag,device_tag>() ? "gpu" : "cpu" );

    const uint64 n_words = util::divide_ri( size, uint64(32) );
    nvbio::vector<system_tag,uint32> filter( n_words );
    nvbio::vector<system_tag,uint8>  temp(N);

    Timer timer;
    timer.start();

    nvbio::for_each<system_tag>(
        N,
        thrust::make_counting_iterator<uint64>(0u),
        bloom_filter_inserter(
            size,
            raw_pointer( filter ) ) );

    cudaDeviceSynchronize();

    timer.stop();
    log_info(stderr, "    insertion: %.1f M/s\n", 1.0e-6f * float(N*ITEMS_PER_THREAD)/timer.seconds());

    timer.start();

    nvbio::transform<system_tag>(
        N,
        thrust::make_counting_iterator<uint64>(0u),
        temp.begin(),
        bloom_filter_lookup(
            size,
            raw_pointer( filter ) ) );

    cudaDeviceSynchronize();

    timer.stop();
    log_info(stderr, "    lookup: %.1f M/s\n", 1.0e-6f * float(N*ITEMS_PER_THREAD)/timer.seconds());
}

int bloom_filter_test(int argc, char* argv[])
{
    uint64 size = 4*1024*1024; // 4MB
    uint32 N    = 100000;

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp( argv[i], "-size" ) == 0)
            size = uint64( atoi( argv[++i] ) ) * uint64( 8*1024u*1024u );
        else if (strcmp( argv[i], "-queries" ) == 0)
            N = atoi( argv[++i] )*1000;
    }

    try
    {
        log_info(stderr, "bloom filter test... started (%.1f MB)\n", float(size) / float(8*1024*1024));

        // set the number of OpenMP threads
        omp_set_num_threads( omp_get_num_procs() );
        log_verbose(stderr, "  cpu: %u threads\n", omp_get_num_procs() );

        do_bloom_filter_test<host_tag>( size, N );
        do_bloom_filter_test<device_tag>( size, N );

        log_info(stderr, "bloom filter test... done\n");
    }
    catch (...)
    {
        log_error(stderr, "error: unknown exception caught!\n");
    }
    return 0;
}

} // namespace nvbio
