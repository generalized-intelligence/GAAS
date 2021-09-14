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

//#define NVBIO_CUDA_DEBUG
//#define NVBIO_CUDA_ASSERTS

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvBowtie/bowtie2/cuda/scoring_queues.h>


namespace nvbio {
namespace bowtie2 {
namespace cuda {

namespace { // anonymous namespace

//
// Setup the contents of a ReadHitsIndex object through a ReadHitsIndexDeviceView.
// Implicitly also testing ReadHitsIndexDeviceView::reference.
//
__global__
void setup_read_hits_index_kernel(ReadHitsIndexDeviceView read_hits_index, const uint32 n_reads, uint32* error)
{
    const uint32 read_index = threadIdx.x;
    if (read_index >= n_reads)
        return;

    // fetch the hits bound to this read
    ReadHitsIndexDeviceView::reference hit_indices( read_hits_index, read_index );

    // bound 2 hits to this read
    hit_indices.resize( 2u );
    hit_indices[0] = read_index * 2u + 0u;
    hit_indices[1] = read_index * 2u + 1u;
}

//
// Check the contents of a ReadHitsIndex object through a ReadHitsIndexDeviceView.
// Implicitly also testing ReadHitsIndexDeviceView::reference.
//
__global__
void check_read_hits_index_kernel(ReadHitsIndexDeviceView read_hits_index, const uint32 n_reads, uint32* error)
{
    const uint32 read_index = threadIdx.x;
    if (read_index >= n_reads)
        return;

    // fetch the hits bound to this read
    ReadHitsIndexDeviceView::reference hit_indices( read_hits_index, read_index );

    // make sure the number of hits bound to this read match our expectations
    if (hit_indices.size() != 2u)
    {
        *error = 1;
        return;
    }

    // make sure their values match our expectations
    if ((hit_indices[0] != read_index * 2u + 0u) ||
        (hit_indices[1] != read_index * 2u + 1u))
    {
        *error = 2;
        return;
    }

    // make sure direct indexing work
    if ((hit_indices[0] != read_hits_index( read_index, 0u )) ||
        (hit_indices[1] != read_hits_index( read_index, 1u )))
    {
        *error = 3;
        return;
    }
}

#pragma hd_warning_disable 
template <typename HitQueuesType>
NVBIO_HOST_DEVICE
void set_hit(HitReference<HitQueuesType> hit, const uint32 read_id)
{
    hit.read_id         = read_id;
    hit.ssa             = 0;
    hit.loc             = 1;
    hit.score           = 2;
    hit.sink            = 3;
    hit.opposite_loc    = 4;
    hit.opposite_score  = 5;
    hit.opposite_sink   = 6;
}
#pragma hd_warning_disable 
template <typename HitQueuesType>
NVBIO_HOST_DEVICE
bool check_hit(HitReference<HitQueuesType> hit, const uint32 read_id)
{
    return (hit.read_id         != read_id ||
            hit.ssa             != 0 ||
            hit.loc             != 1 ||
            hit.score           != 2 ||
            hit.sink            != 3 ||
            hit.opposite_loc    != 4 ||
            hit.opposite_score  != 5 ||
            hit.opposite_sink   != 6);
}

//
// Setup the contents of a HitQueues object through a HitQueuesDeviceView.
// Implicitly also testing HitReference.
//
__global__
void setup_hit_queues_kernel(HitQueuesDeviceView hit_queues, const uint32 n_hits, uint32* error)
{
    const uint32 hit_index = threadIdx.x;
    if (hit_index >= n_hits)
        return;

    // take a reference to this hit
    HitReference<HitQueuesDeviceView> hit = hit_queues[ hit_index ];

    // set it up
    set_hit( hit, hit_index ); // assign a unique index as the read_id, for testing purposes
}
//
// Check the contents of a HitQueues object through a HitQueuesDeviceView.
// Implicitly also testing HitReference.
//
__global__
void check_hit_queues_kernel(HitQueuesDeviceView hit_queues, const uint32 n_hits, uint32* error)
{
    const uint32 hit_index = threadIdx.x;
    if (hit_index >= n_hits)
        return;

    // take a reference to this hit
    HitReference<HitQueuesDeviceView> hit = hit_queues[ hit_index ];

    // check that its values match our expectations
    if (check_hit( hit, hit_index ))
        *error = 1;
}

//
// Test the ReadHitsReference object.
// At this point the read_hits_index and the hit_queues have been already setup, and we just need to make sure
// we can dereference them correctly through ReadHitsReference proxies.
//
__global__
void test_read_hits_ref_kernel(ScoringQueuesDeviceView queues, const uint32 n_reads, uint32* error)
{
    const uint32 read_index = threadIdx.x;
    if (read_index >= n_reads)
        return;

    typedef ReadHitsReference<ScoringQueuesDeviceView> read_hits_reference;
    typedef typename read_hits_reference::reference    hit_reference;

    // create a reference to the sequence of hits bound to this read
    read_hits_reference read_hits( queues, read_index );

    //read_hits.set_read_info( packed_read( read_index ) );

    // check that we have 2 hits for each read
    if (read_hits.size() != 2u)
    {
        *error = 1;
        return;
    }

    // make sure the hits bound to this read match our expectations
    {
        // examine the first hit
        hit_reference hit = read_hits[0];

        if (check_hit( hit, read_index*2u + 0u ))
        {
            *error = 2;
            return;
        }
    }
    {
        // examine the second hit
        hit_reference hit = read_hits[1];

        if (check_hit( hit, read_index*2u + 1u ))
        {
            *error = 3;
            return;
        }
    }
}

//
// Test emission of new hits using a ReadHitsBinder
//
__global__
void test_read_binder_kernel(ScoringQueuesDeviceView queues, const uint32 n_reads, uint32* error)
{
    const uint32 read_index = threadIdx.x;
    if (read_index >= n_reads)
        return;

    typedef ReadHitsReference<ScoringQueuesDeviceView> read_hits_reference;
    typedef ReadHitsBinder<ScoringQueuesDeviceView>    read_hits_binder;
    typedef typename read_hits_reference::reference    hit_reference;

    // create a hit binder for this read
    packed_read       src_read_info = queues.active_read( read_index );
    read_hits_binder  dst_read_hits( queues );

    // drop all odd reads
    if ((read_index & 1) == 1)
        return;

    // get a new slot for the read
    const uint32 dst_slot = atomicAdd( queues.active_reads.out_size, 1u );

    // bind the read to its new location in the output queue
    dst_read_hits.bind( dst_slot );

    // copy from parent
    dst_read_hits.set_read_info( src_read_info );

    // set the number of hits
    dst_read_hits.resize( 1u );

    // get a new slot for a hit
    const uint32 hit_slot = atomicAdd( queues.hits_pool, 1u );

    // bind the hit
    dst_read_hits.bind_hit( 0u, hit_slot );

    // and set it
    set_hit( dst_read_hits[0], read_index );
}

} // anonymous namespace

void test_scoring_queues()
{
    const uint32 n_hits          = 100;
    const uint32 n_reads         = 50;
    const uint32 n_hits_per_read = 2;

    ScoringQueues queues;

    queues.resize( n_reads, n_hits, true );

    // test ReadHitsIndex interfaces
    ReadHitsIndex& read_hits_index = queues.hits_index;
    {
        log_info( stderr, "test ReadHitsIndex... started\n" );

        read_hits_index.setup( n_hits_per_read, n_reads );

        thrust::device_vector<uint32> error(1,0);

        setup_read_hits_index_kernel<<<1,128>>>( read_hits_index.device_view(), n_reads, device_view( error ) );
        check_read_hits_index_kernel<<<1,128>>>( read_hits_index.device_view(), n_reads, device_view( error ) );
        cudaThreadSynchronize();

        const uint32 error_code = error[0];
        if (error_code)
        {
            log_error( stderr, "test ReadHitsIndex... failed! (error code %u)\n", error_code );
            exit(1);
        }

        log_info( stderr, "test ReadHitsIndex... done\n" );
    }

    // test HitQueues interfaces
    HitQueues& hit_queues = queues.hits;
    {
        log_info( stderr, "test HitQueues... started\n" );

        // check host-side access
        for (uint32 i = 0; i < 10; ++i)
        {
            HitReference<HitQueues> hit( hit_queues, i );

            set_hit( hit, i );
        }
        for (uint32 i = 0; i < 10; ++i)
        {
            HitReference<HitQueues> hit( hit_queues, i );

            if (check_hit( hit, i ))
            {
                log_error( stderr, "test HitQueues... failed! (host-side referencing)\n" );
                exit(1);
            }
        }

        thrust::device_vector<uint32> error(1,0);

        setup_hit_queues_kernel<<<1,128>>>( hit_queues.device_view(), n_hits, device_view( error ) );
        check_hit_queues_kernel<<<1,128>>>( hit_queues.device_view(), n_hits, device_view( error ) );
        cudaThreadSynchronize();

        const uint32 error_code = error[0];
        if (error_code)
        {
            log_error( stderr, "test HitQueues... failed! (error code %u)\n", error_code );
            exit(1);
        }

        log_info( stderr, "test HitQueues... done\n" );
    }

    // test ReadHitsReference object
    {
        log_info( stderr, "test ReadHitsReference... started\n" );

        thrust::device_vector<uint32> error(1,0);

        test_read_hits_ref_kernel<<<1,128>>>( queues.device_view(), n_reads, device_view( error ) );
        cudaThreadSynchronize();

        const uint32 error_code = error[0];
        if (error_code)
        {
            log_error( stderr, "test ReadHitsReference... failed! (error code %u)\n", error_code );
            exit(1);
        }

        log_info( stderr, "test ReadHitsReference... done\n" );
    }

    // test ReadHitsBinder object
    {
        log_info( stderr, "test ReadHitsBinder... started\n" );

        thrust::device_vector<uint32> error(1,0);

        // clear output queues
        queues.clear_output();

        test_read_binder_kernel<<<1,128>>>( queues.device_view(), n_reads, device_view( error ) );
        cudaThreadSynchronize();

        const uint32 error_code = error[0];
        if (error_code)
        {
            log_error( stderr, "test ReadHitsBinder... failed! (error code %u)\n", error_code );
            exit(1);
        }

        log_info( stderr, "test ReadHitsBinder... done\n" );
    }
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
