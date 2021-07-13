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

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/utils.h>
#include <nvBowtie/bowtie2/cuda/seed_hit.h>
#include <nvBowtie/bowtie2/cuda/pipeline_states.h>
#include <nvbio/io/alignments.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvbio/basic/cuda/pingpong_queues.h>
#include <nvbio/basic/priority_deque.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/strided_iterator.h>
#include <nvbio/basic/sum_tree.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

//
// Initialize the hit-selection pipeline
//
template <typename ScoringScheme>
void select_init_t(BestApproxScoringPipelineState<ScoringScheme>& pipeline, const ParamsPOD& params)
{
    select_init(
        pipeline.reads.size(),
        pipeline.reads.name_stream(),
        pipeline.reads.name_index(),
        pipeline.hits,
        pipeline.trys,
        pipeline.rseeds,
        params );
}

///@addtogroup nvBowtie
///@{

///@addtogroup Select
///@{

///@addtogroup SelectDetail
///@{

///
/// Prepare for a round of seed extension by selecting the next SA row
///
template <typename BatchType, typename ContextType> __global__ 
void select_kernel(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const ParamsPOD                         params)
{
    __shared__ volatile uint32 sm_broadcast[BLOCKDIM >> 5];
    volatile uint32& warp_broadcast = sm_broadcast[ warp_id() ];

    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= scoring_queues.active_read_count()) return;

    // Fetch the next item from the work queue
    const packed_read read_info = scoring_queues.active_read( thread_id );
    const uint32 read_id   = read_info.read_id;
          uint32 top_flag  = read_info.top_flag;

    NVBIO_CUDA_ASSERT( read_id < read_batch.size() );

    // check whether we finished our loop
    if (context.stop( read_id ))
        return;

    typedef SeedHitDequeArrayDeviceView::reference hit_deque_reference;

    // check whether there's any hits left
    hit_deque_reference hit_deque = hits[ read_id ];
    if (hit_deque.size() == 0)
        return;

    // get a reference to the top element in the hit queue
    SeedHit* hit = const_cast<SeedHit*>( &hit_deque.top() );

    // check whether this hit's range became empty
    if (hit->empty())
    {
        // pop the top of the hit queue
        hit_deque.pop_top();

        // update the hit reference
        if (hit_deque.size() == 0)
            return;

        hit = const_cast<SeedHit*>( &hit_deque.top() );

        // no longer on the top seed, unmask the top flag
        top_flag = 0u;
    }

    // fetch next SA row from the selected hit
    const uint32 sa_pos = hit->pop_front();
    const uint32 r_type = hit->get_readtype() ? 1u : 0u;

    const uint32 slot = alloc( scoring_queues.active_reads.out_size, &warp_broadcast );
    NVBIO_CUDA_ASSERT( slot < scoring_queues.active_reads.in_size );

    // write the output active read
    scoring_queues.active_reads.out_queue[slot] = packed_read( read_id, top_flag );

    // write the output hit
    HitReference<HitQueuesDeviceView> out_hit( scoring_queues.hits, slot );
    out_hit.read_id = read_id;
    out_hit.loc     = sa_pos;
    out_hit.seed    = packed_seed( hit->get_posinread(), hit->get_indexdir(), r_type, top_flag );

    NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_select( read_id ), "select() : selected SA[%u:%u:%u] in slot [%u])\n", sa_pos, hit->get_indexdir(), hit->get_posinread(), slot);
}

template <typename ProbTree>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 randomized_select(ProbTree& prob_tree, SeedHit* hits_data, uint32* rseeds, const uint32 read_id)
{
    for (uint32 i = 0; i < 10; ++i)
    {
      #if 1
        // pick a new random value
        const uint32 ri = 1664525u * rseeds[ read_id ] + 1013904223u;

        // store the new state
        rseeds[ read_id ] = ri;

        // convert to a float
        const float rf = float(ri) / float(0xFFFFFFFFu);
      #else
        // advance the QMC sampler
        const float rf = radical_inverse( rseeds[ read_id ]++ );
      #endif

        // select the next hit
        const uint32 hit_id = sample( prob_tree, rf );
        //NVBIO_CUDA_ASSERT( hit_id < hits.get_size( read_id ) );

        SeedHit* hit = &hits_data[ hit_id ];

        // this should never happen if we had infinite precision, but in practice because of rounding errors prob_tree.sum() might
        // be non-zero even though its leaf values are: let's just check for this and call it a day.
        if (hit->empty() == false)
            return hit_id;
    }
    return 0;
}

///
/// Prepare for a round of seed extension by selecting the next SA row
///
template <typename BatchType, typename ContextType> __global__ 
void rand_select_kernel(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    uint32*                                 rseeds,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const ParamsPOD                         params)
{
    typedef SumTree< float* > ProbTree;

    __shared__ volatile uint32 sm_broadcast[BLOCKDIM >> 5];
    volatile uint32& warp_broadcast = sm_broadcast[ warp_id() ];

    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= scoring_queues.active_read_count()) return;

    // Fetch the next item from the work queue
    const packed_read read_info = scoring_queues.active_read( thread_id );
    const uint32 read_id   = read_info.read_id;
          uint32 top_flag  = read_info.top_flag;

    NVBIO_CUDA_ASSERT( read_id < read_batch.size() );

    // check whether we finished our loop
    if (context.stop( read_id ))
        return;

    typedef SeedHitDequeArrayDeviceView::reference hit_deque_reference;

    // check whether there's any hits left
    hit_deque_reference hit_deque = hits[ read_id ];
    if (hit_deque.size() == 0)
        return;

    ProbTree prob_tree( hit_deque.size(), hit_deque.get_probs() );
    SeedHit* hits_data( hit_deque.get_data() );

    // check if the tree still contains some unexplored entries
    if (prob_tree.sum() <= 0.0f)
    {
        // stop traversal
        hits.erase( read_id );
        return;
    }

    // check if the top hit became empty
    if (top_flag && hits_data[0].empty())
        top_flag = 0;

    const uint32 hit_id = top_flag ? 0 : randomized_select( prob_tree, hits_data, rseeds, read_id );

    // fetch the selected hit
    SeedHit* hit = &hits_data[ hit_id ];

    // this should never happen if we had infinite precision, but in practice because of rounding errors prob_tree.sum() might
    // be non-zero even though its leaf values are: let's just check for this and call it a day.
    if (hit->empty())
    {
        // stop traversal
        hits.erase( read_id );
        return;
    }

    // fetch next SA row from the selected hit
    const uint32 sa_pos = hit->pop_front();
    const uint32 r_type = hit->get_readtype() ? 1u : 0u;

    // if the range for the selected seed hit became empty, zero out its probability
    if (hit->empty())
        prob_tree.set( hit_id, 0.0f );

    const uint32 slot = alloc( scoring_queues.active_reads.out_size, &warp_broadcast );
    NVBIO_CUDA_ASSERT( slot < scoring_queues.active_reads.in_size );

    // write the output active read
    scoring_queues.active_reads.out_queue[slot] = packed_read( read_id, top_flag );

    /*
    typedef ReadHitsBinder<ScoringQueuesDeviceView>    read_hits_binder;
    typedef typename read_hits_binder::reference       hit_reference;

    // create a hit binder for this read
    read_hits_binder dst_read_hits( scoring_queues, slot );

    // copy from parent
    dst_read_hits.set_read_info( packed_read( read_id, top_flag ) );

    // bind the hit
    dst_read_hits.bind_hit( 0u, slot );    // write the output hit
    */

    HitReference<HitQueuesDeviceView> out_hit( scoring_queues.hits, slot );
    out_hit.read_id = read_id;
    out_hit.loc     = sa_pos;
    out_hit.seed    = packed_seed( hit->get_posinread(), hit->get_indexdir(), r_type, top_flag );

    NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_select( read_id ), "select() : selected hit[%u], SA[%u:%u:%u] in slot [%u])\n", hit_id, sa_pos, hit->get_indexdir(), hit->get_posinread(), slot);
}

///
/// Prepare for a round of seed extension by selecting a set of up
/// to 'n_multi' next SA rows.
/// For each read in the input queue, this kernel generates:
///     1. one or zero output reads, in the main output read queue,
///     2. zero to 'n_multi' SA rows. These are made of three entries,
///        one in 'loc_queue', identifying the corresponding SA index,
///        one in 'seed_queue', storing information about the seed hit,
///        and one in 'parent_queue', storing the index of the "parent"
///        read in the output queue (i.e. the slot where the read is
///        is being stored)
///
template <typename BatchType, typename ContextType> __global__ 
void select_multi_kernel(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const uint32                            n_multi,
    const ParamsPOD                         params)
{
    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= scoring_queues.active_read_count()) return;

    const packed_read read_info = scoring_queues.active_read( thread_id );
    const uint32 read_id   = read_info.read_id;
          uint32 top_flag  = read_info.top_flag;

    NVBIO_CUDA_ASSERT( read_id < read_batch.size() );

    // check whether we finished our loop
    if (context.stop( read_id ))
        return;

    typedef SeedHitDequeArrayDeviceView::reference hit_deque_reference;

    // check whether there's any hits left
    hit_deque_reference hit_deque = hits[ read_id ];
    if (hit_deque.size() == 0)
        return;

    typedef ReadHitsBinder<ScoringQueuesDeviceView>    read_hits_binder;
    typedef typename read_hits_binder::reference       hit_reference;

    // create a hit binder for this read
    read_hits_binder dst_read_hits( scoring_queues );

    uint32 output_lane     = uint32(-1);
    uint32 n_selected_hits = 0u;

#if 0
    for (uint32 i = 0; i < n_multi; ++i)
    {
        // get a reference to the top element in the hit queue
        SeedHit* hit = const_cast<SeedHit*>( &hit_deque.top() );

        // check whether this hit's range became empty
        if (hit->empty())
        {
            // pop the top of the hit queue
            hit_deque.pop_top();

            // update the hit reference
            if (hit_deque.size() == 0)
                break;
            else
            {
                hit = const_cast<SeedHit*>( &hit_deque.top() );

                // no longer on the top seed, unmask the top flag
                top_flag = 0u;
            }
        }

        if (output_lane == uint32(-1))
        {
            // grab a slot in the output read queue - we'll call this the 'output_lane'
            output_lane = atomicAdd( scoring_queues.active_reads.out_size, 1u );
            NVBIO_CUDA_ASSERT( output_lane <  scoring_queues.active_reads.in_size );

            // bind the read to its new location in the output queue
            dst_read_hits.bind( output_lane );
        }

        // fetch next SA row from the selected hit
        const uint32 sa_pos = hit->pop_front();
        const uint32 r_type = hit->get_readtype() ? 1u : 0u;

        // grab an output slot for writing our hit
        const uint32 slot = atomicAdd( scoring_queues.hits_pool, 1u );

        // bind the hit
        dst_read_hits.bind_hit( n_selected_hits, slot );

        hit_reference out_hit = dst_read_hits[ n_selected_hits ];
        out_hit.read_id = read_id;
        out_hit.loc     = sa_pos;
        out_hit.seed    = packed_seed( hit->get_posinread(), hit->get_indexdir(), r_type, top_flag );

        ++n_selected_hits;
        NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_select( read_id ), "select() : selected SA[%u:%u:%u] in slot [%u], output-lane[%u:%u])\n", sa_pos, hit->get_indexdir(), hit->get_posinread(), slot, output_lane, i);
    }
#else
    // NOTE: the following loop relies on fragile, unsupported
    // warp-synchronous logic.
    // To make the work queue allocations work correctly, we need
    // to make sure that all threads within each warp keep executing
    // the loop until the very last thread is done.
    // As the compiler takes true branches first, we achieve this
    // by keeping an active flag and looping until it's true.
    __shared__ volatile uint32 sm_broadcast1[BLOCKDIM >> 5];
    __shared__ volatile uint32 sm_broadcast2[BLOCKDIM >> 5];
    volatile uint32& warp_broadcast1 = sm_broadcast1[ warp_id() ];
    volatile uint32& warp_broadcast2 = sm_broadcast2[ warp_id() ];

    bool active = true;

    for (uint32 i = 0; i < n_multi && __any(active); ++i)
    {
        SeedHit* hit = NULL;

        // check whether this thread is actually active, or idling around
        if (active)
        {
            // get a reference to the top element in the hit queue
            hit = const_cast<SeedHit*>( &hit_deque.top() );

            // check whether this hit's range became empty
            if (hit->empty())
            {
                // pop the top of the hit queue
                hit_deque.pop_top();

                // update the hit reference
                if (hit_deque.size() == 0)
                {
                    active = false;
                    // NOTE: it would be tempting to just break here, but
                    // this would break the warp-synchronous allocation
                    // below.
                }
                else
                {
                    hit = const_cast<SeedHit*>( &hit_deque.top() );

                    // no longer on the top seed, unmask the top flag
                    top_flag = 0u;
                }
            }
        }

        // is this thread still active?
        if (active)
        {
            if (output_lane == uint32(-1))
            {
                // grab a slot in the output read queue - we'll call this the 'output_lane'
                output_lane = alloc( scoring_queues.active_reads.out_size, &warp_broadcast1 );

                // bind the read to its new location in the output queue
                dst_read_hits.bind( output_lane );

                NVBIO_CUDA_ASSERT( output_lane < scoring_queues.active_reads.in_size );
            }

            // fetch next SA row from the selected hit
            const uint32 sa_pos = hit->pop_front();
            const uint32 r_type = hit->get_readtype() ? 1u : 0u;

            // grab an output slot for writing our hit
            const uint32 slot = alloc( scoring_queues.hits_pool, &warp_broadcast2 );

            // bind the hit
            dst_read_hits.bind_hit( n_selected_hits, slot );

            hit_reference out_hit = dst_read_hits[ n_selected_hits ];
            out_hit.read_id = read_id;
            out_hit.loc     = sa_pos;
            out_hit.seed    = packed_seed( hit->get_posinread(), hit->get_indexdir(), r_type, top_flag );

            ++n_selected_hits;
            NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_select( read_id ), "select() : selected SA[%u:%u:%u] in slot [%u], output-lane[%u:%u])\n", sa_pos, hit->get_indexdir(), hit->get_posinread(), slot, output_lane, i);
        }
    }
#endif

    // write the output read info.
    // NOTE: this must be done at the very end, because only now we know the final state of the top_flag.
    if (output_lane != uint32(-1))
    {
        // setup the output-lane's read-info
        dst_read_hits.set_read_info( packed_read( read_id, top_flag ) );

        // set the number of hits
        dst_read_hits.resize( n_selected_hits );
    }
}

///
/// Prepare for a round of seed extension by selecting a set of up
/// to 'n_multi' next SA rows.
/// For each read in the input queue, this kernel generates:
///     1. one or zero output reads, in the main output read queue,
///     2. zero to 'n_multi' SA rows. These are made of three entries,
///        one in 'loc_queue', identifying the corresponding SA index,
///        one in 'seed_queue', storing information about the seed hit,
///        and one in 'parent_queue', storing the index of the "parent"
///        read in the output queue (i.e. the slot where the read is
///        is being stored)
///
template <typename BatchType, typename ContextType> __global__ 
void rand_select_multi_kernel(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    uint32*                                 rseeds,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const uint32                            n_multi,
    const ParamsPOD                         params)
{
    typedef SumTree< float* > ProbTree;

    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= scoring_queues.active_read_count()) return;

    const packed_read read_info = scoring_queues.active_read( thread_id );
    const uint32 read_id   = read_info.read_id;
          uint32 top_flag  = read_info.top_flag;

    NVBIO_CUDA_ASSERT( read_id < read_batch.size() );

    // check whether we finished our loop
    if (context.stop( read_id ))
        return;

    typedef SeedHitDequeArrayDeviceView::reference hit_deque_reference;

    // check whether there's any hits left
    hit_deque_reference hit_deque = hits[ read_id ];
    if (hit_deque.size() == 0)
        return;

    ProbTree prob_tree( hit_deque.size(), hit_deque.get_probs() );
    SeedHit* hits_data( hit_deque.get_data() );

    typedef ReadHitsBinder<ScoringQueuesDeviceView>    read_hits_binder;
    typedef typename read_hits_binder::reference       hit_reference;

    // create a hit binder for this read
    read_hits_binder dst_read_hits( scoring_queues );

    uint32 output_lane     = uint32(-1);
    uint32 n_selected_hits = 0u;

    for (uint32 i = 0; i < n_multi; ++i)
    {
        // check if the tree still contains some unexplored entries
        if (prob_tree.sum() <= 0.0f)
        {
            // stop traversal
            hits.erase( read_id );
            break;
        }

        // check if the top hit became empty
        if (top_flag && hits_data[0].empty())
            top_flag = 0;

        const uint32 hit_id = top_flag ? 0 : randomized_select( prob_tree, hits_data, rseeds, read_id );

        // fetch the selected hit
        SeedHit* hit = &hits_data[ hit_id ];

        // this should never happen if we had infinite precision, but in practice because of rounding errors prob_tree.sum() might
        // be non-zero even though its leaf values are: let's just check for this and call it a day.
        if (hit->empty())
        {
            // stop traversal
            hits.erase( read_id );
            continue;
        }

        // fetch next SA row from the selected hit
        const uint32 sa_pos = hit->pop_front();
        const uint32 r_type = hit->get_readtype() ? 1u : 0u;

        // if the range for the selected seed hit became empty, zero out its probability
        if (hit->empty())
            prob_tree.set( hit_id, 0.0f );

        if (output_lane == uint32(-1))
        {
            // grab a slot in the output read queue - we'll call this the 'output_lane'
            output_lane = atomicAdd( scoring_queues.active_reads.out_size, 1u );
            NVBIO_CUDA_ASSERT( output_lane < scoring_queues.active_reads.in_size );

            // bind the read to its new location in the output queue
            dst_read_hits.bind( output_lane );
        }

        // grab an output slot for writing our hit
        const uint32 slot = atomicAdd( scoring_queues.hits_pool, 1u );
        NVBIO_CUDA_ASSERT( slot < scoring_queues.hits.read_id.size() );

        // bind the hit
        dst_read_hits.bind_hit( n_selected_hits, slot );

        hit_reference out_hit = dst_read_hits[ n_selected_hits ];
        out_hit.read_id = read_id;
        out_hit.loc     = sa_pos;
        out_hit.seed    = packed_seed( hit->get_posinread(), hit->get_indexdir(), r_type, top_flag );

        ++n_selected_hits;
        NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_select( read_id ), "select() : selected SA[%u:%u:%u] in slot [%u], output-lane[%u:%u])\n", sa_pos, hit->get_indexdir(), hit->get_posinread(), slot, output_lane, i);
    }

    // write the output read info.
    // NOTE: this must be done at the very end, because only now we know the final state of the top_flag.
    if (output_lane != uint32(-1))
    {
        // setup the output-lane's read-info
        dst_read_hits.set_read_info( packed_read( read_id, top_flag ) );

        // set the number of hits
        dst_read_hits.resize( n_selected_hits );
    }
}

///@}  // group SelectDetail
///@}  // group Select
///@}  // group nvBowtie

//
// Prepare for a round of seed extension by selecting the next SA row
//
template <typename BatchType, typename ContextType>
void select(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const ParamsPOD                         params)
{
    const int blocks = (scoring_queues.active_reads.in_size + BLOCKDIM-1) / BLOCKDIM;

    select_kernel<<<blocks, BLOCKDIM>>>(
        read_batch,
        hits,
        context,
        scoring_queues,
        params );
}

//
// Prepare for a round of seed extension by selecting the next SA row
//
template <typename BatchType, typename ContextType>
void rand_select(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    uint32*                                 rseeds,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const ParamsPOD                         params)
{
    const int blocks = (scoring_queues.active_reads.in_size + BLOCKDIM-1) / BLOCKDIM;

    rand_select_kernel<<<blocks, BLOCKDIM>>>(
        read_batch,
        hits,
        rseeds,
        context,
        scoring_queues,
        params );
}

//
// Prepare for a round of seed extension by selecting a set of up
// to 'n_multi' next SA rows.
// For each read in the input queue, this kernel generates:
//     1. one or zero output reads, in the main output read queue,
//     2. zero to 'n_multi' SA rows. These are made of three entries,
//        one in 'loc_queue', identifying the corresponding SA index,
//        one in 'seed_queue', storing information about the seed hit,
//        and one in 'parent_queue', storing the index of the "parent"
//        read in the output queue (i.e. the slot where the read is
//        is being stored)
//
template <typename BatchType, typename ContextType>
void select_multi(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const uint32                            n_multi,
    const ParamsPOD                         params)
{
    const int blocks = (scoring_queues.active_reads.in_size + BLOCKDIM-1) / BLOCKDIM;

    select_multi_kernel<<<blocks, BLOCKDIM>>>(
        read_batch,
        hits,
        context,
        scoring_queues,
        n_multi,
        params );
}

//
// Prepare for a round of seed extension by selecting a set of up
// to 'n_multi' next SA rows.
// For each read in the input queue, this kernel generates:
//     1. one or zero output reads, in the main output read queue,
//     2. zero to 'n_multi' SA rows. These are made of three entries,
//        one in 'loc_queue', identifying the corresponding SA index,
//        one in 'seed_queue', storing information about the seed hit,
//        and one in 'parent_queue', storing the index of the "parent"
//        read in the output queue (i.e. the slot where the read is
//        is being stored)
//
template <typename BatchType, typename ContextType>
void rand_select_multi(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    uint32*                                 rseeds,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const uint32                            n_multi,
    const ParamsPOD                         params)
{
    const int blocks = (scoring_queues.active_reads.in_size + BLOCKDIM-1) / BLOCKDIM;

    rand_select_multi_kernel<<<blocks, BLOCKDIM>>>(
        read_batch,
        hits, rseeds,
        context,
        scoring_queues,
        n_multi,
        params );
}

//
// Prepare for a round of seed extension by selecting the next SA rows for each read
//
template <typename BatchType, typename ContextType>
void select(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    uint32*                                 rseeds,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const uint32                            n_multi,
    const ParamsPOD                         params)
{
    //
    // Dispatch the call to the proper version based on whether we are performing
    // single-hit or batched-hit selection, and whether we use randomization or not.
    //
    if (params.randomized)
    {
        if (n_multi > 1)
        {
            // batched-hit selection
            rand_select_multi(
                read_batch,
                hits, rseeds,
                context,
                scoring_queues,
                n_multi,
                params );
        }
        else
        {
            // randomized single-hit selection
            rand_select(
                read_batch,
                hits, rseeds,
                context,
                scoring_queues,
                params );
        }
    }
    else
    {
        if (n_multi > 1)
        {
            // batched-hit selection
            select_multi(
                read_batch,
                hits,
                context,
                scoring_queues,
                n_multi,
                params );
        }
        else
        {
            // non-randomized single-hit selection
            select(
                read_batch,
                hits,
                context,
                scoring_queues,
                params );
        }
    }
}

//
// Prepare for a round of seed extension by selecting the next SA rows for each read
//
template <typename ScoringScheme, typename ContextType>
void select_t(
    const ContextType                                       context,
    const BestApproxScoringPipelineState<ScoringScheme>&    pipeline,
    const ParamsPOD                                         params)
{
    select(
        pipeline.reads,
        pipeline.hits,
        pipeline.rseeds,
        context,
        pipeline.scoring_queues,
        pipeline.n_hits_per_read,
        params );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
