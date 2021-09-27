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

///
///\file reduce_inl.h
///

#pragma once

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/seed_hit.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvBowtie/bowtie2/cuda/alignment_utils.h>
#include <nvBowtie/bowtie2/cuda/pipeline_states.h>
#include <nvbio/io/alignments.h>
#include <nvbio/io/utils.h>
#include <nvbio/basic/exceptions.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

namespace detail {

///@addtogroup nvBowtie
///@{

///@addtogroup Reduce
///@{

///@addtogroup ReduceDetail
///@{

///
///
/// Reduce the list of scores associated to each read in the input queue to find
/// the best 2 alignments.
///
/// \details
/// this kernel takes a batch of extension results (one per active read, indexed by a sorting id)
/// and 'reduces' them with best 2 results found so far.
/// The kernel is parameterized by a templated context which can take further actions upon
/// updates to the best or second best scores, as well as bailing-out on failures.
///
/// \param context             the template context
/// \param pipeline            the pipeline state
/// \param params              algorithm parameters
///
template <typename ScoringScheme, typename PipelineType, typename ReduceContext> __global__ 
void score_reduce_kernel(
    const ReduceContext     context,
          PipelineType      pipeline,
    const ParamsPOD         params)
{
    typedef ScoringQueuesDeviceView                 scoring_queues_type;
    typedef ReadHitsReference<scoring_queues_type>  read_hits_type;
    typedef typename read_hits_type::reference      hit_type;

    scoring_queues_type& scoring_queues = pipeline.scoring_queues;

    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= scoring_queues.active_read_count()) return;

    // fetch the active read corresponding to this thread
    read_hits_type read_hits( scoring_queues, thread_id );

    const uint32 read_id = read_hits.read_info().read_id;

    // fetch the best alignments
    io::BestAlignments best(
        pipeline.best_alignments[ read_id ],
        pipeline.best_alignments[ read_id + pipeline.best_stride ] );

    // setup the read stream
    const uint2  read_range = pipeline.reads.get_range(read_id);
    const uint32 read_len   = read_range.y - read_range.x;

    const uint32 count = read_hits.size();

    for (uint32 i = 0; i < count; ++i)
    {
        hit_type hit = read_hits[i];
        NVBIO_CUDA_DEBUG_ASSERT( hit.read_id == read_id, "reduce hit[%u][%u]: expected id %u, got: %u (slot: %u)\n", thread_id, i, read_id, hit.read_id, scoring_queues.hits_index( thread_id, i ));

        const packed_seed seed_info = hit.seed;
        const uint32 read_rc        = seed_info.rc;
        const uint32 top_flag       = seed_info.top_flag;

        const  int32 score          = hit.score;
        const uint32 g_pos          = hit.loc;

        // skip locations that we have already visited without paying the extension attempt
        if ((read_rc == best.m_a1.m_rc && g_pos == best.m_a1.m_align) ||
            (read_rc == best.m_a2.m_rc && g_pos == best.m_a2.m_align))
            continue;

        if (score > best.m_a1.score())
        {
            context.best_score( read_id, params );

            // set the first best score
            best.m_a2 = best.m_a1;
            best.m_a1 = io::Alignment( g_pos, 0u, score, read_rc );
            NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_reduce( read_id ), "update best:   (parent[%u:%u])\n  score[%d], rc[%u], pos[%u]\n", thread_id, i, score, read_rc, g_pos );
        }
        else if ((score > best.m_a2.score()) && io::distinct_alignments( best.m_a1.m_align, best.m_a1.m_rc, g_pos, read_rc, read_len/2 ))
        {
            context.second_score( read_id, params );

            // set the second best score
            best.m_a2 = io::Alignment( g_pos, 0u, score, read_rc );
            NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_reduce( read_id ), "update second: (parent[%u:%u])\n  score[%d], rc[%u], pos[%u]\n", thread_id, i, score, read_rc, g_pos );
        }
        else if (context.failure( i, read_id, top_flag, params ))
        {
            // stop traversal
            pipeline.hits.erase( read_id );

            // NOTE: we keep using the score entries rather than breaking here,
            // as we've already gone through the effort of computing them, and
            // this reduction is only a minor amount of work.
        }
    }

    // report best alignments
    pipeline.best_alignments[ read_id ]                        = best.m_a1;
    pipeline.best_alignments[ read_id + pipeline.best_stride ] = best.m_a2;
}

template <typename ReduceContext>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void update_best(
    const uint32                read_id,
    io::BestPairedAlignments&   best_pairs,
    const io::PairedAlignments& pair,
    ReduceContext&              context,
    const ParamsPOD             params)
{
    context.best_score( read_id, params );

    // the old best becomes second-best, and the second-best gets updated
    best_pairs.m_a2 = best_pairs.m_a1;
    best_pairs.m_o2 = best_pairs.m_o1;
    best_pairs.m_a1 = pair.m_a;
    best_pairs.m_o1 = pair.m_o;
    NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_reduce( read_id ), "update best (anchor[%u]):\n  1. score[%d], rc[%u], pos[%u]\n  2. score[%d], rc[%u], pos[%u,%u]\n", pair.m_a.mate(), pair.m_a.score(), pair.m_a.is_rc(), pair.m_a.alignment(), pair.m_o.score(), pair.m_o.is_rc(), pair.m_o.alignment(), pair.m_o.alignment() + pair.m_o.sink() );
}

template <typename ReduceContext>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void replace_best(
    const uint32                read_id,
    io::BestPairedAlignments&   best_pairs,
    const io::PairedAlignments& pair,
    ReduceContext&              context,
    const ParamsPOD             params)
{
    context.best_score( read_id, params );

    best_pairs.m_a1 = pair.m_a;
    best_pairs.m_o1 = pair.m_o;
    NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_reduce( read_id ), "update best (anchor[%u]):\n  1. score[%d], rc[%u], pos[%u]\n  2. score[%d], rc[%u], pos[%u,%u]\n", pair.m_a.mate(), pair.m_a.score(), pair.m_a.is_rc(), pair.m_a.alignment(), pair.m_o.score(), pair.m_o.is_rc(), pair.m_o.alignment, pair.m_o.alignment() + pair.m_o.sink() );
}

template <typename ReduceContext>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void update_second(
    const uint32                read_id,
    io::BestPairedAlignments&   best_pairs,
    const io::PairedAlignments& pair,
    ReduceContext&              context,
    const ParamsPOD             params)
{
    context.second_score( read_id, params );

    best_pairs.m_a2 = pair.m_a;
    best_pairs.m_o2 = pair.m_o;
    NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_reduce( read_id ), "update second (anchor[%u]):\n  1. score[%d], rc[%u], pos[%u]\n  2. score[%d], rc[%u], pos[%u,%u]\n", pair.m_a.mate(), pair.m_a.score(), pair.m_a.is_rc(), pair.m_a.alignment(), pair.m_o.score(), pair.m_o.is_rc(), pair.m_o.alignment, pair.m_o.alignment() + pair.m_o.sink() );
}

template <typename ReduceContext>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool try_update(
    const uint32                read_id,
    io::BestPairedAlignments&   best_pairs,
    const io::PairedAlignments& pair,
    const uint32                min_distance,
    ReduceContext&              context,
    const ParamsPOD             params)
{
    const int32 score = pair.score();

    // sometimes, we might find an alignment which is better than what previously found, despite not being distinct from it...
    if (distinct_alignments( best_pairs.pair<0>(), pair, min_distance ) == false)
    {
        // this alignment is a candidate for replacing the best one
        //

        if (score > best_pairs.best_score())
            replace_best( read_id, best_pairs, pair, context, params );

        // skip locations that we have already visited, without paying the extension attempt
        return true;
    }
    else if (distinct_alignments( best_pairs.pair<1>(), pair, min_distance ) == false)
    {
        // this alignment is a candidate for replacing the second-best one
        //

        if (score > best_pairs.best_score())            // is it better than the best?
            update_best( read_id, best_pairs, pair, context, params );
        else if (score > best_pairs.second_score())     // is it better than the second-best?
            update_second( read_id, best_pairs, pair, context, params );

        // skip locations that we have already visited, without paying the extension attempt
        return true;
    }
    // check if the best alignment was unpaired or had a worse score
    else if (best_pairs.is_paired() == false || score > best_pairs.best_score())
    {
        update_best( read_id, best_pairs, pair, context, params );
        return true;
    } // check if the second-best alignment was unpaired or had a worse score
    else if (best_pairs.has_second_paired() == false || score > best_pairs.second_score())
    {
        update_second( read_id, best_pairs, pair, context, params );
        return true;
    }
    return false;
}

template <typename ReduceContext>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void update_best(
    const uint32                read_id,
    io::Alignment&              a1,
    io::Alignment&              a2,
    const io::Alignment&        a,
    ReduceContext&              context,
    const ParamsPOD             params)
{
    context.best_score( read_id, params ); // pretend we did find something useful, even though for unpaired alignments

    a2 = a1;
    a1 = a;
    NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_reduce( read_id ), "update best unpaired[%u]:\n  1. score[%d], rc[%u], pos[%u]\n", a.mate(), a.score(), a.is_rc(), a.alignment() );
}

template <typename ReduceContext>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void replace_best(
    const uint32                read_id,
    io::Alignment&              a1,
    io::Alignment&              a2,
    const io::Alignment&        a,
    ReduceContext&              context,
    const ParamsPOD             params)
{
    context.best_score( read_id, params ); // pretend we did find something useful, even though for unpaired alignments

    a1 = a;
    NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_reduce( read_id ), "update best unpaired[%u]:\n  1. score[%d], rc[%u], pos[%u]\n", a.mate(), a.score(), a.is_rc(), a.alignment() );
}

template <typename ReduceContext>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void update_second(
    const uint32                read_id,
    io::Alignment&              a1,
    io::Alignment&              a2,
    const io::Alignment&        a,
    ReduceContext&              context,
    const ParamsPOD             params)
{
    context.second_score( read_id, params ); // pretend we did find something useful, even though for unpaired alignments

    a2 = a;
    NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_reduce( read_id ), "update second unpaired[%u]:\n  score[%d], rc[%u], pos[%u]\n", a.mate(), i, a.score(), a.is_rc(), a.alignment() );
}

template <typename ReduceContext>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool try_update(
    const uint32                read_id,
    io::Alignment&              a1,
    io::Alignment&              a2,
    const io::Alignment&        a,
    const uint32                min_distance,
    ReduceContext&              context,
    const ParamsPOD             params)
{
    // sometimes, we might find an alignment which is better than what previously found, despite not being distinct from it...
    if (distinct_alignments( a1, a, min_distance ) == false)
    {
        // this alignment is a candidate for replacing the best one
        //

        if (a.score() > a1.score())
            replace_best( read_id, a1, a2, a, context, params );

        // skip locations that we have already visited, without paying the extension attempt
        return true;
    }
    else if (distinct_alignments( a2, a, min_distance ) == false)
    {
        // this alignment is a candidate for replacing the second-best one
        //

        if (a.score() > a1.score())                 // is it better than the best?
            update_best( read_id, a1, a2, a, context, params );
        else if (a.score() > a2.score())            // is it better than the second-best?
            update_second( read_id, a1, a2, a, context, params );

        // skip locations that we have already visited, without paying the extension attempt
        return true;
    }
    else if (a.score() > a1.score())
    {
        update_best( read_id, a1, a2, a, context, params );
        return true;
    }
    else if (a.score() > a2.score())
    {
        update_second( read_id, a1, a2, a, context, params );
        return true;
    }
    return false;
}

///
/// Reduce the list of scores associated to each read in the input queue to find
/// the best 2 alignments.
///
/// \details
/// this kernel takes a batch of extension results (one per active read, indexed by a sorting id)
/// and 'reduces' them with best 2 results found so far.
/// The kernel is parameterized by a templated context which can take further actions upon
/// updates to the best or second best scores, as well as bailing-out on failures.
///
/// \param context             the template context
/// \param pipeline            the pipeline state
/// \param params              algorithm parameters
///
template <typename ScoringScheme, typename PipelineType, typename ReduceContext> __global__ 
void score_reduce_paired_kernel(
    const ReduceContext     context,
          PipelineType      pipeline,
    const ParamsPOD         params)
{
    typedef ScoringQueuesDeviceView                 scoring_queues_type;
    typedef ReadHitsReference<scoring_queues_type>  read_hits_type;
    typedef typename read_hits_type::reference      hit_type;

    scoring_queues_type& scoring_queues = pipeline.scoring_queues;

    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= scoring_queues.active_read_count()) return;

    const uint32 anchor = pipeline.anchor;

    // fetch the active read corresponding to this thread
    read_hits_type read_hits( scoring_queues, thread_id );

    const uint32 read_id = read_hits.read_info().read_id;

    // fetch current best alignments
    io::BestPairedAlignments best_pairs = io::BestPairedAlignments(
        io::BestAlignments( pipeline.best_alignments[ read_id ],
                            pipeline.best_alignments[ read_id + pipeline.best_stride ] ),
        io::BestAlignments( pipeline.best_alignments_o[ read_id ],
                            pipeline.best_alignments_o[ read_id + pipeline.best_stride ] ) );

    // setup the read stream
    const uint2  read_range = pipeline.reads.get_range(read_id);
    const uint32 read_len   = read_range.y - read_range.x;

    const uint32 count = read_hits.size();

    // sort the entries in the score queue so that we process them in the order they were selected
    for (uint32 i = 0; i < count; ++i)
    {
        hit_type hit = read_hits[i];
        NVBIO_CUDA_DEBUG_ASSERT( hit.read_id == read_id, "reduce hit[%u][%u]: expected id %u, got: %u (slot: %u)\n", thread_id, i, read_id, hit.read_id, read_hits.slot(i) );

        const packed_seed seed_info = hit.seed;
        const uint32 read_rc        = seed_info.rc;
        const uint32 top_flag       = seed_info.top_flag;

        const uint2  g_pos          = make_uint2( hit.loc, hit.sink );
        const  int32 score_a        = hit.score;
        const  int32 score_o        = hit.opposite_score;
        const  int32 score_o2       = hit.opposite_score2;
        //const  int32 score          = score_a + score_o;
        //const  int32 score2         = score_a + score_o2;

        // compute opposite mate placement & orientation
        bool o_left;
        bool o_fw;

        detail::frame_opposite_mate(
            params.pe_policy,
            anchor,
            !read_rc,
            o_left,
            o_fw );

        const uint2  o_g_pos   = make_uint2( hit.opposite_loc, hit.opposite_sink );
        const uint2  o_g_pos2  = make_uint2( hit.opposite_loc, hit.opposite_sink2 );
        const uint32 o_read_rc = !o_fw;

        const io::PairedAlignments pair(
            io::Alignment(   g_pos.x,   g_pos.y -   g_pos.x, score_a,   read_rc,  anchor, score_o > pipeline.score_limit ),
            io::Alignment( o_g_pos.x, o_g_pos.y - o_g_pos.x, score_o, o_read_rc, !anchor, score_o > pipeline.score_limit ) );

        const io::PairedAlignments pair2(
            io::Alignment(   g_pos.x,    g_pos.y  -   g_pos.x,  score_a,    read_rc,  anchor, score_o2 > pipeline.score_limit ),
            io::Alignment( o_g_pos2.x, o_g_pos2.y - o_g_pos2.x, score_o2, o_read_rc, !anchor, score_o2 > pipeline.score_limit ) );

        bool updated = false;

        const uint32 min_distance = read_len/4;

        if (pair.is_paired())
        {
            if (try_update( read_id, best_pairs, pair, min_distance, context, params ))
                updated = true;

            // and now rinse & repeat with the second hit
            if (pair2.is_paired())
            {
                if (try_update( read_id, best_pairs, pair2, min_distance, context, params ))
                    updated = true;
            }
        }
        else if ((params.pe_unpaired == true) && (best_pairs.is_paired() == false))
        {
            //
            // We didn't find a paired alignment yet - hence we proceed keeping track of the best unpaired alignments
            // for the first and second mate separately.
            // We store best two alignments for the first mate in best_pairs.m_a*, and the ones of the second mate in best_pairs.m_o*.
            //

            const io::Alignment& a  = pair.m_a;
                  io::Alignment& a1 = anchor ? best_pairs.m_o1 : best_pairs.m_a1;
                  io::Alignment& a2 = anchor ? best_pairs.m_o2 : best_pairs.m_a2;

            //
            // update the first mate alignments
            //

            if (try_update( read_id, a1, a2, a, min_distance, context, params ))
                updated = true;
        }

        if ((updated == false) && context.failure( i, read_id, top_flag, params ))
        {
            // stop traversal
            pipeline.hits.erase( read_id );
            //NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_reduce( read_id ), "discard:  (parent[%u:%u])\n  score[%d + %d], rc[%u], pos[%u]\n", anchor, thread_id, i, score_a, score_o, read_rc, g_pos );
        }
    }

    // report best alignments
    pipeline.best_alignments[ read_id ]                          = best_pairs.m_a1;
    pipeline.best_alignments[ read_id + pipeline.best_stride ]   = best_pairs.m_a2;
    pipeline.best_alignments_o[ read_id  ]                       = best_pairs.m_o1;
    pipeline.best_alignments_o[ read_id + pipeline.best_stride ] = best_pairs.m_o2;
}

///@}  // group ReduceDetail
///@}  // group Reduce
///@}  // group nvBowtie

} // namespace detail

//
// Reduce the scores associated to each read in the input queue to find
// the best 2 alignments.
//
template <typename ScoringScheme, typename ReduceContext>
void score_reduce_t(
    const ReduceContext                                     context,
    const BestApproxScoringPipelineState<ScoringScheme>&    pipeline,
    const ParamsPOD                                         params)
{
    typedef BestApproxScoringPipelineState<ScoringScheme> pipeline_type;

    const int blocks = (pipeline.scoring_queues.active_reads.in_size + BLOCKDIM-1) / BLOCKDIM;

    detail::score_reduce_kernel<typename pipeline_type::scheme_type> <<<blocks, BLOCKDIM>>>(
        context,
        pipeline,
        params );
}

//
// call the scoring kernel
//
template <typename ScoringScheme, typename ReduceContext>
void score_reduce_paired_t(
    const ReduceContext                                     context,
    const BestApproxScoringPipelineState<ScoringScheme>&    pipeline,
    const ParamsPOD                                         params)
{
    typedef BestApproxScoringPipelineState<ScoringScheme> pipeline_type;

    const int blocks = (pipeline.scoring_queues.active_reads.in_size + BLOCKDIM-1) / BLOCKDIM;

    detail::score_reduce_paired_kernel<typename pipeline_type::scheme_type> <<<blocks, BLOCKDIM>>>(
        context,
        pipeline,
        params );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
