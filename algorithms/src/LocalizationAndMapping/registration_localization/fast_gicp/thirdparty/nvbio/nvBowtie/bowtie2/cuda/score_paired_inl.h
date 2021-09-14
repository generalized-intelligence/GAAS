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

#include <nvBowtie/bowtie2/cuda/alignment_utils.h>
#include <nvBowtie/bowtie2/cuda/pipeline_states.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

namespace detail {

///@addtogroup nvBowtie
///@{

///@addtogroup Scoring
///@{

///@addtogroup ScoringDetail
///@{

///
/// A scoring stream, fetching the input hits to score from the hit queue
/// indexed by the input sorting order, and assigning them their score and sink
/// attributes.
///
template <typename AlignerType, typename PipelineType>
struct BestAnchorScoreStream : public AlignmentStreamBase<SCORE_STREAM,AlignerType,PipelineType>
{
    typedef AlignmentStreamBase<SCORE_STREAM,AlignerType,PipelineType>  base_type;
    typedef typename base_type::context_type                            context_type;
    typedef typename base_type::scheme_type                             scheme_type;

    /// constructor
    ///
    /// \param _band_len            effective band length;
    ///                             NOTE: this value must match the template BAND_LEN parameter
    ///                             used for instantiating aln::BatchedBandedAlignmentScore.
    ///
    /// \param _pipeline            the pipeline object
    ///
    /// \param _aligner             the aligner object
    ///
    BestAnchorScoreStream(
        const uint32                _band_len,
        const PipelineType          _pipeline,
        const AlignerType           _aligner,
        const ParamsPOD             _params) :
        base_type( _pipeline, _aligner, _params ), m_band_len( _band_len ) {}

    /// return the maximum pattern length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_pattern_length() const { return base_type::m_pipeline.reads.max_read_len(); }

    /// return the maximum text length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_text_length() const { return base_type::m_pipeline.reads.max_read_len() + m_band_len; }

    /// return the stream size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return base_type::m_pipeline.hits_queue_size; }

    /// initialize the i-th context
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool init_context(
        const uint32    i,
        context_type*   context) const
    {
        context->idx = base_type::m_pipeline.idx_queue[i];

        // initialize the sink
        context->sink = aln::BestSink<int32>();

        // fetch the hit to process
        HitReference<HitQueuesDeviceView> hit = base_type::m_pipeline.scoring_queues.hits[ context->idx ];

        const uint32 read_rc = hit.seed.rc;
        const uint32 read_id = hit.read_id;

        const io::BestPairedAlignments best = io::BestPairedAlignments(
            io::BestAlignments( base_type::m_pipeline.best_alignments[ read_id ], base_type::m_pipeline.best_alignments[ read_id + base_type::m_pipeline.best_stride ] ),
            io::BestAlignments( base_type::m_pipeline.best_alignments_o[ read_id ], base_type::m_pipeline.best_alignments_o[ read_id + base_type::m_pipeline.best_stride ] ) );

        // compute the optimal score of the opposite mate
        const uint2  o_read_range    = base_type::m_pipeline.reads_o.get_range( read_id );
        const uint32 o_read_len      = o_read_range.y - o_read_range.x;
        const  int32 o_optimal_score = base_type::m_pipeline.scoring_scheme.perfect_score( o_read_len );
        const  int32 o_worst_score   = base_type::m_pipeline.scoring_scheme.min_score( o_read_len );

        const uint2  a_read_range    = base_type::m_pipeline.reads.get_range( read_id );
        const uint32 a_read_len      = a_read_range.y - a_read_range.x;
        const  int32 a_optimal_score = base_type::m_pipeline.scoring_scheme.perfect_score( a_read_len );
        const  int32 a_worst_score   = base_type::m_pipeline.scoring_scheme.min_score( a_read_len );

        const  int32 target_pair_score = nvbio::min(
            compute_target_score( best, a_worst_score, o_worst_score ) + 1,
            a_optimal_score + o_optimal_score );

        int32 target_mate_score = target_pair_score - o_optimal_score;

      #if 1
        //
        // bound the score of this mate by the worst score allowed for its own read length.
        // NOTE: disabling this is equivalent to allowing a worst score proportional to the total
        // length of the read pair.
        //

        target_mate_score = nvbio::max( target_mate_score, a_worst_score );
      #endif

        // setup the read info
        context->mate       = 0;
        context->read_rc    = read_rc;
        context->read_id    = read_id;
        context->read_range = a_read_range;

        // setup the genome range
        const uint32 g_pos = hit.loc;

        context->genome_begin = g_pos > m_band_len/2 ? g_pos - m_band_len/2 : 0u;
        context->genome_end   = nvbio::min( context->genome_begin + m_band_len + a_read_len, base_type::m_pipeline.genome_length );

        // skip locations that we have already visited
        const uint32 mate = base_type::m_pipeline.anchor;
        const bool skip = (mate == best.m_a1.m_mate && (read_rc == best.m_a1.m_rc && g_pos == best.m_a1.m_align)) ||
                          (mate == best.m_o1.m_mate && (read_rc == best.m_o1.m_rc && g_pos == best.m_o1.m_align)) ||
                          (mate == best.m_a2.m_mate && (read_rc == best.m_a2.m_rc && g_pos == best.m_a2.m_align)) ||
                          (mate == best.m_o2.m_mate && (read_rc == best.m_o2.m_rc && g_pos == best.m_o2.m_align)) ||
                          (context->min_score > a_optimal_score);

        // setup the minimum score
        context->min_score = skip ? Field_traits<int32>::max() : nvbio::max( target_mate_score, base_type::m_pipeline.score_limit );

        return !skip;
    }

    /// handle the output
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void output(
        const uint32        i,
        const context_type* context) const
    {
        // write the final hit.score and hit.sink attributes
        HitReference<HitQueuesDeviceView> hit = base_type::m_pipeline.scoring_queues.hits[ context->idx ];

        const aln::BestSink<int32> sink = context->sink;
        hit.score = (sink.score >= context->min_score) ? sink.score : scheme_type::worst_score;
        hit.sink  = context->genome_begin + sink.sink.x;
        // TODO: rewrite hit.loc
        // hit.loc = context->genome_begin;
        NVBIO_CUDA_DEBUG_PRINT_IF( base_type::m_params.debug.show_score( context->read_id, (sink.score >= context->min_score) ), "score anchor: %d (min[%d], mate[%u], rc[%u], pos[%u], [qid %u])\n", sink.score, context->min_score, base_type::m_pipeline.anchor, context->read_rc, context->genome_begin, i );
    }

    const uint32 m_band_len;
};

///
/// dispatch the execution of a batch of banded-alignment score calculations for the anchor mates
///
template <typename aligner_type, typename pipeline_type>
void banded_anchor_score_best(
    const uint32         band_len,
    const pipeline_type& pipeline,
    const aligner_type   aligner,
    const ParamsPOD      params)
{
    const uint32 static_band_len = 
        (band_len < 4)  ? 3u  :
        (band_len < 8)  ? 7u  :
        (band_len < 16) ? 15u :
                          31u;

    typedef BestAnchorScoreStream<aligner_type,pipeline_type> stream_type;

    stream_type stream(
        static_band_len,
        pipeline,
        aligner,
        params );

    typedef aln::DeviceThreadScheduler scheduler_type;
    //typedef aln::DeviceStagedThreadScheduler scheduler_type;

    if (band_len < 4)
    {
        aln::BatchedBandedAlignmentScore<3u,stream_type,scheduler_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
    else if (band_len < 8)
    {
        aln::BatchedBandedAlignmentScore<7u,stream_type,scheduler_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
    else if (band_len < 16)
    {
        aln::BatchedBandedAlignmentScore<15u,stream_type,scheduler_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
    else
    {
        aln::BatchedBandedAlignmentScore<31u,stream_type,scheduler_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
}

///@}  // group ScoringDetail
///@}  // group Scoring
///@}  // group nvBowtie

} // namespace detail

//
// execute a batch of banded-alignment score calculations for the anchor mates, best mapping
//
template <typename scheme_type>
void anchor_score_best_t(
    const uint32                                        band_len,
    const BestApproxScoringPipelineState<scheme_type>&  pipeline,
    const ParamsPOD                                     params)
{
    if (params.alignment_type == LocalAlignment)
    {
        detail::banded_anchor_score_best(
            band_len,
            pipeline,
            pipeline.scoring_scheme.local_aligner(),
            params );
    }
    else
    {
        detail::banded_anchor_score_best(
            band_len,
            pipeline,
            pipeline.scoring_scheme.end_to_end_aligner(),
            params );
    }
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
