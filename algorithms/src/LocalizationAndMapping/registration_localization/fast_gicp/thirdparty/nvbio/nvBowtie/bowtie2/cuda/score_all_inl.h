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
struct AllScoreStream : public AlignmentStreamBase<SCORE_STREAM,AlignerType,PipelineType>
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
    AllScoreStream(
        const uint32                _band_len,
        const PipelineType          _pipeline,
        const AlignerType           _aligner,
        const ParamsPOD             _params,
        const uint32                _buffer_offset,
        const uint32                _buffer_size,
              uint32*               _counter) :
        base_type( _pipeline, _aligner, _params ), m_band_len( _band_len ),
        m_buffer_offset( _buffer_offset ),
        m_buffer_size( _buffer_size ),
        m_counter( _counter ) {}

    /// return the maximum pattern length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_pattern_length() const { return base_type::m_pipeline.reads.max_sequence_len(); }

    /// return the maximum text length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_text_length() const { return base_type::m_pipeline.reads.max_sequence_len() + m_band_len; }

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

        // setup the read info
        context->mate         = 0;
        context->read_rc      = hit.seed.rc;
        context->read_id      = hit.read_id;
        context->read_range   = base_type::m_pipeline.reads.get_range( context->read_id );

        // setup the genome range
        const uint32 g_pos = hit.loc;

        const uint32 read_len = context->read_range.y - context->read_range.x;
        context->genome_begin = g_pos > m_band_len/2 ? g_pos - m_band_len/2 : 0u;
        context->genome_end   = nvbio::min( context->genome_begin + m_band_len + read_len, base_type::m_pipeline.genome_length );

        // setup the minimum score
        context->min_score = base_type::m_pipeline.scoring_scheme.min_score( read_len );
        return true;
    }

    /// handle the output
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void output(
        const uint32        i,
        const context_type* context) const
    {
        const aln::BestSink<int32> sink = context->sink;

        // append all valid alignments to a list
        if (sink.score >= context->min_score)
        {
            HitReference<HitQueuesDeviceView> hit = base_type::m_pipeline.scoring_queues.hits[ context->idx ];

          #if defined(NVBIO_DEVICE_COMPILATION)
            const uint32 slot = atomicAdd( m_counter, 1u );
          #else
            const uint32 slot = (*m_counter)++;
          #endif
            base_type::m_pipeline.buffer_read_info[ (m_buffer_offset + slot) % m_buffer_size ]  = hit.read_id; //(hit.read_id << 1) | hit.seed.rc;
            base_type::m_pipeline.buffer_alignments[ (m_buffer_offset + slot) % m_buffer_size ] = io::Alignment( hit.loc, 0u, sink.score, hit.seed.rc );
            // TODO: rewrite hit.loc
            // hit.loc = context->genome_begin;
            NVBIO_CUDA_DEBUG_PRINT_IF( base_type::m_params.debug.show_score( context->read_id, (sink.score >= context->min_score) ), "score: %d (rc[%u], pos[%u], [qid %u])]\n", sink.score, context->read_rc, context->genome_begin, i );
        }
    }

    const uint32  m_band_len;
    const uint32  m_buffer_offset;
    const uint32  m_buffer_size;
          uint32* m_counter;
};

///
/// dispatch the execution of a batch of single-ended banded-alignment score calculations
///
template <typename aligner_type, typename pipeline_type>
void banded_score_all(
    const uint32         band_len,
    const pipeline_type& pipeline,
    const aligner_type   aligner,
    const ParamsPOD      params,
    const uint32         buffer_offset,
    const uint32         buffer_size,
          uint32*        counter)
{
    const uint32 static_band_len = 
        (band_len < 4)  ? 3u  :
        (band_len < 8)  ? 7u  :
        (band_len < 16) ? 15u :
                          31u;

    typedef AllScoreStream<aligner_type,pipeline_type> stream_type;

    stream_type stream(
        static_band_len,
        pipeline,
        aligner,
        params,
        buffer_offset,
        buffer_size,
        counter );

    //typedef aln::DeviceThreadScheduler scheduler_type;
    typedef aln::DeviceStagedThreadScheduler scheduler_type;

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
// execute a batch of single-ended banded-alignment score calculations, best mapping
//
// \param band_len             alignment band length
// \param pipeline             all mapping pipeline
// \param params               alignment params
// \param buffer_offset        ring buffer offset
// \param buffer_size          ring buffer size
// \return                     number of valid alignments
template <typename scheme_type>
uint32 score_all_t(
    const uint32                                        band_len,
    const AllMappingPipelineState<scheme_type>&         pipeline,
    const ParamsPOD                                     params,
    const uint32                                        ring_offset,
    const uint32                                        ring_size)
{
    thrust::device_vector<uint32> counter(1,0);

    if (params.alignment_type == LocalAlignment)
    {
        detail::banded_score_all(
            band_len,
            pipeline,
            pipeline.scoring_scheme.local_aligner(),
            params,
            ring_offset,
            ring_size,
            nvbio::device_view(counter) );
    }
    else
    {
        detail::banded_score_all(
            band_len,
            pipeline,
            pipeline.scoring_scheme.end_to_end_aligner(),
            params,
            ring_offset,
            ring_size,
            nvbio::device_view(counter) );
    }
    //cudaDeviceSynchronize();
    //nvbio::cuda::check_error("score kernel");
    return counter[0];
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
