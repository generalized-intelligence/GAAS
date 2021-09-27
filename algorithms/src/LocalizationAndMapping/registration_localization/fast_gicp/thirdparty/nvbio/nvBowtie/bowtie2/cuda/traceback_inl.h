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

namespace nvbio {
namespace bowtie2 {
namespace cuda {

namespace detail {

///@addtogroup nvBowtie
///@{

///@addtogroup Traceback
///@{

///@addtogroup TracebackDetail
///@{

///
/// A scoring stream, fetching the input hits to score from the hit queue
/// indexed by the input sorting order, and assigning them their score and sink
/// attributes.
///
template <uint32 ALN_IDX, typename AlignerType, typename PipelineType>
struct BestTracebackStream : public AlignmentStreamBase<TRACEBACK_STREAM,AlignerType,PipelineType>
{
    typedef AlignmentStreamBase<TRACEBACK_STREAM,AlignerType,PipelineType>  base_type;
    typedef typename base_type::context_type                                context_type;
    typedef typename base_type::scheme_type                                 scheme_type;

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
    BestTracebackStream(
        const MateType              _mate_type,
        const uint32                _count,
        const uint32*               _idx,
              io::Alignment*        _best_data,
        const uint32                _best_stride,
        const uint32                _band_len,
        const PipelineType          _pipeline,
        const AlignerType           _aligner,
        const ParamsPOD             _params) :
        base_type( _pipeline, _aligner, _params ), m_mate_type( _mate_type ), m_count(_count), m_idx(_idx), m_best_data(_best_data), m_best_stride( _best_stride ), m_band_len( _band_len ) {}

    /// return the maximum pattern length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_pattern_length() const { return MAXIMUM_READ_LENGTH; }

    /// return the maximum text length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_text_length() const { return MAXIMUM_INSERT_LENGTH; }

    /// return the mate type
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    MateType mate_type() const { return m_mate_type; }

    /// return the stream size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_count; }

    /// initialize the i-th context
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool init_context(
        const uint32    i,
        context_type*   context) const
    {
        context->idx = m_idx ? m_idx[ i ] : i;

        const io::Alignment& alignment = m_best_data[ context->idx + ALN_IDX * m_best_stride ];
        if (alignment.is_aligned() == false)
            return false;

        context->min_score = alignment.score();

        // setup the read info
        context->mate         = alignment.mate();
        context->read_rc      = alignment.is_rc();
        context->read_id      = context->idx;
        context->read_range   = base_type::m_pipeline.get_reads( context->mate ).get_range( context->read_id );

        const uint32 read_len = context->read_range.y - context->read_range.x;

        // setup the genome range
        const bool paired_opposite_alignment = (m_mate_type == OppositeMate && alignment.is_concordant());
        const uint32 g_pos = paired_opposite_alignment ?
            alignment.alignment() :
            alignment.alignment() > m_band_len/2 ? alignment.alignment() - m_band_len/2 : 0u;
        const uint32 g_len = paired_opposite_alignment ?
            alignment.sink() :
            m_band_len + read_len;

        context->genome_begin = g_pos;
        context->genome_end   = nvbio::min( context->genome_begin + g_len, base_type::m_pipeline.genome_length );
        return true;
    }

    /// handle the output
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void output(
        const uint32        i,
        const context_type* context)
    {
        const uint32 cigar_len = context->backtracer.size;
        if (cigar_len == 0)
            return;

        // alloc the output cigar
        io::Cigar* cigar = base_type::m_pipeline.cigar.alloc( context->read_id, cigar_len );
        NVBIO_CUDA_ASSERT_IF( base_type::m_params.debug.asserts, cigar != NULL, "%s backtrack(): unable to allocate CIGAR!\n  read[%u], mate[%u], cigar: %u!\n", mate_string( m_mate_type ), context->read_id, context->mate, cigar_len );
        if (cigar)
        {
            // copy the local cigar to the output one
            for (uint32 i = 0; i < cigar_len; ++i)
                cigar[i] = context->cigar[i];
        }

        // write the cigar coords
        base_type::m_pipeline.cigar_coords[ context->read_id ] = make_uint2( context->alignment.source.x + (context->alignment.source.y << 16), context->backtracer.size );
        NVBIO_CUDA_DEBUG_PRINT_IF( base_type::m_params.debug.show_traceback( context->read_id ), "BestTracebackStream<%u>:\n  mate[%u], cigar-coords[%u:%u,%u]\n", ALN_IDX, context->mate, context->alignment.source.x, context->alignment.source.y, context->backtracer.size );

        NVBIO_CUDA_DEBUG_CHECK_IF( base_type::m_params.debug.show_traceback( context->read_id ), context->alignment.sink.x != uint32(-1) && context->alignment.sink.y != uint32(-1), "\nerror:\n  %s backtrack(): failed to re-align!\n  read[%u], rc[%u], mate[%u] (expected score: %d)\n", mate_string( m_mate_type ), context->read_id, context->read_rc, context->mate, context->min_score );
        NVBIO_CUDA_DEBUG_CHECK_IF( base_type::m_params.debug.show_traceback( context->read_id ), context->alignment.score == context->min_score,                                     "\nerror:\n  %s backtrack(): score %d different from previously calculated value %d!\n  read[%u], rc[%u], mate[%u]\n", mate_string( m_mate_type ), context->alignment.score, context->min_score, context->read_id, context->read_rc, context->mate );
        NVBIO_CUDA_ASSERT_IF( base_type::m_params.debug.asserts, context->alignment.sink.x != uint32(-1) && context->alignment.sink.y != uint32(-1),     "%s backtrack(): failed to re-align!\n  read[%u], rc[%u], mate[%u] (expected score: %d)\n", mate_string( m_mate_type ), context->read_id, context->read_rc, context->mate, context->min_score );
        NVBIO_CUDA_ASSERT_IF( base_type::m_params.debug.asserts, context->alignment.score == context->min_score,                                         "%s backtrack(): score %d different from previously calculated value %d!\n  read[%u], rc[%u], mate[%u]\n", mate_string( m_mate_type ), context->alignment.score, context->min_score, context->read_id, context->read_rc, context->mate );

        NVBIO_CUDA_ASSERT_IF( base_type::m_params.debug.asserts,
            read_cigar_length( cigar, context->backtracer.size ) == context->read_range.y - context->read_range.x,
            "%s backtrack(): CIGAR length != read length\n  read[%u]\n  cigar len: %u\n  read len: %u\n  source: (%u, %u)\n  sink: (%u, %u)\n", mate_string( m_mate_type ),
            context->read_id, read_cigar_length( cigar, context->backtracer.size ), context->read_range.y - context->read_range.x, context->alignment.source.x, context->alignment.source.y, context->alignment.sink.x, context->alignment.sink.y  );
    }

    /// handle the output
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void finish(
        const uint32        i,
        const context_type* context,
        const uint32        ed,
        const  int32        score)
    {
        // rewrite the alignment
        io::Alignment&      aln  = m_best_data[ context->read_id + ALN_IDX * m_best_stride ];
        aln.m_align     = context->genome_begin;
        aln.m_ed        = ed;
        aln.m_score_sgn = score < 0 ? 1 : 0;
        aln.m_score     = score < 0 ? -score : score;

        NVBIO_CUDA_DEBUG_PRINT_IF( base_type::m_params.debug.show_traceback( context->read_id ), "BestTracebackStream<%u>:\n  mate[%u], ed[%u], score[%d], pos[%u], rc[%u]\n", ALN_IDX, aln.mate(), aln.ed(), aln.score(), aln.alignment(), uint32( aln.is_rc() ));
    }

    const MateType              m_mate_type;
    const uint32                m_count;
    const uint32*               m_idx;
          io::Alignment*        m_best_data;
    const uint32                m_best_stride;
    const uint32                m_band_len;
};

///
/// dispatch the execution of a batch of single-ended banded-alignment traceback calculations
///
template <uint32 ALN_IDX, typename aligner_type, typename pipeline_type>
void banded_traceback_best(
    const MateType              mate_type,
    const uint32                count,
    const uint32*               idx,
          io::Alignment*        best_data,
    const uint32                best_stride,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const aligner_type          aligner,
    const ParamsPOD             params)
{
    const uint32 static_band_len = 
        (band_len < 4)  ? 3u  :
        (band_len < 8)  ? 7u  :
        (band_len < 16) ? 15u :
                          31u;

    typedef BestTracebackStream<ALN_IDX,aligner_type,pipeline_type> stream_type;

    stream_type stream(
        mate_type,
        count,
        idx,
        best_data,
        best_stride,
        static_band_len,
        pipeline,
        aligner,
        params );

    NVBIO_VAR_UNUSED const uint32 CHECKPOINTS = BANDED_DP_CHECKPOINTS;

    if (band_len < 4)
    {
        aln::BatchedBandedAlignmentTraceback<3u,CHECKPOINTS,stream_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
    else if (band_len < 8)
    {
        aln::BatchedBandedAlignmentTraceback<7u,CHECKPOINTS,stream_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
    else if (band_len < 16)
    {
        aln::BatchedBandedAlignmentTraceback<15u,CHECKPOINTS,stream_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
    else
    {
        aln::BatchedBandedAlignmentTraceback<31u,CHECKPOINTS,stream_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
}

///
/// dispatch the execution of a batch of full DP alignment traceback calculations
///
template <uint32 ALN_IDX, typename aligner_type, typename pipeline_type>
void traceback_best(
    const MateType              mate_type,
    const uint32                count,
    const uint32*               idx,
          io::Alignment*        best_data,
    const uint32                best_stride,
    const pipeline_type&        pipeline,
    const aligner_type          aligner,
    const ParamsPOD             params)
{
    typedef BestTracebackStream<ALN_IDX,aligner_type,pipeline_type> stream_type;

    stream_type stream(
        mate_type,
        count,
        idx,
        best_data,
        best_stride,
        0u,         // band-len
        pipeline,
        aligner,
        params );

    typedef aln::DeviceThreadBlockScheduler<128,9> scheduler_type;

    aln::BatchedAlignmentTraceback<FULL_DP_CHECKPOINTS,stream_type,scheduler_type> batch;

    batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
}

///
/// A scoring stream, fetching the input hits to score from the hit queue
/// indexed by the input sorting order, and assigning them their score and sink
/// attributes.
///
template <typename AlignerType, typename PipelineType>
struct AllTracebackStream : public AlignmentStreamBase<TRACEBACK_STREAM,AlignerType,PipelineType>
{
    typedef AlignmentStreamBase<TRACEBACK_STREAM,AlignerType,PipelineType>  base_type;
    typedef typename base_type::context_type                                context_type;
    typedef typename base_type::scheme_type                                 scheme_type;

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
    AllTracebackStream(
        const uint32                _count,
        const uint32*               _idx,
        const uint32                _buffer_offset,
        const uint32                _buffer_size,
              io::Alignment*        _alignments,
        const uint32                _band_len,
        const PipelineType          _pipeline,
        const AlignerType           _aligner,
        const ParamsPOD             _params) :
        base_type( _pipeline, _aligner, _params ),
        m_mate_type( AnchorMate ),
        m_count(_count),
        m_idx(_idx),
        m_buffer_offset(_buffer_offset),
        m_buffer_size(_buffer_size),
        m_alignments(_alignments),
        m_band_len( _band_len ) {}

    /// return the maximum pattern length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_pattern_length() const { return MAXIMUM_READ_LENGTH; }

    /// return the maximum text length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_text_length() const { return MAXIMUM_INSERT_LENGTH; }

    /// return the mate type
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    MateType mate_type() const { return m_mate_type; }

    /// return the stream size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_count; }

    /// initialize the i-th context
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool init_context(
        const uint32    i,
        context_type*   context) const
    {
        context->idx = i;

        const uint32 buffer_idx = (m_idx[ i ] + m_buffer_offset) % m_buffer_size;
        const io::Alignment& alignment = base_type::m_pipeline.buffer_alignments[ buffer_idx ];

        // setup the read info
        context->mate         = alignment.mate();
        context->read_rc      = alignment.is_rc();
        context->read_id      = base_type::m_pipeline.output_read_info[ i ];
        context->read_range   = base_type::m_pipeline.get_reads( context->mate ).get_range( context->read_id );

        const uint32 read_len = context->read_range.y - context->read_range.x;

        // setup the genome range
        const uint32 g_pos = alignment.alignment() > m_band_len/2 ? alignment.alignment() - m_band_len/2 : 0u;
        const uint32 g_len = m_band_len + read_len;

        context->genome_begin = g_pos;
        context->genome_end   = nvbio::min( context->genome_begin + g_len, base_type::m_pipeline.genome_length );
        return true;
    }

    /// handle the output
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void output(
        const uint32        i,
        const context_type* context)
    {
        const uint32 cigar_len = context->backtracer.size;
        if (cigar_len == 0)
            return;

        // alloc the output cigar
        io::Cigar* cigar = base_type::m_pipeline.cigar.alloc( context->idx, cigar_len );
        NVBIO_CUDA_ASSERT_IF( base_type::m_params.debug.asserts, cigar != NULL, "%s backtrack(): unable to allocate CIGAR!\n  aln[%u], read[%u], mate[%u], cigar: %u!\n", mate_string( m_mate_type ), context->idx, context->read_id, context->mate, cigar_len );
        if (cigar)
        {
            // copy the local cigar to the output one
            for (uint32 i = 0; i < cigar_len; ++i)
                cigar[i] = context->cigar[i];
        }

        // write the cigar coords
        base_type::m_pipeline.cigar_coords[ context->idx ] = make_uint2( context->alignment.source.x + (context->alignment.source.y << 16), context->backtracer.size );
        NVBIO_CUDA_DEBUG_PRINT_IF( base_type::m_params.debug.show_traceback( context->read_id ), "AllTracebackStream:\n  mate[%u], cigar-coords[%u:%u,%u]\n", context->mate, context->alignment.source.x, context->alignment.source.y, context->backtracer.size );

        NVBIO_CUDA_DEBUG_CHECK_IF( base_type::m_params.debug.show_traceback( context->read_id ), context->alignment.sink.x != uint32(-1) && context->alignment.sink.y != uint32(-1), "\nerror:\n  backtrack(): failed to re-align!\n  read[%u], rc[%u], mate[%u] (expected score: %d)\n", context->read_id, context->read_rc, context->mate, context->min_score );
        NVBIO_CUDA_DEBUG_CHECK_IF( base_type::m_params.debug.show_traceback( context->read_id ), context->alignment.score == context->min_score,                                     "\nerror:\n  backtrack(): score %d different from previously calculated value %d!\n  read[%u], rc[%u], mate[%u]\n", context->alignment.score, context->min_score, context->read_id, context->read_rc, context->mate );
        NVBIO_CUDA_ASSERT_IF( base_type::m_params.debug.asserts, context->alignment.sink.x != uint32(-1) && context->alignment.sink.y != uint32(-1),     "backtrack(): failed to re-align!\n  read[%u], rc[%u], mate[%u] (expected score: %d)\n", context->read_id, context->read_rc, context->mate, context->min_score );
        NVBIO_CUDA_ASSERT_IF( base_type::m_params.debug.asserts, context->alignment.score == context->min_score,                                         "backtrack(): score %d different from previously calculated value %d!\n  read[%u], rc[%u], mate[%u]\n", context->alignment.score, context->min_score, context->read_id, context->read_rc, context->mate );

        NVBIO_CUDA_ASSERT_IF( base_type::m_params.debug.asserts,
            read_cigar_length( cigar, context->backtracer.size ) == context->read_range.y - context->read_range.x,
            "backtrack(): CIGAR length != read length\n  read[%u]\n  cigar len: %u\n  read len: %u\n  source: (%u, %u)\n  sink: (%u, %u)\n",
            context->read_id, read_cigar_length( cigar, context->backtracer.size ), context->read_range.y - context->read_range.x, context->alignment.source.x, context->alignment.source.y, context->alignment.sink.x, context->alignment.sink.y  );
    }

    /// handle the output
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void finish(
        const uint32        i,
        const context_type* context,
        const uint32        ed,
        const  int32        score)
    {
        const uint32 buffer_idx = (m_idx[ i ] + m_buffer_offset) % m_buffer_size;
        const io::Alignment& in_alignment  = base_type::m_pipeline.buffer_alignments[ buffer_idx ];
        io::Alignment&       out_alignment = m_alignments[ context->idx ];

        // rewrite the alignment
        out_alignment = io::Alignment(
            context->genome_begin,
            ed,
            score,
            in_alignment.is_rc(),
            in_alignment.is_paired(),
            in_alignment.is_discordant() );

        NVBIO_CUDA_DEBUG_PRINT_IF( base_type::m_params.debug.show_traceback( context->read_id ), "finish-alignment:\n  mate[%u],  ed[%u], score[%d], pos[%u], rc[%u]\n", context->mate, out_alignment.ed(), out_alignment.score(), out_alignment.alignment(), uint32( out_alignment.is_rc() ));
    }

    const MateType              m_mate_type;
    const uint32                m_count;
    const uint32*               m_idx;
    const uint32                m_buffer_offset;
    const uint32                m_buffer_size;
          io::Alignment*        m_alignments;
    const uint32                m_band_len;
};

///
/// dispatch the execution of a batch of single-ended banded-alignment traceback calculations
///
template <typename aligner_type, typename pipeline_type>
void banded_traceback_all(
    const uint32                count,
    const uint32*               idx,
    const uint32                buffer_offset,
    const uint32                buffer_size,
          io::Alignment*        alignments,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const aligner_type          aligner,
    const ParamsPOD             params)
{
    const uint32 static_band_len = 
        (band_len < 4)  ? 3u  :
        (band_len < 8)  ? 7u  :
        (band_len < 16) ? 15u :
                          31u;

    typedef AllTracebackStream<aligner_type,pipeline_type> stream_type;

    stream_type stream(
        count,
        idx,
        buffer_offset,
        buffer_size,
        alignments,
        static_band_len,
        pipeline,
        aligner,
        params );

    NVBIO_VAR_UNUSED const uint32 CHECKPOINTS = BANDED_DP_CHECKPOINTS;

    if (band_len < 4)
    {
        aln::BatchedBandedAlignmentTraceback<3u,CHECKPOINTS,stream_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
    else if (band_len < 8)
    {
        aln::BatchedBandedAlignmentTraceback<7u,CHECKPOINTS,stream_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
    else if (band_len < 16)
    {
        aln::BatchedBandedAlignmentTraceback<15u,CHECKPOINTS,stream_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
    else
    {
        aln::BatchedBandedAlignmentTraceback<31u,CHECKPOINTS,stream_type> batch;

        batch.enact( stream, pipeline.dp_buffer_size, pipeline.dp_buffer );
    }
}


///
/// finish computing the final alignments, i.e. computing the MD strings and the final scores (given the CIGARs)
///
template <typename stream_type, typename scheme_type, typename pipeline_type> __global__ 
void finish_alignment_kernel(
          stream_type       stream,
          pipeline_type     pipeline,
    const scheme_type       scoring_scheme,
    const ParamsPOD         params)
{
    const uint32 work_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (work_id >= stream.size()) return;

    typedef typename stream_type::aligner_type aligner_type;
    typedef typename stream_type::context_type context_type;
    typedef typename stream_type::strings_type strings_type;

    // load the alignment context
    context_type context;
    if (stream.init_context( work_id, &context ) == false)
        return;

    // compute the end of the current DP matrix window
    const uint32 read_len = stream.pattern_length( work_id, &context );

    // load the strings to be aligned
    strings_type strings;
    stream.load_strings( work_id, 0, read_len, &context, &strings );

    const uint2  cigar_coord  = pipeline.cigar_coords[ context.idx ];
    const uint32 cigar_offset = cigar_coord.x & 0xFFFF;
    const uint32 cigar_length = cigar_coord.y;

    //
    // compute actual score: this is needed because we might be using plain edit distance in
    // the first stage rather than doing full SW. We also take the chance to compute the MD
    // string here.
    //
    const io::Cigar* cigar_vector = pipeline.cigar[ context.idx ];

    // bail-out if no CIGAR's available!
    if (cigar_vector == NULL)
        return;

    const uint32 SUB_MASK = 1u << io::Cigar::SUBSTITUTION;
    const uint32 DEL_MASK = 1u << io::Cigar::DELETION;
    const uint32 INS_MASK = 1u << io::Cigar::INSERTION;
    const uint32 CLP_MASK = 1u << io::Cigar::SOFT_CLIPPING;

    NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_traceback( context.read_id ), "\nfinish_alignment(%s):\n  cigar slot: %u\n", stream.mate_type() == OppositeMate ? "opposite" : "anchor", pipeline.cigar.slot( context.read_id ) );

    // alloc some local memory for the MDS
    const uint32 MDS_LMEM = 2048;
    uint8 mds_local[MDS_LMEM];

    // leave a blank space in the MDS vector to store its length later on
    uint32 mds_len = 2;
    uint8  mds_op  = io::MDS_INVALID;

    uint32 ed   = 0;
    int32 score = 0;

    const uint32 g_len = strings.text.length();

    for (uint32 i = 0, j = 0/*read_pos*/, k = cigar_offset; i < cigar_length; ++i)
    {
        const uint32 l = cigar_vector[ cigar_length - i - 1u ].m_len;
        const uint32 t = cigar_vector[ cigar_length - i - 1u ].m_type;

        NVBIO_CUDA_ASSERT_IF( params.debug.asserts, (t == io::Cigar::SUBSTITUTION) || (t == io::Cigar::DELETION) || (t == io::Cigar::INSERTION) || (t == io::Cigar::SOFT_CLIPPING), "finish_alignment(%s): unknown op!\n  read[%u], mate[%u], op[%u]: %u\n", stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.read_id, context.mate, i, t );

        const uint8 t_mask = 1u << t;

        // handle deletions & insertions
        if (t_mask & (INS_MASK | DEL_MASK | CLP_MASK))
        //if (t_mask & DEL_MASK) // track deletions only
        {
            NVBIO_CUDA_ASSERT_IF( params.debug.asserts, mds_len+2 < MDS_LMEM, "finish_alignment(%s): exceeded MDS length!\n  read[%u], mate[%u], mds: %u!\n", stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.read_id, context.mate, mds_len+2 );
            mds_op = t_mask & DEL_MASK ? io::MDS_DELETION : io::MDS_INSERTION;
            mds_local[ mds_len++ ] = mds_op;
            mds_local[ mds_len++ ] = l;

            NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_traceback( context.read_id ), "\n%c[%03u] ", t_mask & DEL_MASK ? 'D' : 'I', l);
            NVBIO_CUDA_ASSERT_IF( params.debug.asserts, l, "finish_alignment(%s): zero-length MDS[%u:%u] (%c)\n", context.read_id, mds_len-1, t_mask & DEL_MASK ? 'D' : 'I', l);
        }

        for (uint32 n = 0; n < l; ++n)
        {
            // advance j and k
            j += (t_mask & (SUB_MASK | INS_MASK | CLP_MASK)) ? 1u : 0u;
            k += (t_mask & (SUB_MASK | DEL_MASK))            ? 1u : 0u;
            NVBIO_CUDA_ASSERT_IF( params.debug.asserts, (j <= read_len) && (k <= g_len), "finish_alignment(%s): coordinates out-of-bounds!\n  read[%u], mate[%u], (%u,%u) > (%u,%u) @ %u\n",  stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.read_id, context.mate, j, k, read_len, g_len, context.genome_begin );
            NVBIO_CUDA_ASSERT_IF( params.debug.asserts, (t_mask & (SUB_MASK | INS_MASK | CLP_MASK)) ? (j > 0) : true, "finish_alignment(%s): accessed read at -1!\n  read[%u], mate[%u]\n",   stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.read_id, context.mate );
            NVBIO_CUDA_ASSERT_IF( params.debug.asserts, (t_mask & (SUB_MASK | DEL_MASK))            ? (k > 0) : true, "finish_alignment(%s): accessed genome at -1!\n  read[%u], mate[%u]\n", stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.read_id, context.mate );

            const uint8 readc = j > 0 ? strings.pattern[j-1] : 255u;
            const uint8 refc  = k > 0 ? strings.text[k-1]    : 255u;

            if (t_mask == SUB_MASK)
            {
                if (readc == refc)
                {
                    NVBIO_CUDA_ASSERT_IF( params.debug.asserts, mds_len < MDS_LMEM, "finish_alignment(%s): exceeded MDS length!\n  read[%u], mds: %u!\n", stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.read_id, mds_len+2 );

                    // handle the most common case first: a character match
                    if (mds_op == io::MDS_MATCH && (mds_local[ mds_len-1 ] < 255))
                    {
                        // prolong the sequence of previous matches
                        mds_local[ mds_len-1 ]++;
                        NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_traceback( context.read_id ), "%c", dna_to_char(readc) );
                    }
                    else
                    {
                        // start a new sequence of matches
                        NVBIO_CUDA_ASSERT_IF( params.debug.asserts, mds_len+2 < MDS_LMEM, "finish_alignment(%s): exceeded MDS length!\n  read[%u], mds: %u!\n", stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.read_id, mds_len+2 );
                        mds_local[ mds_len++ ] = mds_op = io::MDS_MATCH;
                        mds_local[ mds_len++ ] = 1;
                        NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_traceback( context.read_id ), "\nM[%03u] %c", l, dna_to_char(readc) );
                    }
                }
                else
                {
                    // handle mismatches
                    NVBIO_CUDA_ASSERT_IF( params.debug.asserts, mds_len+2 < MDS_LMEM, "finish_alignment(%s): exceeded MDS length!\n  read[%u], mds: %u!\n", stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.read_id, mds_len+2 );
                    mds_local[ mds_len++ ] = mds_op = io::MDS_MISMATCH;
                    mds_local[ mds_len++ ] = readc;
                    NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_traceback( context.read_id ), "\nS[%03u] %c", l, dna_to_char(readc) );
                    ++ed;
                }
            }
            else //if (t_mask & DEL_MASK) // track deletions only
            {
                // handle all the rare cases together: deletions/insertions
                NVBIO_CUDA_ASSERT_IF( params.debug.asserts, mds_len+1 < MDS_LMEM, "finish_alignment(%s): exceeded MDS length!\n  read[%u], mds: %u!\n", stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.read_id, mds_len+1 );
                mds_local[ mds_len++ ] = t_mask & DEL_MASK ? refc : readc;

                if (t_mask != CLP_MASK) // don't count soft-clipping in the edit-distance calculation
                    ++ed;

                NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_traceback( context.read_id ), "%c", dna_to_char( (t_mask & DEL_MASK) ? refc : readc ) );
            }

            if (t_mask == SUB_MASK)
            {
                // score the substitution
                const uint8 q = strings.quals[j-1];
                const int32 delta = scoring_scheme.score( readc, 1u << refc, q );
                score += delta;
                NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_traceback( context.read_id ) && delta, " S(%d,%d)", delta, q );
            }
        }

        if      (t_mask == INS_MASK) { score -= scoring_scheme.cumulative_deletion( l );  NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_traceback( context.read_id ), " D(%d)", scoring_scheme.cumulative_deletion( l ) ); }
        else if (t_mask == DEL_MASK) { score -= scoring_scheme.cumulative_insertion( l ); NVBIO_CUDA_DEBUG_PRINT_IF( params.debug.show_traceback( context.read_id ), " I(%d)", scoring_scheme.cumulative_insertion( l ) ); }
    }

  #if 0
    #if defined(NVBIO_CUDA_DEBUG)
    if (params.debug.show_traceback( context.read_id ))
    {
        char read_str[512];
        char gen_str[512];
        char cigar_str[512];
        char mm_string[512];

        dna_to_string( strings.pattern, read_len, read_str );
        dna_to_string( strings.text,    g_len,    gen_str );

        for (uint32 i = 0; i < read_len; ++i)
            mm_string[i] = '0' - int32( scoring_scheme.mismatch( strings.quals[i] ) );
        mm_string[read_len] = '\0';

        uint32 j = 0;
        for (uint32 i = 0; i < cigar_coord.y; ++i)
        {
            const uint32 l = cigar_vector[ cigar_coord.y - i - 1u ].m_len;
            const uint32 t = cigar_vector[ cigar_coord.y - i - 1u ].m_type;
            for (uint32 n = 0; n < l; ++n)
                cigar_str[j++] = "MID"[t];
        }
        cigar_str[j] = '\0';
        NVBIO_CUDA_DEBUG_PRINT("\nfinish_alignment:\n  score: %d, %d, ed: %u, (genome offset: %u)\n  read  : %s\n  genome: %s\n  quals : %s\n  cigar : %s\n  mds len: %u\n", alignment.score(), score, ed, cigar_offset, read_str, gen_str, mm_string, cigar_str, mds_len);
    }
    #endif
  #endif

    // store the MDS length in the first two bytes
    NVBIO_CUDA_ASSERT_IF( params.debug.asserts, mds_len < 65536, "finish_alignment(%s): exceeded representable MDS length!\n  aln[%u], read[%u], mate[%u], mds: %u!\n", stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.idx, context.read_id, context.mate, mds_len );
    mds_local[0] = uint8( mds_len & 0xFF );
    mds_local[1] = uint8( mds_len >> 8 );

    // alloc the output mds
    uint8* mds_vector = pipeline.mds.alloc( context.idx, mds_len );
    NVBIO_CUDA_ASSERT_IF( params.debug.asserts, mds_vector != NULL, "finish_alignment(%s): unable to allocate MDS!\n  aln[%u], read[%u], mate[%u], mds: %u!\n", stream.mate_type() == OppositeMate ? "opposite" : "anchor", context.idx, context.read_id, context.mate, mds_len );
    if (mds_vector)
    {
        // copy the local mds to the output one
        for (uint32 i = 0; i < mds_len; ++i)
            mds_vector[i] = mds_local[i];
    }

    // output the finished alignment
    stream.finish( work_id, &context, ed, score );
}

///
/// finish computing the final alignments, i.e. computing the MD strings and the final scores (given the CIGARs)
///
template <uint32 ALN_IDX, typename scheme_type, typename pipeline_type>
void finish_alignment_best(
    const MateType              mate_type,
    const uint32                count,
    const uint32*               idx,
          io::Alignment*        best_data,
    const uint32                best_stride,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const scheme_type           scoring_scheme,
    const ParamsPOD             params)
{
    const uint32 static_band_len = 
        (band_len < 4)  ? 3u  :
        (band_len < 8)  ? 7u  :
        (band_len < 16) ? 15u :
                          31u;

    typedef typename pipeline_type::scheme_type::local_aligner_type aligner_type;

    typedef BestTracebackStream<ALN_IDX,aligner_type,pipeline_type> stream_type;

    stream_type stream(
        mate_type,
        count,
        idx,
        best_data,
        best_stride,
        static_band_len,
        pipeline,
        pipeline.scoring_scheme.local_aligner(),
        params );

    const uint32 n_blocks = (count + BLOCKDIM-1) / BLOCKDIM;

    finish_alignment_kernel<<<n_blocks, BLOCKDIM>>>(
        stream,
        pipeline,
        scoring_scheme,
        params );

    // check for overflows
    //if (mds.allocated_size() >= mds.arena_size())
    //    log_error( stderr, "finish_alignment(): exceeded MDS pool size\n" );
}

///
/// finish computing the final alignments, i.e. computing the MD strings and the final scores (given the CIGARs)
///
template <typename scheme_type, typename pipeline_type>
void finish_alignment_all(
    const uint32                count,
    const uint32*               idx,
    const uint32                buffer_offset,
    const uint32                buffer_size,
          io::Alignment*        alignments,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const scheme_type           scoring_scheme,
    const ParamsPOD             params)
{
    const uint32 static_band_len = 
        (band_len < 4)  ? 3u  :
        (band_len < 8)  ? 7u  :
        (band_len < 16) ? 15u :
                          31u;

    typedef typename pipeline_type::scheme_type::local_aligner_type aligner_type;

    typedef AllTracebackStream<aligner_type,pipeline_type> stream_type;

    stream_type stream(
        count,
        idx,
        buffer_offset,
        buffer_size,
        alignments,
        static_band_len,
        pipeline,
        pipeline.scoring_scheme.local_aligner(),
        params );

    const uint32 n_blocks = (count + BLOCKDIM-1) / BLOCKDIM;

    finish_alignment_kernel<<<n_blocks, BLOCKDIM>>>(
        stream,
        pipeline,
        scoring_scheme,
        params );

    // check for overflows
    //if (mds.allocated_size() >= mds.arena_size())
    //    log_error( stderr, "finish_alignment(): exceeded MDS pool size\n" );
}

///@}  // group TracebackDetail
///@}  // group Traceback
///@}  // group nvBowtie

} // namespace detail

//
// execute a batch of banded-alignment traceback calculations
//
template <uint32 ALN_IDX, typename pipeline_type>
void banded_traceback_best_t(
    const uint32                count,
    const uint32*               idx,
          io::Alignment*        best_data,
    const uint32                best_stride,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const ParamsPOD             params)
{
    if (params.alignment_type == LocalAlignment)
    {
        detail::banded_traceback_best<ALN_IDX>(
            AnchorMate,
            count,
            idx,
            best_data,
            best_stride,
            band_len,
            pipeline,
            pipeline.scoring_scheme.local_aligner(),
            params );
    }
    else
    {
        detail::banded_traceback_best<ALN_IDX>(
            AnchorMate,
            count,
            idx,
            best_data,
            best_stride,
            band_len,
            pipeline,
            pipeline.scoring_scheme.end_to_end_aligner(),
            params );
    }
}

//
// execute a batch of opposite alignment traceback calculations
//
template <uint32 ALN_IDX, typename pipeline_type>
void opposite_traceback_best_t(
    const uint32                count,
    const uint32*               idx,
          io::Alignment*        best_data,
    const uint32                best_stride,
    const pipeline_type&        pipeline,
    const ParamsPOD             params)
{
    if (params.alignment_type == LocalAlignment)
    {
        detail::traceback_best<ALN_IDX>(
            OppositeMate,
            count,
            idx,
            best_data,
            best_stride,
            pipeline,
            pipeline.scoring_scheme.local_aligner(),
            params );
    }
    else
    {
         detail::traceback_best<ALN_IDX>(
            OppositeMate,
            count,
            idx,
            best_data,
            best_stride,
            pipeline,
            pipeline.scoring_scheme.end_to_end_aligner(),
            params );
    }
}

//
// execute a batch of banded-alignment traceback calculations
//
template <typename pipeline_type>
void banded_traceback_all_t(
    const uint32                count,
    const uint32*               idx,
    const uint32                buffer_offset,
    const uint32                buffer_size,
          io::Alignment*        alignments,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const ParamsPOD             params)
{
    if (params.alignment_type == LocalAlignment)
    {
        detail::banded_traceback_all(
            count,
            idx,
            buffer_offset,
            buffer_size,
            alignments,
            band_len,
            pipeline,
            pipeline.scoring_scheme.local_aligner(),
            params );
    }
    else
    {
        detail::banded_traceback_all(
            count,
            idx,
            buffer_offset,
            buffer_size,
            alignments,
            band_len,
            pipeline,
            pipeline.scoring_scheme.end_to_end_aligner(),
            params );
    }
}

//
// finish a batch of alignment calculations
//
template <uint32 ALN_IDX, typename scoring_scheme_type, typename pipeline_type>
void finish_alignment_best_t(
    const uint32                count,
    const uint32*               idx,
          io::Alignment*        best_data,
    const uint32                best_stride,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const scoring_scheme_type   scoring_scheme,
    const ParamsPOD             params)
{
    detail::finish_alignment_best<ALN_IDX>(
        AnchorMate,
        count,
        idx,
        best_data,
        best_stride,
        band_len,
        pipeline,
        scoring_scheme,
        params );
}

//
// finish a batch of opposite alignment calculations
//
template <uint32 ALN_IDX, typename scoring_scheme_type, typename pipeline_type>
void finish_opposite_alignment_best_t(
    const uint32                count,
    const uint32*               idx,
          io::Alignment*        best_data,
    const uint32                best_stride,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const scoring_scheme_type   scoring_scheme,
    const ParamsPOD             params)
{
    detail::finish_alignment_best<ALN_IDX>(
        OppositeMate,
        count,
        idx,
        best_data,
        best_stride,
        band_len,
        pipeline,
        scoring_scheme,
        params );
}

//
// finish a batch of alignment calculations
//
template <typename scoring_scheme_type, typename pipeline_type>
void finish_alignment_all_t(
    const uint32                count,
    const uint32*               idx,
    const uint32                buffer_offset,
    const uint32                buffer_size,
          io::Alignment*        alignments,
    const uint32                band_len,
    const pipeline_type&        pipeline,
    const scoring_scheme_type   scoring_scheme,
    const ParamsPOD             params)
{
    detail::finish_alignment_all(
        count,
        idx,
        buffer_offset,
        buffer_size,
        alignments,
        band_len,
        pipeline,
        scoring_scheme,
        params );
}


} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
