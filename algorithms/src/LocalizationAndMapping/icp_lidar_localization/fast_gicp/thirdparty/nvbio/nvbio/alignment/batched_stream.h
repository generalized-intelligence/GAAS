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

#include <nvbio/basic/types.h>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/alignment/utils.h>
#include <nvbio/basic/cuda/work_queue.h>
#include <nvbio/basic/strided_iterator.h>

namespace nvbio {
namespace aln {

///@addtogroup private
///@{

///
/// A work-stream class to be used with a WorkQueue. This class implements
/// the context to fetch and run parallel alignment jobs.
///
/// In particular, it provides utilities for accessing all the temporary alignment
/// storage bound to a given queue slot, which is represented by vectors with
/// a strided memory layout in order to favor read/write coalescing.
///
/// \tparam stream_type         the underlying BatchedAlignmentScore stream_type (see \ref BatchAlignment
///                             and BatchedAlignmentScore).
///
template <typename stream_type>
struct ScoreStream
{
    typedef typename stream_type::aligner_type                      aligner_type;       ///< the aligner type
    typedef typename checkpoint_storage_type<aligner_type>::type    cell_type;          ///< the DP cell type
    typedef strided_iterator<cell_type*>                            checkpoint_type;    ///< the strided DP checkpoint type
    typedef strided_iterator<cell_type*>                            column_type;        ///< the strided DP column type

    /// constructor
    ///
    ScoreStream(const stream_type _stream, uint8* _columns, uint8* _checkpoints, const uint32 _stride) :
        stream(_stream),
        columns( (cell_type*)_columns ),
        checkpoints( (cell_type*)_checkpoints ),
        stride( _stride )
    {}

    /// stream size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return stream.size(); }

    /// return the i-th checkpoint
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    checkpoint_type checkpoint(const uint32 i) const
    {
        return checkpoint_type( checkpoints + i, stride );
    }

    /// return the i-th checkpoint
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    column_type column(const uint32 i) const
    {
        return column_type( columns + i, stride );
    }

    /// get the i-th work unit
    ///
    template <typename score_unit_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void get(const uint32 job_id, score_unit_type* unit, const uint2 queue_slot) const
    {
        // setup the given work unit
        unit->setup( job_id, queue_slot.x, stream );
    }

    stream_type stream;         ///< alignment stream
    cell_type*  columns;        ///< temporary columns storage
    cell_type*  checkpoints;    ///< temporary checkpoints storage
    uint32      stride;         ///< memory stride
};

///
/// This class provides the fundamental mechanisms to perform a staged job.
/// In order to allow for configuration, it is templated over a derived_type
/// which must implement the method:
///
/// \code
/// bool execute(
///     const ScoreStream<stream_type>& stream,
///     const uint32                    queue_slot,
///     pattern_string                  pattern,
///     qual_string                     quals,
///     text_string                     text,
///     const int32                     min_score,
///     const uint32                    window_begin,
///     const uint32                    window_end,
///     sink_type&                      sink);
/// \endcode
///
template <typename stream_type, typename derived_type>
struct StagedAlignmentUnitBase
{
    static const uint32 WINDOW_SIZE = 32;

    typedef typename stream_type::context_type                  context_type;
    typedef typename stream_type::strings_type                  strings_type;
    typedef typename stream_type::aligner_type                  aligner_type;

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void setup(const uint32 _job_id, const uint32 _queue_slot, const stream_type& stream)
    {
        job_id       = _job_id;
        queue_slot   = _queue_slot;
        window_begin = 0u;

        // load the alignment context
        valid = stream.init_context( job_id, &context );

        // load the strings to be aligned
        if (valid)
        {
            const uint32 len = equal<typename aligner_type::algorithm_tag,PatternBlockingTag>() ?
                stream.pattern_length( job_id, &context ) :
                stream.text_length( job_id, &context );
            stream.load_strings( job_id, 0, len, &context, &strings );
        }
    }

    // run method
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool run(const ScoreStream<stream_type>& score_stream)
    {
        if (valid)
        {
            // compute the end of the current DP matrix window
            const uint32 len = equal<typename aligner_type::algorithm_tag,PatternBlockingTag>() ?
                strings.pattern.length() :
                strings.text.length();
            const uint32 window_end  = nvbio::min( window_begin + WINDOW_SIZE, len );

            valid = (static_cast<derived_type*>(this))->execute(
                score_stream,
                queue_slot,
                strings.pattern,
                strings.quals,
                strings.text,
                context.min_score,
                window_begin,
                window_end,
                context.sink );

            if (window_end >= len)
                valid = false;

            // update the pattern window
            window_begin = window_end;
        }

        // check whether we are done processing the pattern
        if (valid == false)
        {
            score_stream.stream.output( job_id, &context );
            return false;
        }
        return true;
    }

    uint32          job_id;            ///< the job id
    uint32          queue_slot;        ///< the job's execution slot
    context_type    context;           ///< the alignment context
    strings_type    strings;           ///< the strings to be aligned
    volatile uint32 window_begin;      ///< the beginning of the pattern window
    volatile bool   valid;             ///< valid flag
};

///
/// A staged scoring work unit, implemented inheriting from StagedAlignmentUnitBase
///
template <typename stream_type>
struct StagedScoreUnit :
    public StagedAlignmentUnitBase<
        stream_type,                    // pass the stream_type
        StagedScoreUnit<stream_type> >  // specify self as the derived_type
{
    typedef typename ScoreStream<stream_type>::column_type column_type;

    // execute method
    template <
        typename pattern_string,
        typename qual_string,
        typename text_string,
        typename sink_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool execute(
        const ScoreStream<stream_type>& score_stream,
        const uint32                    queue_slot,
        const pattern_string            pattern,
        const qual_string               quals,
        const text_string               text,
        const int32                     min_score,
        const uint32                    window_begin,
        const uint32                    window_end,
        sink_type&                      sink)
    {
        // fetch this job's temporary storage
        column_type column = score_stream.column( queue_slot );

        // score the current DP matrix window
        return alignment_score(
            score_stream.stream.aligner(),
            pattern,
            quals,
            text,
            min_score,
            window_begin,
            window_end,
            sink,
            column );
    }
};

///
/// A staged scoring work unit, implemented inheriting from StagedAlignmentUnitBase
///
template <uint32 BAND_LEN, typename stream_type>
struct BandedScoreUnit :
    public StagedAlignmentUnitBase<
        stream_type,                                // pass the stream_type
        BandedScoreUnit<BAND_LEN,stream_type> >     // specify self as the derived_type
{
    typedef typename ScoreStream<stream_type>::cell_type        cell_type;
    typedef typename ScoreStream<stream_type>::checkpoint_type  checkpoint_type;
    typedef typename ScoreStream<stream_type>::column_type      column_type;

    // execute method
    template <
        typename pattern_string,
        typename qual_string,
        typename text_string,
        typename sink_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool execute(
        const ScoreStream<stream_type>& score_stream,
        const uint32                    queue_slot,
        const pattern_string            pattern,
        const qual_string               quals,
        const text_string               text,
        const int32                     min_score,
        const uint32                    window_begin,
        const uint32                    window_end,
        sink_type&                      sink)
    {
        // fetch this job's temporary storage
        column_type column = score_stream.column( queue_slot );

        // score the current DP matrix window
        return banded_alignment_score<BAND_LEN>(
            score_stream.stream.aligner(),
            pattern,
            quals,
            text,
            min_score,
            window_begin,
            window_end,
            sink,
            column );
    }
};

///@} // end of private group

///@} // end of BatchAlignment group

///@} // end of the Alignment group

} // namespace alignment
} // namespace nvbio
