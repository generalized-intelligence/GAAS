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

#include <nvbio/alignment/sink.h>
#include <nvbio/alignment/utils.h>
#include <nvbio/alignment/alignment_base_inl.h>
#include <nvbio/alignment/sw/sw_inl.h>
#include <nvbio/alignment/ed/ed_utils.h>


namespace nvbio {
namespace aln {

namespace priv
{

///@addtogroup private
///@{

///
/// Calculate the alignment score between a pattern and a text, using the Smith-Waterman algorithm.
///
/// \tparam TYPE                the alignment type
/// \tparam pattern_string      pattern string 
/// \tparam quals_string        pattern qualities
/// \tparam text_string         text string
/// \tparam column_type         temporary column storage
///
template <
    AlignmentType   TYPE,
    typename        algorithm_tag,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        column_type>
struct alignment_score_dispatch<
    EditDistanceAligner<TYPE,algorithm_tag>,
    pattern_string,
    qual_string,
    text_string,
    column_type>
{
    typedef EditDistanceAligner<TYPE,algorithm_tag> aligner_type;

    /// dispatch scoring across the whole pattern
    ///
    /// \param aligner      scoring scheme
    /// \param pattern      pattern string (horizontal
    /// \param quals        pattern qualities
    /// \param text         text string (vertical)
    /// \param min_score    minimum score
    /// \param sink         output alignment sink
    ///
    /// \return             true iff the minimum score was reached
    ///
    template <typename sink_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static bool dispatch(
        const aligner_type      aligner,
        const pattern_string    pattern,
        const qual_string       quals,
        const text_string       text,
        const  int32            min_score,
              sink_type&        sink,
              column_type       column)
    {
        typedef typename pattern_string::value_type    symbol_type;

        NVBIO_VAR_UNUSED const uint32 BAND_LEN = sw_bandlen_selector<TYPE,1u,symbol_type>::BAND_LEN;

        priv::SWScoringContext<BAND_LEN,TYPE,algorithm_tag> context;

        const uint32 length = equal<algorithm_tag,PatternBlockingTag>() ? pattern.length() : text.length();

        return sw_alignment_score_dispatch<BAND_LEN,TYPE,algorithm_tag,symbol_type>::run( EditDistanceSWScheme(), context, pattern, quals, text, min_score, sink, 0, length, column );
    }

    /// dispatch scoring in a window of the pattern
    ///
    /// \tparam checkpoint_type     a class to represent the checkpoint: an array of size equal to the text,
    ///                             that has to provide the const indexing operator[].
    ///
    /// \param aligner      scoring scheme
    /// \param pattern      pattern string (horizontal
    /// \param quals        pattern qualities
    /// \param text         text string (vertical)
    /// \param min_score    minimum score
    /// \param sink         output alignment sink
    /// \param checkpoint   in/out checkpoint
    ///
    /// \return             true iff the minimum score was reached
    ///
    template <
        typename sink_type,
        typename checkpoint_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static bool dispatch(
        const aligner_type      aligner,
        const pattern_string    pattern,
        const qual_string       quals,
        const text_string       text,
        const  int32            min_score,
        const uint32            window_begin,
        const uint32            window_end,
              sink_type&        sink,
        checkpoint_type         checkpoint,
              column_type       column)
    {
        typedef typename pattern_string::value_type    symbol_type;

        NVBIO_VAR_UNUSED const uint32 BAND_LEN = sw_bandlen_selector<TYPE,1u,symbol_type>::BAND_LEN;

        priv::SWCheckpointedScoringContext<BAND_LEN,TYPE,algorithm_tag,checkpoint_type> context( checkpoint );

        return sw_alignment_score_dispatch<BAND_LEN,TYPE,algorithm_tag,symbol_type>::run( EditDistanceSWScheme(), context, pattern, quals, text, min_score, sink, window_begin, window_end, column );
    }

    /// dispatch scoring in a window of the pattern, retaining the intermediate results in the column
    /// vector, essentially used as a continuation
    ///
    /// \param aligner      scoring scheme
    /// \param pattern      pattern string (horizontal
    /// \param quals        pattern qualities
    /// \param text         text string (vertical)
    /// \param min_score    minimum score
    /// \param sink         output alignment sink
    ///
    /// \return             true iff the minimum score was reached
    ///
    template <typename sink_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static bool dispatch(
        const aligner_type      aligner,
        const pattern_string    pattern,
        const qual_string       quals,
        const text_string       text,
        const  int32            min_score,
        const uint32            window_begin,
        const uint32            window_end,
              sink_type&        sink,
              column_type       column)
    {
        typedef typename pattern_string::value_type    symbol_type;

        NVBIO_VAR_UNUSED const uint32 BAND_LEN = sw_bandlen_selector<TYPE,1u,symbol_type>::BAND_LEN;

        priv::SWScoringContext<BAND_LEN,TYPE,algorithm_tag> context;

        return sw_alignment_score_dispatch<BAND_LEN,TYPE,algorithm_tag,symbol_type>::run( EditDistanceSWScheme(), context, pattern, quals, text, min_score, sink, window_begin, window_end, column );
    }
};

///
/// Calculate the alignment score between a pattern and a text, using the Smith-Waterman algorithm.
///
/// \tparam TYPE                the alignment type
/// \tparam pattern_string      pattern string 
/// \tparam quals_string        pattern qualities
/// \tparam text_string         text string
/// \tparam column_type         temporary column storage
///
template <
    uint32          CHECKPOINTS,
    AlignmentType   TYPE,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        column_type>
struct alignment_checkpointed_dispatch<
    CHECKPOINTS,
    EditDistanceAligner<TYPE>,
    pattern_string,
    qual_string,
    text_string,
    column_type>
{
    typedef EditDistanceAligner<TYPE> aligner_type;

    ///
    /// Calculate a set of checkpoints of the DP matrix for the alignment between a pattern
    /// and a text, using the edit distance.
    ///
    /// \tparam checkpoint_type     a class to represent the collection of checkpoints,
    ///                             represented as a linear array storing each checkpointed
    ///                             band contiguously.
    ///                             The class has to provide the const indexing operator[].
    ///
    /// \param aligner      scoring scheme
    /// \param pattern      pattern string (horizontal
    /// \param quals        pattern qualities
    /// \param text         text string (vertical)
    /// \param min_score    minimum score
    /// \param sink         output alignment sink
    /// \param checkpoints  output checkpoints
    ///
    /// \return             true iff the minimum score was reached
    ///
    template <
        typename    sink_type,
        typename    checkpoint_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static
    void dispatch_checkpoints(
        const aligner_type      aligner,
        const pattern_string    pattern,
        const qual_string       quals,
        const text_string       text,
        const  int32            min_score,
              sink_type&        sink,
        checkpoint_type         checkpoints,
              column_type       column)
    {
        typedef typename pattern_string::value_type    symbol_type;

        NVBIO_VAR_UNUSED const uint32 BAND_LEN = sw_bandlen_selector<TYPE,1u,symbol_type>::BAND_LEN;

        priv::SWCheckpointContext<BAND_LEN,TYPE,CHECKPOINTS,checkpoint_type> context( checkpoints );

        sw_alignment_score_dispatch<BAND_LEN,TYPE,PatternBlockingTag,symbol_type>::run( EditDistanceSWScheme(), context, pattern, quals, text, min_score, sink, 0, pattern.length(), column );
    }

    ///
    /// Compute the banded Dynamic Programming submatrix between two given checkpoints,
    /// storing its flow at each cell.
    /// The function returns the submatrix width.
    ///
    /// \tparam BAND_LEN            size of the DP band
    ///
    /// \tparam checkpoint_type     a class to represent the collection of checkpoints,
    ///                             represented as a linear array storing each checkpointed
    ///                             band contiguously.
    ///                             The class has to provide the const indexing operator[].
    ///
    /// \tparam submatrix_type      a class to store the flow submatrix, represented
    ///                             as a linear array of size (BAND_LEN*CHECKPOINTS).
    ///                             The class has to provide the non-const indexing operator[].
    ///                             Note that the submatrix entries can assume only 3 values,
    ///                             and could hence be packed in 2 bits.
    ///
    /// \param checkpoints          the set of checkpointed rows
    /// \param checkpoint_id        the starting checkpoint used as the beginning of the submatrix
    /// \param submatrix            the output submatrix
    ///
    /// \return                     the submatrix width
    ///
    template <
        typename      checkpoint_type,
        typename      submatrix_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static
    uint32 dispatch_submatrix(
        const aligner_type      aligner,
        const pattern_string    pattern,
        const qual_string       quals,
        const text_string       text,
        const int32             min_score,
        checkpoint_type         checkpoints,
        const uint32            checkpoint_id,
        submatrix_type          submatrix,
        column_type             column)
    {
        typedef typename pattern_string::value_type     symbol_type;

        NVBIO_VAR_UNUSED const uint32 BAND_LEN = sw_bandlen_selector<TYPE,1u,symbol_type>::BAND_LEN;

        priv::SWSubmatrixContext<BAND_LEN,TYPE,CHECKPOINTS,checkpoint_type,submatrix_type>
            context( checkpoints, checkpoint_id, submatrix );

        const uint32 window_begin = checkpoint_id * CHECKPOINTS;
        const uint32 window_end   = nvbio::min( window_begin + CHECKPOINTS, uint32(pattern.length()) );

        NullSink null_sink;
        sw_alignment_score_dispatch<BAND_LEN,TYPE,PatternBlockingTag,symbol_type>::run( EditDistanceSWScheme(), context, pattern, quals, text, min_score, null_sink, window_begin, window_end, column );

        return window_end - window_begin;
    }
};

///
/// Given the Dynamic Programming submatrix between two checkpoints,
/// backtrace from a given destination cell, using edit distance.
/// The function returns the resulting source cell.
///
/// \tparam BAND_LEN            size of the DP band
///
/// \tparam CHECKPOINTS         number of DP rows between each checkpoint
///
/// \tparam checkpoint_type     a class to represent the collection of checkpoints,
///                             represented as a linear array storing each checkpointed
///                             band contiguously.
///                             The class has to provide the const indexing operator[].
///
/// \tparam submatrix_type      a class to store the flow submatrix, represented
///                             as a linear array of size (BAND_LEN*CHECKPOINTS).
///                             The class has to provide the const indexing operator[].
///                             Note that the submatrix entries can assume only 3 values,
///                             and could hence be packed in 2 bits.
///
/// \tparam output_type         a class to store the resulting list of backtracking operations.
///                             Needs to provide a single method:
///                                 void push(uint8 op)
///
/// \param checkpoints          precalculated checkpoints
/// \param checkpoint_id        index of the first checkpoint defining the DP submatrix,
///                             storing all bands between checkpoint_id and checkpoint_id+1.
/// \param submatrix            precalculated flow submatrix
/// \param submatrix_height     submatrix width
/// \param submatrix_height     submatrix height
/// \param sink                 in/out sink of the DP solution
/// \param output               backtracking output handler
///
/// \return                     true if the alignment source has been found, false otherwise
///
template <
    uint32          CHECKPOINTS,
    AlignmentType   TYPE,
    typename        checkpoint_type,
    typename        submatrix_type,
    typename        backtracer_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool alignment_traceback(
    const EditDistanceAligner<TYPE>   aligner,
    checkpoint_type                   checkpoints,
    const uint32                      checkpoint_id,
    submatrix_type                    submatrix,
    const uint32                      submatrix_width,
    const uint32                      submatrix_height,
          uint8&                      state,
          uint2&                      sink,
    backtracer_type&                  backtracer)
{
    return alignment_traceback<CHECKPOINTS>(
        make_smith_waterman_aligner<TYPE>( EditDistanceSWScheme() ),
        checkpoints,
        checkpoint_id,
        submatrix,
        submatrix_width,
        submatrix_height,
        state,
        sink,
        backtracer );
}

/// @} // end of private group

} // namespace priv

} // namespace aln
} // namespace nvbio
