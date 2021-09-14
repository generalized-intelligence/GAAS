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
#include <nvbio/basic/numbers.h>

namespace nvbio {
namespace aln {

namespace priv {
namespace banded {

///@addtogroup private
///@{

// initialize the zero-th row of the banded DP-matrix with Smith-Waterman scoring
//
template <uint32 BAND_LEN, AlignmentType TYPE, typename band_type, typename scoring_type, typename score_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void init_row_zero(
          band_type&    band,
    const scoring_type& scoring,
    const score_type    infimum)
{
    #pragma unroll
    for (uint32 j = 0; j < BAND_LEN; ++j)
        band[j] =  TYPE == GLOBAL ? j * scoring.deletion() : 0;
}

///
/// A helper scoring context class, which can be used to adapt the basic
/// sw_alignment_score_dispatch algorithm to various situations, such as:
///   scoring
///   scoring within a window (i.e. saving only the last band within the window)
///   computing checkpoints
///   computing a flow submatrix
///
template <uint32 BAND_LEN, AlignmentType TYPE>
struct SmithWatermanScoringContext
{
    template <typename band_type, typename scoring_type, typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void init(
        const uint32        i,
              band_type&    band,
        const scoring_type& scoring,
        const score_type    infimum)
    {
        init_row_zero<BAND_LEN,TYPE>( band, scoring, infimum );
    }

    template <typename band_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void previous_row(
        const uint32        i,
        const band_type&    band) {}

    template <typename band_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void last_row(
        const uint32        i,
        const band_type&    band) {}

    template <typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void new_cell(
        const uint32            i,
        const uint32            j,
        const score_type        score,
        const DirectionVector   dir) {}
};

///
/// A helper scoring context class, instantiated to configure sw_alignment_score_dispatch
/// to perform scoring in windows
///
template <uint32 BAND_LEN, AlignmentType TYPE, typename checkpoint_type>
struct SmithWatermanCheckpointedScoringContext
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SmithWatermanCheckpointedScoringContext(checkpoint_type checkpoints) :
        m_checkpoints( checkpoints ) {}

    template <typename band_type, typename scoring_type, typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void init(
        const uint32        i,
              band_type&    band,
        const scoring_type& scoring,
        const score_type    infimum)
    {
        // check whether this is the first row
        if (i == 0)
            init_row_zero<BAND_LEN,TYPE>( band, scoring, infimum );
        else
        {
            // load from checkpoint
            #pragma unroll
            for (uint32 j = 0; j < BAND_LEN; ++j)
                band[j] = m_checkpoints[j];
        }
    }

    template <typename band_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void previous_row(
        const uint32        i,
        const band_type&    band) {}

    template <typename band_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void last_row(
        const uint32        i,
        const band_type&    band)
    {
        const short infimum = Field_traits<short>::min() + 32;

        // save the last row
        #pragma unroll
        for (uint32 j = 0; j < BAND_LEN; ++j)
            m_checkpoints[j] = nvbio::max( band[j], infimum );
    }

    // dir is the backtracking arrow for the H matrix at this cell
    template <typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void new_cell(
        const uint32            i,
        const uint32            j,
        const score_type        score,
        const DirectionVector   dir) {}

    checkpoint_type  m_checkpoints;
};

///
/// A helper scoring context class, instantiated to configure sw_alignment_score_dispatch
/// to store checkpoints of the banded DP matrix along the pattern
///
template <uint32 BAND_LEN, AlignmentType TYPE, uint32 CHECKPOINTS, typename checkpoint_type>
struct SmithWatermanCheckpointContext
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SmithWatermanCheckpointContext(checkpoint_type  checkpoints) :
        m_checkpoints( checkpoints ) {}

    template <typename band_type, typename scoring_type, typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void init(
        const uint32        i,
              band_type&    band,
        const scoring_type& scoring,
        const score_type    infimum)
    {
        init_row_zero<BAND_LEN,TYPE>( band, scoring, infimum );
    }

    template <typename band_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void previous_row(
        const uint32        i,
        const band_type&    band)
    {
        // save checkpoint
        if ((i & (CHECKPOINTS-1)) == 0u)
        {
            const short infimum = Field_traits<short>::min() + 32;

            for (uint32 j = 0; j < BAND_LEN; ++j)
                m_checkpoints[BAND_LEN*(i/CHECKPOINTS) + j] = nvbio::max( band[j], infimum );
        }
    }

    template <typename band_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void last_row(
        const uint32        i,
        const band_type&    band)
    {
        const short infimum = Field_traits<short>::min() + 32;

        // save the last row
        const uint32 checkpoint_row = (i+CHECKPOINTS-1)/CHECKPOINTS;
        for (uint32 j = 0; j < BAND_LEN; ++j)
            m_checkpoints[BAND_LEN*checkpoint_row + j] = nvbio::max( band[j], infimum );
    }

    // dir is the backtracking arrow for the H matrix at this cell
    template <typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void new_cell(
        const uint32            i,
        const uint32            j,
        const score_type        score,
        const DirectionVector   dir) {}

    checkpoint_type  m_checkpoints;
};

///
/// A helper scoring context class, instantiated to configure sw_alignment_score_dispatch
/// to keep track of the direction vectors of a DP submatrix between given checkpoints
///
template <uint32 BAND_LEN, AlignmentType TYPE, uint32 CHECKPOINTS, typename checkpoint_type, typename submatrix_type>
struct SmithWatermanSubmatrixContext
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SmithWatermanSubmatrixContext(
        const checkpoint_type   checkpoints,
        const uint32            checkpoint_id,
        submatrix_type          submatrix) :
        m_checkpoints( checkpoints ),
        m_checkpoint_id( checkpoint_id ),
        m_submatrix( submatrix ) {}

    template <typename band_type, typename scoring_type, typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void init(
        const uint32        i,
              band_type&    band,
        const scoring_type& scoring,
        const score_type    infimum)
    {
        // restore the checkpoint
        #pragma unroll
        for (uint32 j = 0; j < BAND_LEN; ++j)
            band[j] = m_checkpoints[m_checkpoint_id*BAND_LEN + j];
    }

    template <typename band_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void previous_row(
        const uint32        i,
        const band_type&    band) {}

    template <typename band_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void last_row(
        const uint32        i,
        const band_type&    band) {}

    template <typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void new_cell(
        const uint32            i,
        const uint32            j,
        const score_type        score,
        const DirectionVector   dir) 
    {
        // save the direction vectors for H,E
        const uint32 offset = m_checkpoint_id * CHECKPOINTS;
        m_submatrix[ (i - offset)*BAND_LEN + j ] = dir;
    }

    checkpoint_type   m_checkpoints;
    uint32            m_checkpoint_id;
    submatrix_type    m_submatrix;
};

///
/// The template struct to dispatch calls to banded_alignment_score.
///
template <uint32 BAND_LEN, AlignmentType TYPE>
struct sw_alignment_score_dispatch
{
    ///
    /// Calculate the banded alignment score between a string
    /// and a reference, using the Smith-Waterman algorithm.
    ///
    /// \tparam context_type    a context class used to configure the behaviour
    ///                         of the algorithm, initializing and capturing the output DP matrix;
    ///                         it must implement the following interface:
    ///
    /// \code
    /// struct context_type
    /// {
    ///     // initialize the first DP band
    ///     template <typename scoring_type>
    ///     void init(
    ///         const uint32        i,
    ///               int32*        band,
    ///         const scoring_type& scoring,
    ///         const int32         infimum);
    /// 
    ///     // do anything with the i-th band
    ///     void previous_row(
    ///         const uint32        i,
    ///         const int32*        band);
    /// 
    ///     // do anything with the last computed band, at row i
    ///     void last_row(
    ///         const uint32        i,
    ///         const int32*        band);
    /// 
    ///     // do anything with the cell (i,j)
    ///     void new_cell(
    ///         const uint32            i,
    ///         const uint32            j,
    ///         const int32             score,
    ///         const DirectionVector   dir);
    /// };
    /// \endcode
    ///
    /// \param scoring      scoring scheme
    /// \param pattern      shorter string (horizontal)
    /// \param text         longer string (vertical)
    /// \param pos          offset in the reference string
    /// \param context      the context class
    /// \param sink         output alignment sink
    ///
    /// \return             false if the alignment didn't reach the
    ///                     minimum score, true otherwise
    ///
    template <
        typename pattern_type,
        typename qual_type,
        typename text_type,
        typename scoring_type,
        typename context_type,
        typename sink_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static
    bool run(
        const scoring_type& scoring,
        pattern_type        pattern,
        qual_type           quals,
        text_type           text,
        const uint32        window_begin,
        const uint32        window_end,
        const uint32        pos,
        const int32         min_score,
        context_type&       context,
        sink_type&          sink)
    {
        const uint32 pattern_len = pattern.length();
        const uint32 text_len = text.length();
        const uint32 start = pos;

        if (text_len < pattern_len)
            return false;

        typedef int32 score_type;

        typedef typename Reference_cache<BAND_LEN>::type text_cache_type;
        uint32 text_cache_storage[Reference_cache<BAND_LEN>::BAND_WORDS];
        text_cache_type text_cache( &text_cache_storage[0] );

        // load first band of text
        for (uint32 j = 0; j < BAND_LEN-1; ++j)
            text_cache[j] = text[start+window_begin+j];

        const score_type G = scoring.deletion();
        const score_type I = scoring.insertion();
        const score_type infimum = Field_traits<short>::min() + nvbio::max( G, I );

        // initialize the first band (corresponding to the 0-th row of the DP matrix)
        score_type band[BAND_LEN];

        // initialize band
        context.init(
            window_begin,
            band,
            scoring,
            infimum );

        // loop across the short edge of the DP matrix: each band is a segment of the long columns
        for (uint32 i = window_begin; i < window_end; ++i)
        {
            // pass the previous row to the context
            context.previous_row( i, band );

            // load the new pattern character
            const uint8 q = pattern[i];
            const uint8 qq = quals[i];

            const score_type V = scoring.match(qq);

            score_type top, left, diagonal, hi;

            // j == 0 case
            {
                const uint8      g = text_cache[0];
                const score_type S_ij = (g == q) ?  V : scoring.mismatch( g, q, qq );
                diagonal = band[0] + S_ij;
                top      = band[1] + G;
                hi       = nvbio::max( top, diagonal );
                if (TYPE == LOCAL)
                {
                    hi = nvbio::max( hi, score_type(0) );      // clamp to zero
                    sink.report( hi, make_uint2( i+1, i+1 ) );
                }
                band[0] = hi;

                // pass the new cell to the context
                context.new_cell(
                    i, 0u,
                    hi,
                    top > diagonal ? INSERTION : SUBSTITUTION );
            }

            #pragma unroll
            for (uint32 j = 1; j < BAND_LEN-1; ++j)
            {
                const uint8 g = text_cache[j]; text_cache[j-1] = g;
                const score_type S_ij = (g == q) ? V : scoring.mismatch( g, q, qq );
                diagonal = band[j]   + S_ij;
                top      = band[j+1] + G;
                left     = band[j-1] + I;
                hi       = nvbio::max3( top, left, diagonal );
                if (TYPE == LOCAL)
                {
                    hi = nvbio::max( hi, score_type(0) );      // clamp to zero
                    sink.report( hi, make_uint2( i + j + 1, i+1 ) );
                }
                band[j] = hi;

                // pass the new cell to the context
                context.new_cell(
                    i, j,
                    hi,
                    top > left ?
                        (top  > diagonal ? INSERTION : SUBSTITUTION) :
                        (left > diagonal ? DELETION  : SUBSTITUTION) );
            }

            // load the new text character and fill last entry of the band cache
            const uint8 g = start+i+BAND_LEN-1 < text_len ? text[start+i+BAND_LEN-1] : 255u;
            text_cache[ BAND_LEN-2 ] = g;

            // j == BAND_LEN-1 case
            {
                const score_type S_ij = (g == q) ? V : scoring.mismatch( g, q, qq );
                diagonal = band[BAND_LEN-1] + S_ij;
                left     = band[BAND_LEN-2] + I;
                hi       = nvbio::max( left, diagonal );
                if (TYPE == LOCAL)
                {
                    hi = nvbio::max( hi, score_type(0) );      // clamp to zero
                    sink.report( hi, make_uint2( i + BAND_LEN, i+1 ) );
                }
                band[ BAND_LEN-1 ] = hi;

                // pass the new cell to the context
                context.new_cell(
                    i, BAND_LEN-1,
                    hi,
                    left > diagonal ? DELETION : SUBSTITUTION );
            }
        }

        if (window_end < pattern_len)
        {
            // compute the maximum score we got
            score_type max_score = band[0];
            #pragma unroll
            for (uint32 j = 1; j < BAND_LEN; ++j)
                max_score = nvbio::max( max_score, band[j] );

            // and check whether min_score is within reach
            const score_type threshold_score = score_type( min_score ) + score_type(pattern_len - window_end)*scoring.match(0);
            if (max_score < threshold_score)
                return false;
        }

        // pass the last row to the context
        context.last_row( window_end, band );

        if (window_end == pattern_len)
        {
            if (TYPE == GLOBAL)
                sink.report( band[BAND_LEN-1], make_uint2( pattern_len + BAND_LEN-1, pattern_len ) );
            else if (TYPE == SEMI_GLOBAL)
            {
                const uint32 m = nvbio::min( pattern_len + BAND_LEN - 1u, text_len ) - (pattern_len-1u);

                // get the highest score along the long edge of the path graph
                sink.report( band[0], make_uint2( pattern_len + 0, pattern_len ) );
                #pragma unroll
                for (uint32 j = 1; j < BAND_LEN; ++j)
                {
                    if (j < m)
                        sink.report( band[j], make_uint2( pattern_len + j, pattern_len ) );
                }
            }
        }
        return true;
    }
};

/// @} // end of private group

} // namespace banded

///@addtogroup private
///@{

///
/// Calculate the banded alignment score between a pattern and a text string
/// using the Smith-Waterman algorithm.
///
/// \param pattern      shorter string (horizontal)
/// \param quals        qualities string
/// \param text         longer string (vertical)
/// \param pos          offset in the reference string
/// \param sink         output alignment sink
///
/// \return             false if the minimum score was not reached, true otherwise
///
template <
    uint32 BAND_LEN,
    AlignmentType TYPE,
    typename scoring_type,
    typename pattern_type,
    typename qual_type,
    typename text_type,
    typename sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_score(
    const SmithWatermanAligner<TYPE,scoring_type>&  aligner,
    pattern_type                                    pattern,
    qual_type                                       quals,
    text_type                                       text,
    const int32                                     min_score,
    sink_type&                                      sink)
{
    priv::banded::SmithWatermanScoringContext<BAND_LEN,TYPE> context;

    return priv::banded::sw_alignment_score_dispatch<BAND_LEN,TYPE>::run( aligner.scheme, pattern, quals, text, 0u, pattern.length(), 0u, min_score, context, sink );
}

///
/// Calculate a window of the banded alignment matrix between a pattern and a text strings,
/// using the Smith-Waterman algorithm. A checkpoint is used to pass the initial row
/// and store the final one at the end of the window.
///
/// \param pattern      shorter string (horizontal)
/// \param quals        qualities string
/// \param text         longer string (vertical)
/// \param pos          offset in the reference string
/// \param sink         output alignment sink
///
/// \return             false if the minimum score was not reached, true otherwise
///
template <
    uint32 BAND_LEN,
    AlignmentType TYPE,
    typename scoring_type,
    typename pattern_type,
    typename qual_type,
    typename text_type,
    typename sink_type,
    typename checkpoint_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_score(
    const SmithWatermanAligner<TYPE,scoring_type>&  aligner,
    pattern_type                                    pattern,
    qual_type                                       quals,
    text_type                                       text,
    const int32                                     min_score,
    const uint32                                    window_begin,
    const uint32                                    window_end,
    sink_type&                                      sink,
    checkpoint_type                                 checkpoint)
{
    priv::banded::SmithWatermanCheckpointedScoringContext<BAND_LEN,TYPE,checkpoint_type> context( checkpoint );

    return priv::banded::sw_alignment_score_dispatch<BAND_LEN,TYPE>::run( aligner.scheme, pattern, quals, text, window_begin, window_end, 0u, min_score, context, sink );
}

///
/// Calculate the banded Smith-Waterman between a pattern and a text string
/// while saving "checkpoints" along the way, i.e. saving one band
/// of the DP matrix every CHECKPOINTS rows.
///
/// \tparam BAND_LEN            size of the DP band
///
/// \tparam CHECKPOINTS         number of DP rows between each checkpoint
///
/// \tparam checkpoint_type     a class to represent the collection of checkpoints,
///                             represented as a linear array storing each checkpointed
///                             band contiguously.
///                             The class has to provide the non-const indexing operator[].
///
/// \param pattern              pattern to be aligned
/// \param quals                qualities string
/// \param text                 text to align the pattern to
/// \param pos                  offset in the text where the pattern should start aligning
/// \param min_score            minimum tolerated score
/// \param sink                 output alignment sink
/// \param checkpoints          output checkpoints
///
template <
    uint32          BAND_LEN,
    uint32          CHECKPOINTS,
    AlignmentType   TYPE,
    typename        scoring_type,
    typename        pattern_type,
    typename        qual_type,
    typename        text_type,
    typename        sink_type,
    typename        checkpoint_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_checkpoints(
    const SmithWatermanAligner<TYPE,scoring_type>&  aligner,
    pattern_type                                    pattern,
    qual_type                                       quals,
    text_type                                       text,
    const int32                                     min_score,
    sink_type&                                      sink,
    checkpoint_type                                 checkpoints)
{
    priv::banded::SmithWatermanCheckpointContext<BAND_LEN,TYPE,CHECKPOINTS,checkpoint_type> context( checkpoints );

    return priv::banded::sw_alignment_score_dispatch<BAND_LEN,TYPE>::run( aligner.scheme, pattern, quals, text, 0u, pattern.length(), 0u, min_score, context, sink );
}

///
/// Compute the banded Dynamic Programming submatrix between two given checkpoints,
/// storing its flow at each cell.
/// The function returns the submatrix height.
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
/// \tparam submatrix_type      a class to store the flow H, E and F submatrix, represented
///                             as a linear array of size (BAND_LEN*CHECKPOINTS).
///                             The class has to provide the non-const indexing operator[].
///                             Note that the H submatrix entries can assume only 3 values,
///                             while the E and F only 2 - hence the aggregate needs 4 bits
///                             per cell.
///
/// \param pattern              pattern to be aligned
/// \param quals                pattern quality scores
/// \param text                 text to align the pattern to
/// \param pos                  offset in the text where the pattern should start aligning
/// \param min_score            minimum tolerated score
/// \param checkpoints          precalculated checkpoints
/// \param checkpoint_id        index of the first checkpoint defining the submatrix location
/// \param submatrix            output flow submatrix
///
/// \return                     submatrix height
///
template <
    uint32          BAND_LEN,
    uint32          CHECKPOINTS,
    AlignmentType   TYPE,
    typename        scoring_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        checkpoint_type,
    typename        submatrix_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 banded_alignment_submatrix(
    const SmithWatermanAligner<TYPE,scoring_type>&  aligner,
    pattern_string                                  pattern,
    qual_string                                     quals,
    text_string                                     text,
    const int32                                     min_score,
    checkpoint_type                                 checkpoints,
    const uint32                                    checkpoint_id,
    submatrix_type                                  submatrix)
{
    priv::banded::SmithWatermanSubmatrixContext<BAND_LEN,TYPE,CHECKPOINTS,checkpoint_type,submatrix_type> context( checkpoints, checkpoint_id, submatrix );
    const uint32 window_begin = checkpoint_id * CHECKPOINTS;
    const uint32 window_end   = nvbio::min( window_begin + CHECKPOINTS, uint32(pattern.length()) );
    NullSink sink;

    priv::banded::sw_alignment_score_dispatch<BAND_LEN,TYPE>::run( aligner.scheme, pattern, quals, text, window_begin, window_end, 0u, min_score, context, sink );

    return window_end - window_begin;
}

///
/// Given the Dynamic Programming submatrix between two checkpoints,
/// backtrack from a given destination cell.
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
/// \tparam backtracer_type     a class to store the resulting list of backtracking operations.
///                             A model of \ref Backtracer.
///
/// \param checkpoints          precalculated checkpoints
/// \param checkpoint_id        index of the first checkpoint defining the DP submatrix,
///                             storing all bands between checkpoint_id and checkpoint_id+1.
/// \param submatrix            precalculated flow submatrix
/// \param submatrix_height     submatrix height
/// \param sink                 in/out sink of the DP solution
/// \param backtracer           backtracking output handler
/// \param state                matrix state, unused.
///
/// \return                     true if the alignment source has been found, false otherwise
///
template <
    uint32          BAND_LEN,
    uint32          CHECKPOINTS,
    AlignmentType   TYPE,
    typename        scoring_type,
    typename        checkpoint_type,
    typename        submatrix_type,
    typename        backtracer_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_traceback(
    const SmithWatermanAligner<TYPE,scoring_type>&  aligner,
    checkpoint_type                                 checkpoints,
    const uint32                                    checkpoint_id,
    submatrix_type                                  submatrix,
    const uint32                                    submatrix_height,
          uint8&                                    state,
          uint2&                                    sink,
    backtracer_type&                                backtracer)
{
    // Backtrack from the second checkpoint to the first looking up the flow submatrices

    int32 current_entry = sink.x - sink.y;
    int32 current_row   = sink.y - checkpoint_id*CHECKPOINTS - 1u;

    NVBIO_CUDA_DEBUG_ASSERT( current_entry >= 0 &&
                       current_entry < BAND_LEN, "sw::banded_alignment_backtrack(): sink (%u,%u) -> local x coordinate %d not in [0,%d[\n", sink.x, sink.y, current_entry, BAND_LEN );
    NVBIO_CUDA_DEBUG_ASSERT( current_row >= 0,                   "sw::banded_alignment_backtrack(): sink (%u,%u) -> local y coordinate %d not in [0,%u[\n", sink.x, sink.y, current_row, submatrix_height );
    NVBIO_CUDA_DEBUG_ASSERT( current_row <  submatrix_height,    "sw::banded_alignment_backtrack(): sink (%u,%u) -> local y coordinate %d not in [0,%u[\n", sink.x, sink.y, current_row, submatrix_height );

    while (current_row >= 0)
    {
        const uint32 submatrix_cell = current_row * BAND_LEN + current_entry;
        const uint8  op = submatrix[ submatrix_cell ];

        if (TYPE == LOCAL)
        {
            if (op == SINK)
            {
                sink.y = current_row + checkpoint_id*CHECKPOINTS + 1u;
                sink.x = current_entry + sink.y;
                return true;
            }
        }

        if (op == DELETION)
        {
            --current_entry;
            backtracer.push( DELETION );
        }
        else if (op == INSERTION)
        {
            ++current_entry; --current_row;
            backtracer.push( INSERTION );
        }
        else
        {
            --current_row;
            backtracer.push( SUBSTITUTION );
        }

        NVBIO_CUDA_ASSERT( current_entry >= 0 && current_entry < BAND_LEN );
    }
    sink.y = checkpoint_id*CHECKPOINTS;
    sink.x = current_entry + sink.y;
    return false;
}

/// @} // end of private group

} // namespace priv

} // namespace aln
} // namespace nvbio

