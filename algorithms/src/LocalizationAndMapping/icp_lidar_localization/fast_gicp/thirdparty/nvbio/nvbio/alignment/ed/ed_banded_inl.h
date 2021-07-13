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
#include <nvbio/alignment/sw/sw_banded_inl.h>
#include <nvbio/alignment/ed/ed_utils.h>

namespace nvbio {
namespace aln {

namespace priv {

///@addtogroup private
///@{

///
/// Calculate the banded alignment score between a pattern and a text string
/// under the edit distance.
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
    typename pattern_type,
    typename qual_type,
    typename text_type,
    typename sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_score(
    const EditDistanceAligner<TYPE>&  aligner,
    pattern_type                      pattern,
    qual_type                         quals,
    text_type                         text,
    const int32                       min_score,
    sink_type&                        sink)
{
    return banded_alignment_score<BAND_LEN>(
        make_smith_waterman_aligner<TYPE>( EditDistanceSWScheme() ),
        pattern,
        quals,
        text,
        min_score,
        sink );
}

///
/// Calculate a window of the banded edit distance Dynamic Programming alignment matrix
/// between a pattern and a text strings. A checkpoint is used to pass the initial row
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
    typename pattern_type,
    typename qual_type,
    typename text_type,
    typename sink_type,
    typename checkpoint_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_score(
    const EditDistanceAligner<TYPE>&  aligner,
    pattern_type                      pattern,
    qual_type                         quals,
    text_type                         text,
    const int32                       min_score,
    const uint32                      window_begin,
    const uint32                      window_end,
    sink_type&                        sink,
    checkpoint_type                   checkpoint)
{
    return banded_alignment_score<BAND_LEN>(
        make_smith_waterman_aligner<TYPE>( EditDistanceSWScheme() ),
        pattern,
        quals,
        text,
        min_score,
        window_begin,
        window_end,
        sink,
        checkpoint );
}

///
/// Calculate the banded edit distance between a pattern and a text string
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
    typename        pattern_type,
    typename        qual_type,
    typename        text_type,
    typename        sink_type,
    typename        checkpoint_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_checkpoints(
    const EditDistanceAligner<TYPE>&  aligner,
    pattern_type                      pattern,
    qual_type                         quals,
    text_type                         text,
    const int32                       min_score,
    sink_type&                        sink,
    checkpoint_type                   checkpoints)
{
    return banded_alignment_checkpoints<BAND_LEN,CHECKPOINTS>(
        make_smith_waterman_aligner<TYPE>( EditDistanceSWScheme() ),
        pattern,
        quals,
        text,
        min_score,
        sink,
        checkpoints );
}

///
/// Compute the banded edit distance Dynamic Programming submatrix between two given checkpoints,
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
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        checkpoint_type,
    typename        submatrix_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 banded_alignment_submatrix(
    const EditDistanceAligner<TYPE>&  aligner,
    pattern_string                    pattern,
    qual_string                       quals,
    text_string                       text,
    const int32                       min_score,
    checkpoint_type                   checkpoints,
    const uint32                      checkpoint_id,
    submatrix_type                    submatrix)
{
    return banded_alignment_submatrix<BAND_LEN,CHECKPOINTS>(
        make_smith_waterman_aligner<TYPE>( EditDistanceSWScheme() ),
        pattern,
        quals,
        text,
        min_score,
        checkpoints,
        checkpoint_id,
        submatrix );
}

///
/// Given the banded edit distance Dynamic Programming submatrix between two checkpoints,
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
    typename        checkpoint_type,
    typename        submatrix_type,
    typename        backtracer_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_traceback(
    const EditDistanceAligner<TYPE>&  aligner,
    checkpoint_type                   checkpoints,
    const uint32                      checkpoint_id,
    submatrix_type                    submatrix,
    const uint32                      submatrix_height,
          uint8&                      state,
          uint2&                      sink,
    backtracer_type&                  backtracer)
{
    return banded_alignment_traceback<BAND_LEN,CHECKPOINTS>(
        make_smith_waterman_aligner<TYPE>( EditDistanceSWScheme() ),
        checkpoints,
        checkpoint_id,
        submatrix,
        submatrix_height,
        state,
        sink,
        backtracer );
}

/// @} // end of private group

} // namespace priv

} // namespace aln
} // namespace nvbio

