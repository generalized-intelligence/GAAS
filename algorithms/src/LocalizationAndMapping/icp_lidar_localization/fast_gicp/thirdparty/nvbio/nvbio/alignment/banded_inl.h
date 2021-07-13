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
#include <nvbio/alignment/utils.h>
#include <nvbio/alignment/alignment_base_inl.h>
#include <nvbio/alignment/ed/ed_banded_inl.h>
#include <nvbio/alignment/sw/sw_banded_inl.h>
#include <nvbio/alignment/gotoh/gotoh_banded_inl.h>
#include <nvbio/alignment/myers/myers_banded_inl.h>
namespace nvbio {
namespace aln {

// a function to compute the alignment score between a pattern and a text string 
// with banded DP alignment.
//
// \param aligner             alignment algorithm
// \param pattern             pattern string
// \param quals               quality string
// \param text                text string
// \param min_score           threshold alignment score
// \param sink                output sink
//
template <
    uint32          BAND_LEN,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const  int32            min_score,
          sink_type&        sink)
{
    return priv::banded_alignment_score<BAND_LEN>(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        sink );
}

// a function to compute the alignment score between a pattern and a text string 
// with banded DP alignment.
//
// \param aligner             alignment algorithm
// \param pattern             pattern string
// \param quals               quality string
// \param text                text string
// \param min_score           threshold alignment score
// \param sink                output sink
//
template <
    uint32          BAND_LEN,
    typename        aligner_type,
    typename        pattern_string,
    typename        text_string,
    typename        sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const text_string       text,
    const  int32            min_score,
          sink_type&        sink)
{
    return priv::banded_alignment_score<BAND_LEN>(
        aligner,
        pattern,
        trivial_quality_string(),
        text,
        min_score,
        sink );
}

// a function to compute the alignment score between a pattern and a text string 
// with banded DP alignment.
//
// \param aligner              alignment algorithm
// \param pattern              pattern string
// \param quals                quality string
// \param text                 text string
// \param min_score            threshold alignment score
//
// \return                      best alignment score
//
template <
    uint32          BAND_LEN,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
int32 banded_alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const  int32            min_score)
{
    BestSink<int32> sink;
    priv::banded_alignment_score<BAND_LEN>(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        sink );
    return sink.score;
}

// a function to compute the alignment score between a pattern and a text string 
// with banded DP alignment.
//
// \param aligner              alignment algorithm
// \param pattern              pattern string
// \param text                 text string
// \param min_score            threshold alignment score
//
// \return                      best alignment score
//
template <
    uint32          BAND_LEN,
    typename        aligner_type,
    typename        pattern_string,
    typename        text_string>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
int32 banded_alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const text_string       text,
    const  int32            min_score)
{
    BestSink<int32> sink;
    priv::banded_alignment_score<BAND_LEN>(
        aligner,
        pattern,
        trivial_quality_string(),
        text,
        min_score,
        sink );
    return sink.score;
}

// a function to compute the alignment score between a pattern and a text string 
// with banded DP alignment in multiple passes, where each pass handles a window
// of the pattern and saves a checkpoint for the next.
//
// \param aligner             alignment algorithm
// \param pattern             pattern string
// \param quals               quality string
// \param text                text string
// \param min_score           threshold alignment score
// \param sink                output sink
//
template <
    uint32          BAND_LEN,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        sink_type,
    typename        checkpoint_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool banded_alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const  int32            min_score,
    const uint32            window_begin,
    const uint32            window_end,
          sink_type&        sink,
          checkpoint_type   checkpoint)
{
    return priv::banded_alignment_score<BAND_LEN>(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        window_begin,
        window_end,
        sink,
        checkpoint );
}

//
// Calculate a set of checkpoints of the DP matrix for the alignment between a pattern
// and a text.
//
// \param aligner              scoring scheme
// \param pattern              pattern string (horizontal
// \param quals                pattern qualities
// \param text                 text string (vertical)
// \param min_score            minimum score
// \param sink                 output sink
// \param checkpoints          the set of output checkpointed rows
//
template <
    uint32          BAND_LEN,
    uint32          CHECKPOINTS,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        sink_type,
    typename        checkpoint_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void banded_alignment_score_checkpoints(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const  int32            min_score,
          sink_type&        sink,
          checkpoint_type   checkpoints)
{
    priv::banded_alignment_checkpoints<BAND_LEN,CHECKPOINTS>(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        sink,
        checkpoints );
}

//
// Compute the banded Dynamic Programming submatrix between two given checkpoints,
// storing its flow at each cell.
// The function returns the submatrix width.
//
// \tparam BAND_LEN            size of the DP band
//
// \tparam CHECKPOINTS         number of DP rows between each checkpoint
//
// \tparam checkpoint_type     a class to represent the collection of checkpoints,
//                             represented as a linear array storing each checkpointed
//                             band contiguously.
//                             The class has to provide the const indexing operator[].
//
// \tparam submatrix_type      a class to store the flow submatrix, represented
//                             as a linear array of size (BAND_LEN*CHECKPOINTS).
//                             The class has to provide the non-const indexing operator[].
//                             Note that the submatrix entries can assume only 3 values,
//                             and could hence be packed in 2 bits.
//
// \param aligner              scoring scheme
// \param pattern              pattern string (horizontal
// \param quals                pattern qualities
// \param text                 text string (vertical)
// \param min_score            minimum score
// \param checkpoints          the set of checkpointed rows
// \param checkpoint_id        the starting checkpoint used as the beginning of the submatrix
// \param submatrix            the output submatrix
//
// \return                     the submatrix width
//
template <
    uint32          BAND_LEN,
    uint32          CHECKPOINTS,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        checkpoint_type,
    typename        submatrix_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 banded_alignment_score_submatrix(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const int32             min_score,
    checkpoint_type         checkpoints,
    const uint32            checkpoint_id,
    submatrix_type          submatrix)
{
    return priv::banded_alignment_submatrix<BAND_LEN,CHECKPOINTS>(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        checkpoints,
        checkpoint_id,
        submatrix );
}

//
// Backtrace an optimal alignment using a full DP algorithm.
//
// \tparam CHECKPOINTS         number of DP rows between each checkpoint
// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
// \tparam pattern_string      a string representing the pattern.
// \tparam qual_string         an array representing the pattern qualities.
// \tparam text_string         a string representing the text.
// \tparam checkpoints_type    an array-like class defining operator[], used to represent a reduced DP score matrix,
//                             containing all the matrix columns whose index is a multiple of CHECKPOINTS;
//                             the type of the matrix cells depends on the aligner, and can be obtained as
//                             typename checkpoints_storage_type<aligner_type>::type
// \tparam submatrix_type      an array-like class defining operator[], used to represent a temporary DP flow submatrix,
//                             containing all the matrix flow cells between two checkpointed columns.
//                             the number of bits needed for the submatrix cells depends on the aligner, and can be obtained as
//                             direction_vector_traits<aligner_type>::BITS
// \tparam backtracer_type     a model of \ref Backtracer.
//
// \param aligner              alignment algorithm
// \param pattern              pattern to be aligned
// \param quals                pattern quality scores
// \param text                 text to align the pattern to
// \param min_score            minimum accepted score
// \param backtracer           backtracking delegate
// \param checkpoints          temporary checkpoints storage
// \param submatrix            temporary submatrix storage
//
// \return                     reported alignment
//
template <
    uint32          BAND_LEN,
    uint32          CHECKPOINTS,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        backtracer_type,
    typename        checkpoints_type,
    typename        submatrix_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
Alignment<int32> banded_alignment_traceback(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const int32             min_score,
    backtracer_type&        backtracer,
    checkpoints_type        checkpoints,
    submatrix_type          submatrix)
{
    //
    // This function performs backtracing in a completely generic fashion that works
    // for any DP based aligner.
    // It does so using 2 module-wide generic functions:
    //      * alignment_score_checkpoints()
    //      * alignment_score_submatrix()
    //
    // and one aligner-specific function:
    //      * priv::alignment_traceback()
    //
    BestSink<int32> best;
    banded_alignment_score_checkpoints<BAND_LEN,CHECKPOINTS>(
        aligner, pattern, quals, text, /*pos,*/ min_score, best, checkpoints );

    const uint32 n_checkpoints = 1u + (pattern.length() + CHECKPOINTS-1)/CHECKPOINTS;

    uint2 sink = best.sink;
    if (sink.x == uint32(-1) ||
        sink.y == uint32(-1))
        return Alignment<int32>( best.score, make_uint2( uint32(-1), uint32(-1) ), make_uint2( uint32(-1), uint32(-1) ) );

    // clip the end of the alignment
    backtracer.clip( pattern.length() - sink.y );

    // find the checkpoint containing the sink
    int32 checkpoint_id = n_checkpoints-2;

    if (aligner_type::TYPE == LOCAL)
    {
        for (; checkpoint_id >= 0; --checkpoint_id)
        {
            if (checkpoint_id * CHECKPOINTS < sink.y)
                break;
        }
    }

    //store state (H, E, or F) between checkpoints
    uint8 state = HSTATE;

    // backtrack until needed
    for (; checkpoint_id >= 0; --checkpoint_id)
    {
        const uint32 submatrix_height = banded_alignment_score_submatrix<BAND_LEN,CHECKPOINTS>(
            aligner, pattern, quals, text, /*pos,*/ min_score, checkpoints, checkpoint_id, submatrix );

        if (priv::banded_alignment_traceback<BAND_LEN,CHECKPOINTS>(
            aligner, checkpoints, checkpoint_id, submatrix, submatrix_height, state, sink, backtracer ))
            break;
    }

    // clip the beginning of the alignment
    backtracer.clip( sink.y );

    return Alignment<int32>( best.score, sink, best.sink );
}

//
// Backtrace an optimal alignment using a full DP algorithm.
//
// \tparam MAX_PATTERN_LEN     maximum pattern length
// \tparam MAX_TEXT_LEN        maximum text length
// \tparam CHECKPOINTS         number of DP rows between each checkpoint
// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
// \tparam pattern_string      a string representing the pattern.
// \tparam qual_string         an array representing the pattern qualities.
// \tparam text_string         a string representing the text.
// \tparam backtracer_type     a model of \ref Backtracer.
//
// \param aligner              alignment algorithm
// \param pattern              pattern to be aligned
// \param quals                pattern quality scores
// \param text                 text to align the pattern to
// \param min_score            minimum accepted score
// \param backtracer           backtracking delegate
//
// \return                     reported alignment
//
template <
    uint32          BAND_LEN,
    uint32          MAX_PATTERN_LEN,
    uint32          CHECKPOINTS,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        backtracer_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
Alignment<int32> banded_alignment_traceback(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const int32             min_score,
    backtracer_type&        backtracer)
{
    typedef typename checkpoint_storage_type<aligner_type>::type checkpoint_type;
    typedef typename column_storage_type<aligner_type>::type     cell_type;

    const uint32 BITS = direction_vector_traits<aligner_type>::BITS;
    const uint32 ELEMENTS_PER_WORD = 32 / BITS;

    checkpoint_type checkpoints[ BAND_LEN*(1u + (MAX_PATTERN_LEN + CHECKPOINTS-1)/CHECKPOINTS) ];

    NVBIO_VAR_UNUSED const uint32 SUBMATRIX_WORDS = (BAND_LEN*CHECKPOINTS + ELEMENTS_PER_WORD-1) / ELEMENTS_PER_WORD;
    uint32 submatrix_base[ SUBMATRIX_WORDS ];
    PackedStream<uint32*,uint8,BITS,false> submatrix( submatrix_base );

    return banded_alignment_traceback<BAND_LEN,CHECKPOINTS>(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        backtracer,
        checkpoints,
        submatrix );
}

} // namespace alignment
} // namespace nvbio
