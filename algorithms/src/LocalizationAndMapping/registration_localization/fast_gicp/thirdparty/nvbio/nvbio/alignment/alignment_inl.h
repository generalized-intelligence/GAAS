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
#include <nvbio/alignment/sw/sw_inl.h>
#include <nvbio/alignment/ed/ed_inl.h>
#include <nvbio/alignment/gotoh/gotoh_inl.h>
#include <nvbio/alignment/hamming/hamming_inl.h>

#if defined(__CUDACC__)
#include <nvbio/alignment/sw/sw_warp_inl.h>
#include <nvbio/alignment/ed/ed_warp_inl.h>
#include <nvbio/alignment/gotoh/gotoh_warp_inl.h>
#endif

namespace nvbio {
namespace aln {

// a function to compute the alignment score between a pattern and a text string 
// with full DP alignment.
//
// \param aligner             alignment algorithm
// \param pattern             pattern string
// \param quals               quality string
// \param text                text string
// \param min_score           threshold alignment score
// \param sink                output sink
//
template <
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        sink_type,
    typename        column_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const  int32            min_score,
          sink_type&        sink,
          column_type       column)
{
    typedef priv::alignment_score_dispatch<aligner_type,pattern_string,qual_string,text_string,column_type> dispatcher;
    return dispatcher::dispatch(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        sink,
        column );
}

// a function to compute the alignment score between a pattern and a text string 
// with full DP alignment.
//
// \param aligner             alignment algorithm
// \param pattern             pattern string
// \param quals               quality string
// \param text                text string
// \param min_score           threshold alignment score
// \param sink                output sink
//
template <
    uint32          MAX_TEXT_LEN,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const  int32            min_score,
          sink_type&        sink)
{
    typedef typename column_storage_type<aligner_type>::type    cell_type;

    cell_type column[MAX_TEXT_LEN];

    typedef priv::alignment_score_dispatch<aligner_type,pattern_string,qual_string,text_string,cell_type*> dispatcher;
    return dispatcher::dispatch(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        sink,
        column );
}

// a function to compute the alignment score between a pattern and a text string 
// with full DP alignment in multiple passes, where each pass handles a window
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
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        sink_type,
    typename        column_type,
    typename        checkpoint_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const  int32            min_score,
    const uint32            window_begin,
    const uint32            window_end,
          sink_type&        sink,
          checkpoint_type   checkpoint,
          column_type       column)
{
     typedef priv::alignment_score_dispatch<aligner_type,pattern_string,qual_string,text_string,column_type> dispatcher;
     return dispatcher::dispatch(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        window_begin,
        window_end,
        sink,
        checkpoint,
        column );
}

// a function to compute the alignment score between a pattern and a text string 
// with full DP alignment in multiple passes, where each pass handles a window
// of the pattern and saves a checkpoint for the next.
// The checkpoint is stored in the column itself.
//
// \param aligner             alignment algorithm
// \param pattern             pattern string
// \param quals               quality string
// \param text                text string
// \param min_score           threshold alignment score
// \param sink                output sink
//
template <
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        sink_type,
    typename        column_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool alignment_score(
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
     typedef priv::alignment_score_dispatch<aligner_type,pattern_string,qual_string,text_string,column_type> dispatcher;
     return dispatcher::dispatch(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        window_begin,
        window_end,
        sink,
        column );
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
    uint32          CHECKPOINTS,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        sink_type,
    typename        checkpoint_type,
    typename        column_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void alignment_score_checkpoints(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const  int32            min_score,
          sink_type&        sink,
          checkpoint_type   checkpoints,
          column_type       column)
{
    typedef priv::alignment_checkpointed_dispatch<CHECKPOINTS,aligner_type,pattern_string,qual_string,text_string,column_type> dispatcher;
    dispatcher::dispatch_checkpoints(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        sink,
        checkpoints,
        column );
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
    uint32          CHECKPOINTS,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        column_type,
    typename        checkpoint_type,
    typename        submatrix_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 alignment_score_submatrix(
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
    typedef priv::alignment_checkpointed_dispatch<CHECKPOINTS,aligner_type,pattern_string,qual_string,text_string,column_type> dispatcher;
    return dispatcher::dispatch_submatrix(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        checkpoints,
        checkpoint_id,
        submatrix,
        column );
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
    uint32          CHECKPOINTS,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        backtracer_type,
    typename        checkpoints_type,
    typename        submatrix_type,
    typename        column_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
Alignment<int32> alignment_traceback(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const int32             min_score,
    backtracer_type&        backtracer,
    checkpoints_type        checkpoints,
    submatrix_type          submatrix,
    column_type             column)
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

    //
    // TODO: use the following strategy: perform two scoring passes, one to find the end,
    // and one (backwards, reversing both sequences) to find the beginning of the alignment
    // (if the traceback function was passed an exact end cell, the first pass could be skipped).
    // At that point one can isolate a much a smaller submatrix in which to perform checkpointing,
    // and the following algorithm can be employed.
    //

    typedef typename pattern_string::value_type         symbol_type;

    // the submatrix height is equal to the text length (remember the DP matrix has the pattern as rows and the text as columns)
    const uint32 submatrix_height = text.length();

    // compute a set of checkpoints along the pattern
    BestSink<int32> best;
    alignment_score_checkpoints<CHECKPOINTS>(
        aligner, pattern, quals, text, min_score, best, checkpoints, column );

    const uint32 n_checkpoints = (pattern.length() + CHECKPOINTS-1)/CHECKPOINTS;

    // check whether we found a valid alignment
    uint2 sink = best.sink;
    if (sink.x == uint32(-1) ||
        sink.y == uint32(-1))
        return Alignment<int32>( best.score, make_uint2( uint32(-1), uint32(-1) ), make_uint2( uint32(-1), uint32(-1) ) );

    // clip the end of the pattern, in case the alignment terminated early
    backtracer.clip( pattern.length() - sink.y );

    // find the checkpoint containing the sink
    int32 checkpoint_id = n_checkpoints-1;

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
        const uint32 submatrix_width = alignment_score_submatrix<CHECKPOINTS>(
            aligner, pattern, quals, text, min_score, checkpoints, checkpoint_id, submatrix, column );

        if (priv::alignment_traceback<CHECKPOINTS>(
            aligner, checkpoints, checkpoint_id, submatrix, submatrix_width, submatrix_height, state, sink, backtracer ))
            break;
    }

    // finish backtracking along the first row and first column (not explicitly stored)
    // until we get to a cell containing the value zero.
    if (aligner_type::TYPE == SEMI_GLOBAL || aligner_type::TYPE == GLOBAL)
    {
        if (sink.x == 0)
        {
            for (; sink.y > 0; --sink.y)
                backtracer.push( INSERTION );
        }
    }
    if (aligner_type::TYPE == GLOBAL)
    {
        if (sink.y == 0)
        {
            for (; sink.x > 0; --sink.x)
                backtracer.push( DELETION );
        }
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
    uint32          MAX_PATTERN_LEN,
    uint32          MAX_TEXT_LEN,
    uint32          CHECKPOINTS,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        backtracer_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
Alignment<int32> alignment_traceback(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const int32             min_score,
    backtracer_type&        backtracer)
{
    typedef typename checkpoint_storage_type<aligner_type>::type checkpoint_type;
    typedef typename column_storage_type<aligner_type>::type     cell_type;

    cell_type column[MAX_TEXT_LEN];
    const uint32 BITS = direction_vector_traits<aligner_type>::BITS;
    const uint32 ELEMENTS_PER_WORD = 32 / BITS;

    checkpoint_type checkpoints[ (MAX_TEXT_LEN+1)*((MAX_PATTERN_LEN + CHECKPOINTS-1)/CHECKPOINTS) ];

    NVBIO_VAR_UNUSED const uint32 SUBMATRIX_WORDS = (MAX_TEXT_LEN*CHECKPOINTS + ELEMENTS_PER_WORD-1) / ELEMENTS_PER_WORD;
    uint32 submatrix_base[ SUBMATRIX_WORDS ];
    PackedStream<uint32*,uint8,BITS,false> submatrix( submatrix_base );

    return alignment_traceback<CHECKPOINTS>(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        backtracer,
        checkpoints,
        submatrix,
        column );
}

#if defined(__CUDACC__)
namespace warp {

// Compute the alignment score between a pattern and a text string with full DP alignment using a warp.
//
// This is a low-level function, requiring all needed temporary storage to be passed from the caller.
// The purpose is allowing the caller to allocate such storage (possibly among kernel threads) using different
// strategies.
//
// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
// \tparam pattern_string      a string representing the pattern.
// \tparam qual_string         an array representing the pattern qualities.
// \tparam text_string         a string representing the text.
// \tparam column_type         an array-like class defining operator[], used to represent a partial matrix column;
//                             the type of the matrix cells depends on the aligner, and can be obtained as
//                             typename column_storage_type<aligner_type>::type;
//
// \param aligner             alignment algorithm
// \param pattern             pattern string
// \param quals               quality string
// \param text                text string
// \param min_score           threshold alignment score
// \param sink                output sink
// \param column              temporary storage for a matrix column, must be at least as large as the text
//
template <
    uint32          BLOCKDIM,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        column_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
int32 alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const  int32            min_score,
          uint2*            sink,
          column_type       column)
{
    return priv::alignment_score<BLOCKDIM>(
        aligner,
        pattern,
        quals,
        text,
        min_score,
        sink,
        column );
}

} // namespace warp
#endif

} // namespace alignment
} // namespace nvbio
