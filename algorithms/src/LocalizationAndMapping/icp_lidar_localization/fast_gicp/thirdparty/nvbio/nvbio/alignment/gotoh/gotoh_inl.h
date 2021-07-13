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
#include <nvbio/basic/iterator.h>
#include <nvbio/strings/vectorized_string.h>

namespace nvbio {
namespace aln {

// ----------------------------- Gotoh functions ---------------------------- //

#define NVBIO_SW_VECTOR_LOADING

namespace priv
{

template <typename string_type> struct gotoh_use_vectorization                          { static const bool VALUE = false; };
template <typename T>           struct gotoh_use_vectorization< vector_view<      T*> > { static const bool VALUE = true; };
template <typename T>           struct gotoh_use_vectorization< vector_view<const T*> > { static const bool VALUE = true; };

//
// A helper scoring context class, which can be used to adapt the basic
// algorithm to various situations, such as:
//   scoring
//   scoring within a window (i.e. saving only the last band within the window)
//   computing checkpoints
//   computing a flow submatrix
//
template <uint32 BAND_LEN, AlignmentType TYPE, typename algorithm_tag>
struct GotohScoringContext
{
    // initialize the j-th column of the DP matrix
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    // \param scoring   scoring scheme
    // \param zero      zero value
    // \param infimum   infimum value
    template <typename column_type, typename scoring_type, typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void init(
        const uint32        j,
        const uint32        N,
              column_type&  column,
        const scoring_type& scoring,
        const score_type    zero,
        const score_type    infimum)
    {
        if (j == 0) // ensure this context can be used for multi-pass scoring
        {
            for (uint32 i = 0; i < N; ++i)
            {
                column[i].x = equal<algorithm_tag,PatternBlockingTag>() ?
                    TYPE == GLOBAL ? scoring.text_gap_open() + scoring.text_gap_extension() * i : zero :
                    TYPE != LOCAL  ? scoring.text_gap_open() + scoring.text_gap_extension() * i : zero;
                column[i].y = TYPE == LOCAL  ? zero : infimum;
            }
        }
    }

    // do something with the previous column
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    template <typename column_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void previous_column(
        const uint32        j,
        const uint32        N,
        const column_type   column) {}

    // do something with the last column
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    template <typename column_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void last_column(
        const uint32        j,
        const uint32        M,
        const uint32        N,
        const column_type   column) {}

    // do something with the newly computed cell
    //
    // \param i         row index
    // \param N         number of rows (column size)
    // \param j         column index
    // \param M         number of columns (row size)
    // \param score     computed score
    // \param dir       direction flow
    template <typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void new_cell(
        const uint32            i,
        const uint32            N,
        const uint32            j,
        const uint32            M,
        const score_type        score,
        const DirectionVector   dir,
        const DirectionVector   edir,
        const DirectionVector   fdir) {}
};

//
// A helper checkpointed-scoring context class which allows to perform scoring in multiple
// passes, saving & restoring a checkpoint each time.
//
template <uint32 BAND_LEN, AlignmentType TYPE, typename algorithm_tag, typename checkpoint_type>
struct GotohCheckpointedScoringContext
{
    // constructor
    //
    // \param checkpoints       input checkpoints array
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    GotohCheckpointedScoringContext(checkpoint_type checkpoint) :
        m_checkpoint( checkpoint ) {}

    // initialize the j-th column of the DP matrix
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    // \param scoring   scoring scheme
    // \param zero      zero value
    // \param infimum   infimum value
    template <typename column_type, typename scoring_type, typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void init(
        const uint32        j,
        const uint32        N,
              column_type&  column,
        const scoring_type& scoring,
        const score_type    zero,
        const score_type    infimum)
    {
        if (j == 0)
        {
            for (uint32 i = 0; i < N; ++i)
            {
                column[i].x = equal<algorithm_tag,PatternBlockingTag>() ?
                    TYPE == GLOBAL ? scoring.text_gap_open() + scoring.text_gap_extension() * i : zero :
                    TYPE != LOCAL  ? scoring.text_gap_open() + scoring.text_gap_extension() * i : zero;
                column[i].y = TYPE == LOCAL  ? zero : infimum;
            }
        }
        else
        {
            for (uint32 i = 0; i < N; ++i)
            {
                column[i].x = m_checkpoint[i].x;
                column[i].y = m_checkpoint[i].y;
            }
        }
    }

    // do something with the previous column
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    template <typename column_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void previous_column(
        const uint32        j,
        const uint32        N,
        const column_type   column) {}

    // do something with the last column
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    template <typename column_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void last_column(
        const uint32        j,
        const uint32        M,
        const uint32        N,
        const column_type   column)
    {
        for (uint32 i = 0; i < N; ++i)
        {
            m_checkpoint[i].x = column[i].x;
            m_checkpoint[i].y = column[i].y;
        }
    }

    // do something with the newly computed cell
    //
    // \param i         row index
    // \param N         number of rows (column size)
    // \param j         column index
    // \param M         number of columns (row size)
    // \param score     computed score
    // \param dir       direction flow
    template <typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void new_cell(
        const uint32            i,
        const uint32            N,
        const uint32            j,
        const uint32            M,
        const score_type        score,
        const DirectionVector   dir,
        const DirectionVector   edir,
        const DirectionVector   fdir) {}

    checkpoint_type  m_checkpoint;
};

//
// A helper scoring context class, instantiated to keep track of checkpoints
//
template <uint32 BAND_LEN, AlignmentType TYPE, uint32 CHECKPOINTS, typename checkpoint_type>
struct GotohCheckpointContext
{
    // constructor
    //
    // \param checkpoints       input checkpoints array
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    GotohCheckpointContext(checkpoint_type checkpoints) :
        m_checkpoints( checkpoints ) {}

    // initialize the j-th column of the DP matrix
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    // \param scoring   scoring scheme
    // \param zero      zero value
    // \param infimum   infimum value
    template <typename column_type, typename scoring_type, typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void init(
        const uint32        j,
        const uint32        N,
              column_type&  column,
        const scoring_type& scoring,
        const score_type    zero,
        const score_type    infimum)
    {
        for (uint32 i = 0; i < N; ++i)
        {
            column[i].x = TYPE == GLOBAL ? scoring.text_gap_open() + scoring.text_gap_extension() * i : zero;
            column[i].y = TYPE == LOCAL  ? zero : infimum;
        }
    }

    // do something with the previous column
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    template <typename column_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void previous_column(
        const uint32        j,
        const uint32        N,
        const column_type   column)
    {
        typedef typename std::iterator_traits<column_type>::value_type vector_type;
        typedef typename vector_traits<vector_type>::value_type        value_type;

        // save checkpoint
        if ((j & (CHECKPOINTS-1)) == 0u)
        {
            const uint32 checkpoint_id = j / CHECKPOINTS;

            for (uint32 i = 0; i < N; ++i)
                m_checkpoints[ checkpoint_id*N + i ] = make_vector<value_type>( column[i].x, column[i].y );
        }
    }

    // do something with the last column
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    template <typename column_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void last_column(
        const uint32        j,
        const uint32        M,
        const uint32        N,
        const column_type   column) {}

    // do something with the newly computed cell
    //
    // \param i         row index
    // \param N         number of rows (column size)
    // \param j         column index
    // \param M         number of columns (row size)
    // \param score     computed score
    // \param dir       direction flow
    template <typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void new_cell(
        const uint32            i,
        const uint32            N,
        const uint32            j,
        const uint32            M,
        const score_type        score,
        const DirectionVector   dir,
        const DirectionVector   edir,
        const DirectionVector   fdir) {}

    checkpoint_type  m_checkpoints;
};

//
// A helper scoring context class, instantiated to keep track of the direction vectors
// of a DP submatrix between given checkpoints
//
template <uint32 BAND_LEN, AlignmentType TYPE, uint32 CHECKPOINTS, typename checkpoint_type, typename submatrix_type>
struct GotohSubmatrixContext
{
    // constructor
    //
    // \param checkpoints       input checkpoints array
    // \param checkpoint_id     id of the checkpoint defining the first column of the submatrix
    // \param submatrix         submatrix output storage
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    GotohSubmatrixContext(
        const checkpoint_type   checkpoints,
        const uint32            checkpoint_id,
        const submatrix_type    submatrix) :
        m_checkpoints( checkpoints ),
        m_checkpoint_id( checkpoint_id ),
        m_submatrix( submatrix ) {}

    // initialize the j-th column of the DP matrix
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    // \param scoring   scoring scheme
    // \param zero      zero value
    // \param infimum   infimum value
    template <typename column_type, typename scoring_type, typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void init(
        const uint32        j,
        const uint32        N,
              column_type&  column,
        const scoring_type& scoring,
        const score_type    zero,
        const score_type    infimum)
    {
        typedef typename std::iterator_traits<column_type>::value_type value_type;

        // restore the checkpoint
        for (uint32 i = 0; i < N; ++i)
        {
            const value_type f = m_checkpoints[ m_checkpoint_id*N + i ];
            column[i].x = f.x;
            column[i].y = f.y;
        }
    }

    // do something with the previous column
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    template <typename column_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void previous_column(
        const uint32        j,
        const uint32        N,
        const column_type   column) {}

    // do something with the last column
    //
    // \param j         column index
    // \param N         column size
    // \param column    column values
    template <typename column_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void last_column(
        const uint32        j,
        const uint32        M,
        const uint32        N,
        const column_type   column) {}

    // do something with the newly computed cell
    //
    // \param i         row index
    // \param N         number of rows (column size)
    // \param j         column index
    // \param M         number of columns (row size)
    // \param score     computed score
    // \param dir       direction flow
    template <typename score_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void new_cell(
        const uint32            i,
        const uint32            N,
        const uint32            j,
        const uint32            M,
        const score_type        score,
        const DirectionVector   hdir,
        const DirectionVector   edir,
        const DirectionVector   fdir)
    {
        // save the direction vector
        const uint32 offset = m_checkpoint_id * CHECKPOINTS;

        const uint8 cdir =
            (TYPE == LOCAL ? (score == 0 ? SINK : hdir) : hdir) | edir | fdir;

        m_submatrix[ i * CHECKPOINTS + (j - offset) ] = cdir;
    }

    checkpoint_type     m_checkpoints;
    uint32              m_checkpoint_id;
    submatrix_type      m_submatrix;
};

template <uint32 BAND_LEN, AlignmentType TYPE, typename algorithm_tag, typename symbol_type>
struct gotoh_alignment_score_dispatch {};

//
// A template struct used to possibly specialize the implementation of the Gotoh-based alignment based on
// the template parameters.
//
template <uint32 BAND_LEN, AlignmentType TYPE, typename symbol_type>
struct gotoh_alignment_score_dispatch<BAND_LEN,TYPE,PatternBlockingTag,symbol_type>
{
    // update a DP row
    //
    template <
        bool CHECK_M,
        typename context_type,
        typename query_cache,
        typename score_type,
        typename temp_iterator,
        typename sink_type,
        typename scoring_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void update_row(
        context_type&        context,
        const uint32         block,
        const uint32         M,
        const uint32         i,
        const uint32         N,
        const symbol_type    r_i,
        const query_cache    q_cache,
        temp_iterator        temp,
        score_type&          temp_i,
        score_type*          H_band,
        score_type*          F_band,
        sink_type&           sink,
        const score_type     min_score,
        score_type&          max_score,
        const score_type     G_o,
        const score_type     G_e,
        const score_type     zero,
        const scoring_type   scoring)
    {
        typedef typename std::iterator_traits<temp_iterator>::value_type temp_cell_type;
        typedef typename vector_traits<temp_cell_type>::value_type       temp_scalar_type;

        //
        // NOTE:
        // It might look as if we were going to make lots of out-of-bounds accesses here,
        // as we loop across BAND_LEN cells irrespectively of whether the band straddles
        // the end of the pattern.
        // However, these accesses don't cause any page faults as they refer to properly
        // sized temporary arrays (that in practice are placed in registers), and don't
        // affect the results as they contribute to unused portions of the DP matrices.
        // The reporting of the scores is properly guarded.
        //

        // set the 0-th coefficient in the band to be equal to the (i-1)-th row of the left column (diagonal term)
        score_type H_diag = temp_i;

        H_band[0]    = temp_i = temp[i].x;
        score_type E =          temp[i].y;

        #pragma unroll
        for (uint32 j = 1; j <= BAND_LEN; ++j)
        {
            // update F
            const score_type ftop = F_band[j] + G_e;
            const score_type htop = H_band[j] + G_o;
            F_band[j] = nvbio::max( ftop, htop );
            const DirectionVector fdir = ftop > htop ? DELETION_EXT : SUBSTITUTION;

            // update E
            const score_type eleft = E + G_e;
            const score_type hleft = H_band[j-1] + G_o;
            E = nvbio::max( eleft, hleft );
            const DirectionVector edir = eleft > hleft ? INSERTION_EXT : SUBSTITUTION;

            const int2   info_j       = q_cache[ j-1 ];
            //const int4   info_j       = q_cache[ j-1 ];
            const symbol_type q_j     = symbol_type(info_j.x);
            const score_type  qq_j    = score_type(info_j.y);
            //const score_type  m_j     = score_type(info_j.z);
            //const score_type S_ij     = (r_i == q_j) ? m_j : scoring.mismatch( r_i, q_j, qq_j );
            const score_type S_ij     = scoring.substitution( i, block + j, r_i, q_j, qq_j );
            const score_type diagonal = H_diag + S_ij;
            const score_type top      = F_band[j];
            const score_type left     = E;
                  score_type hi       = nvbio::max3( left, top, diagonal );
                             hi       = (TYPE == LOCAL)? nvbio::max( hi, zero ) : hi; // clamp to zero
            H_diag                    = H_band[j];
            H_band[j]                 = hi;

            if ((CHECK_M == false) || (block + j <= M))
            {
                context.new_cell(
                    i,             N,
                    block + j - 1, M,
                    hi,
                    top > left ?
                        (top  > diagonal ? DELETION  : SUBSTITUTION) :
                        (left > diagonal ? INSERTION : SUBSTITUTION),
                        edir,
                        fdir );
            }
        }

        // save the last entry of the band
        temp[i] = make_vector<temp_scalar_type>( H_band[ BAND_LEN ], E );

        NVBIO_CUDA_ASSERT( H_band[ BAND_LEN ] >= Field_traits<temp_scalar_type>::min() );
        NVBIO_CUDA_ASSERT( H_band[ BAND_LEN ] <= Field_traits<temp_scalar_type>::max() );
        NVBIO_CUDA_ASSERT( E                  >= Field_traits<temp_scalar_type>::min() );
        NVBIO_CUDA_ASSERT( E                  <= Field_traits<temp_scalar_type>::max() );

        max_score = nvbio::max( max_score, H_band[ BAND_LEN ] );

        if (TYPE == LOCAL)
        {
            if (CHECK_M)
            {
                // during local alignment we save the best score across all bands
                for (uint32 j = 1; j <= BAND_LEN; ++j)
                {
                    if (block + j <= M)
                        sink.report( H_band[j], make_uint2( i+1, block+j ) );
                }
            }
            else
            {
                // during local alignment we save the best score across all bands
                for (uint32 j = 1; j <= BAND_LEN; ++j)
                    sink.report( H_band[j], make_uint2( i+1, block+j ) );
            }
        }
        else if (CHECK_M)
        {
            if (TYPE == SEMI_GLOBAL)
            {
                // during semi-global alignment we save the best score across the last column H[*][M], at each row
                save_boundary<BAND_LEN>( block, M, H_band, i, sink );
            }
        }
    }

    //
    // Calculate the alignment score between a string and a reference, using the Smith-Waterman algorithm,
    // using a templated column storage.
    //
    // This function is templated over:
    //   - a context that is passed the computed DP matrix values, and can be
    //     used to specialize its behavior.
    //   - a sink that is used to report successful alignments
    //
    // Furthermore, the function can be called on a window of the pattern, assuming that the context
    // will provide the proper initialization for the first column of the corresponding DP matrix window.
    //
    // \param context       template context class, used to specialize the behavior of the aligner
    // \param query         input pattern (horizontal string)
    // \param quals         input pattern qualities (horizontal string)
    // \param ref           input text (vertical string)
    // \param scoring       scoring scheme
    // \param min_score     minimum output score
    // \param sink          alignment sink
    // \param window_begin  beginning of pattern window
    // \param window_end    end of pattern window
    //
    // \return              false if early-exited, true otherwise
    //
    template <
        typename context_type,
        typename query_type,
        typename qual_type,
        typename ref_type,
        typename scoring_type,
        typename sink_type,
        typename column_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static
    bool run(
        const scoring_type& scoring,
        context_type&       context,
        query_type         query,
        qual_type           quals,
        ref_type            ref,
        const int32         min_score,
        sink_type&          sink,
        const uint32        window_begin,
        const uint32        window_end,
        column_type         temp)
    {
        //
        // This function breaks the DP matrix in vertical stripes of BAND_LEN cells,
        // so as to keep the current reduced row (a band of coefficients) in registers
        // throughout the entire computation.
        // Within each stripe, the matrix is updated top-to-bottom, and the right-most
        // border of the stripe is saved to a local memory array so as to allow resuming
        // its values when processing the next stripe.
        //
        const uint32 M = query.length();
        const uint32 N = ref.length();

        typedef int32 score_type;
        score_type H_band[BAND_LEN+1];
        score_type F_band[BAND_LEN+1];

        typedef typename std::iterator_traits<column_type>::value_type  temp_cell_type;
        typedef typename vector_traits<temp_cell_type>::value_type      temp_scalar_type;

        const score_type G_o     = scoring.pattern_gap_open();
        const score_type G_e     = scoring.pattern_gap_extension();
        const score_type zero    = score_type(0);
        const score_type infimum = Field_traits<temp_scalar_type>::min() - nvbio::min( G_o, G_e );

        int2 q_cache[ BAND_LEN ];
        //int4 q_cache[ BAND_LEN ];

        // initialize the first column
        context.init( window_begin, N, temp, scoring, zero, infimum );

        const uint32 end_block = (window_end == M) ?
            nvbio::max( BAND_LEN, BAND_LEN * ((M + BAND_LEN-1) / BAND_LEN) ) :
            window_end + BAND_LEN;

        // loop across the short edge of the DP matrix (i.e. the columns)
        for (uint32 block = window_begin; block + BAND_LEN < end_block; block += BAND_LEN)
        {
            // save the previous column
            context.previous_column( block, N, temp );

            // load a block of entries from each query
            #pragma unroll
            for (uint32 t = 0; t < BAND_LEN; ++t)
            {
                const symbol_type q  = query[ block + t ];
                const uint8       qq = quals[ block + t ];

                q_cache[ t ] = make_int2( q, qq );
                //q_cache[ t ] = make_int4( q, qq, scoring.match(qq), 0  );
            }

            // initialize the first band
            #pragma unroll
            for (uint32 j = 0; j <= BAND_LEN; ++j)
            {
                H_band[j] = (TYPE != LOCAL) ? (block + j > 0 ? G_o + G_e * (block + j - 1u) : zero) : zero;
                F_band[j] = infimum;
            }

            score_type max_score = Field_traits<score_type>::min();

            score_type temp_i = H_band[0];

            //typedef typename string_traits<ref_type>::forward_iterator  forward_ref_iterator;

        #if defined(NVBIO_SW_VECTOR_LOADING)
            // check whether we should use vectorized loads
            if (gotoh_use_vectorization< ref_type >::VALUE)
            {
                //
                // loop across the long edge of the DP matrix (i.e. the rows)
                //
                const uint32 REF_VECTOR_WIDTH = vectorized_string<ref_type>::VECTOR_WIDTH;

                const uint2 vec_range = vectorized_string_range( ref );

                for (uint32 i = 0; i < vec_range.x; ++i)
                {
                    // load the new character from the reference
                    const uint8 r_i = ref[i];

                    update_row<false>(
                        context,
                        block, M,
                        i, N,
                        r_i,
                        q_cache,
                        temp,
                        temp_i,
                        H_band,
                        F_band,
                        sink,
                        min_score,
                        max_score,
                        G_o,G_e,
                        zero,
                        scoring );
                }
                for (uint32 i = vec_range.x; i < vec_range.y; i += REF_VECTOR_WIDTH)
                {
                    uint8 r[REF_VECTOR_WIDTH];

                    // load REF_VECTOR_WIDTH new characters from the reference
                    vectorized_string_load( ref, i, r );

                    for (uint32 j = 0; j < REF_VECTOR_WIDTH; ++j)
                    {
                        // load the new character from the reference
                        const uint8 r_i = r[j];

                        update_row<false>(
                            context,
                            block, M,
                            i + j, N,
                            r_i,
                            q_cache,
                            temp,
                            temp_i,
                            H_band,
                            F_band,
                            sink,
                            min_score,
                            max_score,
                            G_o,G_e,
                            zero,
                            scoring );
                    }
                }

                for (uint32 i = vec_range.y; i < N; ++i)
                {
                    // load the new character from the reference
                    const uint8 r_i = ref[i];

                    update_row<false>(
                        context,
                        block, M,
                        i, N,
                        r_i,
                        q_cache,
                        temp,
                        temp_i,
                        H_band,
                        F_band,
                        sink,
                        min_score,
                        max_score,
                        G_o,G_e,
                        zero,
                        scoring );
                }
            }
            else
        #endif
            {
                //
                // loop across the long edge of the DP matrix (i.e. the rows)
                //

                //forward_ref_iterator ref_it( ref.begin() );

                for (uint32 i = 0; i < N; ++i)
                {
                    // load the new character from the reference
                    const uint8 r_i = ref[i];
                    //const uint8 r_i = *ref_it; ++ref_it;

                    update_row<false>(
                        context,
                        block, M,
                        i, N,
                        r_i,
                        q_cache,
                        temp,
                        temp_i,
                        H_band,
                        F_band,
                        sink,
                        min_score,
                        max_score,
                        G_o,G_e,
                        zero,
                        scoring );
                }
            }

            // we are now (M - block - BAND_LEN) columns from the last one: check whether
            // we could theoretically reach the minimum score
            const score_type missing_cols = score_type(M - block - BAND_LEN);
            if (max_score + missing_cols * scoring.match(255) < score_type( min_score ))
                return false;
        }

        // process the very last stripe
        if (window_end == M)
        {
            const uint32 block = end_block - BAND_LEN;

            // save the previous column
            context.previous_column( block, N, temp );

            // load a block of entries from each query
            const uint32 block_end = nvbio::min( block + BAND_LEN, M );
            #pragma unroll
            for (uint32 t = 0; t < BAND_LEN; ++t)
            {
                if (block + t < block_end)
                {
                    const symbol_type q  = query[ block + t ];
                    const uint8       qq = quals[ block + t ];

                    q_cache[ t ] = make_int2( q, qq );
                    //q_cache[ t ] = make_int4( q, qq, scoring.match(qq), 0  );
                }
            }

            // initialize the first band
            #pragma unroll
            for (uint32 j = 0; j <= BAND_LEN; ++j)
            {
                H_band[j] = (TYPE != LOCAL) ? (block + j > 0 ? G_o + G_e * (block + j - 1u) : zero) : zero;
                F_band[j] = infimum;
            }

            score_type max_score = Field_traits<score_type>::min();

            score_type temp_i = H_band[0];

            // loop across the long edge of the DP matrix (i.e. the rows)
            for (uint32 i = 0; i < N; ++i)
            {
                // load the new character from the reference
                const uint8 r_i = ref[i];

                update_row<true>(
                    context,
                    block, M,
                    i, N,
                    r_i,
                    q_cache,
                    temp,
                    temp_i,
                    H_band,
                    F_band,
                    sink,
                    min_score,
                    max_score,
                    G_o,G_e,
                    zero,
                    scoring );
            }
        }

        // save the last column
        context.last_column( window_end, M, N, temp );

        if (TYPE == GLOBAL)
            save_Mth<BAND_LEN>( M, H_band, N-1, sink );

        return true;
    }

    //
    // Calculate the alignment score between a string and a reference, using the Smith-Waterman algorithm,
    // using local memory storage for the boundary columns.
    //
    // This function is templated over:
    //   1. a context that is passed the computed DP matrix values, and can be
    //      used to specialize its behavior.
    //   2. a sink that is used to report successful alignments
    //
    // Furthermore, the function can be called on a window of the pattern, assuming that the context
    // will provide the proper initialization for the first column of the corresponding DP matrix window.
    //
    // \param context       template context class, used to specialize the behavior of the aligner
    // \param query         input pattern (horizontal string)
    // \param quals         input pattern qualities (horizontal string)
    // \param ref           input text (vertical string)
    // \param scoring       scoring scheme
    // \param min_score     minimum output score
    // \param sink          alignment sink
    // \param window_begin  beginning of pattern window
    // \param window_end    end of pattern window
    //
    // \return              false if early-exited, true otherwise
    //
    template <
        uint32   MAX_REF_LEN,
        typename context_type,
        typename query_type,
        typename qual_type,
        typename ref_type,
        typename scoring_type,
        typename sink_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static
    bool run(
        const scoring_type& scoring,
        context_type&       context,
        query_type         query,
        qual_type           quals,
        ref_type            ref,
        const int32         min_score,
        sink_type&          sink,
        const uint32        window_begin,
        const uint32        window_end)
    {
        // instantiated a local memory array
        short2  temp[MAX_REF_LEN];
        short2* temp_ptr = temp;

        return run(
            context,
            query,
            quals,
            ref,
            scoring,
            min_score,
            sink,
            window_begin,
            window_end,
            temp_ptr );
    }
};

//
// A template struct used to possibly specialize the implementation of the Gotoh-based alignment based on
// the template parameters.
//
template <uint32 BAND_LEN, AlignmentType TYPE, typename symbol_type>
struct gotoh_alignment_score_dispatch<BAND_LEN,TYPE,TextBlockingTag,symbol_type>
{
    // update a DP row
    //
    template <
        bool CHECK_N,
        typename context_type,
        typename ref_cache,
        typename score_type,
        typename temp_iterator,
        typename sink_type,
        typename scoring_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void update_row(
        context_type&        context,
        const uint32         block,
        const uint32         N,
        const uint32         i,
        const uint32         M,
        const symbol_type    q_i,
        const uint8          qq_i,
        //const score_type     m_i,
        const ref_cache      r_cache,
        temp_iterator        temp,
        score_type&          temp_i,
        score_type*          H_band,
        score_type*          F_band,
        sink_type&           sink,
        const score_type     min_score,
        score_type&          max_score,
        const score_type     G_o,
        const score_type     G_e,
        const score_type     zero,
        const scoring_type   scoring)
    {
        typedef typename std::iterator_traits<temp_iterator>::value_type temp_cell_type;
        typedef typename vector_traits<temp_cell_type>::value_type       temp_scalar_type;

        //
        // NOTE:
        // It might look as if we were going to make lots of out-of-bounds accesses here,
        // as we loop across BAND_LEN cells irrespectively of whether the band straddles
        // the end of the pattern.
        // However, these accesses don't cause any page faults as they refer to properly
        // sized temporary arrays (that in practice are placed in registers), and don't
        // affect the results as they contribute to unused portions of the DP matrices.
        // The reporting of the scores is properly guarded.
        //

        // set the 0-th coefficient in the band to be equal to the (i-1)-th row of the left column (diagonal term)
        score_type H_diag = temp_i;

        H_band[0]    = temp_i = temp[i].x;
        score_type E =          temp[i].y;

        #pragma unroll
        for (uint32 j = 1; j <= BAND_LEN; ++j)
        {
            // update F
            const score_type ftop = F_band[j] + G_e;
            const score_type htop = H_band[j] + G_o;
            F_band[j] = nvbio::max( ftop, htop );
            const DirectionVector fdir = ftop > htop ? INSERTION_EXT : SUBSTITUTION;

            // update E
            const score_type eleft = E + G_e;
            const score_type hleft = H_band[j-1] + G_o;
            E = nvbio::max( eleft, hleft );
            const DirectionVector edir = eleft > hleft ? DELETION_EXT : SUBSTITUTION;

            const symbol_type r_j     = r_cache[ j-1 ];
            //const score_type S_ij     = (r_j == q_i) ? m_i : scoring.mismatch( r_j, q_i, qq_i );
            const score_type S_ij     = scoring.substitution( block + j, i, r_j, q_i, qq_i );
            const score_type diagonal = H_diag + S_ij;
            const score_type top      = F_band[j];
            const score_type left     = E;
                  score_type hi       = nvbio::max3( left, top, diagonal );
                             hi       = (TYPE == LOCAL)? nvbio::max( hi, zero ) : hi; // clamp to zero
            H_diag                    = H_band[j];
            H_band[j]                 = hi;

            if ((CHECK_N == false) || (block + j <= N))
            {
                context.new_cell(
                    i,             M,
                    block + j - 1, N,
                    hi,
                    top > left ?
                        (top  > diagonal ? INSERTION : SUBSTITUTION) :
                        (left > diagonal ? DELETION  : SUBSTITUTION),
                        edir,
                        fdir );
            }
        }

        // save the last entry of the band
        temp[i] = make_vector<temp_scalar_type>( H_band[ BAND_LEN ], E );

        NVBIO_CUDA_ASSERT( H_band[ BAND_LEN ] >= Field_traits<temp_scalar_type>::min() );
        NVBIO_CUDA_ASSERT( H_band[ BAND_LEN ] <= Field_traits<temp_scalar_type>::max() );
        NVBIO_CUDA_ASSERT( E                  >= Field_traits<temp_scalar_type>::min() );
        NVBIO_CUDA_ASSERT( E                  <= Field_traits<temp_scalar_type>::max() );

        max_score = nvbio::max( max_score, H_band[ BAND_LEN ] );

        if (TYPE == LOCAL)
        {
            if (CHECK_N)
            {
                // during local alignment we save the best score across all bands
                for (uint32 j = 1; j <= BAND_LEN; ++j)
                {
                    if (block + j <= N)
                        sink.report( H_band[j], make_uint2( block+j, i+1 ) );
                }
            }
            else
            {
                // during local alignment we save the best score across all bands
                for (uint32 j = 1; j <= BAND_LEN; ++j)
                    sink.report( H_band[j], make_uint2( block+j, i+1 ) );
            }
        }
    }

    //
    // Calculate the alignment score between a string and a reference, using the Smith-Waterman algorithm,
    // using a templated column storage.
    //
    // This function is templated over:
    //   - a context that is passed the computed DP matrix values, and can be
    //     used to specialize its behavior.
    //   - a sink that is used to report successful alignments
    //
    // Furthermore, the function can be called on a window of the pattern, assuming that the context
    // will provide the proper initialization for the first column of the corresponding DP matrix window.
    //
    // \param context       template context class, used to specialize the behavior of the aligner
    // \param query         input pattern (horizontal string)
    // \param quals         input pattern qualities (horizontal string)
    // \param ref           input text (vertical string)
    // \param scoring       scoring scheme
    // \param min_score     minimum output score
    // \param sink          alignment sink
    // \param window_begin  beginning of pattern window
    // \param window_end    end of pattern window
    //
    // \return              false if early-exited, true otherwise
    //
    template <
        typename context_type,
        typename query_type,
        typename qual_type,
        typename ref_type,
        typename scoring_type,
        typename sink_type,
        typename column_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static
    bool run(
        const scoring_type& scoring,
        context_type&       context,
        query_type         query,
        qual_type           quals,
        ref_type            ref,
        const int32         min_score,
        sink_type&          sink,
        const uint32        window_begin,
        const uint32        window_end,
        column_type         temp)
    {
        //
        // This function breaks the DP matrix in vertical stripes of BAND_LEN cells,
        // so as to keep the current reduced row (a band of coefficients) in registers
        // throughout the entire computation.
        // Within each stripe, the matrix is updated top-to-bottom, and the right-most
        // border of the stripe is saved to a local memory array so as to allow resuming
        // its values when processing the next stripe.
        //
        const uint32 M = query.length();
        const uint32 N = ref.length();

        typedef int32 score_type;
        score_type H_band[BAND_LEN+1];
        score_type F_band[BAND_LEN+1];

        typedef typename std::iterator_traits<column_type>::value_type  temp_cell_type;
        typedef typename vector_traits<temp_cell_type>::value_type      temp_scalar_type;

        const score_type G_o     = scoring.pattern_gap_open();
        const score_type G_e     = scoring.pattern_gap_extension();
        const score_type zero    = score_type(0);
        const score_type infimum = Field_traits<temp_scalar_type>::min() - nvbio::min( G_o, G_e );

        uint8 r_cache[ BAND_LEN ];

        // initialize the first column
        context.init( window_begin, M, temp, scoring, zero, infimum );

        const uint32 end_block = (window_end == N) ?
            nvbio::max( BAND_LEN, BAND_LEN * ((N + BAND_LEN-1) / BAND_LEN) ) :
            window_end + BAND_LEN;

        // loop across the short edge of the DP matrix (i.e. the columns)
        for (uint32 block = window_begin; block + BAND_LEN < end_block; block += BAND_LEN)
        {
            // save the previous column
            context.previous_column( block, M, temp );

            // load a block of entries from the reference
            #pragma unroll
            for (uint32 t = 0; t < BAND_LEN; ++t)
                r_cache[ t ] = ref[ block + t ];

            // initialize the first band
            #pragma unroll
            for (uint32 j = 0; j <= BAND_LEN; ++j)
            {
                H_band[j] = (TYPE == GLOBAL) ? (block + j > 0 ? G_o + G_e * (block + j - 1u) : zero) : zero;
                F_band[j] = infimum;
            }

            score_type max_score = Field_traits<score_type>::min();

            score_type temp_i = H_band[0];

            // loop across the short edge of the DP matrix (i.e. the rows)
            for (uint32 i = 0; i < M; ++i)
            {
                // load the new character from the query
                const uint8 q_i  = query[i];
                const uint8 qq_i = quals[i];

                //const int32 m_i = scoring.match(qq_i);

                update_row<false>(
                    context,
                    block, N,
                    i, M,
                    q_i,
                    qq_i,
                    //m_i,
                    r_cache,
                    temp,
                    temp_i,
                    H_band,
                    F_band,
                    sink,
                    min_score,
                    max_score,
                    G_o,G_e,
                    zero,
                    scoring );
            }
            if (TYPE == SEMI_GLOBAL)
            {
                // during semi-global alignment we save the best score across the last row
                for (uint32 j = 1; j <= BAND_LEN; ++j)
                    sink.report( H_band[j], make_uint2( block+j, M ) );
            }

            // we are now (N - block - BAND_LEN) columns from the last one: check whether
            // we could theoretically reach the minimum score
            const score_type missing_cols = score_type(N - block - BAND_LEN);
            if (max_score + missing_cols * scoring.match(255) < score_type( min_score ))
                return false;
        }

        // process the very last stripe
        if (window_end == N)
        {
            const uint32 block = end_block - BAND_LEN;

            // save the previous column
            context.previous_column( block, M, temp );

            // load a block of entries from each query
            const uint32 block_end = nvbio::min( block + BAND_LEN, N );
            #pragma unroll
            for (uint32 t = 0; t < BAND_LEN; ++t)
            {
                if (block + t < block_end)
                    r_cache[ t ] = ref[ block + t ];
            }

            // initialize the first band
            #pragma unroll
            for (uint32 j = 0; j <= BAND_LEN; ++j)
            {
                H_band[j] = (TYPE == GLOBAL) ? (block + j > 0 ? G_o + G_e * (block + j - 1u) : zero) : zero;
                F_band[j] = infimum;
            }

            score_type max_score = Field_traits<score_type>::min();

            score_type temp_i = H_band[0];

        #if defined(NVBIO_SW_VECTOR_LOADING)
            // check whether we should use vectorized loads
            if (gotoh_use_vectorization< query_type >::VALUE)
            {
                //
                // loop across the long edge of the DP matrix (i.e. the rows)
                //
                const uint32 QUERY_VECTOR_WIDTH = vectorized_string<query_type>::VECTOR_WIDTH;

                const uint2 vec_range = vectorized_string_range( query );

                for (uint32 i = 0; i < vec_range.x; ++i)
                {
                    // load the new character from the query
                    const uint8 q_i  = query[i];
                    const uint8 qq_i = quals[i];

                    update_row<true>(
                        context,
                        block, N,
                        i, M,
                        q_i,
                        qq_i,
                        r_cache,
                        temp,
                        temp_i,
                        H_band,
                        F_band,
                        sink,
                        min_score,
                        max_score,
                        G_o,G_e,
                        zero,
                        scoring );
                }
                for (uint32 i = vec_range.x; i < vec_range.y; i += QUERY_VECTOR_WIDTH)
                {
                    uint8 q[QUERY_VECTOR_WIDTH];
                    uint8 qq[QUERY_VECTOR_WIDTH];

                    // load QUERY_VECTOR_WIDTH new characters from the query
                    vectorized_string_load( query, i, q );

                    for (uint32 j = 0; j < QUERY_VECTOR_WIDTH; ++j)
                        qq[j] = quals[i + j];

                    for (uint32 j = 0; j < QUERY_VECTOR_WIDTH; ++j)
                    {
                        // load the new character from the reference
                        const uint8 q_i = q[j];
                        const uint8 qq_i = qq[j];

                        update_row<true>(
                            context,
                            block, N,
                            i + j, M,
                            q_i,
                            qq_i,
                            r_cache,
                            temp,
                            temp_i,
                            H_band,
                            F_band,
                            sink,
                            min_score,
                            max_score,
                            G_o,G_e,
                            zero,
                            scoring );
                    }
                }
                for (uint32 i = vec_range.y; i < M; ++i)
                {
                    // load the new character from the query
                    const uint8 q_i  = query[i];
                    const uint8 qq_i = quals[i];

                    update_row<true>(
                        context,
                        block, N,
                        i, M,
                        q_i,
                        qq_i,
                        r_cache,
                        temp,
                        temp_i,
                        H_band,
                        F_band,
                        sink,
                        min_score,
                        max_score,
                        G_o,G_e,
                        zero,
                        scoring );
                }
            }
            else
        #endif
            {
                //typedef typename string_traits<query_type>::forward_iterator forward_query_iterator;
                //typedef typename string_traits<qual_type>::forward_iterator  forward_qual_iterator;

                //forward_query_iterator query_it( query.begin() );
                //forward_qual_iterator  quals_it( quals.begin() );

                //
                // loop across the short edge of the DP matrix (i.e. the query)
                //
                for (uint32 i = 0; i < M; ++i)
                {
                    // load the new character from the query
                    const uint8 q_i  = query[i];
                    const uint8 qq_i = quals[i];
                    //const uint8 q_i  = *query_it; ++query_it;
                    //const uint8 qq_i = *quals_it; ++quals_it;

                    update_row<true>(
                        context,
                        block, N,
                        i, M,
                        q_i,
                        qq_i,
                        r_cache,
                        temp,
                        temp_i,
                        H_band,
                        F_band,
                        sink,
                        min_score,
                        max_score,
                        G_o,G_e,
                        zero,
                        scoring );
                }
            }

            if (TYPE == SEMI_GLOBAL)
            {
                // during semi-global alignment we save the best score across the last row
                for (uint32 j = 1; j <= BAND_LEN; ++j)
                {
                    if (block + j <= N)
                        sink.report( H_band[j], make_uint2( block+j, M ) );
                }
            }
            else if (TYPE == GLOBAL)
            {
                // during global alignment we save the best score at cell [N][M]
                for (uint32 j = 1; j <= BAND_LEN; ++j)
                {
                    if (block + j == N)
                        sink.report( H_band[j], make_uint2( block+j, M ) );
                }
            }
        }

        // save the last column
        context.last_column( window_end, N, M, temp );
        return true;
    }

    //
    // Calculate the alignment score between a string and a reference, using the Smith-Waterman algorithm,
    // using local memory storage for the boundary columns.
    //
    // This function is templated over:
    //   1. a context that is passed the computed DP matrix values, and can be
    //      used to specialize its behavior.
    //   2. a sink that is used to report successful alignments
    //
    // Furthermore, the function can be called on a window of the pattern, assuming that the context
    // will provide the proper initialization for the first column of the corresponding DP matrix window.
    //
    // \param context       template context class, used to specialize the behavior of the aligner
    // \param query         input pattern (horizontal string)
    // \param quals         input pattern qualities (horizontal string)
    // \param ref           input text (vertical string)
    // \param scoring       scoring scheme
    // \param min_score     minimum output score
    // \param sink          alignment sink
    // \param window_begin  beginning of pattern window
    // \param window_end    end of pattern window
    //
    // \return              false if early-exited, true otherwise
    //
    template <
        uint32   MAX_PATTERN_LEN,
        typename context_type,
        typename string_type,
        typename qual_type,
        typename ref_type,
        typename scoring_type,
        typename sink_type>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static
    bool run(
        const scoring_type& scoring,
        context_type&       context,
        string_type         query,
        qual_type           quals,
        ref_type            ref,
        const int32         min_score,
        sink_type&          sink,
        const uint32        window_begin,
        const uint32        window_end)
    {
        // instantiate a local memory array
        short2  temp[MAX_PATTERN_LEN];
        short2* temp_ptr = temp;

        return run(
            context,
            query,
            quals,
            ref,
            scoring,
            min_score,
            sink,
            window_begin,
            window_end,
            temp_ptr );
    }
};

template <AlignmentType TYPE, uint32 DIM, typename symbol_type>
struct gotoh_bandlen_selector
{
    static const uint32 BAND_LEN = 8u / DIM;
};

template <AlignmentType TYPE, uint32 DIM>
struct gotoh_bandlen_selector<TYPE,DIM,simd4u8>
{
#if __CUDA_ARCH__ >= 300
    static const uint32 BAND_LEN = 8u;
#else
    static const uint32 BAND_LEN = 1u;
#endif
};

//
// Calculate the alignment score between a pattern and a text, using the Gotoh algorithm.
//
// \param aligner      scoring scheme
// \param pattern      pattern string (horizontal
// \param quals        pattern qualities
// \param text         text string (vertical)
// \param min_score    minimum score
//
template <
    AlignmentType   TYPE,
    typename        scoring_type,
    typename        algorithm_tag,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        column_type>
struct alignment_score_dispatch<
    GotohAligner<TYPE,scoring_type,algorithm_tag>,
    pattern_string,
    qual_string,
    text_string,
    column_type>
{
    typedef GotohAligner<TYPE,scoring_type,algorithm_tag> aligner_type;

    /// dispatch scoring across the whole pattern
    ///
    ///
    /// \param aligner      scoring scheme
    /// \param pattern      pattern string (horizontal)
    /// \param quals        pattern qualities
    /// \param text         text string (vertical)
    /// \param min_score    minimum score
    /// \param sink         output alignment sink
    /// \param column       temporary column storage
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

        NVBIO_VAR_UNUSED const uint32 BAND_LEN = gotoh_bandlen_selector<TYPE,1u,symbol_type>::BAND_LEN;

        priv::GotohScoringContext<BAND_LEN,TYPE,algorithm_tag> context;

        const uint32 length = equal<algorithm_tag,PatternBlockingTag>() ? pattern.length() : text.length();

        return gotoh_alignment_score_dispatch<BAND_LEN,TYPE,algorithm_tag,symbol_type>::run( aligner.scheme, context, pattern, quals, text, min_score, sink, 0, length, column );
    }

    /// dispatch scoring in a window of the pattern
    ///
    /// \tparam checkpoint_type     a class to represent the checkpoint: an array of size equal to the text,
    ///                             that has to provide the const indexing operator[].
    ///
    /// \param aligner      scoring scheme
    /// \param pattern      pattern string (horizontal)
    /// \param quals        pattern qualities
    /// \param text         text string (vertical)
    /// \param min_score    minimum score
    /// \param sink         output alignment sink
    /// \param column       temporary column storage
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

        NVBIO_VAR_UNUSED const uint32 BAND_LEN = gotoh_bandlen_selector<TYPE,1u,symbol_type>::BAND_LEN;

        priv::GotohCheckpointedScoringContext<BAND_LEN,TYPE,algorithm_tag,checkpoint_type> context( checkpoint );

        return gotoh_alignment_score_dispatch<BAND_LEN,TYPE,algorithm_tag,symbol_type>::run( aligner.scheme, context, pattern, quals, text, min_score, sink, window_begin, window_end, column );
    }

    /// dispatch scoring in a window of the pattern, retaining the intermediate results in the column
    /// vector, essentially used as a continuation
    ///
    /// \param aligner      scoring scheme
    /// \param pattern      pattern string (horizontal)
    /// \param quals        pattern qualities
    /// \param text         text string (vertical)
    /// \param min_score    minimum score
    /// \param sink         output alignment sink
    /// \param column       temporary column storage
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

        NVBIO_VAR_UNUSED const uint32 BAND_LEN = gotoh_bandlen_selector<TYPE,1u,symbol_type>::BAND_LEN;

        priv::GotohScoringContext<BAND_LEN,TYPE,algorithm_tag> context;

        return gotoh_alignment_score_dispatch<BAND_LEN,TYPE,algorithm_tag,symbol_type>::run( aligner.scheme, context, pattern, quals, text, min_score, sink, window_begin, window_end, column );
    }
};

//
// Calculate the alignment score between a pattern and a text, using the Gotoh algorithm.
//
// \tparam CHECKPOINTS  number of columns between each checkpoint
//
// \param aligner      scoring scheme
// \param pattern      pattern string (horizontal
// \param quals        pattern qualities
// \param text         text string (vertical)
// \param min_score    minimum score
//
template <
    uint32          CHECKPOINTS,
    AlignmentType   TYPE,
    typename        scoring_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        column_type>
struct alignment_checkpointed_dispatch<
    CHECKPOINTS,
    GotohAligner<TYPE,scoring_type>,
    pattern_string,
    qual_string,
    text_string,
    column_type>
{
    typedef GotohAligner<TYPE,scoring_type> aligner_type;

    //
    // Calculate a set of checkpoints of the DP matrix for the alignment between a pattern
    // and a text, using the Gotoh-Smith-Waterman algorithm.
    //
    // \tparam checkpoint_type     a class to represent the collection of checkpoints,
    //                             represented as a linear array storing each checkpointed
    //                             band contiguously.
    //                             The class has to provide the const indexing operator[].
    //
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

        NVBIO_VAR_UNUSED const uint32 BAND_LEN = gotoh_bandlen_selector<TYPE,1u,symbol_type>::BAND_LEN;

        priv::GotohCheckpointContext<BAND_LEN,TYPE,CHECKPOINTS,checkpoint_type> context( checkpoints );

        gotoh_alignment_score_dispatch<BAND_LEN,TYPE,PatternBlockingTag,symbol_type>::run( aligner.scheme, context, pattern, quals, text, min_score, sink, 0, pattern.length(), column );
    }

    //
    // Compute the banded Dynamic Programming submatrix between two given checkpoints,
    // storing its flow at each cell.
    // The function returns the submatrix width.
    //
    // \tparam BAND_LEN            size of the DP band
    //
    // \tparam checkpoint_type     a class to represent the collection of checkpoints,
    //                             represented as a linear array storing each checkpointed
    //                             band contiguously.
    //                             The class has to provide the const indexing operator[].
    //
    // \tparam submatrix_type      a class to store the flow H, E and F submatrix, represented
    //                             as a linear array of size (BAND_LEN*CHECKPOINTS).
    //                             The class has to provide the non-const indexing operator[].
    //                             Note that the H submatrix entries can assume only 3 values,
    //                             while the E and F only 2 - hence the aggregate needs 4 bits
    //                             per cell.
    //
    // \param checkpoints          the set of checkpointed rows
    // \param checkpoint_id        the starting checkpoint used as the beginning of the submatrix
    // \param submatrix            the output submatrix
    //
    // \return                     the submatrix width
    //
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

        NVBIO_VAR_UNUSED const uint32 BAND_LEN = gotoh_bandlen_selector<TYPE,1u,symbol_type>::BAND_LEN;

        priv::GotohSubmatrixContext<BAND_LEN,TYPE,CHECKPOINTS,checkpoint_type,submatrix_type>
            context( checkpoints, checkpoint_id, submatrix );

        const uint32 window_begin = checkpoint_id * CHECKPOINTS;
        const uint32 window_end   = nvbio::min( window_begin + CHECKPOINTS, uint32(pattern.length()) );

        NullSink null_sink;
        gotoh_alignment_score_dispatch<BAND_LEN,TYPE,PatternBlockingTag,symbol_type>::run( aligner.scheme, context, pattern, quals, text, min_score, null_sink, window_begin, window_end, column );

        return window_end - window_begin;
    }
};

//
// Given the Dynamic Programming submatrix between two checkpoints,
// backtrace from a given destination cell, using Gotoh's algorithm.
// The function returns the resulting source cell.
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
//                             The class has to provide the const indexing operator[].
//                             Note that the submatrix entries can assume only 3 values,
//                             and could hence be packed in 2 bits.
//
// \tparam backtracer_type     a class to store the resulting list of backtracking operations.
//                             A model of \ref Backtracer.
//
// \param checkpoints          precalculated checkpoints
// \param checkpoint_id        index of the first checkpoint defining the DP submatrix,
//                             storing all bands between checkpoint_id and checkpoint_id+1.
// \param submatrix            precalculated flow submatrix
// \param submatrix_height     submatrix width
// \param submatrix_height     submatrix height
// \param sink                 in/out sink of the DP solution
// \param output               backtracking output handler
//
// \return                     true if the alignment source has been found, false otherwise
//
template <
    uint32          CHECKPOINTS,
    AlignmentType   TYPE,
    typename        scoring_type,
    typename        checkpoint_type,
    typename        submatrix_type,
    typename        backtracer_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool alignment_traceback(
    const GotohAligner<TYPE,scoring_type>   aligner,
    checkpoint_type                         checkpoints,
    const uint32                            checkpoint_id,
    submatrix_type                          submatrix,
    const uint32                            submatrix_width,
    const uint32                            submatrix_height,
          uint8&                            state,
          uint2&                            sink,
    backtracer_type&                        backtracer)
{
    //
    // Backtrace from the second checkpoint to the first looking up the flow submatrix.
    //
    int32 current_row = sink.x;
    int32 current_col = sink.y - checkpoint_id*CHECKPOINTS - 1u;

    NVBIO_CUDA_DEBUG_ASSERT( current_row >  0 &&
                             current_row <= submatrix_height, "sw::alignment_backtrack(): sink (%u,%u) -> local x coordinate %d not in [0,%d[\n", sink.x, sink.y, current_row, submatrix_height );
    NVBIO_CUDA_DEBUG_ASSERT( current_col >= 0,                "sw::alignment_backtrack(): sink (%u,%u) -> local y coordinate %d not in [0,%u[ (checkpt %u)\n", sink.x, sink.y, current_col, submatrix_width, checkpoint_id );
    NVBIO_CUDA_DEBUG_ASSERT( current_col <  submatrix_width,  "sw::alignment_backtrack(): sink (%u,%u) -> local y coordinate %d not in [0,%u[ (checkpt %u)\n", sink.x, sink.y, current_col, submatrix_width, checkpoint_id );

    while (current_row >  0 &&
           current_col >= 0)
    {
        const uint32 submatrix_cell = (current_row-1u) * CHECKPOINTS + current_col;
        const uint8   op = submatrix[ submatrix_cell ];
        const uint8 h_op = op & HMASK;

        if (TYPE == LOCAL)
        {
            if (state == HSTATE && h_op == SINK)
            {
                sink.x = current_row;
                sink.y = current_col + checkpoint_id*CHECKPOINTS + 1u;
                return true;
            }
        }

        if (state == ESTATE)
        {
            if ((op & INSERTION_EXT) == 0u) state = HSTATE;
            --current_col; backtracer.push( INSERTION );
        }
        else if (state == FSTATE)
        {
            if ((op & DELETION_EXT) == 0u) state = HSTATE;
            --current_row; backtracer.push( DELETION );
        }
        else
        {
            if (h_op == INSERTION)
                state = ESTATE;
            else if (h_op == DELETION)
                state = FSTATE;  // NOTE: commenting this line reverts backtracking to plain SW behavior
            else
            {
                --current_col;
                --current_row;
                backtracer.push( SUBSTITUTION );
            }
        }
    }
    sink.x = current_row;
    sink.y = current_col + checkpoint_id*CHECKPOINTS + 1u;
    return current_row ? false : true; // if current_row == 0 we reached the end of the alignment along the text
}

} // namespace priv

} // namespace aln
} // namespace nvbio
