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
#include <nvbio/alignment/alignment_base.h>

///\page alignment_page Alignment Module
///\htmlonly
/// <img src="nvidia_cubes.png" style="position:relative; bottom:-10px; border:0px;"/>
///\endhtmlonly
///\par
/// This module contains a series of functions to perform string alignment. Most of them
/// can be called both from host (CPU) and device (GPU) code, although some target
/// only the device (e.g. the warp-parallel ones), and others are host functions that
/// execute various kernels on the device (e.g. the ones for \ref BatchAlignmentSection).\n
/// Generally speaking, all these functions are parametrized over the alignment
/// algorithm, which is specified by an \ref Aligner "Aligner" object.
/// The aligner encodes the scheme used to score character matches, mismatches, insertions
/// and deletions.
/// Currently, there are three types of aligners, \ref EditDistanceAligner "Edit Distance",
/// \ref SmithWatermanAligner "Smith-Waterman" and \ref GotohAligner "Gotoh".
///\par
/// The main functionalities are scoring, traceback and batch alignment.
/// The following is a brief overview; for a detailed list of all classes and functions
/// see the \ref Alignment documentation.
///
/// \section ScoringSection Scoring
///\par
/// Scoring is the process of computing the dynamic programming matrix of the scores of all
/// possible alignments between a pattern string P and a text string T (roughly speaking, the
/// cell (i,j) of the DP matrix encodes the best possible score between the suffix T[0:i] and
/// the suffix P[0,j]).
/// The scoring functions do not compute the actual alignments, they just assign scores to all
/// possible valid alignments and identify them by their last cell.
/// These, are passed to an \ref AlignmentSink "Alignment Sink", a user-defined delegate responsible
/// for handling the results.
///\par
/// The module exposes several variants of the following functions:
///
/// - banded_alignment_score()
/// - alignment_score()
///\par
/// according to whether one wants to process a pattern with qualities or not, whether he's interested
/// in a single best score or the best N, and so on.
/// To see a concrete usage example, consider the following code snippet:
///
/// \code
/// typedef vector_view<const char*> const_string;
///
/// const_string  text = make_string("AAAAGGGTGCTCAA");
/// const_string  pattern  = make_string("GGGTGCTCAA");
///
/// const int32 ed = aln::banded_alignment_score<5>(
///        aln::make_edit_distance_aligner<aln::SEMI_GLOBAL>(), // build an edit-distance aligner
///        pattern,                                             // pattern string
///        text,                                                // text string
///        -255 );                                              // minimum accepted score
///
///
/// const aln::SimpleGotohScheme scoring(                       // build a Gotoh scoring scheme
///      2,                                                     // match bonus
///     -1,                                                     // mismatch penalty
///     -1,                                                     // gap open penalty
///     -1 )                                                    // gap extension penalty
///
/// aln::Best2Sink<int32> best2;                                // keep the best 2 scores
/// aln::banded_alignment_score<5>(
///        aln::make_gotoh_aligner<aln::LOCAL>( scoring ),      // build a local Gotoh aligner
///        pattern,                                             // pattern string
///        text,                                                // text string
///        -255,                                                // minimum accepted score
///        best2 );                                             // alignment sink
///
/// \endcode
///
///\anchor AlignersAnchor
/// \section AlignersSection Aligners and Alignment Algorithms
///\par
/// In the previous section we have seen that scoring functions take an aligner as an input. NVBIO exposes
/// three such types, which specify the actual type of alignment scheme to use:
///\par
/// - \ref EditDistanceAligner
/// - \ref SmithWatermanAligner
/// - \ref GotohAligner
///\par
/// These objects are parameterized by an \ref AlignmentTypeModule "AlignmentType", which can be any of GLOBAL,
/// SEMI_GLOBAL or LOCAL, and an \ref AlgorithmTag "Algorithm Tag", which specifies the
/// actual algorithm to employ.
/// At the moment, there are three such algorithms:
///\par
/// - \ref PatternBlockingTag : a DP algorithm which blocks the matrix in stripes along the pattern
/// - \ref TextBlockingTag : a DP algorithm which blocks the matrix in stripes along the text
/// - \ref MyersTag : the Myers bit-vector algorithm, a very fast algorithm to perform edit distance computations
///
/// \section TracebackSection Traceback
///\par
/// Traceback is the process of computing the actual alignment between two strings, that is to say
/// the series of edits (character replacements, insertions and deletions) necessary to obtain
/// a given alignment.
/// Traceback functions can either report a single optimal alignment, or enumerate all possible
/// alignments.
/// Once again, the output alignments are built implicitly, calling a user-defined \ref Backtracer "Backtracer"
/// delegate.
///\par
/// Again, the module exposes both banded and whole-matrix traceback:
///
/// - banded_alignment_traceback()
/// - alignment_traceback()
///
/// \subsection Backtracer Backtracer Model
///\par
/// A backtracer is a user-defined delegate implementing the following interface:
///
/// \code
/// struct Backtracer
/// {
///     // soft-clip the next n characters
///     void clip(const uint32 n);
/// 
///     // push the given string edit
///     void push(const DirectionVector op);  // op = {SUBSTITUTION|INSERTION|DELETION}
/// };
/// \endcode
///\par
/// Notice that traceback algorithms will push operations backwards, starting from the end of the alignment
/// and proceeding towards the source. The very first and very last operations will be calls to clip(n), potentially
/// with n = 0, signalling the length of the soft-clipping areas respectively at the end and beginning of the pattern.
///
/// \section MemoryMangamentSection Memory Management
///\par
/// Some of the functions in this module require a certain amount of temporary storage,
/// for example to store a column of the DP matrix while scoring it, or a stripe of the
/// flow matrix for traceback.
/// On a CPU, running a modest amount of threads, one would typically either allocate this
/// storage on the heap inside the function itself and free it before returning, or require
/// the user to pass an allocator.
/// On a GPU, where thousands of threads might call into the same function at once, both
/// strategies would likely be very inefficient because of resource contention (an allocator
/// would have to do some form of locking to be thread safe).
/// Instead, nvbio delegates the responsibility to the caller, who needs to pass the necessary
/// temporary storage (e.g. a matrix column) to the function.
/// The temporary storage type is a template iterator, meaning that the user is able to control
/// freely and exactly what kind of memory is passed and its addressing scheme (for example,
/// one could pass a plain local memory array, but also a strided_iterator pointing to some global
/// memory arena).
/// Convenience functions are available to let nvbio do the necessary allocations at compile-time in
/// local memory, but the user needs to be aware of the fact that:
///  - 1. the maximum problem size needs to be statically provided at compile-time
///  - 2. when calling into these functions from a large thread grid, the amount of allocated
///       local memory might be significant.
///       This is particularly problematic for traceback functions, which can easily need up to ~50KB
///       per thread.
///
/// \section BatchAlignmentSection Batch Alignment
///\par
/// Besides offering functions to compute an individual alignment between a single pattern and a single text
/// (from within a thread or a warp), this module offers \ref BatchAlignment "parallel execution contexts"
/// for performing larger batches of alignment jobs on the <em>device</em> with a single call.
/// Under the hood, these contexts dispatch the parallel execution invoking one or multiple kernels,
/// possibly splitting the computations of each alignment in many smaller chunks so as to load balance
/// and re-gather execution coherence.
/// The parallel algorithm employed is specified at compile-time by a \ref BatchScheduler "Batch Scheduler".
///\par
/// The following code snippet shows an example using the convenience function batch_alignment_score() :
///
///\anchor BatchAlignmentExample
///\code
/// const uint32 n_strings   = 10000;
/// const uint32 pattern_len = 100;
/// const uint32 text_len    = 200;
///
/// // build two concatenated string sets
/// thrust::device_vector<uint8>  pattern_storage( n_strings * pattern_len );
/// thrust::device_vector<uint32> pattern_offsets( n_strings+1 );
/// thrust::device_vector<uint8>  text_storage( n_strings * text_len );
/// thrust::device_vector<uint32> text_offsets( n_strings+1 );
///
/// // fill their content with random characters in [0,4)
/// const uint32 ALPHABET_SIZE = 4u;
/// fill_random( ALPHABET_SIZE, pattern_storage.begin(), pattern_storage.end() );
/// fill_random( ALPHABET_SIZE, text_storage.begin(), text_storage.end() );
///
/// // prepare their offset vectors
/// thrust::sequence( pattern_offsets.begin(), pattern_offsets.begin() + n_strings+1, 0u, pattern_len );
/// thrust::sequence( text_offsets.begin(),    text_offsets.begin()    + n_strings+1, 0u, text_len );
///
/// // prepare a vector of alignment sinks
/// thrust::device_vector< aln::BestSink<uint32> > sinks( n_strings );
///
/// // and execute the batch alignment
/// aln::batch_alignment_score(
///     aln::make_edit_distance_aligner<aln::SEMI_GLOBAL, MyersTag<ALPHABET_SIZE> >(),
///     make_concatenated_string_set( n_strings, pattern_storage.begin(), pattern_offsets.begin() ),
///     make_concatenated_string_set( n_strings, text_storage.begin(),    text_offsets.begin() ),
///     sinks.begin(),
///     aln::DeviceThreadScheduler() );
///\endcode
///
/// \section TechnicalOverviewSection Technical Overview
///\par
/// A complete list of the classes and functions in this module is given in the \ref Alignment documentation.
///

namespace nvbio {

///@addtogroup Alignment
///@{
namespace aln {
///@}

///
///@addtogroup Alignment Alignment Module
/// This module contains a series of functions to perform string alignment.
/// The the kind of alignment performed is selected according to an
/// \ref Aligner "Aligner".
///@{
///

/// Compute the alignment score between a pattern and a text string 
/// with banded DP alignment.
///
/// \tparam BAND_LEN            size of the DP band
///
/// \param aligner             alignment algorithm
/// \param pattern             pattern string
/// \param quals               quality string
/// \param text                text string
/// \param min_score           threshold alignment score
/// \param sink                output sink
///
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
          sink_type&        sink);

/// Compute the alignment score between a pattern and a text string 
/// with banded DP alignment.
///
/// \tparam BAND_LEN            size of the DP band
///
/// \param aligner             alignment algorithm
/// \param pattern             pattern string
/// \param text                text string
/// \param min_score           threshold alignment score
/// \param sink                output sink
///
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
    const text_string       text,
    const  int32            min_score,
          sink_type&        sink);

/// Compute the alignment score between a pattern and a text string 
/// with banded DP alignment.
///
/// \tparam BAND_LEN            size of the DP band
///
/// \param aligner             alignment algorithm
/// \param pattern             pattern string
/// \param quals               quality string
/// \param text                text string
/// \param min_score           threshold alignment score
///
/// \return                     best alignment score
///
template <
    uint32          BAND_LEN,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
int32 banded_alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const qual_string       quals,
    const text_string       text,
    const  int32            min_score);

/// Compute the alignment score between a pattern and a text string 
/// with banded DP alignment.
///
/// \tparam BAND_LEN            size of the DP band
///
/// \param aligner              alignment algorithm
/// \param pattern              pattern string
/// \param text                 text string
/// \param min_score            threshold alignment score
///
/// \return                     best alignment score
///
template <
    uint32          BAND_LEN,
    typename        aligner_type,
    typename        pattern_string,
    typename        qual_string,
    typename        text_string,
    typename        sink_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
int32 banded_alignment_score(
    const aligner_type      aligner,
    const pattern_string    pattern,
    const text_string       text,
    const  int32            min_score);

///
/// Backtrace an alignment using a banded DP algorithm.
///
/// \tparam BAND_LEN            size of the DP band
/// \tparam CHECKPOINTS         number of DP rows between each checkpoint
/// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
/// \tparam pattern_string      a string representing the pattern.
/// \tparam qual_string         an array representing the pattern qualities.
/// \tparam text_string         a string representing the text.
/// \tparam checkpoints_type    an array-like class defining operator[], used to represent a reduced DP score matrix,
///                             containing all the matrix columns whose index is a multiple of CHECKPOINTS;
///                             the type of the matrix cells depends on the aligner, and can be obtained as
///                             typename checkpoint_storage_type<aligner_type>::type
/// \tparam submatrix_type      an array-like class defining operator[], used to represent a temporary DP flow submatrix,
///                             containing all the matrix flow cells between two checkpointed columns.
///                             the number of bits needed for the submatrix cells depends on the aligner, and can be obtained as
///                             direction_vector_traits<aligner_type>::BITS
/// \tparam backtracer_type     a model of \ref Backtracer.
///
/// \param aligner              alignment algorithm
/// \param pattern              pattern to be aligned
/// \param quals                pattern quality scores
/// \param text                 text to align the pattern to
/// \param min_score            minimum accepted score
/// \param backtracer           backtracking delegate
/// \param checkpoints          temporary checkpoints storage
/// \param submatrix            temporary submatrix storage
///
/// \return                     reported alignment
///
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
    submatrix_type          submatrix);


/// Compute the alignment score between a pattern and a text string 
/// with full DP alignment.
///
/// This is a low-level function, requiring all needed temporary storage to be passed from the caller.
/// The purpose is allowing the caller to allocate such storage (possibly among kernel threads) using different
/// strategies.
///
/// Example:
/// \code
/// const_string text = make_string("AAAAGGGTGCTCAA");
/// const_string pattern  = make_string("GGGTGCTCAA");
///
/// typedef aln::GotohAligner<aln::LOCAL,aln::SimpleGotohScheme> aligner_type;
/// typedef aln::column_storage_type<aligner_type>::type         cell_type;
///
/// cell_type column[14]; // a column as big as the text
///
/// const aln::SimpleGotohScheme scoring(                       // build a Gotoh scoring scheme
///      2,                                                     // match bonus
///     -1,                                                     // mismatch penalty
///     -1,                                                     // gap open penalty
///     -1 )                                                    // gap extension penalty
///
/// aln::Best2Sink<int32> best2;                                // keep the best 2 scores
/// aln::alignment_score(
///        aln::make_gotoh_aligner<aln::LOCAL>( scoring ),      // build a local Gotoh aligner
///        pattern,                                             // pattern string
///        aln::trivial_quality_string(),                       // pattern qualities
///        text,                                                // text string
///        -255,                                                // minimum accepted score
///        best2,                                               // alignment sink
///        column );                                            // temporary column storage
/// \endcode
///
/// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
/// \tparam pattern_string      a string representing the pattern.
/// \tparam qual_string         an array representing the pattern qualities.
/// \tparam text_string         a string representing the text.
/// \tparam sink_type           an \ref AlignmentSink "Alignment Sink".
/// \tparam column_type         an array-like class defining operator[], used to represent a matrix column;
///                             the type of the matrix cells depends on the aligner, and can be obtained as
///                             typename column_storage_type<aligner_type>::type
///
/// \param aligner             alignment algorithm
/// \param pattern             pattern string
/// \param quals               quality string
/// \param text                text string
/// \param min_score           threshold alignment score
/// \param sink                output sink
/// \param column              temporary storage for a matrix column, must be at least as large as the text
///
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
          column_type       column);

/// Compute the alignment score between a pattern and a text string 
/// with full DP alignment.
///
/// This is a high level function, allocating all necessary temporary storage in local memory.
/// In order to size the allocations properly, the function requires compile-time knowledge
/// of the maximum text length.
///
/// Example:
/// \code
/// const_string text = make_string("AAAAGGGTGCTCAA");
/// const_string pattern  = make_string("GGGTGCTCAA");
///
/// const aln::SimpleGotohScheme scoring(                       // build a Gotoh scoring scheme
///      2,                                                     // match bonus
///     -1,                                                     // mismatch penalty
///     -1,                                                     // gap open penalty
///     -1 )                                                    // gap extension penalty
///
/// aln::Best2Sink<int32> best2;                                // keep the best 2 scores
/// aln::alignment_score<14>(                                   // MAX_TEXT_LEN
///        aln::make_gotoh_aligner<aln::LOCAL>( scoring ),      // build a local Gotoh aligner
///        pattern,                                             // pattern string
///        aln::trivial_quality_string(),                       // pattern qualities
///        text,                                                // text string
///        -255,                                                // minimum accepted score
///        best2 );                                             // alignment sink
/// \endcode
///
/// \tparam MAX_TEXT_LEN        specifies the maximum text length, used to allocate
///                             statically the necessary temporary storage.
///
/// \param aligner             alignment algorithm
/// \param pattern             pattern string
/// \param quals               quality string
/// \param text                text string
/// \param min_score           threshold alignment score
/// \param sink                output sink
///
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
          sink_type&        sink);

///
/// Backtrace an optimal alignment using a full DP algorithm.
///
/// This is a low-level function, requiring all needed temporary storage to be passed from the caller.
/// The purpose is allowing the caller to allocate such storage (possibly among kernel threads) using different
/// strategies.
///
/// Example:
/// \code
/// const uint32 TEXT_LEN = 64;
/// const uint32 PATTERN_LEN = 16;
/// const_string text = make_random_string(TEXT_LEN);
/// const_string pattern  = make_random_string(PATTERN_LEN);
///
/// typedef aln::GotohAligner<aln::LOCAL,aln::SimpleGotohScheme> aligner_type;
/// typedef aln::column_storage_type<aligner_type>::type         cell_type;
///
/// const uint32 CHECKPOINTS = 16;
///
/// // column storage
/// cell_type column[TEXT_LEN]; // a column as big as the text
///
/// // checkpoints storage
/// cell_type checkpoints[TEXT_LEN*util::divide_ri(PATTERN_LEN,CHECKPOINTS) ];
///
/// // submatrix storage (packed using a few bits per cell)
/// const uint32 SUBMATRIX_BITS = direction_vector_traits<aligner_type>::BITS;
/// typedef PackedStream<uint32*,uint8,SUBMATRIX_BITS,false> submatrix_type;
/// uint32 submatrix_storage[ util::divide_ri(TEXT_LEN*CHECKPOINTS*SUBMATRIX_BITS,32) ];
/// submatrix_type submatrix( submatrix_storage );
///
/// const aln::SimpleGotohScheme scoring(                       // build a Gotoh scoring scheme
///      2,                                                     // match bonus
///     -1,                                                     // mismatch penalty
///     -1,                                                     // gap open penalty
///     -1 )                                                    // gap extension penalty
///
/// MyBacktracer backtracer;                                    // my backtracing context
/// aln::alignment_traceback(
///        aln::make_gotoh_aligner<aln::LOCAL>( scoring ),      // build a local Gotoh aligner
///        pattern,                                             // pattern string
///        aln::trivial_quality_string(),                       // pattern qualities
///        text,                                                // text string
///        -255,                                                // minimum accepted score
///        backtracer,                                          // backtracing context
///        checkpoints,                                         // temporary checkpoints storage
///        submatrix,                                           // temporary submatrix storage
///        column );                                            // temporary column storage
/// \endcode
///
/// \tparam CHECKPOINTS         number of DP rows between each checkpoint
/// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
/// \tparam pattern_string      a string representing the pattern.
/// \tparam qual_string         an array representing the pattern qualities.
/// \tparam text_string         a string representing the text.
/// \tparam checkpoints_type    an array-like class defining operator[], used to represent a reduced DP score matrix,
///                             containing all the matrix columns whose index is a multiple of CHECKPOINTS;
///                             the type of the matrix cells depends on the aligner, and can be obtained as
///                             typename checkpoint_storage_type<aligner_type>::type;
///                             the array must contain at least text.length()*((pattern.length() + CHECKPOINTS-1)/CHECKPOINTS) entries
/// \tparam submatrix_type      an array-like class defining operator[], used to represent a temporary DP flow submatrix,
///                             containing all the matrix flow cells between two checkpointed columns.
///                             the number of bits needed for the submatrix cells depends on the aligner, and can be obtained as
///                             direction_vector_traits<aligner_type>::BITS
/// \tparam backtracer_type     a model of \ref Backtracer.
/// \tparam column_type         an array-like class defining operator[], used to represent a matrix column;
///                             the type of the matrix cells depends on the aligner, and can be obtained as
///                             typename column_storage_type<aligner_type>::type
///
/// \param aligner              alignment algorithm
/// \param pattern              pattern to be aligned
/// \param quals                pattern quality scores
/// \param text                 text to align the pattern to
/// \param min_score            minimum accepted score
/// \param backtracer           backtracking delegate
/// \param checkpoints          temporary checkpoints storage
/// \param submatrix            temporary submatrix storage
/// \param column               temporary storage for a matrix column, must be at least as large as the text
///
/// \return                     reported alignment
///
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
    column_type             column);

///
/// Backtrace an optimal alignment using a full DP algorithm.
///
/// This is a high level function, allocating all necessary temporary storage in local memory.
/// In order to size the allocations properly, the function requires compile-time knowledge
/// of the maximum pattern and text length.
///
/// \tparam MAX_PATTERN_LEN     maximum pattern length
/// \tparam MAX_TEXT_LEN        maximum text length
/// \tparam CHECKPOINTS         number of DP rows between each checkpoint
/// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
/// \tparam pattern_string      a string representing the pattern.
/// \tparam qual_string         an array representing the pattern qualities.
/// \tparam text_string         a string representing the text.
/// \tparam backtracer_type     a model of \ref Backtracer.
///
/// \param aligner              alignment algorithm
/// \param pattern              pattern to be aligned
/// \param quals                pattern quality scores
/// \param text                 text to align the pattern to
/// \param min_score            minimum accepted score
/// \param backtracer           backtracking delegate
///
/// \return                     reported alignment
///
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
    backtracer_type&        backtracer);

#if defined(__CUDACC__)
namespace warp {

/// Compute the alignment score between a pattern and a text string with full DP alignment using a warp.
///
/// This is a low-level function, requiring all needed temporary storage to be passed from the caller.
/// The purpose is allowing the caller to allocate such storage (possibly among kernel threads) using different
/// strategies.
///
/// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
/// \tparam pattern_string      a string representing the pattern.
/// \tparam qual_string         an array representing the pattern qualities.
/// \tparam text_string         a string representing the text.
/// \tparam column_type         an array-like class defining operator[], used to represent a partial matrix column;
///                             the type of the matrix cells depends on the aligner, and can be obtained as
///                             typename column_storage_type<aligner_type>::type;
///
/// \param aligner             alignment algorithm
/// \param pattern             pattern string
/// \param quals               quality string
/// \param text                text string
/// \param min_score           threshold alignment score
/// \param sink                output sink
/// \param column              temporary storage for a matrix column, must be at least as large as the text
///
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
          column_type       column);

} // namespace warp
#endif

///@} // end of the Alignment group

} // namespace aln
} // namespace nvbio

#include <nvbio/alignment/alignment_inl.h>
#include <nvbio/alignment/banded_inl.h>
#include <nvbio/alignment/utils.h>
