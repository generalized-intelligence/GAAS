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

#include <nvbio/alignment/alignment.h>

namespace nvbio {
namespace aln {

///@addtogroup Alignment
///@{

///
///@defgroup BatchScheduler Batch Schedulers
/// A Batch Scheduler is a tag specifying the algorithm used to execute a batch of jobs in parallel.
/// Four such algorithms are currently available:
///
///     - HostThreadScheduler
///     - DeviceThreadScheduler (inheriting from DeviceThreadBlockScheduler)
///     - DeviceStagedThreadScheduler
///     - DeviceWarpScheduler
///@{

/// Identify a thread-parallel \ref BatchScheduler "batch scheduling" algorithm
///
struct HostThreadScheduler {};

/// Identify a device thread-parallel \ref BatchScheduler "batch scheduling" algorithm, specifying
/// the CUDA kernel grid configuration
///
/// \tparam BLOCKDIM_T          thread-block (CTA) size
/// \tparam MINBLOCKS_T         minimum number of blocks per SM
///
template <uint32 BLOCKDIM_T, uint32 MINBLOCKS_T>
struct DeviceThreadBlockScheduler
{
    static const uint32 BLOCKDIM  = BLOCKDIM_T;
    static const uint32 MINBLOCKS = MINBLOCKS_T;
};

/// Identify a staged thread-parallel \ref BatchScheduler "batch scheduling" algorithm
///
typedef DeviceThreadBlockScheduler<128,1> DeviceThreadScheduler;

/// Identify a staged thread-parallel \ref BatchScheduler "batch scheduling" algorithm
///
struct DeviceStagedThreadScheduler {};

/// Identify a warp-parallel \ref BatchScheduler "batch scheduling" algorithm
///
struct DeviceWarpScheduler {};

template <typename T,typename Scheduler>
struct supports_scheduler { static const bool pred = false; };

template <AlignmentType TYPE, typename AlgorithmTag> struct supports_scheduler<EditDistanceAligner<TYPE,AlgorithmTag>, HostThreadScheduler>         { static const bool pred = true; };
template <AlignmentType TYPE, typename AlgorithmTag> struct supports_scheduler<EditDistanceAligner<TYPE,AlgorithmTag>, DeviceThreadScheduler>       { static const bool pred = true; };
template <AlignmentType TYPE, typename AlgorithmTag> struct supports_scheduler<EditDistanceAligner<TYPE,AlgorithmTag>, DeviceStagedThreadScheduler> { static const bool pred = true; };
template <AlignmentType TYPE, typename AlgorithmTag> struct supports_scheduler<EditDistanceAligner<TYPE,AlgorithmTag>, DeviceWarpScheduler>         { static const bool pred = true; };

template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<SmithWatermanAligner<TYPE,ScoringScheme,AlgorithmTag>, HostThreadScheduler>          { static const bool pred = true; };
template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<SmithWatermanAligner<TYPE,ScoringScheme,AlgorithmTag>, DeviceThreadScheduler>        { static const bool pred = true; };
template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<SmithWatermanAligner<TYPE,ScoringScheme,AlgorithmTag>, DeviceStagedThreadScheduler>  { static const bool pred = true; };
template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<SmithWatermanAligner<TYPE,ScoringScheme,AlgorithmTag>, DeviceWarpScheduler>          { static const bool pred = true; };

template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<GotohAligner<TYPE,ScoringScheme,AlgorithmTag>, HostThreadScheduler>                  { static const bool pred = true; };
template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<GotohAligner<TYPE,ScoringScheme,AlgorithmTag>, DeviceThreadScheduler>                { static const bool pred = true; };
template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<GotohAligner<TYPE,ScoringScheme,AlgorithmTag>, DeviceStagedThreadScheduler>          { static const bool pred = true; };
template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<GotohAligner<TYPE,ScoringScheme,AlgorithmTag>, DeviceWarpScheduler>                  { static const bool pred = true; };

template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<HammingDistanceAligner<TYPE,ScoringScheme,AlgorithmTag>, HostThreadScheduler>            { static const bool pred = true; };
template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<HammingDistanceAligner<TYPE,ScoringScheme,AlgorithmTag>, DeviceThreadScheduler>          { static const bool pred = true; };
template <AlignmentType TYPE, typename ScoringScheme, typename AlgorithmTag> struct supports_scheduler<HammingDistanceAligner<TYPE,ScoringScheme,AlgorithmTag>, DeviceStagedThreadScheduler>    { static const bool pred = true; };

///@} // end of BatchScheduler group

///
///@defgroup BatchAlignment Batch Alignments
/// A batch alignment is a parallel execution context for performing large batches of
/// individual alignment jobs.
/// There are several types of alignment jobs, mirroring the types of possible alignment
/// operations, such as \ref ScoringSection "scoring", \ref TracebackSection "traceback", etc.
///
/// Batches can be executed according to several different parallel algorithms, specified
/// by a \ref BatchScheduler "Batch Scheduler".
///
///@{

///
/// A convenience function for aligning a batch of patterns to a corresponding batch of texts on
/// the device.
/// <b><em>NOTE</em></b>: this function allocates temporary storage, and doesn't perform
/// any local memory caching of the input strings: hence, it might not attain maximum performance.
/// For maximum speed, check the BatchedAlignmentScore contexts.
///\par
/// All the involved string sets and iterators must reside in <em>device memory</em>.
///\par
/// For a usage example, see \ref BatchAlignmentSection.
///
/// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
/// \tparam pattern_set_type    a string set storing the patterns
/// \tparam text_set_type       a string set storing the texts
/// \tparam sink_iterator       a random access iterator to the output alignment sinks
/// \tparam scheduler_type      a \ref BatchScheduler "Batch Scheduler"
///
/// \param aligner              the \ref Aligner "Aligner" algorithm
/// \param patterns             the patterns string set
/// \param texts                the texts string set
/// \param sinks                the output alignment sinks
/// \param scheduler            the \ref BatchScheduler "Batch Scheduler"
///
template <
    typename aligner_type,
    typename pattern_set_type,
    typename text_set_type,
    typename sink_iterator,
    typename scheduler_type>
void batch_alignment_score(
    const aligner_type      aligner,
    const pattern_set_type  patterns,
    const text_set_type     texts,
          sink_iterator     sinks,
    const scheduler_type    scheduler,
    const uint32            max_pattern_length = 1000,
    const uint32            max_text_length = 1000);

///
/// A convenience function for aligning a batch of patterns to a corresponding batch of texts on
/// the device.
/// <b><em>NOTE</em></b>: this function allocates temporary storage, and doesn't perform
/// any local memory caching of the input strings: hence, it might not attain maximum performance.
/// For maximum speed, check the BatchedAlignmentScore contexts.
///\par
/// All the involved string sets and iterators must reside in <em>device memory</em>.
///\par
/// For a usage example, see \ref BatchAlignmentSection.
///
/// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
/// \tparam pattern_set_type    a string set storing the patterns
/// \tparam qualities_set_type  a string set storing the qualities
/// \tparam text_set_type       a string set storing the texts
/// \tparam sink_iterator       a random access iterator to the output alignment sinks
/// \tparam scheduler_type      a \ref BatchScheduler "Batch Scheduler"
///
/// \param aligner              the \ref Aligner "Aligner" algorithm
/// \param patterns             the patterns string set
/// \param quals                the pattern qualities string set
/// \param texts                the texts string set
/// \param sinks                the output alignment sinks
/// \param scheduler            the \ref BatchScheduler "Batch Scheduler"
///
template <
    typename aligner_type,
    typename pattern_set_type,
    typename qualities_set_type,
    typename text_set_type,
    typename sink_iterator,
    typename scheduler_type>
void batch_alignment_score(
    const aligner_type          aligner,
    const pattern_set_type      patterns,
    const qualities_set_type    quals,
    const text_set_type         texts,
          sink_iterator         sinks,
    const scheduler_type        scheduler,
    const uint32                max_pattern_length = 1000,
    const uint32                max_text_length = 1000);

///
/// A convenience function for aligning a batch of patterns to a corresponding batch of texts on
/// the device.
/// <b><em>NOTE</em></b>: this function allocates temporary storage, and doesn't perform
/// any local memory caching of the input strings: hence, it might not attain maximum performance.
/// For maximum speed, check the BatchedAlignmentScore contexts.
///\par
/// All the involved string sets and iterators must reside in <em>device memory</em>.
///\par
/// For a usage example, see \ref BatchAlignmentSection.
///
/// \tparam aligner_type        an \ref Aligner "Aligner" algorithm
/// \tparam pattern_set_type    a string set storing the patterns
/// \tparam text_set_type       a string set storing the texts
/// \tparam sink_iterator       a random access iterator to the output alignment sinks
/// \tparam scheduler_type      a \ref BatchScheduler "Batch Scheduler"
///
/// \param aligner              the \ref Aligner "Aligner" algorithm
/// \param patterns             the patterns string set
/// \param texts                the texts string set
/// \param sinks                the output alignment sinks
/// \param scheduler            the \ref BatchScheduler "Batch Scheduler"
///
template <
    uint32   BAND_LEN,
    typename aligner_type,
    typename pattern_set_type,
    typename text_set_type,
    typename sink_iterator,
    typename scheduler_type>
void batch_banded_alignment_score(
    const aligner_type      aligner,
    const pattern_set_type  patterns,
    const text_set_type     texts,
          sink_iterator     sinks,
    const scheduler_type    scheduler,
    const uint32            max_pattern_length,
    const uint32            max_text_length);

///
/// Execution context for a batch of alignment jobs.
///
/// \tparam stream_type     the stream of alignment jobs
/// \tparam algorithm_type  a \ref BatchScheduler "Batch Scheduler" specifier
///
/// This class has to provide the following interface:
///
/// \code
/// stream_type
/// {
///     typedef ... aligner_type;           // define the aligner type
///
///     // an alignment context
///     struct context_type
///     {
///         int32           min_score;      // the threshold score for a valid alignment
///         sink_type       sink;           // alignment sink
///         ...                             // private data
///     };
///     // a container for the strings to be aligned
///     struct strings_type
///     {
///         pattern_type    pattern;        // the pattern to align
///         qual_type       quals;          // the pattern qualities
///         text_type       text;           // the text to align
///         ...                             // private data
///     };
///
///     // return the aligner
///     aligner_type aligner() const;
///
///     // return the maximum pattern length
///     uint32 max_pattern_length() const;
///
///     // return the maximum text length
///     uint32 max_text_length() const;
///
///     // stream size
///     uint32 size() const;
///
///     // return the i-th pattern length
///     uint32 pattern_length(const uint32 i, context_type* context) const;
///
///     // return the i-th text length
///     uint32 text_length(const uint32 i, context_type* context) const;
///
///     // initialize the context relative to the i-th alignment;
///     // return false if the alignment is to be skipped
///     bool init_context(
///          const uint32        i,                             // job index
///          context_type*       context) const;                // output job context
///
///     // get the context relative to the i-th alignment
///     void load_strings(
///          const uint32        i,                             // job index
///          const uint32        pattern_begin,                 // pattern window begin
///          const uint32        pattern_end                    // pattern window end
///          context_type*       context,                       // job context
///          strings_type*       strings) const;                // output strings
///
///     // consume the output
///     void output(const uint32 i, context_type* context);
/// };
/// \endcode
/// While this design might seem a little convoluted it is motivated by efficiency and generality.
/// In fact, it does:
///    - allow the scheduler to break the alignment process into multiple stages while performing
///      initializations only once through the stream_type::init_context() method;
///    - allow the user to perform string caching in registers/local memory: in fact, schedulers will
///      instantiate the strings_type class on the stack, thus guaranteeing that any user-defined private
///      data will be placed either in registers or local memory.
///      The stream_type::load_strings() method is then free to use such private data to
///      implement a cache for any of the involved strings. With this design cached strings will
///      then be allowed to just contain pointers to the caches, making sure that they can be
///      passed by value without inadvertedly copying the caches themselves.
///
template <
    typename stream_type,
    typename algorithm_type = DeviceThreadScheduler>
struct BatchedAlignmentScore
{
    typedef typename stream_type::aligner_type                      aligner_type;
    typedef typename checkpoint_storage_type<aligner_type>::type    cell_type;

    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL);
};

/// see BatchedAlignmentScore
/// 
template <
    uint32 BAND_LEN,
    typename stream_type,
    typename algorithm_type = DeviceThreadScheduler>
struct BatchedBandedAlignmentScore
{
    typedef typename stream_type::aligner_type                      aligner_type;
    typedef typename checkpoint_storage_type<aligner_type>::type    cell_type;

    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL);
};

///
/// Consume a stream of alignment traceback jobs
///
/// \tparam stream_type     the stream of alignment jobs
/// \tparam algorithm_type  a \ref BatchScheduler "Batch Scheduler" specifier
///
/// This class has to provide the following interface:
///
/// \code
/// stream_type
/// {
///     typedef ... aligner_type;           // define the aligner type
///
///     // an alignment context
///     struct context_type
///     {
///         int32            min_score;     // the threshold score for a valid alignment
///         backtracer_type  backtracer;    // alignment backtracer
///         Alignment<int32> alignment;     // alignment result
///         ...                             // private data
///     };
///     // a container for the strings to be aligned
///     struct strings_type
///     {
///         pattern_type    pattern;        // the pattern to align
///         qual_type       quals;          // the pattern qualities
///         text_type       text;           // the text to align
///         ...                             // private data
///     };
///
///     // return the aligner
///     aligner_type aligner() const;
///
///     // return the maximum pattern length
///     uint32 max_pattern_length() const;
///
///     // return the maximum text length
///     uint32 max_text_length() const;
///
///     // stream size
///     uint32 size() const;
///
///     // return the i-th pattern length
///     uint32 pattern_length(const uint32 i, context_type* context) const;
///
///     // return the i-th text length
///     uint32 text_length(const uint32 i, context_type* context) const;
///
///     // initialize the context relative to the i-th alignment
///     bool init_context(
///          const uint32        i,                             // job index
///          context_type*       context) const;                // output job context
///
///     // get the context relative to the i-th alignment
///     void load_strings(
///          const uint32        i,                             // job index
///          const uint32        pattern_begin,                 // pattern window begin
///          const uint32        pattern_end                    // pattern window end
///          context_type*       context,                       // job context
///          strings_type*       strings) const;                // output strings
///
///     // consume the output
///     void output(const uint32 i, context_type* context);
/// };
/// \endcode
/// While this design might seem a little convoluted it is motivated by efficiency and generality.
/// In fact, it does:
///    - allow the scheduler to break the alignment process into multiple stages while performing
///      initializations only once through the stream_type::init_context() method;
///    - allow the user to perform string caching in registers/local memory: in fact, schedulers will
///      instantiate the strings_type class on the stack, thus guaranteeing that any user-defined private
///      data will be placed either in registers or local memory.
///      The stream_type::load_strings() method is then free to use such private data to
///      implement a cache for any of the involved strings. With this design cached strings will
///      then be allowed to just contain pointers to the caches, making sure that they can be
///      passed by value without inadvertedly copying the caches themselves.
///
template <
    uint32      CHECKPOINTS,
    typename    stream_type,
    typename    algorithm_type = DeviceThreadScheduler>
struct BatchedAlignmentTraceback
{
    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL);
};

///
/// Consume a stream of banded alignment traceback jobs
///
/// \tparam stream_type     the stream of alignment jobs
/// \tparam algorithm_type  a \ref BatchScheduler "Batch Scheduler" specifier
///
/// See BatchedAlignmentTraceback.
///
template <
    uint32      BAND_LEN,
    uint32      CHECKPOINTS,
    typename    stream_type,
    typename    algorithm_type = DeviceThreadScheduler>
struct BatchedBandedAlignmentTraceback
{
    /// return the minimum number of bytes required by the algorithm
    ///
    static uint64 min_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// return the maximum number of bytes required by the algorithm
    ///
    static uint64 max_temp_storage(const uint32 max_pattern_len, const uint32 max_text_len, const uint32 stream_size);

    /// enact the batch execution
    ///
    void enact(stream_type stream, uint64 temp_size = 0u, uint8* temp = NULL);
};

///@} // end of BatchAlignment group

///@} // end of the Alignment group

} // namespace aln
} // namespace nvbio

#include <nvbio/alignment/batched_inl.h>
#include <nvbio/alignment/batched_banded_inl.h>
