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

#include <nvBowtie/bowtie2/cuda/aligner.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

///@addtogroup nvBowtie
///@{

///@defgroup PipelineStates Pipeline States
///
/// This module defines a set of classes that represent the state of the various pipelines on the device.
/// All their members are device-views - objects and arrays that can and should be dereferenced from
/// device kernels only.
///
/// There are a few different classes, corresponding to different scoring and traceback pipelines:
///
/// - BaseScoringPipelineState: the basic scoring pipeline class every scoring pipeline inherits from
/// - BestApproxScoringPipelineState: the best-approximate scoring pipeline
/// - TracebackPipelineState: the traceback pipeline
///

///@addtogroup PipelineStates
///@{

///
///  Base pipeline state object that other scoring pipeline classes will inherit from (see \ref PipelineStates).
///
template <typename ScoringScheme>
struct BaseScoringPipelineState
{
    typedef Aligner::fmi_type                           fmi_type;
    typedef Aligner::rfmi_type                          rfmi_type;
    typedef Aligner::read_batch_type                    read_batch_type;
    typedef Aligner::genome_iterator                    genome_iterator;
    typedef ScoringScheme                               scheme_type;

    BaseScoringPipelineState(
        const uint32            _anchor,
        const read_batch_type   _reads,
        const read_batch_type   _reads_o,
        const uint32            _genome_len,
        const genome_iterator   _genome,
        const fmi_type          _fmi,
        const rfmi_type         _rfmi,
        const ScoringScheme     _scoring_scheme,
        const int32             _score_limit,
        Aligner&                _aligner) :
        anchor                  ( _anchor ),
        reads                   ( _reads ),
        reads_o                 ( _reads_o ),
        genome_length           ( _genome_len ),
        genome                  ( _genome ),
        fmi                     ( _fmi ),
        rfmi                    ( _rfmi ),
        hits                    ( _aligner.hit_deques.device_view() ),
        scoring_queues          ( _aligner.scoring_queues.device_view() ),
        opposite_queue          ( _aligner.opposite_queue_dptr ),
        dp_buffer               ( _aligner.dp_buffer_dptr ),
        dp_buffer_size          ( _aligner.dp_buffer_dvec.size() ),
        scoring_scheme          ( _scoring_scheme ),
        score_limit             ( _score_limit )
    {}

    /// return the set of reads corresponding to a given mate
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const read_batch_type& get_reads(const uint32 mate) const { return mate ? reads_o : reads; }

    const uint32                    anchor;
    const read_batch_type           reads;
    const read_batch_type           reads_o;

    const uint32                    genome_length;
    const genome_iterator           genome;

    const fmi_type                  fmi;
    const rfmi_type                 rfmi;

    SeedHitDequeArrayDeviceView     hits;

    ScoringQueuesDeviceView         scoring_queues;

    uint32                          n_hits_per_read;
    uint32                          hits_queue_size;
    uint32*                         idx_queue;
    uint32                          opposite_queue_size;
    uint32*                         opposite_queue;

    uint8*                          dp_buffer;
    uint64                          dp_buffer_size;

    const ScoringScheme             scoring_scheme;
    const int32                     score_limit;
};

///
/// This object encapsulates the pipeline state for scoring best-approximate alignments (see \ref PipelineStates).
///
template <typename ScoringScheme>
struct BestApproxScoringPipelineState : public BaseScoringPipelineState<ScoringScheme>
{
    typedef BaseScoringPipelineState<ScoringScheme>     base_type;
    typedef Aligner::fmi_type                           fmi_type;
    typedef Aligner::rfmi_type                          rfmi_type;
    typedef Aligner::read_batch_type                    read_batch_type;
    typedef Aligner::genome_iterator                    genome_iterator;
    typedef ScoringScheme                               scheme_type;

    BestApproxScoringPipelineState(
        const uint32                    _anchor,
        const read_batch_type           _reads,
        const read_batch_type           _reads_o,
        const uint32                    _genome_len,
        const genome_iterator           _genome,
        const fmi_type                  _fmi,
        const rfmi_type                 _rfmi,
        const ScoringScheme             _scoring_scheme,
        const int32                     _score_limit,
        Aligner&                        _aligner) :
        base_type(
            _anchor,
            _reads,
            _reads_o,
            _genome_len,
            _genome,
            _fmi,
            _rfmi,
            _scoring_scheme,
            _score_limit,
            _aligner ),
        trys                    ( _aligner.trys_dptr ),
        rseeds                  ( _aligner.rseeds_dptr ),
        best_alignments         ( _aligner.best_data_dptr ),
        best_alignments_o       ( _aligner.best_data_dptr_o ),
        best_stride             ( _aligner.BATCH_SIZE )
    {}

    uint32*                         trys;
    uint32*                         rseeds;

    io::Alignment*                  best_alignments;
    io::Alignment*                  best_alignments_o;
    uint32                          best_stride;
};

///
/// This object encapsulates the pipeline state for scoring best-approximate alignments (see \ref PipelineStates).
///
template <typename ScoringScheme>
struct AllMappingPipelineState : public BaseScoringPipelineState<ScoringScheme>
{
    typedef BaseScoringPipelineState<ScoringScheme>     base_type;
    typedef Aligner::fmi_type                           fmi_type;
    typedef Aligner::rfmi_type                          rfmi_type;
    typedef Aligner::read_batch_type                    read_batch_type;
    typedef Aligner::genome_iterator                    genome_iterator;
    typedef ScoringScheme                               scheme_type;

    AllMappingPipelineState(
        const uint32            _anchor,
        const read_batch_type   _reads,
        const read_batch_type   _reads_o,
        const uint32            _genome_len,
        const genome_iterator   _genome,
        const fmi_type          _fmi,
        const rfmi_type         _rfmi,
        const ScoringScheme     _scoring_scheme,
        const int32             _score_limit,
        Aligner&                _aligner) :
        base_type(
            _anchor,
            _reads,
            _reads_o,
            _genome_len,
            _genome,
            _fmi,
            _rfmi,
            _scoring_scheme,
            _score_limit,
            _aligner ),
        output_read_info        ( _aligner.output_read_info_dptr ),
        buffer_read_info        ( _aligner.buffer_read_info_dptr ),
        buffer_alignments       ( _aligner.buffer_alignments_dptr ),
        cigar                   ( nvbio::device_view( _aligner.cigar ) ),
        cigar_coords            ( nvbio::device_view( _aligner.cigar_coords_dvec ) ),
        mds                     ( nvbio::device_view( _aligner.mds ) ),
        dp_buffer               ( nvbio::device_view( _aligner.dp_buffer_dvec ) ),
        dp_buffer_size          ( _aligner.dp_buffer_dvec.size() )
    {}

    uint32*                                         output_read_info;
    uint32*                                         buffer_read_info;
    io::Alignment*                                  buffer_alignments;

    nvbio::VectorArrayView<io::Cigar>               cigar;          ///< cigar arena
    uint2*                                          cigar_coords;   ///< cigar coords
    nvbio::VectorArrayView<uint8>                   mds;            ///< mds arena

    uint8*                                          dp_buffer;      ///< DP buffer
    uint64                                          dp_buffer_size; ///< DP buffer size
};

///
/// This object encapsulates the state of the pipeline during traceback.
/// As such, it includes references to the following objects:
///
///     * the reads for both mates
///     * the genome iterator
///     * the CIGAR and MD string arenas
///     * the temporary DP buffer
///
template <typename ScoringScheme>
struct TracebackPipelineState
{
    typedef Aligner::read_batch_type                    read_batch_type;
    typedef Aligner::genome_iterator                    genome_iterator;
    typedef ScoringScheme                               scheme_type;

    /// constructor
    ///
    /// \param  _reads1         the reads corresponding to mate #1
    /// \param  _reads2         the reads corresponding to mate #2
    /// \param  _genome_len     the length of the genome
    /// \param  _genome         the genome iterator
    /// \param  _aligner        the aligner object
    TracebackPipelineState(
        const read_batch_type   _reads1,
        const read_batch_type   _reads2,
        const uint32            _genome_len,
        const genome_iterator   _genome,
        const ScoringScheme     _scoring_scheme,
        Aligner&                _aligner) :
        reads                   ( _reads1 ),
        reads_o                 ( _reads2 ),
        genome_length           ( _genome_len ),
        genome                  ( _genome ),
        scoring_scheme          ( _scoring_scheme ),
        cigar                   ( nvbio::device_view( _aligner.cigar ) ),
        cigar_coords            ( nvbio::device_view( _aligner.cigar_coords_dvec ) ),
        mds                     ( nvbio::device_view( _aligner.mds ) ),
        dp_buffer               ( nvbio::device_view( _aligner.dp_buffer_dvec ) ),
        dp_buffer_size          ( _aligner.dp_buffer_dvec.size() )
    {}

    /// return the set of reads corresponding to a given mate
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const read_batch_type& get_reads(const uint32 mate) const { return mate ? reads_o : reads; }

    const read_batch_type                           reads;          ///< first mates
    const read_batch_type                           reads_o;        ///< second mates

    const uint32                                    genome_length;  ///< genome length
    const genome_iterator                           genome;         ///< genome iterator

    const ScoringScheme                             scoring_scheme; ///< scoring scheme

    nvbio::VectorArrayView<io::Cigar>               cigar;          ///< cigar arena
    uint2*                                          cigar_coords;   ///< cigar coords
    nvbio::VectorArrayView<uint8>                   mds;            ///< mds arena

    uint8*                                          dp_buffer;      ///< DP buffer
    uint64                                          dp_buffer_size; ///< DP buffer size
};

///@}  // group PipelineStates
///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
