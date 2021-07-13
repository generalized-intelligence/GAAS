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

///
///\file select.h
///

#pragma once

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/seed_hit.h>
#include <nvBowtie/bowtie2/cuda/seed_hit_deque_array.h>
#include <nvBowtie/bowtie2/cuda/scoring_queues.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvbio/basic/cuda/pingpong_queues.h>
#include <nvbio/io/alignments.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

template <typename ScoringScheme> struct BaseScoringPipelineState;
template <typename ScoringScheme> struct BestApproxScoringPipelineState;

///@addtogroup nvBowtie
///@{

/// \defgroup Select
///
/// The functions in this module implement a pipeline stage in which, for all the active reads in the
/// \ref ScoringQueues, a group of seed hits is selected for a new round of \ref Scoring.
/// In terms of inputs and outputs, this stage takes the set of input ScoringQueues::active_reads, and
/// fills the ScoringQueues::hits with a new set of hits, writing the HitQueues::read_id, HitQueues::seed,
/// and HitQueues::loc fields. The hits' location specified by the HitQueues::loc field is expressed in
/// Suffix Array coordinates.
/// Finally, this stage is also responsible for producing a new set of ScoringQueues::active_reads.
///
/// \b inputs:
///  - SeedHitDequeArray
///  - ScoringQueues::active_reads
///
/// \b outputs:
///  - SeedHitDequeArray
///  - ScoringQueues::active_reads
///  - HitQueues::read_id
///  - HitQueues::seed
///  - HitQueues::loc
///

///@addtogroup Select
///@{

///
/// Initialize the hit-selection pipeline
///
void select_init(
    const uint32                        count,
    const char*                         read_names,
    const uint32*                       read_names_idx,
    const SeedHitDequeArrayDeviceView   hits,
    uint32*                             trys,
    uint32*                             rseeds,
    const ParamsPOD                     params);

///
/// Initialize the hit-selection pipeline
///
void select_init(BestApproxScoringPipelineState<EditDistanceScoringScheme>& pipeline, const ParamsPOD& params);

///
/// Initialize the hit-selection pipeline
///
void select_init(BestApproxScoringPipelineState<SmithWatermanScoringScheme<> >& pipeline, const ParamsPOD& params);

///
/// a context class for the select_kernel
///
struct SelectBestApproxContext
{
    // constructor
    //
    // \param trys      the per-read vector of extension trys
    //
    SelectBestApproxContext(uint32* trys) : m_trys( trys ) {}

    // stopping function
    //
    // \return          true iff we can stop the hit selection process
    //
    NVBIO_FORCEINLINE NVBIO_DEVICE
    bool stop(const uint32 read_id) const { return m_trys[ read_id ] == 0; }

private:
    uint32* m_trys;
};

///
/// select next hit extensions from the top seed ranges
///
__global__ 
void select_n_from_top_range_kernel(
    const uint32        begin,
    const uint32        count,
    const uint32        n_reads,
    const SeedHit*      hit_data,
    const uint32*       hit_range_scan,
          uint32*       loc_queue,
          uint32*       seed_queue,
          uint32*       read_info);

///
/// Prepare for a round of seed extension by selecting the next SA row from each
/// of the seed-hit deque arrays (SeedHitDequeArray) bound to the active-reads in
/// the scoring queues (ScoringQueues::active_reads).
///
void select(
    const SelectBestApproxContext                                           context,
    const BestApproxScoringPipelineState<EditDistanceScoringScheme>&        pipeline,
    const ParamsPOD                                                         params);

///
/// Prepare for a round of seed extension by selecting the next SA row from each
/// of the seed-hit deque arrays (SeedHitDequeArray) bound to the active-reads in
/// the scoring queues (ScoringQueues::active_reads).
///
void select(
    const SelectBestApproxContext                                           context,
    const BestApproxScoringPipelineState<SmithWatermanScoringScheme<> >&    pipeline,
    const ParamsPOD                                                         params);

///
/// Select next hit extensions for all-mapping
///
void select_all(
    const uint64                        begin,
    const uint32                        count,
    const uint32                        n_reads,
    const uint32                        n_hit_ranges,
    const uint64                        n_hits,
    const SeedHitDequeArrayDeviceView   hits,
    const uint32*                       hit_count_scan,
    const uint64*                       hit_range_scan,
          HitQueuesDeviceView           scoring_queues);

///@}  // group Select
///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
