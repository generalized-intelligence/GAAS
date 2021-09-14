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
#include <nvbio/io/alignments.h>
#include <nvbio/basic/cuda/pingpong_queues.h>
#include <nvbio/basic/priority_deque.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/strided_iterator.h>
#include <nvbio/basic/sum_tree.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

template <typename ScoringScheme> struct BaseScoringPipelineState;
template <typename ScoringScheme> struct BestApproxScoringPipelineState;

///@addtogroup nvBowtie
///@{

///@addtogroup Select
///@{

///
/// Initialize the hit-selection pipeline
///
template <typename ScoringScheme>
void select_init_t(BestApproxScoringPipelineState<ScoringScheme>& pipeline, const ParamsPOD& params);

///
/// Prepare for a round of seed extension by selecting the next SA row from each
/// of the seed-hit deque arrays (SeedHitDequeArray) bound to the active-reads in
/// the scoring queues (ScoringQueues::active_reads).
///
template <typename BatchType, typename ContextType>
void select(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const ParamsPOD                         params);

///
/// Prepare for a round of seed extension by selecting the next SA row from each
/// of the seed-hit deque arrays (SeedHitDequeArray) bound to the active-reads in
/// the scoring queues (ScoringQueues::active_reads).
///
template <typename BatchType, typename ContextType>
void rand_select(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    uint32*                                 rseeds,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const ParamsPOD                         params);

///
/// Prepare for a round of seed extension by selecting a set of up
/// to 'n_multi' SA rows from each of the seed-hit deque arrays (SeedHitDequeArray)
/// bound to the active-reads in the scoring queues (ScoringQueues::active_reads).
///
/// For each read in the input queue, this kernel generates:
///     1. one or zero output reads, in the main output read queue,
///     2. zero to 'n_multi' SA rows. These are made of three entries,
///        one in 'loc_queue', identifying the corresponding SA index,
///        one in 'seed_queue', storing information about the seed hit,
///        and one in 'parent_queue', storing the index of the "parent"
///        read in the output queue (i.e. the slot where the read is
///        is being stored)
///
template <typename BatchType, typename ContextType>
void select_multi(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const uint32                            n_multi,
    const ParamsPOD                         params);

///
/// Prepare for a round of seed extension by selecting the next SA row from each
/// of the seed-hit deque arrays (SeedHitDequeArray) bound to the active-reads in
/// the scoring queues (ScoringQueues::active_reads).
///
template <typename BatchType, typename ContextType>
void select(
    const BatchType                         read_batch,
    SeedHitDequeArrayDeviceView             hits,
    uint32*                                 rseeds,
    const ContextType                       context,
          ScoringQueuesDeviceView           scoring_queues,
    const uint32                            n_multi,
    const ParamsPOD                         params);

///
/// Prepare for a round of seed extension by selecting the next SA row from each
/// of the seed-hit deque arrays (SeedHitDequeArray) bound to the active-reads in
/// the scoring queues (ScoringQueues::active_reads).
///
template <typename ScoringScheme, typename ContextType>
void select_t(
    const ContextType                                       context,
    const BestApproxScoringPipelineState<ScoringScheme>&    pipeline,
    const ParamsPOD                                         params);

///@}  // group Select
///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio

#include <nvBowtie/bowtie2/cuda/select_inl.h>
