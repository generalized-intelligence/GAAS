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
///\file mapping.h
///

#pragma once

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/utils.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvBowtie/bowtie2/cuda/seed_hit.h>
#include <nvBowtie/bowtie2/cuda/seed_hit_deque_array.h>
#include <nvBowtie/bowtie2/cuda/reads_def.h>
#include <nvBowtie/bowtie2/cuda/fmindex_def.h>
#include <nvbio/io/utils.h>
#include <nvbio/basic/cuda/pingpong_queues.h>
#include <nvbio/basic/cached_iterator.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/priority_deque.h>
#include <nvbio/basic/strided_iterator.h>
#include <nvbio/basic/algorithms.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

///@addtogroup nvBowtie
///@{

/// \defgroup Mapping
///
/// The functions in this module implement the very first pipeline stage: seed mapping.
/// In this stage each read is broken up into many short, possibly overlapping seeds
/// which get mapped against the reference genome using an FM-index.
/// The output is a vector of variable-lenth "priority deques", one for each read,
/// containing the set of Suffix Array ranges (\ref SeedHits) where the seeds align,
/// prioritized by the inverse of the range size (see \ref SeedHitDequeArray).
///
/// The module implements many mapping algorithms:
///
/// - exact: allowing exact matches only
/// - approx-hybrid: allowing 0 mismatches in a subseed of the seed, and up to 1 mismatch in the rest
/// - approx-case-pruning: allowing 1 mismatch across the entire seed, using 2 FM-indices to perform
///   the search with case pruning (i.e. searching an alignment with 0 mismatches in the first half of
///   the seed in the forward FM-index, and an alignment with 0 mismatches in the second half in the
///   reverse FM-index).
///

///@addtogroup Mapping
///@{

///
/// For all the seed hit ranges, output the range size in out_ranges.
///
void gather_ranges(
    const uint32                    count,
    const uint32                    n_reads,
    SeedHitDequeArrayDeviceView     hits,
    const uint32*                   hit_counts_scan,
          uint64*                   out_ranges);

///
/// perform exact read mapping
///
void map_whole_read(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc);

///
/// perform one run of exact seed mapping for all the reads in the input queue,
/// writing reads that need another run in the output queue
///
void map_exact(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc);

///
/// perform multiple runs of exact seed mapping in one go and keep the best
///
void map_exact(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    SeedHitDequeArrayDeviceView                     hits,
    const uint2                                     seed_range,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc);

///
/// perform one run of approximate seed mapping for all the reads in the input queue,
/// writing reads that need another run in the output queue
///
void map_approx(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc);

///
/// perform multiple runs of approximate seed mapping in one go and keep the best
///
void map_approx(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    SeedHitDequeArrayDeviceView                     hits,
    const uint2                                     seed_range,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc);

///
/// perform one run of seed mapping
///
void map(
    const ReadsDef::type&                           read_batch,
    const FMIndexDef::type                          fmi,
    const FMIndexDef::type                          rfmi,
    const uint32                                    retry,
    const nvbio::cuda::PingPongQueuesView<uint32>   queues,
    uint8*                                          reseed,
    SeedHitDequeArrayDeviceView                     hits,
    const ParamsPOD                                 params,
    const bool                                      fw,
    const bool                                      rc);

///@}  // group Mapping
///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
