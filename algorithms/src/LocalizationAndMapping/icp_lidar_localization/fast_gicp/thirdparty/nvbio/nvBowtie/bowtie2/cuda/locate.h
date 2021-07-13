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
///\file locate.h
///

#pragma once

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvbio/io/alignments.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

template <typename ScoringScheme> struct BaseScoringPipelineState;
template <typename ScoringScheme> struct BestApproxScoringPipelineState;

///@addtogroup nvBowtie
///@{

/// \defgroup Locate
///
/// The functions in this module implement a pipeline stage in which all the previously selected hits (\ref Select)
/// locations are converted from Suffix Array coordinates to linear genome coordinates.
/// In terms of inputs and outputs, this stage takes the HitQueues::seed and HitQueues::loc fields, and rewrite
/// the HitQueues::loc field with the linear value.
///
/// \b inputs:
///  - HitQueues::seed
///  - HitQueues::loc
///
/// \b outputs:
///  - HitQueues::loc
///

///@addtogroup Locate
///@{

///
/// Locate the SA row of the the hits in the HitQueues.
/// Since the input might have been sorted to gather locality, the entries
/// in the HitQueues are now specified by an index (idx_queue).
/// This function reads HitQueues::seed and HitQueues::loc fields and rewrites
/// the HitQueues::loc field.
///
template <typename BatchType, typename FMType, typename rFMType>
void locate(
    const BatchType             read_batch, const FMType fmi, const rFMType rfmi,
    const uint32                in_count,
    const uint32*               idx_queue,
          HitQueuesDeviceView   hits,
    const ParamsPOD             params);

///
/// Locate the SA row of the the hits in the HitQueues.
/// Since the input might have been sorted to gather locality, the entries
/// in the HitQueues are now specified by an index (idx_queue).
/// This function reads HitQueues::seed and HitQueues::loc fields and writes
/// the HitQueues::loc and HitQueues::ssa fields with temporary values that
/// can be later consumed by locate_lookup.
///
template <typename BatchType, typename FMType, typename rFMType>
void locate_init(
    const BatchType             read_batch, const FMType fmi, const rFMType rfmi,
    const uint32                in_count,
    const uint32*               idx_queue,
          HitQueuesDeviceView   hits,
    const ParamsPOD             params);

///
/// Locate the SA row of the the hits in the HitQueues.
/// Since the input might have been sorted to gather locality, the entries
/// in the HitQueues are now specified by an index (idx_queue).
/// This function reads HitQueues::seed, HitQueues::loc and HitQueues::ssa fields
/// (which must have been previously produced by a call to locate_init) and rewrites
/// the HitQueues::loc field with the final linear coordinate value.
///
template <typename BatchType, typename FMType, typename rFMType>
void locate_lookup(
    const BatchType             read_batch, const FMType fmi, const rFMType rfmi,
    const uint32                in_count,
    const uint32*               idx_queue,
          HitQueuesDeviceView   hits,
    const ParamsPOD             params);

///
/// Locate the SA row of the the hits in the HitQueues.
/// This function reads HitQueues::seed and HitQueues::loc fields and writes
/// the HitQueues::loc and HitQueues::ssa fields with temporary values that
/// can be later consumed by locate_lookup.
///
template <typename ScoringScheme>
void locate_init(
    const BaseScoringPipelineState<ScoringScheme>&  pipeline,
    const ParamsPOD                                 params);

///
/// Locate the SA row of the the hits in the HitQueues.
/// This function reads HitQueues::seed, HitQueues::loc and HitQueues::ssa fields
/// (which must have been previously produced by a call to locate_init) and rewrites
/// the HitQueues::loc field with the final linear coordinate value.
///
template <typename ScoringScheme>
void locate_lookup(
    const BaseScoringPipelineState<ScoringScheme>&  pipeline,
    const ParamsPOD                                 params);

///
/// mark seeds straddling the reference boundaries.
///
template <typename index_iterator, typename flags_iterator>
void mark_straddling(
    const uint32                in_count,
    const uint32*               idx_queue,
    const uint32                reference_count,
    const index_iterator        reference_index,
          HitQueuesDeviceView   hits,
          flags_iterator        flags,
    const ParamsPOD             params);

///@}  // group Locate
///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio

#include <nvBowtie/bowtie2/cuda/locate_inl.h>
