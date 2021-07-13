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
///\file reduce.h
///

#pragma once

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/scoring.h>
#include <nvBowtie/bowtie2/cuda/params.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

struct ParamsPOD;

template <typename ScoringScheme> struct BaseScoringPipelineState;
template <typename ScoringScheme> struct BestApproxScoringPipelineState;

///@addtogroup nvBowtie
///@{

///@addtogroup Reduce
///@{

///
/// A context class for the score_reduce_kernel to be used in best-approx pipeline.
///
/// \details
/// Implements the basic extension bail-out mechanisms of Bowtie2, namely stopping
/// when a given number of extensions of a read failed in a row.
/// This is done keeping a vector of per-read extension 'trys' counters, which
/// start from the maximum allowed number, and get decremented towards zero upon
/// each failure, or reset upon successful extensions.
///
struct ReduceBestApproxContext
{
    /// constructor
    ///
    /// \param trys        trys vector
    /// \param n_ext       total number of extensions (i.e. extension loops) already performed
    ///
    ReduceBestApproxContext(uint32* trys, const uint32 n_ext) : m_trys( trys ), m_ext( n_ext ) {}

    /// this method is called from score_reduce_kernel to report updates to the best score.
    ///
    NVBIO_FORCEINLINE NVBIO_DEVICE 
    void best_score(const uint32 read_id, const ParamsPOD& params) const
    {
        // reset the try counter
        m_trys[ read_id ] = params.max_effort;
    }
    /// this method is called from score_reduce_kernel to report updates to the second best score.
    ///
    NVBIO_FORCEINLINE NVBIO_DEVICE 
    void second_score(const uint32 read_id, const ParamsPOD& params) const
    {
        // reset the try counter
        m_trys[ read_id ] = params.max_effort;
    }
    /// this method is called from score_reduce_kernel to report extension failures.
    ///
    NVBIO_FORCEINLINE NVBIO_DEVICE 
    bool failure(const uint32 idx, const uint32 read_id, const uint32 top_flag, const ParamsPOD& params) const
    {
        if (m_trys[ read_id ] > 0)
        {
            if (((m_ext+idx >= params.min_ext) && (top_flag == 0) && (--m_trys[ read_id ] == 0)) || // bowtie2 does 1 more alignment than effort limit, we don't
                 (m_ext+idx >= params.max_ext))
                 return true;
        }
        return false;
    }

private:
    uint32* m_trys;
    uint32  m_ext;
};

///
/// A context class for the score_reduce_kernel to be used in best-exact pipeline.
///
/// \details
/// A trivial implementation, that never bails out.
///
struct ReduceBestExactContext
{
    ReduceBestExactContext() {}

    /// this method is called from score_reduce_kernel to report updates to the best score.
    ///
    NVBIO_FORCEINLINE NVBIO_DEVICE 
    void best_score(const uint32 read_id, const ParamsPOD& params) const {}

    /// this method is called from score_reduce_kernel to report updates to the second best score.
    ///
    NVBIO_FORCEINLINE NVBIO_DEVICE 
    void second_score(const uint32 read_id, const ParamsPOD& params) const {}

    /// this method is called from score_reduce_kernel to report extension failures.
    ///
    NVBIO_FORCEINLINE NVBIO_DEVICE 
    bool failure(const uint32 idx, const uint32 read_id, const uint32 top_flag, const ParamsPOD& params) const { return false; }
};

///
/// Reduce the scores associated to each read in the scoring queue to find the best 2 alignments.
///
void score_reduce(
    const ReduceBestApproxContext                                       context,
    const BestApproxScoringPipelineState<EditDistanceScoringScheme>&    pipeline,
    const ParamsPOD&                                                    params);

///
/// Reduce the scores associated to each read in the scoring queue to find the best 2 alignments.
///
void score_reduce(
    const ReduceBestApproxContext                                           context,
    const BestApproxScoringPipelineState<SmithWatermanScoringScheme<> >&    pipeline,
    const ParamsPOD&                                                        params);

///
/// Reduce the scores associated to each paired-end read in the scoring queue to find the best 2 alignments.
///
void score_reduce_paired(
    const ReduceBestApproxContext                                       context,
    const BestApproxScoringPipelineState<EditDistanceScoringScheme>&    pipeline,
    const ParamsPOD&                                                    params);

///
/// Reduce the scores associated to each paired-end read in the scoring queue to find the best 2 alignments.
///
void score_reduce_paired(
    const ReduceBestApproxContext                                           context,
    const BestApproxScoringPipelineState<SmithWatermanScoringScheme<> >&    pipeline,
    const ParamsPOD&                                                        params);

///@}  // group Reduce
///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio

#include <nvBowtie/bowtie2/cuda/reduce_inl.h>
