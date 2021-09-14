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

#include <nvbio/fmindex/fmindex.h>

namespace nvbio {

///@addtogroup FMIndex
///@{

/// forward extension using a bidirectional FM-index, extending the range
/// of a pattern P to the pattern Pc.
///\par
/// <b>Note:</b> extension can be performed without a sampled suffix array, so that
/// there's no need to store two of them; in practice, the FM-indices can
/// also be of type fm_index <RankDictionary,null_type>.
///
/// \param f_fmi    forward FM-index
/// \param r_fmi    reverse FM-index
/// \param f_range  current forward range
/// \param r_range  current reverse range
/// \param c        query character
///
template <
    typename TRankDictionary1,
    typename TSuffixArray1,
    typename TRankDictionary2,
    typename TSuffixArray2>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void extend_forward(
    const fm_index<TRankDictionary1,TSuffixArray1>&                 f_fmi,
    const fm_index<TRankDictionary2,TSuffixArray2>&                 r_fmi,
    typename fm_index<TRankDictionary1,TSuffixArray1>::range_type&  f_range,
    typename fm_index<TRankDictionary2,TSuffixArray2>::range_type&  r_range,
    uint8                                                           c);

/// backwards extension using a bidirectional FM-index, extending the range
/// of a pattern P to the pattern cP
///\par
/// <b>Note:</b> extension can be performed without a sampled suffix array, so that
/// there's no need to store two of them; in practice, the FM-indices can
/// also be of type fm_index <RankDictionary,null_type>.
///
/// \param f_fmi    forward FM-index
/// \param r_fmi    reverse FM-index
/// \param f_range  current forward range
/// \param r_range  current reverse range
/// \param c        query character
///
template <
    typename TRankDictionary1,
    typename TSuffixArray1,
    typename TRankDictionary2,
    typename TSuffixArray2>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void extend_backwards(
    const fm_index<TRankDictionary1,TSuffixArray1>&                 f_fmi,
    const fm_index<TRankDictionary2,TSuffixArray2>&                 r_fmi,
    typename fm_index<TRankDictionary1,TSuffixArray1>::range_type&  f_range,
    typename fm_index<TRankDictionary2,TSuffixArray2>::range_type&  r_range,
    uint8                                                           c);

///@} // end of the FMIndex group

} // namespace nvbio

#include <nvbio/fmindex/bidir_inl.h>
