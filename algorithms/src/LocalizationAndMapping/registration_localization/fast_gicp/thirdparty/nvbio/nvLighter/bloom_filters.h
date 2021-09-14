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

// bloom_filters.h
//

#pragma once

#include "utils.h"
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/threads.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/basic/popcount.h>

///@defgroup nvLighterModule nvLighter
/// This module contains all of \subpage nvlighter_page "nvLighter"'s class hierarchy.
/// nvLighter is composed of the following important pieces:
///\par
/// - the BloomFilters class, containing all the per-device data needed during nvLighter's execution,
///   i.e. the <i>sampled</i> and <i>trusted kmers</i> Bloom filters and the error correction statistics
/// - the InputStage, i.e. an object encapsulating an nvbio::Pipeline stage loading read batches
/// - the OutputStage, i.e. an object encapsulating an nvbio::Pipeline stage writing read batches
/// - the SampleKmersStage, i.e. an object encapsulating an nvbio::Pipeline stage performing kmer sampling
/// - the TrustedKmersStage, i.e. an object encapsulating an nvbio::Pipeline stage performing trusted kmer marking
/// - the ErrorCorrectStage, i.e. an object encapsulating an nvbio::Pipeline stage performing the actual error correction
///

///@addtogroup nvLighterModule
///@{

/// The kmer Bloom filter types
///
enum KmersType { SAMPLED_KMERS = 0, TRUSTED_KMERS = 1 };

/// Bloom filters container - this class contains all the per-device data
/// needed during nvLighter's execution
///
template <typename system_tag>
struct BloomFilters
{
    /// setup the internal storage
    ///
    bool setup(const int _device, const uint64 sampled_words, const uint64 trusted_words);

    const nvbio::vector<system_tag,uint32>& get_kmers(const KmersType type) const;
          nvbio::vector<system_tag,uint32>& get_kmers(const KmersType type);

    void get_kmers(const KmersType type, nvbio::vector<nvbio::host_tag,uint32>& bf);
    void set_kmers(const KmersType type, const nvbio::vector<nvbio::host_tag,uint32>& bf);
    void set_threshold(const nvbio::vector<nvbio::host_tag,uint32>& _threshold);

    void set_device() const;
    void device_memory(size_t* free_device, size_t* total_device) const;

public:
    int                              device;
    nvbio::vector<system_tag,uint32> sampled_kmers_storage;
    nvbio::vector<system_tag,uint32> trusted_kmers_storage;
    nvbio::vector<system_tag,uint32> threshold;
    nvbio::vector<system_tag,uint64> stats;
};

/// merge several Bloom filters
///
inline
void merge(
    BloomFilters<nvbio::host_tag>*      h_bloom_filters,
    const uint32                        device_count,
    BloomFilters<nvbio::device_tag>*    d_bloom_filters,
    const KmersType                     type);

/// merge several stats
///
inline
void merged_stats(
    const BloomFilters<nvbio::host_tag>*    h_bloom_filters,
    const uint32                            device_count,
    const BloomFilters<nvbio::device_tag>*  d_bloom_filters,
    nvbio::vector<nvbio::host_tag,uint64>&  stats);

/// compute Bloom filter usage statistics
///
template <typename system_tag>
void compute_bloom_filter_stats(
    const BloomFilters<system_tag>& bloom_filters,
    const KmersType                 type,
    const uint32                    K,
    float&                          occupancy,
    float&                          approx_size,
    float&                          fp);

///@}  // group nvLighterModule

#include "bloom_filters_inl.h"
