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

// sample_kmers.h
//

#pragma once

#include "utils.h"
#include <nvbio/basic/pipeline_context.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/io/sequence/sequence.h>
#include <stdio.h>
#include <stdlib.h>

///@addtogroup nvLighterModule
///@{

///
/// A small class implementing an nvbio::Pipeline sink stage building the sampled kmers filter
///
struct SampleKmersStage
{
    typedef nvbio::io::SequenceDataHost   argument_type;

    /// empty constructor
    ///
    SampleKmersStage() {}

    /// constructor
    ///
    ///\param _k                kmer length
    ///\param _alpha            the sampling frequency
    ///\param _filter_size      the kmer Bloom filter's size, in bits
    ///\param _filter_storage   the kmer Bloom filter's storage
    ///
    SampleKmersStage(
        const int       _device,
        const uint32    _k,
        const float     _alpha,
        const uint64    _filter_size,
        uint32*         _filter_storage,
        SequenceStats*  _stats) :
        device( _device ),
        k( _k ),
        alpha( _alpha ),
        filter_size( _filter_size ),
        filter_storage( _filter_storage ),
        stats( _stats )
    {}

    /// process the next batch
    ///
    bool process(nvbio::PipelineContext& context);

    int                 device;
    uint32              k;
    float               alpha;
    uint64              filter_size;
    uint32*             filter_storage;
    SequenceStats*      stats;

    nvbio::for_each_enactor<nvbio::host_tag>   host_for_each;
    nvbio::for_each_enactor<nvbio::device_tag> device_for_each;
};


///
/// A small class implementing an nvbio::Pipeline sink stage building the trusted kmers filter
///
struct TrustedKmersStage
{
    typedef nvbio::io::SequenceDataHost   argument_type;

    /// default constructor
    ///
    TrustedKmersStage() {}

    /// constructor
    ///
    ///\param _k                kmer length
    ///\param _alpha            the sampling frequency
    ///\param _filter_size      the kmer Bloom filter's size, in bits
    ///\param _filter_storage   the kmer Bloom filter's storage
    ///
    TrustedKmersStage(
        const int       _device, 
        const uint32    _k,
        const uint64    _sampled_filter_size,
        const uint32*   _sampled_filter_storage,
        const uint64    _trusted_filter_size,
              uint32*   _trusted_filter_storage,
        const uint32*   _threshold,
        SequenceStats*  _stats) :
        device( _device ),
        k( _k ),
        sampled_filter_size( _sampled_filter_size ),
        sampled_filter_storage( _sampled_filter_storage ),
        trusted_filter_size( _trusted_filter_size ),
        trusted_filter_storage( _trusted_filter_storage ),
        threshold(_threshold),
        stats( _stats )
    {}

    /// process the next batch
    ///
    bool process(nvbio::PipelineContext& context);

    int                 device;
    uint32              k;
    uint64              sampled_filter_size;
    const uint32*       sampled_filter_storage;
    uint64              trusted_filter_size;
    uint32*             trusted_filter_storage;
    const uint32*       threshold;
    SequenceStats*      stats;

    nvbio::for_each_enactor<nvbio::host_tag>   host_for_each;
    nvbio::for_each_enactor<nvbio::device_tag> device_for_each;
};

///@}  // group nvLighterModule
