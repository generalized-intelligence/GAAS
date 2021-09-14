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

#include <nvbio/io/output/output_types.h>
#include <nvbio/io/output/output_stats.h>
#include <nvbio/io/output/output_file.h>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include <nvbio/basic/vector_array.h>
#include <nvbio/io/sequence/sequence.h>

#include <stdio.h>

namespace nvbio {
namespace io {

/// base class for representing a batch of alignment results on the device
///
struct DeviceOutputBatchSE
{
public:
    uint32                                     count;

    thrust::device_vector<io::Alignment>&      alignments;
    DeviceCigarArray                           cigar;
    nvbio::DeviceVectorArray<uint8>&           mds;
    thrust::device_vector<uint8>&              mapq;
    thrust::device_vector<uint32>*             read_ids;

    DeviceOutputBatchSE(uint32                                    _count,
                   thrust::device_vector<io::Alignment>&          _alignments,
                   DeviceCigarArray                               _cigar,
                   nvbio::DeviceVectorArray<uint8>&               _mds,
                   thrust::device_vector<uint8>&                  _mapq,
                   thrust::device_vector<uint32>*                 _read_ids = NULL)
        : count(_count),
          alignments(_alignments),
          cigar(_cigar),
          mds(_mds),
          mapq(_mapq),
          read_ids(_read_ids)
    {}

    // copy best score data into host memory
    void readback_scores(thrust::host_vector<io::Alignment>& host_alignments) const;
    // copy cigars into host memory
    void readback_cigars(HostCigarArray& host_cigars) const;
    // copy md strings into host memory
    void readback_mds(nvbio::HostVectorArray<uint8>& host_mds) const;
    // copy mapq into host memory
    void readback_mapq(thrust::host_vector<uint8>& host_mapq) const;
    // copy ids into host memory
    void readback_ids(thrust::host_vector<uint32>& host_ids) const;
};

/// a batch of alignment results on the CPU
///
struct HostOutputBatchSE
{
public:
    uint32 count;

    // we have two alignments, cigar and MDS arrays, one for each mate
    thrust::host_vector<io::Alignment>           alignments;
    HostCigarArray                               cigar;
    HostMdsArray                                 mds;
    thrust::host_vector<uint8>                   mapq;
    thrust::host_vector<uint32>                  read_ids;

    // pointer to the host-side read data for each mate
    const io::SequenceDataHost*                  read_data;

    void readback(const DeviceOutputBatchSE);

public:
    /// constructor
    ///
    HostOutputBatchSE() : count(0) {}
};

/// a batch of alignment results on the CPU
///
struct HostOutputBatchPE
{
public:
    uint32 count;

    // we have two alignments, cigar and MDS arrays, one for each mate
    thrust::host_vector<io::Alignment>           alignments[2];
    HostCigarArray                               cigar[2];
    HostMdsArray                                 mds[2];
    thrust::host_vector<uint8>                   mapq[2];
    thrust::host_vector<uint32>                  read_ids;

    // pointer to the host-side read data for each mate
    const io::SequenceDataHost*                  read_data[2];

    void readback(const DeviceOutputBatchSE, const AlignmentMate mate);

public:
    /// constructor
    ///
    HostOutputBatchPE() : count(0) {}
};

} // namespace io
} // namespace nvbio
