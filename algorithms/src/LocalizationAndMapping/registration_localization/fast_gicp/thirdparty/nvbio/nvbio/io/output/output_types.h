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

#include <nvbio/basic/types.h>
#include <nvbio/io/alignments.h>

#include <nvbio/io/sequence/sequence.h>
#include <nvbio/basic/vector_array.h>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include <nvbio/io/output/output_utils.h>

namespace nvbio {
namespace io {

/**
   \page output_io_page Output
   This module implements alignment data output in various formats.

   The interface consists of a single method call that takes GPU buffers post-alignment and accumulates the results in host memory until we have enough data to write out a batch to disk. Additional method calls are used to signal the start/end of a batch of alignment data, as well as to configure the output. The main interface for this module is the OutputFile class, which exposes most of the functionality.

   The following classes are exposed as the interface to this module:

   - OutputFile
   - DeviceCigarArray
   - DeviceOutputBatchSE


   @addtogroup IO
   @{

   @addtogroup Output
   @{
*/

// this was used by mapq.h in bowtie2, but seems to no longer be used anywhere
//struct MapQInfo
//{
//    int32 best;
//    int32 second_best;
//};

/// Encapsulates all sequence data required for alignment.
/// This is a simple convenience wrapper for FMIndexData.
struct BNT
{
    uint32          n_seqs;
    const char*     names;
    const uint32*   names_index;
    const uint32*   sequence_index;

    BNT(const io::ConstSequenceDataView& reference)
        : n_seqs( reference.size() ),
          names( reference.name_stream() ),
          names_index( reference.name_index() ),
          sequence_index( reference.sequence_index() )
    {}
};

/// Helper enum to identify the type of alignment we're doing
///
typedef enum {
    SINGLE_END,
    PAIRED_END
} AlignmentType;

/// Helper enum to identify a mate in an alignment.
///
typedef enum {
    MATE_1 = 0,
    MATE_2 = 1,
} AlignmentMate;

/// Helper enum to identify a scoring pass.
///
typedef enum {
    BEST_SCORE,
    SECOND_BEST_SCORE,
} AlignmentScore;

/// Wrapper struct to keep CIGAR arrays and CIGAR coords in one place. This is the device version.
///
struct DeviceCigarArray
{
    nvbio::DeviceVectorArray<io::Cigar>&       array;
    thrust::device_vector<uint2>&              coords;

    DeviceCigarArray(nvbio::DeviceVectorArray<io::Cigar>& array,
                     thrust::device_vector<uint2>&        coords)
        : array(array),
          coords(coords)
    {}
};

/// Wrapper struct to keep CIGAR arrays and CIGAR coords in one place (host version).
///
struct HostCigarArray
{
    nvbio::HostVectorArray<io::Cigar>   array;
    thrust::host_vector<uint2>          coords;
};

/// The type for the MDS array in the host
///
typedef nvbio::HostVectorArray<uint8> HostMdsArray;

/**
   @} // Output
   @} // IO
*/

} // namespace io
} // namespace nvbio
