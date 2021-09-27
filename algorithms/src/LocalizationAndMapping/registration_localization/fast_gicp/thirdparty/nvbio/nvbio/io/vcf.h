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

#include <nvbio/basic/types.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/packed_vector.h>

#include <vector>
#include <string>

#pragma once

namespace nvbio {
namespace io {

struct SNP_sequence_index
{
    // these indices are stored in base-pairs since variants are extremely short
    uint32 reference_start;
    uint32 reference_len;
    uint32 variant_start;
    uint32 variant_len;

    SNP_sequence_index()
        : reference_start(0), reference_len(0),
          variant_start(0), variant_len(0)
    { }

    SNP_sequence_index(uint32 reference_start, uint32 reference_len,
                       uint32 variant_start, uint32 variant_len)
        : reference_start(reference_start), reference_len(reference_len),
          variant_start(variant_start), variant_len(variant_len)
    { }
};

struct SNPDatabase
{
    // the name of the reference sequence
    // note: VCF allows this to be an integer ID encoded in a string that references
    // a contig from an assembly referenced in the header; this is not supported yet
    std::vector<std::string> reference_sequence_names;

    // start (x) and stop (y) positions of the variant in the reference sequence (first base in the sequence is position 1)
    // the "stop" position is either start + len or the contents of the END= info tag
    nvbio::vector<host_tag, uint2> sequence_positions;

    // packed reference sequences
    nvbio::PackedVector<host_tag, 4> reference_sequences;
    // packed variant sequences
    nvbio::PackedVector<host_tag, 4> variants;
    // an index for both references and variants
    nvbio::vector<host_tag, SNP_sequence_index> ref_variant_index;

    // quality value assigned to each variant
    nvbio::vector<host_tag, uint8> variant_qualities;

    SNPDatabase()
    {
        reference_sequences.clear();
        variants.clear();
        ref_variant_index.clear();
    }
};

// loads variant data from file_name and appends to output
bool loadVCF(SNPDatabase& output, const char *file_name);

} // namespace io
} // namespace nvbio
