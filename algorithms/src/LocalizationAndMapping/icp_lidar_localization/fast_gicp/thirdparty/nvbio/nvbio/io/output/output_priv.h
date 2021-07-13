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
#include <nvbio/io/output/output_batch.h>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include <nvbio/basic/vector_array.h>
#include <nvbio/io/sequence/sequence.h>

#include <stdio.h>

namespace nvbio {
namespace io {

// Utility struct to gather all data related to a given alignment.
// This breaks out the alignment data for a given alignment by setting up
// pointers to the relevant bits of data pulled from the GPU buffers.
struct AlignmentData
{
    typedef io::SequenceDataHost                            read_data_type;
    typedef io::SequenceDataAccess<DNA_N>                   read_access_type;
    typedef read_access_type::sequence_stream_type          read_type;

    bool             valid;             ///< set to true if this is a valid alignment
    const Alignment* aln;               ///< the alignment itself
    uint32           aln_id;            ///< the alignment index
    uint32           read_id;           ///< the read id of this alignment within the batch
    uint32           mapq;              ///< the mapping quality score

    // Pointers to the read_data, cigar and mds arrays for this read.
    // These are not really meant to be used outside AlignmentData and
    // should probably be removed

    const io::SequenceDataHost          *read_data_batch_p;
    const HostCigarArray                *cigar_array_p;
    const HostMdsArray                  *mds_array_p;

    // the remaining fields are derived from best, read_data_batch,
    // cigar_array and mds_array in the ctor
    // they are commonly used when writing alignment data out to disk

    uint32 read_offset;                 ///< the offset of the read from the start of read_data
    uint32 read_len;                    ///< length of the read
    const char *read_name;              ///< read name

    read_type read_data;                ///< the iterator for the read data, acts as an array of uint8
    const char *qual;                   ///< quality data

    const Cigar *cigar;                 ///< CIGAR for this alignment
    uint32 cigar_pos;                   ///< the position of the cigar in the cigar array for this batch
                                        ///< (should really not be here, only used to get the BNT)
    uint32 cigar_len;                   ///< CIGAR length
    const uint8 *mds_vec;               ///< MDS vector

    AlignmentData()
        : valid(false),
          aln(NULL),
          aln_id(0xffffffff),
          read_id(0xffffffff),
          read_data_batch_p(NULL),
          cigar_array_p(NULL),
          mds_array_p(NULL),
          read_offset(0xffffffff),
          read_len(0xffffffff),
          read_name(NULL),
          qual(NULL),
          cigar(NULL),
          cigar_pos(0xffffffff),
          cigar_len(0xffffffff)
    {}

    AlignmentData(const Alignment*              _aln,
                  const uint32                  _mapq,
                  const uint32                  _aln_id,
                  const uint32                  _read_id,
                  const io::SequenceDataHost*   read_data_batch,
                  const HostCigarArray*         cigar_array,
                  const HostMdsArray*           mds_array)
        : valid(true),
          aln(_aln),
          aln_id(_aln_id),
          read_id(_read_id),
          mapq(_mapq),
          read_data_batch_p(read_data_batch),
          cigar_array_p(cigar_array),
          mds_array_p(mds_array)
    {
        uint2 cigar_coord;

        read_access_type read_data_access( *read_data_batch );

        read_offset = read_data_access.sequence_index()[read_id];
        read_len    = read_data_access.sequence_index()[read_id + 1] - read_offset;
        read_name   = read_data_access.name_stream() + read_data_access.name_index()[read_id];

        read_data   = read_data_access.sequence_stream() + read_offset;
        qual        = read_data_access.qual_stream() + read_offset;

        cigar       = cigar_array_p->array[aln_id];
        cigar_coord = cigar_array_p->coords[aln_id];
        cigar_pos   = compute_cigar_pos(cigar_coord.x, aln->alignment());
        cigar_len   = cigar_coord.y;

        mds_vec = (*mds_array)[aln_id];
    }

    static AlignmentData invalid(void) { return AlignmentData(); }
};

// extract alignment data for a given mate
// note that the mates can be different for the cigar, since mate 1 is always the anchor mate for cigars
AlignmentData get(HostOutputBatchSE& batch, const uint32 aln_id);

// extract alignment data for a given mate
AlignmentData get_mate(HostOutputBatchPE& batch, const uint32 aln_id, const AlignmentMate mate);
// extract alignment data for the anchor mate
AlignmentData get_anchor_mate(HostOutputBatchPE& batch, const uint32 aln_id);
// extract alignment data for the opposite mate
AlignmentData get_opposite_mate(HostOutputBatchPE& batch, const uint32 aln_id);

} // namespace io
} // namespace nvbio
