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

#include <nvbio/io/output/output_priv.h>
#include <nvbio/io/output/output_batch.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/vector.h>

#include <stdio.h>
#include <stdarg.h>

namespace nvbio {
namespace io {

// extract alignment data for a given mate
// note that the mates can be different for the cigar, since mate 1 is always the anchor mate for cigars
AlignmentData get(HostOutputBatchSE& batch, const uint32 aln_id)
{
    const uint32 read_id = batch.read_ids.size() ?
                           batch.read_ids[ aln_id ] : aln_id;

    return AlignmentData(&batch.alignments[aln_id],
                         batch.mapq[aln_id],
                         aln_id,
                         read_id,
                         batch.read_data,
                         &batch.cigar,
                         &batch.mds);
}

// extract alignment data for a given mate
// note that the mates can be different for the cigar, since mate 1 is always the anchor mate for cigars
AlignmentData get_mate(HostOutputBatchPE& batch, const uint32 aln_id, const AlignmentMate mate)
{
    const uint32 read_id = batch.read_ids.size() ?
                           batch.read_ids[ aln_id ] : aln_id;

    if (batch.alignments[0][aln_id].mate() == mate)
    {
        return AlignmentData(&batch.alignments[0][aln_id],
                             batch.mapq[0][aln_id],
                             aln_id,
                             read_id,
                             batch.read_data[ mate ],
                             &batch.cigar[0],
                             &batch.mds[0]);
    }
    else
    {
        return AlignmentData(&batch.alignments[1][aln_id],
                             batch.mapq[1][aln_id],
                             aln_id,
                             read_id,
                             batch.read_data[ mate ],
                             &batch.cigar[1],
                             &batch.mds[1]);
    }
}

// extract alignment data for the anchor mate
AlignmentData get_anchor_mate(HostOutputBatchPE& batch, const uint32 aln_id)
{
    const uint32 read_id = batch.read_ids.size() ?
                           batch.read_ids[ aln_id ] : aln_id;
    const uint32 mate    = batch.alignments[0][aln_id].mate();

    return AlignmentData(&batch.alignments[0][aln_id],
                         batch.mapq[0][aln_id],
                         aln_id,
                         read_id,
                         batch.read_data[ mate ],
                         &batch.cigar[0],
                         &batch.mds[0]);
}

// extract alignment data for the opposite mate
AlignmentData get_opposite_mate(HostOutputBatchPE& batch, const uint32 aln_id)
{
    const uint32 read_id = batch.read_ids.size() ?
                           batch.read_ids[ aln_id ] : aln_id;
    const uint32 mate    = batch.alignments[1][aln_id].mate();

    return AlignmentData(&batch.alignments[1][aln_id],
                         batch.mapq[1][aln_id],
                         aln_id,
                         read_id,
                         batch.read_data[ mate ],
                         &batch.cigar[1],
                         &batch.mds[1]);
}

} // namespace io
} // namespace nvbio
