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

#include <nvbio/io/output/output_debug.h>
#include <nvbio/basic/numbers.h>
#include <crc/crc.h>

namespace nvbio {
namespace io {

DebugOutput::DebugOutput(const char *file_name, AlignmentType alignment_type, BNT bnt)
    : OutputFile(file_name, alignment_type, bnt)
{
    if (alignment_type == PAIRED_END)
    {
        char fname[256];
        char *mate_number;

        NVBIO_CUDA_ASSERT(strlen(file_name) < sizeof(fname));
        strncpy(fname, file_name, sizeof(fname));

        mate_number = strstr(fname, "#");
        if (mate_number)
        {
            *mate_number = '1';
        }

        fp = gzopen(fname, "wb");

        if (mate_number)
        {
            *mate_number = '2';
            fp_opposite_mate = gzopen(fname, "wb");
        } else {
            fp_opposite_mate = fp;
        }
    } else {
        fp = gzopen(file_name, "wb");
        fp_opposite_mate = NULL;
    }
}

DebugOutput::~DebugOutput()
{
    if (fp)
    {
        gzclose(fp);
    }

    if (fp_opposite_mate && fp != fp_opposite_mate)
    {
        gzclose(fp_opposite_mate);
    }

    fp = NULL;
    fp_opposite_mate = NULL;
}


void DebugOutput::process(struct HostOutputBatchSE& batch)
{
    for(uint32 c = 0; c < batch.count; c++)
    {
        AlignmentData mate_1 = get(batch, c);
        AlignmentData mate_2 = AlignmentData::invalid();

        process_one_alignment(mate_1, mate_2);
    }
}

void DebugOutput::process(struct HostOutputBatchPE& batch)
{
    for(uint32 c = 0; c < batch.count; c++)
    {
        AlignmentData mate_1 = get_mate(batch, c, MATE_1);
        AlignmentData mate_2 = get_mate(batch, c, MATE_2);

        process_one_alignment(mate_1, mate_2);
    }
}

void DebugOutput::close(void)
{
    if (fp)
    {
        gzclose(fp);
    }

    if (fp_opposite_mate && fp != fp_opposite_mate)
    {
        gzclose(fp_opposite_mate);
    }

    fp = NULL;
    fp_opposite_mate = NULL;
}

void DebugOutput::output_alignment(gzFile& fp, const struct DbgAlignment& al, const struct DbgInfo& info)
{
    gzwrite(fp, &al, sizeof(al));
    if (al.alignment_pos)
    {
        gzwrite(fp, &info, sizeof(info));
    }
}

void DebugOutput::process_one_alignment(const AlignmentData& mate_1, const AlignmentData& mate_2)
{
    DbgAlignment al;
    DbgInfo info;

    const uint32 mapq = mate_1.mapq;

    // process the first mate
    process_one_mate(al, info, mate_1, mate_2, mapq);
    output_alignment(fp, al, info);

    if (alignment_type == PAIRED_END)
    {
        // process the second mate
        process_one_mate(al, info, mate_2, mate_1, mapq);
        output_alignment(fp_opposite_mate, al, info);
    }
}

// fill out al and info for a given mate
// this does *not* fill in mapq, as that requires knowledge of which mate is the anchor as well as second-best scores
void DebugOutput::process_one_mate(DbgAlignment& al,
                                   DbgInfo& info,
                                   const AlignmentData& alignment,
                                   const AlignmentData& mate,
                                   const uint32 mapq)
{
    // output read_id as CRC of read name
    al.read_id = crcCalc(alignment.read_name, strlen(alignment.read_name));
    al.read_len = alignment.read_len;

    if (alignment.aln->is_aligned())
    {
        // setup alignment information
        const uint32 seq_index = uint32(std::upper_bound(
            bnt.sequence_index,
            bnt.sequence_index + bnt.n_seqs,
            alignment.cigar_pos ) - bnt.sequence_index) - 1u;

        al.alignment_pos = alignment.cigar_pos - int32(bnt.sequence_index[ seq_index ]) + 1u;
        info.flag = (alignment.aln->mate() ? DbgInfo::READ_2 : DbgInfo::READ_1) |
                    (alignment.aln->is_rc() ? DbgInfo::REVERSE : 0u);

        if (alignment_type == PAIRED_END)
        {
            if (alignment.aln->is_paired()) // FIXME: this should be other_mate.is_concordant()
                info.flag |= DbgInfo::PROPER_PAIR;

            if (mate.aln->is_aligned() == false)
                info.flag |= DbgInfo::MATE_UNMAPPED;
        }

        const uint32 ref_cigar_len = reference_cigar_length(alignment.cigar, alignment.cigar_len);
        if (alignment.cigar_pos + ref_cigar_len > bnt.sequence_index[ seq_index+1 ])
        {
            // flag UNMAPPED as this alignment bridges two adjacent reference sequences
            info.flag |= DbgInfo::UNMAPPED;
        }

        uint32 n_mm;
        uint32 n_gapo;
        uint32 n_gape;

        analyze_md_string(alignment.mds_vec, n_mm, n_gapo, n_gape);

        info.ref_id = seq_index;
        info.mate   = alignment.aln->mate();
        info.score  = alignment.aln->score();
        info.mapQ   = mapq;
        info.ed     = alignment.aln->ed();
        info.subs   = count_symbols(Cigar::SUBSTITUTION, alignment.cigar, alignment.cigar_len);
        info.ins    = count_symbols(Cigar::INSERTION, alignment.cigar, alignment.cigar_len);
        info.dels   = count_symbols(Cigar::DELETION, alignment.cigar, alignment.cigar_len);
        info.mms    = n_mm;
        info.gapo   = n_gapo;
        info.gape   = n_gape;
        info.sec_score = Field_traits<int16>::min(); // TODO!
        info.has_second = 0;
        /*
        if (info.sec_score == alignment.second_best->is_aligned())
        {
            info.sec_score = alignment.second_best->score();
            info.has_second = 1;
        } else {
            info.sec_score = Field_traits<int16>::min();
            info.has_second = 0;
        }
        */

        info.pad = 0x69;
    } else {
        // unmapped alignment
        al.alignment_pos = 0;
    }
}

} // namespace io
} // namespace nvbio
