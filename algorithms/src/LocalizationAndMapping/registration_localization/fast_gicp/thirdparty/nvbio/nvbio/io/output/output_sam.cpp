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

#include <nvbio/io/output/output_sam.h>
#include <nvbio/basic/numbers.h>

#include <stdio.h>
#include <stdarg.h>

namespace nvbio {
namespace io {

SamOutput::SamOutput(const char *file_name, AlignmentType alignment_type, BNT bnt)
    : OutputFile(file_name, alignment_type, bnt)
{
    fp = file_name ? fopen(file_name, "wt") : stdout;

    if (fp == NULL)
    {
        log_error(stderr, "SamOutput: could not open %s for writing\n", file_name);
        return;
    }

    // set a 256kb output buffer on fp and make sure it's not line buffered
    // this makes sure small fwrites do not land on disk straight away
    // (256kb was chosen based on the default stripe size for Linux mdraid RAID-5 volumes)
    setvbuf(fp, NULL, _IOFBF, 256 * 1024);
}

SamOutput::~SamOutput()
{
    if (fp && fp != stdout)
    {
        fclose(fp);
        fp = NULL;
    }
}

void SamOutput::write_formatted_string(const char *fmt, ...)
{
    va_list args;

    fwrite("\t", 1, 1, fp);
    va_start(args, fmt);
    vfprintf(fp, fmt, args);
}

void SamOutput::write_string(const char *str, bool tab)
{
    if (tab)
        fwrite("\t", 1, 1, fp);

    fwrite(str, 1, strlen(str), fp);
}

namespace {
// utility function to convert an int to a base-10 string representation
template <typename T> int itoa(char *buf, T in)
{
    int len = 0;
    bool negative = false;

    // track the sign
    if (in < 0)
    {
        negative = true;
        in = -int(in);
    }

    // convert to base10
    do
    {
        buf[len] = "0123456789"[in % 10];
        in /= 10;
        len++;
    } while(in);

    // add sign
    if (negative)
    {
        buf[len] = '-';
        len++;
    }

    // reverse
    for(int c = 0; c < len / 2; c++)
    {
        char tmp;
        tmp = buf[c];
        buf[c] = buf[len - c - 1];
        buf[len - c - 1] = tmp;
    }

    // terminate
    buf[len] = 0;
    return len;
}
}

template<typename T>
void SamOutput::write_int(T i, bool tab)
{
    char str[32];
    int len;

    len = itoa(str, i);

    if (tab)
    {
        fwrite("\t", 1, 1, fp);
    }

    fwrite(str, len, 1, fp);
}

template<typename T>
void SamOutput::write_tag(const char *name, T value)
{
    write_string(name);
    write_string(":i:", false);
    write_int(value, false);
}

template<>
void SamOutput::write_tag(const char *name, const char *value)
{
    write_string(name);
    write_string(":Z:", false);
    write_string(value, false);
}

void SamOutput::linebreak(void)
{
    fwrite("\n", 1, 1, fp);
}

void SamOutput::output_header(void)
{
    write_string("@HD", false);
    write_string("VN:1.3");
    linebreak();

    if (!rg_id.empty())
    {
        // write the RG:ID
        write_string("@RG", false);
        write_formatted_string("ID:%s",rg_id.c_str());

        // write the other RG tags
        if (!rg_string.empty())
            write_string( rg_string.c_str(), false );

        linebreak();
    }

    write_string("@PG", false);
    write_formatted_string("ID:%s",pg_id.c_str());
    write_formatted_string("PN:%s",pg_name.c_str());
    write_formatted_string("VN:%s",pg_version.c_str());
    write_formatted_string("CL:\"%s\"",pg_args.c_str());
    linebreak();

    // output the sequence info
    for (uint32 i = 0; i < bnt.n_seqs; i++)
    {
        // sequence header
        write_string("@SQ", false);
        write_formatted_string("SN:%s", bnt.names + bnt.names_index[i]);
        write_formatted_string("LN:%d", bnt.sequence_index[i+1] - bnt.sequence_index[i]);
        linebreak();
    }
}

// generate the alignment's CIGAR string
// returns the computed length of the corresponding read based on the CIGAR operations
uint32 SamOutput::generate_cigar_string(struct SamAlignment& sam_align,
                                        const AlignmentData& alignment)
{
    char *output = sam_align.cigar;
    uint32 read_len = 0;

    for(uint32 i = 0; i < alignment.cigar_len; i++)
    {
        const Cigar& cigar_entry = alignment.cigar[alignment.cigar_len - i - 1u];
        const char   cigar_op    = "MIDS"[cigar_entry.m_type];
        int len;

        // output count
        len = itoa(output, cigar_entry.m_len);
        output += len;
        // output CIGAR op
        *output = cigar_op;
        output++;

        // check that we didn't overflow the CIGAR buffer
        assert((unsigned long)(output - sam_align.cigar) < (unsigned long) (sizeof(sam_align.cigar) - 1));

        // keep track of number of BPs in the original read
        if (cigar_op != 'D')
            read_len += cigar_entry.m_len;
    }

    // terminate the output string
    *output = '\0';
    return read_len;
}


// generate the MD string
uint32 SamOutput::generate_md_string(SamAlignment& sam_align, const AlignmentData& alignment)
{
    if (alignment.mds_vec == NULL)
    {
        log_warning(stderr, "  SAM: alignment %u from read %u has an empty MD string\n", alignment.aln_id, alignment.read_id);
        sam_align.md_string[0] = '\0';
        return 0;
    }
    const uint32 mds_len = uint32(alignment.mds_vec[0]) | (uint32(alignment.mds_vec[1]) << 8);

    char *buffer = sam_align.md_string;
    uint32 buffer_len = 0;

    uint32 i;

    sam_align.mm   = 0;
    sam_align.gapo = 0;
    sam_align.gape = 0;

    i = 2;
    do
    {
        const uint8 op = alignment.mds_vec[i++];
        switch (op)
        {
        case MDS_MATCH:
            {
                uint8 l = alignment.mds_vec[i++];

                // prolong the MDS match if it spans multiple tokens
                while (i < mds_len && alignment.mds_vec[i] == MDS_MATCH)
                    l += alignment.mds_vec[i++];

                buffer_len += itoa(buffer + buffer_len, l);
            }

            break;

        case MDS_MISMATCH:
            {
                const char c = dna_to_char(alignment.mds_vec[i++]);
                buffer[buffer_len++] = c;

                sam_align.mm++;
            }

            break;

        case MDS_INSERTION:
            {
                const uint8 l = alignment.mds_vec[i++];
                i += l;

                sam_align.gapo++;
                sam_align.gape += l - 1;
            }

            break;

        case MDS_DELETION:
            {
                const uint8 l = alignment.mds_vec[i++];

                buffer[buffer_len++] = '^';
                for(uint8 n = 0; n < l; n++)
                {
                    buffer[buffer_len++] = dna_to_char(alignment.mds_vec[i++]);
                }

                buffer[buffer_len++] = '0';

                sam_align.gapo++;
                sam_align.gape += l - 1;
            }

            break;
        }
    } while(i < mds_len);

    buffer[buffer_len] = '\0';
    return buffer_len;
}

// output a SAM alignment (but not tags)
void SamOutput::output_alignment(const struct SamAlignment& sam_align)
{
    write_string(sam_align.qname, false);
    write_int((uint32)sam_align.flags);

//    log_verbose(stderr, "%s: ed(NM)=%d score(AS)=%d second_score(XS)=%d mm(XM)=%d gapo(XO)=%d gape(XG)=%d\n",
//                sam_align.qname,
//                sam_align.ed, sam_align.score, sam_align.second_score, sam_align.mm, sam_align.gapo, sam_align.gape);
    if (sam_align.flags & SAM_FLAGS_UNMAPPED)
    {
        // output * or 0 for every other required field
        write_string("*\t0\t0\t*\t*\t0\t0");
        write_string(sam_align.seq);
        write_string(sam_align.qual);

        linebreak();

        return;
    }

    write_string(sam_align.rname);
    write_int(sam_align.pos);
    write_int(sam_align.mapq);
    write_string(sam_align.cigar);

    if (sam_align.rnext)
        write_string(sam_align.rnext);
    else
        write_string("*");

    write_int(sam_align.pnext);
    write_int(sam_align.tlen);

    write_string(sam_align.seq);
    write_string(sam_align.qual);

    write_tag("NM", sam_align.ed);
    write_tag("AS", sam_align.score);
    if (sam_align.second_score_valid)
        write_tag("XS", sam_align.second_score);

    write_tag("XM", sam_align.mm);
    write_tag("XO", sam_align.gapo);
    write_tag("XG", sam_align.gape);
    if (sam_align.md_string[0])
        write_tag("MD", sam_align.md_string);
    else
        write_tag("MD", "*");

    linebreak();
}

uint32 SamOutput::process_one_alignment(const AlignmentData& alignment,
                                        const AlignmentData& mate)
{
    // output SamAlignment structure, to be filled out and sent to output_alignment
    SamAlignment sam_align;

    const uint32 ref_cigar_len = reference_cigar_length(alignment.cigar, alignment.cigar_len);

    // setup alignment information
    const uint32 seq_index = uint32(std::upper_bound(
        bnt.sequence_index,
        bnt.sequence_index + bnt.n_seqs,
        alignment.cigar_pos ) - bnt.sequence_index) - 1u;

    // if we're doing paired-end alignment, the mate must be valid
    NVBIO_CUDA_ASSERT(alignment_type == SINGLE_END || mate.valid == true);

    // fill out read name
    sam_align.qname = alignment.read_name;

    // fill out sequence data
    for(uint32 i = 0; i < alignment.read_len; i++)
    {
        uint8 s;

        if (alignment.aln->m_rc)
        {
            nvbio::complement_functor<4> complement;
            s = complement(alignment.read_data[i]);
        }
        else
            s = alignment.read_data[alignment.read_len - i - 1];

        sam_align.seq[i] = dna_to_char(s);
    }
    sam_align.seq[alignment.read_len] = '\0';

    // fill out quality data
    for(uint32 i = 0; i < alignment.read_len; i++)
    {
        char q;

        if (alignment.aln[MATE_1].m_rc)
            q = alignment.qual[i];
        else
            q = alignment.qual[alignment.read_len - i - 1];

        sam_align.qual[i] = q + 33;
    }
    sam_align.qual[alignment.read_len] = '\0';

    // compute mapping quality
    sam_align.mapq = alignment.mapq;

    // if we didn't map, or mapped with low quality, output an unmapped alignment and return
    if (!(alignment.aln->is_aligned() || sam_align.mapq < mapq_filter))
    {
        sam_align.flags = SAM_FLAGS_UNMAPPED;
        // mark the md string as empty
        sam_align.md_string[0] = '\0';

        // unaligned reads don't need anything else; output and return
        output_alignment(sam_align);
        return 0;
    }

    // compute alignment flags
    sam_align.flags  = (alignment.aln->mate() ? SAM_FLAGS_READ_2 : SAM_FLAGS_READ_1);
    if (alignment.aln->m_rc)
        sam_align.flags |= SAM_FLAGS_REVERSE;

    if (alignment_type == PAIRED_END)
    {
        NVBIO_CUDA_ASSERT(mate.valid);

        sam_align.flags |= SAM_FLAGS_PAIRED;

        if (mate.aln->is_concordant())
            sam_align.flags |= SAM_FLAGS_PROPER_PAIR;

        if (!mate.aln->is_aligned())
            sam_align.flags |= SAM_FLAGS_MATE_UNMAPPED;

        if (mate.aln->is_rc())
            sam_align.flags |= SAM_FLAGS_MATE_REVERSE;
    }

    if (alignment.cigar_pos + ref_cigar_len > bnt.sequence_index[ seq_index+1 ])
    {
        // flag UNMAP as this alignment bridges two adjacent reference sequences
        // xxxnsubtil: we still output the rest of the alignment data, does that make sense?
        sam_align.flags |= SAM_FLAGS_UNMAPPED;
        // unmapped segments get their mapq set to 0
        sam_align.mapq = 0;
    }

    sam_align.rname = bnt.names + bnt.names_index[ seq_index ];
    sam_align.pos = uint32( alignment.cigar_pos - bnt.sequence_index[ seq_index ] + 1 );

    // fill out the cigar string...
    uint32 computed_cigar_len = generate_cigar_string(sam_align, alignment);
    // ... and make sure it makes (some) sense
    if (computed_cigar_len != alignment.read_len)
    {
        log_error(stderr, "SAM output : cigar length doesn't match read %u (%u != %u)\n",
                  alignment.read_id /* xxxnsubtil: global_read_id */,
                  computed_cigar_len, alignment.read_len);
        return sam_align.mapq;
    }

    if (alignment_type == PAIRED_END)
    {
        if (mate.aln->is_aligned())
        {
            const uint32 o_ref_cigar_len = reference_cigar_length(mate.cigar, mate.cigar_len);

            // setup alignment information for the mate
            const uint32 o_seq_index = uint32(std::upper_bound(
                bnt.sequence_index,
                bnt.sequence_index + bnt.n_seqs,
                mate.cigar_pos ) - bnt.sequence_index) - 1u;

            if (o_seq_index == seq_index)
                sam_align.rnext = "=";
            else
                sam_align.rnext = bnt.names + bnt.names_index[ o_seq_index ];

            sam_align.pnext = uint32( mate.cigar_pos - bnt.sequence_index[ o_seq_index ] + 1 );
            if (o_seq_index != seq_index)
                sam_align.tlen = 0;
            else
            {
                sam_align.tlen = nvbio::max(mate.cigar_pos + o_ref_cigar_len,
                                            alignment.cigar_pos + ref_cigar_len) -
                                 nvbio::min(mate.cigar_pos, alignment.cigar_pos);

                if (mate.cigar_pos < alignment.cigar_pos)
                    sam_align.tlen = -sam_align.tlen;
            }
        } else {
            // other mate is unmapped
            sam_align.rnext = "=";
            sam_align.pnext = (int)(alignment.cigar_pos - bnt.sequence_index[ seq_index ] + 1);
            // xxx: check whether this is really correct...
            sam_align.tlen = 0;
        }
    } else {
        sam_align.rnext = NULL;
        sam_align.pnext = 0;
        sam_align.tlen = 0;
    }

    // fill out tag data
    sam_align.ed = alignment.aln->ed();
    sam_align.score = alignment.aln->score();

    sam_align.second_score_valid = false; // TODO!

    generate_md_string(sam_align, alignment);

    // write out the alignment
    output_alignment(sam_align);

    return sam_align.mapq;
}

void SamOutput::process(struct HostOutputBatchSE& batch)
{
    float time = 0.0f;
    {
        ScopedTimer<float> timer( &time );
        ScopedLock lock( &mutex );

        for(uint32 c = 0; c < batch.count; c++)
        {
            AlignmentData alignment = get(batch, c);
            AlignmentData mate = AlignmentData::invalid();

            process_one_alignment(alignment, mate);
        }
    }
    iostats.n_reads += batch.count;
    iostats.output_process_timings.add( batch.count, time );
}

void SamOutput::process(struct HostOutputBatchPE& batch)
{
    float time = 0.0f;
    {
        ScopedTimer<float> timer( &time );
        ScopedLock lock( &mutex );

        for(uint32 c = 0; c < batch.count; c++)
        {
            AlignmentData alignment = get_anchor_mate(batch,c);
            AlignmentData mate      = get_opposite_mate(batch,c);

            process_one_alignment(alignment, mate);
            process_one_alignment(mate, alignment);
        }
    }
    iostats.n_reads += batch.count;
    iostats.output_process_timings.add( batch.count, time );
}

void SamOutput::close(void)
{
    fclose(fp);
    fp = NULL;
}

} // namespace io
} // namespace nvbio
