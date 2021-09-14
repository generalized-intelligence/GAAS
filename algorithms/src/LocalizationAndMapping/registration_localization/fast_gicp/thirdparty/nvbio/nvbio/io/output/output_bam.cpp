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

#include <nvbio/io/output/output_bam.h>
#include <nvbio/io/output/output_sam.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/omp.h>

#include <stdio.h>
#include <stdarg.h>

namespace nvbio {
namespace io {

BamOutput::BamOutput(const char *file_name, AlignmentType alignment_type, BNT bnt)
    : OutputFile(file_name, alignment_type, bnt)
{
    fp = fopen(file_name, "wt");
    if (fp == NULL)
    {
        log_error(stderr, "BamOutput: could not open %s for writing\n", file_name);
        return;
    }

    // set a 256kb output buffer on fp and make sure it's not line buffered
    // this makes sure small fwrites do not land on disk straight away
    // (256kb was chosen based on the default stripe size for Linux mdraid RAID-5 volumes)
    setvbuf(fp, NULL, _IOFBF, 256 * 1024);

    buffer_id = 0;
}

BamOutput::~BamOutput()
{
    if (fp)
    {
        fclose(fp);
        fp = NULL;
    }
}

uint32 BamOutput::generate_cigar(struct BAM_alignment& alnh,
                                 struct BAM_alignment_data_block& alnd,
                                 const AlignmentData& alignment)
{
    uint32 *output = alnd.cigar;
    uint32 read_len = 0;

    for(uint32 i = 0; i < alignment.cigar_len; i++)
    {
        const Cigar&  cigar_entry  = alignment.cigar[alignment.cigar_len - i - 1u];
        // convert our "MIDS" -> { 0, 1, 2, 3 } into BAM's "MIDS" -> {0, 1, 2, 4} encoding
        const uint8   cigar_op     = "\0\1\2\4"[cigar_entry.m_type];
        const uint32  cigar_op_len = cigar_entry.m_len;

        output[i] = cigar_op_len << 4 | cigar_op;

        // check that we didn't overflow the CIGAR buffer
        assert((unsigned long)(&output[i] - alnd.cigar) < (unsigned long) (sizeof(alnd.cigar) * sizeof(alnd.cigar[0]) - 1));

        // keep track of number of BPs in the original read
        if (cigar_op != 2)  // 2 == 'D'
            read_len += cigar_entry.m_len;
    }

    // output n_cigar_op
    alnh.flag_nc = (alnh.flag_nc & 0xffff0000) | (alignment.cigar_len & 0xffff);

    return read_len;
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
        in = -in;
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

// generate the MD string
uint32 BamOutput::generate_md_string(BAM_alignment& alnh, BAM_alignment_data_block& alnd, const AlignmentData& alignment)
{
    if (alignment.mds_vec == NULL)
    {
        log_warning(stderr, "  BAM: alignment %u from read %u has an empty MD string\n", alignment.aln_id, alignment.read_id);
        alnd.md_string[0] = '\0';
        return 0;
    }
    const uint32 mds_len = uint32(alignment.mds_vec[0]) | (uint32(alignment.mds_vec[1]) << 8);
    char *buffer = alnd.md_string;
    uint32 buffer_len = 0;

    uint32 i;

    alnd.mm   = 0;
    alnd.gapo = 0;
    alnd.gape = 0;

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

                alnd.mm++;
            }

            break;

        case MDS_INSERTION:
            {
                const uint8 l = alignment.mds_vec[i++];
                i += l;

                alnd.gapo++;
                alnd.gape += l - 1;
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

                alnd.gapo++;
                alnd.gape += l - 1;
            }

            break;
        }
    } while(i < mds_len);

    buffer[buffer_len] = '\0';
    return buffer_len;
}

// convert our ACGT = {0, 1, 2, 3} into BAM's ACGT = {1, 2, 4, 8} encoding
uint8 BamOutput::encode_bp(uint8 bp)
{
    if (bp > 3)
    {
        // unknown BP, map to N
        return 15;
    } else {
        return 1 << bp;
    }
}

uint32 BamOutput::process_one_alignment(AlignmentData& alignment, AlignmentData& mate)
{
    // BAM alignment header
    struct BAM_alignment alnh;
    // data block with actual alignment info
    struct BAM_alignment_data_block alnd;

    uint8 mapq;

    // xxxnsubtil: this is probably not needed
    memset(&alnh, 0, sizeof(alnh));
    memset(&alnd, 0, sizeof(alnd));

    const uint32 ref_cigar_len = reference_cigar_length(alignment.cigar, alignment.cigar_len);

    // setup alignment information
   const uint32 seq_index = uint32(std::upper_bound(
        bnt.sequence_index,
        bnt.sequence_index + bnt.n_seqs,
        alignment.cigar_pos ) - bnt.sequence_index) - 1u;

    // fill out read name and length
    alnd.name = alignment.read_name;
    alnh.bin_mq_nl = (uint8)(strlen(alnd.name) + 1);

    // fill out read data
    // (PackedStream is not used here to avoid doing a read-modify-write on every BP)
    {
        for(uint32 i = 0; i < alignment.read_len; i += 2)
        {
            uint8 out_bp;
            uint8 s;

            if (alignment.aln->m_rc)
            {
                nvbio::complement_functor<4> complement;

                s = complement(alignment.read_data[i]);
                s = encode_bp(s);
                out_bp = s << 4;

                if (i + 1 < alignment.read_len)
                {
                    s = complement(alignment.read_data[(i + 1)]);
                    s = encode_bp(s);
                    out_bp |= s;
                }
            } else {
                s = alignment.read_data[alignment.read_len - i - 1];
                s = encode_bp(s);
                out_bp = s << 4;

                if (i + 1 < alignment.read_len)
                {
                    s = alignment.read_data[alignment.read_len - (i + 1) - 1];
                    s = encode_bp(s);
                    out_bp |= s;
                }
            }

            alnd.seq[i / 2] = out_bp;
        }

        alnh.l_seq = alignment.read_len;
    }

    // fill out quality data
    for(uint32 i = 0; i < alignment.read_len; i++)
    {
        char q;

        if (alignment.aln->m_rc)
            q = alignment.qual[i];
        else
            q = alignment.qual[alignment.read_len - i - 1];

        alnd.qual[i] = q;
    }

    // compute mapping quality
    mapq = alignment.mapq;

    // check if we're mapped
    if (alignment.aln->is_aligned() == false || mapq < mapq_filter)
    {
        alnh.refID = -1;
        alnh.pos = -1;
        alnh.flag_nc = BAM_FLAGS_UNMAPPED;
        alnh.next_refID = -1;
        alnh.next_pos = -1;
        // mark the md string as empty
        alnd.md_string[0] = '\0';

        // unaligned reads don't need anything else; output and return
        output_alignment(alnh, alnd);
        return 0;
    }

    // compute alignment flags
    alnh.flag_nc = (alignment.aln->mate() ? BAM_FLAGS_READ_2 : BAM_FLAGS_READ_1);
    if (alignment.aln->m_rc)
        alnh.flag_nc |= BAM_FLAGS_REVERSE;

    if (alignment_type == PAIRED_END)
    {
        alnh.flag_nc |= BAM_FLAGS_PAIRED;

        if (mate.aln->is_concordant())
            alnh.flag_nc |= BAM_FLAGS_PROPER_PAIR;

        if (!mate.aln->is_aligned())
            alnh.flag_nc |= BAM_FLAGS_MATE_UNMAPPED;

        if (mate.aln->is_rc())
            alnh.flag_nc |= BAM_FLAGS_MATE_REVERSE;
    }

    if (alignment.cigar_pos + ref_cigar_len > bnt.sequence_index[ seq_index+1 ])
    {
        // flag UNMAP as this alignment bridges two adjacent reference sequences
        // xxxnsubtil: we still output the rest of the alignment data, does that make sense?
        alnh.flag_nc |= BAM_FLAGS_UNMAPPED;

        // make this look like a real unmapped alignment
        alnh.refID = -1;
        alnh.pos = -1;
        alnh.next_refID = -1;
        alnh.next_pos = -1;
        alnd.md_string[0] = '\0';

        output_alignment(alnh, alnd);
        return 0;
    }

    // fill out alignment reference ID and position
    alnh.refID = seq_index;
    alnh.pos = uint32(alignment.cigar_pos - bnt.sequence_index[ seq_index ]);

    // write out mapq
    alnh.bin_mq_nl |= (mapq << 8);
    // BAM alignment bin is always 0
    // xxxnsubtil: is the bin useful?

    // fill out the cigar string...
    uint32 computed_cigar_len = generate_cigar(alnh, alnd, alignment);
    // ... and make sure it makes (some) sense
    if (computed_cigar_len != alignment.read_len)
    {
        log_error(stderr, "BAM output : cigar length doesn't match read %u (%u != %u)\n",
                  alignment.read_id /* xxxnsubtil: global_read_id */,
                  computed_cigar_len, alignment.read_len);
        return mapq;
    }

    if (alignment_type == PAIRED_END)
    {
        if (mate.aln->is_aligned())
        {
            const uint32 o_ref_cigar_len = reference_cigar_length(mate.cigar, mate.cigar_len);

            // setup alignment information for the opposite mate
            const uint32 o_seq_index = uint32(std::upper_bound(
                bnt.sequence_index,
                bnt.sequence_index + bnt.n_seqs,
                mate.cigar_pos ) - bnt.sequence_index) - 1u;

            alnh.next_refID = uint32(o_seq_index - seq_index);
            // next_pos here is equivalent to SAM's PNEXT,
            // but it's zero-based in BAM and one-based in SAM
            alnh.next_pos = int32( mate.cigar_pos - bnt.sequence_index[ o_seq_index ] );

            if (o_seq_index != seq_index)
                alnh.tlen = 0;
            else
            {
                alnh.tlen = nvbio::max(mate.cigar_pos + o_ref_cigar_len,
                                       alignment.cigar_pos + ref_cigar_len) -
                            nvbio::min(mate.cigar_pos, alignment.cigar_pos);

                if (mate.cigar_pos < alignment.cigar_pos)
                    alnh.tlen = -alnh.tlen;
            }
        }
        else
        {
            // other mate is unmapped
            // xxxnsubtil: this follows the same convention that was documented in the old code for SAM,
            // except that BAM does not have an encoding for '=' here
            // it's somewhat unclear whether this is correct
            alnh.next_refID = alnh.refID;
            alnh.next_pos = int32( alignment.cigar_pos - bnt.sequence_index[ seq_index ] );
            // xxx: check whether this is really correct
            alnh.tlen = 0;
        }
    } else {
        alnh.next_refID = -1;
        alnh.next_pos = -1;
        alnh.tlen = 0;
    }

    // fill out tag data
    alnd.ed = alignment.aln->ed();
    alnd.score = alignment.aln->score();

    alnd.second_score_valid = false; // TODO!

    generate_md_string(alnh, alnd, alignment);

    // write out the alignment
    output_alignment(alnh, alnd);

    return mapq;
}

void BamOutput::output_tag_uint32(DataBuffer& out, const char *tag, uint32 val)
{
    out.append_data(tag, 2);
    out.append_uint8('i');
    out.append_uint32(val);
}

void BamOutput::output_tag_uint8(DataBuffer& out, const char *tag, uint8 val)
{
    out.append_data(tag, 2);
    out.append_uint8('c');
    out.append_uint8(val);
}

void BamOutput::output_tag_string(DataBuffer& out, const char *tag, const char *val)
{
    out.append_data(tag, 2);
    out.append_uint8('Z');
    out.append_string(val);
    out.append_uint8('\0');
}

void BamOutput::output_alignment(BAM_alignment& alnh, BAM_alignment_data_block& alnd)
{
    DataBuffer& out = data_buffers[ buffer_id ];

    // keep track of the block size offset so we can compute the block size and update it later
    uint32 off_block_size = out.get_pos();
    out.skip_ahead(sizeof(alnh.block_size));

    out.append_int32(alnh.refID);
    out.append_int32(alnh.pos);
    out.append_int32(alnh.bin_mq_nl);
    out.append_int32(alnh.flag_nc);
    out.append_int32(alnh.l_seq);
    out.append_int32(alnh.next_refID);
    out.append_int32(alnh.next_pos);
    out.append_int32(alnh.tlen);

    out.append_string(alnd.name);
    out.append_uint8('\0');

    // cigar_len = n_cigar_op (lower 16-bits of flag_nc) * sizeof(uint32)
    const uint32 cigar_len = (alnh.flag_nc & 0xffff) * sizeof(uint32);
    out.append_data(alnd.cigar, cigar_len);

    out.append_data(alnd.seq, (alnh.l_seq + 1) / 2);
    out.append_data(alnd.qual, alnh.l_seq);

    // unmapped alignments don't get auxiliary data
    if (!(alnh.flag_nc & BAM_FLAGS_UNMAPPED))
    {
        output_tag_uint8(out, "NM", alnd.ed);
        output_tag_uint32(out, "AS", alnd.score);

        if (alnd.second_score_valid)
            output_tag_uint32(out, "XS", alnd.second_score);

        output_tag_uint8(out, "XM", alnd.mm);
        output_tag_uint8(out, "XO", alnd.gapo);
        output_tag_uint8(out, "XG", alnd.gape);
        if (alnd.md_string[0])
            output_tag_string(out, "MD", alnd.md_string);
    }

    // compute alignment data block size and write it out
    const int32 aln_len = out.get_pos() - off_block_size - sizeof(alnh.block_size);
    out.poke_int32(off_block_size, aln_len);

    if (out.is_full())
        write_block();
}

void BamOutput::process(struct HostOutputBatchSE& batch)
{
    // protect this section
    ScopedLock lock( &mutex );

    float time = 0.0f;
    {
        ScopedTimer<float> timer( &time );

        for(uint32 c = 0; c < batch.count; c++)
        {
            AlignmentData alignment = get(batch, c);
            AlignmentData mate = AlignmentData::invalid();

            process_one_alignment(alignment, mate);
        }

        // flush at the end of each batch
        //if (data_buffers[ buffer_id ].get_pos())
        //    flush_blocks();
    }
    iostats.n_reads += batch.count;
    iostats.output_process_timings.add( batch.count, time );
}

void BamOutput::process(struct HostOutputBatchPE& batch)
{
    // protect this section
    ScopedLock lock( &mutex );

    float time = 0.0f;
    {
        ScopedTimer<float> timer( &time );

        for(uint32 c = 0; c < batch.count; c++)
        {
            AlignmentData alignment = get_anchor_mate(batch,c);
            AlignmentData mate      = get_opposite_mate(batch,c);

            process_one_alignment(alignment, mate);
            process_one_alignment(mate, alignment);
        }

        // flush at the end of each batch
        //if (data_buffers[ buffer_id ].get_pos())
        //    flush_blocks();
    }
    iostats.n_reads += batch.count;
    iostats.output_process_timings.add( batch.count, time );
}

void BamOutput::write_block()
{
    ++buffer_id;

    // check whether we need to flush the blocks
    if (buffer_id == BUFFERS)
        flush_blocks();
}

void BamOutput::flush_blocks()
{
    #pragma omp parallel for
    for (int32 i = 0; i < buffer_id; ++i)
    {
        bgzf[i].start_block( compressed_buffers[i] );
        bgzf[i].compress( compressed_buffers[i], data_buffers[i] );
        bgzf[i].end_block( compressed_buffers[i] );

        data_buffers[i].rewind();
    }

    // write out the compressed buffers, in order
    for (int32 i = 0; i < buffer_id; ++i)
    {
        // write the compressed buffer
        fwrite( compressed_buffers[i].get_base_ptr(), compressed_buffers[i].get_pos(), 1, fp );

        // and rewind it
        compressed_buffers[i].rewind();
    }

    // reset the buffer id and the work-queue counter
    buffer_id = 0;
}

void BamOutput::output_header(void)
{
    int pos_l_text, pos_start_header, header_len;

    // names in parenthesis refer to the field names in the BAM spec
    DataBuffer& data_buffer = data_buffers[ buffer_id ];

    // write magic string (magic)
    data_buffer.append_string("BAM\1");
    // skip ahead header length field (l_text), will fill later
    pos_l_text = data_buffer.get_pos();
    data_buffer.skip_ahead(sizeof(uint32));

    pos_start_header = data_buffer.get_pos();

    // fill out SAM header (text)
    data_buffer.append_string("@HD\t");
    data_buffer.append_string("VN:1.3\n");
    if (!rg_id.empty())
    {
        // write the RG:ID
        data_buffer.append_string("@RG");
        data_buffer.append_string("\tID:");
        data_buffer.append_string(rg_id.c_str());

        // write the other RG tags
        if (!rg_string.empty())
            data_buffer.append_string( rg_string.c_str() );

        data_buffer.append_string("\n");
    }
    data_buffer.append_string("@PG");
    data_buffer.append_string("\tID:");
    data_buffer.append_string(pg_id.c_str());
    data_buffer.append_string("\tPN:");
    data_buffer.append_string(pg_name.c_str());
    // VN was bumped to 0.5.1 to distinguish between the new and old output code
    data_buffer.append_string("\tVN:");
    data_buffer.append_string(pg_version.c_str());
    data_buffer.append_string("\n");
    // samtools does not cope with a null terminator here, so don't write one out
//    data_buffer.append_int8(0);

    // compute and write out the size of the SAM header (l_text)
    header_len = data_buffer.get_pos() - pos_start_header;
    data_buffer.poke_int32(pos_l_text, header_len);

    // output the number of reference sequences (n_ref)...
    data_buffer.append_int32(bnt.n_seqs);
    // ... and the information for each
    for (uint32 i = 0; i < bnt.n_seqs; i++)
    {
        const char *name = bnt.names + bnt.names_index[i];

        // sequence name length including null terminator (l_name)
        data_buffer.append_int32( int32( strlen(name) + 1 ) );
        // write out sequence name string and null-terminator (name)
        data_buffer.append_string(name);
        data_buffer.append_int8(0);
        // sequence length (l_ref)
        data_buffer.append_int32(bnt.sequence_index[i+1] - bnt.sequence_index[i]);
    }

    // compress and write out the header block separately
    // (this yields a slightly smaller file)
    write_block();
}

void BamOutput::close()
{
    // protect this section
    ScopedLock lock( &mutex );

    // flush all non-emtpy blocks
    if (data_buffers[ buffer_id ].get_pos())
        write_block();

    flush_blocks();

    NVBIO_CUDA_ASSERT(fp);

    // write out the BAM EOF marker
    static const unsigned char magic[28] =  { 0037, 0213, 0010, 0004, 0000, 0000, 0000, 0000, 0000,
                                              0377, 0006, 0000, 0102, 0103, 0002, 0000, 0033, 0000,
                                              0003, 0000, 0000, 0000, 0000, 0000, 0000, 0000, 0000, 0000 };

    fwrite(magic, sizeof(magic), 1, fp);

    fclose(fp);
    fp = NULL;
}

} // namespace io
} // namespace nvbio
