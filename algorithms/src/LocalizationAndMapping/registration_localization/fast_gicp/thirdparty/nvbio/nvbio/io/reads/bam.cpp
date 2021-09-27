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

#include <stdlib.h>
#include <string.h>
#include <zlib/zlib.h>

#include <nvbio/basic/console.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/io/reads/bam.h>
#include <nvbio/io/reads/sam.h>

namespace nvbio {
namespace io {

///@addtogroup IO
///@{

///@addtogroup ReadsIO
///@{

///@addtogroup ReadsIODetail
///@{

ReadDataFile_BAM::ReadDataFile_BAM(const char *read_file_name,
                                   const uint32 max_reads,
                                   const uint32 truncate_read_len,
                                   const ReadEncoding flags)
  : ReadDataFile(max_reads, truncate_read_len, flags)
{
    fp = gzopen(read_file_name, "rb");
    if (fp == NULL)
    {
        // this will cause init() to fail below
        log_error(stderr, "unable to open BAM file %s\n", read_file_name);
        m_file_state = FILE_OPEN_FAILED;
    } else {
        m_file_state = FILE_OK;
    }
}

bool ReadDataFile_BAM::readData(void *output, unsigned int len)
{
    unsigned int ret;

    ret = gzread(fp, output, len);
    if (ret > 0)
    {
        return true;
    } else {
        // check for EOF separately; zlib will not always return Z_STREAM_END at EOF below
        if (gzeof(fp))
        {
            m_file_state = FILE_EOF;
        } else {
            // ask zlib what happened and inform the user
            int err;
            const char *msg;

            msg = gzerror(fp, &err);
            // we're making the assumption that we never see Z_STREAM_END here
            assert(err != Z_STREAM_END);

            log_error(stderr, "error processing BAM file: zlib error %d (%s)\n", err, msg);
            m_file_state = FILE_STREAM_ERROR;
        }

        return false;
    }
}

// read in a structure field from fp
// returns error (local variable of the right type) if read fails
#define GZREAD(field)                                           \
    if (readData(&(field), sizeof(field)) == false) {           \
        return error;                                           \
    }

// skip bytes in fp
// note that gzseek won't actually detect EOF, so we don't check return values here
#define GZFWD(bytes) \
    gzseek(fp, (bytes), SEEK_CUR)

// skip a structure field in fp
#define GZSKIP(field) \
    gzseek(fp, sizeof(field), SEEK_CUR)

bool ReadDataFile_BAM::init(void)
{
    // error constant for GZREAD
    static const bool error = false;

    BAM_header header;
    int c;

    if (fp == NULL)
    {
        // file failed to open
        return false;
    }

    // parse the BAM header
    GZREAD(header.magic);

    if (header.magic[0] != 'B' ||
        header.magic[1] != 'A' ||
        header.magic[2] != 'M' ||
        header.magic[3] != '\1')
    {
        log_error(stderr, "error parsing BAM file (invalid magic)\n");
        m_file_state = FILE_PARSE_ERROR;
        return false;
    }

    // read in header text length and skip header text
    GZREAD(header.l_text);
    GZFWD(header.l_text);

    // skip reference sequence data
    GZREAD(header.n_ref);
    for(c = 0; c < header.n_ref; c++)
    {
        BAM_reference ref;
        GZREAD(ref.l_name);
        GZFWD(ref.l_name + sizeof(ref.l_ref));
    }

    return true;
}

namespace {

// decode a BAM bp into ascii
inline unsigned char decode_BAM_bp(uint8 bp)
{
    static const char table[] = "=ACMGRSVTWYHKDBN";

    assert(bp < 16);
    return table[bp];
}

}

// grab the next chunk of reads from the file, up to max_reads
int ReadDataFile_BAM::nextChunk(ReadDataRAM *output, uint32 max_reads, uint32 max_bps)
{
    if (max_bps < ReadDataFile::LONG_READ)
        return 0;

    // error code for gzread
    static const int error = -1;

    // small utility structure to keep track of data buffers and free them as appropriate
    // (note the slightly evil initializer that sets all pointers to NULL)
    struct data
    {
        char *read_name;
        uint8 *encoded_read;
        uint8 *decoded_read;
        uint8 *quality;

        ~data()
        {
            if (read_name)
                free(read_name);

            if (encoded_read)
                free(encoded_read);

            if (decoded_read)
                free(decoded_read);

            if (quality)
                free(quality);
        }
    } data = { };

    // utility structure to keep track of alignment header data
    BAM_alignment align;

    z_off_t read_block_start;
    int read_name_len;
    int read_flags;
    int cigar_len;
    int encoded_read_len;

    int c;

    // are we done?
    if (gzeof(fp))
    {
        m_file_state = FILE_EOF;
        return 0;
    }

    // parse and skip all non-primary reads
    do {
        // read in the block_size
        GZREAD(align.block_size);

        // record the starting file position for this read block
        read_block_start = gztell(fp);

        // skip uninsteresting fields
        GZFWD(sizeof(align.refID) +
              sizeof(align.pos));

        GZREAD(align.bin_mq_nl);
        GZREAD(align.flag_nc);

        // compute read flags
        read_flags = align.flag_nc >> 16;
        if (read_flags & SAMFlag_SecondaryAlignment)
        {
            // we're not interested in this read; skip the remainder of the read block and loop
            uint32 skip = align.block_size - (gztell(fp) - read_block_start);
            assert(skip);
            GZFWD(skip);

            continue;
        }
    } while (read_flags & SAMFlag_SecondaryAlignment);

    // this is a primary read, so read the read
    GZREAD(align.l_seq);
    GZFWD(sizeof(align.next_refID) +
          sizeof(align.next_pos) +
          sizeof(align.tlen));

    // read in the name (and add a null-terminator just in case)
    read_name_len = align.bin_mq_nl & 0xff;
    data.read_name = (char *) malloc(read_name_len + 1);
    data.read_name[read_name_len] = 0;

    if (gzread(fp, data.read_name, read_name_len) != read_name_len)
    {
        log_error(stderr, "error processing BAM file (could not fetch read name)\n");
        m_file_state = FILE_STREAM_ERROR;
        return 0;
    }

    // skip the cigar
    cigar_len = (align.flag_nc & 0xffff) * sizeof(uint32);
    GZFWD(cigar_len);

    // grab the read data
    encoded_read_len = (align.l_seq + 1) / 2;
    data.encoded_read = (uint8 *) malloc((align.l_seq + 1) / 2);

    if (gzread(fp, data.encoded_read, encoded_read_len) != encoded_read_len)
    {
        log_error(stderr, "error processing BAM file (could not fetch sequence data)\n");
        m_file_state = FILE_STREAM_ERROR;
        return 0;
    }

    // read in the quality data
    data.quality = (uint8 *) malloc(align.l_seq);
    if (gzread(fp, data.quality, align.l_seq) != align.l_seq)
    {
        log_error(stderr, "error processing BAM file (could not fetch quality data)\n");
        m_file_state = FILE_STREAM_ERROR;
        return 0;
    }

    // skip the rest of the read block
    uint32 skip = align.block_size - (gztell(fp) - read_block_start);
    GZFWD(skip);

    // decode the read data into a null-terminated string
    data.decoded_read = (uint8 *) malloc(align.l_seq + 1);
    data.decoded_read[align.l_seq] = 0;

    PackedStream<uint8*, uint8, 4, true> stream(data.encoded_read);
    for(c = 0; c < align.l_seq; c++)
    {
        data.decoded_read[c] = decode_BAM_bp(stream[c]);
    }

    if (m_flags & FORWARD)
    {
        const ReadDataRAM::StrandOp op = (read_flags & SAMFlag_ReverseComplemented) ?
            ReadDataRAM::REVERSE_COMPLEMENT_OP : ReadDataRAM::NO_OP;

        // add the read into the batch
        output->push_back(align.l_seq,
                          data.read_name,
                          data.decoded_read,
                          data.quality,
                          Phred,
                          m_truncate_read_len,
                          op );
    }
    if (m_flags & REVERSE)
    {
        const ReadDataRAM::StrandOp op = (read_flags & SAMFlag_ReverseComplemented) ?
            ReadDataRAM::COMPLEMENT_OP : ReadDataRAM::REVERSE_OP;

        output->push_back(align.l_seq,
                          data.read_name,
                          data.decoded_read,
                          data.quality,
                          Phred,
                          m_truncate_read_len,
                          op );
    }
    if (m_flags & FORWARD_COMPLEMENT)
    {
        const ReadDataRAM::StrandOp op = (read_flags & SAMFlag_ReverseComplemented) ?
            ReadDataRAM::REVERSE_OP : ReadDataRAM::COMPLEMENT_OP;

        output->push_back(align.l_seq,
                          data.read_name,
                          data.decoded_read,
                          data.quality,
                          Phred,
                          m_truncate_read_len,
                          op );
    }
    if (m_flags & REVERSE_COMPLEMENT)
    {
        const ReadDataRAM::StrandOp op = (read_flags & SAMFlag_ReverseComplemented) ?
            ReadDataRAM::NO_OP : ReadDataRAM::REVERSE_COMPLEMENT_OP;

        output->push_back(align.l_seq,
                          data.read_name,
                          data.decoded_read,
                          data.quality,
                          Phred,
                          m_truncate_read_len,
                          op );
    }

    // we always return 1 read at a time
    return 1;
}

///@} // ReadsIODetail
///@} // ReadsIO
///@} // IO

} // namespace io
} // namespace nvbio
