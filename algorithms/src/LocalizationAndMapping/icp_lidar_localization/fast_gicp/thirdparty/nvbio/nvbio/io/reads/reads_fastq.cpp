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

#include <nvbio/io/reads/reads_fastq.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/timer.h>

#include <string.h>
#include <ctype.h>

namespace nvbio {
namespace io {

///@addtogroup IO
///@{

///@addtogroup ReadsIO
///@{

///@addtogroup ReadsIODetail
///@{

int ReadDataFile_FASTQ_parser::nextChunk(ReadDataRAM *output, uint32 max_reads, uint32 max_bps)
{
    uint32 n_reads = 0;
    uint32 n_bps   = 0;
    uint8  marker;

    const uint32 read_mult =
        ((m_flags & FORWARD)            ? 1u : 0u) +
        ((m_flags & REVERSE)            ? 1u : 0u) +
        ((m_flags & FORWARD_COMPLEMENT) ? 1u : 0u) +
        ((m_flags & REVERSE_COMPLEMENT) ? 1u : 0u);

    while (n_reads + read_mult                         <= max_reads &&
           n_bps   + read_mult*ReadDataFile::LONG_READ <= max_bps)
    {
        // consume spaces & newlines
        do {
            marker = get();

            // count lines
            if (marker == '\n')
                m_line++;
        }
        while (marker == '\n' || marker == ' ');

        // check for EOF or read errors
        if (m_file_state != FILE_OK)
            break;

        // if the newlines didn't end in a read marker,
        // issue a parsing error...
        if (marker != '@')
        {
            m_file_state = FILE_PARSE_ERROR;
            m_error_char = marker;
            return uint32(-1);
        }

        // read all the line
        uint32 len = 0;
        for (uint8 c = get(); c != '\n' && c != 0; c = get())
        {
            m_name[ len++ ] = c;

            // expand on demand
            if (m_name.size() <= len)
                m_name.resize( len * 2u );
        }

        m_name[ len++ ] = '\0';

        // check for errors
        if (m_file_state != FILE_OK)
        {
            log_error(stderr, "incomplete read!\n");

            m_error_char = 0;
            return uint32(-1);
        }

        m_line++;

        // start reading the bp read
        len = 0;
        for (uint8 c = get(); c != '+' && c != 0; c = get())
        {
            // if (isgraph(c))
            if (c >= 0x21 && c <= 0x7E)
                m_read_bp[ len++ ] = c;
            else if (c == '\n')
                m_line++;

            // expand on demand
            if (m_read_bp.size() <= len)
            {
                m_read_bp.resize( len * 2u );
                m_read_q.resize(  len * 2u );
            }
        }

        // check for errors
        if (m_file_state != FILE_OK)
        {
            log_error(stderr, "incomplete read!\n");

            m_error_char = 0;
            return uint32(-1);
        }

        // read all the line
        for(uint8 c = get(); c != '\n' && c != 0; c = get()) {}

        // check for errors
        if (m_file_state != FILE_OK)
        {
            log_error(stderr, "incomplete read!\n");

            m_error_char = 0;
            return uint32(-1);
        }

        m_line++;

        // start reading the quality read
        len = 0;
        for (uint8 c = get(); c != '\n' && c != 0; c = get())
            m_read_q[ len++ ] = c;

        // check for errors
        if (m_file_state != FILE_OK)
        {
            log_error(stderr, "incomplete read!\n");

            m_error_char = 0;
            return uint32(-1);
        }

        m_line++;

        if (m_flags & FORWARD)
        {
            output->push_back( len,
                              &m_name[0],
                              &m_read_bp[0],
                              &m_read_q[0],
                              m_quality_encoding,
                              m_truncate_read_len,
                              ReadDataRAM::NO_OP );
        }
        if (m_flags & REVERSE)
        {
            output->push_back( len,
                              &m_name[0],
                              &m_read_bp[0],
                              &m_read_q[0],
                              m_quality_encoding,
                              m_truncate_read_len,
                              ReadDataRAM::REVERSE_OP );
        }
        if (m_flags & FORWARD_COMPLEMENT)
        {
            output->push_back( len,
                              &m_name[0],
                              &m_read_bp[0],
                              &m_read_q[0],
                              m_quality_encoding,
                              m_truncate_read_len,
                              ReadDataRAM::COMPLEMENT_OP );
        }
        if (m_flags & REVERSE_COMPLEMENT)
        {
            output->push_back( len,
                              &m_name[0],
                              &m_read_bp[0],
                              &m_read_q[0],
                              m_quality_encoding,
                              m_truncate_read_len,
                              ReadDataRAM::REVERSE_COMPLEMENT_OP );
        }

        n_bps   += read_mult * len;
        n_reads += read_mult;
    }
    return n_reads;
}

ReadDataFile_FASTQ_gz::ReadDataFile_FASTQ_gz(const char *read_file_name,
                                             const QualityEncoding qualities,
                                             const uint32 max_reads,
                                             const uint32 max_read_len,
                                             const ReadEncoding flags)
    : ReadDataFile_FASTQ_parser(read_file_name, qualities, max_reads, max_read_len, flags)
{
    m_file = gzopen(read_file_name, "r");
    if (!m_file) {
        m_file_state = FILE_OPEN_FAILED;
    } else {
        m_file_state = FILE_OK;
    }

    gzbuffer(m_file, m_buffer_size);
}

static float time = 0.0f;

ReadDataFile_FASTQ_parser::FileState ReadDataFile_FASTQ_gz::fillBuffer(void)
{
    m_buffer_size = gzread(m_file, &m_buffer[0], (uint32)m_buffer.size());

    if (m_buffer_size <= 0)
    {
        // check for EOF separately; zlib will not always return Z_STREAM_END at EOF below
        if (gzeof(m_file))
        {
            return FILE_EOF;
        } else {
            // ask zlib what happened and inform the user
            int err;
            const char *msg;

            msg = gzerror(m_file, &err);
            // we're making the assumption that we never see Z_STREAM_END here
            assert(err != Z_STREAM_END);

            log_error(stderr, "error processing FASTQ file: zlib error %d (%s)\n", err, msg);
            return FILE_STREAM_ERROR;
        }
    }
    return FILE_OK;
}

///@} // ReadsIODetail
///@} // ReadsIO
///@} // IO

} // namespace io
} // namespace nvbio
