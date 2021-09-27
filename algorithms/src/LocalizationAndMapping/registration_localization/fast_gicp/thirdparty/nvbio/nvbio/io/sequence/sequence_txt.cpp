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

#include <nvbio/io/sequence/sequence_txt.h>
#include <nvbio/io/sequence/sequence_encoder.h>
#include <nvbio/basic/types.h>

#include <string.h>
#include <ctype.h>

namespace nvbio {
namespace io {

///@addtogroup IO
///@{

///@addtogroup SequenceIO
///@{

///@addtogroup SequenceIODetail
///@{

int SequenceDataFile_TXT::nextChunk(SequenceDataEncoder* output, uint32 max_reads, uint32 max_bps)
{
    const char* name = "";

    uint32 n_reads = 0;
    uint32 n_bps   = 0;

    const uint32 read_mult =
        ((m_options.flags & FORWARD)            ? 1u : 0u) +
        ((m_options.flags & REVERSE)            ? 1u : 0u) +
        ((m_options.flags & FORWARD_COMPLEMENT) ? 1u : 0u) +
        ((m_options.flags & REVERSE_COMPLEMENT) ? 1u : 0u);

    const uint32 max_read_len = 16*1024*1024;

    const uint8 best_quality = uint8('~'); // ASCII code 126

    if (m_read_bp.size() < max_read_len)
    {
        m_read_bp.resize( max_read_len );
        m_read_q.resize( max_read_len, best_quality );
    }

    while (n_reads + read_mult                             <= max_reads &&
           n_bps   + read_mult*SequenceDataFile::LONG_READ <= max_bps)
    {
      #define NVBIO_TXT_LINE_PARSING

      #if defined(NVBIO_TXT_LINE_PARSING)
        uint32 read_len = 0;
        uint32 read_off = 0;

        while (1)
        {
            char* read_ptr = (char*)&m_read_bp[0] + read_off;

            if (gets( read_ptr, m_read_bp.size() - read_off ) == 0)
            {
                m_file_state = FILE_EOF;
                break;
            }

            // look for the new read length
            read_len = strlen( read_ptr ) + read_off;

            // check whether we reached the end of the buffer without finding a newline
            if (read_len == m_read_bp.size()-1 &&
                m_read_bp[ read_len-1 ] != '\n')
            {
                // resize the read
                m_read_bp.resize( m_read_bp.size() * 2 );

                // advance the buffer offset
                read_off = read_len;
            }
            else
                break;
        }

        // drop the newline
        if (read_len && m_read_bp[ read_len-1 ] == '\n')
            read_len--;
     #else
        // reset the read
        uint32 read_len = 0;

        // read an entire line
        for (uint8 c = get(); c != '\n' && c != 0; c = get())
        {
            // if (isgraph(c))
            if (c >= 0x21 && c <= 0x7E)
            {
                if (m_read_bp.size() < read_len)
                    m_read_bp.resize( read_len );

                m_read_bp[ read_len++ ] = c;
            }
        }
     #endif

        ++m_line;

        if (m_read_q.size() < read_len)
        {
            // extend the quality score vector if needed
            const size_t old_size = m_read_q.size();
            m_read_q.resize( read_len );
            for (size_t i = old_size; i < read_len; ++i)
                m_read_q[i] = best_quality;
        }

        if (read_len)
        {
            if (m_options.flags & FORWARD)
            {
                output->push_back(
                                read_len,
                                name,
                                &m_read_bp[0],
                                &m_read_q[0],
                                m_options.qualities,
                                m_options.max_sequence_len,
                                m_options.trim3,
                                m_options.trim5,
                                SequenceDataEncoder::NO_OP );
            }
            if (m_options.flags & REVERSE)
            {
                output->push_back(
                                read_len,
                                name,
                                &m_read_bp[0],
                                &m_read_q[0],
                                m_options.qualities,
                                m_options.max_sequence_len,
                                m_options.trim3,
                                m_options.trim5,
                                SequenceDataEncoder::REVERSE_OP );
            }
            if (m_options.flags & FORWARD_COMPLEMENT)
            {
                output->push_back(
                                read_len,
                                name,
                                &m_read_bp[0],
                                &m_read_q[0],
                                m_options.qualities,
                                m_options.max_sequence_len,
                                m_options.trim3,
                                m_options.trim5,
                                SequenceDataEncoder::COMPLEMENT_OP );
            }
            if (m_options.flags & REVERSE_COMPLEMENT)
            {
                output->push_back(
                                read_len,
                                name,
                                &m_read_bp[0],
                                &m_read_q[0],
                                m_options.qualities,
                                m_options.max_sequence_len,
                                m_options.trim3,
                                m_options.trim5,
                                SequenceDataEncoder::REVERSE_COMPLEMENT_OP );
            }

            n_bps   += read_mult * (uint32)read_len;
            n_reads += read_mult;
        }

        // check for end-of-file
        if (m_file_state != FILE_OK)
            break;
    }
    return n_reads;
}

SequenceDataFile_TXT_gz::SequenceDataFile_TXT_gz(
    const char*             read_file_name,
    const Options&          options,
    const uint32            buffer_size)
    : SequenceDataFile_TXT(read_file_name, options, buffer_size)
{
    m_file = gzopen(read_file_name, "r");
    if (!m_file) {
        m_file_state = FILE_OPEN_FAILED;
    } else {
        m_file_state = FILE_OK;
    }

    gzbuffer(m_file, m_buffer_size);
}

SequenceDataFile_TXT_gz::~SequenceDataFile_TXT_gz()
{
    gzclose( m_file );
}

// rewind the file
//
bool SequenceDataFile_TXT_gz::rewind()
{
    if (m_file == NULL)
        return false;

    gzrewind( m_file );

    m_file_state = FILE_OK;

    m_buffer_size = (uint32)m_buffer.size();
    m_buffer_pos  = (uint32)m_buffer.size();
    m_line        = 0;
    return true;
}

SequenceDataFile_TXT::FileState SequenceDataFile_TXT_gz::fillBuffer(void)
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

            log_error(stderr, "error processing TXT file: zlib error %d (%s)\n", err, msg);
            return FILE_STREAM_ERROR;
        }
    }

    return FILE_OK;
}

// constructor
//
SequenceDataOutputFile_TXT::SequenceDataOutputFile_TXT(
    const char* file_name,
    const char* compressor,
    const char* options)
  : m_file_name(file_name)
{
    m_file = open_output_file( file_name, compressor, options );
}

namespace {

template <Alphabet ALPHABET>
void write(
   OutputStream*                            output_file,
   const io::SequenceDataAccess<ALPHABET>&  sequence_data)
{
    typedef typename io::SequenceDataAccess<ALPHABET>::sequence_string  sequence_string;

    const uint32 buffer_len = sequence_data.size() + sequence_data.bps();

    std::vector<char> buffer( buffer_len );

    #pragma omp parallel for
    for (int32 i = 0; i < (int32)sequence_data.size(); ++i)
    {
        const sequence_string read = sequence_data.get_read( i );

        uint32 buffer_offset = sequence_data.get_range( i ).x + i;

        to_string<ALPHABET>( read.begin(), read.size(), &buffer[0] + buffer_offset );

        buffer[ buffer_offset + read.size() ] = '\n';
    }

    if (output_file->write( buffer_len, &buffer[0] ) == 0)
        throw runtime_error( "failed writing TXT output file" );
}

} // anonymous namespace

// next batch
//
void SequenceDataOutputFile_TXT::next(const SequenceDataHost& sequence_data)
{
    if (sequence_data.alphabet() == DNA)
        write( m_file, io::SequenceDataAccess<DNA>( sequence_data ) );
    else if (sequence_data.alphabet() == DNA_N)
        write( m_file, io::SequenceDataAccess<DNA_N>( sequence_data ) );
    else if (sequence_data.alphabet() == PROTEIN)
        write( m_file, io::SequenceDataAccess<PROTEIN>( sequence_data ) );
    else if (sequence_data.alphabet() == RNA)
        write( m_file, io::SequenceDataAccess<RNA>( sequence_data ) );
    else if (sequence_data.alphabet() == RNA_N)
        write( m_file, io::SequenceDataAccess<RNA_N>( sequence_data ) );
    else if (sequence_data.alphabet() == ASCII)
        write( m_file, io::SequenceDataAccess<ASCII>( sequence_data ) );
}

// return whether the stream is ok
//
bool SequenceDataOutputFile_TXT::is_ok() { return m_file && m_file->is_valid(); }

///@} // SequenceIODetail
///@} // SequenceIO
///@} // IO

} // namespace io
} // namespace nvbio
