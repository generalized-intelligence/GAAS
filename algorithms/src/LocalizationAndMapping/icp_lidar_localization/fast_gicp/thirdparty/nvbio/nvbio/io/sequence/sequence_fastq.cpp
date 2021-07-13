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

#include <nvbio/io/sequence/sequence_fastq.h>
#include <nvbio/io/sequence/sequence_encoder.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/timer.h>

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

int SequenceDataFile_FASTQ_parser::nextChunk(SequenceDataEncoder *output, uint32 max_reads, uint32 max_bps)
{
    uint32 n_reads = 0;
    uint32 n_bps   = 0;
    char   marker;

    const uint32 read_mult =
        ((m_options.flags & FORWARD)            ? 1u : 0u) +
        ((m_options.flags & REVERSE)            ? 1u : 0u) +
        ((m_options.flags & FORWARD_COMPLEMENT) ? 1u : 0u) +
        ((m_options.flags & REVERSE_COMPLEMENT) ? 1u : 0u);

    while (n_reads + read_mult                             <= max_reads &&
           n_bps   + read_mult*SequenceDataFile::LONG_READ <= max_bps)
    {
    #if defined(NVBIO_WEAK_FASTQ_SUPPORT)
        //
        // This parser works only with properly formatted, modern FASTQ files which
        // don't split reads across multiple lines and don't contain extraneous
        // characters...
        //

        const int MAX_LINE = 32*1024;
        char line_buffer[MAX_LINE];

        // read the name line
        if (gets( line_buffer, MAX_LINE ) == false)
        {
            m_file_state = FILE_EOF;
            break;
        }
        m_line++;

        strcpy( (char*)&m_name[0], &line_buffer[1] );

        // read the sequence line
        if (gets( (char*)&m_read_bp[0], (int)m_read_bp.size() ) == false)
        {
            log_error(stderr, "FASTQ loader: incomplete read at line %u!\n", m_line);
            m_file_state = FILE_PARSE_ERROR;
            return -1;
        }
        m_line++;

        const uint32 len = strlen( (const char*)&m_read_bp[0] );

        // read the "+" line
        if (gets( line_buffer, MAX_LINE ) == false)
        {
            log_error(stderr, "FASTQ loader: incomplete read at line %u!\n", m_line);
            m_file_state = FILE_PARSE_ERROR;
            return -1;
        }
        m_line++;

        // read the qualities line
        if (gets( (char*)&m_read_q[0], (int)m_read_q.size() ) == false)
        {
            log_error(stderr, "FASTQ loader: incomplete read at line %u!\n", m_line);
            m_file_state = FILE_PARSE_ERROR;
            return -1;
        }
        m_line++;
    #else
        // consume spaces & newlines
        do {
            marker = get();

            // count lines
            if (marker == '\n')
                m_line++;
        }
        while (marker >= 1 && marker <= 31);

        // check for EOF or read errors
        if (m_file_state != FILE_OK)
            break;

        // if the newlines didn't end in a read marker,
        // issue a parsing error...
        if (marker != '@')
        {
            log_error(stderr, "FASTQ loader: parsing error at %u!\n", m_line);

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
            log_error(stderr, "FASTQ loader: incomplete read at line %u!\n", m_line);

            m_error_char = 0;
            return uint32(-1);
        }

        m_line++;

        // start reading the bp read
        len = 0;
        for (char c = get(); c != '+' && c != 0; c = get())
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

        const uint32 read_len = len;

        // check for errors
        if (m_file_state != FILE_OK)
        {
            log_error(stderr, "FASTQ loader: incomplete read at line %u!\n", m_line);

            m_error_char = 0;
            return uint32(-1);
        }

        // read all the line
        for (char c = get(); c != '\n' && c != 0; c = get()) {}

        // check for errors
        if (m_file_state != FILE_OK)
        {
            log_error(stderr, "FASTQ loader: incomplete read at line %u!\n", m_line);

            m_error_char = 0;
            return uint32(-1);
        }

        m_line++;

        // start reading the quality read
        len = 0;

        // read as many qualities as there are in the read
        for (char c = get(); (len < read_len) && (m_file_state == FILE_OK); c = get())
        {
            // if (isgraph(c))
            if (c >= 0x21 && c <= 0x7E)
                m_read_q[ len++ ] = c;
            else if (c == '\n')
                m_line++;
        }
        /*
        // the below works for proper FASTQ files, but not for old Sanger ones
        // allowing strings to span multiple lines...
        for (uint8 c = get(); c != '\n' && c != 0; c = get())
            m_read_q[ len++ ] = c;

        if (len < read_len)
        {
            m_read_bp[ read_len ] = '\0';
            m_read_q[ len ] = '\0';

            log_error(stderr, "FASTQ loader: qualities and read lengths differ at line %u!\n", m_line);
            log_error(stderr, "  name: %s\n", &m_name[0]);
            log_error(stderr, "  read: %s\n", &m_read_bp[0]);
            log_error(stderr, "  qual: %s\n", &m_read_q[0]);

            m_file_state = FILE_PARSE_ERROR;

            m_error_char = 0;
            return uint32(-1);
        }*/

        // check for errors
        if (m_file_state != FILE_OK)
        {
            log_error(stderr, "FASTQ loader: incomplete read at line %u!\n", m_line);

            m_error_char = 0;
            return uint32(-1);
        }

        m_line++;
    #endif

        if (m_options.flags & FORWARD)
        {
            output->push_back( len,
                              &m_name[0],
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
            output->push_back( len,
                              &m_name[0],
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
            output->push_back( len,
                              &m_name[0],
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
            output->push_back( len,
                              &m_name[0],
                              &m_read_bp[0],
                              &m_read_q[0],
                              m_options.qualities,
                              m_options.max_sequence_len,
                              m_options.trim3,
                              m_options.trim5,
                              SequenceDataEncoder::REVERSE_COMPLEMENT_OP );
        }

        n_bps   += read_mult * len;
        n_reads += read_mult;
    }
    return n_reads;
}

SequenceDataFile_FASTQ_gz::SequenceDataFile_FASTQ_gz(
    const char*                         read_file_name,
    const SequenceDataFile::Options&    options)
    : SequenceDataFile_FASTQ_parser(read_file_name, options)
{
    m_file = gzopen(read_file_name, "r");
    if (!m_file) {
        m_file_state = FILE_OPEN_FAILED;
    } else {
        m_file_state = FILE_OK;
    }

    gzbuffer(m_file, m_buffer_size);
}

SequenceDataFile_FASTQ_gz::~SequenceDataFile_FASTQ_gz()
{
    gzclose( m_file );
}

//static float time = 0.0f;

SequenceDataFile_FASTQ_parser::FileState SequenceDataFile_FASTQ_gz::fillBuffer(void)
{
    m_buffer_size = gzread(m_file, &m_buffer[0], (uint32)m_buffer.size());

    if (m_buffer_size <= 0)
    {
        // check for EOF separately; zlib will not always return Z_STREAM_END at EOF below
        if (gzeof(m_file))
            return FILE_EOF;
        else
        {
            // ask zlib what happened and inform the user
            int err;
            const char *msg;
            log_warning(stderr, "zlib error\n");

            msg = gzerror(m_file, &err);
            // we're making the assumption that we never see Z_STREAM_END here
            assert(err != Z_STREAM_END);

            log_error(stderr, "error processing FASTQ file: zlib error %d (%s)\n", err, msg);
            return FILE_STREAM_ERROR;
        }
    }
    return FILE_OK;
}

// rewind
//
bool SequenceDataFile_FASTQ_gz::rewind()
{
    if (m_file == NULL || (m_file_state != FILE_OK && m_file_state != FILE_EOF))
        return false;

    gzrewind( m_file );

    m_file_state = FILE_OK;

    m_buffer_size = (uint32)m_buffer.size();
    m_buffer_pos  = (uint32)m_buffer.size();
    m_line        = 0;
    return true;
}


SequenceDataFile_FASTQ::SequenceDataFile_FASTQ(
    const char*                         read_file_name,
    const SequenceDataFile::Options&    options)
    : SequenceDataFile_FASTQ_parser(read_file_name, options)
{
    m_file = read_file_name ? fopen(read_file_name, "r") : stdin;
    if (!m_file) {
        m_file_state = FILE_OPEN_FAILED;
    } else {
        m_file_state = FILE_OK;
    }
}

SequenceDataFile_FASTQ::~SequenceDataFile_FASTQ()
{
    if (m_file != stdin)
        fclose( m_file );
}

//static float time = 0.0f;

SequenceDataFile_FASTQ_parser::FileState SequenceDataFile_FASTQ::fillBuffer(void)
{
    m_buffer_size = fread(&m_buffer[0], 1u, (uint32)m_buffer.size(), m_file);

    if (m_buffer_size <= 0)
    {
        // check for EOF separately; zlib will not always return Z_STREAM_END at EOF below
        if (feof(m_file))
            return FILE_EOF;
        else
        {
            // ask zlib what happened and inform the user
            int err;
            const char *msg;
            log_warning(stderr, "zlib error\n");

            msg = gzerror(m_file, &err);
            // we're making the assumption that we never see Z_STREAM_END here
            assert(err != Z_STREAM_END);

            log_error(stderr, "error processing FASTQ file: zlib error %d (%s)\n", err, msg);
            return FILE_STREAM_ERROR;
        }
    }
    return FILE_OK;
}

// rewind
//
bool SequenceDataFile_FASTQ::rewind()
{
    if (m_file == NULL || m_file == stdin || (m_file_state != FILE_OK && m_file_state != FILE_EOF))
        return false;

    fseek( m_file, 0u, SEEK_SET );

    m_file_state = FILE_OK;

    m_buffer_size = (uint32)m_buffer.size();
    m_buffer_pos  = (uint32)m_buffer.size();
    m_line        = 0;
    return true;
}

// constructor
//
SequenceDataOutputFile_FASTQ::SequenceDataOutputFile_FASTQ(
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
    typedef typename io::SequenceDataAccess<ALPHABET>::qual_string      qual_string;
    typedef typename io::SequenceDataAccess<ALPHABET>::name_string      name_string;

    const uint32 buffer_len =
        sequence_data.size() * 5 +          // the number of delimiters consumed for each read (i.e. '@', '\n', '+')
        sequence_data.bps()  * 2 +          // the number of characters consumed by reads and qualities
        sequence_data.name_stream_len();    // the number of characters consumed by names

    std::vector<char> buffer( buffer_len );

    #pragma omp parallel for
    for (int32 i = 0; i < (int32)sequence_data.size(); ++i)
    {
        const sequence_string read = sequence_data.get_read( i );
        const qual_string     qual = sequence_data.get_quals( i );
        const name_string     name = sequence_data.get_name( i );

        uint32 buffer_offset = i*5 + sequence_data.get_range( i ).x*2 + sequence_data.name_index()[ i ];

        buffer[ buffer_offset++ ] = '@';

        for (uint32 j = 0; j < name.size(); ++j)
            buffer[ buffer_offset++ ] = name[j];

        // setup the ASCII read
        buffer[ buffer_offset++ ] = '\n';

        to_string<ALPHABET>( read.begin(), read.size(), &buffer[0] + buffer_offset );

        buffer_offset += read.size();

        buffer[ buffer_offset++ ] = '\n';
        buffer[ buffer_offset++ ] = '+';
        buffer[ buffer_offset++ ] = '\n';

        // copy the qualities to the output buffer
        for (uint32 j = 0; j < read.size(); ++j)
            buffer[ buffer_offset++ ] = char( qual[j] );

        buffer[ buffer_offset++ ] = '\n';
    }

    if (output_file->write( buffer_len, &buffer[0] ) == 0)
        throw runtime_error( "failed writing FASTQ output file" );
}

} // anonymous namespace

// next batch
//
void SequenceDataOutputFile_FASTQ::next(const SequenceDataHost& sequence_data)
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
bool SequenceDataOutputFile_FASTQ::is_ok() { return m_file && m_file->is_valid(); }


///@} // SequenceIODetail
///@} // SequenceIO
///@} // IO

} // namespace io
} // namespace nvbio
