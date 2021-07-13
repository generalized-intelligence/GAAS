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

namespace nvbio {

// get the next character, or 255 if EOF
//
inline
uint8 FASTQ_gzfile::get()
{
    if (m_buffer_pos >= m_buffer_size)
    {
        m_buffer_size = read( &m_buffer[0], (uint32)m_buffer.size() );
        m_buffer_pos  = 0;
    }
    return (m_buffer_pos < m_buffer_size) ? m_buffer[ m_buffer_pos++ ] : FASTQ_EOF;
};

// constructor
//
inline
FASTQ_file::FASTQ_file(const uint32 buffer_size)
  : m_file( NULL ),
    m_buffer( buffer_size ),
    m_buffer_size( 0 ),
    m_buffer_pos( 0 )
{}
    
// constructor
//
inline
FASTQ_file::FASTQ_file(const char* filename, const uint32 buffer_size)
    : m_buffer( buffer_size ), m_buffer_size( 0 ), m_buffer_pos( 0 )
{
    m_file = fopen( filename, "r" );
}

// open a new file
//
inline
void FASTQ_file::open(const char* filename)
{
    if (m_file)
        fclose( m_file );

    m_file = fopen( filename, "r" );
}

// destructor
//
inline
FASTQ_file::~FASTQ_file()
{
    if (m_file)
        fclose( m_file );
}

// get the next character, or 255 if EOF
//
inline
uint8 FASTQ_file::get()
{
    if (m_buffer_pos >= m_buffer_size)
    {
        m_buffer_size = (uint32)fread( &m_buffer[0], sizeof(uint8), m_buffer.size(), m_file );
        m_buffer_pos  = 0;
    }
    return (m_buffer_pos < m_buffer_size) ? m_buffer[ m_buffer_pos++ ] : FASTQ_EOF;
};

// constructor
//
template <typename FASTQ_stream>
FASTQ_reader<FASTQ_stream>::FASTQ_reader(FASTQ_stream& stream) :
    m_stream( &stream ),
    m_error(0),
    m_line(0)
{}

// read a batch of bp reads
//
template <typename FASTQ_stream>
template <typename Writer>
uint32 FASTQ_reader<FASTQ_stream>::read(const uint32 n_reads, Writer& writer)
{
    uint32 n = 0;
    uint8 marker;

    while (n < n_reads)
    {
        // consume spaces & newlines
        for (marker = get();
             marker == '\n' ||
             marker == ' ';
             marker = get())
        {
            if (marker == '\n')
                m_line++;
        }

        // check for EOF
        if (marker == FASTQ_EOF)
            break;

        // if the newlines didn't end in a read marker,
        // issue a parsing error...
        if (marker != '@')
        {
            m_error = 1;
            m_error_char = marker;
            return uint32(-1);
        }

        // read all the line
        m_name.erase( m_name.begin(), m_name.end() );
        for (uint8 c = get(); c != '\n'; c = get())
        {
            if (c == FASTQ_EOF)
            {
                log_error(stderr, "incomplete read at EOF!\n");
                m_name.erase(m_name.begin(), m_name.end());

                m_error = 1;
                m_error_char = FASTQ_EOF;
                return uint32(-1);
            }

            m_name.push_back(c);
        }

        m_name.push_back('\0');

        m_line++;

        // start reading the bp read
        m_read_bp.erase( m_read_bp.begin(), m_read_bp.end() );
        for (uint8 c = get(); c != '+'; c = get())
        {
            if (c == FASTQ_EOF)
            {
                log_error(stderr, "incomplete read at EOF!\n");
                m_name.erase(m_name.begin(), m_name.end());

                m_error = 1;
                m_error_char = FASTQ_EOF;
                return uint32(-1);
            }

            if (isgraph(c))
                m_read_bp.push_back( c );

            if (c == '\n')
                m_line++;
        }

        // read all the line
        for(uint8 c = get(); c != '\n'; c = get())
        {
            if (c == FASTQ_EOF)
            {
                log_error(stderr, "incomplete read at EOF!\n");
                m_name.erase(m_name.begin(), m_name.end());
                m_read_bp.erase(m_read_bp.begin(), m_read_bp.end());

                m_error = 1;
                m_error_char = FASTQ_EOF;
                return uint32(-1);
            }

        }

        m_line++;

        // start reading the quality read
        m_read_q.erase( m_read_q.begin(), m_read_q.end() );
        for (uint8 c = get(); c != '\n'; c = get())
        {
            if (c == FASTQ_EOF)
            {
                log_error(stderr, "incomplete read at EOF!\n");
                m_name.erase(m_name.begin(), m_name.end());
                m_read_bp.erase(m_read_bp.begin(), m_read_bp.end());

                m_error = 1;
                m_error_char = FASTQ_EOF;
                return uint32(-1);
            }

            m_read_q.push_back( c );
        }

        m_line++;

        writer.push_back(
            uint32( m_read_bp.size() ),
            &m_name[0],
            &m_read_bp[0],
            &m_read_q[0] );

        ++n;
    }
    return n;
}

// error string
//
template <typename FASTQ_stream>
void FASTQ_reader<FASTQ_stream>::error_string(char* error)
{
    if (m_error == 1)
        sprintf(error, "line %u, expected '@', got '%c'", m_line, m_error_char);
    else
        sprintf(error, "line %u, expected '+', got '%c'", m_line, m_error_char);
}

} // namespace nvbio
