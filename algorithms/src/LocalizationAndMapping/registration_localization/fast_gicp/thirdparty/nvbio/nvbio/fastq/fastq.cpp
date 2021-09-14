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

#include <nvbio/fastq/fastq.h>
#include <zlib/zlib.h>


namespace nvbio {

struct FASTQ_gzfile::Impl
{
    Impl(const char* filename, const uint32 buffer_size)
    {
        m_file = gzopen( filename, "r" );

        if (m_file)
            gzbuffer( m_file, buffer_size );
    }

    ~Impl()
    {
        if (m_file)
            gzclose( m_file );
    }

    gzFile m_file;
};

// constructor
//
FASTQ_gzfile::FASTQ_gzfile(const uint32 buffer_size) :
    m_buffer( buffer_size ),
    m_buffer_size( 0 ),
    m_buffer_pos( 0 )
{
    m_impl = NULL;
}

// constructor
//
FASTQ_gzfile::FASTQ_gzfile(const char* filename, const uint32 buffer_size) :
    m_buffer( buffer_size ),
    m_buffer_size( 0 ),
    m_buffer_pos( 0 )
{
    m_impl = new Impl( filename, buffer_size );
}

// destructor
//
FASTQ_gzfile::~FASTQ_gzfile() { delete m_impl; }

// open a new file
//
void FASTQ_gzfile::open(const char* filename)
{
    if (m_impl)
        delete m_impl;

    m_impl = new Impl( filename, uint32( m_buffer.size() ) );
}

// return whether the file is valid
//
bool FASTQ_gzfile::valid() const
{
    return m_impl->m_file != NULL;
}

// read a buffer
//
uint32 FASTQ_gzfile::read(uint8* buffer, const uint32 n)
{
    return gzread( m_impl->m_file, buffer, n );
}

} // namespace nvbio
