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

#include <nvbio/io/output_stream.h>
#include <nvbio/basic/console.h>
#include <zlib/zlib.h>
#include <lz4/lz4frame.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

namespace nvbio {

GZOutputFile::GZOutputFile(const char* name, const char* comp)
{
    char options_string[16];
    sprintf( options_string, "w%s", comp);

    m_file = gzopen( name, options_string );
}
GZOutputFile::~GZOutputFile()
{
    if (m_file)
        gzclose( m_file );
}

uint32 GZOutputFile::write(const uint32 bytes, const void* buffer)
{
    if (bytes == 0)
        return 0;

    const int r = gzwrite( m_file, buffer, bytes );
    if (r <= 0)
    {
        gzclose( m_file );
        m_file = NULL;
        return 0;
    }
    return uint32(r);
}

LZ4OutputFile::LZ4OutputFile(const char* name, const char* comp)
{
    m_file = (FILE*)fopen( name, "wb" );
    if (m_file == NULL)
        return;

    m_buffer.resize( 1024*1024 );

    // create a compression context
    LZ4F_createCompressionContext( &m_context, LZ4F_VERSION );

    LZ4F_preferences_t* preferences = (LZ4F_preferences_t*)calloc( sizeof(LZ4F_preferences_t), 1u );
        // TODO: handle compression modes

    const size_t compressed_bytes = LZ4F_compressBegin( m_context, &m_buffer[0], m_buffer.size(), preferences );

    if (compressed_bytes == 0)
        return;

    // write down
    if (fwrite( &m_buffer[0], 1u, compressed_bytes, (FILE*)m_file ) < compressed_bytes)
    {
        // an error has occurred, shutdown the file
        fclose( (FILE*)m_file ); m_file = NULL;

        LZ4F_freeCompressionContext( m_context ); m_context = NULL;
    }
}
LZ4OutputFile::~LZ4OutputFile()
{
    // flush any remaining data
    if (m_context && m_file)
    {
        const size_t compressed_bytes = LZ4F_compressEnd( m_context, &m_buffer[0], m_buffer.size(), NULL );

        fwrite( &m_buffer[0], 1u, compressed_bytes, (FILE*)m_file );
    }

    if (m_file)
        fclose( (FILE*)m_file );

    if (m_context)
        LZ4F_freeCompressionContext( m_context );
}

uint32 LZ4OutputFile::write(const uint32 bytes, const void* buffer)
{
    if (bytes == 0)
        return 0;

    const size_t dst_max = LZ4F_compressBound( bytes, NULL );
    if (m_buffer.size() <= dst_max)
        m_buffer.resize( dst_max );

    // compress in-memory
    const size_t compressed_bytes = LZ4F_compressUpdate( m_context, &m_buffer[0], m_buffer.size(), buffer, bytes, NULL );

    // check whether the compressor has actually produced any output
    if (compressed_bytes == 0)
        return bytes;

    // write down
    if (fwrite( &m_buffer[0], 1u, compressed_bytes, (FILE*)m_file ) < compressed_bytes)
    {
        // an error has occurred, shutdown the file
        fclose( (FILE*)m_file ); m_file = NULL;

        LZ4F_freeCompressionContext( m_context ); m_context = NULL;
    }
    return bytes;
}

// output file factory method
//
OutputStream* open_output_file(const char* file_name, const char* compressor, const char* options)
{
    if (compressor == NULL || strcmp( compressor, "" ) == 0)
        return new GZOutputFile( file_name, "T" );
    else if (strcmp( compressor, "gzip" ) == 0 || strcmp( compressor, "gz" ) == 0)
        return new GZOutputFile( file_name, options );
    else if (strcmp( compressor, "lz4" ) == 0)
        return new LZ4OutputFile( file_name, options );

    log_warning(stderr, "unknown output file compressor \"%s\"\n", compressor );
    return NULL;
}

} // namespace nvbio
