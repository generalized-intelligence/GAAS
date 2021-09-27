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

#include <nvbio/sufsort/file_bwt_lz4.h>
#include <lz4/lz4.h>
#include <lz4/lz4hc.h>

namespace nvbio {

//**************************************
// Compiler-specific functions
//**************************************
#if defined(__GNUC__)
#define GCC_VERSION (__GNUC__ * 100 + __GNUC_MINOR__)
#endif

#if defined(_MSC_VER)    // Visual Studio
#  define swap32 _byteswap_ulong
#elif GCC_VERSION >= 403
#  define swap32 __builtin_bswap32
#else
  static inline unsigned int swap32(unsigned int x)
  {
    return ((x << 24) & 0xff000000 ) |
           ((x <<  8) & 0x00ff0000 ) |
           ((x >>  8) & 0x0000ff00 ) |
           ((x >> 24) & 0x000000ff );
  }
#endif

//**************************************
// Architecture Macros
//**************************************
static const int one = 1;
#define CPU_LITTLE_ENDIAN   (*(char*)(&one))
#define CPU_BIG_ENDIAN      (!CPU_LITTLE_ENDIAN)
#define LITTLE_ENDIAN_32(i) (CPU_LITTLE_ENDIAN?(i):swap32(i))

static const int _1BIT  = 0x01;
static const int _2BITS = 0x03;
static const int _3BITS = 0x07;

static const uint32 BLOCK_SIZE             = 4*1024*1024;
static const unsigned int LZ4S_MAGICNUMBER = 0x184D2204;
static const unsigned int LZ4S_EOS         = 0;

// constructor
//
LZ4FileWriter::LZ4FileWriter(FILE* _file) :
    m_file(NULL), m_buffer(BLOCK_SIZE), m_buffer_size(0), m_comp_buffer(BLOCK_SIZE)
{
    if (_file != NULL)
        open( _file );
}

// destructor
//
LZ4FileWriter::~LZ4FileWriter() { close(); }

// open a session
//
void LZ4FileWriter::open(FILE* _file)
{
    m_file = _file;

    const int blockIndependence = 1;
    const int blockChecksum     = 0;
    const int streamChecksum    = 0;
    const int blockSizeId       = 7; // 64 KB   (4 = 64 KB, 5 = 256 KB, 6 = 1 MB, 7 = 4 MB)

    unsigned int checkbits = 0;

    // write the archive header
    char out_buff[7];
    *(unsigned int*)out_buff = LITTLE_ENDIAN_32(LZ4S_MAGICNUMBER);   // Magic Number, in Little Endian convention
    *(out_buff+4)  = (1 & _2BITS) << 6 ;                             // Version('01')
    *(out_buff+4) |= (blockIndependence & _1BIT) << 5;
    *(out_buff+4) |= (blockChecksum & _1BIT) << 4;
    *(out_buff+4) |= (streamChecksum & _1BIT) << 2;
    *(out_buff+5)  = (char)((blockSizeId & _3BITS) << 4);
    //checkbits = XXH32((out_buff+4), 2, LZ4S_CHECKSUM_SEED);
    //checkbits = LZ4S_GetCheckBits_FromXXH(checkbits);
    *(out_buff+6)  = (unsigned char)checkbits;
    fwrite( out_buff, 1, 7, m_file );
}

// close a session
//
void LZ4FileWriter::close()
{
    if (m_file == NULL)
        return;

    // encode any remaining bytes
    if (m_buffer_size)
    {
        encode_block( m_buffer_size, &m_buffer[0] );
        m_buffer_size = 0;
    }

    // write the LZ4 End-Of-Stream marker
    const unsigned int eos = LZ4S_EOS;
    fwrite( &eos, 1, 4, m_file );

    // invalidate the file pointer
    m_file = NULL;
}

// write a block to the output
//
void LZ4FileWriter::write(uint32 n_bytes, const void* _src)
{
    // convert input to a uint8 pointer
    const uint8* src = (const uint8*)_src;

    if (m_buffer_size)
    {
        //
        // we have some pending bytes in the buffer, let's add as much as we can and
        // eventually output a block if full
        //
        const uint32 n_needed = nvbio::min( BLOCK_SIZE - m_buffer_size, n_bytes );

        // copy the given block from the source
        memcpy( &m_buffer[0] + m_buffer_size, src, n_needed );

        m_buffer_size += n_needed;
        src           += n_needed;
        n_bytes       -= n_needed;

        if (m_buffer_size == BLOCK_SIZE)
        {
            encode_block( m_buffer_size, &m_buffer[0] );
            m_buffer_size = 0;
        }
    }

    //
    // at this point either we have filled the buffer and emptied it, or
    // we have terminated the source (i.e. n_bytes = 0)
    //

    for (uint32 block_begin = 0; block_begin < n_bytes; block_begin += BLOCK_SIZE)
    {
        const uint32 block_end = nvbio::min( block_begin + BLOCK_SIZE, n_bytes );

        if (block_end - block_begin == BLOCK_SIZE)
        {
            // encode directly without buffering
            encode_block( BLOCK_SIZE, src + block_begin );
        }
        else
        {
            // buffer the remaining bytes - happens only at the last iteration
            memcpy( &m_buffer[0], src + block_begin, block_end - block_begin );
            m_buffer_size += block_end - block_begin;
        }
    }
}

// encode a given block and write it to the output
//
void LZ4FileWriter::encode_block(uint32 n_bytes, const uint8* src)
{
    const uint32 n_compressed = (uint32)LZ4_compressHC_limitedOutput( (const char*)src, (char*)&m_comp_buffer[0], n_bytes, n_bytes-1 );

    if (n_compressed)
    {
        const uint32 block_header = LITTLE_ENDIAN_32( n_compressed );
        fwrite( &block_header, sizeof(uint32), 1u, m_file );
        fwrite( &m_comp_buffer[0], sizeof(uint8), n_compressed, m_file );
    }
    else
    {
        const uint32 block_header = LITTLE_ENDIAN_32( n_bytes | 0x80000000);   // Add Uncompressed flag
        fwrite( &block_header, sizeof(uint32), 1u, m_file );
        fwrite( src, sizeof(uint8), n_bytes, m_file );
    }
}


// constructor
//
BWTLZ4Writer::BWTLZ4Writer() :
    output_file(NULL),
    index_file(NULL)
{}

// destructor
//
BWTLZ4Writer::~BWTLZ4Writer()
{
    output_file_writer.close();
    index_file_writer.close();

    fclose( output_file );
    fclose( index_file );
}

// open
//
void BWTLZ4Writer::open(const char* output_name, const char* index_name, const char* compression)
{
    log_verbose(stderr,"  opening bwt file \"%s\" (compression level: %s)\n", output_name, compression);
    log_verbose(stderr,"  opening index file \"%s\" (compression level: %s)\n", index_name, compression);
    output_file = fopen( output_name, "wb" );
    index_file  = fopen( index_name,  "wb" );

    output_file_writer.open( output_file );
    index_file_writer.open( index_file );
}

// write to the bwt
//
uint32 BWTLZ4Writer::bwt_write(const uint32 n_bytes, const void* buffer)
{
    output_file_writer.write( n_bytes, buffer );
    return n_bytes;
}

// write to the index
//
uint32 BWTLZ4Writer::index_write(const uint32 n_bytes, const void* buffer)
{
    index_file_writer.write( n_bytes, buffer );
    return n_bytes;
}

// return whether the file is in a good state
//
bool BWTLZ4Writer::is_ok() const { return output_file != NULL || index_file != NULL; }

} // namespace nvbio
