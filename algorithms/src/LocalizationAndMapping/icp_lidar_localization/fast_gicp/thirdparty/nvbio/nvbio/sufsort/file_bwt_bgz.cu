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

#include <nvbio/sufsort/file_bwt_bgz.h>
#include <nvbio/basic/exceptions.h>
#include <zlib/zlib.h>
#ifdef _OPENMP
#include <omp.h>
#endif

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

static const uint32 BLOCK_SIZE             = 256*1024;      // the compression unit, in bytes
static const unsigned int BGZS_MAGICNUMBER = 0x0F1F2F3F;    // just a magic number
static const unsigned int BGZS_EOS         = 0;             // a stream terminator
static const uint32 NUM_BLOCKS             = 128;           // the highest, the better for load balancing

// constructor
//
BGZFileWriter::BGZFileWriter(FILE* _file) :
    m_file(NULL), m_buffer(NUM_BLOCKS*BLOCK_SIZE), m_buffer_size(0), m_comp_buffer(NUM_BLOCKS*BLOCK_SIZE)
{
    if (_file != NULL)
        open( _file, Z_DEFAULT_COMPRESSION, Z_DEFAULT_STRATEGY );
}

// destructor
//
BGZFileWriter::~BGZFileWriter() { close(); }

// open a session
//
void BGZFileWriter::open(FILE* _file, const int level, const int strategy)
{
    m_file = _file;

    const uint32 blockSizeId = nvbio::log2( BLOCK_SIZE );

    // write the archive header
    char out_buff[7];
    *(unsigned int*)out_buff = LITTLE_ENDIAN_32(BGZS_MAGICNUMBER);   // Magic Number, in Little Endian convention
    *(out_buff+4)  = 1;                                              // Version('01')
    *(out_buff+5)  = (char)blockSizeId;
    fwrite( out_buff, 1, 8, m_file );                                // reserve 8 bytes in total

    m_level    = level;
    m_strategy = strategy;
}

// close a session
//
void BGZFileWriter::close()
{
    if (m_file == NULL)
        return;

    // encode any remaining bytes
    if (m_buffer_size)
    {
        encode_block( m_buffer_size, &m_buffer[0] );
        m_buffer_size = 0;
    }

    // write the BGZ End-Of-Stream marker
    const unsigned int eos = BGZS_EOS;
    fwrite( &eos, 1, 4, m_file );

    // invalidate the file pointer
    m_file = NULL;
}

// write a block to the output
//
void BGZFileWriter::write(uint32 n_bytes, const void* _src)
{
    // convert input to a uint8 pointer
    const uint8* src = (const uint8*)_src;

    const uint32 NB = NUM_BLOCKS;//(uint32)omp_get_num_procs();

    if (m_buffer_size)
    {
        //
        // we have some pending bytes in the buffer, let's add as much as we can and
        // eventually output a block if full
        //
        const uint32 n_needed = nvbio::min( NB*BLOCK_SIZE - m_buffer_size, n_bytes );

        // copy the given block from the source
        memcpy( &m_buffer[0] + m_buffer_size, src, n_needed );

        m_buffer_size += n_needed;
        src           += n_needed;
        n_bytes       -= n_needed;

        if (m_buffer_size == NB*BLOCK_SIZE)
        {
            encode_block( m_buffer_size, &m_buffer[0] );
            m_buffer_size = 0;
        }
    }

    //
    // at this point either we have filled the buffer and emptied it, or
    // we have terminated the source (i.e. n_bytes = 0)
    //

    for (uint32 block_begin = 0; block_begin < n_bytes; block_begin += NB*BLOCK_SIZE)
    {
        const uint32 block_end = nvbio::min( block_begin + NB*BLOCK_SIZE, n_bytes );

        if (block_end - block_begin == NB*BLOCK_SIZE)
        {
            // encode directly without buffering
            encode_block( NB*BLOCK_SIZE, src + block_begin );
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
void BGZFileWriter::encode_block(uint32 n_bytes, const uint8* src)
{
    uint32 block_sizes[NUM_BLOCKS];

    #pragma omp parallel for
    for (int block = 0; block < int( n_bytes ); block += BLOCK_SIZE)
    {
        const uint32 block_size = nvbio::min( BLOCK_SIZE, uint32( n_bytes - block ) );
        block_sizes[ block/BLOCK_SIZE ] = compress( src + block, &m_comp_buffer[0] + block, block_size );
    }

    for (int block = 0; block < int( n_bytes ); block += BLOCK_SIZE)
    {
        const uint32 block_size = nvbio::min( BLOCK_SIZE, uint32( n_bytes - block ) );
        const uint32 n_compressed = block_sizes[ block/BLOCK_SIZE ];
        if (n_compressed)
        {
            const uint32 block_header = LITTLE_ENDIAN_32( n_compressed );
            fwrite( &block_header, sizeof(uint32), 1u, m_file );
            fwrite( &m_comp_buffer[0] + block, sizeof(uint8), n_compressed, m_file );
        }
        else
        {
            const uint32 block_header = LITTLE_ENDIAN_32( block_size | 0x80000000);   // Add Uncompressed flag
            fwrite( &block_header, sizeof(uint32), 1u, m_file );
            fwrite( src + block, sizeof(uint8), block_size, m_file );
        }
    }
}

// compress a given block
//
uint32 BGZFileWriter::compress(const uint8* src, uint8* dst, const uint32 n_bytes)
{
    // initialize the gzip header
    // note that we don't actually care about most of these fields
    // gzip header for the stream
    //gz_header_s gzh;
    //gzh.text        = 0;
    //gzh.time        = 0;
    //gzh.xflags      = 0;
    //gzh.extra       = Z_NULL;
    //gzh.extra_len   = 0;
    //gzh.os          = 255;       // meaning unknown OS
    //gzh.name        = Z_NULL;
    //gzh.comment     = Z_NULL;
    //gzh.hcrc        = 0;

    // initialize the zlib stream
    z_stream stream;
    stream.zalloc   = Z_NULL;
    stream.zfree    = Z_NULL;
    stream.opaque   = Z_NULL;

    stream.next_in  = (Bytef *)src;
    stream.avail_in = n_bytes;

    stream.next_out  = (Bytef *)dst;
    stream.avail_out = n_bytes-1;

    int ret = deflateInit2(&stream,             // stream object
                       m_level,                 // compression level (0-9, default = 6)
                       Z_DEFLATED,              // compression method (no other choice...)
                       15 + 16,                 // log2 of compression window size + 16 to switch zlib to gzip format
                       9,                       // memlevel (1..9, default 8: 1 uses less memory but is slower, 9 uses more memory and is faster)
                       m_strategy);             // compression strategy, may affect compression ratio and/or performance

    if (ret != Z_OK)
        throw nvbio::runtime_error("BWTBGZWriter::compress() deflateInit2 failed");

    // compress the data
    ret = deflate(&stream, Z_FINISH);
    if (ret != Z_STREAM_END)
    {
        deflateEnd(&stream);
        return 0;           // 0 bytes compressed
    }

    // finalize the compression routine
    if(deflateEnd(&stream) != Z_OK)
        throw nvbio::runtime_error("BWTBGZWriter::compress() deflate end failed");

    return stream.total_out;
}

// constructor
//
BWTBGZWriter::BWTBGZWriter() :
    output_file(NULL),
    index_file(NULL)
{}

// destructor
//
BWTBGZWriter::~BWTBGZWriter()
{
    output_file_writer.close();
    index_file_writer.close();

    fclose( output_file );
    fclose( index_file );
}

// open
//
void BWTBGZWriter::open(const char* output_name, const char* index_name, const char* compression)
{
    log_verbose(stderr,"  opening bwt file \"%s\" (compression level: %s)\n", output_name, compression);
    log_verbose(stderr,"  opening index file \"%s\" (compression level: %s)\n", index_name, compression);
    output_file = fopen( output_name, "wb" );
    index_file  = fopen( index_name,  "wb" );

    // parse the compression string
    int level    = Z_DEFAULT_COMPRESSION;
    int strategy = Z_DEFAULT_STRATEGY;

    if (strlen( compression ) >= 1)
    {
        level = compression[0] - '0';

        if (strlen( compression ) >= 2)
        {
            switch (compression[1])
            {
            case 'r':
            case 'R':
                strategy = Z_RLE;
                break;
            case 'h':
            case 'H':
                strategy = Z_HUFFMAN_ONLY;
                break;
            case 'f':
                strategy = Z_FILTERED;
                break;
            case 'F':
                strategy = Z_FIXED;
                break;
            }
        }
    }

    output_file_writer.open( output_file, level, strategy );
    index_file_writer.open( index_file, level, strategy );
}

// write to the bwt
//
uint32 BWTBGZWriter::bwt_write(const uint32 n_bytes, const void* buffer)
{
    output_file_writer.write( n_bytes, buffer );
    return n_bytes;
}

// write to the index
//
uint32 BWTBGZWriter::index_write(const uint32 n_bytes, const void* buffer)
{
    index_file_writer.write( n_bytes, buffer );
    return n_bytes;
}

// return whether the file is in a good state
//
bool BWTBGZWriter::is_ok() const { return output_file != NULL || index_file != NULL; }

} // namespace nvbio
