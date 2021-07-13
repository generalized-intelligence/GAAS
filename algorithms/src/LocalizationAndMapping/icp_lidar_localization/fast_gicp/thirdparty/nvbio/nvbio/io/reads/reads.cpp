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

#include <nvbio/io/reads/reads.h>
#include <nvbio/io/reads/reads_priv.h>
#include <nvbio/io/reads/reads_fastq.h>
#include <nvbio/io/reads/reads_txt.h>
#include <nvbio/io/reads/sam.h>
#include <nvbio/io/reads/bam.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/timer.h>
#include <cuda_runtime.h>

#include <string.h>

namespace nvbio {
namespace io {

// factory method to open a read file, tries to detect file type based on file name
ReadDataStream *open_read_file(const char*           read_file_name,
                               const QualityEncoding qualities,
                               const uint32          max_reads,
                               const uint32          truncate_read_len,
                               const ReadEncoding    flags)
{
    // parse out file extension; look for .fastq.gz, .fastq suffixes
    uint32 len = uint32( strlen(read_file_name) );
    bool is_gzipped = false;

    // do we have a .gz suffix?
    if (len >= strlen(".gz"))
    {
        if (strcmp(&read_file_name[len - strlen(".gz")], ".gz") == 0)
        {
            is_gzipped = true;
            len = uint32(len - strlen(".gz"));
        }
    }

    // check for fastq suffix
    if (len >= strlen(".fastq"))
    {
        if (strncmp(&read_file_name[len - strlen(".fastq")], ".fastq", strlen(".fastq")) == 0)
        {
            return new ReadDataFile_FASTQ_gz(read_file_name,
                                             qualities,
                                             max_reads,
                                             truncate_read_len,
                                             flags);
        }
    }

    // check for fastq suffix
    if (len >= strlen(".fq"))
    {
        if (strncmp(&read_file_name[len - strlen(".fq")], ".fq", strlen(".fq")) == 0)
        {
            return new ReadDataFile_FASTQ_gz(read_file_name,
                                             qualities,
                                             max_reads,
                                             truncate_read_len,
                                             flags);
        }
    }

    // check for txt suffix
    if (len >= strlen(".txt"))
    {
        if (strncmp(&read_file_name[len - strlen(".txt")], ".txt", strlen(".txt")) == 0)
        {
            return new ReadDataFile_TXT_gz(read_file_name,
                                           qualities,
                                           max_reads,
                                           truncate_read_len,
                                           flags);
        }
    }

    // check for sam suffix
    if (len >= strlen(".sam"))
    {
        if (strncmp(&read_file_name[len - strlen(".sam")], ".sam", strlen(".sam")) == 0)
        {
            ReadDataFile_SAM *ret;

            ret = new ReadDataFile_SAM(read_file_name,
                                       max_reads,
                                       truncate_read_len,
                                       flags);

            if (ret->init() == false)
            {
                delete ret;
                return NULL;
            }

            return ret;
        }
    }

    // check for bam suffix
    if (len >= strlen(".bam"))
    {
        if (strncmp(&read_file_name[len - strlen(".bam")], ".bam", strlen(".bam")) == 0)
        {
            ReadDataFile_BAM *ret;

            ret = new ReadDataFile_BAM(read_file_name,
                                       max_reads,
                                       truncate_read_len,
                                       flags);

            if (ret->init() == false)
            {
                delete ret;
                return NULL;
            }

            return ret;
        }
    }

    // we don't actually know what this is; guess fastq
    log_warning(stderr, "could not determine file type for %s; guessing %sfastq\n", read_file_name, is_gzipped ? "compressed " : "");
    return new ReadDataFile_FASTQ_gz(read_file_name,
                                     qualities,
                                     max_reads,
                                     truncate_read_len,
                                     flags);
}

namespace { // anonymous

// converts ASCII characters for amino-acids into
// a 5 letter alphabet for { A, C, G, T, N }.
inline unsigned char nst_nt4_encode(unsigned char c)
{
    static unsigned char nst_nt4_table[256] = {
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 5 /*'-'*/, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 0, 4, 1,  4, 4, 4, 2,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  3, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 0, 4, 1,  4, 4, 4, 2,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  3, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4
    };

    return nst_nt4_table[c];
}

// convert a quality value in one of the supported encodings to Phred
template <QualityEncoding encoding>
inline unsigned char convert_to_phred_quality(const uint8 q)
{
    // this table maps Solexa quality values to Phred scale
    static unsigned char s_solexa_to_phred[] = {
        0, 1, 1, 1, 1, 1, 1, 2, 2, 3,
        3, 4, 4, 5, 5, 6, 7, 8, 9, 10,
        10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
        20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
        40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
        50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
        60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
        70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
        80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
        90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
        100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
        110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
        120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
        130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
        140, 141, 142, 143, 144, 145, 146, 147, 148, 149,
        150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
        160, 161, 162, 163, 164, 165, 166, 167, 168, 169,
        170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
        180, 181, 182, 183, 184, 185, 186, 187, 188, 189,
        190, 191, 192, 193, 194, 195, 196, 197, 198, 199,
        200, 201, 202, 203, 204, 205, 206, 207, 208, 209,
        210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
        220, 221, 222, 223, 224, 225, 226, 227, 228, 229,
        230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
        240, 241, 242, 243, 244, 245, 246, 247, 248, 249,
        250, 251, 252, 253, 254, 255
    };

    switch(encoding)
    {
    case Phred:
        return q;

    case Phred33:
        return q - 33;

    case Phred64:
        return q - 64;

    case Solexa:
        return s_solexa_to_phred[q];
    }

    // gcc is dumb
    return q;
}

// complement a base pair
inline unsigned char complement_bp(unsigned char bp)
{
    switch(bp)
    {
    case 'A':
        return 'T';

    case 'a':
        return 't';

    case 'T':
        return 'A';

    case 't':
        return 'a';

    case 'G':
        return 'C';

    case 'g':
        return 'c';

    case 'C':
        return 'G';

    case 'c':
        return 'g';

    default:
        return bp;
    }
}

} // anonymous namespace

ReadDataRAM::ReadDataRAM()
  : ReadData()
{
    // old mechanism employed before introducing reserve()
    //m_read_vec.reserve( 8*1024*1024 );
    //m_qual_vec.reserve( 64*1024*1024 );

    m_read_index_vec.reserve( 16*1024 );
    m_read_index_vec.resize( 1u );
    m_read_index_vec[0] = 0u;

    m_name_index_vec.reserve( 16*1024 );
    m_name_index_vec.resize( 1u );
    m_name_index_vec[0] = 0u;
}

#if defined(PRESIZED_VECTORS)
  #define PRESIZE_VECTORS(vec, x) vec.resize(x)
  #define ADJUST_VECTORS(vec, x)  vec.resize(x)
  #define RESIZE_VECTORS(vec, x)  if (vec.size() < x) vec.resize(x)
#else
  #define PRESIZE_VECTORS(vec, x)
  #define ADJUST_VECTORS(vec, x)
  #define RESIZE_VECTORS(vec, x)  vec.resize(x)
#endif

// reserve enough storage for a given number of reads and bps
//
void ReadDataRAM::reserve(const uint32 n_reads, const uint32 n_bps)
{
    // a default read id length used to reserve enough space upfront and avoid frequent allocations
    const uint32 AVG_NAME_LENGTH = 250;

    const uint32 bps_per_word = 32u / ReadData::READ_BITS;

    m_read_vec.reserve( n_bps / bps_per_word );
    m_qual_vec.reserve( n_bps );
    m_read_index_vec.reserve( n_reads+1 );
    m_name_index_vec.reserve( AVG_NAME_LENGTH * n_reads );
    m_name_index_vec.reserve( n_reads+1 );

    // pre-size the vectors
    PRESIZE_VECTORS( m_read_vec, n_bps / bps_per_word );
    PRESIZE_VECTORS( m_qual_vec, n_bps );
}

// signals that the batch is complete
void ReadDataRAM::end_batch(void)
{
    assert(m_read_stream_words == (m_read_stream_len + 7) / 8);

    m_avg_read_len = (uint32) ceilf(float(m_read_stream_len) / float(m_n_reads));

    // adjust the vector sizes
    ADJUST_VECTORS( m_read_vec, m_read_stream_words );
    ADJUST_VECTORS( m_qual_vec, m_read_stream_len );

    // set the stream pointers
    m_read_stream = nvbio::plain_view( m_read_vec );
    m_qual_stream = nvbio::plain_view( m_qual_vec );
    m_read_index  = nvbio::plain_view( m_read_index_vec );

    m_name_stream = nvbio::plain_view( m_name_vec );
    m_name_index  = nvbio::plain_view( m_name_index_vec );
}

// a small read class supporting REVERSE | COMPLEMENT operations
//
template <ReadDataRAM::StrandOp FLAGS>
struct read_string
{
    // constructor
    read_string(const uint32 len, const uint8* read, const uint8* qual) : m_len(len), m_read(read), m_qual(qual) {}

    // string length
    uint32 length() const { return m_len; }

    // indexing operator
    uint8 operator[] (const uint32 i) const
    {
        const uint32 index = (FLAGS & ReadDataRAM::REVERSE_OP) ? m_len - i - 1u : i;

        // fetch the bp
        const uint8 bp = nst_nt4_encode( m_read[index] );

        if (FLAGS & ReadDataRAM::COMPLEMENT_OP)
            return bp < 4u ? 3u - bp : 4u;

        return bp;
    }

    // quality operator
    uint8 quality(const uint32 i) const
    {
        const uint32 index = (FLAGS & REVERSE) ? m_len - i - 1u : i;

        return m_qual[index];
    }

    const uint32 m_len;
    const uint8* m_read;
    const uint8* m_qual;
};

// encode a read according to a compile-time quality-encoding
//
template <QualityEncoding quality_encoding, typename read_type>
void encode(
    const read_type                         read,
    ReadData::read_stream_type::iterator    stream,
    char*                                   qual_stream)
{
  #if 1

    // use the custom PackedStream assign() method
    assign( read.length(), read, stream  );

    // naive serial implementation
    for (uint32 i = 0; i < read.length(); i++)
        qual_stream[i] = convert_to_phred_quality<quality_encoding>(read.quality(i));

  #else
    // naive serial implementation
    for (uint32 i = 0; i < read.length(); i++)
    {
        stream[i] = read[i];
        qual_stream[i] = convert_to_phred_quality<quality_encoding>(read.quality(i));
    }
  #endif
}

// encode a read according to some compile-time flags and run-time quality-encoding
//
template <typename read_type>
void encode(
    const QualityEncoding                   quality_encoding,
    const read_type                         read,
    ReadData::read_stream_type::iterator    stream,
    char*                                   qual_stream)
{
    switch (quality_encoding)
    {
    case Phred:
        encode<Phred>( read, stream, qual_stream );
        break;
    case Phred33:
        encode<Phred33>( read, stream, qual_stream );
        break;
    case Phred64:
        encode<Phred64>( read, stream, qual_stream );
        break;
    case Solexa:
        encode<Solexa>( read, stream, qual_stream );
        break;
    }
}

// encode a read according to some given run-time flags and quality-encoding
//
void encode(
    const ReadDataRAM::StrandOp             conversion_flags,
    const QualityEncoding                   quality_encoding,
    const uint32                            read_len,
    const uint8*                            read,
    const uint8*                            quality,
    ReadData::read_stream_type::iterator    stream,
    char*                                   qual_stream)
{

    const read_string<ReadDataRAM::REVERSE_OP>              r_read( read_len, read, quality );
    const read_string<ReadDataRAM::REVERSE_COMPLEMENT_OP>   rc_read( read_len, read, quality );
    const read_string<ReadDataRAM::COMPLEMENT_OP>           fc_read( read_len, read, quality );
    const read_string<ReadDataRAM::NO_OP>                   f_read( read_len, read, quality );

    if (conversion_flags & ReadDataRAM::REVERSE_OP)
    {
        if (conversion_flags & ReadDataRAM::COMPLEMENT_OP)
            encode( quality_encoding, rc_read, stream, qual_stream );
        else
            encode( quality_encoding, r_read,  stream, qual_stream );
    }
    else
    {
        if (conversion_flags & ReadDataRAM::COMPLEMENT_OP)
            encode( quality_encoding, fc_read, stream, qual_stream );
        else
            encode( quality_encoding, f_read,  stream, qual_stream );
    }
}

// add a read to this batch
void ReadDataRAM::push_back(uint32 read_len,
                            const char *name,
                            const uint8* read,
                            const uint8* quality,
                            const QualityEncoding quality_encoding,
                            const uint32 truncate_read_len,
                            const StrandOp conversion_flags)
{
    // truncate read
    // xxx: should we do this silently?
    read_len = nvbio::min(read_len, truncate_read_len);

    assert(read_len);

    // resize the reads & quality buffers
    {
        static const uint32 bps_per_word = 32u / ReadData::READ_BITS;
        const uint32 stream_len = m_read_stream_len + read_len;
        const uint32 words      = (stream_len + bps_per_word - 1) / bps_per_word;

        RESIZE_VECTORS( m_read_vec, words );
        RESIZE_VECTORS( m_qual_vec, stream_len );

        m_read_stream_words = words;
    }

    // encode the read data
    ReadData::read_stream_type stream(&m_read_vec[0]);
    encode(
        conversion_flags,
        quality_encoding,
        read_len,
        read,
        quality,
        stream.begin() + m_read_stream_len,
        &m_qual_vec[0] + m_read_stream_len );

    // update read and bp counts
    m_n_reads++;
    m_read_stream_len += read_len;
    m_read_index_vec.push_back(m_read_stream_len);

    m_min_read_len = nvbio::min(m_min_read_len, read_len);
    m_max_read_len = nvbio::max(m_max_read_len, read_len);

    // store the read name
    const uint32 name_len = uint32(strlen(name));
    const uint32 name_offset = m_name_stream_len;

    m_name_vec.resize(name_offset + name_len + 1);
    //strcpy(&m_name_vec[name_offset], name);
    memcpy(&m_name_vec[name_offset],name,name_len + 1);

    m_name_stream_len += name_len + 1;
    m_name_index_vec.push_back(m_name_stream_len);
}

// utility function to alloc and copy a vector in device memory
template <typename T>
static void cudaAllocAndCopyVector(T*& dst, const T* src, const uint32 words, uint64& allocated)
{
    const uint32 words4 = 4u * ((words + 3u) / 4u);
    if (src)
    {
        cudaMalloc( &dst, sizeof(T) * words4 );
        if (dst == NULL)
            throw std::bad_alloc(WINONLY("ReadDataDevice: not enough device memory"));

        cudaMemcpy( dst, src, sizeof(T) * words, cudaMemcpyHostToDevice );

        allocated += words4 * sizeof(T);
    }
    else
        dst = NULL;
}

ReadDataDevice::ReadDataDevice(const ReadData& host_data, const uint32 flags)
  : ReadData(),
    m_allocated( 0 )
{
    m_name_stream_len   = 0;
    m_name_stream       = NULL;
    m_name_index        = NULL;
    m_qual_stream       = NULL;
    m_read_stream       = NULL;
    m_read_index        = NULL;

    m_n_reads           = host_data.m_n_reads;
    m_read_stream_len   = 0;
    m_read_stream_words = 0;

    m_min_read_len = host_data.m_min_read_len;
    m_max_read_len = host_data.m_max_read_len;
    m_avg_read_len = host_data.m_avg_read_len;

    if (flags & READS)
    {
        m_read_stream_len   = host_data.m_read_stream_len;
        m_read_stream_words = host_data.m_read_stream_words;

        cudaAllocAndCopyVector( m_read_stream, host_data.m_read_stream, m_read_stream_words, m_allocated );
        cudaAllocAndCopyVector( m_read_index,  host_data.m_read_index,  m_n_reads+1,         m_allocated );
    }
    if (flags & QUALS)
        cudaAllocAndCopyVector( m_qual_stream, host_data.m_qual_stream, m_read_stream_len, m_allocated );
}

ReadDataDevice::~ReadDataDevice()
{
    if (m_read_stream)
        cudaFree( m_read_stream );

    if (m_read_index)
        cudaFree( m_read_index );

    if (m_qual_stream)
        cudaFree( m_qual_stream );
}

// grab the next batch of reads into a host memory buffer
ReadData *ReadDataFile::next(const uint32 batch_size, const uint32 batch_bps)
{
    const uint32 reads_to_load = std::min(m_max_reads - m_loaded, batch_size);

    if (!is_ok() || reads_to_load == 0)
        return NULL;

    // a default average read length used to reserve enough space
    const uint32 AVG_READ_LENGTH = 100;

    ReadDataRAM *reads = new ReadDataRAM();
    reads->reserve(
        batch_size,
        batch_bps == uint32(-1) ? batch_size * AVG_READ_LENGTH : batch_bps ); // try to use a default read length

    while (reads->size() < reads_to_load &&
           reads->bps()  < batch_bps)
    {
        // load 100 at a time if possible
        const uint32 chunk_reads = nvbio::min(reads_to_load - reads->size(), uint32(100));
        const uint32 chunk_bps   = batch_bps - reads->bps();

        const int n = nextChunk(reads, chunk_reads, chunk_bps);
        assert(n <= (int) chunk_reads);
        if (n == 0)
            break;

        assert(reads->size() <= reads_to_load);
    }

    if (reads->size() == 0)
    {
        delete reads;
        return NULL;
    }

    m_loaded += reads->size();

    reads->end_batch();

    return reads;
}

} // namespace io
} // namespace nvbio
