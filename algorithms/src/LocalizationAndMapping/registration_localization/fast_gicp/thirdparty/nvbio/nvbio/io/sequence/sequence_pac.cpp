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

#include <nvbio/io/sequence/sequence.h>
#include <nvbio/io/sequence/sequence_mmap.h>
#include <nvbio/basic/bnt.h>
#include <nvbio/basic/console.h>
#include <stdio.h>
#include <stdlib.h>

namespace nvbio {
namespace io {

namespace { // anonymous namespace

template <typename T>
uint64 block_fread(T* dst, const uint64 n, FILE* file)
{
#if defined(WIN32)
    // use blocked reads on Windows, which seems to otherwise become less responsive while reading.
    const uint64 BATCH_SIZE = 16*1024*1024;
    for (uint64 batch_begin = 0; batch_begin < n; batch_begin += BATCH_SIZE)
    {
        const uint64 batch_end = nvbio::min( batch_begin + BATCH_SIZE, n );
        const uint64 batch_size = batch_end - batch_begin;

        const uint64 n_words = fread( dst + batch_begin, sizeof(T), batch_size, file );
        if (n_words != batch_size)
            return batch_begin + n_words;
    }
    return n;
#else
    return fread( dst, sizeof(T), n, file );
#endif
}

template <Alphabet         ALPHABET>
bool load_pac(
    const char*     prefix,
    uint32*         stream,
    uint32          seq_length,
    uint32          seq_words)
{
    std::string wpac_file_name = std::string( prefix ) + ".wpac";
    std::string  pac_file_name = std::string( prefix ) + ".pac";
    const char*  file_name     = wpac_file_name.c_str();
    bool         wpac          = true;

    FILE* file = fopen( wpac_file_name.c_str(), "rb" );
    if (file == NULL)
    {
        file        = fopen( pac_file_name.c_str(), "rb" );
        file_name   = pac_file_name.c_str();
        wpac        = false;
    }

    if (file == NULL)
    {
        log_warning(stderr, "unable to open %s.[w]pac\n", prefix);
        return false;
    }

    typedef SequenceDataTraits<ALPHABET> sequence_traits;

    typedef PackedStream<
        uint32*,uint8,
        sequence_traits::SEQUENCE_BITS,
        sequence_traits::SEQUENCE_BIG_ENDIAN>   output_stream_type;

    if (wpac)
    {
        // read a .wpac file
        uint64 field;
        if (!fread( &field, sizeof(field), 1, file ))
        {
            log_error(stderr, "failed reading %s\n", file_name);
            return false;
        }

        const uint32 _seq_length = uint32(field);
        if (_seq_length != seq_length)
        {
            log_error(stderr, "mismatching sequence lengths in %s, expected: %u, found: %u\n", file_name, seq_length, _seq_length);
            return false;
        }

        if (ALPHABET == DNA && sequence_traits::SEQUENCE_BIG_ENDIAN == true)
        {
            // read the 2-bit per symbol words in the final destination
            const uint32 n_words = (uint32)block_fread( stream, seq_words, file );
            if (n_words != seq_words)
            {
                log_error(stderr, "failed reading %s\n", file_name);
                return false;
            }
        }
        else
        {
            // read the 2-bit per symbol words in a temporary array
            const uint32 pac_words = uint32( util::divide_ri( seq_length, 16u ) );

            std::vector<uint32> pac_vec( pac_words );
            uint32* pac_stream = &pac_vec[0];

            const uint32 n_words = (uint32)block_fread( pac_stream, pac_words, file );
            if (n_words != pac_words)
            {
                log_error(stderr, "failed reading %s\n", file_name);
                return false;
            }

            // build the input wpac stream
            typedef PackedStream<const uint32*,uint8,2,true> pac_stream_type;
            pac_stream_type pac( pac_stream );

            // build the output stream
            output_stream_type out( stream );

            // copy the pac stream into the output
            assign( seq_length, pac, out ); // TODO: transform pac using a DNA -> ALPHABET conversion
        }
    }
    else
    {
        // read a .pac file
        fseek( file, -1, SEEK_END );
        const uint32 packed_file_len = ftell( file );
        uint8 last_byte_len;
        if (!fread( &last_byte_len, sizeof(unsigned char), 1, file ))
        {
            log_error(stderr, "failed reading %s\n", file_name);
            return false;
        }
        const uint32 _seq_length = (packed_file_len - 1u) * 4u + last_byte_len;
        if (_seq_length != seq_length)
        {
            log_error(stderr, "mismatching sequence lengths in %s, expected: %u, found: %u\n", file_name, seq_length, _seq_length);
            return false;
        }

        fseek( file, 0, SEEK_SET );

        const uint32 seq_bytes = uint32( util::divide_ri( seq_length, 4u ) );

        std::vector<uint8> pac_vec( seq_bytes );
        uint8* pac_stream = &pac_vec[0];

        const uint64 n_bytes = block_fread( pac_stream, seq_bytes, file );
        if (n_bytes != seq_bytes)
        {
            log_error(stderr, "failed reading %s\n", file_name);
            return false;
        }

        // build the input pac stream
        typedef PackedStream<const uint8*,uint8,2,true> pac_stream_type;
        pac_stream_type pac( pac_stream );

        // build the output stream
        output_stream_type out( stream );

        // copy the pac stream into the output
        assign( seq_length, pac, out ); // TODO: transform pac using a DNA -> ALPHABET conversion
    }
    fclose( file );
    return true;
}

struct BNTLoader : public nvbio::BNTSeqLoader
{
    typedef nvbio::vector<host_tag,uint32> IndexVector;
    typedef nvbio::vector<host_tag,char>   StringVector;

    BNTLoader(
        IndexVector&  index_vec,
        IndexVector&  name_index_vec,
        StringVector& name_vec) :
        m_name_vec( &name_vec ),
        m_name_index_vec( &name_index_vec ),
        m_index_vec( &index_vec ) {}

    void set_info(const nvbio::BNTInfo info)
    {
        m_info.m_n_seqs              = uint32( info.n_seqs );
        m_info.m_sequence_stream_len = uint32( info.l_pac );
        m_info.m_avg_sequence_len    = uint32( info.l_pac / info.n_seqs );
        m_info.m_min_sequence_len    = uint32(-1);
        m_info.m_max_sequence_len    = 0u;
    }
    void read_ann(const nvbio::BNTAnnInfo& info, nvbio::BNTAnnData& data)
    {
        // keep track of the current name offset
        const uint32 name_offset = (uint32)m_name_vec->size();

        // copy the name of this sequence into the output vector
        m_name_vec->resize( name_offset + info.name.length() + 1u );
        strcpy( &m_name_vec->front() + name_offset, info.name.c_str() );

        // push back the name and sequence offsets
        m_name_index_vec->push_back( uint32( m_name_vec->size() ) );
        m_index_vec->push_back( uint32( data.offset + data.len ) );

        // keep sequence stats
        m_info.m_min_sequence_len = nvbio::min( m_info.m_min_sequence_len, uint32( data.len ) );
        m_info.m_max_sequence_len = nvbio::max( m_info.m_max_sequence_len, uint32( data.len ) );
    }

    void read_amb(const nvbio::BNTAmb& amb) {}

    SequenceDataInfo m_info;
    StringVector*    m_name_vec;
    IndexVector*     m_name_index_vec;
    IndexVector*     m_index_vec;
};

} // anonymous namespace

// check whether the file name points to a pac archive
//
bool is_pac_archive(const char* sequence_file_name)
{
    const std::string ann  = std::string(sequence_file_name) + ".ann";
    const std::string pac  = std::string(sequence_file_name) + ".pac";
    const std::string wpac = std::string(sequence_file_name) + ".wpac";
    FILE* ann_file  = fopen( ann.c_str(), "rb" );
    FILE* pac_file  = fopen( pac.c_str(), "rb" );
    FILE* wpac_file = fopen( wpac.c_str(), "rb" );

    bool ann_ok = ann_file != NULL;
    bool seq_ok = (pac_file != NULL || wpac_file != NULL);

    if (ann_file)  fclose( ann_file );
    if (pac_file)  fclose( pac_file );
    if (wpac_file) fclose( wpac_file );

    return ann_ok && seq_ok;
}

// load a sequence file
//
// \param sequence_file_name   the file to open
// \param qualities            the encoding of the qualities
// \param max_seqs             maximum number of reads to input
// \param max_sequence_len     maximum read length - reads will be truncated
// \param flags                a set of flags indicating which strands to encode
//                             in the batch for each read.
//                             For example, passing FORWARD | REVERSE_COMPLEMENT
//                             will result in a stream containing BOTH the forward
//                             and reverse-complemented strands.
//
bool load_pac(
    const Alphabet              alphabet,
    SequenceDataHost*           sequence_data,
    const char*                 prefix,
    const SequenceFlags         load_flags,
    const QualityEncoding       qualities)
{
    // prepare the sequence index
    sequence_data->m_sequence_index_vec.resize( 1 );
    sequence_data->m_sequence_index_vec[0] = 0;

    // prepare the name index
    sequence_data->m_name_index_vec.resize( 1 );
    sequence_data->m_name_index_vec[0] = 0;

    // load the BNS files
    SequenceDataInfo info;
    try
    {
        BNTLoader loader(
            sequence_data->m_sequence_index_vec,
            sequence_data->m_name_index_vec,
            sequence_data->m_name_vec );

        load_bns( &loader, prefix );

        info = loader.m_info;
    }
    catch (...)
    {
        log_error(stderr, "loading BNS files failed\n");
        return false;
    }

    const uint32 bits             = bits_per_symbol( alphabet );
    const uint32 symbols_per_word = 32 / bits;

    const uint32 seq_length         = info.bps();
    const uint32 seq_words          = uint32( util::divide_ri( seq_length, symbols_per_word ) );
    const uint32 aligned_seq_words  = align<4>( seq_words );

    // setup all basic info
    sequence_data->SequenceDataInfo::operator=( info );
    sequence_data->m_alphabet               = alphabet;
    sequence_data->m_sequence_stream_words  = aligned_seq_words;
    sequence_data->m_name_stream_len        = uint32( sequence_data->m_name_vec.size() );
    sequence_data->m_has_qualities          = false;

    // alloc sequence storage
    sequence_data->m_sequence_vec.resize( sequence_data->m_sequence_stream_words );

    // initialize the alignment slack
    for (uint32 i = seq_words; i < aligned_seq_words; ++i)
        sequence_data->m_sequence_vec[i] = 0u;

    switch (alphabet)
    {
    case DNA:
        return load_pac<DNA>( prefix, &sequence_data->m_sequence_vec[0], seq_length, seq_words );
        break;
    case DNA_N:
        return load_pac<DNA_N>( prefix, &sequence_data->m_sequence_vec[0], seq_length, seq_words );
        break;
    case PROTEIN:
        return load_pac<PROTEIN>( prefix, &sequence_data->m_sequence_vec[0], seq_length, seq_words );
        break;

    default: break;
    }
    return false;
}

// load a sequence file
//
// \param sequence_file_name   the file to open
// \param qualities            the encoding of the qualities
// \param max_seqs             maximum number of reads to input
// \param max_sequence_len     maximum read length - reads will be truncated
// \param flags                a set of flags indicating which strands to encode
//                             in the batch for each read.
//                             For example, passing FORWARD | REVERSE_COMPLEMENT
//                             will result in a stream containing BOTH the forward
//                             and reverse-complemented strands.
//
bool load_pac(
    const Alphabet              alphabet,
    SequenceDataMMAPServer*     sequence_data,
    const char*                 prefix,
    const char*                 mapped_name,
    const SequenceFlags         load_flags,
    const QualityEncoding       qualities)
{
    std::string info_name           = SequenceDataMMAPServer::info_file_name( mapped_name );
    std::string sequence_name       = SequenceDataMMAPServer::sequence_file_name( mapped_name );
    std::string sequence_index_name = SequenceDataMMAPServer::sequence_index_file_name( mapped_name );
    std::string name_name           = SequenceDataMMAPServer::name_file_name( mapped_name );
    std::string name_index_name     = SequenceDataMMAPServer::name_index_file_name( mapped_name );

    // prepare the sequence index
    nvbio::vector<host_tag,uint32> sequence_index_vec;
    nvbio::vector<host_tag,uint32> name_index_vec;
    nvbio::vector<host_tag,char>   name_vec;

    sequence_index_vec.resize( 1 );
    sequence_index_vec[0] = 0;

    // prepare the name index
    name_index_vec.resize( 1 );
    name_index_vec[0] = 0;

    // load the BNS files
    SequenceDataInfo info;
    try
    {
        BNTLoader loader( sequence_index_vec, name_index_vec, name_vec );
        load_bns( &loader, prefix );

        info = loader.m_info;
    }
    catch (...)
    {
        log_error(stderr, "loading BNS files failed\n");
        return false;
    }

    const uint32 bits             = bits_per_symbol( alphabet );
    const uint32 symbols_per_word = 32 / bits;

    const uint32 seq_length         = info.bps();
    const uint32 seq_words          = uint32( util::divide_ri( seq_length, symbols_per_word ) );
    const uint32 aligned_seq_words  = align<4>( seq_words );

    // setup all basic info
    info.m_alphabet                 = alphabet;
    info.m_sequence_stream_words    = aligned_seq_words;
    info.m_name_stream_len          = uint32( name_vec.size() );
    info.m_has_qualities            = false;

    try
    {
        // alloc sequence storage
        uint32* sequence_ptr = (uint32*)sequence_data->m_sequence_file.init(
            sequence_name.c_str(),
            aligned_seq_words * sizeof(uint32),
            NULL );

        // initialize the alignment slack
        for (uint32 i = seq_words; i < aligned_seq_words; ++i)
            sequence_ptr[i] = 0u;

        // alloc sequence_index storage
        uint32* sequence_index_ptr = (uint32*)sequence_data->m_sequence_index_file.init(
            sequence_index_name.c_str(),
            sequence_index_vec.size() * sizeof(uint32),
            NULL );

        // alloc name_index storage
        uint32* name_index_ptr = (uint32*)sequence_data->m_name_index_file.init(
            name_index_name.c_str(),
            name_index_vec.size() * sizeof(uint32),
            NULL );

        // alloc name storage
        char* name_ptr = (char*)sequence_data->m_name_file.init(
            name_name.c_str(),
            name_vec.size() * sizeof(char),
            NULL );

        // alloc info storage
        SequenceDataInfo* info_ptr = (SequenceDataInfo*)sequence_data->m_info_file.init(
            info_name.c_str(),
            sizeof(SequenceDataInfo),
            NULL );

        // copy the loaded index and names vectors
        memcpy( sequence_index_ptr, &sequence_index_vec[0], sequence_index_vec.size() * sizeof(uint32) );
        memcpy( name_index_ptr,     &name_index_vec[0],     name_index_vec.size()     * sizeof(uint32) );
        memcpy( name_ptr,           &name_vec[0],           name_vec.size()           * sizeof(char) );

        *info_ptr = info;

        // load the actual sequence
        switch (alphabet)
        {
        case DNA:
            return load_pac<DNA>( prefix, sequence_ptr, seq_length, seq_words );
            break;
        case DNA_N:
            return load_pac<DNA_N>( prefix, sequence_ptr, seq_length, seq_words );
            break;
        case RNA:
            return load_pac<RNA>( prefix, sequence_ptr, seq_length, seq_words );
            break;
        case RNA_N:
            return load_pac<RNA_N>( prefix, sequence_ptr, seq_length, seq_words );
            break;
        case PROTEIN:
            return load_pac<PROTEIN>( prefix, sequence_ptr, seq_length, seq_words );
            break;

        default: break;
        }
    }
    catch (ServerMappedFile::mapping_error e)
    {
        log_error(stderr, "  mapping error while mapping file: %s (code: %u)\n", e.m_file_name, e.m_code );
    }
    catch (ServerMappedFile::view_error e)
    {
        log_error(stderr, "  view error while mapping file: %s (code: %u)\n", e.m_file_name, e.m_code );
    }
    return false;
}

///@} // SequenceIO
///@} // IO

} // namespace io
} // namespace nvbio
