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

#include <nvbio/io/fmindex/fmindex.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/bnt.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/dna.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/vector.h>
#include <nvbio/fmindex/bwt.h>
#include <nvbio/fmindex/ssa.h>
#include <nvbio/fmindex/fmindex.h>
#include <crc/crc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <string>
#if defined(_OPENMP)
#include <omp.h>
#endif

#define FMI_ALIGNMENT 4u

namespace nvbio {
namespace io {

///@addtogroup IO
///@{

///@addtogroup FMIndexIO
///@{

namespace { // anonymous namespace

///@addtogroup FMIndexIODetail
///@{

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

struct file_mismatch {};

struct VectorAllocator
{
    VectorAllocator(nvbio::vector<host_tag,uint32>& vec) : m_vec( vec ) {}

    uint32* alloc(const uint32 words)
    {
        m_vec.resize( words );
        return raw_pointer( m_vec );
    }

    nvbio::vector<host_tag,uint32>& m_vec;
};
struct MMapAllocator
{
    MMapAllocator(
        const char*       name,
        ServerMappedFile& mmap) : m_name( name ), m_mmap( mmap ) {}

    uint32* alloc(const uint32 words)
    {
        return (uint32*)m_mmap.init(
            m_name,
            words * sizeof(uint32),
            NULL );
    }

    const char*       m_name;
    ServerMappedFile& m_mmap;
};

template <typename Allocator>
uint32* load_bwt(
    const char*     bwt_file_name,
    Allocator&      allocator,
    uint32&         seq_length,
    uint32&         seq_words,
    uint32&         primary)
{
    FILE* bwt_file = fopen( bwt_file_name, "rb" );
    if (bwt_file == NULL)
    {
        log_warning(stderr, "unable to open bwt \"%s\"\n", bwt_file_name);
        return 0;
    }
    uint32 field;
    if (!fread( &field, sizeof(field), 1, bwt_file ))
    {
        log_error(stderr, "error: failed reading bwt \"%s\"\n", bwt_file_name);
        return 0;
    }
    primary = uint32(field);

    // discard frequencies
    seq_length = 0;
    for (uint32 i = 0; i < 4; ++i)
    {
        if (!fread( &field, sizeof(field), 1, bwt_file ))
        {
            log_error(stderr, "error: failed reading bwt \"%s\"\n", bwt_file_name);
            return 0;
        }

        // the sum of the frequencies gives the total length
        if (i == 3)
            seq_length = uint32(field);
    }

    // compute the number of words needed to store the sequence
    seq_words = util::divide_ri( seq_length, FMIndexDataCore::BWT_SYMBOLS_PER_WORD );

    // pad the size to a multiple of 4
    seq_words = align<4>( seq_words );

    // allocate the stream storage
    uint32* bwt_stream = allocator.alloc( seq_words );

    const uint32 n_words = (uint32)block_fread( bwt_stream, seq_words, bwt_file );
    if (align<4>( n_words ) != seq_words)
    {
        log_error(stderr, "error: failed reading bwt \"%s\"\n", bwt_file_name);
        return 0;
    }

    // initialize the slack due to sequence padding
    for (uint32 i = n_words; i < seq_words; ++i)
        bwt_stream[i] = 0u;

    fclose( bwt_file );
    return bwt_stream;
}

template <typename Allocator>
uint32* load_sa(
    const char*     sa_file_name,
    Allocator&      allocator,
    const uint32    seq_length,
    const uint32    primary,
    const uint32    SA_INT)
{
    uint32* ssa = NULL;

    FILE* sa_file = fopen( sa_file_name, "rb" );
    if (sa_file != NULL)
    {
        log_info(stderr, "reading SSA... started\n");

        try
        {
            uint32 field;

            if (!fread( &field, sizeof(field), 1, sa_file ))
            {
                log_error(stderr, "error: failed reading SSA \"%s\"\n", sa_file_name);
                return 0;
            }
            if (field != primary)
            {
                log_error(stderr, "SA file mismatch \"%s\"\n  expected primary %u, got %u\n", sa_file_name, primary, field);
                throw file_mismatch();
            }

            for (uint32 i = 0; i < 4; ++i)
            {
                if (!fread( &field, sizeof(field), 1, sa_file ))
                {
                    log_error(stderr, "error: failed reading SSA \"%s\"\n", sa_file_name);
                    return 0;
                }
            }

            if (!fread( &field, sizeof(field), 1, sa_file ))
            {
                log_error(stderr, "error: failed reading SSA \"%s\"\n", sa_file_name);
                return 0;
            }
            if (field != SA_INT)
            {
                log_error(stderr, "unsupported SA interval (found %u, expected %u)\n", field, SA_INT);
                throw file_mismatch();
            }

            if(!fread( &field, sizeof(field), 1, sa_file ))
            {
                log_error(stderr, "error: failed reading SSA \"%s\"\n", sa_file_name);
                return 0;
            }
            if (field != seq_length)
            {
                log_error(stderr, "SA file mismatch \"%s\"\n  expected length %u, got %u", sa_file_name, seq_length, field);
                throw file_mismatch();
            }

            const uint32 sa_size = (seq_length + SA_INT) / SA_INT;

            ssa = allocator.alloc( sa_size );
            ssa[0] = uint32(-1);
            if (!fread( &ssa[1], sizeof(uint32), sa_size-1, sa_file ))
            {
                log_error(stderr, "error: failed reading SSA \"%s\"\n", sa_file_name);
                return 0;
            }
        }
        catch (...)
        {
            // just skip the ssa file
        }
        fclose( sa_file );

        log_info(stderr, "reading SSA... done\n");
    }
    return ssa;
}

template <typename Allocator>
uint32* build_occurrence_table(
    const uint32                            seq_length,
    const uint32                            seq_words,
    const nvbio::vector<host_tag,uint32>&   bwt_vec,
    Allocator&                              allocator,
    uint32&                                 bwt_occ_words,
    uint32*                                 L2)
{
    typedef PackedStream<const uint32*,uint8,FMIndexDataCore::BWT_BITS,FMIndexDataCore::BWT_BIG_ENDIAN> stream_type;

    // build a bwt stream
    stream_type bwt( raw_pointer( bwt_vec ) );

    // compute the number of words needed to store the occurrences
    const uint32 occ_words = util::divide_ri( seq_length, FMIndexDataCore::OCC_INT ) * 4;

    // build the occurrence table
    nvbio::vector<host_tag,uint32> occ_vec( occ_words, 0u );
    uint32 cnt[4];

    nvbio::build_occurrence_table<FMIndexDataCore::BWT_BITS,FMIndexDataCore::OCC_INT>(
        bwt,
        bwt + seq_length,
        raw_pointer( occ_vec ),
        cnt );

    if (occ_words != seq_words)
    {
        log_error(stderr, "error: bwt size != occurrence table size!\n  words: %u, %u\n", seq_words, occ_words);
        return 0;
    }
    if ((seq_words % 4u) != 0)
    {
        log_error(stderr, "error: occ size not a multiple of 4\n  words: %u\n", seq_words);
        return 0;
    }
    if ((occ_words % 4u) != 0)
    {
        log_error(stderr, "error: occ size not a multiple of 4\n  words: %u\n", occ_words);
        return 0;
    }

    // fuse the BWT & OCC vectors
    bwt_occ_words = seq_words + occ_words;
    uint32* bwt_occ = allocator.alloc( bwt_occ_words );

    #if defined(_OPENMP)
    #pragma omp parallel for
    #endif
    for (int64 w = 0; w < int64( seq_words ); w += 4)
    {
        bwt_occ[ w*2+0 ] = bwt_vec[ w+0 ];
        bwt_occ[ w*2+1 ] = bwt_vec[ w+1 ];
        bwt_occ[ w*2+2 ] = bwt_vec[ w+2 ];
        bwt_occ[ w*2+3 ] = bwt_vec[ w+3 ];
        bwt_occ[ w*2+4 ] = occ_vec[ w+0 ];
        bwt_occ[ w*2+5 ] = occ_vec[ w+1 ];
        bwt_occ[ w*2+6 ] = occ_vec[ w+2 ];
        bwt_occ[ w*2+7 ] = occ_vec[ w+3 ];
    }

    // compute the L2 table
    L2[0] = 0;
    for (uint32 c = 0; c < 4; ++c)
        L2[c+1] = L2[c] + cnt[c];

    return bwt_occ;
}

///@} // FMIndexIODetails

} // anonymous namespace

// constructor
//
FMIndexData::FMIndexData()
{
}

int FMIndexDataHost::load(
    const char* genome_prefix,
    const uint32 flags)
{
    log_visible(stderr, "FMIndexData: loading... started\n");
    log_visible(stderr, "  genome : %s\n", genome_prefix);

    // initialize the core
    this->FMIndexDataCore::operator=( FMIndexDataCore() );

    // bind pointers to static vectors
    m_flags       = flags;
    m_count_table = &m_count_table_vec[0];
    m_L2          = &m_L2_vec[0];

    std::string bwt_string    = std::string( genome_prefix ) + ".bwt";
    std::string rbwt_string   = std::string( genome_prefix ) + ".rbwt";
    std::string sa_string     = std::string( genome_prefix ) + ".sa";
    std::string rsa_string    = std::string( genome_prefix ) + ".rsa";

    const char* bwt_file_name  = bwt_string.c_str();
    const char* rbwt_file_name = rbwt_string.c_str();
    const char* sa_file_name   = sa_string.c_str();
    const char* rsa_file_name  = rsa_string.c_str();

    uint32 seq_length;
    uint32 seq_words;

    if (flags & FORWARD)
    {
        nvbio::vector<host_tag,uint32> bwt_vec;

        // read bwt
        log_info(stderr, "reading bwt... started\n");
        {
            VectorAllocator allocator( bwt_vec );
            if (load_bwt(
                bwt_file_name,
                allocator,
                seq_length,
                seq_words,
                m_primary ) == NULL)
                return 0;
        }
        log_info(stderr, "reading bwt... done\n");
        log_verbose(stderr, "  length: %u\n", seq_length);

        log_info(stderr, "building occurrence table... started\n");
        {
            VectorAllocator allocator( m_bwt_occ_vec );

            m_bwt_occ = build_occurrence_table(
                seq_length,
                seq_words,
                bwt_vec,
                allocator,
                m_bwt_occ_words,
                m_L2 );
        }
        log_info(stderr, "building occurrence table... done\n");
        log_info(stderr, "  size: %u words\n", m_bwt_occ_words );
    }

    if (flags & REVERSE)
    {
        nvbio::vector<host_tag,uint32> rbwt_vec;

        log_info(stderr, "reading rbwt... started\n");
        {
            VectorAllocator allocator( rbwt_vec );
            if (load_bwt(
                rbwt_file_name,
                allocator,
                seq_length,
                seq_words,
                m_rprimary ) == NULL)
                return 0;
        }
        log_info(stderr, "reading rbwt... done\n");
        log_verbose(stderr, "  length: %u\n", seq_length);

        log_info(stderr, "building occurrence table... started\n");
        {
            VectorAllocator allocator( m_rbwt_occ_vec );

            m_rbwt_occ = build_occurrence_table(
                seq_length,
                seq_words,
                rbwt_vec,
                allocator,
                m_bwt_occ_words,
                m_L2 );
        }
        log_info(stderr, "building occurrence table... done\n");
    }

    // record the sequence length
    m_seq_length = seq_length;

    if (flags & FORWARD) log_visible(stderr, "   primary : %u\n", uint32(m_primary));
    if (flags & REVERSE) log_visible(stderr, "  rprimary : %u\n", uint32(m_rprimary));

    // read ssa
    if (flags & SA)
    {
        if (flags & FORWARD)
        {
            VectorAllocator allocator( m_ssa_vec );
            m_ssa.m_ssa = load_sa(
                sa_file_name,
                allocator,
                seq_length,
                m_primary,
                SA_INT );
        }
        // read rssa
        if (flags & REVERSE)
        {
            VectorAllocator allocator( m_rssa_vec );
            m_rssa.m_ssa = load_sa(
                rsa_file_name,
                allocator,
                seq_length,
                m_rprimary,
                SA_INT );
        }

        // record the number of SA words
        m_sa_words = (seq_length + SA_INT) / SA_INT;
    }

    // generate the count table
    gen_bwt_count_table( m_count_table );

    const uint32 has_fw     = (m_flags & FORWARD) ? 1u : 0;
    const uint32 has_rev    = (m_flags & REVERSE) ? 1u : 0;
    const uint32 has_sa     = (m_flags & SA)      ? 1u : 0;

    const uint64 memory_footprint =
                 (has_fw + has_rev) * sizeof(uint32)*m_bwt_occ_words +
        has_sa * (has_fw + has_rev) * sizeof(uint32)*m_sa_words;

    log_visible(stderr, "  memory   : %.1f MB\n", float(memory_footprint)/float(1024*1024));

    log_visible(stderr, "FMIndexData: loading... done\n");
    return 1;
}

int FMIndexDataMMAPServer::load(const char* genome_prefix, const char* mapped_name)
{
    log_visible(stderr, "FMIndexData: loading... started\n");
    log_visible(stderr, "  genome : %s\n", genome_prefix);

    std::string bwt_string    = std::string( genome_prefix ) + ".bwt";
    std::string rbwt_string   = std::string( genome_prefix ) + ".rbwt";
    std::string sa_string     = std::string( genome_prefix ) + ".sa";
    std::string rsa_string    = std::string( genome_prefix ) + ".rsa";

    const char* bwt_file_name  = bwt_string.c_str();
    const char* rbwt_file_name = rbwt_string.c_str();
    const char* sa_file_name   = sa_string.c_str();
    const char* rsa_file_name  = rsa_string.c_str();

    std::string infoName = std::string("nvbio.") + std::string( mapped_name ) + ".info";
    std::string bwtName  = std::string("nvbio.") + std::string( mapped_name ) + ".bwt_occ";
    std::string rbwtName = std::string("nvbio.") + std::string( mapped_name ) + ".rbwt_occ";
    std::string saName   = std::string("nvbio.") + std::string( mapped_name ) + ".sa";
    std::string rsaName  = std::string("nvbio.") + std::string( mapped_name ) + ".rsa";

    // initialize the core
    this->FMIndexDataCore::operator=( FMIndexDataCore() );

    // bind pointers to static vectors
    m_count_table = &m_count_table_vec[0];
    m_L2          = &m_L2_vec[0];

    m_flags = FORWARD | REVERSE | SA;

    try
    {
        uint32 seq_length;
        uint32 seq_words;

        // forward BWT
        {
            nvbio::vector<host_tag,uint32> bwt_vec;

            log_info(stderr, "reading bwt... started\n");
            {
                VectorAllocator allocator( bwt_vec );
                if (load_bwt(
                    bwt_file_name,
                    allocator,
                    seq_length,
                    seq_words,
                    m_primary ) == NULL)
                    return 0;
            }
            log_info(stderr, "reading bwt... done\n");
            log_verbose(stderr, "  length: %u\n", seq_length);

            log_info(stderr, "building occurrence table... started\n");
            {
                MMapAllocator allocator( bwtName.c_str(), m_bwt_occ_file );

                m_bwt_occ = build_occurrence_table(
                    seq_length,
                    seq_words,
                    bwt_vec,
                    allocator,
                    m_bwt_occ_words,
                    m_L2 );
            }
            log_info(stderr, "building occurrence table... done\n");
        }

        // reverse BWT
        {
            nvbio::vector<host_tag,uint32> rbwt_vec;

            log_info(stderr, "reading bwt... started\n");
            {
                VectorAllocator allocator( rbwt_vec );
                if (load_bwt(
                    rbwt_file_name,
                    allocator,
                    seq_length,
                    seq_words,
                    m_rprimary ) == NULL)
                    return 0;
            }
            log_info(stderr, "reading bwt... done\n");
            log_verbose(stderr, "  length: %u\n", seq_length);

            log_info(stderr, "building occurrence table... started\n");
            {
                MMapAllocator allocator( rbwtName.c_str(), m_rbwt_occ_file );

                m_rbwt_occ = build_occurrence_table(
                    seq_length,
                    seq_words,
                    rbwt_vec,
                    allocator,
                    m_bwt_occ_words,
                    m_L2 );
            }
            log_info(stderr, "building occurrence table... done\n");
        }

        log_visible(stderr, "   primary : %u\n", uint32(m_primary));
        log_visible(stderr, "  rprimary : %u\n", uint32(m_rprimary));

        // read ssa
        {
            MMapAllocator allocator( saName.c_str(), m_sa_file );
            m_ssa.m_ssa = load_sa(
                sa_file_name,
                allocator,
                seq_length,
                m_primary,
                SA_INT );
        }
        // read rssa
        {
            MMapAllocator allocator( rsaName.c_str(), m_rsa_file );
            m_rssa.m_ssa = load_sa(
                rsa_file_name,
                allocator,
                seq_length,
                m_rprimary,
                SA_INT );
        }

        // record the sequence length
        m_seq_length = seq_length;

        // record the number of SA words
        m_sa_words = has_ssa() ? (seq_length + SA_INT) / SA_INT : 0u;

        // generate the count table
        gen_bwt_count_table( m_count_table );

        const uint32 has_fw     = (m_flags & FORWARD) ? 1u : 0;
        const uint32 has_rev    = (m_flags & REVERSE) ? 1u : 0;
        const uint32 has_sa     = (m_flags & SA)      ? 1u : 0;

        const uint64 memory_footprint =
                     (has_fw + has_rev) * sizeof(uint32)*m_bwt_occ_words +
            has_sa * (has_fw + has_rev) * sizeof(uint32)*m_sa_words;

        log_visible(stderr, "  memory   : %.1f MB\n", float(memory_footprint)/float(1024*1024));

        m_info.sequence_length = m_seq_length;
        m_info.bwt_occ_words   = m_bwt_occ_words;
        m_info.sa_words        = m_sa_words;
        m_info.primary         = m_primary;
        m_info.rprimary        = m_rprimary;
        for (uint32 i = 0; i < 5; ++i)
            m_info.L2[i]  = m_L2[i];

        m_info_file.init(
            infoName.c_str(),
            sizeof(Info),
            &m_info );
    }
    catch (ServerMappedFile::mapping_error error)
    {
        log_error(stderr,"could not create file mapping object \"%s\" (error %d)\n",
            error.m_file_name,
            error.m_code );
    }
    catch (ServerMappedFile::view_error error)
    {
        log_error(stderr, "could not map view file \"%s\" (error %d)\n",
            error.m_file_name,
            error.m_code );
    }
    catch (...)
    {
    };

    log_visible(stderr, "FMIndexData: loading... done\n");
    return 1;
}

int FMIndexDataMMAP::load(
    const char* file_name)
{
    std::string infoName = std::string("nvbio.") + std::string( file_name ) + ".info";
    std::string bwtName  = std::string("nvbio.") + std::string( file_name ) + ".bwt_occ";
    std::string rbwtName = std::string("nvbio.") + std::string( file_name ) + ".rbwt_occ";
    std::string saName   = std::string("nvbio.") + std::string( file_name ) + ".sa";
    std::string rsaName  = std::string("nvbio.") + std::string( file_name ) + ".rsa";

    // initialize the core
    this->FMIndexDataCore::operator=( FMIndexDataCore() );

    // bind pointers to static vectors
    m_count_table = &m_count_table_vec[0];
    m_L2          = &m_L2_vec[0];

    try {
        const Info* info = (const Info*)m_info_file.init( infoName.c_str(), sizeof(Info) );

        const uint64 bwt_file_size = info->bwt_occ_words * sizeof(uint32);
        const uint64 sa_file_size  = info->sa_words      * sizeof(uint32);

        m_bwt_occ       = (uint32*) m_bwt_occ_file.init(  bwtName.c_str(), bwt_file_size );
        m_rbwt_occ      = (uint32*)m_rbwt_occ_file.init( rbwtName.c_str(), bwt_file_size );
        if (info->sa_words)
        {
            m_ssa.m_ssa  = (uint32*) m_sa_file.init(  saName.c_str(), sa_file_size );
            m_rssa.m_ssa = (uint32*)m_rsa_file.init( rsaName.c_str(), sa_file_size );
            m_sa_words   = info->sa_words;
        }
        else
        {
            m_ssa.m_ssa  = NULL;
            m_rssa.m_ssa = NULL;
            m_sa_words   = 0u;
        }

        // record the core info
        m_seq_length    = info->sequence_length;
        m_bwt_occ_words = info->bwt_occ_words;
        m_primary       = info->primary;
        m_rprimary      = info->rprimary;
        for (uint32 i = 0; i < 5; ++i)
            m_L2[i] = info->L2[i];

        // generate the count table
        gen_bwt_count_table( m_count_table_vec );
    }
    catch (MappedFile::mapping_error error)
    {
        log_error(stderr, "FMIndexDataMMAP: error mapping file \"%s\" (%d)!\n", error.m_file_name, error.m_code);
        return 0;
    }
    catch (MappedFile::view_error error)
    {
        log_error(stderr, "FMIndexDataMMAP: error viewing file \"%s\" (%d)!\n", error.m_file_name, error.m_code);
        return 0;
    }
    catch (...)
    {
        log_error(stderr, "FMIndexDataMMAP: error mapping file (unknown)!\n");
        return 0;
    }
    return 1;
}

void init_ssa(
    const FMIndexData&              driver_data,
    FMIndexData::ssa_storage_type&  ssa,
    FMIndexData::ssa_storage_type&  rssa)
{
    typedef FMIndexData::ssa_storage_type   SSA_type;

    log_info(stderr, "building SSA... started\n");
    ssa = SSA_type( driver_data.partial_index() /*, SA_INT*/ );
    log_info(stderr, "building SSA... done\n");

    log_info(stderr, "building reverse SSA... started\n");
    rssa = SSA_type( driver_data.rpartial_index() /*, SA_INT*/ );
    log_info(stderr, "building reverse SSA... done\n");
}

FMIndexDataDevice::FMIndexDataDevice(const FMIndexData& host_data, const uint32 flags) :
    m_allocated( 0u )
{
    // initialize the core
    this->FMIndexDataCore::operator=( FMIndexDataCore() );

    m_seq_length    = host_data.m_seq_length;
    m_bwt_occ_words = host_data.m_bwt_occ_words;
    m_sa_words      = host_data.m_sa_words;
    m_primary       = host_data.m_primary;
    m_rprimary      = host_data.m_rprimary;

    m_L2_vec.resize( 5 );
    m_L2 = raw_pointer( m_L2_vec );

    m_count_table_vec.resize( 256 );
    m_count_table = raw_pointer( m_count_table_vec );

    thrust::copy( host_data.m_L2,           host_data.m_L2          + 5,    m_L2_vec.begin() );
    thrust::copy( host_data.m_count_table,  host_data.m_count_table + 256,  m_count_table_vec.begin() );

    if (flags & FORWARD)
    {
        if (host_data.m_bwt_occ == NULL)
            log_warning(stderr, "FMIndexDataDevice: requested forward BWT is not available!\n");

        m_bwt_occ_vec.resize( m_bwt_occ_words );
        m_bwt_occ = raw_pointer( m_bwt_occ_vec );

        thrust::copy(
            host_data.m_bwt_occ,
            host_data.m_bwt_occ + m_bwt_occ_words,
            m_bwt_occ_vec.begin() );

        m_allocated += sizeof(uint32)*( m_bwt_occ_words );

        if (flags & SA)
        {
            if (host_data.m_ssa.m_ssa == NULL)
                log_warning(stderr, "FMIndexDataDevice: requested forward SSA is not available!\n");

            m_ssa_vec.resize( m_sa_words );
            m_ssa.m_ssa = raw_pointer( m_ssa_vec );

            thrust::copy(
                host_data.m_ssa.m_ssa,
                host_data.m_ssa.m_ssa + m_sa_words,
                m_ssa_vec.begin() );

            m_allocated += sizeof(uint32)*( m_sa_words );
        }
    }

    if (flags & REVERSE)
    {
        if (host_data.m_rbwt_occ == NULL)
            log_warning(stderr, "FMIndexDataDevice: requested reverse BWT is not available!\n");

        m_rbwt_occ_vec.resize( m_bwt_occ_words );
        m_rbwt_occ = raw_pointer( m_rbwt_occ_vec );

        thrust::copy(
            host_data.m_rbwt_occ,
            host_data.m_rbwt_occ + m_bwt_occ_words,
            m_rbwt_occ_vec.begin() );

        m_allocated += sizeof(uint32)*( m_bwt_occ_words );

        if (flags & SA)
        {
            if (host_data.m_rssa.m_ssa == NULL)
                log_warning(stderr, "FMIndexDataDevice: requested reverse SSA is not available!\n");

            m_rssa_vec.resize( m_sa_words );
            m_rssa.m_ssa = raw_pointer( m_rssa_vec );

            thrust::copy(
                host_data.m_rssa.m_ssa,
                host_data.m_rssa.m_ssa + m_sa_words,
                m_rssa_vec.begin() );

            m_allocated += sizeof(uint32)*( m_sa_words );
        }
    }

    nvbio::cuda::check_error("FMIndexDataDevice");
}

void init_ssa(
    const FMIndexDataDevice&              driver_data,
    FMIndexDataDevice::ssa_storage_type&  ssa,
    FMIndexDataDevice::ssa_storage_type&  rssa)
{
    log_info(stderr, "building SSA... started\n");
    ssa.init( driver_data.partial_index() );
    log_info(stderr, "building SSA... done\n");

    log_info(stderr, "building reverse SSA... started\n");
    rssa.init( driver_data.rpartial_index() );
    log_info(stderr, "building reverse SSA... done\n");
}

///@} // FMIndexIO
///@} // IO

} // namespace io
} // namespace nvbio
