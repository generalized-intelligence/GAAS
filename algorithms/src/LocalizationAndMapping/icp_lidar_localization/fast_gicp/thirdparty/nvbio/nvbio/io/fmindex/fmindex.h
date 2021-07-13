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

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/mmap.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/deinterleaved_iterator.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/fmindex/fmindex.h>
#include <nvbio/fmindex/ssa.h>

namespace nvbio {
///@addtogroup IO
///@{
namespace io {
///@}

///
/// \page fmindex_io_page FM-Index I/O
///\par
/// This module contains a series of classes to load FM-indices from disk into:
///\par
///  - host memory
///  - device memory
///  - mapped system memory
///\par
/// Specifically, it exposes the following classes:
///\par
/// - io::FMIndexData
/// - io::FMIndexDataHost
/// - io::FMIndexDataDevice
/// - io::FMIndexDataMMAP
/// - io::FMIndexDataMMAPServer
///

///@addtogroup IO
///@{

///
///@defgroup FMIndexIO FM-Index I/O
/// This module contains a series of classes to load FM-indices from disk into:
///  - RAM
///  - mapped memory
///  - CUDA device memory
///@{
///

///
/// Basic FM-index interface.
///
/// This class holds pointers to data that is typically going to be allocated/loaded/deallocated
/// by inheriting classes.
/// The idea is that accessing this basic information is fast and requires no virtual function
/// calls.
struct FMIndexDataCore
{
    static const uint32 FORWARD = 0x02;
    static const uint32 REVERSE = 0x04;
    static const uint32 SA      = 0x10;

    static const uint32 BWT_BITS             = 2u;                              // NOTE: DNA alphabet
    static const bool   BWT_BIG_ENDIAN       = true;                            // NOTE: needs to be true to allow fast BWT construction
    static const uint32 BWT_SYMBOLS_PER_WORD = (8*sizeof(uint32))/BWT_BITS;

    static const uint32 OCC_INT = 64;
    static const uint32 SA_INT  = 16;

    typedef const uint32*               bwt_occ_type;
    typedef const uint32*               count_table_type;

    typedef SSA_index_multiple_context<
        SA_INT,
        const uint32*>                  ssa_type;

    ///< empty constructor
    ///
    FMIndexDataCore() :
        m_flags         ( 0 ),
        m_seq_length    ( 0 ),
        m_bwt_occ_words ( 0 ),
        m_sa_words      ( 0 ),
        m_primary       ( 0 ),
        m_rprimary      ( 0 ),
        m_L2            ( NULL ),
        m_bwt_occ       ( NULL ),
        m_rbwt_occ      ( NULL ),
        m_count_table   ( NULL )
    {}
    
    uint32        flags()           const { return m_flags; }               ///< return loading flags
    uint32        length()          const { return m_seq_length; }          ///< return sequence length
    uint32        primary()         const { return m_primary; }             ///< return the primary key
    uint32        rprimary()        const { return m_rprimary; }            ///< return the reverse primary key
    bool          has_ssa()         const { return m_ssa.m_ssa != NULL; }   ///< return whether the sampled suffix array is present
    bool          has_rssa()        const { return m_rssa.m_ssa != NULL; }  ///< return whether the reverse sampled suffix array is present
    const uint32*  bwt_occ()        const { return m_bwt_occ; }             ///< return the BWT stream
    const uint32* rbwt_occ()        const { return m_rbwt_occ; }            ///< return the reverse BWT stream
    const uint32*  count_table()    const { return m_count_table; }         ///< return the count table
    uint32        bwt_occ_words()   const { return m_bwt_occ_words; }       ///< return the number of sequence words
    uint32        sa_words()        const { return m_sa_words; }            ///< return the number of SA words
    ssa_type      ssa()             const { return m_ssa; }
    ssa_type      rssa()            const { return m_rssa; }
    const uint32* L2()              const { return m_L2; }                  ///< return the L2 table

public:
    uint32      m_flags;
    uint32      m_seq_length;
    uint32      m_bwt_occ_words;
    uint32      m_sa_words;
    uint32      m_primary;
    uint32      m_rprimary;

    uint32*     m_L2;
    uint32*     m_bwt_occ;
    uint32*     m_rbwt_occ;
    uint32*     m_count_table;
    ssa_type    m_ssa;
    ssa_type    m_rssa;
};

///
/// Basic FM-index interface.
///
/// This class holds pointers to data that is typically going to be allocated/loaded/deallocated
/// by inheriting classes.
/// The idea is that accessing this basic information is fast and requires no virtual function
/// calls.
struct FMIndexData : public FMIndexDataCore
{
    typedef const uint4*                                            bwt_occ_type;
    typedef deinterleaved_iterator<2,0,bwt_occ_type>                bwt_type;
    typedef deinterleaved_iterator<2,1,bwt_occ_type>                occ_type;

    typedef const uint32*                                           count_table_type;
    typedef SSA_index_multiple<SA_INT>                              ssa_storage_type;
    typedef PackedStream<bwt_type,uint8,BWT_BITS,BWT_BIG_ENDIAN>    bwt_stream_type;

    typedef rank_dictionary<
        BWT_BITS,
        FMIndexDataCore::OCC_INT,
        bwt_stream_type,
        occ_type,
        count_table_type>                                            rank_dict_type;

    typedef fm_index<rank_dict_type, ssa_type>                       fm_index_type;
    typedef fm_index<rank_dict_type, null_type>              partial_fm_index_type;

             FMIndexData();                                                 ///< empty constructor
    virtual ~FMIndexData() {}                                               ///< virtual destructor

    /// iterators access
    ///
    occ_type  occ_iterator() const { return occ_type(bwt_occ_type( bwt_occ())); }
    occ_type rocc_iterator() const { return occ_type(bwt_occ_type(rbwt_occ())); }

    bwt_type  bwt_iterator() const { return bwt_type(bwt_occ_type( bwt_occ())); }
    bwt_type rbwt_iterator() const { return bwt_type(bwt_occ_type(rbwt_occ())); }

    ssa_type  ssa_iterator() const { return ssa(); }
    ssa_type rssa_iterator() const { return rssa(); }

    count_table_type count_table_iterator() const { return count_table_type( count_table() ); }

    rank_dict_type  rank_dict() const { return rank_dict_type( bwt_stream_type(  bwt_iterator() ),  occ_iterator(), count_table_iterator() ); }
    rank_dict_type rrank_dict() const { return rank_dict_type( bwt_stream_type( rbwt_iterator() ), rocc_iterator(), count_table_iterator() ); }

    fm_index_type  index() const { return fm_index_type( length(),  primary(),  L2(),  rank_dict(),  ssa_iterator() ); }
    fm_index_type rindex() const { return fm_index_type( length(), rprimary(),  L2(), rrank_dict(), rssa_iterator() ); }

    partial_fm_index_type  partial_index() const { return partial_fm_index_type( length(),  primary(), L2(),  rank_dict(), null_type() ); }
    partial_fm_index_type rpartial_index() const { return partial_fm_index_type( length(), rprimary(), L2(), rrank_dict(), null_type() ); }
};

void init_ssa(
    const FMIndexData&              driver_data,
    FMIndexData::ssa_storage_type&  ssa,
    FMIndexData::ssa_storage_type&  rssa);

///
/// An in-RAM FM-index.
///
struct FMIndexDataHost : public FMIndexData
{
    /// load a genome from file
    ///
    /// \param genome_prefix            prefix file name
    /// \param flags                    loading flags specifying which elements to load
    int load(
        const char* genome_prefix,
        const uint32 flags = FORWARD | REVERSE | SA);

    nvbio::vector<host_tag,uint32>  m_bwt_occ_vec;          ///< local storage for the forward BWT/OCC
    nvbio::vector<host_tag,uint32>  m_rbwt_occ_vec;         ///< local storage for the reverse BWT/OCC
    nvbio::vector<host_tag,uint32>  m_ssa_vec;              ///< local storage for the forward SSA
    nvbio::vector<host_tag,uint32>  m_rssa_vec;             ///< local storage for the reverse SSA
    uint32                          m_count_table_vec[256]; ///< local storage for the BWT counting table
    uint32                          m_L2_vec[5];            ///< local storage for the L2 vector
};

struct FMIndexDataMMAPInfo
{
    uint32  sequence_length;
    uint32  bwt_occ_words;
    uint32  sa_words;
    uint32  primary;
    uint32  rprimary;
    uint32  L2[5];
};

///
/// A memory-mapped FM-index server, which can load an FM-index from disk and map it to
/// a shared memory arena.
///
struct FMIndexDataMMAPServer : public FMIndexData
{
    typedef FMIndexDataMMAPInfo Info;

    /// load a genome from file
    ///
    /// \param genome_prefix            prefix file name
    /// \param mapped_name              memory mapped object name
    int load(
        const char* genome_prefix, const char* mapped_name);

private:
    Info                m_info;                         ///< internal info object storage
    ServerMappedFile    m_bwt_occ_file;                 ///< internal memory-mapped forward occurrence table object server
    ServerMappedFile    m_rbwt_occ_file;                ///< internal memory-mapped reverse occurrence table object server
    ServerMappedFile    m_sa_file;                      ///< internal memory-mapped forward SSA table object server
    ServerMappedFile    m_rsa_file;                     ///< internal memory-mapped reverse SSA table object server
    ServerMappedFile    m_info_file;                    ///< internal memory-mapped reverse SSA table object server

    uint32              m_count_table_vec[256];         ///< local storage for the BWT counting table
    uint32              m_L2_vec[5];                    ///< local storage for the L2 vector
};

///
/// A memory-mapped FM-index client, which can connect to a shared-memory FM-index
/// and present it as local.
///
struct FMIndexDataMMAP : public FMIndexData
{
    typedef FMIndexDataMMAPInfo Info;

    /// load from a memory mapped object
    ///
    /// \param genome_name          memory mapped object name
    int load(
        const char*  genome_name);

    MappedFile          m_bwt_occ_file;                 ///< internal memory-mapped forward BWT object
    MappedFile          m_rbwt_occ_file;                ///< internal memory-mapped reverse BWT object
    MappedFile          m_sa_file;                      ///< internal memory-mapped forward SSA table object
    MappedFile          m_rsa_file;                     ///< internal memory-mapped reverse SSA table object
    MappedFile          m_info_file;                    ///< internal memory-mapped info object

    uint32              m_count_table_vec[256];         ///< local storage for the BWT counting table
    uint32              m_L2_vec[5];
};

///
/// A device-side FM-index - which can take a host memory FM-index and map it to
/// device memory.
///
struct FMIndexDataDevice : public FMIndexData
{
    static const uint32 FORWARD = 0x02;
    static const uint32 REVERSE = 0x04;
    static const uint32 SA      = 0x10;

    // FM-index type interfaces
    //
    typedef cuda::ldg_pointer<uint4>                                bwt_occ_type;
    typedef deinterleaved_iterator<2,0,bwt_occ_type>                bwt_type;
    typedef deinterleaved_iterator<2,1,bwt_occ_type>                occ_type;
    typedef cuda::ldg_pointer<uint32>                               count_table_type;
    typedef cuda::ldg_pointer<uint32>                               ssa_ldg_type;
    typedef SSA_index_multiple_device<SA_INT>                       ssa_storage_type;
    typedef PackedStream<bwt_type,uint8,BWT_BITS,BWT_BIG_ENDIAN>    bwt_stream_type;

    typedef SSA_index_multiple_context<
        FMIndexDataCore::SA_INT,
        ssa_ldg_type>                                               ssa_type;

    typedef rank_dictionary<
        BWT_BITS,
        FMIndexDataCore::OCC_INT,
        bwt_stream_type,
        occ_type,
        count_table_type>                                           rank_dict_type;

    typedef fm_index<rank_dict_type,ssa_type>                       fm_index_type;
    typedef fm_index<rank_dict_type,null_type>              partial_fm_index_type;

    /// load a host-memory FM-index in device memory
    ///
    /// \param host_data                                host-memory FM-index to load
    /// \param flags                                    specify which parts of the FM-index to load
    FMIndexDataDevice(const FMIndexData& host_data, const uint32 flags = FORWARD | REVERSE);

    uint64 allocated() const { return m_allocated; }    ///< return the amount of allocated device memory

    /// iterators access
    ///
    occ_type  occ_iterator() const { return occ_type(bwt_occ_type((const uint4*) bwt_occ())); }
    occ_type rocc_iterator() const { return occ_type(bwt_occ_type((const uint4*)rbwt_occ())); }

    bwt_type  bwt_iterator() const { return bwt_type(bwt_occ_type((const uint4*) bwt_occ())); }
    bwt_type rbwt_iterator() const { return bwt_type(bwt_occ_type((const uint4*)rbwt_occ())); }

    ssa_type  ssa_iterator() const { return ssa_type(ssa_ldg_type( m_ssa.m_ssa)); }
    ssa_type rssa_iterator() const { return ssa_type(ssa_ldg_type(m_rssa.m_ssa)); }

    count_table_type count_table_iterator() const { return count_table_type( count_table() ); }

    rank_dict_type  rank_dict() const { return rank_dict_type( bwt_stream_type(  bwt_iterator() ),  occ_iterator(), count_table_iterator() ); }
    rank_dict_type rrank_dict() const { return rank_dict_type( bwt_stream_type( rbwt_iterator() ), rocc_iterator(), count_table_iterator() ); }

    fm_index_type  index() const { return fm_index_type( length(),  primary(), L2(),  rank_dict(),  ssa_iterator() ); }
    fm_index_type rindex() const { return fm_index_type( length(), rprimary(), L2(), rrank_dict(), rssa_iterator() ); }

    partial_fm_index_type  partial_index() const { return partial_fm_index_type( length(),  primary(), L2(),  rank_dict(), null_type() ); }
    partial_fm_index_type rpartial_index() const { return partial_fm_index_type( length(), rprimary(), L2(), rrank_dict(), null_type() ); }

private:
    uint64                            m_allocated;          ///< # of allocated device memory bytes
    nvbio::vector<device_tag,uint32>  m_bwt_occ_vec;        ///< local storage for the forward BWT/OCC
    nvbio::vector<device_tag,uint32>  m_rbwt_occ_vec;       ///< local storage for the reverse BWT/OCC
    nvbio::vector<device_tag,uint32>  m_ssa_vec;            ///< local storage for the forward SSA
    nvbio::vector<device_tag,uint32>  m_rssa_vec;           ///< local storage for the reverse SSA
    nvbio::vector<device_tag,uint32>  m_count_table_vec;    ///< local storage for the BWT counting table
    nvbio::vector<device_tag,uint32>  m_L2_vec;             ///< local storage for the L2 vector
};

/// initialize the sampled suffix arrays on the GPU given a device-side FM-index.
///
void init_ssa(
    const FMIndexDataDevice&                driver_data,
    FMIndexDataDevice::ssa_storage_type&    ssa,
    FMIndexDataDevice::ssa_storage_type&    rssa);

///@} // FMIndexIO
///@} // IO

} // namespace io
} // namespace nvbio
