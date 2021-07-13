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

#include <nvbio/io/sequence/sequence.h>
#include <nvbio/basic/mmap.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>

namespace nvbio {
namespace io {

///@addtogroup IO
///@{

///@addtogroup SequenceIO
///@{

///
/// A system mapped memory server for sequence data
///
struct SequenceDataMMAPServer
{
    /// load a sequence from file
    ///
    /// \param alphabet                 the alphabet to use for encoding
    /// \param file_name                file name
    /// \param mapped_name              memory mapped object name
    bool load(
        const Alphabet          alphabet,
        const char*             file_name,
        const char*             mapped_name,
        const SequenceFlags     load_flags  = io::SequenceFlags( io::SEQUENCE_DATA | io::SEQUENCE_QUALS | io::SEQUENCE_NAMES ),
        const QualityEncoding   qualities   = Phred33);

    static std::string info_file_name(const char* name);
    static std::string sequence_file_name(const char* name);
    static std::string sequence_index_file_name(const char* name);
    static std::string qual_file_name(const char* name);
    static std::string name_file_name(const char* name);
    static std::string name_index_file_name(const char* name);

    ServerMappedFile m_info_file;                    ///< internal memory-mapped info object server
    ServerMappedFile m_sequence_file;                ///< internal memory-mapped genome object server
    ServerMappedFile m_sequence_index_file;          ///< internal memory-mapped forward occurrence table object server
    ServerMappedFile m_qual_file;                    ///< internal memory-mapped reverse occurrence table object server
    ServerMappedFile m_name_file;                    ///< internal memory-mapped forward BWT object server
    ServerMappedFile m_name_index_file;              ///< internal memory-mapped reverse BWT object server
};

///
/// A concrete SequenceData storage implementation in system mapped memory
///
struct SequenceDataMMAP : public SequenceData
{
    typedef SequenceDataView                  plain_view_type;
    typedef ConstSequenceDataView       const_plain_view_type;

    /// constructor
    ///
    SequenceDataMMAP() :
        m_sequence_ptr( NULL ),
        m_sequence_index_ptr( NULL ),
        m_qual_ptr( NULL ),
        m_name_ptr( NULL ),
        m_name_index_ptr( NULL )
        {}

    /// load from a memory mapped object
    ///
    /// \param name          memory mapped object name
    bool load(
        const char* name);

    /// convert to a plain_view
    ///
    operator plain_view_type()
    {
        return plain_view_type(
            static_cast<const SequenceDataInfo&>( *this ),
            m_sequence_ptr,
            m_sequence_index_ptr,
            m_qual_ptr,
            m_name_ptr,
            m_name_index_ptr );
    }
    /// convert to a const plain_view
    ///
    operator const_plain_view_type() const
    {
        return const_plain_view_type(
            static_cast<const SequenceDataInfo&>( *this ),
            m_sequence_ptr,
            m_sequence_index_ptr,
            m_qual_ptr,
            m_name_ptr,
            m_name_index_ptr );
    }

    uint32* m_sequence_ptr;
    uint32* m_sequence_index_ptr;
    char*   m_qual_ptr;
    char*   m_name_ptr;
    uint32* m_name_index_ptr;

    MappedFile m_info_file;                    ///< internal memory-mapped info object server
    MappedFile m_sequence_file;                ///< internal memory-mapped genome object server
    MappedFile m_sequence_index_file;          ///< internal memory-mapped forward occurrence table object server
    MappedFile m_qual_file;                    ///< internal memory-mapped reverse occurrence table object server
    MappedFile m_name_file;                    ///< internal memory-mapped forward BWT object server
    MappedFile m_name_index_file;              ///< internal memory-mapped reverse BWT object server
};

/// map a sequence file into mapped system memory
///
/// \param sequence_file_name   the file to open
/// \return                     a heap allocated SequenceDataMMAP object on success, NULL otherwise
///
SequenceDataMMAP* map_sequence_file(const char* sequence_file_name);

///@} // SequenceIO
///@} // IO

} // namespace io
} // namespace nvbio
