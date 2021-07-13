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

#include <nvbio/io/sequence/sequence_mmap.h>
#include <nvbio/io/sequence/sequence_pac.h>
#include <nvbio/basic/console.h>

namespace nvbio {
namespace io {

// load a sequence from file
//
// \param alphabet                 the alphabet to use for encoding
// \param prefix                   prefix file name
// \param mapped_name              memory mapped object name
bool SequenceDataMMAPServer::load(
    const Alphabet          alphabet,
    const char*             file_name,
    const char*             mapped_name,
    const SequenceFlags     load_flags,
    const QualityEncoding   qualities)
{
    log_visible(stderr, "SequenceDataMMAPServer::loading... started\n");

    // TODO: check the extension; if there's no extension, assume it's a pac index
    bool r = load_pac( alphabet, this, file_name, mapped_name, load_flags, qualities );

    log_visible(stderr, "SequenceDataMMAPServer::loading... done\n");
    return r;
}

std::string SequenceDataMMAPServer::info_file_name(const char* name)            { return std::string("nvbio.") + std::string( name ) + ".seq_info";}
std::string SequenceDataMMAPServer::sequence_file_name(const char* name)        { return std::string("nvbio.") + std::string( name ) + ".seq"; }
std::string SequenceDataMMAPServer::sequence_index_file_name(const char* name)  { return std::string("nvbio.") + std::string( name ) + ".seq_index"; }
std::string SequenceDataMMAPServer::qual_file_name(const char* name)            { return std::string("nvbio.") + std::string( name ) + ".qual"; }
std::string SequenceDataMMAPServer::name_file_name(const char* name)            { return std::string("nvbio.") + std::string( name ) + ".name"; }
std::string SequenceDataMMAPServer::name_index_file_name(const char* name)      { return std::string("nvbio.") + std::string( name ) + ".name_index"; }

// load from a memory mapped object
//
// \param name          memory mapped object name
bool SequenceDataMMAP::load(const char* file_name)
{
    std::string infoName        = SequenceDataMMAPServer::info_file_name( file_name );
    std::string seqName         = SequenceDataMMAPServer::sequence_file_name( file_name );
    std::string seqIndexName    = SequenceDataMMAPServer::sequence_index_file_name( file_name );
    std::string qualName        = SequenceDataMMAPServer::qual_file_name( file_name );
    std::string nameName        = SequenceDataMMAPServer::name_file_name( file_name );
    std::string nameIndexName   = SequenceDataMMAPServer::name_index_file_name( file_name );

    try {
        const SequenceDataInfo* info = (const SequenceDataInfo*)m_info_file.init( infoName.c_str(), sizeof(SequenceDataInfo) );

        this->SequenceDataInfo::operator=( *info );

        const uint64 index_file_size    = info->size()  * sizeof(uint32);
        const uint64 seq_file_size      = info->words() * sizeof(uint32);
        const uint64 qual_file_size     = info->qs()    * sizeof(char);
        const uint64 name_file_size     = info->m_name_stream_len * sizeof(char);

        m_sequence_ptr        = (uint32*)m_sequence_file.init( seqName.c_str(), seq_file_size );
        m_sequence_index_ptr  = (uint32*)m_sequence_index_file.init( seqIndexName.c_str(), index_file_size );
        m_qual_ptr            = qual_file_size ? (char*)m_qual_file.init( qualName.c_str(), qual_file_size ) : NULL;
        m_name_ptr            =   (char*)m_name_file.init( nameName.c_str(), name_file_size );
        m_name_index_ptr      = (uint32*)m_sequence_index_file.init( nameIndexName.c_str(), index_file_size );
    }
    catch (MappedFile::mapping_error error)
    {
        log_info(stderr, "SequenceDataMMAP: error mapping file \"%s\" (%d)!\n", error.m_file_name, error.m_code);
        return false;
    }
    catch (MappedFile::view_error error)
    {
        log_info(stderr, "SequenceDataMMAP: error viewing file \"%s\" (%d)!\n", error.m_file_name, error.m_code);
        return false;
    }
    catch (...)
    {
        log_info(stderr, "SequenceDataMMAP: error mapping file (unknown)!\n");
        return false;
    }
    return true;
}

// map a sequence file
//
// \param sequence_file_name   the file to open
//
SequenceDataMMAP* map_sequence_file(const char* sequence_file_name)
{
    SequenceDataMMAP* ret = new SequenceDataMMAP; 
    if (ret->load( sequence_file_name ) == false)
    {
        delete ret;
        return NULL;
    }
    return ret;
}

} // namespace io
} // namespace nvbio
