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

#include <nvbio/basic/types.h>

namespace nvbio {

/// \page memory_mapping_page Memory Mapping
///
/// This module implements basic server-client memory mapping functionality
///
/// \section AtAGlanceSection At a Glance
///
/// - MappedFile
/// - ServerMappedFile
///
/// \section MMAPExampleSection Example
///
/// A typical scenario would be to have a server process map a region of its own memory so
/// that other processes can access it:
///\code
/// // server program
/// void main()
/// {
///   // map a block of 100MB
///   ServerMappedFile* mapped_file = new ServerMappedFile();
///   mapped_file->init("my_file", 100*1024*1024, my_buffer);
///
///   // and loop until somebody tell us to stop
///   while (exit_signal() == false) {}
///
///   // delete the mapped_file object, releasing the memory mapping
///   delete mapped_file;
/// }
///\endcode
///
/// and a client read that mapped file:
///\code
/// // client program
/// void main()
/// {
///   // map the shared block in the client's memory space
///   MappedFile mapped_file;
///   void* mapped_buffer = mapped_file.init("my_file", 100*1024*1024);
///
///   // and do something with it
///   do_something( mapped_buffer );
/// }
///\endcode
///
/// \section TechnicalOverviewSection Technical Overview
///
/// See the \ref MemoryMappingModule module documentation.
///

///@addtogroup Basic
///@{

///@defgroup MemoryMappingModule Memory Mapping
/// This module implements basic server-client memory mapping functionality
///@{

///
/// A class to map a memory object into a client process.
/// See ServerMappedFile.
///
struct MappedFile
{
    struct mapping_error
    {
        mapping_error(const char* name, int32 code) : m_file_name( name ), m_code( code ) {}

        const char* m_file_name;
        int32       m_code;
    };
    struct view_error
    {
        view_error(const char* name, uint32 code) : m_file_name( name ), m_code( code ) {}

        const char* m_file_name;
        int32       m_code;
    };

    /// constructor
    ///
    MappedFile();

    /// destructor
    ///
    ~MappedFile();

    /// initialize the memory mapped file
    ///
    void* init(const char* name, const uint64 file_size);

private:
    struct Impl;
    Impl* impl;
};

///
/// A class to create a memory mapped object into a server process. The mapped file is destroyed
/// when the destructor is called.
/// See MappedFile.
///
struct ServerMappedFile
{
    struct mapping_error
    {
        mapping_error(const char* name, int32 code) : m_file_name( name ), m_code( code ) {}

        const char* m_file_name;
        int32       m_code;
    };
    struct view_error
    {
        view_error(const char* name, uint32 code) : m_file_name( name ), m_code( code ) {}

        const char* m_file_name;
        int32       m_code;
    };

    /// constructor
    ///
    ServerMappedFile();

    /// destructor
    ///
    ~ServerMappedFile();

    /// initialize the memory mapped file
    void* init(const char* name, const uint64 file_size, const void* src);

private:
    struct Impl;
    Impl* impl;
};

///@} MemoryMappingModule
///@} Basic

} // namespace nvbio
