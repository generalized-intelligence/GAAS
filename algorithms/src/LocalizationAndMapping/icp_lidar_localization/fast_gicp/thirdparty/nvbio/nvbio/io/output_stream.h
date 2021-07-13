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

#include <nvbio/basic/types.h>
#include <vector>

#pragma once

namespace nvbio {

///@addtogroup IO
///@{

/// Base abstract output file class
///
struct OutputStream
{
    /// virtual destructor
    ///
    virtual ~OutputStream() {}

    /// write a given number of bytes
    ///
    virtual uint32 write(const uint32 bytes, const void* buffer) { return 0; }

    /// is valid?
    ///
    virtual bool is_valid() const { return true; }
};

// Base abstract output file class
//
struct GZOutputFile : public OutputStream
{
    /// constructor
    ///
    GZOutputFile(const char* name, const char* comp);

    /// destructor
    ///
    ~GZOutputFile();

    /// write a given number of bytes
    ///
    uint32 write(const uint32 bytes, const void* buffer);

    /// is valid?
    ///
    bool is_valid() const { return m_file != NULL; }

    void* m_file;
};

// Base abstract output file class
//
struct LZ4OutputFile : public OutputStream
{
    /// constructor
    ///
    LZ4OutputFile(const char* name, const char* comp);

    /// destructor
    ///
    ~LZ4OutputFile();

    /// write a given number of bytes
    ///
    uint32 write(const uint32 bytes, const void* buffer);

    /// is valid?
    ///
    bool is_valid() const { return m_file != NULL; }

    void*               m_context;
    void*               m_file;
    std::vector<uint8>  m_buffer;
};

/// output file factory method
///
OutputStream* open_output_file(const char* file_name, const char* compressor, const char* options);

///@} // IO

} // namespace nvbio
