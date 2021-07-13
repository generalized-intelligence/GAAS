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

#include <nvbio/sufsort/file_bwt.h>

namespace nvbio {

struct BGZFileWriter
{
    /// constructor
    ///
    BGZFileWriter(FILE* _file = NULL);

    /// destructor
    ///
    ~BGZFileWriter();

    /// open a session
    ///
    void open(FILE* _file, const int level, const int strategy);

    /// close a session
    ///
    void close();

    /// write a block to the output
    ///
    void write(uint32 n_bytes, const void* _src);

private:
    /// encode a given block and write it to the output
    ///
    void encode_block(uint32 n_bytes, const uint8* src);

    /// compress a given block
    ///
    uint32 compress(const uint8* src, uint8* dst, const uint32 n_bytes);

    FILE*              m_file;
    std::vector<uint8> m_buffer;
    std::vector<uint8> m_comp_buffer;
    uint32             m_buffer_size;
    int                m_level;
    int                m_strategy;
};

/// A class to output the BWT to an BGZ-compressed binary file
///
struct BWTBGZWriter
{
    /// constructor
    ///
    BWTBGZWriter();

    /// destructor
    ///
    ~BWTBGZWriter();

    /// open
    void open(const char* output_name, const char* index_name, const char* compression);

    /// write to the bwt
    ///
    uint32 bwt_write(const uint32 n_bytes, const void* buffer);

    /// write to the index
    ///
    uint32 index_write(const uint32 n_bytes, const void* buffer);

    /// return whether the file is in a good state
    ///
    bool is_ok() const;

private:
    FILE*           output_file;
    FILE*           index_file;
    BGZFileWriter   output_file_writer;
    BGZFileWriter   index_file_writer;
};

} // namespace nvbio
