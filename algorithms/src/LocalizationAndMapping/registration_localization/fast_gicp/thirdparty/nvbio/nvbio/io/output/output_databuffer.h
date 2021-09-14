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

#include <nvbio/io/output/output_types.h>
#include <vector>

#include <stdio.h>

namespace nvbio {
namespace io {

// xxxnsubtil: BAM is little-endian, and so is x86/amd64
// we do no endinaness conversion here, though we provide
// methods to fill out integers in case we ever need to
struct DataBuffer
{
    // the maximum size for a BGZF block is 64kb + 1, which seems like a horrible design decision
    static const int BUFFER_SIZE = 60 * 1024;
    static const int BUFFER_EXTRA = 64 * 1024 - BUFFER_SIZE;

    std::vector<char> buffer;
    int pos;

    DataBuffer();
    ~DataBuffer();

    // append raw data to the buffer
    void append_data(const void *data, int size);
    // apend integral values to the data buffer
    // we explicitly implement each of these instead of using a template to make sure we always
    // output the correct types
    void append_int32(int32 value);
    void append_uint32(uint32 value);
    void append_int8(int8 value);
    void append_uint8(uint8 value);

    // append a formatted string to the buffer (note: this can fail if there's too much data!)
    void append_formatted_string(const char *fmt, ...);
    // append a plain string to the buffer
    void append_string(const char *str);
    // append a linebreak
    void append_linebreak();

    // get current offset
    int get_pos(void);
    // move write pointer forward by num bytes
    void skip_ahead(int n);
    // poke data at a given offset
    void poke_data(int offset, const void *data, int size);
    void poke_int32(int offset, int32 val);
    void poke_uint16(int offset, uint16 val);
    // check whether buffer is full
    bool is_full(void);

    // grab the current pointer
    void *get_cur_ptr(void);
    // grab the base pointer
    void *get_base_ptr(void);
    // get number of bytes remaining in this buffer
    int get_remaining_size(void);

    // rewind pos back to 0
    void rewind(void);
};

} // namespace io
} // namespace nvbio
