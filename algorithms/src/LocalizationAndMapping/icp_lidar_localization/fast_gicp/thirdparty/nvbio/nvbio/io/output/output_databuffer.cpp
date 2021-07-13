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

#include <nvbio/io/output/output_bam.h>
#include <nvbio/basic/numbers.h>

#include <stdio.h>
#include <stdarg.h>

namespace nvbio {
namespace io {

DataBuffer::DataBuffer()
    : pos(0)
{
    buffer.resize( BUFFER_SIZE + BUFFER_EXTRA );
    NVBIO_CUDA_ASSERT(buffer.size());
}

DataBuffer::~DataBuffer()
{}

void DataBuffer::append_data(const void *data, int size)
{
    NVBIO_CUDA_ASSERT(pos + size < BUFFER_SIZE + BUFFER_EXTRA);

    memcpy(&buffer[0] + pos, data, size);
    pos += size;
}

void DataBuffer::append_int32(int32 val)
{
    append_data(&val, sizeof(val));
}

void DataBuffer::append_uint32(uint32 val)
{
    append_data(&val, sizeof(val));
}

void DataBuffer::append_int8(int8 val)
{
    append_data(&val, sizeof(val));
}

void DataBuffer::append_uint8(uint8 val)
{
    append_data(&val, sizeof(val));
}

void DataBuffer::append_formatted_string(const char *fmt, ...)
{
    int bytes_left = BUFFER_SIZE + BUFFER_EXTRA - pos;
    int bytes_written;
    va_list args;

    va_start(args, fmt);
    bytes_written = vsnprintf(&buffer[pos], bytes_left, fmt, args);

    // if we hit the very end, the string was likely truncated
    NVBIO_CUDA_ASSERT(bytes_written < bytes_left);

    pos += bytes_written;
}

void DataBuffer::append_string(const char *str)
{
    append_data(str, strlen(str));
}

int DataBuffer::get_pos(void)
{
    return pos;
}

void DataBuffer::skip_ahead(int n)
{
    NVBIO_CUDA_ASSERT(pos + n < BUFFER_SIZE + BUFFER_EXTRA);

    pos += n;
}

void DataBuffer::poke_data(int offset, const void *data, int size)
{
    NVBIO_CUDA_ASSERT(offset + size < BUFFER_SIZE + BUFFER_EXTRA);

    memcpy(&buffer[0] + offset, data, size);
}

void DataBuffer::poke_int32(int offset, int32 val)
{
    poke_data(offset, &val, sizeof(val));
}

void DataBuffer::poke_uint16(int offset, uint16 val)
{
    poke_data(offset, &val, sizeof(val));
}

bool DataBuffer::is_full(void)
{
    if (pos > BUFFER_SIZE)
    {
        return true;
    } else {
        return false;
    }
}

void DataBuffer::rewind(void)
{
    pos = 0;
}

void *DataBuffer::get_base_ptr(void)
{
    return &buffer[0];
}

void *DataBuffer::get_cur_ptr(void)
{
    return &buffer[pos];
}

int DataBuffer::get_remaining_size(void)
{
    return BUFFER_SIZE + BUFFER_EXTRA - pos;
}

} // namespace io
} // namespace nvbio

