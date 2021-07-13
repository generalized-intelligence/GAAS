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

#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/thrust_view.h>
#include <cub/cub.cuh>

namespace nvbio {
namespace cuda {

SortEnactor::SortEnactor()
{
    m_impl = NULL; // we might want to use this later for the temp storage
}
SortEnactor::~SortEnactor()
{
}

void SortEnactor::sort(const uint32 count, SortBuffers<uint8*, uint32*>& buffers, const uint32 begin_bit, const uint32 end_bit)
{
    cub::DoubleBuffer<uint8>  key_buffers;
    cub::DoubleBuffer<uint32> value_buffers;

    // Create ping-pong storage wrapper
    key_buffers.selector       = buffers.selector;
    value_buffers.selector     = buffers.selector;
    key_buffers.d_buffers[0]   = buffers.keys[0];
    key_buffers.d_buffers[1]   = buffers.keys[1];
    value_buffers.d_buffers[0] = buffers.values[0];
    value_buffers.d_buffers[1] = buffers.values[1];

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortPairs( NULL, temp_storage_bytes, key_buffers, value_buffers, count, begin_bit, end_bit );

    thrust::device_vector<uint8> d_temp( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortPairs( nvbio::plain_view( d_temp ), temp_storage_bytes, key_buffers, value_buffers, count, begin_bit, end_bit );

    // keep track of the current buffer
    buffers.selector = key_buffers.selector;
}
void SortEnactor::sort(const uint32 count, SortBuffers<uint16*,uint32*>& buffers, const uint32 begin_bit, const uint32 end_bit)
{
    cub::DoubleBuffer<uint16> key_buffers;
    cub::DoubleBuffer<uint32> value_buffers;

    // Create ping-pong storage wrapper
    key_buffers.selector       = buffers.selector;
    value_buffers.selector     = buffers.selector;
    key_buffers.d_buffers[0]   = buffers.keys[0];
    key_buffers.d_buffers[1]   = buffers.keys[1];
    value_buffers.d_buffers[0] = buffers.values[0];
    value_buffers.d_buffers[1] = buffers.values[1];

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortPairs( NULL, temp_storage_bytes, key_buffers, value_buffers, count, begin_bit, end_bit );

    thrust::device_vector<uint8> d_temp( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortPairs( nvbio::plain_view( d_temp ), temp_storage_bytes, key_buffers, value_buffers, count, begin_bit, end_bit );

    // keep track of the current buffer
    buffers.selector = key_buffers.selector;
}
void SortEnactor::sort(const uint32 count, SortBuffers<uint32*,uint32*>& buffers, const uint32 begin_bit, const uint32 end_bit)
{
    cub::DoubleBuffer<uint32> key_buffers;
    cub::DoubleBuffer<uint32> value_buffers;

    // Create ping-pong storage wrapper
    key_buffers.selector       = buffers.selector;
    value_buffers.selector     = buffers.selector;
    key_buffers.d_buffers[0]   = buffers.keys[0];
    key_buffers.d_buffers[1]   = buffers.keys[1];
    value_buffers.d_buffers[0] = buffers.values[0];
    value_buffers.d_buffers[1] = buffers.values[1];

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortPairs( NULL, temp_storage_bytes, key_buffers, value_buffers, count, begin_bit, end_bit );

    thrust::device_vector<uint8> d_temp( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortPairs( nvbio::plain_view( d_temp ), temp_storage_bytes, key_buffers, value_buffers, count, begin_bit, end_bit );

    // keep track of the current buffer
    buffers.selector = key_buffers.selector;
}
void SortEnactor::sort(const uint32 count, SortBuffers<uint32*,uint64*>& buffers, const uint32 begin_bit, const uint32 end_bit)
{
    cub::DoubleBuffer<uint32> key_buffers;
    cub::DoubleBuffer<uint64> value_buffers;

    // Create ping-pong storage wrapper
    key_buffers.selector       = buffers.selector;
    value_buffers.selector     = buffers.selector;
    key_buffers.d_buffers[0]   = buffers.keys[0];
    key_buffers.d_buffers[1]   = buffers.keys[1];
    value_buffers.d_buffers[0] = buffers.values[0];
    value_buffers.d_buffers[1] = buffers.values[1];

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortPairs( NULL, temp_storage_bytes, key_buffers, value_buffers, count, begin_bit, end_bit );

    thrust::device_vector<uint8> d_temp( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortPairs( nvbio::plain_view( d_temp ), temp_storage_bytes, key_buffers, value_buffers, count, begin_bit, end_bit );

    // keep track of the current buffer
    buffers.selector = key_buffers.selector;
}
void SortEnactor::sort(const uint32 count, SortBuffers<uint64*,uint32*>& buffers, const uint32 begin_bit, const uint32 end_bit)
{
    cub::DoubleBuffer<uint64> key_buffers;
    cub::DoubleBuffer<uint32> value_buffers;

    // Create ping-pong storage wrapper
    key_buffers.selector       = buffers.selector;
    value_buffers.selector     = buffers.selector;
    key_buffers.d_buffers[0]   = buffers.keys[0];
    key_buffers.d_buffers[1]   = buffers.keys[1];
    value_buffers.d_buffers[0] = buffers.values[0];
    value_buffers.d_buffers[1] = buffers.values[1];

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortPairs( NULL, temp_storage_bytes, key_buffers, value_buffers, count, begin_bit, end_bit );

    thrust::device_vector<uint8> d_temp( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortPairs( nvbio::plain_view( d_temp ), temp_storage_bytes, key_buffers, value_buffers, count, begin_bit, end_bit );

    // keep track of the current buffer
    buffers.selector = key_buffers.selector;
}

void SortEnactor::sort(const uint32 count, SortBuffers<uint8*>& buffers, const uint32 begin_bit, const uint32 end_bit)
{
    cub::DoubleBuffer<uint8> key_buffers;

    // Create ping-pong storage wrapper
    key_buffers.selector       = buffers.selector;
    key_buffers.d_buffers[0]   = buffers.keys[0];
    key_buffers.d_buffers[1]   = buffers.keys[1];

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortKeys( NULL, temp_storage_bytes, key_buffers, count, begin_bit, end_bit );

    thrust::device_vector<uint8> d_temp( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortKeys( nvbio::plain_view( d_temp ), temp_storage_bytes, key_buffers, count, begin_bit, end_bit );

    // keep track of the current buffer
    buffers.selector = key_buffers.selector;

}
void SortEnactor::sort(const uint32 count, SortBuffers<uint16*>& buffers, const uint32 begin_bit, const uint32 end_bit)
{
    cub::DoubleBuffer<uint16> key_buffers;

    // Create ping-pong storage wrapper
    key_buffers.selector       = buffers.selector;
    key_buffers.d_buffers[0]   = buffers.keys[0];
    key_buffers.d_buffers[1]   = buffers.keys[1];

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortKeys( NULL, temp_storage_bytes, key_buffers, count, begin_bit, end_bit );

    thrust::device_vector<uint8> d_temp( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortKeys( nvbio::plain_view( d_temp ), temp_storage_bytes, key_buffers, count, begin_bit, end_bit );

    // keep track of the current buffer
    buffers.selector = key_buffers.selector;
}
void SortEnactor::sort(const uint32 count, SortBuffers<uint32*>& buffers, const uint32 begin_bit, const uint32 end_bit)
{
    cub::DoubleBuffer<uint32> key_buffers;

    // Create ping-pong storage wrapper
    key_buffers.selector       = buffers.selector;
    key_buffers.d_buffers[0]   = buffers.keys[0];
    key_buffers.d_buffers[1]   = buffers.keys[1];

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortKeys( NULL, temp_storage_bytes, key_buffers, count, begin_bit, end_bit );

    thrust::device_vector<uint8> d_temp( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortKeys( nvbio::plain_view( d_temp ), temp_storage_bytes, key_buffers, count, begin_bit, end_bit );

    // keep track of the current buffer
    buffers.selector = key_buffers.selector;
}
void SortEnactor::sort(const uint32 count, SortBuffers<uint64*>& buffers, const uint32 begin_bit, const uint32 end_bit)
{
    cub::DoubleBuffer<uint64> key_buffers;

    // Create ping-pong storage wrapper
    key_buffers.selector       = buffers.selector;
    key_buffers.d_buffers[0]   = buffers.keys[0];
    key_buffers.d_buffers[1]   = buffers.keys[1];

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortKeys( NULL, temp_storage_bytes, key_buffers, count, begin_bit, end_bit );

    thrust::device_vector<uint8> d_temp( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortKeys( nvbio::plain_view( d_temp ), temp_storage_bytes, key_buffers, count, begin_bit, end_bit );

    // keep track of the current buffer
    buffers.selector = key_buffers.selector;
}

} // namespace cuda
} // namespace nvbio
