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

/*! \file sort.h
 *   \brief Define CUDA based sort primitives.
 */

#pragma once

#include <nvbio/basic/types.h>

namespace nvbio {
namespace cuda {

/// \page sorting_page Sorting
///\par
/// The SortEnactor provides a convenient wrapper around the fastest CUDA sorting library available,
/// allowing to perform both key-only and key-value pair sorting of arrays with the following
/// data-types:
///\par
/// - uint8
/// - uint16
/// - uint32
/// - uint64
/// - (uint8,uint32)
/// - (uint16,uint32)
/// - (uint32,uint32)
/// - (uint64,uint32)
///\par
/// The way most parallel sorting algorithms work require having a set of ping-pong buffers
/// that are exchanged at every pass through the data.
/// In order to do this, and communicate where the sorted data lies after its work, SortEnactor 
/// employs an auxiliary class, SortBuffers.
/// The following example shows their combined usage.
///
///\code
/// void sort_test(const uint32 n, uint32* h_keys, uint32* h_data)
/// {
///     // allocate twice as much storage as the input to accomodate for ping-pong buffers
///     nvbio::vector<device_tag,uint32> d_keys( n * 2 );
///     nvbio::vector<device_tag,uint32> d_data( n * 2 );
///
///     // copy the test data to the host
///     thrust::copy( h_keys, h_keys + n, d_keys.begin() );
///     thrust::copy( h_data, h_data + n, d_data.begin() );
///
///     // prepare the sorting buffers
///     cuda::SortBuffers<uint32*,uint32*> sort_buffers;
///     sort_buffers.keys[0] = raw_pointer( d_keys );
///     sort_buffers.keys[1] = raw_pointer( d_keys ) + n;
///     sort_buffers.data[0] = raw_pointer( d_data );
///     sort_buffers.data[1] = raw_pointer( d_data ) + n;
///
///     // and sort the data
///     cuda::SortEnactor sort_enactor;
///     sort_enactor.sort( n, sort_buffers );
///
///     // the sorted device data is now in here:
///     uint32* d_sorted_keys = sort_buffers.current_keys();
///     uint32* d_sorted_data = sort_buffers.current_values();
///     ...
/// }
///\endcode
///

///@addtogroup Basic
///@{

///@defgroup SortEnactors Sort Enactors
/// This module implements simple classes to sort device-memory buffers of key/value pairs of various primitive types.
///@{

/// A sorting buffer to hold vectors of key-value pairs
///
template <typename Keys, typename Values = null_type>
struct SortBuffers
{
    /// constructor
    ///
    SortBuffers() : selector(0) {}

    /// return the currently selected keys
    ///
    Keys current_keys() const { return keys[ selector ]; }

    /// return the currently selected values
    ///
    Values current_values() const { return values[ selector ]; }

    uint32  selector;
    Keys    keys[2];
    Values  values[2];
};

///\par
/// A simple class to enact sorts of various kinds
///\par
/// The way most parallel sorting algorithms work require having a set of ping-pong buffers
/// that are exchanged at every pass through the data.
/// In order to do this, and communicate where the sorted data lies after its work, SortEnactor 
/// employs an auxiliary class, SortBuffers.
/// The following example shows their combined usage.
///
///\code
/// void sort_test(const uint32 n, uint32* h_keys, uint32* h_data)
/// {
///     // allocate twice as much storage as the input to accomodate for ping-pong buffers
///     nvbio::vector<device_tag,uint32> d_keys( n * 2 );
///     nvbio::vector<device_tag,uint32> d_data( n * 2 );
///
///     // copy the test data to the host
///     thrust::copy( h_keys, h_keys + n, d_keys.begin() );
///     thrust::copy( h_data, h_data + n, d_data.begin() );
///
///     // prepare the sorting buffers
///     cuda::SortBuffers<uint32*,uint32*> sort_buffers;
///     sort_buffers.keys[0] = raw_pointer( d_keys );
///     sort_buffers.keys[1] = raw_pointer( d_keys ) + n;
///     sort_buffers.data[0] = raw_pointer( d_data );
///     sort_buffers.data[1] = raw_pointer( d_data ) + n;
///
///     // and sort the data
///     cuda::SortEnactor sort_enactor;
///     sort_enactor.sort( n, sort_buffers );
///
///     // the sorted device data is now in here:
///     uint32* d_sorted_keys = sort_buffers.current_keys();
///     uint32* d_sorted_data = sort_buffers.current_values();
///     ...
/// }
///\endcode
///
struct SortEnactor
{
    /// constructor
    ///
    SortEnactor();

    /// destructor
    ///
    ~SortEnactor();

    void sort(const uint32 count, SortBuffers<uint8*, uint32*>& buffers, const uint32 begin_bit = 0, const uint32 end_bit = 8);
    void sort(const uint32 count, SortBuffers<uint16*,uint32*>& buffers, const uint32 begin_bit = 0, const uint32 end_bit = 16);
    void sort(const uint32 count, SortBuffers<uint32*,uint32*>& buffers, const uint32 begin_bit = 0, const uint32 end_bit = 32);
    void sort(const uint32 count, SortBuffers<uint32*,uint64*>& buffers, const uint32 begin_bit = 0, const uint32 end_bit = 32);
    void sort(const uint32 count, SortBuffers<uint64*,uint32*>& buffers, const uint32 begin_bit = 0, const uint32 end_bit = 64);
    void sort(const uint32 count, SortBuffers<uint8*>&          buffers, const uint32 begin_bit = 0, const uint32 end_bit = 8);
    void sort(const uint32 count, SortBuffers<uint16*>&         buffers, const uint32 begin_bit = 0, const uint32 end_bit = 16);
    void sort(const uint32 count, SortBuffers<uint32*>&         buffers, const uint32 begin_bit = 0, const uint32 end_bit = 32);
    void sort(const uint32 count, SortBuffers<uint64*>&         buffers, const uint32 begin_bit = 0, const uint32 end_bit = 64);

private:
    void*  m_impl;
};

///@} SortEnactors
///@} Basic

} // namespace cuda
} // namespace nvbio
