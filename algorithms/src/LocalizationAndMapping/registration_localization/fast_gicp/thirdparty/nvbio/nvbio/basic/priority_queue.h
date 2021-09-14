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

/*! \file priority_queue.h
 *   \brief A CUDA-compatible, fixed-size priority queue
 */

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/popcount.h>

namespace nvbio {

/// \page priority_queues_page Priority Queues
///
/// This module implements a priority queue adaptor, supporting O(log(N)) push/pop operations.
/// Unlike std::priority_queue, this class can be used both in host and device CUDA code:
///
/// - priority_queue
///
/// \section ExampleSection Example
///
///\code
/// // build a simple priority_queue over 4 integers
/// typedef vector_view<uint32*>                vector_type;
/// typedef priority_queue<uint32, vector_type> queue_type;
///
/// uint32 queue_storage[4];
///
/// // construct the queue
/// queue_type queue( vector_type( 0u, queue_storage ) );
///
/// // push a few items
/// queue.push( 3 );
/// queue.push( 8 );
/// queue.push( 1 );
/// queue.push( 5 );
///
/// // pop from the top
/// printf( "%u\n", queue.top() );      // -> 8
/// queue.pop();
/// printf( "%u\n", queue.top() );      // -> 5
/// queue.pop();
/// printf( "%u\n", queue.top() );      // -> 3
/// queue.pop();
/// printf( "%u\n", queue.top() );      // -> 1
///\endcode
///

///@addtogroup Basic
///@{

///@defgroup PriorityQueues Priority Queues
/// This module implements a priority queue adaptor.
///@{
    
///
/// A priority queue
///
/// \tparam Key         the key type
/// \tparam Container   the underlying container used to hold keys, must implement push_back(), size(), resize(), clear()
/// \tparam Compare     the comparison binary functor, Compare(a,b) == true iff a < b
///
template <typename Key, typename Container, typename Compare>
struct priority_queue
{
    typedef Key                                         value_type;
    typedef Container                                   container_type;
    typedef typename container_type::const_iterator     const_iterator;
    typedef const_iterator                              iterator;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE priority_queue(Container cont = Container(), const Compare cmp = Compare());

    /// is queue empty?
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool  empty() const;

    /// return queue size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE uint32 size() const;

    /// push an element
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void push(const Key key);

    /// pop an element
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void pop();

    /// top of the queue
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Key& top();

    /// top of the queue
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Key top() const;

    /// return the i-th element in the heap
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE const Key& operator[] (const uint32 i) const;

    /// clear the queue
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void clear();

    /// starting iterator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    const_iterator begin() const { return m_queue.size() ? m_queue.begin() + 1u : m_queue.end(); }

    /// starting iterator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    iterator begin() { return m_queue.size() ? m_queue.begin() + 1u : m_queue.end(); }

    /// ending iterator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    const_iterator end() const { return m_queue.end(); }

    /// ending iterator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    iterator end() { return m_queue.end(); }

    /// locate the largest element v such that v <= x; return end() if no
    /// such element exists
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE iterator upper_bound(const Key& x);

    uint32      m_size;
    Container   m_queue;
    Compare     m_cmp;
};

///@} PriorityQueues
///@} Basic

} // namespace nvbio

#include <nvbio/basic/priority_queue_inline.h>
