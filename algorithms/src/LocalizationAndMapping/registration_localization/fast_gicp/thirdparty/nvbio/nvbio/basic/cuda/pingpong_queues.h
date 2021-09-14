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

#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/thrust_view.h>

#pragma once

namespace nvbio {
namespace cuda {

/// \page ping_pong_queues_page Ping-Pong Queues
///
/// This module implements device <i>ping-pong queues</i>, i.e. a pair of input / output queues
/// built on top of some ping-pong device memory storage that gets swapped at each
/// iteration.
///
/// \section AtAGlanceSection At a Glance
///
/// - PingPongQueues
/// - PingPongQueuesView
///
/// \section PingPongQueuesExample Example
///
///\code
/// // copy even input-queue entries into the output-queue
/// __global__ void copy_even_kernel(PingPongQueuesView<uint32> queues)
/// {
///     const uint32 idx = threadIdx.x + blockIdx.x * blockDim.x;
///     if (idx >= queues.in_size) return;
///
///     // fetch an element from the input queue
///     const uint32 n = queues.in_queue[idx];
///
///     // if n is even, append it to the output
///     if ((n & 1) == 0)
///     {
///         const uint32 slot = atomicAdd( queues.out_size, 1u );
///         queues.out_queue[ slot ] = n;
///     }
/// }
///
/// PingPongQueues<uint32> queues;
///
/// // reserve some storage
/// queues.resize_arena( 1000 );
///
/// // resize the input queue
/// queues.resize( 1000 );
///
/// // fill the input queue with numbers 0,...,999
/// thrust::copy(
///     thrust::make_counting_iterator<uint32>(0),
///     thrust::make_counting_iterator<uint32>(1000),
///     thrust::device_ptr<uint32>( queues.raw_input_queue() ) );
///
/// while (queues.in_size)
/// {
///     // clear the output queue
///     queues.clear_output();
///
///     // run our kernel
///     copy_even_kernel<<<1,1000>>>( plain_view( queues ) );
///     cudaDeviceSynchronize();
///
///     // swap the input & output queues
///     queues.swap();
/// }
///\endcode
///    
/// \section TechnicalOverviewSection Technical Overview
///
/// See the \ref PingPongQueuesModule module documentation.

///@addtogroup Basic
///@{

///\defgroup PingPongQueuesModule Ping-Pong Queues
///
/// This module implements device <i>ping-pong queues</i>, i.e. a pair of input / output queues
/// built on top of some ping-pong device memory storage that gets swapped at each
/// iteration.
/// See \ref ping_pong_queues_page.
///

///@addtogroup PingPongQueuesModule
///@{

///
///
/// This data structure represents the device-side view a pair of input and
/// output queues built on top of some ping-pong device storage that can be
/// iteratively swapped.
///
template <typename T = uint32>
struct PingPongQueuesView
{
    uint32          in_size;
    const T*        in_queue;
    uint32*         out_size;
    T*              out_queue;
};

///
/// This data structure represents a pair of input and output queues
/// built on top of some ping-pong device storage that can be iteratively
/// swapped.
///
/// See \ref PingPongQueuesExample.
///
template <typename T = uint32>
struct PingPongQueues
{
    typedef PingPongQueuesView<T> device_view_type;
    typedef PingPongQueuesView<T> plain_view_type;

    uint32                          in_size;
    thrust::device_vector<T>        in_queue;
    thrust::device_vector<uint32>   out_size;
    thrust::device_vector<T>        out_queue;

    /// resize the arena
    ///
    uint64 resize_arena(const uint32 size, const bool do_alloc = true)
    {
        if (do_alloc)
        {
            in_size = 0;
            in_queue.resize( size );
            out_queue.resize( size );
            out_size.resize(1);
        }
        return 2u*size*sizeof(T) + sizeof(uint32);
    }

    /// resize input queue
    ///
    void resize(const uint32 size) { in_size = size; }

    /// clear the output queue
    ///
    void clear_output() { out_size[0] = 0; }

    /// swap the input and output queues
    ///
    void swap()
    {
        in_size = out_size[0];
        in_queue.swap( out_queue );
    }

    /// raw input queue
    ///
    const T* raw_input_queue() const { return nvbio::device_view( in_queue ); }

    /// raw output queue
    ///
    const T* raw_output_queue() const { return nvbio::device_view( out_queue ); }

    /// raw output queue
    ///
    T* raw_output_queue() { return nvbio::device_view( out_queue ); }

    /// return the output size
    ///
    uint32 output_size() const { return out_size[0]; }

    /// return a view of the queues
    ///
    device_view_type device_view()
    {
        device_view_type q;
        q.in_size   = in_size;
        q.in_queue  = nvbio::device_view( in_queue );
        q.out_size  = nvbio::device_view( out_size );
        q.out_queue = nvbio::device_view( out_queue );
        return q;
    }
};

///@} // PingPongQueuesModule
///@} // Basic

} // namespace cuda

///\relates cuda::PingPongQueues
/// return a view of the queues
///
template <typename T>
cuda::PingPongQueuesView<T> device_view(cuda::PingPongQueues<T>& queues) { return queues.device_view(); }

///\relates cuda::PingPongQueues
/// return a view of the queues
///
template <typename T>
cuda::PingPongQueuesView<T> plain_view(cuda::PingPongQueues<T>& queues) { return queues.device_view(); }

} // namespace nvbio
