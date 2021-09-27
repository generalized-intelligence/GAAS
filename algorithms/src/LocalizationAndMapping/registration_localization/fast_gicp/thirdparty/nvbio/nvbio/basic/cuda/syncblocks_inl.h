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
#include <nvbio/basic/numbers.h>
#include <thrust/device_vector.h>
#include <thrust/iterator/constant_iterator.h>

namespace nvbio {
namespace cuda {

// constructor
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
syncblocks::syncblocks(int32* counter) : m_counter( counter ) {}

// implements an inter-CTA synchronization primitive
//
NVBIO_FORCEINLINE NVBIO_DEVICE
bool syncblocks::enact(const uint32 max_iter)
{
    __threadfence();
    __syncthreads();

    // each block does an atomicAdd on an integer, waiting for all CTAs to be
    // counted. When this happens, a global semaphore is released.
    // The CTA counter is always increased across multiple calls to syncblocks,
    // so that its value will say which syncblocks each CTA is participating
    // too.
    // Similarly, the semaphore is always increasing. As soon as the semaphore
    // is higher than the syncblocks a CTA has just entered, the semaphore is
    // considered 'released' for that syncblocks.
    __shared__ volatile bool ret;
    if (threadIdx.x == 0)
    {
        const uint32 grid_size = gridDim.x * gridDim.y * gridDim.z;

        int32* semaphore = (m_counter + 1);

        // add 1 atomically to the shared counter
        const uint32 slot = atomicAdd( m_counter, 1 );

        // compute which syncblocks we are particpating too based on the result we got from the atomicAdd
        const uint32 iteration = slot / grid_size;

        const bool is_last_block = (slot - iteration*grid_size) == (grid_size-1);
        if (is_last_block)
        {
            // release the semaphore
            atomicAdd( semaphore, 1 );
        }

        // wait for the semaphore write to become public
        __threadfence();

        // spin until the semaphore is released
        for (uint32 iter = 0; iter < max_iter && *(volatile int32*)semaphore <= iteration; ++iter) {}

        ret = (*(volatile int32*)semaphore > iteration);
    }

    // synchronize all threads in this CTA
    __syncthreads();
    return ret;
}

// constructor
//
inline syncblocks_storage::syncblocks_storage()
{
    // alloc a counter and a semaphore
    m_counter.resize( 2, 0 );
}

// return a syncblocks object
//
inline syncblocks syncblocks_storage::get()
{
    return syncblocks( thrust::raw_pointer_cast( &m_counter.front() ) );
}

// clear the syncblocks, useful if one wants to reuse it
// across differently sized kernel launches.
//
inline void syncblocks_storage::clear()
{
    thrust::fill( m_counter.begin(), m_counter.end(), 0 );
}

} // namespace cuda
} // namespace nvbio
