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

namespace nvbio {
namespace cuda {

/// implements an inter-CTA synchronization primitive which can be called
/// multiple times from the same grid, or even across multiple kernel
/// launches, as long as all kernel launches have the same size.
///
/// ACHTUNG!
/// this primitive is NOT SAFE, and only works if all CTAs are resident!
///
struct syncblocks
{
    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    syncblocks(int32* counter = NULL);

    /// enact the syncblocks
    ///
    /// \return     true on successful completion, false otherwise
    ///
    NVBIO_FORCEINLINE NVBIO_DEVICE
    bool enact(const uint32 max_iter = uint32(-1));

    int32* m_counter;
};

/// a class to build a syncblocks primitive from the host
///
struct syncblocks_storage
{
    /// constructor
    ///
    syncblocks_storage();

    /// return a syncblocks object
    ///
    syncblocks get();

    /// clear the syncblocks, useful if one wants to reuse it
    /// across differently sized kernel launches.
    ///
    void clear();

private:
    thrust::device_vector<int32> m_counter;
};

} // namespace cuda
} // namespace nvbio

#include <nvbio/basic/cuda/syncblocks_inl.h>
