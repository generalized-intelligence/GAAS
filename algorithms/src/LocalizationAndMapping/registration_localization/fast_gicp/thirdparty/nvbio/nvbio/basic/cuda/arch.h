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
#include <nvbio/basic/console.h>
#include <nvbio/basic/exceptions.h>
#include <cuda_runtime.h>
#include <thrust/version.h>

// used for thrust_copy_dtoh only
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>

namespace nvbio {
namespace cuda {

struct Arch
{
    static const uint32 LOG_WARP_SIZE = 5;
    static const uint32 WARP_SIZE     = 1u << LOG_WARP_SIZE;
};

// granularity of shared memory allocation (for the current device)
inline void device_arch(uint32& major, uint32& minor);

// granularity of the maximum grid size (for the current device)
inline uint32 max_grid_size();

// number of multiprocessors (for the current device)
inline size_t multiprocessor_count();

// granularity of shared memory allocation
inline size_t smem_allocation_unit(const cudaDeviceProp& properties);

// granularity of register allocation
inline size_t reg_allocation_unit(const cudaDeviceProp& properties, const size_t regsPerThread);

// granularity of warp allocation
inline size_t warp_allocation_multiple(const cudaDeviceProp& properties);

// number of "sides" into which the multiprocessor is partitioned
inline size_t num_sides_per_multiprocessor(const cudaDeviceProp& properties);

// maximum number of blocks per multiprocessor
inline size_t max_blocks_per_multiprocessor(const cudaDeviceProp& properties);

// number of registers allocated per block
inline size_t num_regs_per_block(const cudaDeviceProp& properties, const cudaFuncAttributes& attributes, const size_t CTA_SIZE);

template <typename KernelFunction>
inline cudaFuncAttributes function_attributes(KernelFunction kernel);

template <typename KernelFunction>
size_t max_active_blocks_per_multiprocessor(KernelFunction kernel, const size_t CTA_SIZE, const size_t dynamic_smem_bytes);

template <typename KernelFunction>
size_t max_active_blocks(KernelFunction kernel, const size_t CTA_SIZE, const size_t dynamic_smem_bytes);

template <typename KernelFunction>
size_t num_registers(KernelFunction kernel);

template <typename KernelFunction>
size_t max_blocksize_with_highest_occupancy(KernelFunction kernel, size_t dynamic_smem_bytes_per_thread);

inline bool is_tcc_enabled();

inline void check_error(const char *message);

/// a generic syncthreads() implementation to synchronize contiguous
/// blocks of N threads at a time
///
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void syncthreads();

} // namespace cuda
} // namespace nvbio

#include <nvbio/basic/cuda/arch_inl.h>
