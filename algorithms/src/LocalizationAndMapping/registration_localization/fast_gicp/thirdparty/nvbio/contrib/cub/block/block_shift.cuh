/******************************************************************************
 * Copyright (c) 2011, Duane Merrill.  All rights reserved.
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the NVIDIA CORPORATION nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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
 *
 ******************************************************************************/

/**
 * \file
 * The cub::BlockShift class provides [<em>collective</em>](index.html#sec0) methods for rearranging data partitioned across a CUDA thread block.
 */

#pragma once

#include "../util_arch.cuh"
#include "../util_macro.cuh"
#include "../util_type.cuh"
#include "../util_namespace.cuh"

/// Optional outer namespace(s)
CUB_NS_PREFIX

/// CUB namespace
namespace cub {

/**
 * \brief The BlockShift class provides [<em>collective</em>](index.html#sec0) methods for shifting data partitioned across a CUDA thread block. ![](transpose_logo.png)
 * \ingroup BlockModule
 *
 * \tparam T                    The data type to be exchanged.
 * \tparam BLOCK_THREADS        The thread block size in threads.
 *
 * \par Overview
 * It is commonplace for blocks of threads to rearrange data items between
 * threads.  The BlockShift abstraction allows threads to efficiently shift items
 * either (a) up to their successor or (b) down to their predecessor.
 *
 */
template <
    typename        T,
    int             BLOCK_THREADS>
class BlockShift
{
private:

    /******************************************************************************
     * Constants
     ******************************************************************************/

    enum
    {
        LOG_WARP_THREADS            = CUB_PTX_LOG_WARP_THREADS,
        WARP_THREADS                = 1 << LOG_WARP_THREADS,
        WARPS                       = (BLOCK_THREADS + CUB_PTX_WARP_THREADS - 1) / CUB_PTX_WARP_THREADS,
    };

    /******************************************************************************
     * Type definitions
     ******************************************************************************/

    /// Shared memory storage layout type
#if CUB_PTX_VERSION >= 300
    typedef T _TempStorage[WARPS];              // Kepler+ only needs smem to share between warps
#else
    typedef T _TempStorage[BLOCK_THREADS];
#endif

public:

    /// \smemstorage{BlockShift}
    struct TempStorage : Uninitialized<_TempStorage> {};

private:


    /******************************************************************************
     * Thread fields
     ******************************************************************************/

    /// Shared storage reference
    _TempStorage &temp_storage;

    /// Linear thread-id
    int linear_tid;
    int warp_lane;
    int warp_id;


    /******************************************************************************
     * Utility methods
     ******************************************************************************/

    /// Internal storage allocator
    __device__ __forceinline__ _TempStorage& PrivateStorage()
    {
        __shared__ _TempStorage private_storage;
        return private_storage;
    }


public:

    /******************************************************************//**
     * \name Collective constructors
     *********************************************************************/
    //@{

    /**
     * \brief Collective constructor for 1D thread blocks using a private static allocation of shared memory as temporary storage.  Threads are identified using <tt>threadIdx.x</tt>.
     */
    __device__ __forceinline__ BlockShift()
    :
        temp_storage(PrivateStorage()),
        linear_tid(threadIdx.x),
        warp_lane(linear_tid & (WARP_THREADS - 1)),
        warp_id(linear_tid >> LOG_WARP_THREADS)
    {}


    /**
     * \brief Collective constructor for 1D thread blocks using the specified memory allocation as temporary storage.  Threads are identified using <tt>threadIdx.x</tt>.
     */
    __device__ __forceinline__ BlockShift(
        TempStorage &temp_storage)             ///< [in] Reference to memory allocation having layout type TempStorage
    :
        temp_storage(temp_storage.Alias()),
        linear_tid(threadIdx.x),
        warp_lane(linear_tid & (WARP_THREADS - 1)),
        warp_id(linear_tid >> LOG_WARP_THREADS)
    {}


    /**
     * \brief Collective constructor using a private static allocation of shared memory as temporary storage.  Each thread is identified using the supplied linear thread identifier
     */
    __device__ __forceinline__ BlockShift(
        int linear_tid)                        ///< [in] A suitable 1D thread-identifier for the calling thread (e.g., <tt>(threadIdx.y * blockDim.x) + linear_tid</tt> for 2D thread blocks)
    :
        temp_storage(PrivateStorage()),
        linear_tid(linear_tid),
        warp_lane(linear_tid & (WARP_THREADS - 1)),
        warp_id(linear_tid >> LOG_WARP_THREADS)
    {}


    /**
     * \brief Collective constructor using the specified memory allocation as temporary storage.  Each thread is identified using the supplied linear thread identifier.
     */
    __device__ __forceinline__ BlockShift(
        TempStorage &temp_storage,              ///< [in] Reference to memory allocation having layout type TempStorage
        int         linear_tid)                 ///< [in] <b>[optional]</b> A suitable 1D thread-identifier for the calling thread (e.g., <tt>(threadIdx.y * blockDim.x) + linear_tid</tt> for 2D thread blocks)
    :
        temp_storage(temp_storage.Alias()),
        linear_tid(linear_tid),
        warp_lane(linear_tid & (WARP_THREADS - 1)),
        warp_id(linear_tid >> LOG_WARP_THREADS)
    {}


    //@}  end member group
    /******************************************************************//**
     * \name Shift exchanges
     *********************************************************************/
    //@{


    /**
     * \brief Each thread obtains the \p input provided by its predecessor.  The first thread receives \p block_prefix.
     *
     * \par
     * - \smemreuse
     */
    __device__ __forceinline__ void Up(
        T input,            ///< [in] Input item
        T &output,          ///< [out] Output item
        T block_prefix)     ///< [in] Prefix item to be provided to <em>thread</em><sub>0</sub>
    {
#if CUB_PTX_VERSION >= 300
        if (warp_lane == WARP_THREADS - 1)
            temp_storage[warp_id] = input;

        __syncthreads();

        output = ShuffleUp(input, 1);
        if (warp_lane == 0)
        {
            output = (linear_tid == 0) ?
                block_prefix :
                temp_storage[warp_id - 1];
        }
#else
        temp_storage[linear_tid] = input;

        __syncthreads();

        output = (linear_tid == 0) ?
            block_prefix :
            temp_storage[linear_tid - 1];
#endif
    }


    /**
     * \brief Each thread receives the \p input provided by its predecessor.  The first thread receives \p block_prefix.  All threads receive the \p input provided by <em>thread</em><sub><tt>BLOCK_THREADS-1</tt></sub>.
     *
     * \par
     * - \smemreuse
     */
    __device__ __forceinline__ void Up(
        T input,            ///< [in] Input item
        T &output,          ///< [out] Output item
        T block_prefix,     ///< [in] Prefix item to be provided to <em>thread</em><sub>0</sub>
        T &block_suffix)    ///< [out] Suffix item shifted out by the <em>thread</em><sub><tt>BLOCK_THREADS-1</tt></sub> to be provided to all threads
    {
#if CUB_PTX_VERSION >= 300
        if (warp_lane == WARP_THREADS - 1)
            temp_storage[warp_id] = input;

        __syncthreads();

        output = ShuffleUp(input, 1);
        if (warp_lane == 0)
        {
            output = (linear_tid == 0) ?
                block_prefix :
                temp_storage[warp_id - 1];
        }
        block_suffix = temp_storage[WARPS - 1];
#else
        temp_storage[linear_tid] = input;

        __syncthreads();

        output = (linear_tid == 0) ?
            block_prefix :
            temp_storage[linear_tid - 1];

        block_suffix = temp_storage[BLOCK_THREADS - 1];
#endif
    }


    /**
     * \brief Each thread obtains the \p input provided by its successor.  The last thread receives \p block_suffix.
     *
     * \par
     * - \smemreuse
     */
    __device__ __forceinline__ void Down(
        T input,            ///< [in] Input item
        T &output,          ///< [out] Output item
        T block_suffix)     ///< [in] Suffix item to be provided to <em>thread</em><sub><tt>BLOCK_THREADS-1</tt></sub>
    {
#if CUB_PTX_VERSION >= 300
        if (warp_lane == 0)
            temp_storage[warp_id] = input;

        __syncthreads();

        output = ShuffleDown(input, 1);
        if (warp_lane == WARP_THREADS - 1)
        {
            output = (linear_tid == BLOCK_THREADS - 1) ?
                block_suffix :
                temp_storage[warp_id + 1];
        }
#else
        temp_storage[linear_tid] = input;

        __syncthreads();

        output = (linear_tid == BLOCK_THREADS - 1) ?
            block_suffix :
            temp_storage[linear_tid + 1];
#endif
    }


    /**
     * \brief Each thread obtains the \p input provided by its successor.  The last thread receives \p block_suffix.  All threads receive the \p input provided by <em>thread</em><sub>0</sub>.
     *
     * \par
     * - \smemreuse
     */
    __device__ __forceinline__ void Down(
        T input,            ///< [in] Input item
        T &output,          ///< [out] Output item
        T block_suffix,     ///< [in] Suffix item to be provided to <em>thread</em><sub><tt>BLOCK_THREADS-1</tt></sub>
        T &block_prefix)    ///< [out] Prefix item shifted out by the <em>thread</em><sub>0</sub> to be provided to all threads
    {
#if CUB_PTX_VERSION >= 300
        if (warp_lane == 0)
            temp_storage[warp_id] = input;

        __syncthreads();

        output = ShuffleDown(input, 1);
        if (warp_lane == WARP_THREADS - 1)
        {
            output = (linear_tid == BLOCK_THREADS - 1) ?
                block_suffix :
                temp_storage[warp_id + 1];
        }
#else
        temp_storage[linear_tid] = input;

        __syncthreads();

        output = (linear_tid == BLOCK_THREADS - 1) ?
            block_suffix :
            temp_storage[linear_tid + 1];
#endif

        block_prefix = temp_storage[0];
    }

    //@}  end member group


};

}               // CUB namespace
CUB_NS_POSTFIX  // Optional outer namespace(s)

