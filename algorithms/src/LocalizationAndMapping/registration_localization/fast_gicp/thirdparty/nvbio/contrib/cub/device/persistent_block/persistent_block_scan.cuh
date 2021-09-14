/******************************************************************************
 * Copyright (c) 2011, Duane Merrill.  All rights reserved.
 * Copyright (c) 2011-2013, NVIDIA CORPORATION.  All rights reserved.
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
 * cub::PersistentBlockScan implements an abstraction of CUDA thread blocks for
 * participating in device-wide prefix scan.
 */

#pragma once

#include <iterator>

#include "../grid/grid_mapping.cuh"
#include "../grid/grid_even_share.cuh"
#include "../grid/grid_queue.cuh"
#include "../../block/block_load.cuh"
#include "../../block/block_reduce.cuh"
#include "../../util_vector.cuh"
#include "../../util_namespace.cuh"

/// Optional outer namespace(s)
CUB_NS_PREFIX

/// CUB namespace
namespace cub {


enum
{
    BLOCK_SCAN_OOB,
    BLOCK_SCAN_INVALID,
    BLOCK_SCAN_PARTIAL,
    BLOCK_SCAN_PREFIX,
};


/**
 * Tuning policy for PersistentBlockScan
 */
template <
    int                         _BLOCK_THREADS,
    int                         _ITEMS_PER_THREAD,
    BlockLoadPolicy             _LOAD_POLICY,
    BlockStorePolicy            _STORE_POLICY,
    BlockScanAlgorithm          _SCAN_ALGORITHM>
struct PersistentBlockScanPolicy
{
    enum
    {
        BLOCK_THREADS       = _BLOCK_THREADS,
        ITEMS_PER_THREAD    = _ITEMS_PER_THREAD,
    };

    static const BlockLoadPolicy        LOAD_POLICY      = _LOAD_POLICY;
    static const BlockStorePolicy       STORE_POLICY     = _STORE_POLICY;
    static const BlockScanAlgorithm     SCAN_ALGORITHM   = _SCAN_ALGORITHM;

    // Data type of block-signaling flag
    typedef int BlockFlag;
};


/**
 * \brief PersistentBlockScan implements an abstraction of CUDA thread blocks for
 * participating in device-wide reduction.
 */
template <
    typename PersistentBlockScanPolicy,
    typename InputIteratorRA,
    typename OutputIteratorRA,
    typename SizeT>
class PersistentBlockScan
{
public:

    //---------------------------------------------------------------------
    // Types and constants
    //---------------------------------------------------------------------

    // Data type of input iterator
    typedef typename std::iterator_traits<InputIteratorRA>::value_type T;

    // Data type of block-signaling flag
    typedef typename PersistentBlockScanPolicy::BlockFlag BlockFlag;

    // Constants
    enum
    {
        TILE_ITEMS = PersistentBlockScanPolicy::BLOCK_THREADS * PersistentBlockScanPolicy::ITEMS_PER_THREAD,
    };

    struct Signal
    {
        BlockFlag   flag;
        T           value;
    };

    template <typename ScanOp>
    struct SignalReduceOp
    {
        ScanOp scan_op;

        // Constructor
        __device__ __forceinline__
        SignalReduceOp(ScanOp scan_op) : scan_op(scan_op) {}

        __device__ __forceinline__
        Signal operator()(const Signal& first, const Signal& second)
        {
            if ((first.flag == BLOCK_SCAN_OOB) || (second.flag == BLOCK_SCAN_PREFIX))
                return second;

            Signal retval;
            retval.flag = first.flag;
            retval.value = scan_op(first.value, second.value);
            return retval;
        }
    };

    // Parameterized block load
    typedef BlockLoad<
        InputIteratorRA,
        PersistentBlockScanPolicy::BLOCK_THREADS,
        PersistentBlockScanPolicy::ITEMS_PER_THREAD,
        PersistentBlockScanPolicy::LOAD_POLICY>          BlockLoadT;

    // Parameterized block store
    typedef BlockStore<
        OutputIteratorRA,
        PersistentBlockScanPolicy::BLOCK_THREADS,
        PersistentBlockScanPolicy::ITEMS_PER_THREAD,
        PersistentBlockScanPolicy::STORE_POLICY>         BlockStoreT;

    // Parameterized block scan
    typedef BlockScan<
        T,
        PersistentBlockScanPolicy::BLOCK_THREADS,
        PersistentBlockScanPolicy::SCAN_ALGORITHM>       BlockScanT;

    // Parameterized warp reduce
    typedef WarpReduce<Signal>                      WarpReduceT;

    // Shared memory type for this threadblock
    struct SmemStorage
    {
        union
        {
            typename BlockLoadT::SmemStorage    load;   // Smem needed for tile loading
            typename BlockStoreT::SmemStorage   store;  // Smem needed for tile storing
            typename BlockScanT::SmemStorage    scan;   // Smem needed for tile scanning
        };

        typename WarpReduceT::SmemStorage       warp_reduce;    // Smem needed for warp reduction
    };


    // Stateful prefix functor
    template <typename ScanOp>
    struct BlockPrefixOp
    {
        T*              d_partials;
        T*              d_prefixes;
        BlockFlag*      d_flags;
        ScanOp          scan_op;
        SmemStorage&    smem_storage;

        // Constructor
        __device__ __forceinline__
        BlockPrefixOp(
            T*              d_partials,
            T*              d_prefixes,
            BlockFlag*      d_flags,
            ScanOp          scan_op,
            SmemStorage&    smem_storage) :
                d_partials(d_partials), d_prefixes(d_prefixes), d_flags(d_flags), scan_op(scan_op), smem_storage(smem_storage) {}

        // Prefix functor (called by the first warp)
        __device__ __forceinline__
        T operator()(T block_aggregate)
        {
            // Update our partial
            if (threadIdx.x == 0)
            {
                d_partials[PtxArchProps::WARP_THREADS + blockIdx.x] = block_aggregate;
                __threadfence();
                d_flags[PtxArchProps::WARP_THREADS + blockIdx.x] = BLOCK_SCAN_PARTIAL;
            }

            // Wait for predecessor blocks to become valid and at least one prefix to show up.
            Signal predecessor_signal;
            unsigned int predecessor_idx = PtxArchProps::WARP_THREADS + blockIdx.x - threadIdx.x;
            do
            {
                predecessor_signal.flag = ThreadLoad<PTX_LOAD_CG>(d_flags + predecessor_idx);
            }
            while (__any(predecessor_signal.flag == BLOCK_SCAN_INVALID) || __all(predecessor_signal.flag != BLOCK_SCAN_PREFIX));

            // Grab predecessor block's corresponding partial/prefix
            predecessor_signal.value = (predecessor_signal.flag == BLOCK_SCAN_PREFIX) ?
                d_prefixes[predecessor_idx] :
                d_partials[predecessor_idx];

            // Reduce predecessor partials/prefixes to get our block-wide exclusive prefix
            Signal prefix_signal = WarpReduceT::Reduce(
                smem_storage.warp_reduce,
                predecessor_signal,
                SignalReduceOp(scan_op));

            // Update the signals with our inclusive prefix
            if (threadIdx.x == 0)
            {
                T inclusive_prefix = scan_op(prefix_signal.value, block_aggregate);

                d_prefixes[PtxArchProps::WARP_THREADS + blockIdx.x] = inclusive_prefix;
                __threadfence();
                d_flags[PtxArchProps::WARP_THREADS + blockIdx.x] = BLOCK_SCAN_PREFIX;
            }

            // Return block-wide exclusive prefix
            return prefix_signal.value;
        }
    };


    //---------------------------------------------------------------------
    // Utility operations
    //---------------------------------------------------------------------

    /**
     * Process a single, full tile.  Specialized for d_in is an iterator (not a native pointer)
     *
     * Each thread reduces only the values it loads.  If \p FIRST_TILE,
     * this partial reduction is stored into \p thread_aggregate.  Otherwise
     * it is accumulated into \p thread_aggregate.
     */
    template <
        typename    SizeT,
        typename    ScanOp>
    static __device__ __forceinline__ void ConsumeFullTile(
        SmemStorage             &smem_storage,
        InputIteratorRA           d_in,
        OutputIteratorRA          d_out,
        SizeT                   block_offset,
        ScanOp                  &scan_op,
        T                       &thread_aggregate)
    {
        T items[PersistentBlockScanPolicy::ITEMS_PER_THREAD];

        BlockLoadT::Load(smem_storage.load, d_in + block_offset, items);

        __syncthreads();

        BlockScanT::Load(smem_storage.load, d_in + block_offset, items);

        __syncthreads();

        BlockStoreT::Store(smem_storage.load, d_in + block_offset, items);
    }


    /**
     * Process a single, full tile.  Specialized for native pointers
     *
     * Each thread reduces only the values it loads.  If \p FIRST_TILE,
     * this partial reduction is stored into \p thread_aggregate.  Otherwise
     * it is accumulated into \p thread_aggregate.
     *
     * Performs a block-wide barrier synchronization
     */
    template <
        bool FIRST_TILE,
        typename SizeT,
        typename ScanOp>
    static __device__ __forceinline__ void ConsumeFullTile(
        SmemStorage             &smem_storage,
        T                       *d_in,
        SizeT                   block_offset,
        ScanOp             &scan_op,
        T                       &thread_aggregate)
    {
        if ((size_t(d_in) & (VECTOR_LOAD_LENGTH - 1)) == 0)
        {
            T items[ITEMS_PER_THREAD];

            typedef typename VectorHelper<T, VECTOR_LOAD_LENGTH>::Type VectorT;

            // Alias items as an array of VectorT and load it in striped fashion
            BlockLoadDirectStriped(
                reinterpret_cast<VectorT*>(d_in + block_offset),
                reinterpret_cast<VectorT (&)[ITEMS_PER_THREAD / VECTOR_LOAD_LENGTH]>(items));

            // Prevent hoisting
            __syncthreads();

            T partial = ThreadReduce(items, scan_op);

            thread_aggregate = (FIRST_TILE) ?
                partial :
                scan_op(thread_aggregate, partial);
        }
        else
        {
            T items[ITEMS_PER_THREAD];

            BlockLoadDirectStriped(
                d_in + block_offset,
                items);

            // Prevent hoisting
            __syncthreads();

            T partial = ThreadReduce(items, scan_op);

            thread_aggregate = (FIRST_TILE) ?
                partial :
                scan_op(thread_aggregate, partial);
        }

    }


    /**
     * Process a single, partial tile.
     *
     * Each thread reduces only the values it loads.  If \p FIRST_TILE,
     * this partial reduction is stored into \p thread_aggregate.  Otherwise
     * it is accumulated into \p thread_aggregate.
     */
    template <
        bool FIRST_TILE,
        typename SizeT,
        typename ScanOp>
    static __device__ __forceinline__ void ConsumePartialTile(
        SmemStorage             &smem_storage,
        InputIteratorRA           d_in,
        SizeT                   block_offset,
        const SizeT             &block_oob,
        ScanOp             &scan_op,
        T                       &thread_aggregate)
    {
        SizeT thread_offset = block_offset + threadIdx.x;

        if ((FIRST_TILE) && (thread_offset < block_oob))
        {
            thread_aggregate = ThreadLoad<LOAD_MODIFIER>(d_in + thread_offset);
            thread_offset += BLOCK_THREADS;
        }

        while (thread_offset < block_oob)
        {
            T item = ThreadLoad<LOAD_MODIFIER>(d_in + thread_offset);
            thread_aggregate = scan_op(thread_aggregate, item);
            thread_offset += BLOCK_THREADS;
        }
    }

public:

    //---------------------------------------------------------------------
    // Interface
    //---------------------------------------------------------------------

    /**
     * \brief Consumes input tiles using an even-share policy, computing a threadblock-wide reduction for thread<sub>0</sub> using the specified binary reduction functor.
     *
     * The return value is undefined in threads other than thread<sub>0</sub>.
     */
    template <typename SizeT, typename ScanOp>
    static __device__ __forceinline__ T ProcessPersistentBlockEvenShare(
        SmemStorage             &smem_storage,
        InputIteratorRA           d_in,
        SizeT                   block_offset,
        const SizeT             &block_oob,
        ScanOp             &scan_op)
    {
        if (block_offset + TILE_ITEMS <= block_oob)
        {
            // We have at least one full tile to consume
            T thread_aggregate;
            ConsumeFullTile<true>(smem_storage, d_in, block_offset, scan_op, thread_aggregate);
            block_offset += TILE_ITEMS;

            // Consume any other full tiles
            while (block_offset + TILE_ITEMS <= block_oob)
            {
                ConsumeFullTile<false>(smem_storage, d_in, block_offset, scan_op, thread_aggregate);
                block_offset += TILE_ITEMS;
            }

            // Consume any remaining input
            ConsumePartialTile<false>(smem_storage, d_in, block_offset, block_oob, scan_op, thread_aggregate);

            // Compute the block-wide reduction (every thread has a valid input)
            return BlockReduceT::Reduce(smem_storage.reduce, thread_aggregate, scan_op);
        }
        else
        {
            // We have less than a full tile to consume
            T thread_aggregate;
            ConsumePartialTile<true>(smem_storage, d_in, block_offset, block_oob, scan_op, thread_aggregate);

            // Compute the block-wide reduction  (up to block_items threads have valid inputs)
            SizeT block_items = block_oob - block_offset;
            return BlockReduceT::Reduce(smem_storage.reduce, thread_aggregate, scan_op, block_items);
        }
    }


    /**
     * \brief Consumes input tiles using a dynamic queue policy, computing a threadblock-wide reduction for thread<sub>0</sub> using the specified binary reduction functor.
     *
     * The return value is undefined in threads other than thread<sub>0</sub>.
     */
    template <typename SizeT, typename ScanOp>
    static __device__ __forceinline__ T ProcessPersistentBlockDynamic(
        SmemStorage             &smem_storage,
        InputIteratorRA           d_in,
        SizeT                   num_items,
        GridQueue<SizeT>        &queue,
        ScanOp             &scan_op)
    {
        // Each thread block is statically assigned at some input, otherwise its
        // block_aggregate will be undefined.
        SizeT block_offset = blockIdx.x * TILE_ITEMS;

        if (block_offset + TILE_ITEMS <= num_items)
        {
            // We have a full tile to consume
            T thread_aggregate;
            ConsumeFullTile<true>(smem_storage, d_in, block_offset, scan_op, thread_aggregate);

            // Dynamically consume other tiles
            SizeT even_share_base = gridDim.x * TILE_ITEMS;

            if (even_share_base < num_items)
            {
                // There are tiles left to consume
                while (true)
                {
                    // Dequeue up to TILE_ITEMS
                    if (threadIdx.x == 0)
                    {
                        smem_storage.block_offset = queue.Drain(TILE_ITEMS) + even_share_base;
                    }

                    __syncthreads();

                    block_offset = smem_storage.block_offset;

                    if (block_offset + TILE_ITEMS > num_items)
                    {
                        if (block_offset < num_items)
                        {
                            // We have less than a full tile to consume
                            ConsumePartialTile<false>(smem_storage, d_in, block_offset, num_items, scan_op, thread_aggregate);
                        }

                        // No more work to do
                        break;
                    }

                    // We have a full tile to consume (which performs a barrier to protect smem_storage.block_offset WARs)
                    ConsumeFullTile<false>(smem_storage, d_in, block_offset, scan_op, thread_aggregate);
                }
            }

            // Compute the block-wide reduction (every thread has a valid input)
            return BlockReduceT::Reduce(smem_storage.reduce, thread_aggregate, scan_op);
        }
        else
        {
            // We have less than a full tile to consume
            T thread_aggregate;
            SizeT block_items = num_items - block_offset;
            ConsumePartialTile<true>(smem_storage, d_in, block_offset, num_items, scan_op, thread_aggregate);

            // Compute the block-wide reduction  (up to block_items threads have valid inputs)
            return BlockReduceT::Reduce(smem_storage.reduce, thread_aggregate, scan_op, block_items);
        }
    }

};


}               // CUB namespace
CUB_NS_POSTFIX  // Optional outer namespace(s)

