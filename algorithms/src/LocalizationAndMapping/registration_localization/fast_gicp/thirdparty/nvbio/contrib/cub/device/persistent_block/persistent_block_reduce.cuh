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
 * cub::PersistentBlockReduce implements a stateful abstraction of CUDA thread blocks for participating in device-wide reduction.

 */

#pragma once

#include <iterator>

#include "../../grid/grid_mapping.cuh"
#include "../../grid/grid_even_share.cuh"
#include "../../grid/grid_queue.cuh"
#include "../../block/block_load.cuh"
#include "../../block/block_reduce.cuh"
#include "../../util_vector.cuh"
#include "../../util_namespace.cuh"

/// Optional outer namespace(s)
CUB_NS_PREFIX

/// CUB namespace
namespace cub {


/**
 * Tuning policy for PersistentBlockReduce
 */
template <
    int                     _BLOCK_THREADS,
    int                     _ITEMS_PER_THREAD,
    int                     _VECTOR_LOAD_LENGTH,
    BlockReduceAlgorithm    _BLOCK_ALGORITHM,
    PtxLoadModifier         _LOAD_MODIFIER,
    GridMappingStrategy     _GRID_MAPPING>
struct PersistentBlockReducePolicy
{
    enum
    {
        BLOCK_THREADS       = _BLOCK_THREADS,
        ITEMS_PER_THREAD    = _ITEMS_PER_THREAD,
        VECTOR_LOAD_LENGTH  = _VECTOR_LOAD_LENGTH,
    };

    static const BlockReduceAlgorithm  BLOCK_ALGORITHM      = _BLOCK_ALGORITHM;
    static const GridMappingStrategy   GRID_MAPPING         = _GRID_MAPPING;
    static const PtxLoadModifier       LOAD_MODIFIER        = _LOAD_MODIFIER;
};


/**
 * \brief PersistentBlockReduce implements a stateful abstraction of CUDA thread blocks for participating in device-wide reduction.
 */
template <
    typename PersistentBlockReducePolicy,
    typename InputIteratorRA,
    typename SizeT,
    typename ReductionOp>
struct PersistentBlockReduce
{

    //---------------------------------------------------------------------
    // Types and constants
    //---------------------------------------------------------------------

    typedef typename std::iterator_traits<InputIteratorRA>::value_type      T;              // Type of input iterator
    typedef VectorHelper<T, PersistentBlockReducePolicy::VECTOR_LOAD_LENGTH>      VecHelper;      // Helper type for vectorizing loads of T
    typedef typename VecHelper::Type                                        VectorT;        // Vector of T

    // Constants
    enum
    {
        BLOCK_THREADS       = PersistentBlockReducePolicy::BLOCK_THREADS,
        ITEMS_PER_THREAD    = PersistentBlockReducePolicy::ITEMS_PER_THREAD,
        TILE_ITEMS          = BLOCK_THREADS * ITEMS_PER_THREAD,
        VECTOR_LOAD_LENGTH  = PersistentBlockReducePolicy::VECTOR_LOAD_LENGTH,

        // Can vectorize according to the policy if the input iterator is a native pointer to a built-in primitive
        CAN_VECTORIZE       = (PersistentBlockReducePolicy::VECTOR_LOAD_LENGTH > 1) &&
                                (IsPointer<InputIteratorRA>::VALUE) &&
                                (VecHelper::BUILT_IN),

    };

    static const BlockReduceAlgorithm BLOCK_ALGORITHM = PersistentBlockReducePolicy::BLOCK_ALGORITHM;

    // Parameterized BlockReduce primitive
    typedef BlockReduce<T, BLOCK_THREADS, PersistentBlockReducePolicy::BLOCK_ALGORITHM> BlockReduceT;

    // Shared memory type required by this thread block
    typedef typename BlockReduceT::SmemStorage SmemStorage;


    //---------------------------------------------------------------------
    // Per-thread fields
    //---------------------------------------------------------------------

    T                       thread_aggregate;   ///< Each thread's partial reduction
    SmemStorage&            smem_storage;       ///< Reference to smem_storage
    InputIteratorRA         d_in;               ///< Input data to reduce
    ReductionOp             reduction_op;       ///< Binary reduction operator
    int                     first_tile_size;    ///< Size of first tile consumed
    bool                    input_aligned;      ///< Whether or not input is vector-aligned


    //---------------------------------------------------------------------
    // Interface
    //---------------------------------------------------------------------

    /**
     * Constructor
     */
    __device__ __forceinline__ PersistentBlockReduce(
        SmemStorage&            smem_storage,       ///< Reference to smem_storage
        InputIteratorRA         d_in,               ///< Input data to reduce
        ReductionOp             reduction_op) :     ///< Binary reduction operator
            smem_storage(smem_storage),
            d_in(d_in),
            reduction_op(reduction_op),
            first_tile_size(TILE_ITEMS),
            input_aligned(CAN_VECTORIZE && ((size_t(d_in) & (sizeof(VectorT) - 1)) == 0)){}


    /**
     * The number of items processed per "tile"
     */
    __device__ __forceinline__ int TileItems()
    {
        return TILE_ITEMS;
    }


    /**
     * Process a single tile.
     *
     * Each thread reduces only the values it loads. If \p FIRST_TILE, this
     * partial reduction is stored into \p thread_aggregate.  Otherwise it is
     * accumulated into \p thread_aggregate.
     */
    __device__ __forceinline__ void ConsumeTile(
        bool    &sync_after,
        SizeT   block_offset,
        int     num_valid,
        bool    first_tile)
    {
        if (num_valid < TILE_ITEMS)
        {
            // Our first tile is a partial tile size
            if (first_tile) first_tile_size = num_valid;

            // Partial tile
            int thread_offset = threadIdx.x;

            if ((first_tile) && (thread_offset < num_valid))
            {
                thread_aggregate = ThreadLoad<PersistentBlockReducePolicy::LOAD_MODIFIER>(d_in + block_offset + thread_offset);
                thread_offset += BLOCK_THREADS;
            }

            while (thread_offset < num_valid)
            {
                T item = ThreadLoad<PersistentBlockReducePolicy::LOAD_MODIFIER>(d_in + block_offset + thread_offset);
                thread_aggregate = reduction_op(thread_aggregate, item);
                thread_offset += BLOCK_THREADS;
            }
        }
        else
        {
            T items[ITEMS_PER_THREAD];

            // Load full tile
            if (input_aligned)
            {
                // Alias items as an array of VectorT and load it in striped fashion
                BlockLoadDirectStriped(
                    reinterpret_cast<VectorT*>(d_in + block_offset),
                    reinterpret_cast<VectorT (&)[ITEMS_PER_THREAD / VECTOR_LOAD_LENGTH]>(items));
            }
            else
            {
                // Load items in striped fashion
                BlockLoadDirectStriped(d_in + block_offset, items);
            }

            // Prevent hoisting
            __threadfence_block();

            // Reduce items within each thread
            T partial = ThreadReduce(items, reduction_op);

            // Update|assign the thread's running aggregate
            thread_aggregate = (first_tile) ?
                partial :
                reduction_op(thread_aggregate, partial);
        }

        // No synchronization needed after tile processing
        sync_after = false;
    }


    /**
     * Finalize the computation.
     */
    __device__ __forceinline__ void Finalize(
        T& block_aggregate)
    {
        // Cooperative reduction across the thread block (guarded reduction if our first tile was a partial tile)
        block_aggregate = (first_tile_size < TILE_ITEMS) ?
            BlockReduceT::Reduce(smem_storage, thread_aggregate, reduction_op, first_tile_size) :
            BlockReduceT::Reduce(smem_storage, thread_aggregate, reduction_op);
    }

};


}               // CUB namespace
CUB_NS_POSTFIX  // Optional outer namespace(s)

