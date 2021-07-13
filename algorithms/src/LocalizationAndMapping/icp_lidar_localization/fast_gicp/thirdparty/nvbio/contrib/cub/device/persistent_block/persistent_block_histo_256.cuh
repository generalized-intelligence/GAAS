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
 * cub::PersistentBlockHisto256 implements a stateful abstraction of CUDA thread blocks for histogramming multiple tiles as part of device-wide 256-bin histogram.
 */

#pragma once

#include <iterator>

#include "../../util_arch.cuh"
#include "../../block/block_load.cuh"
#include "../../block/block_histo_256.cuh"
#include "../../block/block_radix_sort.cuh"
#include "../../block/block_discontinuity.cuh"
#include "../../grid/grid_mapping.cuh"
#include "../../grid/grid_even_share.cuh"
#include "../../grid/grid_queue.cuh"
#include "../../util_vector.cuh"
#include "../../util_namespace.cuh"

/// Optional outer namespace(s)
CUB_NS_PREFIX

/// CUB namespace
namespace cub {


/******************************************************************************
 * Algorithmic variants
 ******************************************************************************/


/**
 * \brief PersistentBlockHisto256Algorithm enumerates alternative algorithms for the parallel construction of 8b histograms.
 */
enum PersistentBlockHisto256Algorithm
{

    /**
     * \par Overview
     * A two-kernel approach in which:
     * -# Thread blocks in the first kernel aggregate their own privatized
     *    histograms using block-wide sorting (see BlockHisto256Algorithm::BLOCK_HISTO_256_SORT).
     * -# A single thread block in the second kernel reduces them into the output histogram(s).
     *
     * \par Performance Considerations
     * Delivers consistent throughput regardless of sample bin distribution.
     */
    GRID_HISTO_256_SORT,


    /**
     * \par Overview
     * A two-kernel approach in which:
     * -# Thread blocks in the first kernel aggregate their own privatized
     *    histograms using shared-memory \p atomicAdd().
     * -# A single thread block in the second kernel reduces them into the
     *    output histogram(s).
     *
     * \par Performance Considerations
     * Performance is strongly tied to the hardware implementation of atomic
     * addition, and may be significantly degraded for non uniformly-random
     * input distributions where many concurrent updates are likely to be
     * made to the same bin counter.
     */
    GRID_HISTO_256_SHARED_ATOMIC,


    /**
     * \par Overview
     * A single-kernel approach in which thread blocks update the output histogram(s) directly
     * using global-memory \p atomicAdd().
     *
     * \par Performance Considerations
     * Performance is strongly tied to the hardware implementation of atomic
     * addition, and may be significantly degraded for non uniformly-random
     * input distributions where many concurrent updates are likely to be
     * made to the same bin counter.
     */
    GRID_HISTO_256_GLOBAL_ATOMIC,

};


/******************************************************************************
 * Tuning policy
 ******************************************************************************/

/**
 * Tuning policy for PersistentBlockHisto256
 */
template <
    int                         _BLOCK_THREADS,
    int                         _ITEMS_PER_THREAD,
    PersistentBlockHisto256Algorithm  _GRID_ALGORITHM,
    GridMappingStrategy         _GRID_MAPPING,
    int                         _SM_OCCUPANCY>
struct PersistentBlockHisto256Policy
{
    enum
    {
        BLOCK_THREADS       = _BLOCK_THREADS,
        ITEMS_PER_THREAD    = _ITEMS_PER_THREAD,
        SM_OCCUPANCY        = _SM_OCCUPANCY,
    };

    static const PersistentBlockHisto256Algorithm     GRID_ALGORITHM      = _GRID_ALGORITHM;
    static const GridMappingStrategy            GRID_MAPPING        = _GRID_MAPPING;
};



/******************************************************************************
 * PersistentBlockHisto256
 ******************************************************************************/

/**
 * \brief implements a stateful abstraction of CUDA thread blocks for histogramming multiple tiles as part of device-wide 256-bin histogram.
 */
template <
    typename                PersistentBlockHisto256Policy,                                        ///< Tuning policy
    int                     CHANNELS,                                                       ///< Number of channels interleaved in the input data (may be greater than the number of active channels being histogrammed)
    int                     ACTIVE_CHANNELS,                                                ///< Number of channels actively being histogrammed
    typename                InputIteratorRA,                                                ///< The input iterator type (may be a simple pointer type).  Must have a value type that is assignable to <tt>unsigned char</tt>
    typename                HistoCounter,                                                   ///< Integral type for counting sample occurrences per histogram bin
    typename                SizeT,                                                          ///< Integer type for offsets
    PersistentBlockHisto256Algorithm  GRID_ALGORITHM = PersistentBlockHisto256Policy::GRID_ALGORITHM>
struct PersistentBlockHisto256;


/**
 * Specialized for GRID_HISTO_256_GLOBAL_ATOMIC
 */
template <
    typename                PersistentBlockHisto256Policy,    ///< Tuning policy
    int                     CHANNELS,                   ///< Number of channels interleaved in the input data (may be greater than the number of active channels being histogrammed)
    int                     ACTIVE_CHANNELS,            ///< Number of channels actively being histogrammed
    typename                InputIteratorRA,            ///< The input iterator type (may be a simple pointer type).  Must have a value type that is assignable to <tt>unsigned char</tt>
    typename                HistoCounter,               ///< Integral type for counting sample occurrences per histogram bin
    typename                SizeT>                      ///< Integer type for offsets
struct PersistentBlockHisto256<PersistentBlockHisto256Policy, CHANNELS, ACTIVE_CHANNELS, InputIteratorRA, HistoCounter, SizeT, GRID_HISTO_256_GLOBAL_ATOMIC>
{
    //---------------------------------------------------------------------
    // Types and constants
    //---------------------------------------------------------------------

    // Constants
    enum
    {
        BLOCK_THREADS       = PersistentBlockHisto256Policy::BLOCK_THREADS,
        ITEMS_PER_THREAD    = PersistentBlockHisto256Policy::ITEMS_PER_THREAD,
        TILE_CHANNEL_ITEMS  = BLOCK_THREADS * ITEMS_PER_THREAD,
        TILE_ITEMS          = TILE_CHANNEL_ITEMS * CHANNELS,
    };

    // Shared memory type required by this thread block
    struct SmemStorage {};


    //---------------------------------------------------------------------
    // Per-thread fields
    //---------------------------------------------------------------------

    /// Reference to smem_storage
    SmemStorage &smem_storage;

    /// Reference to output histograms
    HistoCounter* (&d_out_histograms)[ACTIVE_CHANNELS];

    /// Input data to reduce
    InputIteratorRA d_in;


    //---------------------------------------------------------------------
    // Interface
    //---------------------------------------------------------------------

    /**
     * Constructor
     */
    __device__ __forceinline__ PersistentBlockHisto256(
        SmemStorage         &smem_storage,                                  ///< Reference to smem_storage
        InputIteratorRA     d_in,                                           ///< Input data to reduce
        HistoCounter*       (&d_out_histograms)[ACTIVE_CHANNELS]) :         ///< Reference to output histograms
            smem_storage(smem_storage),
            d_in(d_in),
            d_out_histograms(d_out_histograms)
    {}


    /**
     * The number of items processed per "tile"
     */
    __device__ __forceinline__ int TileItems()
    {
        return TILE_ITEMS;
    }


    /**
     * Process a single tile.
     */
    __device__ __forceinline__ void ConsumeTile(
        bool    &sync_after,
        SizeT   block_offset,
        int     num_valid)
    {
        if (num_valid < TILE_ITEMS)
        {
            // Only a partially-full tile of samples to read and composite
            int bounds = num_valid - (threadIdx.x * CHANNELS);

            #pragma unroll
            for (int ITEM = 0; ITEM < ITEMS_PER_THREAD; ++ITEM)
            {
                #pragma unroll
                for (int CHANNEL = 0; CHANNEL < CHANNELS; ++CHANNEL)
                {
                    if (((ACTIVE_CHANNELS == CHANNELS) || (CHANNEL < ACTIVE_CHANNELS)) && ((ITEM * BLOCK_THREADS * CHANNELS) + CHANNEL < bounds))
                    {
                        unsigned char item  = d_in[block_offset + (ITEM * BLOCK_THREADS * CHANNELS) + (threadIdx.x * CHANNELS) + CHANNEL];
                        atomicAdd(d_out_histograms[CHANNEL] + item, 1);
                    }
                }
            }

        }
        else
        {
            // Full tile of samples to read and composite
            unsigned char items[ITEMS_PER_THREAD][CHANNELS];

            #pragma unroll
            for (int ITEM = 0; ITEM < ITEMS_PER_THREAD; ITEM++)
            {
                #pragma unroll
                for (int CHANNEL = 0; CHANNEL < CHANNELS; ++CHANNEL)
                {
                    if (CHANNEL < ACTIVE_CHANNELS)
                    {
                        items[ITEM][CHANNEL] = d_in[block_offset + (ITEM * BLOCK_THREADS * CHANNELS) + (threadIdx.x * CHANNELS) + CHANNEL];
                    }
                }
            }

            __threadfence_block();

            #pragma unroll
            for (int ITEM = 0; ITEM < ITEMS_PER_THREAD; ITEM++)
            {
                #pragma unroll
                for (int CHANNEL = 0; CHANNEL < CHANNELS; ++CHANNEL)
                {
                    if (CHANNEL < ACTIVE_CHANNELS)
                    {
                        atomicAdd(d_out_histograms[CHANNEL] + items[ITEM][CHANNEL], 1);
                    }
                }
            }
        }

        // No need to sync after processing this tile to ensure smem coherence
        sync_after = false;
    }


    /**
     * Finalize the computation.
     */
    __device__ __forceinline__ void Finalize(
        int dummy_result)
    {}
};




/**
 * Specialized for GRID_HISTO_256_SHARED_ATOMIC
 */
template <
    typename                PersistentBlockHisto256Policy,    ///< Tuning policy
    int                     CHANNELS,                   ///< Number of channels interleaved in the input data (may be greater than the number of active channels being histogrammed)
    int                     ACTIVE_CHANNELS,            ///< Number of channels actively being histogrammed
    typename                InputIteratorRA,            ///< The input iterator type (may be a simple pointer type).  Must have a value type that is assignable to <tt>unsigned char</tt>
    typename                HistoCounter,               ///< Integral type for counting sample occurrences per histogram bin
    typename                SizeT>                      ///< Integer type for offsets
struct PersistentBlockHisto256<PersistentBlockHisto256Policy, CHANNELS, ACTIVE_CHANNELS, InputIteratorRA, HistoCounter, SizeT, GRID_HISTO_256_SHARED_ATOMIC>
{
    //---------------------------------------------------------------------
    // Types and constants
    //---------------------------------------------------------------------

    // Constants
    enum
    {
        BLOCK_THREADS       = PersistentBlockHisto256Policy::BLOCK_THREADS,
        ITEMS_PER_THREAD    = PersistentBlockHisto256Policy::ITEMS_PER_THREAD,
        TILE_CHANNEL_ITEMS  = BLOCK_THREADS * ITEMS_PER_THREAD,
        TILE_ITEMS          = TILE_CHANNEL_ITEMS * CHANNELS,
    };

    // Shared memory type required by this thread block
    struct SmemStorage
    {
        HistoCounter histograms[ACTIVE_CHANNELS][256];
    };


    //---------------------------------------------------------------------
    // Per-thread fields
    //---------------------------------------------------------------------

    /// Reference to smem_storage
    SmemStorage &smem_storage;

    /// Reference to output histograms
    HistoCounter* (&d_out_histograms)[ACTIVE_CHANNELS];

    /// Input data to reduce
    InputIteratorRA d_in;


    //---------------------------------------------------------------------
    // Interface
    //---------------------------------------------------------------------

    /**
     * Constructor
     */
    __device__ __forceinline__ PersistentBlockHisto256(
        SmemStorage         &smem_storage,                                  ///< Reference to smem_storage
        InputIteratorRA     d_in,                                           ///< Input data to reduce
        HistoCounter*       (&d_out_histograms)[ACTIVE_CHANNELS]) :         ///< Reference to output histograms
            smem_storage(smem_storage),
            d_in(d_in),
            d_out_histograms(d_out_histograms)
    {
        // Initialize histogram bin counts to zeros
        #pragma unroll
        for (int CHANNEL = 0; CHANNEL < ACTIVE_CHANNELS; ++CHANNEL)
        {
            int histo_offset = 0;

            #pragma unroll
            for(; histo_offset + BLOCK_THREADS <= 256; histo_offset += BLOCK_THREADS)
            {
                smem_storage.histograms[CHANNEL][histo_offset + threadIdx.x] = 0;
            }
            // Finish up with guarded initialization if necessary
            if ((histo_offset < BLOCK_THREADS) && (histo_offset + threadIdx.x < 256))
            {
                smem_storage.histograms[CHANNEL][histo_offset + threadIdx.x] = 0;
            }
        }
    }


    /**
     * The number of items processed per "tile"
     */
    __device__ __forceinline__ int TileItems()
    {
        return TILE_ITEMS;
    }


    /**
     * Process a single tile.
     */
    __device__ __forceinline__ void ConsumeTile(
        bool    &sync_after,
        SizeT   block_offset,
        int     num_valid)
    {
        if (num_valid < TILE_ITEMS)
        {
            // Only a partially-full tile of samples to read and composite
            int bounds = num_valid - (threadIdx.x * CHANNELS);

            #pragma unroll
            for (int ITEM = 0; ITEM < ITEMS_PER_THREAD; ++ITEM)
            {
                #pragma unroll
                for (int CHANNEL = 0; CHANNEL < CHANNELS; ++CHANNEL)
                {
                    if (((ACTIVE_CHANNELS == CHANNELS) || (CHANNEL < ACTIVE_CHANNELS)) && ((ITEM * BLOCK_THREADS * CHANNELS) + CHANNEL < bounds))
                    {
                        unsigned char item  = d_in[block_offset + (ITEM * BLOCK_THREADS * CHANNELS) + (threadIdx.x * CHANNELS) + CHANNEL];
                        atomicAdd(smem_storage.histograms[CHANNEL] + item, 1);
                    }
                }
            }

        }
        else
        {
            // Full tile of samples to read and composite
            unsigned char items[ITEMS_PER_THREAD][CHANNELS];

            #pragma unroll
            for (int ITEM = 0; ITEM < ITEMS_PER_THREAD; ITEM++)
            {
                #pragma unroll
                for (int CHANNEL = 0; CHANNEL < CHANNELS; ++CHANNEL)
                {
                    if (CHANNEL < ACTIVE_CHANNELS)
                    {
                        items[ITEM][CHANNEL] = d_in[block_offset + (ITEM * BLOCK_THREADS * CHANNELS) + (threadIdx.x * CHANNELS) + CHANNEL];
                    }
                }
            }

            __threadfence_block();

            #pragma unroll
            for (int ITEM = 0; ITEM < ITEMS_PER_THREAD; ITEM++)
            {
                #pragma unroll
                for (int CHANNEL = 0; CHANNEL < CHANNELS; ++CHANNEL)
                {
                    if (CHANNEL < ACTIVE_CHANNELS)
                    {
                        atomicAdd(smem_storage.histograms[CHANNEL] + items[ITEM][CHANNEL], 1);
                    }
                }
            }
        }

        // No need to sync after processing this tile to ensure smem coherence
        sync_after = false;
    }


    /**
     * Finalize the computation.
     */
    __device__ __forceinline__ void Finalize(
        int dummy_result)
    {
        // Barrier to ensure shared memory histograms are coherent
        __syncthreads();

        // Copy shared memory histograms to output
        #pragma unroll
        for (int CHANNEL = 0; CHANNEL < ACTIVE_CHANNELS; ++CHANNEL)
        {
            int channel_offset  = (blockIdx.x * 256);
            int histo_offset    = 0;

            #pragma unroll
            for(; histo_offset + BLOCK_THREADS <= 256; histo_offset += BLOCK_THREADS)
            {
                d_out_histograms[CHANNEL][channel_offset + histo_offset + threadIdx.x] = smem_storage.histograms[CHANNEL][histo_offset + threadIdx.x];
            }
            // Finish up with guarded initialization if necessary
            if ((histo_offset < BLOCK_THREADS) && (histo_offset + threadIdx.x < 256))
            {
                d_out_histograms[CHANNEL][channel_offset + histo_offset + threadIdx.x] = smem_storage.histograms[CHANNEL][histo_offset + threadIdx.x];
            }
        }
    }
};


/**
 * Specialized for GRID_HISTO_256_SORT
 */
template <
    typename                PersistentBlockHisto256Policy,    ///< Tuning policy
    int                     CHANNELS,                   ///< Number of channels interleaved in the input data (may be greater than the number of active channels being histogrammed)
    int                     ACTIVE_CHANNELS,            ///< Number of channels actively being histogrammed
    typename                InputIteratorRA,            ///< The input iterator type (may be a simple pointer type).  Must have a value type that is assignable to <tt>unsigned char</tt>
    typename                HistoCounter,               ///< Integral type for counting sample occurrences per histogram bin
    typename                SizeT>                      ///< Integer type for offsets
struct PersistentBlockHisto256<PersistentBlockHisto256Policy, CHANNELS, ACTIVE_CHANNELS, InputIteratorRA, HistoCounter, SizeT, GRID_HISTO_256_SORT>
{
    //---------------------------------------------------------------------
    // Types and constants
    //---------------------------------------------------------------------

    // Constants
    enum
    {
        BLOCK_THREADS       = PersistentBlockHisto256Policy::BLOCK_THREADS,
        ITEMS_PER_THREAD    = PersistentBlockHisto256Policy::ITEMS_PER_THREAD,
        TILE_CHANNEL_ITEMS  = BLOCK_THREADS * ITEMS_PER_THREAD,
        TILE_ITEMS          = TILE_CHANNEL_ITEMS * CHANNELS,

        STRIPED_COUNTERS_PER_THREAD = (256 + BLOCK_THREADS - 1) / BLOCK_THREADS,
    };

    // Parameterize BlockRadixSort type for our thread block
    typedef BlockRadixSort<unsigned char, BLOCK_THREADS, ITEMS_PER_THREAD> BlockRadixSortT;

    // Parameterize BlockDiscontinuity type for our thread block
    typedef BlockDiscontinuity<unsigned char, BLOCK_THREADS> BlockDiscontinuityT;

    // Shared memory type required by this thread block
    union SmemStorage
    {
        // Storage for sorting bin values
        typename BlockRadixSortT::SmemStorage sort_storage;

        struct
        {
            // Storage for detecting discontinuities in the tile of sorted bin values
            typename BlockDiscontinuityT::SmemStorage discont_storage;

            // Storage for noting begin/end offsets of bin runs in the tile of sorted bin values
            unsigned int run_begin[BLOCK_THREADS * STRIPED_COUNTERS_PER_THREAD];
            unsigned int run_end[BLOCK_THREADS * STRIPED_COUNTERS_PER_THREAD];
        };
    };


    // Discontinuity functor
    struct DiscontinuityOp
    {
        // Reference to smem_storage
        SmemStorage &smem_storage;

        // Constructor
        __device__ __forceinline__ DiscontinuityOp(SmemStorage &smem_storage) : smem_storage(smem_storage) {}

        // Discontinuity predicate
        __device__ __forceinline__ bool operator()(const unsigned char &a, const unsigned char &b, unsigned int b_index)
        {
            if (a != b)
            {
                // Note the begin/end offsets in shared storage
                smem_storage.run_begin[b] = b_index;
                smem_storage.run_end[a] = b_index;

                return true;
            }
            else
            {
                return false;
            }
        }
    };


    //---------------------------------------------------------------------
    // Per-thread fields
    //---------------------------------------------------------------------

    /// Reference to smem_storage
    SmemStorage &smem_storage;

    /// Histogram counters striped across threads
    HistoCounter thread_counters[ACTIVE_CHANNELS][STRIPED_COUNTERS_PER_THREAD];

    /// Reference to output histograms
    HistoCounter* (&d_out_histograms)[ACTIVE_CHANNELS];

    /// Input data to reduce
    InputIteratorRA d_in;


    //---------------------------------------------------------------------
    // Interface
    //---------------------------------------------------------------------

    /**
     * Constructor
     */
    __device__ __forceinline__ PersistentBlockHisto256(
        SmemStorage         &smem_storage,                                  ///< Reference to smem_storage
        InputIteratorRA     d_in,                                           ///< Input data to reduce
        HistoCounter*       (&d_out_histograms)[ACTIVE_CHANNELS]) :         ///< Reference to output histograms
            smem_storage(smem_storage),
            d_in(d_in),
            d_out_histograms(d_out_histograms)
    {
        // Initialize histogram counters striped across threads
        #pragma unroll
        for (int CHANNEL = 0; CHANNEL < ACTIVE_CHANNELS; ++CHANNEL)
        {
            #pragma unroll
            for (int COUNTER = 0; COUNTER < STRIPED_COUNTERS_PER_THREAD; ++COUNTER)
            {
                thread_counters[CHANNEL][COUNTER] = 0;
            }
        }
    }


    /**
     * The number of items processed per "tile"
     */
    __device__ __forceinline__ int TileItems()
    {
        return TILE_ITEMS;
    }


    /**
     * Composite a tile of input items
     */
    __device__ __forceinline__ void Composite(
        unsigned char   (&items)[ITEMS_PER_THREAD],                     ///< Tile of samples
        HistoCounter    thread_counters[STRIPED_COUNTERS_PER_THREAD])   ///< Histogram counters striped across threads
    {
        // Sort bytes in blocked arrangement
        BlockRadixSortT::SortBlocked(smem_storage.sort_storage, items);

        __syncthreads();

        // Initialize the shared memory's run_begin and run_end for each bin
        #pragma unroll
        for (int COUNTER = 0; COUNTER < STRIPED_COUNTERS_PER_THREAD; ++COUNTER)
        {
            smem_storage.run_begin[(COUNTER * BLOCK_THREADS) + threadIdx.x] = TILE_CHANNEL_ITEMS;
            smem_storage.run_end[(COUNTER * BLOCK_THREADS) + threadIdx.x] = TILE_CHANNEL_ITEMS;
        }

        __syncthreads();

        // Note the begin/end run offsets of bin runs in the sorted tile
        int flags[ITEMS_PER_THREAD];                // unused
        DiscontinuityOp flag_op(smem_storage);
        BlockDiscontinuityT::Flag(smem_storage.discont_storage, items, flag_op, flags);

        // Update begin for first item
        if (threadIdx.x == 0) smem_storage.run_begin[items[0]] = 0;

        __syncthreads();

        // Composite into histogram
        // Initialize the shared memory's run_begin and run_end for each bin
        #pragma unroll
        for (int COUNTER = 0; COUNTER < STRIPED_COUNTERS_PER_THREAD; ++COUNTER)
        {
            int bin = (COUNTER * BLOCK_THREADS) + threadIdx.x;
            thread_counters[COUNTER] += smem_storage.run_end[bin] - smem_storage.run_begin[bin];
        }
    }


    /**
     * Process one channel within a tile.
     */
    __device__ __forceinline__ void ConsumeTileChannel(
        int     channel,
        SizeT   block_offset,
        int     num_valid)
    {
        // Load items in striped fashion
        if (num_valid < TILE_ITEMS)
        {
            // Only a partially-full tile of samples to read and composite
            unsigned char items[ITEMS_PER_THREAD];

            // Assign our tid as the bin for out-of-bounds items (to give an even distribution), and keep track of how oob items to subtract out later
            int bounds = (num_valid - (threadIdx.x * CHANNELS));

            #pragma unroll
            for (int ITEM = 0; ITEM < ITEMS_PER_THREAD; ITEM++)
            {
                items[ITEM] = ((ITEM * BLOCK_THREADS * CHANNELS) < bounds) ?
                    d_in[channel + block_offset + (ITEM * BLOCK_THREADS * CHANNELS) + (threadIdx.x * CHANNELS)] :
                    0;
            }

            // Composite our histogram data
            Composite(items, thread_counters[channel]);

            __syncthreads();

            // Correct the overcounting in the zero-bin from invalid (out-of-bounds) items
            if (threadIdx.x == 0)
            {
                int extra = (TILE_ITEMS - num_valid) / CHANNELS;
                thread_counters[channel][0] -= extra;
            }
        }
        else
        {
            // Full tile of samples to read and composite
            unsigned char items[ITEMS_PER_THREAD];

            // Unguarded loads
            #pragma unroll
            for (int ITEM = 0; ITEM < ITEMS_PER_THREAD; ITEM++)
            {
                items[ITEM] = d_in[channel + block_offset + (ITEM * BLOCK_THREADS * CHANNELS) + (threadIdx.x * CHANNELS)];
            }

            // Composite our histogram data
            Composite(items, thread_counters[channel]);
        }
    }


    /**
     * Template iteration over channels (to silence not-unrolled warnings for SM10-13).  Inductive step.
     */
    template <int CHANNEL, int END>
    struct IterateChannels
    {
        /**
         * Process one channel within a tile.
         */
        static __device__ __forceinline__ void ConsumeTileChannel(
            PersistentBlockHisto256   *persistent_block_histo,
            SizeT           block_offset,
            int             num_valid)
        {
            __syncthreads();

            persistent_block_histo->ConsumeTileChannel(CHANNEL, block_offset, num_valid);

            IterateChannels<CHANNEL + 1, END>::ConsumeTileChannel(persistent_block_histo, block_offset, num_valid);
        }
    };


    /**
     * Template iteration over channels (to silence not-unrolled warnings for SM10-13).  Base step.
     */
    template <int END>
    struct IterateChannels<END, END>
    {
        static __device__ __forceinline__ void ConsumeTileChannel(PersistentBlockHisto256 *persistent_block_histo, SizeT block_offset, int num_valid) {}
    };


    /**
     * Process a single tile.
     *
     * We take several passes through the tile in this variant, extracting the samples for one channel at a time
     */
    __device__ __forceinline__ void ConsumeTile(
        bool    &sync_after,
        SizeT   block_offset,
        int     num_valid)
    {
        // First channel
        ConsumeTileChannel(0, block_offset, num_valid);

        // Iterate through remaining channels
        IterateChannels<1, ACTIVE_CHANNELS>::ConsumeTileChannel(this, block_offset, num_valid);

        // Need to sync after processing this tile to ensure smem coherence
        sync_after = true;
    }


    /**
     * Finalize the computation.
     */
    __device__ __forceinline__ void Finalize(
        int dummy_result)
    {
        // Copy counters striped across threads into the histogram output
        #pragma unroll
        for (int CHANNEL = 0; CHANNEL < ACTIVE_CHANNELS; ++CHANNEL)
        {
            int channel_offset  = (blockIdx.x * 256);

            #pragma unroll
            for (int COUNTER = 0; COUNTER < STRIPED_COUNTERS_PER_THREAD; ++COUNTER)
            {
                int bin = (COUNTER * BLOCK_THREADS) + threadIdx.x;

                if ((STRIPED_COUNTERS_PER_THREAD * BLOCK_THREADS == 256) || (bin < 256))
                {
                    d_out_histograms[CHANNEL][channel_offset + bin] = thread_counters[CHANNEL][COUNTER];
                }
            }
        }
    }
};




}               // CUB namespace
CUB_NS_POSTFIX  // Optional outer namespace(s)

