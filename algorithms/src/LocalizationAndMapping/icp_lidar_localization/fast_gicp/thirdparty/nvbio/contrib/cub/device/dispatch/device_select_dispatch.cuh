
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
 * cub::DeviceSelect provides device-wide, parallel operations for selecting items from sequences of data items residing within global memory.
 */

#pragma once

#include <stdio.h>
#include <iterator>

#include "device_scan_dispatch.cuh"
#include "../block_region/block_region_select.cuh"
#include "../../thread/thread_operators.cuh"
#include "../../grid/grid_queue.cuh"
#include "../../util_device.cuh"
#include "../../util_namespace.cuh"

/// Optional outer namespace(s)
CUB_NS_PREFIX

/// CUB namespace
namespace cub {

/******************************************************************************
 * Kernel entry points
 *****************************************************************************/

/**
 * Select kernel entry point (multi-block)
 *
 * Performs functor-based selection if SelectOp functor type != NullType
 * Otherwise performs flag-based selection if FlagIterator's value type != NullType
 * Otherwise performs discontinuity selection (keep unique)
 */
template <
    typename            BlockRegionSelectPolicy,    ///< Parameterized BlockRegionSelectPolicy tuning policy type
    typename            InputIterator,              ///< Random-access input iterator type for reading input items
    typename            FlagIterator,               ///< Random-access input iterator type for reading selection flags (NullType* if a selection functor or discontinuity flagging is to be used for selection)
    typename            OutputIterator,             ///< Random-access output iterator type for writing selected items
    typename            NumSelectedIterator,        ///< Output iterator type for recording the number of items selected
    typename            TileLookbackStatus,         ///< Tile status interface type
    typename            SelectOp,                   ///< Selection operator type (NullType if selection flags or discontinuity flagging is to be used for selection)
    typename            EqualityOp,                 ///< Equality operator type (NullType if selection functor or selection flags is to be used for selection)
    typename            Offset,                     ///< Signed integer type for global offsets
    bool                KEEP_REJECTS>               ///< Whether or not we push rejected items to the back of the output
__launch_bounds__ (int(BlockRegionSelectPolicy::BLOCK_THREADS))
__global__ void SelectRegionKernel(
    InputIterator       d_in,                       ///< [in] Pointer to input sequence of data items
    FlagIterator        d_flags,                    ///< [in] Pointer to the input sequence of selection flags
    OutputIterator      d_out,                      ///< [in] Pointer to output sequence of selected data items
    NumSelectedIterator d_num_selected,             ///< [in] Pointer to total number of items selected (i.e., length of \p d_out)
    TileLookbackStatus  tile_status,                ///< [in] Tile status interface
    SelectOp            select_op,                  ///< [in] Selection operator
    EqualityOp          equality_op,                ///< [in] Equality operator
    Offset              num_items,                  ///< [in] Total number of input items (i.e., length of \p d_in)
    int                 num_tiles,                  ///< [in] Total number of tiles for the entire problem
    GridQueue<int>      queue)                      ///< [in] Drain queue descriptor for dynamically mapping tile data onto thread blocks
{
    // Thread block type for selecting data from input tiles
    typedef BlockRegionSelect<
        BlockRegionSelectPolicy,
        InputIterator,
        FlagIterator,
        OutputIterator,
        SelectOp,
        EqualityOp,
        Offset,
        KEEP_REJECTS> BlockRegionSelectT;

    // Shared memory for BlockRegionSelect
    __shared__ typename BlockRegionSelectT::TempStorage temp_storage;

    // Process tiles
    BlockRegionSelectT(temp_storage, d_in, d_flags, d_out, select_op, equality_op, num_items).ConsumeRegion(
        num_tiles,
        queue,
        tile_status,
        d_num_selected);
}




/******************************************************************************
 * Dispatch
 ******************************************************************************/

/**
 * Utility class for dispatching the appropriately-tuned kernels for DeviceSelect
 */
template <
    typename    InputIterator,                  ///< Random-access input iterator type for reading input items
    typename    FlagIterator,                   ///< Random-access input iterator type for reading selection flags (NullType* if a selection functor or discontinuity flagging is to be used for selection)
    typename    OutputIterator,                 ///< Random-access output iterator type for writing selected items
    typename    NumSelectedIterator,            ///< Output iterator type for recording the number of items selected
    typename    SelectOp,                       ///< Selection operator type (NullType if selection flags or discontinuity flagging is to be used for selection)
    typename    EqualityOp,                     ///< Equality operator type (NullType if selection functor or selection flags is to be used for selection)
    typename    Offset,                         ///< Signed integer type for global offsets
    bool        KEEP_REJECTS>                   ///< Whether or not we push rejected items to the back of the output
struct DeviceSelectDispatch
{
    /******************************************************************************
     * Types and constants
     ******************************************************************************/

    // Data type of input iterator
    typedef typename std::iterator_traits<InputIterator>::value_type T;

    // Data type of flag iterator
    typedef typename std::iterator_traits<FlagIterator>::value_type Flag;

    enum
    {
        INIT_KERNEL_THREADS = 128,
    };

    // Tile status descriptor interface type
    typedef TileLookbackStatus<Offset> TileLookbackStatus;


    /******************************************************************************
     * Tuning policies
     ******************************************************************************/

    /// SM35
    struct Policy350
    {
        enum {
            NOMINAL_4B_ITEMS_PER_THREAD = 11,
            ITEMS_PER_THREAD            = CUB_MIN(NOMINAL_4B_ITEMS_PER_THREAD, CUB_MAX(1, (NOMINAL_4B_ITEMS_PER_THREAD * 4 / sizeof(T)))),
        };

        typedef BlockRegionSelectPolicy<
                128,
                ITEMS_PER_THREAD,
                BLOCK_LOAD_DIRECT,
                LOAD_LDG,
                true,
                BLOCK_SCAN_WARP_SCANS>
            SelectRegionPolicy;
    };

    /// SM30
    struct Policy300
    {
        enum {
            NOMINAL_4B_ITEMS_PER_THREAD = 5,
            ITEMS_PER_THREAD            = CUB_MIN(NOMINAL_4B_ITEMS_PER_THREAD, CUB_MAX(1, (NOMINAL_4B_ITEMS_PER_THREAD * 4 / sizeof(T)))),
        };

        typedef BlockRegionSelectPolicy<
                256,
                ITEMS_PER_THREAD,
                BLOCK_LOAD_WARP_TRANSPOSE,
                LOAD_DEFAULT,
                true,
                BLOCK_SCAN_RAKING_MEMOIZE>
            SelectRegionPolicy;
    };

    /// SM20
    struct Policy200
    {
        enum {
            NOMINAL_4B_ITEMS_PER_THREAD = (KEEP_REJECTS) ? 7 : 17,
            ITEMS_PER_THREAD            = CUB_MIN(NOMINAL_4B_ITEMS_PER_THREAD, CUB_MAX(1, (NOMINAL_4B_ITEMS_PER_THREAD * 4 / sizeof(T)))),
        };

        typedef BlockRegionSelectPolicy<
                128,
                ITEMS_PER_THREAD,
                BLOCK_LOAD_WARP_TRANSPOSE,
                LOAD_DEFAULT,
                true,
                BLOCK_SCAN_WARP_SCANS>
            SelectRegionPolicy;
    };

    /// SM13
    struct Policy130
    {
        enum {
            NOMINAL_4B_ITEMS_PER_THREAD = 9,
            ITEMS_PER_THREAD            = CUB_MIN(NOMINAL_4B_ITEMS_PER_THREAD, CUB_MAX(1, (NOMINAL_4B_ITEMS_PER_THREAD * 4 / sizeof(T)))),
        };

        typedef BlockRegionSelectPolicy<
                64,
                ITEMS_PER_THREAD,
                BLOCK_LOAD_WARP_TRANSPOSE,
                LOAD_DEFAULT,
                true,
                BLOCK_SCAN_RAKING_MEMOIZE>
            SelectRegionPolicy;
    };

    /// SM10
    struct Policy100
    {
        enum {
            NOMINAL_4B_ITEMS_PER_THREAD = 9,
            ITEMS_PER_THREAD            = CUB_MIN(NOMINAL_4B_ITEMS_PER_THREAD, CUB_MAX(1, (NOMINAL_4B_ITEMS_PER_THREAD * 4 / sizeof(T)))),
        };

        typedef BlockRegionSelectPolicy<
                256,
                ITEMS_PER_THREAD,
                BLOCK_LOAD_WARP_TRANSPOSE,
                LOAD_DEFAULT,
                true,
                BLOCK_SCAN_RAKING_MEMOIZE>
            SelectRegionPolicy;
    };


    /******************************************************************************
     * Tuning policies of current PTX compiler pass
     ******************************************************************************/

#if (CUB_PTX_VERSION >= 350)
    typedef Policy350 PtxPolicy;

#elif (CUB_PTX_VERSION >= 300)
    typedef Policy300 PtxPolicy;

#elif (CUB_PTX_VERSION >= 200)
    typedef Policy200 PtxPolicy;

#elif (CUB_PTX_VERSION >= 130)
    typedef Policy130 PtxPolicy;

#else
    typedef Policy100 PtxPolicy;

#endif

    // "Opaque" policies (whose parameterizations aren't reflected in the type signature)
    struct PtxSelectRegionPolicy : PtxPolicy::SelectRegionPolicy {};


    /******************************************************************************
     * Utilities
     ******************************************************************************/

    /**
     * Initialize kernel dispatch configurations with the policies corresponding to the PTX assembly we will use
     */
    template <typename KernelConfig>
    __host__ __device__ __forceinline__
    static void InitConfigs(
        int             ptx_version,
        KernelConfig    &select_region_config)
    {
    #if (CUB_PTX_VERSION > 0)

        // We're on the device, so initialize the kernel dispatch configurations with the current PTX policy
        select_region_config.template Init<PtxSelectRegionPolicy>();

    #else

        // We're on the host, so lookup and initialize the kernel dispatch configurations with the policies that match the device's PTX version
        if (ptx_version >= 350)
        {
            select_region_config.template Init<typename Policy350::SelectRegionPolicy>();
        }
        else if (ptx_version >= 300)
        {
            select_region_config.template Init<typename Policy300::SelectRegionPolicy>();
        }
        else if (ptx_version >= 200)
        {
            select_region_config.template Init<typename Policy200::SelectRegionPolicy>();
        }
        else if (ptx_version >= 130)
        {
            select_region_config.template Init<typename Policy130::SelectRegionPolicy>();
        }
        else
        {
            select_region_config.template Init<typename Policy100::SelectRegionPolicy>();
        }

    #endif
    }


    /**
     * Kernel kernel dispatch configuration.  Mirrors the constants within BlockRegionSelectPolicy.
     */
    struct KernelConfig
    {
        int                     block_threads;
        int                     items_per_thread;
        BlockLoadAlgorithm      load_policy;
        bool                    two_phase_scatter;
        BlockScanAlgorithm      scan_algorithm;

        template <typename BlockRegionSelectPolicy>
        __host__ __device__ __forceinline__
        void Init()
        {
            block_threads               = BlockRegionSelectPolicy::BLOCK_THREADS;
            items_per_thread            = BlockRegionSelectPolicy::ITEMS_PER_THREAD;
            load_policy                 = BlockRegionSelectPolicy::LOAD_ALGORITHM;
            two_phase_scatter           = BlockRegionSelectPolicy::TWO_PHASE_SCATTER;
            scan_algorithm              = BlockRegionSelectPolicy::SCAN_ALGORITHM;
        }

        __host__ __device__ __forceinline__
        void Print()
        {
            printf("%d, %d, %d, %d, %d",
                block_threads,
                items_per_thread,
                load_policy,
                two_phase_scatter,
                scan_algorithm);
        }
    };


    /******************************************************************************
     * Dispatch entrypoints
     ******************************************************************************/

    /**
     * Internal dispatch routine for computing a device-wide prefix scan using the
     * specified kernel functions.
     */
    template <
        typename                    ScanInitKernelPtr,              ///< Function type of cub::ScanInitKernel
        typename                    SelectRegionKernelPtr>          ///< Function type of cub::SelectRegionKernelPtr
    __host__ __device__ __forceinline__
    static cudaError_t Dispatch(
        void                        *d_temp_storage,                ///< [in] %Device allocation of temporary storage.  When NULL, the required allocation size is written to \p temp_storage_bytes and no work is done.
        size_t                      &temp_storage_bytes,            ///< [in,out] Reference to size in bytes of \p d_temp_storage allocation
        InputIterator               d_in,                           ///< [in] Pointer to input sequence of data items
        FlagIterator                d_flags,                        ///< [in] Pointer to the input sequence of selection flags
        OutputIterator              d_out,                          ///< [in] Pointer to output sequence of selected data items
        NumSelectedIterator         d_num_selected,                 ///< [in] Pointer to total number of items selected (i.e., length of \p d_out)
        SelectOp                    select_op,                      ///< [in] Selection operator
        EqualityOp                  equality_op,                    ///< [in] Equality operator
        Offset                      num_items,                      ///< [in] Total number of input items (i.e., length of \p d_in)
        cudaStream_t                stream,                         ///< [in] CUDA stream to launch kernels within.  Default is stream<sub>0</sub>.
        bool                        debug_synchronous,              ///< [in] Whether or not to synchronize the stream after every kernel launch to check for errors.  Also causes launch configurations to be printed to the console.  Default is \p false.
        int                         ptx_version,                    ///< [in] PTX version of dispatch kernels
        ScanInitKernelPtr           init_kernel,                    ///< [in] Kernel function pointer to parameterization of cub::ScanInitKernel
        SelectRegionKernelPtr       select_region_kernel,           ///< [in] Kernel function pointer to parameterization of cub::SelectRegionKernel
        KernelConfig                select_region_config)           ///< [in] Dispatch parameters that match the policy that \p select_region_kernel was compiled for
    {

#ifndef CUB_RUNTIME_ENABLED

        // Kernel launch not supported from this device
        return CubDebug(cudaErrorNotSupported);

#else

        cudaError error = cudaSuccess;
        do
        {
            // Get device ordinal
            int device_ordinal;
            if (CubDebug(error = cudaGetDevice(&device_ordinal))) break;

            // Get device SM version
            int sm_version;
            if (CubDebug(error = SmVersion(sm_version, device_ordinal))) break;

            // Get SM count
            int sm_count;
            if (CubDebug(error = cudaDeviceGetAttribute (&sm_count, cudaDevAttrMultiProcessorCount, device_ordinal))) break;

            // Number of input tiles
            int tile_size = select_region_config.block_threads * select_region_config.items_per_thread;
            int num_tiles = (num_items + tile_size - 1) / tile_size;

            // Specify temporary storage allocation requirements
            size_t  allocation_sizes[2];
            if (CubDebug(error = TileLookbackStatus::AllocationSize(num_tiles, allocation_sizes[0]))) break;    // bytes needed for tile status descriptors
            allocation_sizes[1] = GridQueue<int>::AllocationSize();                                             // bytes needed for grid queue descriptor

            // Compute allocation pointers into the single storage blob (or set the necessary size of the blob)
            void* allocations[2];
            if (CubDebug(error = AliasTemporaries(d_temp_storage, temp_storage_bytes, allocations, allocation_sizes))) break;
            if (d_temp_storage == NULL)
            {
                // Return if the caller is simply requesting the size of the storage allocation
                return cudaSuccess;
            }

            // Construct the tile status interface
            TileLookbackStatus tile_status;
            if (CubDebug(error = tile_status.Init(num_tiles, allocations[0], allocation_sizes[0]))) break;

            // Construct the grid queue descriptor
            GridQueue<int> queue(allocations[1]);

            // Log init_kernel configuration
            int init_grid_size = (num_tiles + INIT_KERNEL_THREADS - 1) / INIT_KERNEL_THREADS;
            if (debug_synchronous) CubLog("Invoking init_kernel<<<%d, %d, 0, %lld>>>()\n", init_grid_size, INIT_KERNEL_THREADS, (long long) stream);

            // Invoke init_kernel to initialize tile descriptors and queue descriptors
            init_kernel<<<init_grid_size, INIT_KERNEL_THREADS, 0, stream>>>(
                queue,
                tile_status,
                num_tiles);

            // Sync the stream if specified
            if (debug_synchronous && (CubDebug(error = SyncStream(stream)))) break;

            // Get SM occupancy for select_region_kernel
            int select_region_sm_occupancy;
            if (CubDebug(error = MaxSmOccupancy(
                select_region_sm_occupancy,            // out
                sm_version,
                select_region_kernel,
                select_region_config.block_threads))) break;

            // Get grid size for scanning tiles
            dim3 select_grid_size;
            if (ptx_version <= 130)
            {
                // Blocks are launched in order, so just assign one block per tile
                int max_dim_x = 32 * 1024;
                select_grid_size.z = 1;
                select_grid_size.y = (num_tiles + max_dim_x - 1) / max_dim_x;
                select_grid_size.x = CUB_MIN(num_tiles, max_dim_x);
            }
            else
            {
                // Blocks may not be launched in order, so use atomics
                int select_region_occupancy = select_region_sm_occupancy * sm_count;        // Whole-device occupancy for select_region_kernel
                select_grid_size.z = 1;
                select_grid_size.y = 1;
                select_grid_size.x = (num_tiles < select_region_occupancy) ?
                    num_tiles :                     // Not enough to fill the device with threadblocks
                    select_region_occupancy;        // Fill the device with threadblocks
            }

            // Log select_region_kernel configuration
            if (debug_synchronous) CubLog("Invoking select_region_kernel<<<{%d,%d,%d}, %d, 0, %lld>>>(), %d items per thread, %d SM occupancy\n",
                select_grid_size.x, select_grid_size.y, select_grid_size.z, select_region_config.block_threads, (long long) stream, select_region_config.items_per_thread, select_region_sm_occupancy);

            // Invoke select_region_kernel
            select_region_kernel<<<select_grid_size, select_region_config.block_threads, 0, stream>>>(
                d_in,
                d_flags,
                d_out,
                d_num_selected,
                tile_status,
                select_op,
                equality_op,
                num_items,
                num_tiles,
                queue);

            // Sync the stream if specified
            if (debug_synchronous && (CubDebug(error = SyncStream(stream)))) break;
        }
        while (0);

        return error;

#endif  // CUB_RUNTIME_ENABLED
    }


    /**
     * Internal dispatch routine
     */
    __host__ __device__ __forceinline__
    static cudaError_t Dispatch(
        void                        *d_temp_storage,                ///< [in] %Device allocation of temporary storage.  When NULL, the required allocation size is written to \p temp_storage_bytes and no work is done.
        size_t                      &temp_storage_bytes,            ///< [in,out] Reference to size in bytes of \p d_temp_storage allocation
        InputIterator               d_in,                           ///< [in] Pointer to input sequence of data items
        FlagIterator                d_flags,                        ///< [in] Pointer to the input sequence of selection flags
        OutputIterator              d_out,                          ///< [in] Pointer to output sequence of selected data items
        NumSelectedIterator         d_num_selected,                 ///< [in] Pointer to total number of items selected (i.e., length of \p d_out)
        SelectOp                    select_op,                      ///< [in] Selection operator
        EqualityOp                  equality_op,                    ///< [in] Equality operator
        Offset                      num_items,                      ///< [in] Total number of input items (i.e., length of \p d_in)
        cudaStream_t                stream,                         ///< [in] <b>[optional]</b> CUDA stream to launch kernels within.  Default is stream<sub>0</sub>.
        bool                        debug_synchronous)              ///< [in] <b>[optional]</b> Whether or not to synchronize the stream after every kernel launch to check for errors.  Also causes launch configurations to be printed to the console.  Default is \p false.
    {
        cudaError error = cudaSuccess;
        do
        {
            // Get PTX version
            int ptx_version;
    #if (CUB_PTX_VERSION == 0)
            if (CubDebug(error = PtxVersion(ptx_version))) break;
    #else
            ptx_version = CUB_PTX_VERSION;
    #endif

            // Get kernel kernel dispatch configurations
            KernelConfig select_region_config;
            InitConfigs(ptx_version, select_region_config);

            // Dispatch
            if (CubDebug(error = Dispatch(
                d_temp_storage,
                temp_storage_bytes,
                d_in,
                d_flags,
                d_out,
                d_num_selected,
                select_op,
                equality_op,
                num_items,
                stream,
                debug_synchronous,
                ptx_version,
                ScanInitKernel<Offset, TileLookbackStatus>,
                SelectRegionKernel<PtxSelectRegionPolicy, InputIterator, FlagIterator, OutputIterator, NumSelectedIterator, TileLookbackStatus, SelectOp, EqualityOp, Offset, KEEP_REJECTS>,
                select_region_config))) break;
        }
        while (0);

        return error;
    }
};


}               // CUB namespace
CUB_NS_POSTFIX  // Optional outer namespace(s)


