
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
 * cub::DeviceReduceByKey provides device-wide, parallel operations for reducing segments of values residing within global memory.
 */

#pragma once

#include <stdio.h>
#include <iterator>

#include "device_scan.cuh"
#include "region/block_reduce_by_key_region.cuh"
#include "../thread/thread_operators.cuh"
#include "../grid/grid_queue.cuh"
#include "../util_device.cuh"
#include "../util_namespace.cuh"

/// Optional outer namespace(s)
CUB_NS_PREFIX

/// CUB namespace
namespace cub {


/******************************************************************************
 * Kernel entry points
 *****************************************************************************/

#ifndef DOXYGEN_SHOULD_SKIP_THIS    // Do not document


/**
 * Reduce-by-key kernel entry point (multi-block)
 */
template <
    typename            BlockReduceByKeyRegionPolicy,   ///< Parameterized BlockReduceByKeyRegionPolicy tuning policy type
    typename            KeyInputIterator,               ///< Random-access input iterator type for keys
    typename            KeyOutputIterator,              ///< Random-access output iterator type for keys
    typename            ValueInputIterator,             ///< Random-access input iterator type for values
    typename            ValueOutputIterator,            ///< Random-access output iterator type for values
    typename            NumSegmentsIterator,            ///< Output iterator type for recording number of segments encountered
    typename            TileLookbackStatus,             ///< Tile status interface type
    typename            EqualityOp,                     ///< Key equality operator type
    typename            ReductionOp,                    ///< Value reduction operator type
    typename            Offset>                         ///< Signed integer type for global offsets
__launch_bounds__ (int(BlockReduceByKeyRegionPolicy::BLOCK_THREADS))
__global__ void ReduceByKeyRegionKernel(
    KeyInputIterator    d_keys_in,                      ///< [in] Pointer to consecutive runs of input keys
    KeyOutputIterator   d_keys_out,                     ///< [in] Pointer to output keys (one key per run)
    ValueInputIterator  d_values_in,                    ///< [in] Pointer to consecutive runs of input values
    ValueOutputIterator d_values_out,                   ///< [in] Pointer to output value aggregates (one aggregate per run)
    NumSegmentsIterator d_num_segments,                 ///< [in] Pointer to total number of runs
    TileLookbackStatus  tile_status,                    ///< [in] Tile status interface
    EqualityOp          equality_op,                    ///< [in] Key equality operator
    ReductionOp         reduction_op,                   ///< [in] Value reduction operator
    Offset              num_items,                      ///< [in] Total number of items to select from
    int                 num_tiles,                      ///< [in] Total number of tiles for the entire problem
    GridQueue<int>      queue)                          ///< [in] Drain queue descriptor for dynamically mapping tile data onto thread blocks
{
    // Thread block type for reducing tiles of value segments
    typedef BlockReduceByKeyRegion<
        BlockReduceByKeyRegionPolicy,
        KeyInputIterator,
        KeyOutputIterator,
        ValueInputIterator,
        ValueOutputIterator,
        EqualityOp,
        ReductionOp,
        Offset> BlockReduceByKeyRegionT;

    // Shared memory for BlockReduceByKeyRegion
    __shared__ typename BlockReduceByKeyRegionT::TempStorage temp_storage;

    // Process tiles
    BlockReduceByKeyRegionT(temp_storage, d_keys_in, d_keys_out, d_values_in, d_values_out, equality_op, reduction_op, num_items).ConsumeRegion(
        num_tiles,
        queue,
        tile_status,
        d_num_segments);
}




/******************************************************************************
 * Dispatch
 ******************************************************************************/

/**
 * Utility class for dispatching the appropriately-tuned kernels for DeviceReduceByKey
 */
template <
    typename    KeyInputIterator,               ///< Random-access input iterator type for keys
    typename    KeyOutputIterator,              ///< Random-access output iterator type for keys
    typename    ValueInputIterator,             ///< Random-access input iterator type for values
    typename    ValueOutputIterator,            ///< Random-access output iterator type for values
    typename    NumSegmentsIterator,            ///< Output iterator type for recording number of segments encountered
    typename    EqualityOp,                     ///< Key equality operator type
    typename    ReductionOp,                    ///< Value reduction operator type
    typename    Offset>                         ///< Signed integer type for global offsets
struct DeviceReduceByKeyDispatch
{
    /******************************************************************************
     * Types and constants
     ******************************************************************************/

    // Data type of key input iterator
    typedef typename std::iterator_traits<KeyInputIterator>::value_type Key;

    // Data type of value input iterator
    typedef typename std::iterator_traits<ValueInputIterator>::value_type Value;

    enum
    {
        INIT_KERNEL_THREADS     = 128,
        MAX_INPUT_BYTES         = CUB_MAX(sizeof(Key), sizeof(Value)),
        COMBINED_INPUT_BYTES    = sizeof(Key) + sizeof(Value),
    };

    // Value-offset tuple type for scanning (maps accumulated values to segment index)
    typedef ItemOffsetPair<Value, Offset> ValueOffsetPair;

    // Tile status descriptor interface type
    typedef ReduceByKeyTileLookbackStatus<Value, Offset> TileLookbackStatus;


    /******************************************************************************
     * Tuning policies
     ******************************************************************************/

    /// SM35
    struct Policy350
    {
        enum {
            NOMINAL_4B_ITEMS_PER_THREAD = 8,
            ITEMS_PER_THREAD            = (MAX_INPUT_BYTES <= 8) ? 8 : CUB_MIN(NOMINAL_4B_ITEMS_PER_THREAD, CUB_MAX(1, ((NOMINAL_4B_ITEMS_PER_THREAD * 8) + COMBINED_INPUT_BYTES - 1) / COMBINED_INPUT_BYTES)),
        };

        typedef BlockReduceByKeyRegionPolicy<
                128,
                ITEMS_PER_THREAD,
                BLOCK_LOAD_DIRECT,
                LOAD_LDG,
                true,
                BLOCK_SCAN_WARP_SCANS>
            ReduceByKeyPolicy;
    };

    /// SM30
    struct Policy300
    {
        enum {
            NOMINAL_4B_ITEMS_PER_THREAD = 6,
            ITEMS_PER_THREAD            = CUB_MIN(NOMINAL_4B_ITEMS_PER_THREAD, CUB_MAX(1, ((NOMINAL_4B_ITEMS_PER_THREAD * 8) + COMBINED_INPUT_BYTES - 1) / COMBINED_INPUT_BYTES)),
        };

        typedef BlockReduceByKeyRegionPolicy<
                128,
                ITEMS_PER_THREAD,
                BLOCK_LOAD_WARP_TRANSPOSE,
                LOAD_DEFAULT,
                true,
                BLOCK_SCAN_WARP_SCANS>
            ReduceByKeyPolicy;
    };

    /// SM20
    struct Policy200
    {
        enum {
            NOMINAL_4B_ITEMS_PER_THREAD = 13,
            ITEMS_PER_THREAD            = CUB_MIN(NOMINAL_4B_ITEMS_PER_THREAD, CUB_MAX(1, ((NOMINAL_4B_ITEMS_PER_THREAD * 8) + COMBINED_INPUT_BYTES - 1) / COMBINED_INPUT_BYTES)),
        };

        typedef BlockReduceByKeyRegionPolicy<
                128,
                ITEMS_PER_THREAD,
                BLOCK_LOAD_WARP_TRANSPOSE,
                LOAD_DEFAULT,
                true,
                BLOCK_SCAN_WARP_SCANS>
            ReduceByKeyPolicy;
    };

    /// SM13
    struct Policy130
    {
        enum {
            NOMINAL_4B_ITEMS_PER_THREAD = 7,
            ITEMS_PER_THREAD            = CUB_MIN(NOMINAL_4B_ITEMS_PER_THREAD, CUB_MAX(1, ((NOMINAL_4B_ITEMS_PER_THREAD * 8) + COMBINED_INPUT_BYTES - 1) / COMBINED_INPUT_BYTES)),
        };

        typedef BlockReduceByKeyRegionPolicy<
                128,
                ITEMS_PER_THREAD,
                BLOCK_LOAD_WARP_TRANSPOSE,
                LOAD_DEFAULT,
                true,
                BLOCK_SCAN_WARP_SCANS>
            ReduceByKeyPolicy;
    };

    /// SM10
    struct Policy100
    {
        enum {
            NOMINAL_4B_ITEMS_PER_THREAD = 5,
            ITEMS_PER_THREAD            = CUB_MIN(NOMINAL_4B_ITEMS_PER_THREAD, CUB_MAX(1, (NOMINAL_4B_ITEMS_PER_THREAD * 8) / COMBINED_INPUT_BYTES)),
        };

        typedef BlockReduceByKeyRegionPolicy<
                64,
                ITEMS_PER_THREAD,
                BLOCK_LOAD_WARP_TRANSPOSE,
                LOAD_DEFAULT,
                true,
                BLOCK_SCAN_RAKING>
            ReduceByKeyPolicy;
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
    struct PtxReduceByKeyPolicy : PtxPolicy::ReduceByKeyPolicy {};


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
        KernelConfig    &reduce_by_key_region_config)
    {
    #if (CUB_PTX_VERSION > 0)

        // We're on the device, so initialize the kernel dispatch configurations with the current PTX policy
        reduce_by_key_region_config.template Init<PtxReduceByKeyPolicy>();

    #else

        // We're on the host, so lookup and initialize the kernel dispatch configurations with the policies that match the device's PTX version
        if (ptx_version >= 350)
        {
            reduce_by_key_region_config.template Init<typename Policy350::ReduceByKeyPolicy>();
        }
        else if (ptx_version >= 300)
        {
            reduce_by_key_region_config.template Init<typename Policy300::ReduceByKeyPolicy>();
        }
        else if (ptx_version >= 200)
        {
            reduce_by_key_region_config.template Init<typename Policy200::ReduceByKeyPolicy>();
        }
        else if (ptx_version >= 130)
        {
            reduce_by_key_region_config.template Init<typename Policy130::ReduceByKeyPolicy>();
        }
        else
        {
            reduce_by_key_region_config.template Init<typename Policy100::ReduceByKeyPolicy>();
        }

    #endif
    }


    /**
     * Kernel kernel dispatch configuration.  Mirrors the constants within BlockReduceByKeyRegionPolicy.
     */
    struct KernelConfig
    {
        int                     block_threads;
        int                     items_per_thread;
        BlockLoadAlgorithm      load_policy;
        bool                    two_phase_scatter;
        BlockScanAlgorithm      scan_algorithm;
        cudaSharedMemConfig     smem_config;

        template <typename BlockReduceByKeyRegionPolicy>
        __host__ __device__ __forceinline__
        void Init()
        {
            block_threads               = BlockReduceByKeyRegionPolicy::BLOCK_THREADS;
            items_per_thread            = BlockReduceByKeyRegionPolicy::ITEMS_PER_THREAD;
            load_policy                 = BlockReduceByKeyRegionPolicy::LOAD_ALGORITHM;
            two_phase_scatter           = BlockReduceByKeyRegionPolicy::TWO_PHASE_SCATTER;
            scan_algorithm              = BlockReduceByKeyRegionPolicy::SCAN_ALGORITHM;
            smem_config                 = cudaSharedMemBankSizeEightByte;
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
        typename                    ReduceByKeyRegionKernelPtr>     ///< Function type of cub::ReduceByKeyRegionKernelPtr
    __host__ __device__ __forceinline__
    static cudaError_t Dispatch(
        void                        *d_temp_storage,                ///< [in] %Device allocation of temporary storage.  When NULL, the required allocation size is written to \p temp_storage_bytes and no work is done.
        size_t                      &temp_storage_bytes,            ///< [in,out] Reference to size in bytes of \p d_temp_storage allocation
        KeyInputIterator            d_keys_in,                      ///< [in] Pointer to consecutive runs of input keys
        KeyOutputIterator           d_keys_out,                     ///< [in] Pointer to output keys (one key per run)
        ValueInputIterator          d_values_in,                    ///< [in] Pointer to consecutive runs of input values
        ValueOutputIterator         d_values_out,                   ///< [in] Pointer to output value aggregates (one aggregate per run)
        NumSegmentsIterator         d_num_segments,                 ///< [in] Pointer to total number of runs
        EqualityOp                  equality_op,                    ///< [in] Key equality operator
        ReductionOp                 reduction_op,                   ///< [in] Value reduction operator
        Offset                      num_items,                      ///< [in] Total number of items to select from
        cudaStream_t                stream,                         ///< [in] CUDA stream to launch kernels within.  Default is stream<sub>0</sub>.
        bool                        debug_synchronous,              ///< [in] Whether or not to synchronize the stream after every kernel launch to check for errors.  Also causes launch configurations to be printed to the console.  Default is \p false.
        int                         ptx_version,                    ///< [in] PTX version of dispatch kernels
        ScanInitKernelPtr           init_kernel,                    ///< [in] Kernel function pointer to parameterization of cub::ScanInitKernel
        ReduceByKeyRegionKernelPtr  reduce_by_key_region_kernel,    ///< [in] Kernel function pointer to parameterization of cub::ReduceByKeyRegionKernel
        KernelConfig                reduce_by_key_region_config)    ///< [in] Dispatch parameters that match the policy that \p reduce_by_key_region_kernel was compiled for
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
            int tile_size = reduce_by_key_region_config.block_threads * reduce_by_key_region_config.items_per_thread;
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

            // Get SM occupancy for reduce_by_key_region_kernel
            int reduce_by_key_region_sm_occupancy;
            if (CubDebug(error = MaxSmOccupancy(
                reduce_by_key_region_sm_occupancy,            // out
                sm_version,
                reduce_by_key_region_kernel,
                reduce_by_key_region_config.block_threads))) break;

            // Get grid size for scanning tiles
            dim3 reduce_by_key_grid_size;
            if (ptx_version <= 130)
            {
                // Blocks are launched in order, so just assign one block per tile
                int max_dim_x = 32 * 1024;
                reduce_by_key_grid_size.z = 1;
                reduce_by_key_grid_size.y = (num_tiles + max_dim_x - 1) / max_dim_x;
                reduce_by_key_grid_size.x = CUB_MIN(num_tiles, max_dim_x);
            }
            else
            {
                // Blocks may not be launched in order, so use atomics
                int reduce_by_key_region_occupancy = reduce_by_key_region_sm_occupancy * sm_count;      // Whole-device occupancy for reduce_by_key_region_kernel
                reduce_by_key_grid_size.z = 1;
                reduce_by_key_grid_size.y = 1;
                reduce_by_key_grid_size.x = (num_tiles < reduce_by_key_region_occupancy) ?
                    num_tiles :                             // Not enough to fill the device with threadblocks
                    reduce_by_key_region_occupancy;         // Fill the device with threadblocks
            }

#if (CUB_PTX_VERSION == 0)
            // Get current smem bank configuration
            cudaSharedMemConfig original_smem_config;
            if (CubDebug(error = cudaDeviceGetSharedMemConfig(&original_smem_config))) break;
            cudaSharedMemConfig current_smem_config = original_smem_config;

            // Update smem config if necessary
            if (current_smem_config != reduce_by_key_region_config.smem_config)
            {
                if (CubDebug(error = cudaDeviceSetSharedMemConfig(reduce_by_key_region_config.smem_config))) break;
                current_smem_config = reduce_by_key_region_config.smem_config;
            }
#endif

            // Log reduce_by_key_region_kernel configuration
            if (debug_synchronous) CubLog("Invoking reduce_by_key_region_kernel<<<{%d,%d,%d}, %d, 0, %lld>>>(), %d items per thread, %d SM occupancy\n",
                reduce_by_key_grid_size.x, reduce_by_key_grid_size.y, reduce_by_key_grid_size.z, reduce_by_key_region_config.block_threads, (long long) stream, reduce_by_key_region_config.items_per_thread, reduce_by_key_region_sm_occupancy);

            // Invoke reduce_by_key_region_kernel
            reduce_by_key_region_kernel<<<reduce_by_key_grid_size, reduce_by_key_region_config.block_threads, 0, stream>>>(
                d_keys_in,
                d_keys_out,
                d_values_in,
                d_values_out,
                d_num_segments,
                tile_status,
                equality_op,
                reduction_op,
                num_items,
                num_tiles,
                queue);

            // Sync the stream if specified
            if (debug_synchronous && (CubDebug(error = SyncStream(stream)))) break;

#if (CUB_PTX_VERSION == 0)
            // Reset smem config if necessary
            if (current_smem_config != original_smem_config)
            {
                if (CubDebug(error = cudaDeviceSetSharedMemConfig(original_smem_config))) break;
            }
#endif

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
        KeyInputIterator            d_keys_in,                      ///< [in] Pointer to consecutive runs of input keys
        KeyOutputIterator           d_keys_out,                     ///< [in] Pointer to output keys (one key per run)
        ValueInputIterator          d_values_in,                    ///< [in] Pointer to consecutive runs of input values
        ValueOutputIterator         d_values_out,                   ///< [in] Pointer to output value aggregates (one aggregate per run)
        NumSegmentsIterator         d_num_segments,                 ///< [in] Pointer to total number of runs
        EqualityOp                  equality_op,                    ///< [in] Key equality operator
        ReductionOp                 reduction_op,                   ///< [in] Value reduction operator
        Offset                      num_items,                      ///< [in] Total number of items to select from
        cudaStream_t                stream,                         ///< [in] CUDA stream to launch kernels within.  Default is stream<sub>0</sub>.
        bool                        debug_synchronous)              ///< [in] Whether or not to synchronize the stream after every kernel launch to check for errors.  Also causes launch configurations to be printed to the console.  Default is \p false.
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
            KernelConfig reduce_by_key_region_config;
            InitConfigs(ptx_version, reduce_by_key_region_config);

            // Dispatch
            if (CubDebug(error = Dispatch(
                d_temp_storage,
                temp_storage_bytes,
                d_keys_in,
                d_keys_out,
                d_values_in,
                d_values_out,
                d_num_segments,
                equality_op,
                reduction_op,
                num_items,
                stream,
                debug_synchronous,
                ptx_version,
                ScanInitKernel<Offset, TileLookbackStatus>,
                ReduceByKeyRegionKernel<PtxReduceByKeyPolicy, KeyInputIterator, KeyOutputIterator, ValueInputIterator, ValueOutputIterator, NumSegmentsIterator, TileLookbackStatus, EqualityOp, ReductionOp, Offset>,
                reduce_by_key_region_config))) break;
        }
        while (0);

        return error;
    }
};



#endif // DOXYGEN_SHOULD_SKIP_THIS


}               // CUB namespace
CUB_NS_POSTFIX  // Optional outer namespace(s)


