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

namespace nvbio {
namespace cuda {

// granularity of shared memory allocation
inline void device_arch(uint32& major, uint32& minor)
{
    int            device;
    cudaGetDevice( &device );
    cudaDeviceProp properties;
    cudaGetDeviceProperties( &properties, device );

    major = properties.major;
    minor = properties.minor;
}

// granularity of the maximum grid size
inline uint32 max_grid_size()
{
    uint32 major, minor;
    device_arch( major, minor );
    return major <= 2 ? 32*1024 : uint32(-1);
}

// number of multiprocessors (for the current device)
inline size_t multiprocessor_count()
{
    int            device;
    cudaGetDevice( &device );

    cudaDeviceProp properties;
    cudaGetDeviceProperties( &properties, device );

    return properties.multiProcessorCount;
}

// granularity of shared memory allocation
inline size_t smem_allocation_unit(const cudaDeviceProp& properties)
{
    return 512;
}

// granularity of register allocation
inline size_t reg_allocation_unit(const cudaDeviceProp& properties, const size_t regsPerThread)
{
  switch(properties.major)
  {
    case 1:  return (properties.minor <= 1) ? 256 : 512;
    case 2:  switch(regsPerThread)
             {
               case 21:
               case 22:
               case 29:
               case 30:
               case 37:
               case 38:
               case 45:
               case 46:
                 return 128;
               default:
                 return 64;
             }
    case 3:  return 256;
    default: return 256; // unknown GPU; have to guess
  }
}

// granularity of warp allocation
inline size_t warp_allocation_multiple(const cudaDeviceProp& properties)
{
    return 2;
}

template <typename KernelFunction>
inline cudaFuncAttributes function_attributes(KernelFunction kernel)
{
    cudaFuncAttributes attributes;

#ifdef __CUDACC__
    typedef void (*fun_ptr_type)();

    fun_ptr_type fun_ptr = reinterpret_cast<fun_ptr_type>(kernel);

    cudaFuncGetAttributes(&attributes, fun_ptr);
#endif
    return attributes;
}

// maximum number of blocks per multiprocessor
inline size_t max_blocks_per_multiprocessor(const cudaDeviceProp& properties)
{
    return properties.major <= 2 ? 8 : 16;
}

// number of "sides" into which the multiprocessor is partitioned
inline size_t num_sides_per_multiprocessor(const cudaDeviceProp& properties)
{
  switch(properties.major)
  {
    case 1:  return 1;
    case 2:  return 2;
    case 3:  return 4;
    default: return 4; // unknown GPU; have to guess
  }
}

// number of registers allocated per block
inline size_t num_regs_per_block(const cudaDeviceProp& properties, const cudaFuncAttributes& attributes, const size_t CTA_SIZE)
{
    const size_t maxBlocksPerSM         = max_blocks_per_multiprocessor(properties);
    const size_t regAllocationUnit      = reg_allocation_unit(properties, attributes.numRegs);
    const size_t warpAllocationMultiple = warp_allocation_multiple(properties);

    // Number of warps (round up to nearest whole multiple of warp size & warp allocation multiple)
    const size_t numWarps = util::round_i(util::divide_ri(CTA_SIZE, properties.warpSize), warpAllocationMultiple);

    if (properties.major < 2)
        return util::round_i(attributes.numRegs * properties.warpSize * numWarps, regAllocationUnit);
    else
    {
        const size_t regsPerWarp = util::round_i(attributes.numRegs * properties.warpSize, regAllocationUnit);
        const size_t numSides = num_sides_per_multiprocessor(properties);
        const size_t numRegsPerSide = properties.regsPerBlock / numSides;
        return regsPerWarp > 0 ? ((numRegsPerSide / regsPerWarp) * numSides) / numWarps : maxBlocksPerSM;
    }
}

inline size_t max_active_blocks_per_multiprocessor(const cudaDeviceProp&        properties,
                                                   const cudaFuncAttributes&    attributes,
                                                   size_t CTA_SIZE,
                                                   size_t dynamic_smem_bytes)
{
    // Determine the maximum number of CTAs that can be run simultaneously per SM
    // This is equivalent to the calculation done in the CUDA Occupancy Calculator spreadsheet
    const size_t smemAllocationUnit     = smem_allocation_unit(properties);
    const size_t maxThreadsPerSM        = properties.maxThreadsPerMultiProcessor;  // 768, 1024, 1536, etc.
    const size_t maxBlocksPerSM         = max_blocks_per_multiprocessor(properties);

    // Number of regs is regs per thread times number of warps times warp size
    const size_t regsPerCTA = num_regs_per_block( properties, attributes, CTA_SIZE );

    const size_t smemBytes  = attributes.sharedSizeBytes + dynamic_smem_bytes;
    const size_t smemPerCTA = util::round_i(smemBytes, smemAllocationUnit);

    const size_t ctaLimitRegs    = regsPerCTA > 0 ? properties.regsPerBlock      / regsPerCTA : maxBlocksPerSM;
    const size_t ctaLimitSMem    = smemPerCTA > 0 ? properties.sharedMemPerBlock / smemPerCTA : maxBlocksPerSM;
    const size_t ctaLimitThreads =                  maxThreadsPerSM              / CTA_SIZE;

    return nvbio::min( (uint32)ctaLimitRegs, nvbio::min( (uint32)ctaLimitSMem, nvbio::min((uint32)ctaLimitThreads, (uint32)maxBlocksPerSM)));
}

template <typename KernelFunction>
size_t max_active_blocks_per_multiprocessor(KernelFunction kernel, const size_t CTA_SIZE, const size_t dynamic_smem_bytes)
{
    int            device;
    cudaGetDevice( &device );

    cudaDeviceProp properties;
    cudaGetDeviceProperties( &properties, device );

    cudaFuncAttributes attributes = function_attributes( kernel );

    return max_active_blocks_per_multiprocessor(properties, attributes, CTA_SIZE, dynamic_smem_bytes);
}

template <typename KernelFunction>
size_t max_active_blocks(KernelFunction kernel, const size_t CTA_SIZE, const size_t dynamic_smem_bytes)
{
    int            device;
    cudaGetDevice( &device );

    cudaDeviceProp properties;
    cudaGetDeviceProperties( &properties, device );

    cudaFuncAttributes attributes = function_attributes( kernel );

    return properties.multiProcessorCount * max_active_blocks_per_multiprocessor(properties, attributes, CTA_SIZE, dynamic_smem_bytes);
}

template <typename KernelFunction>
size_t num_registers(KernelFunction kernel)
{
    cudaFuncAttributes attributes = function_attributes( kernel );
    return attributes.numRegs;
}

inline size_t max_blocksize_with_highest_occupancy(const cudaDeviceProp&        properties,
                                                   const cudaFuncAttributes&    attributes,
                                                   size_t dynamic_smem_bytes_per_thread)
{
    size_t max_occupancy      = properties.maxThreadsPerMultiProcessor;
    size_t largest_blocksize  = nvbio::min( properties.maxThreadsPerBlock, attributes.maxThreadsPerBlock );
    size_t granularity        = properties.warpSize;
    size_t max_blocksize      = 0;
    size_t highest_occupancy  = 0;

    for(size_t blocksize = largest_blocksize; blocksize != 0; blocksize -= granularity)
    {
        size_t occupancy = blocksize * max_active_blocks_per_multiprocessor(properties, attributes, blocksize, dynamic_smem_bytes_per_thread * blocksize);

        if (occupancy > highest_occupancy)
        {
            max_blocksize = blocksize;
            highest_occupancy = occupancy;
        }

        // early out, can't do better
        if (highest_occupancy == max_occupancy)
            return max_blocksize;
    }

    return max_blocksize;
}

template <typename KernelFunction>
size_t max_blocksize_with_highest_occupancy(KernelFunction kernel, size_t dynamic_smem_bytes_per_thread)
{
    int            device;
    cudaDeviceProp properties;
    cudaGetDevice( &device );
    cudaGetDeviceProperties( &properties, device );

    cudaFuncAttributes attributes = function_attributes( kernel );

    return max_blocksize_with_highest_occupancy(properties, attributes, dynamic_smem_bytes_per_thread);
}

inline bool is_tcc_enabled()
{
    int            device;
    cudaDeviceProp device_properties;
    cudaGetDevice(&device);
    cudaGetDeviceProperties( &device_properties, device );
    return device_properties.tccDriver ? true : false;
}

inline void check_error(const char *message)
{
	cudaError_t error = cudaGetLastError();
	if(error != cudaSuccess)
    {
        const char* error_string = cudaGetErrorString(error);
		log_error(stderr,"%s: %s\n", message, error_string );
        throw cuda_error( error_string );
	}
}

// a generic syncthreads() implementation to synchronize contiguous
// blocks of N threads at a time
//
template <uint32 N>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void syncthreads()
{
    #if defined(NVBIO_DEVICE_COMPILATION)
        __syncthreads();
    #endif
}

} // namespace cuda
} // namespace nvbio
