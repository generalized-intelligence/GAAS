/******************************************************************************
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 *
 * Code and text by Sean Baxter, NVIDIA Research
 * See http://nvlabs.github.io/moderngpu for repository and documentation.
 *
 ******************************************************************************/

#pragma once

#include "../mgpuhost.cuh"
#include "../kernels/search.cuh"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// KernelBulkRemove
// Copy the values that are not matched by an index. This is like the 
// anti-gather.

template<typename Tuning, typename InputIt, typename IndicesIt, 
	typename OutputIt>
MGPU_LAUNCH_BOUNDS void KernelBulkRemove(InputIt source_global, int sourceCount, 
	IndicesIt indices_global, int indicesCount, const int* p_global,
	OutputIt dest_global) {

	typedef MGPU_LAUNCH_PARAMS Params;
	typedef typename std::iterator_traits<InputIt>::value_type T;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;

	typedef CTAScan<NT> S;
	union Shared {
		int indices[NV];
		typename S::Storage scan;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = block * NV;
	sourceCount = min(NV, sourceCount - gid);

	// Search for begin and end iterators of interval to load.
	int p0 = p_global[block];
	int p1 = p_global[block + 1];

	// Set the flags to 1. The default is to copy a value.
	#pragma unroll
	for(int i = 0; i < VT; ++i) {
		int index = NT * i + tid;
		shared.indices[index] = index < sourceCount;
	}
	__syncthreads();

	// Load the indices into register.
	int begin = p0;
	int indexCount = p1 - begin;
	int indices[VT];
	DeviceGlobalToReg<NT, VT>(indexCount, indices_global + begin, tid, indices);

	// Set the counter to 0 for each index we've loaded.
	#pragma unroll
	for(int i = 0; i < VT; ++i)
		if(NT * i + tid < indexCount) 
			shared.indices[indices[i] - gid] = 0;
	__syncthreads();

	// Run a raking scan over the flags. We count the set flags - this is the 
	// number of elements to load in per thread.
	int x = 0;
	#pragma unroll
	for(int i = 0; i < VT; ++i)
		x += indices[i] = shared.indices[VT * tid + i];
	__syncthreads();

	// Run a CTA scan and scatter the gather indices to shared memory.
	int scan = S::Scan(tid, x, shared.scan);
	#pragma unroll
	for(int i = 0; i < VT; ++i)
		if(indices[i]) shared.indices[scan++] = VT * tid + i;
	__syncthreads();

	// Load the gather indices into register.
	DeviceSharedToReg<NT, VT>(shared.indices, tid, indices);

	// Gather the data into register. The number of values to copy is 
	// sourceCount - indexCount.
	source_global += gid;
	int count = sourceCount - indexCount;
	T values[VT];
	DeviceGather<NT, VT>(count, source_global, indices, tid, values, false);

	// Store all the valid registers to dest_global.
	DeviceRegToGlobal<NT, VT>(count, values, tid, dest_global + gid - begin);
}

////////////////////////////////////////////////////////////////////////////////
// BulkRemove 

template<typename InputIt, typename IndicesIt, typename OutputIt>
MGPU_HOST void BulkRemove(InputIt source_global, int sourceCount,
	IndicesIt indices_global, int indicesCount, OutputIt dest_global,
	CudaContext& context) {

	const int NT = 128;
	const int VT = 11;
	typedef LaunchBoxVT<NT, VT> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;

	MGPU_MEM(int) partitionsDevice = BinarySearchPartitions<MgpuBoundsLower>(
		sourceCount, indices_global, indicesCount, NV, mgpu::less<int>(), 
		context);

	int numBlocks = MGPU_DIV_UP(sourceCount, NV);
	KernelBulkRemove<Tuning><<<numBlocks, launch.x, 0, context.Stream()>>>(
		source_global, sourceCount, indices_global, indicesCount, 
		partitionsDevice->get(), dest_global);
	MGPU_SYNC_CHECK("KernelBulkRemove");
}

} // namespace mgpu
