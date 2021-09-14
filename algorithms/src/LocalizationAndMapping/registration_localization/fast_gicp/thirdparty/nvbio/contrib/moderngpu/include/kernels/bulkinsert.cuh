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
// KernelBulkInsert

template<int NT, int VT>
struct CTABulkInsert {
	enum { NV = NT * VT };
	union Storage {
		int indices[NV];
		typename CTAScan<NT>::Storage scanStorage;
	};

	// Returns gather indices in strided order.
	template<typename IndicesIt>
	MGPU_DEVICE static void BuildGatherIndices(int4 range, 
		IndicesIt indices_global, int tid, Storage& storage) {

		int a0 = range.x;
		int b0 = range.z;
		int aCount = range.y - range.x;

		// Load the indices.
		int indices[VT];
		DeviceGlobalToReg<NT, VT>(aCount, indices_global + a0, tid, indices);

		// Initialize the indices to 0.
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			storage.indices[NT * i + tid] = 0;
		__syncthreads();

		// Set the counters for all the loaded indices. This has the effect of 
		// pushing the scanned values to the right, causing the B data to be 
		// inserted to the right of each insertion point.
		#pragma unroll
		for(int i = 0; i < VT; ++i) {
			int index = NT * i + tid;
			if(index < aCount) storage.indices[index + indices[i] - b0] = 1;
		}
		__syncthreads();

		// Run a raking scan over the indices.
		int x = 0;
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			x += indices[i] = storage.indices[VT * tid + i];
		__syncthreads();

		// Run a CTA scan over the thread totals.
		int scan = CTAScan<NT>::Scan(tid, x, storage.scanStorage);
		
		// Complete the scan to compute merge-style gather indices. Indices
		// between in the interval (0, aCount) are from array A (the new 
		// values). Indices in (aCount, aCount + bCount) are from array B (the
		// sources). This style of indexing lets us use 
		// DeviceTransferMergeValues to do global memory transfers. 
		#pragma unroll
		for(int i = 0; i < VT; ++i) {
			int index = VT * tid + i;
			int gather = indices[i] ? scan++ : aCount + index - scan;
			storage.indices[index] = gather;
		}
		__syncthreads();
	}	
};

// Insert the values from a_global into the positions marked by indices_global.
template<typename Tuning, typename InputIt1, typename IndicesIt, 
	typename InputIt2, typename OutputIt>
MGPU_LAUNCH_BOUNDS void KernelBulkInsert(InputIt1 a_global, 
	IndicesIt indices_global, int aCount, InputIt2 b_global, int bCount, 
	const int* mp_global, OutputIt dest_global) {

	typedef MGPU_LAUNCH_PARAMS Params;
	typedef typename std::iterator_traits<InputIt1>::value_type T;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;

	union Shared {
		typename CTABulkInsert<NT, VT>::Storage storage;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	
	int4 range = ComputeMergeRange(aCount, bCount, block, 0, NV, mp_global);
	
	CTABulkInsert<NT, VT>::BuildGatherIndices(range, indices_global, tid,
		shared.storage);

	int a0 = range.x;		// A is array of values to insert.
	int b0 = range.z;		// B is source array.
	aCount = range.y - range.x;
	bCount = range.w - range.z;

	DeviceTransferMergeValuesShared<NT, VT>(aCount + bCount, a_global + a0, 
		b_global + b0, aCount, shared.storage.indices, tid, 
		dest_global + a0 + b0, false);
}


////////////////////////////////////////////////////////////////////////////////
// BulkInsert
// Insert elements from A into elements from B before indices.

template<typename InputIt1, typename IndicesIt, typename InputIt2,
	typename OutputIt>
MGPU_HOST void BulkInsert(InputIt1 a_global, IndicesIt indices_global, 
	int aCount, InputIt2 b_global, int bCount, OutputIt dest_global,
	CudaContext& context) {

	typedef LaunchBoxVT<
		128, 7, 0,
		128, 7, 0,
		128, 7, 0
	> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;

	MGPU_MEM(int) partitionsDevice = MergePathPartitions<MgpuBoundsLower>(
		indices_global, aCount, mgpu::counting_iterator<int>(0), bCount, NV, 0,
		mgpu::less<int>(), context);

	int numBlocks = MGPU_DIV_UP(aCount + bCount, NV);
	KernelBulkInsert<Tuning><<<numBlocks, launch.x, 0, context.Stream()>>>(
		a_global, indices_global, aCount, b_global, bCount, 
		partitionsDevice->get(), dest_global);
	MGPU_SYNC_CHECK("KernelBulkInsert");
}

} // namespace mgpu
