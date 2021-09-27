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
#include "../device/launchbox.cuh"
#include "../device/loadstore.cuh"
#include "../device/ctaloadbalance.cuh"
#include "../kernels/search.cuh"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// KernelIntervalExpand

template<typename Tuning, typename IndicesIt, typename ValuesIt,
	typename OutputIt>
MGPU_LAUNCH_BOUNDS void KernelIntervalExpand(int destCount, 
	IndicesIt indices_global, ValuesIt values_global, int sourceCount, 
	const int* mp_global, OutputIt output_global) {

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	typedef typename std::iterator_traits<ValuesIt>::value_type T;

	union Shared {
		int indices[NT * (VT + 1)];
		T values[NT * VT];
	};
	__shared__ Shared shared;
	int tid = threadIdx.x;
	int block = blockIdx.x;

	// Compute the input and output intervals this CTA processes.
	int4 range = CTALoadBalance<NT, VT>(destCount, indices_global, sourceCount,
		block, tid, mp_global, shared.indices, true);
	
	// The interval indices are in the left part of shared memory (moveCount).
	// The scan of interval counts are in the right part (intervalCount).
	destCount = range.y - range.x;
	sourceCount = range.w - range.z;

	// Copy the source indices into register.
	int sources[VT];
	DeviceSharedToReg<NT, VT>(shared.indices, tid, sources);

	// Load the source fill values into shared memory. Each value is fetched
	// only once to reduce latency and L2 traffic.
	DeviceMemToMemLoop<NT>(sourceCount, values_global + range.z, tid,
		shared.values);
	
	// Gather the values from shared memory into register. This uses a shared
	// memory broadcast - one instance of a value serves all the threads that
	// comprise its fill operation.
	T values[VT];
	DeviceGather<NT, VT>(destCount, shared.values - range.z, sources, tid,
		values, false);

	// Store the values to global memory.
	DeviceRegToGlobal<NT, VT>(destCount, values, tid, output_global + range.x);
}

////////////////////////////////////////////////////////////////////////////////
// IntervalExpand

template<typename IndicesIt, typename ValuesIt, typename OutputIt>
MGPU_HOST void IntervalExpand(int moveCount, IndicesIt indices_global, 
	ValuesIt values_global, int intervalCount, OutputIt output_global,
	CudaContext& context) {

	const int NT = 128;
	const int VT = 7;
	typedef LaunchBoxVT<NT, VT> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);

	int NV = launch.x * launch.y;
	int numBlocks = MGPU_DIV_UP(moveCount + intervalCount, NV);

	// Partition the input and output sequences so that the load-balancing
	// search results in a CTA fit in shared memory.
	MGPU_MEM(int) partitionsDevice = MergePathPartitions<MgpuBoundsUpper>(
		mgpu::counting_iterator<int>(0), moveCount, indices_global, 
		intervalCount, NV, 0, mgpu::less<int>(), context);

	KernelIntervalExpand<Tuning><<<numBlocks, launch.x, 0, context.Stream()>>>(
		moveCount, indices_global, values_global, intervalCount,
		partitionsDevice->get(), output_global);
	MGPU_SYNC_CHECK("KernelIntervalExpand");
}

////////////////////////////////////////////////////////////////////////////////
// KernelIntervalMove

template<typename Tuning, bool Gather, bool Scatter, typename GatherIt,
	typename ScatterIt, typename IndicesIt, typename InputIt, typename OutputIt>
MGPU_LAUNCH_BOUNDS void KernelIntervalMove(int moveCount,
	GatherIt gather_global, ScatterIt scatter_global, IndicesIt indices_global, 
	int intervalCount, InputIt input_global, const int* mp_global, 
	OutputIt output_global) {
	
	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;

	__shared__ int indices_shared[NT * (VT + 1)];
	int tid = threadIdx.x;
	int block = blockIdx.x;
	
	// Load balance the move IDs (counting_iterator) over the scan of the
	// interval sizes.
	int4 range = CTALoadBalance<NT, VT>(moveCount, indices_global, 
		intervalCount, block, tid, mp_global, indices_shared, true);

	// The interval indices are in the left part of shared memory (moveCount).
	// The scan of interval counts are in the right part (intervalCount).
	moveCount = range.y - range.x;
	intervalCount = range.w - range.z;
	int* move_shared = indices_shared;
	int* intervals_shared = indices_shared + moveCount;
	int* intervals_shared2 = intervals_shared - range.z;

	// Read out the interval indices and scan offsets.
	int interval[VT], rank[VT];
	#pragma unroll
	for(int i = 0; i < VT; ++i) {
		int index = NT * i + tid;
		int gid = range.x + index;
		interval[i] = range.z;
		if(index < moveCount) {
			interval[i] = move_shared[index];
			rank[i] = gid - intervals_shared2[interval[i]];
		}
	}
	__syncthreads();
	
	// Load and distribute the gather and scatter indices.
	int gather[VT], scatter[VT];
	if(Gather) {
		// Load the gather pointers into intervals_shared.
		DeviceMemToMemLoop<NT>(intervalCount, gather_global + range.z, tid,
			intervals_shared);

		// Make a second pass through shared memory. Grab the start indices of
		// the interval for each item and add the scan into it for the gather
		// index.
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			gather[i] = intervals_shared2[interval[i]] + rank[i];
		__syncthreads();
	} 
	if(Scatter) {
		// Load the scatter pointers into intervals_shared.
		DeviceMemToMemLoop<NT>(intervalCount, scatter_global + range.z, tid,
			intervals_shared);

		// Make a second pass through shared memory. Grab the start indices of
		// the interval for each item and add the scan into it for the scatter
		// index.
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			scatter[i] = intervals_shared2[interval[i]] + rank[i];
		__syncthreads();
	}

	// Gather the data into register.
	typedef typename std::iterator_traits<InputIt>::value_type T;
	T data[VT];
	if(Gather)
		DeviceGather<NT, VT>(moveCount, input_global, gather, tid, data, false);
	else
		DeviceGlobalToReg<NT, VT>(moveCount, input_global + range.x, tid, data);

	// Scatter the data into global.
	if(Scatter)
		DeviceScatter<NT, VT>(moveCount, data, tid, scatter, output_global,
			false);
	else
		DeviceRegToGlobal<NT, VT>(moveCount, data, tid, 
			output_global + range.x);	
}

////////////////////////////////////////////////////////////////////////////////
// IntervalGather

template<typename GatherIt, typename IndicesIt, typename InputIt,
	typename OutputIt>
MGPU_HOST void IntervalGather(int moveCount, GatherIt gather_global, 
	IndicesIt indices_global, int intervalCount, InputIt input_global,
	OutputIt output_global, CudaContext& context) {

	const int NT = 128;
	const int VT = 7;
	typedef LaunchBoxVT<NT, VT> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);

	int NV = launch.x * launch.y;
	int numBlocks = MGPU_DIV_UP(moveCount + intervalCount, NV);
	
	MGPU_MEM(int) partitionsDevice = MergePathPartitions<MgpuBoundsUpper>(
		mgpu::counting_iterator<int>(0), moveCount, indices_global,
		intervalCount, NV, 0, mgpu::less<int>(), context);

	KernelIntervalMove<Tuning, true, false>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(moveCount, gather_global,
		(const int*)0, indices_global, intervalCount, input_global,
		partitionsDevice->get(), output_global);
	MGPU_SYNC_CHECK("KernelIntervalMove");
}

////////////////////////////////////////////////////////////////////////////////
// IntervalScatter

template<typename ScatterIt, typename IndicesIt, typename InputIt,
	typename OutputIt>
MGPU_HOST void IntervalScatter(int moveCount, ScatterIt scatter_global,
	IndicesIt indices_global, int intervalCount, InputIt input_global,
	OutputIt output_global, CudaContext& context) {

	const int NT = 128;
	const int VT = 7;
	typedef LaunchBoxVT<NT, VT> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);

	int NV = launch.x * launch.y;
	int numBlocks = MGPU_DIV_UP(moveCount + intervalCount, NV);
	
	MGPU_MEM(int) partitionsDevice = MergePathPartitions<MgpuBoundsUpper>(
		mgpu::counting_iterator<int>(0), moveCount, indices_global, 
		intervalCount, NV, 0, mgpu::less<int>(), context);

	KernelIntervalMove<Tuning, false, true>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(moveCount, (const int*)0,
		scatter_global, indices_global, intervalCount, input_global, 
		partitionsDevice->get(), output_global);
	MGPU_SYNC_CHECK("KernelIntervalMove");
}

////////////////////////////////////////////////////////////////////////////////
// IntervalMove

template<typename GatherIt, typename ScatterIt, typename IndicesIt, 
	typename InputIt, typename OutputIt>
MGPU_HOST void IntervalMove(int moveCount, GatherIt gather_global, 
	ScatterIt scatter_global, IndicesIt indices_global, int intervalCount, 
	InputIt input_global, OutputIt output_global, CudaContext& context) {

	const int NT = 128;
	const int VT = 7;
	typedef LaunchBoxVT<NT, VT> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);

	int NV = launch.x * launch.y;
	int numBlocks = MGPU_DIV_UP(moveCount + intervalCount, NV);
	
	MGPU_MEM(int) partitionsDevice = MergePathPartitions<MgpuBoundsUpper>(
		mgpu::counting_iterator<int>(0), moveCount, indices_global, 
		intervalCount, NV, 0, mgpu::less<int>(), context);

	KernelIntervalMove<Tuning, true, true>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(moveCount, gather_global, 
		scatter_global, indices_global, intervalCount, input_global,
		partitionsDevice->get(), output_global);
	MGPU_SYNC_CHECK("KernelIntervalMove");
}

} // namespace mgpu
