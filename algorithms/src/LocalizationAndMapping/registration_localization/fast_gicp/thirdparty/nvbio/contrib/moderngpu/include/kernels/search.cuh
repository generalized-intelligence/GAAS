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
#include "../device/ctamerge.cuh"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// BinarySearchPartitions

template<int NT, MgpuBounds Bounds, typename It, typename Comp>
__global__ void KernelBinarySearch(int count, It data_global, int numItems,
	int nv, int* partitions_global, int numSearches, Comp comp) {

	int gid = NT * blockIdx.x + threadIdx.x;
	if(gid < numSearches) {
		int p = BinarySearch<Bounds>(data_global, numItems, 
			min(nv * gid, count), comp);
		partitions_global[gid] = p;
	}
}

template<MgpuBounds Bounds, typename It1, typename Comp>
MGPU_MEM(int) BinarySearchPartitions(int count, It1 data_global, int numItems,
	int nv, Comp comp, CudaContext& context) {

	const int NT = 64;
	int numBlocks = MGPU_DIV_UP(count, nv);
	int numPartitionBlocks = MGPU_DIV_UP(numBlocks + 1, NT);
	MGPU_MEM(int) partitionsDevice = context.Malloc<int>(numBlocks + 1);

	KernelBinarySearch<NT, Bounds>
		<<<numPartitionBlocks, NT, 0, context.Stream()>>>(count, data_global, 
		numItems, nv, partitionsDevice->get(), numBlocks + 1, comp);
	MGPU_SYNC_CHECK("KernelBinarySearch");

	return partitionsDevice;
}

////////////////////////////////////////////////////////////////////////////////
// MergePathPartitions

template<int NT, MgpuBounds Bounds, typename It1, typename It2, typename Comp>
__global__ void KernelMergePartition(It1 a_global, int aCount, It2 b_global, 
	int bCount, int nv, int coop, int* mp_global, int numSearches, Comp comp) {

	int partition = NT * blockIdx.x + threadIdx.x;
	if(partition < numSearches) {
		int a0 = 0, b0 = 0;
		int gid = nv * partition;
		if(coop) {
			int3 frame = FindMergesortFrame(coop, partition, nv);
			a0 = frame.x;
			b0 = min(aCount, frame.y);
			bCount = min(aCount, frame.y + frame.z) - b0;
			aCount = min(aCount, frame.x + frame.z) - a0;

			// Put the cross-diagonal into the coordinate system of the input
			// lists.
			gid -= a0;
		}
		int mp = MergePath<Bounds>(a_global + a0, aCount, b_global + b0, bCount,
			min(gid, aCount + bCount), comp);
		mp_global[partition] = mp;
	}
}

template<MgpuBounds Bounds, typename It1, typename It2, typename Comp>
MGPU_MEM(int) MergePathPartitions(It1 a_global, int aCount, It2 b_global,
	int bCount, int nv, int coop, Comp comp, CudaContext& context) {

	const int NT = 64;
	int numPartitions = MGPU_DIV_UP(aCount + bCount, nv);
	int numPartitionBlocks = MGPU_DIV_UP(numPartitions + 1, NT);
	MGPU_MEM(int) partitionsDevice = context.Malloc<int>(numPartitions + 1);

	KernelMergePartition<NT, Bounds>
		<<<numPartitionBlocks, NT, 0, context.Stream()>>>(a_global, aCount,
		b_global, bCount, nv, coop, partitionsDevice->get(), numPartitions + 1, 
		comp);
	MGPU_SYNC_CHECK("KernelMergePartition");

	return partitionsDevice;
}

////////////////////////////////////////////////////////////////////////////////
// FindSetPartitions

template<int NT, bool Duplicates, typename InputIt1, typename InputIt2,
	typename Comp>
__global__ void KernelSetPartition(InputIt1 a_global, int aCount, 
	InputIt2 b_global, int bCount, int nv, int* bp_global, int numSearches,
	Comp comp) {

	int gid = NT * blockIdx.x + threadIdx.x;
	if(gid < numSearches) {
		int diag = min(aCount + bCount, gid * nv);

		// Search with level 4 bias. This helps the binary search converge 
		// quickly for small runs of duplicates (the common case).
		int2 bp = BalancedPath<Duplicates, int64>(a_global, aCount, b_global,
			bCount, diag, 4, comp);

		if(bp.y) bp.x |= 0x80000000;
		bp_global[gid] = bp.x;
	}
}

template<bool Duplicates, typename It1, typename It2, typename Comp>
MGPU_MEM(int) FindSetPartitions(It1 a_global, int aCount, It2 b_global,
	int bCount, int nv, Comp comp, CudaContext& context) {

	const int NT = 64;
	int numPartitions = MGPU_DIV_UP(aCount + bCount, nv);
	int numPartitionBlocks = MGPU_DIV_UP(numPartitions + 1, NT);
	MGPU_MEM(int) partitionsDevice = context.Malloc<int>(numPartitions + 1);

	KernelSetPartition<NT, Duplicates>
		<<<numPartitionBlocks, NT, 0, context.Stream()>>>(a_global, aCount,
		b_global, bCount, nv, partitionsDevice->get(), numPartitions + 1, comp);
	MGPU_SYNC_CHECK("KernelSetPartition");

	return partitionsDevice;
}

} // namespace mgpu
