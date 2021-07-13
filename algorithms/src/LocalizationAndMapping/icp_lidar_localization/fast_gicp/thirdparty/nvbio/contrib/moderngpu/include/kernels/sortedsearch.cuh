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
#include "../device/ctasortedsearch.cuh"
#include "../kernels/search.cuh"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// DeviceLoadSortedSearch
// Loads data from global memory and performs a sorted search. Leaves the
// indices in shared memory.

template<int NT, int VT, MgpuBounds Bounds, bool IndexA, bool MatchA, 
	bool IndexB, bool MatchB, typename InputIt1, typename InputIt2, typename T,
	typename Comp>
MGPU_DEVICE int2 DeviceLoadSortedSearch(int4 range, InputIt1 a_global, 
	int aCount, InputIt2 b_global, int bCount, int tid, int block,
	T* keys_shared, int* indices_shared, Comp comp) {

	int a0 = range.x;
	int a1 = range.y;
	int b0 = range.z;
	int b1 = range.w;
	int aCount2 = a1 - a0;
	int bCount2 = b1 - b0;

	// For matching:
	// If UpperBound
	//		MatchA requires preceding B
	//		MatchB requires trailing A
	// If LowerBound
	//		MatchA requires trailing B
	//		MatchB requires preceding A
	int leftA = MatchB && (MgpuBoundsLower == Bounds) && (a0 > 0);
	int leftB = MatchA && (MgpuBoundsUpper == Bounds) && (b0 > 0);
	int rightA = a1 < aCount;
	int rightB = b1 < bCount;

	int aStart = leftA;
	int aEnd = aStart + aCount2 + rightA;
	int bStart = aEnd + leftB;
	int bEnd = bStart + bCount2 + rightB;

	// Cooperatively load all the data including halos.
	DeviceLoad2ToShared<NT, VT, VT + 1>(a_global + a0 - leftA, aEnd, 
		b_global + b0 - leftB, bEnd - aEnd, tid, keys_shared);

	// Run the serial searches and compact the indices into shared memory.
	bool extended = rightA && rightB && (!MatchA || leftB) &&
		(!MatchB || leftA);
	int2 matchCount = CTASortedSearch<NT, VT, Bounds, IndexA, MatchA, IndexB,
		MatchB>(keys_shared, aStart, aCount2, aEnd, a0, bStart, bCount2, bEnd,
		b0, extended, tid, indices_shared, comp);

	return matchCount;
}

////////////////////////////////////////////////////////////////////////////////
// KernelSortedSearch

// If Indices is true, return the array of lower_bound or upper_bound of needles
// A and haystack B.

// If MatchA is true, set the most significant bit (0x80000000) of each index

// If there is at least one element in the sequence B that matches the element
// in A.

// If MatchB is 1, return a true or false flag if there is at least one element
// in A that matches the element in B.

// If MatchB is 2, return the upper_bound of B into A. Set the most significant
// bit of each match index if there is at least one element in A that matches
// the element in B. This element will immediately precede the returned index.

template<typename Tuning, MgpuBounds Bounds, bool IndexA, bool MatchA,
	bool IndexB, bool MatchB, typename InputIt1, typename InputIt2, 
	typename OutputIt1, typename OutputIt2, typename Comp>
MGPU_LAUNCH_BOUNDS void KernelSortedSearch(InputIt1 a_global, int aCount,
	InputIt2 b_global, int bCount, const int* mp_global, 
	OutputIt1 aIndices_global, OutputIt2 bIndices_global, int* aMatchCount, 
	int* bMatchCount, Comp comp) {

	typedef typename std::iterator_traits<InputIt1>::value_type T;
	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;

	typedef CTAReduce<NT> R;
	union Shared {
		T keys[NT * (VT + 1)];
		int indices[NV];
		typename R::Storage reduce;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int4 range = ComputeMergeRange(aCount, bCount, block, 0, NV, mp_global);

	int2 matchCount = DeviceLoadSortedSearch<NT, VT, Bounds, IndexA, MatchA, 
		IndexB, MatchB>(range, a_global, aCount, b_global, bCount, tid, block,
		shared.keys, shared.indices, comp);
	aCount = range.y - range.x;
	bCount = range.w - range.z;

	// Store A indices to global memory.
	if(IndexA || MatchA)
		DeviceMemToMemLoop<NT>(aCount, shared.indices, tid, 
			aIndices_global + range.x);
	
	// Store B match flags to global memory.
	if(IndexB || MatchB)
		DeviceMemToMemLoop<NT>(bCount, shared.indices + aCount, tid, 
			bIndices_global + range.z);
			
	// Set the atomic counters for A matches and B matches.
	if((MatchA || MatchB) && (aMatchCount || bMatchCount)) {
		int x = bfi(matchCount.y, matchCount.x, 16, 16);
		int total = R::Reduce(tid, x, shared.reduce);
		if(!tid && aMatchCount) atomicAdd(aMatchCount, 0xffff & total);
		if(!tid && bMatchCount) atomicAdd(bMatchCount, total>> 16);
	}
}

////////////////////////////////////////////////////////////////////////////////
// SortedSearch

template<MgpuBounds Bounds, MgpuSearchType TypeA, MgpuSearchType TypeB,
	typename InputIt1, typename InputIt2, typename OutputIt1, 
	typename OutputIt2, typename Comp>
MGPU_HOST void SortedSearch(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, OutputIt1 aIndices_global, OutputIt2 bIndices_global,
	Comp comp, CudaContext& context, int* aMatchCount, int* bMatchCount) {

	MGPU_VAR_UNUSED const bool IndexA = MgpuSearchTypeIndex == TypeA || 
		MgpuSearchTypeIndexMatch == TypeA;
	MGPU_VAR_UNUSED const bool MatchA = MgpuSearchTypeMatch == TypeA ||
		MgpuSearchTypeIndexMatch == TypeA;
	MGPU_VAR_UNUSED const bool IndexB = MgpuSearchTypeIndex == TypeB ||
		MgpuSearchTypeIndexMatch == TypeB;
	MGPU_VAR_UNUSED const bool MatchB = MgpuSearchTypeMatch == TypeB ||
		MgpuSearchTypeIndexMatch == TypeB;

	typedef typename std::iterator_traits<InputIt1>::value_type T;
	typedef LaunchBoxVT<
		128, 11, 0,
		128, 7, 0,
		128, 7, 0
	> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;

	int numBlocks = MGPU_DIV_UP(aCount + bCount, NV);

	// Run a global merge path for each CTA to find partitions.
	MGPU_MEM(int) partitionsDevice = MergePathPartitions<Bounds>(a_global,
		aCount, b_global, bCount, NV, 0, comp, context);

	// Allocate counters if we want match totals returned.
	MGPU_MEM(int) counters;
	int* aMatchDevice = 0, *bMatchDevice = 0;
	if(aMatchCount || bMatchCount) {
		counters = context.Malloc<int>(2);
		cudaMemsetAsync(counters->get(), 0, 2 * sizeof(int), context.Stream());
		aMatchDevice = counters->get();
		bMatchDevice = aMatchDevice + 1;
	}

	// Launch the kernel.
	KernelSortedSearch<Tuning, Bounds, IndexA, MatchA, IndexB, MatchB>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(a_global, aCount, 
		b_global, bCount, partitionsDevice->get(), aIndices_global,
		bIndices_global, aMatchDevice, bMatchDevice, comp);
	MGPU_SYNC_CHECK("KernelSortedSearch");
	
	// Copy counters back to host memory.
	if((MatchA && aMatchCount) || (MatchB && bMatchCount)) {
		int2 host;
		cudaMemcpy(&host, counters->get(), sizeof(int2), 
			cudaMemcpyDeviceToHost);
		if(aMatchCount) *aMatchCount = host.x;
		if(bMatchCount) *bMatchCount = host.y;
	}
}
template<MgpuBounds Bounds, MgpuSearchType TypeA, MgpuSearchType TypeB,
	typename InputIt1, typename InputIt2, typename OutputIt1, 
	typename OutputIt2>
MGPU_HOST void SortedSearch(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, OutputIt1 aIndices_global, OutputIt2 bIndices_global,
	CudaContext& context, int* aMatchCount, int* bMatchCount) {

	typedef mgpu::less<typename std::iterator_traits<InputIt1>::value_type> C;
	SortedSearch<Bounds, TypeA, TypeB>(a_global, aCount, b_global, bCount,
		aIndices_global, bIndices_global, C(), context, aMatchCount,
		bMatchCount);
}

template<MgpuBounds Bounds, typename InputIt1, typename InputIt2,
	typename OutputIt, typename Comp>
MGPU_HOST void SortedSearch(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, OutputIt aIndices_global, Comp comp, CudaContext& context) {

	SortedSearch<Bounds, MgpuSearchTypeIndex, MgpuSearchTypeNone>(a_global,
		aCount, b_global, bCount, aIndices_global, (int*)0, comp, context);
}
template<MgpuBounds Bounds, typename InputIt1, typename InputIt2,
	typename OutputIt>
MGPU_HOST void SortedSearch(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, OutputIt aIndices_global, CudaContext& context) {

	typedef mgpu::less<typename std::iterator_traits<InputIt1>::value_type> C;
	SortedSearch<Bounds>(a_global, aCount, b_global, bCount, aIndices_global,
		C(), context);
}

////////////////////////////////////////////////////////////////////////////////
// SortedEqualityCount
// Provide arrays A, B, and lower_bound(A, B). This function returns the number
// of elements in B that match each element in A. This is a building block for
// relational joins.

struct SortedEqualityOp {
	MGPU_HOST_DEVICE int operator()(int lb, int ub) const {
		return ub - lb;
	}
};

template<typename Tuning, typename InputIt1, typename InputIt2, 
	typename InputIt3, typename OutputIt, typename Comp, typename Op>
MGPU_LAUNCH_BOUNDS void KernelSortedEqualityCount(InputIt1 a_global, int aCount,
	InputIt2 b_global, int bCount, const int* mp_global, InputIt3 lb_global,
	OutputIt counts_global, Comp comp, Op op) {

	typedef typename std::iterator_traits<InputIt1>::value_type T;
	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;

	union Shared {
		T keys[NT * (VT + 1)];
		int indices[NV];
	};
	__shared__ Shared shared;
	
	int tid = threadIdx.x;
	int block = blockIdx.x;
	int4 range = ComputeMergeRange(aCount, bCount, block, 0, NV, mp_global);

	// Compute the upper bound.
	int2 matchCount = DeviceLoadSortedSearch<NT, VT, MgpuBoundsUpper, true,
		false, false, false>(range, a_global, aCount, b_global, bCount, tid, 
		block, shared.keys, shared.indices, comp);
	int aCount2 = range.y - range.x;

	// Load the lower bounds computed by the previous launch.
	int lb[VT];
	DeviceGlobalToReg<NT, VT>(aCount2, lb_global + range.x, tid, lb);

	// Subtract the lower bound from the upper bound and store the count.
	counts_global += range.x;
	#pragma unroll
	for(int i = 0; i < VT; ++i) {
		int index = NT * i + tid;
		if(index < aCount2) {
			int count = op(lb[i], shared.indices[index]);
			counts_global[index] = count;
		}
	}
}

template<typename InputIt1, typename InputIt2, typename InputIt3,
	typename OutputIt, typename Comp, typename Op>
MGPU_HOST void SortedEqualityCount(InputIt1 a_global, int aCount,
	InputIt2 b_global, int bCount, InputIt3 lb_global, OutputIt counts_global, 
	Comp comp, Op op, CudaContext& context) {

	const int NT = 128;
	const int VT = 11;
	typedef LaunchBoxVT<NT, VT> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;

	MGPU_MEM(int) partitionsDevice = MergePathPartitions<MgpuBoundsUpper>(
		a_global, aCount, b_global, bCount, NV, 0, comp, context);

	int numBlocks = MGPU_DIV_UP(aCount + bCount, NV);
	KernelSortedEqualityCount<Tuning>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(a_global, aCount, 
		b_global, bCount, partitionsDevice->get(), lb_global, counts_global,
		comp, op);
	MGPU_SYNC_CHECK("KernelSortedEqualityCount");
}
template<typename InputIt1, typename InputIt2, typename InputIt3,
	typename OutputIt, typename Op>
MGPU_HOST void SortedEqualityCount(InputIt1 a_global, int aCount,
	InputIt2 b_global, int bCount, InputIt3 lb_global, OutputIt counts_global, 
	Op op, CudaContext& context) {

	typedef mgpu::less<typename std::iterator_traits<InputIt1>::value_type> C;
	SortedEqualicyCount(a_global, aCount, b_global, bCount, lb_global, 
		counts_global, C(), op, context);
}

} // namespace mgpu
