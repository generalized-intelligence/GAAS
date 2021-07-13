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

#include "../mgpuhost.cuh"
#include "../device/ctaloadbalance.cuh"
#include "sortedsearch.cuh"
#include "scan.cuh"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// KernelLeftJoin

template<typename Tuning, bool LeftJoin>
MGPU_LAUNCH_BOUNDS void KernelLeftJoin(int total, const int* aLowerBound_global,
	const int* aCountsScan_global, int aCount, const int* mp_global,
	int* aIndices_global, int* bIndices_global) { 

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;

	__shared__ int indices_shared[NT * (VT + 1)];
	int tid = threadIdx.x;
	int block = blockIdx.x;

	int4 range = CTALoadBalance<NT, VT>(total, aCountsScan_global, aCount,
		block, tid, mp_global, indices_shared, true);
	int outputCount = range.y - range.x;
	int inputCount = range.w - range.z;
	int* output_shared = indices_shared;
	int* input_shared = indices_shared + outputCount;

	int aIndex[VT], rank[VT];
	#pragma unroll
	for(int i = 0; i < VT; ++i) {
		int index = NT * i + tid;
		if(index < outputCount) {
			int gid = range.x + index;
			aIndex[i] = output_shared[index];
			rank[i] = gid - input_shared[aIndex[i] - range.z];
			aIndices_global[gid] = aIndex[i];
		}
	}
	__syncthreads();

	// Load the lower bound of A into B for each element of A.
	DeviceMemToMemLoop<NT>(inputCount, aLowerBound_global + range.z, tid,
		input_shared);

	// Store the lower bound of A into B back for every output.
	#pragma unroll
	for(int i = 0; i < VT; ++i) {
		int index = NT * i + tid;
		if(index < outputCount) {
			int gid = range.x + index;
			int lb = input_shared[aIndex[i] - range.z];
			int bIndex;
			if(LeftJoin)
				bIndex = (0x80000000 & lb) ? 
					((0x7fffffff & lb) + rank[i]) :
					-1;
			else
				bIndex = lb + rank[i];
			bIndices_global[gid] = bIndex;
		}
	}
}

struct LeftJoinEqualityOp {
	MGPU_HOST_DEVICE int operator()(int lb, int ub) const {
		lb &= 0x7fffffff;
		return max(1, ub - lb);
	}
};

////////////////////////////////////////////////////////////////////////////////
// Right-join compaction kernels.

template<int NT>
__global__ void KernelRightJoinUpsweep(const uint64* matches_global, int count,
	int* totals_global) {
		
	typedef CTAReduce<NT> R;
	__shared__ typename R::Storage reduce;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = 8 * NT * block;
	int count2 = min(8 * NT, count - gid);

	int x = 0;
	if(gid + 8 * tid < count) {
		uint64 packed = matches_global[NT * block + tid];
		count2 -= 8 * tid;
		if(count2 < 8) {
			// Clear the bits above count2
			uint64 mask = (1ll<< (8 * count2)) - 1;
			packed = (packed & mask) | (0x0101010101010101ll & ~mask);
		}
		uint2 pair = ulonglong_as_uint2(packed);
		x = 8 - popc(pair.x) - popc(pair.y);
	}
	
	int total = R::Reduce(tid, x, reduce);
	if(!tid) totals_global[block] = total;
}

template<int NT>
__global__ void KernelRightJoinDownsweep(const uint64* matches_global, 
	int count, const int* scan_global, int* rightJoin_global) {

	typedef CTAScan<NT> S;
	union Shared {
		int indices[8 * NT];
		typename S::Storage scan;
	};
	__shared__ Shared shared;
	
	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = 8 * NT * block;
	int count2 = min(8 * NT, count - gid);

	int start = scan_global[block];

	uint64 packed;
	int x = 0;
	if(gid + 8 * tid < count) {
		packed = matches_global[NT * block + tid];
		count2 -= 8 * tid;
		if(count2 < 8) {
			// Clear the bits above count2
			uint64 mask = (1ll<< (8 * count2)) - 1;
			packed = (packed & mask) | (0x0101010101010101ll & ~mask);
		}
		uint2 pair = ulonglong_as_uint2(packed);
		x = 8 - popc(pair.x) - popc(pair.y);

		packed ^= 0x0101010101010101ll;
	}

	int total;
	int scan = S::Scan(tid, x, shared.scan, &total);

	if(x) {
		#pragma unroll
		for(int i = 0; i < 8; ++i)
			if((1ll<< (8 * i)) & packed) 
				shared.indices[scan++] = gid + 8 * tid + i;
	}
	__syncthreads();

	DeviceMemToMemLoop<NT>(total, shared.indices, tid, 
		rightJoin_global + start, false);
}

////////////////////////////////////////////////////////////////////////////////
// RelationalJoin

template<MgpuJoinKind Kind, typename InputIt1, typename InputIt2,
	typename Comp>
MGPU_HOST int RelationalJoin(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, MGPU_MEM(int)* ppAJoinIndices, MGPU_MEM(int)* ppBJoinIndices, 
	Comp comp, CudaContext& context) {

	typedef typename std::iterator_traits<InputIt1>::value_type T;
	const bool SupportLeft = MgpuJoinKindLeft == Kind || 
		MgpuJoinKindOuter == Kind;
	const bool SupportRight = MgpuJoinKindRight == Kind ||
		MgpuJoinKindOuter == Kind;

	const MgpuSearchType LeftType = SupportLeft ? 
		MgpuSearchTypeIndexMatch : MgpuSearchTypeIndex;

	MGPU_MEM(int) aLowerBound = context.Malloc<int>(aCount);
	MGPU_MEM(byte) bMatches;

	// Find the lower bound of A into B. If we are right joining also return the
	// set of matches of B into A.
	int rightJoinTotal = 0;
	if(SupportRight) {
		// Support a right or outer join. Count the number of B elements that
		// have matches in A. These matched values are included in the inner
		// join part. The other values (bCount - bMatchCount) are copied to the
		// end for the right join part.
		bMatches = context.Malloc<byte>(bCount);
		int bMatchCount;
		SortedSearch<MgpuBoundsLower, LeftType, MgpuSearchTypeMatch>(a_global,
			aCount, b_global, bCount, aLowerBound->get(), bMatches->get(), comp,
			context, 0, &bMatchCount);
		rightJoinTotal = bCount - bMatchCount;
	} else 
		SortedSearch<MgpuBoundsLower, LeftType, MgpuSearchTypeNone>(a_global,
			aCount, b_global, bCount, aLowerBound->get(), (int*)0, comp,
			context, 0, 0);

	// Use the lower bounds to compute the counts for each element.
	MGPU_MEM(int) aCounts = context.Malloc<int>(aCount);
	if(SupportLeft) 
		SortedEqualityCount(a_global, aCount, b_global, bCount,
			aLowerBound->get(), aCounts->get(), comp, LeftJoinEqualityOp(), 
			context);
	else
		SortedEqualityCount(a_global, aCount, b_global, bCount,
			aLowerBound->get(), aCounts->get(), comp, SortedEqualityOp(), 
			context);

	// Scan the product counts. This is part of the load-balancing search.
	int leftJoinTotal;
	ScanExc(aCounts->get(), aCount, &leftJoinTotal, context);

	// Allocate space for the join indices from the sum of left and right join
	// sizes.
	int joinTotal = leftJoinTotal + rightJoinTotal;
	MGPU_MEM(int) aIndicesDevice = context.Malloc<int>(joinTotal);
	MGPU_MEM(int) bIndicesDevice = context.Malloc<int>(joinTotal);

	// Launch the inner/left join kernel. Run an upper-bounds partitioning 
	// to load-balance the data.
	typedef LaunchBoxVT<128, 7> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	int NV = launch.x * launch.y;
	
	MGPU_MEM(int) partitionsDevice = MergePathPartitions<MgpuBoundsUpper>(
		mgpu::counting_iterator<int>(0), leftJoinTotal, aCounts->get(),
		aCount, NV, 0, mgpu::less<int>(), context);

	int numBlocks = MGPU_DIV_UP(leftJoinTotal + aCount, NV);
	KernelLeftJoin<Tuning, SupportLeft>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(leftJoinTotal, 
		aLowerBound->get(), aCounts->get(), aCount, partitionsDevice->get(),
		aIndicesDevice->get(), bIndicesDevice->get());
	MGPU_SYNC_CHECK("KernelLeftJoin");

	// Launch the right join kernel. Compact the non-matches from B into A.
	if(SupportRight) {
		const int NT = 128;
		int numBlocks = MGPU_DIV_UP(bCount, 8 * NT);

		MGPU_MEM(int) totals = context.Malloc<int>(numBlocks);
		KernelRightJoinUpsweep<NT><<<numBlocks, NT>>>(
			(const uint64*)bMatches->get(), bCount, totals->get());
		MGPU_SYNC_CHECK("KernelRightJoinUpsweep");
		
		ScanExc(totals->get(), numBlocks, context);

		KernelRightJoinDownsweep<NT><<<numBlocks, NT>>>(
			(const uint64*)bMatches->get(), bCount, totals->get(), 
			bIndicesDevice->get() + leftJoinTotal);
		MGPU_SYNC_CHECK("KernelRightJoinDownsweep");

		cudaMemset(aIndicesDevice->get() + leftJoinTotal, -1, 
			sizeof(int) * rightJoinTotal);
	}

	*ppAJoinIndices = aIndicesDevice;
	*ppBJoinIndices = bIndicesDevice;
	return joinTotal;
}

template<MgpuJoinKind Kind, typename InputIt1, typename InputIt2>
MGPU_HOST int RelationalJoin(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, MGPU_MEM(int)* ppAJoinIndices, MGPU_MEM(int)* ppBJoinIndices, 
	CudaContext& context) {

	typedef typename std::iterator_traits<InputIt1>::value_type T;
	return RelationalJoin<Kind>(a_global, aCount, b_global, bCount,
		ppAJoinIndices, ppBJoinIndices, mgpu::less<T>(), context);
}

} // namespace mgpu
