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
#include "../device/serialsets.cuh"
#include "../kernels/scan.cuh"
#include "../kernels/search.cuh"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// DeviceComputeSetAvailability
// Load set keys from global memory and run the local Balanced Path and serial
// set ops. Return the results and indices in register.

template<int NT, int VT, MgpuSetOp Op, bool Duplicates, typename InputIt1,
	typename InputIt2, typename T, typename Comp>
MGPU_DEVICE int DeviceComputeSetAvailability(InputIt1 a_global, int aCount,
	InputIt2 b_global, int bCount, const int* bp_global, Comp comp,
	int tid, int block, T* results, int* indices, int4& range, 
	bool& extended, T* keys_shared) {
		 
	const int NV = NT * VT;
	int gid = NV * block;
	int bp0 = bp_global[block];
	int bp1 = bp_global[block + 1];

	// Compute the intervals into the two source arrays.
	int a0 = 0x7fffffff & bp0;
	int a1 = 0x7fffffff & bp1;
	int b0 = gid - a0;
	int b1 = min(aCount + bCount, gid + NV) - a1;

	// If the most sig bit flag is set, we're dealing with a 'starred' diagonal
	// that shifts the point of intersection up.
	int bit0 = (0x80000000 & bp0) ? 1 : 0;
	int bit1 = (0x80000000 & bp1) ? 1 : 0;
	b0 += bit0;
	b1 += bit1;

	// Attempt to load an 'extended' frame by grabbing an extra value from each
	// array.
	int aCount2 = a1 - a0;
	int bCount2 = b1 - b0;
	extended = (a1 < aCount) && (b1 < bCount);
	int bStart = aCount2 + (int)extended;

	DeviceLoad2ToShared<NT, VT, VT + 1>(a_global + a0, aCount2 + (int)extended,
		b_global + b0, bCount2 + (int)extended, tid, keys_shared);
	int count = aCount2 + bCount2;
	
	// Run a Balanced Path search for each thread's starting point.
	int diag = min(VT * tid - bit0, count);
	int2 bp = BalancedPath<Duplicates, int>(keys_shared, aCount2, 
		keys_shared + bStart, bCount2, diag, 2, comp);

	int a0tid = bp.x;
	int b0tid = VT * tid + bp.y - bp.x - bit0;

	int commit;
	if(extended)
		commit = SerialSetOp<VT, false, Op>(keys_shared, a0tid, aCount2, 
			bStart + b0tid, bStart + bCount2, bp.y, results, indices, comp);
	else
		commit = SerialSetOp<VT, true, Op>(keys_shared, a0tid, aCount2, 
			bStart + b0tid, bStart + bCount2, bp.y, results, indices, comp);
	
	range = make_int4(a0, a1, b0, b1);
	return commit;
}

////////////////////////////////////////////////////////////////////////////////
// KernelSetOp

template<typename Tuning, MgpuSetOp Op, bool Duplicates, int Stage,
	bool HasValues, typename KeysIt1, typename KeysIt2, typename KeysIt3,
	typename ValsIt1, typename ValsIt2, typename ValsIt3, typename Comp>
MGPU_LAUNCH_BOUNDS void KernelSetOp(KeysIt1 aKeys_global, ValsIt1 aVals_global,
	int aCount, KeysIt2 bKeys_global, ValsIt2 bVals_global, int bCount,
	int* counts_global, const int* bp_global, KeysIt3 keys_global, 
	ValsIt3 values_global, Comp comp) {

	typedef typename std::iterator_traits<KeysIt1>::value_type KeyType;
	typedef typename std::iterator_traits<ValsIt1>::value_type ValType;
	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;

	typedef CTAReduce<NT> R;
	typedef CTAScan<NT> S;

	union Shared {
		KeyType keys[NT * (VT + 1)];
		int indices[NV];
		typename R::Storage reduceStorage;
		typename S::Storage scanStorage;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	
	// Run the set operation. Return a bitfield for the selected keys.
	KeyType results[VT];
	int indices[VT];
	int4 range;
	bool extended;
	int commit = DeviceComputeSetAvailability<NT, VT, Op, Duplicates>(
		aKeys_global, aCount, bKeys_global, bCount, bp_global, comp, tid, block,
		results, indices, range, extended, shared.keys);
	aCount = range.y - range.x;
	bCount = range.w - range.z;

	// scan or reduce over the number of emitted keys per thread.
	int outputCount = popc(commit);
	int outputTotal;
	if(0 == Stage) {
		// Stage 0 - count the outputs.
		outputTotal = R::Reduce(tid, outputCount, shared.reduceStorage);
	} else {
		int globalStart = (1 == Stage) ? counts_global[block] : (NV * block);

		// Stage 1 or 2 - stream the keys.
		int scan = S::Scan(tid, outputCount, shared.scanStorage, &outputTotal);

		// Write the commit results to shared memory.
		int start = scan;
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			if((1<< i) & commit)
				shared.keys[start++] = results[i];
		__syncthreads();

		// Store keys to global memory.
		DeviceSharedToGlobal<NT, VT>(outputTotal, shared.keys, tid, 
			keys_global + globalStart);

		if(HasValues) {
			// indices[] has gather indices in thread order. Compact and store
			// these to shared memory for a transpose to strided order.		
			start = scan;
			#pragma unroll
			for(int i = 0; i < VT; ++i)
				if((1<< i) & commit)
					shared.indices[start++] = indices[i];
			__syncthreads();

			aVals_global += range.x;
			bVals_global += range.z;
			values_global += globalStart;
			if(MgpuSetOpIntersection == Op || MgpuSetOpDiff == Op)
				DeviceGatherGlobalToGlobal<NT, VT>(outputTotal, aVals_global,
					shared.indices, tid, values_global, false);
			else
				DeviceTransferMergeValuesShared<NT, VT>(outputTotal,
					aVals_global, bVals_global, aCount + (int)extended,
					shared.indices, tid, values_global, false);
		}
	}

	if(1 != Stage && !tid)
		counts_global[block] = outputTotal;
}

////////////////////////////////////////////////////////////////////////////////
// KernelSetCompact

template<int NT, typename T, typename OutputIt>
__global__ void KernelSetCompact(const T* source_global, const int* scan_global,
	int numSegments, int blockSize, OutputIt dest_global) {

	const int NumWarps = NT / WARP_SIZE;
	int tid = threadIdx.x;
	int warp = tid / WARP_SIZE;
	int lane = (WARP_SIZE - 1) & tid;
	int block = blockIdx.x;

	int gid = block * NumWarps + warp;
	if(gid >= numSegments) return;

	int start = scan_global[gid];
	int end = scan_global[gid + 1];

	source_global += blockSize * gid;
	dest_global += start;
	  
	// Round the count up by 4 * WARP_SIZE and unroll to get four outstanding
	// loads. This makes the outer loop non-divergent.
	int count = end - start;
	for(int i = 0; i < count; i += 4 * WARP_SIZE) {
		int count2 = min(4 * WARP_SIZE, count - i);
		T values[4];
		DeviceGlobalToReg<WARP_SIZE, 4>(count2, source_global + i, lane, 
			values);
		DeviceRegToGlobal<WARP_SIZE, 4>(count2, values, lane, 
			dest_global + i);
	}
}

////////////////////////////////////////////////////////////////////////////////
// SetOpKeys

template<MgpuSetOp Op, bool Duplicates, typename It1, typename It2,
	typename T, typename Comp>
MGPU_HOST int SetOpKeys(It1 a_global, int aCount, It2 b_global, int bCount,
	MGPU_MEM(T)* ppKeys_global, Comp comp, CudaContext& context, bool compact) {

	typedef LaunchBoxVT<
		128, 23, 0,
		128, 11, 0,
		128, 11, 0
	> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;
	int numBlocks = MGPU_DIV_UP(aCount + bCount, NV);

	// BalancedPath search to establish partitions.
	MGPU_MEM(int) partitionsDevice = FindSetPartitions<Duplicates>(a_global,
		aCount, b_global, bCount, NV, comp, context);

	MGPU_MEM(int) countsDevice = context.Malloc<int>(numBlocks + 1);
	MGPU_MEM(T) keysDevice;
	int total;
	if(compact) {
		// Allocate enough temporary space for all outputs.
		MGPU_MEM(T) keysTempDevice = context.Malloc<T>(NV * numBlocks);

		KernelSetOp<Tuning, Op, Duplicates, 2, false>
			<<<numBlocks, launch.x, 0, context.Stream()>>>(a_global, 
			(const int*)0, aCount, b_global, (const int*)0, bCount,
			countsDevice->get(), partitionsDevice->get(), keysTempDevice->get(),
			(int*)0, comp);
		MGPU_SYNC_CHECK("KernelSetOp");

		// Scan block counts.
		Scan<MgpuScanTypeExc>(countsDevice->get(), numBlocks, 0, 
			mgpu::plus<int>(), countsDevice->get() + numBlocks, &total, 
			countsDevice->get(), context);
		
		// Compact keys into destination.
		keysDevice = context.Malloc<T>(total);

		const int NT2 = 256;
		int numCompactBlocks = MGPU_DIV_UP(numBlocks, NT2 / WARP_SIZE);
		KernelSetCompact<256><<<numCompactBlocks, NT2, 0, context.Stream()>>>(
			keysTempDevice->get(), countsDevice->get(), numBlocks, NV,
			keysDevice->get());
		MGPU_SYNC_CHECK("KernelSetCompact");

	} else {
		KernelSetOp<Tuning, Op, Duplicates, 0, false>
			<<<numBlocks, launch.x, 0, context.Stream()>>>(a_global, 
			(const int*)0, aCount, b_global, (const int*)0, bCount, 
			countsDevice->get(), partitionsDevice->get(), (T*)0, (int*)0, comp);
		MGPU_SYNC_CHECK("KernelSetOp");

		// Scan block counts.
		ScanExc(countsDevice->get(), numBlocks, &total, context);

		// Allocate storage for the keys. Run the set operations again, but
		// this time stream the outputs.
		keysDevice = context.Malloc<T>(total);
		KernelSetOp<Tuning, Op, Duplicates, 1, false>
			<<<numBlocks, launch.x, 0, context.Stream()>>>(a_global, (int*)0, 
			aCount, b_global, (int*)0, bCount, countsDevice->get(), 
			partitionsDevice->get(), keysDevice->get(), (int*)0, comp);
		MGPU_SYNC_CHECK("KernelSetOp");
	}
	*ppKeys_global = keysDevice;
	return total;
}
template<MgpuSetOp Op, bool Duplicates, typename It1, typename It2, typename T>
MGPU_HOST int SetOpKeys(It1 a_global, int aCount, It2 b_global, int bCount,
	MGPU_MEM(T)* ppKeys_global, CudaContext& context, bool compact) {

	typedef mgpu::less<typename std::iterator_traits<It1>::value_type> Comp;
		return SetOpKeys<Op, Duplicates>(a_global, aCount, b_global, bCount, 
		ppKeys_global, Comp(), context, compact);
}

////////////////////////////////////////////////////////////////////////////////
// SetOpPairs

template<MgpuSetOp Op, bool Duplicates, typename KeysIt1, typename KeysIt2,
	typename ValsIt1, typename ValsIt2, typename KeyType, typename ValType,
	typename Comp>
MGPU_HOST int SetOpPairs(KeysIt1 aKeys_global, ValsIt1 aVals_global, int aCount,
	KeysIt2 bKeys_global, ValsIt2 bVals_global, int bCount,
	MGPU_MEM(KeyType)* ppKeys_global, MGPU_MEM(ValType)* ppVals_global, 
	Comp comp, CudaContext& context) {

	const int NT = 128;
	const int VT = 7;
	typedef LaunchBoxVT<NT, VT> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;
	int numBlocks = MGPU_DIV_UP(aCount + bCount, NV);

	// BalancedPath search to establish partitions.
	MGPU_MEM(int) partitionsDevice = FindSetPartitions<Duplicates>(aKeys_global,
		aCount, bKeys_global, bCount, NV, comp, context);

	// Run the kernel once to count outputs per block.
	MGPU_MEM(int) countsDevice = context.Malloc<int>(numBlocks + 1);
	KernelSetOp<Tuning, Op, Duplicates, 0, false>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(aKeys_global,
		(const int*)0, aCount, bKeys_global, (const int*)0, bCount, 
		countsDevice->get(), partitionsDevice->get(), (KeyType*)0, (int*)0,
		comp);
	MGPU_SYNC_CHECK("KernelSetOp");

	// Scan outputs and allocate output arrays.
	int total;
	ScanExc(countsDevice->get(), numBlocks, &total, context);
	MGPU_MEM(KeyType) keysDevice = context.Malloc<KeyType>(total);
	MGPU_MEM(ValType) valsDevice = context.Malloc<ValType>(total);

	// Recompute and stream the outputs.
	KernelSetOp<Tuning, Op, Duplicates, 1, true>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(aKeys_global, 
		aVals_global, aCount, bKeys_global, bVals_global, bCount,
		countsDevice->get(), partitionsDevice->get(), keysDevice->get(),
		valsDevice->get(), comp);
	MGPU_SYNC_CHECK("KernelSetOp");

	*ppKeys_global = keysDevice;
	*ppVals_global = valsDevice;
	return total;
}
template<MgpuSetOp Op, bool Duplicates, typename KeysIt1, typename KeysIt2,
	typename ValsIt1, typename ValsIt2, typename KeyType, typename ValType>
MGPU_HOST int SetOpPairs(KeysIt1 aKeys_global, ValsIt1 aVals_global, int aCount,
	KeysIt2 bKeys_global, ValsIt2 bVals_global, int bCount,
	MGPU_MEM(KeyType)* ppKeys_global, MGPU_MEM(ValType)* ppVals_global, 
	CudaContext& context) {
 
	typedef mgpu::less<typename std::iterator_traits<KeysIt1>::value_type> Comp;
	return SetOpPairs<Op, Duplicates>(aKeys_global, aVals_global, aCount,
		bKeys_global, bVals_global, bCount, ppKeys_global, ppVals_global,
		Comp(), context);
}

} // namespace mgpu
