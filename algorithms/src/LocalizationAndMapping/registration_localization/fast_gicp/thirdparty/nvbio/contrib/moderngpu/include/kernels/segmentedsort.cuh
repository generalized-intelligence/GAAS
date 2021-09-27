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
#include "../device/ctasegsort.cuh"
#include "../kernels/search.cuh"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// KernelSegBlocksortIndices

template<typename Tuning, bool Stable, bool HasValues, typename InputIt1,
	typename InputIt2, typename OutputIt1, typename OutputIt2, typename Comp>
MGPU_LAUNCH_BOUNDS void KernelSegBlocksortIndices(InputIt1 keys_global,
	InputIt2 values_global, int count, const int* indices_global, 
	const int* partitions_global, OutputIt1 keysDest_global,
	OutputIt2 valsDest_global, int* ranges_global, Comp comp) {

	typedef typename std::iterator_traits<InputIt1>::value_type KeyType;
	typedef typename std::iterator_traits<InputIt2>::value_type ValType;
	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;
	
	const int FlagWordsPerThread = MGPU_DIV_UP(VT, 4);
	struct Shared {
		union {
			byte flags[NV];
			int words[NT * FlagWordsPerThread];
			KeyType keys[NV];
			ValType values[NV];
		};
		int ranges[NT];
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = NV * block;
	int count2 = min(NV, count - gid);

	int headFlags = DeviceIndicesToHeadFlags<NT, VT>(indices_global,
		partitions_global, tid, block, count2, shared.words, shared.flags);
	
	DeviceSegBlocksort<NT, VT, Stable, HasValues>(keys_global, values_global,
		count2, shared.keys, shared.values, shared.ranges, headFlags, tid, 
		block, keysDest_global, valsDest_global, ranges_global, comp);
}

////////////////////////////////////////////////////////////////////////////////
// KernelSegBlocksortFlags

template<typename Tuning, bool Stable, bool HasValues, typename InputIt1,
	typename InputIt2, typename OutputIt1, typename OutputIt2, typename Comp>
MGPU_LAUNCH_BOUNDS void KernelSegBlocksortFlags(InputIt1 keys_global,
	InputIt2 values_global, int count, const uint* flags_global, 
	OutputIt1 keysDest_global, OutputIt2 valsDest_global, int* ranges_global, 
	Comp comp) {

	typedef typename std::iterator_traits<InputIt1>::value_type KeyType;
	typedef typename std::iterator_traits<InputIt2>::value_type ValType;
	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;
	struct Shared {
		union {
			KeyType keys[NV + VT];
			ValType values[NV];
		};
		union {
			int ranges[NT];
			uint flags[NT];
		};
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = NV * block;
	int count2 = min(NV, count - gid);

	// Load the head flags and keys.
	int flagsToLoad = min(32, count2 - 32 * tid);
	uint flags = 0xffffffff;
	if(flagsToLoad > 0) {
		flags = flags_global[(NV / 32) * block + tid];
		if(flagsToLoad < 32)
			flags |= 0xffffffff<< (31 & count2);
	}
	shared.flags[tid] = flags;
	__syncthreads();

	// Each thread extracts its own head flags from the array in shared memory.
	flags = DeviceExtractHeadFlags(shared.flags, VT * tid, VT);
	
	DeviceSegBlocksort<NT, VT, Stable, HasValues>(keys_global, values_global,
		count2, shared.keys, shared.values, shared.ranges, flags, tid, block,
		keysDest_global, valsDest_global, ranges_global, comp);
}

////////////////////////////////////////////////////////////////////////////////
// KernelSegSortMerge

template<typename Tuning, bool Segments, bool HasValues, typename KeyType, 
	typename ValueType, typename Comp>
MGPU_LAUNCH_BOUNDS void KernelSegSortMerge(const KeyType* keys_global, 
	const ValueType* values_global, SegSortSupport support, int count, 
	int pass, KeyType* keysDest_global, ValueType* valsDest_global, Comp comp) {

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;
	union Shared {
		KeyType keys[NT * (VT + 1)];
		int indices[NV];
		int4 range;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	
	// Check for merge work.
	while(true) {
		if(!tid) {
			int4 range = make_int4(-1, 0, 0, 0);
			int next = atomicAdd(&support.queueCounters_global->x, -1) - 1;
			if(next >= 0) range = support.mergeList_global[next];
			shared.range = range;
		}
		__syncthreads();

		int4 range = shared.range;
		__syncthreads();

		if(range.x < 0) break;

		int block = range.w;
		int gid = NV * block;
		int count2 = min(NV, count - gid);
		range.w = count2 - (range.y - range.x) + range.z;

		if(Segments) {			
			int compressedRange = support.ranges_global[block];
			int segStart = 0x0000ffff & compressedRange;
			int segEnd = compressedRange>> 16;

			DeviceSegSortMerge<NT, VT, HasValues>(keys_global, values_global,
				make_int2(segStart, segEnd), tid, block, range, pass, 
				shared.keys, shared.indices, keysDest_global, valsDest_global, 
				comp);
		} else
			// Unsegmented merge (from device/ctamerge.cuh)
			DeviceMerge<NT, VT, HasValues, false>(keys_global, values_global, 
				0, keys_global, values_global, 0, tid, block, range,
				shared.keys, shared.indices, keysDest_global, valsDest_global, 
				comp);
	}
	
	// Check for copy work.
	while(true) {
		if(!tid) {
			int list = -1;
			int next = atomicAdd(&support.queueCounters_global->y, -1) - 1;
			if(next >= 0) list = support.copyList_global[next];
			shared.range.x = list;
		}
		__syncthreads();

		int listBlock = shared.range.x;
		__syncthreads();

		if(listBlock < 0) break;

		DeviceSegSortCopy<NT, VT, HasValues>(keys_global, values_global,
			tid, listBlock, count, keysDest_global, valsDest_global);
	}
}

////////////////////////////////////////////////////////////////////////////////
// DeviceSegSortCreateJob
// Job-queueing code that is called by both seg sort partition kernels.

template<int NT>
MGPU_DEVICE void DeviceSegSortCreateJob(SegSortSupport support,
	int count, bool active, int3 frame, int tid, int pass, int nv, int block,
	int p0, int p1, int* shared) {

	typedef CTAScan<NT> S; 
	typename S::Storage* scan = (typename S::Storage*)shared;
		
	// Compute the gid'th work time.
	bool mergeOp = false;
	bool copyOp = false;
	int gid = nv * block;
	int count2 = min(nv, count - gid);

	int4 mergeRange;
	if(active) {
		mergeRange = FindMergesortInterval(frame, 2<< pass, block, nv, count, 
			p0, p1);

		// Merge if the source interval does not exactly cover the destination
		// interval. Otherwise copy or skip.
		int2 interval = (gid < frame.y) ? 
			make_int2(mergeRange.x, mergeRange.y) : // A coverage
			make_int2(mergeRange.z, mergeRange.w);	// B coverage.

		mergeOp = (gid != interval.x) || (interval.y - interval.x != count2); 
		copyOp = !mergeOp && (!pass || !support.copyStatus_global[block]);

	//	if(mergeOp)
	//		printf("%4d: %8d (%8d, %8d, %8d) (%8d, %8d: %4d) (%8d, %8d: %4d)\n", block,
	//			gid, frame.x, frame.y, frame.y + frame.z, mergeRange.x, mergeRange.y, mergeRange.y - mergeRange.x,
	//			mergeRange.z, mergeRange.w, mergeRange.w - mergeRange.z);

		// Use the 4th component to store the tile index. The actual b1
		// term is inferred from the tile size and array length. 
		mergeRange.w = block;
	}

	int mergeTotal, copyTotal;
	int mergeScan = S::Scan(tid, mergeOp, *scan, &mergeTotal);
	int copyScan = S::Scan(tid, copyOp, *scan, &copyTotal);
	if(!tid) {
		shared[0] = atomicAdd(&support.queueCounters_global->x, mergeTotal);
		shared[1] = atomicAdd(&support.queueCounters_global->y, copyTotal);
	}
	__syncthreads();

	if(mergeOp) {
		support.copyStatus_global[block] = 0;
		support.mergeList_global[shared[0] + mergeScan] = mergeRange;
	} 
	if(copyOp) {
		support.copyStatus_global[block] = 1;
		support.copyList_global[shared[1] + copyScan] = block;
	}
}


////////////////////////////////////////////////////////////////////////////////
// KernelSegSortPartitionBase
// KernelSegSortPartitionDerived

template<int NT, bool Segments, typename KeyType, typename Comp>
__global__ void KernelSegSortPartitionBase(const KeyType* keys_global,
	SegSortSupport support, int count, int nv, int numPartitions, Comp comp) {

	union Shared {
		int partitions[NT];
		typename CTAScan<NT>::Storage scan;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int partition = tid + (NT - 1) * blockIdx.x;

	// Compute one extra partition per CTA. If the CTA size is 128 threads, we
	// compute 128 partitions and 127 blocks. The next CTA then starts at
	// partition 127 rather than 128. This relieves us from having to launch
	// a second kernel to build the work queues.
	int p0;
	int3 frame;
	if(partition < numPartitions) {
		frame = FindMergesortFrame(2, partition, nv);
		int listLen = frame.z;
		int a0 = frame.x;
		int b0 = min(frame.y, count);
		int diag = nv * partition - a0;
		int aCount = min(listLen, count - a0);
		int bCount = min(listLen, count - b0);

		if(Segments) {
			// Segmented merge path calculation. Use the ranges as constraints.
			int leftRange = support.ranges_global[~1 & partition];
			int rightRange = support.ranges_global[
				min(numPartitions - 2, 1 | partition)];
		
			// Unpack the left and right ranges. Transform them into the global
			// coordinate system by adding a0 or b0.
			int leftStart = 0x0000ffff & leftRange;
			int leftEnd = leftRange>> 16;
			if(nv == leftStart) leftStart = count;
			else leftStart += a0;
			if(-1 != leftEnd) leftEnd += a0;

			int rightStart = 0x0000ffff & rightRange;
			int rightEnd = rightRange>> 16;
			if(nv == rightStart) rightStart = count;
			else rightStart += b0;
			if(-1 != rightEnd) rightEnd += b0;

			if(0 == diag)
				support.ranges2_global[partition>> 1] = make_int2(
					min(leftStart, rightStart),
					max(leftEnd, rightEnd));

			p0 = SegmentedMergePath(keys_global, a0, aCount, b0, bCount, 
				leftEnd, rightStart, diag, comp);
	
		} else
			// Unsegmented merge path search.
			p0 = MergePath<MgpuBoundsLower>(keys_global + a0, aCount,
				keys_global + b0, bCount, diag, comp);
		
		shared.partitions[tid] = p0;
	}
	__syncthreads();

	bool active = (tid < NT - 1) && (partition < numPartitions - 1);
	int p1 = active ? shared.partitions[tid + 1] : 0;
	__syncthreads();

	DeviceSegSortCreateJob<NT>(support, count, active, frame, tid, 0, nv,
		partition, p0, p1, shared.partitions);
}

template<int NT, bool Segments, typename KeyType, typename Comp>
__global__ void KernelSegSortPartitionDerived(const KeyType* keys_global,
	SegSortSupport support, int count, int numSources, int pass, int nv,
	int numPartitions, Comp comp) {

	union Shared {
		int partitions[NT];
		typename CTAScan<NT>::Storage scan;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int partition = tid + (NT - 1) * blockIdx.x;

	int3 frame;
	int p0;
	if(partition < numPartitions) {
		frame = FindMergesortFrame(2<< pass, partition, nv);
		int listLen = frame.z;
		int a0 = frame.x;
		int b0 = min(frame.y, count);
		int aCount = min(listLen, count - a0);
		int bCount = min(listLen, count - b0);
		int diag = min(nv * partition - a0, aCount + bCount);

		if(Segments) {
			int rangeIndex = partition>> pass;
			int2 leftRange = support.ranges2_global[~1 & rangeIndex];
			int2 rightRange = support.ranges2_global[
				min(numSources - 1, 1 | rangeIndex)];

			int leftEnd = leftRange.y;
			int rightStart = max(b0, rightRange.x);
			p0 = SegmentedMergePath(keys_global, a0, aCount, b0, bCount, 
				leftEnd, rightStart, diag, comp);

			if(0 == diag) 
				support.ranges2_global[numSources + (rangeIndex>> 1)] =
					make_int2(
						min(leftRange.x, rightRange.x),
						max(leftRange.y, rightRange.y));
	
		} else
			// Unsegmented merge path search.
			p0 = MergePath<MgpuBoundsLower>(keys_global + a0, aCount,
				keys_global + b0, bCount, diag, comp);

		shared.partitions[tid] = p0;
	}
	__syncthreads();

	bool active = (tid < NT - 1) && (partition < numPartitions - 1);
	int p1 = active ? shared.partitions[tid + 1] : 0;
	__syncthreads();

	DeviceSegSortCreateJob<NT>(support, count, active, frame, tid, pass, nv, 
		partition, p0, p1, shared.partitions);
	
	// Clear the counters for the next pass.
	if(!partition) *support.nextCounters_global = make_int2(0, 0);
}

////////////////////////////////////////////////////////////////////////////////
// AllocSegSortBuffers

MGPU_HOST MGPU_MEM(byte) AllocSegSortBuffers(int count, int nv,
	SegSortSupport& support, bool segments, CudaContext& context) {

	int numBlocks = MGPU_DIV_UP(count, nv);
	int numPasses = FindLog2(numBlocks, true);
	int numRanges = 1;
	int numBlocks2 = MGPU_DIV_UP(numBlocks, 2);
	for(int pass = 1; pass < numPasses; ++pass) {
		numRanges += numBlocks2;
		numBlocks2 = MGPU_DIV_UP(numBlocks2, 2);
	}

	int rangesSize = MGPU_ROUND_UP_POW2(sizeof(int) * numBlocks, 128);
	int ranges2Size = MGPU_ROUND_UP_POW2(sizeof(int2) * numRanges, 128);
	int mergeListSize = MGPU_ROUND_UP_POW2(sizeof(int4) * numBlocks, 128);
	int copyListSize = MGPU_ROUND_UP_POW2(sizeof(int) * numBlocks, 128);
	int countersSize = MGPU_ROUND_UP_POW2(sizeof(int4), 128);
	int copyStatusSize = MGPU_ROUND_UP_POW2(sizeof(byte) * numBlocks, 128);

	if(!segments) rangesSize = ranges2Size = 0;

	int total = rangesSize + ranges2Size + mergeListSize + copyListSize +
		countersSize + copyStatusSize;
	MGPU_MEM(byte) mem = context.Malloc<byte>(total);

	if(segments) {
		support.ranges_global = PtrOffset((int*)mem->get(), 0);
		support.ranges2_global = PtrOffset((int2*)support.ranges_global, 
			rangesSize);
		support.mergeList_global = PtrOffset((int4*)support.ranges2_global,
			ranges2Size);
	} else {
		support.ranges_global = 0;
		support.ranges2_global = 0;
		support.mergeList_global = (int4*)mem->get();
	}

	support.copyList_global = PtrOffset((int*)support.mergeList_global,
		mergeListSize);
	support.queueCounters_global = PtrOffset((int2*)support.copyList_global,
		copyListSize);
	support.nextCounters_global = PtrOffset(support.queueCounters_global, 
		sizeof(int2));
	support.copyStatus_global = PtrOffset((byte*)support.queueCounters_global,
		countersSize);

	// Fill the counters with 0s on the first run.
	cudaMemsetAsync(support.queueCounters_global, 0, sizeof(int4), 
		context.Stream());
	
	return mem;
}

class SegSortPassInfo {
public:
	SegSortPassInfo(int numBlocks_) : 
	  numBlocks(numBlocks_), totalMerge(0), totalCopy(0) { }

	void Pass(SegSortSupport& support, int pass) {
		int2 counters;
		cudaMemcpy(&counters, support.queueCounters_global, sizeof(int2), 
			cudaMemcpyDeviceToHost);

		printf("pass %2d:   %7d (%6.2lf%%)     %7d (%6.2lf%%)\n", pass,
			counters.x, 100.0 * counters.x / numBlocks,
			counters.y, 100.0 * counters.y / numBlocks);

		totalMerge += counters.x;
		totalCopy += counters.y;
	}
	void Final(int numPasses) {
		if(numPasses) {
			printf("average:   %7d (%6.2lf%%)     %7d (%6.2lf%%)\n", 
				totalMerge / numPasses,
				100.0 * totalMerge / (numPasses * numBlocks),
				totalCopy / numPasses,
				100.0 * totalCopy / (numPasses * numBlocks));
			printf("total  :   %7d (%6.2lf%%)     %7d (%6.2lf%%)\n",
				totalMerge, 100.0 * totalMerge / numBlocks,
				totalCopy, 100.0 * totalCopy / numBlocks);
		}
	}

	int numBlocks, totalMerge, totalCopy;
};

////////////////////////////////////////////////////////////////////////////////
// SegSortPasses
// Multi-pass segmented mergesort process. Factored out to allow simpler 
// specialization over head flags delivery in blocksort.

template<typename Tuning, bool Segments, bool HasValues, typename KeyType, 
	typename ValType, typename Comp>
MGPU_HOST void SegSortPasses(SegSortSupport& support, 
KeyType* keysSource_global, ValType* valsSource_global,
	int count, int numBlocks, int numPasses, KeyType* keysDest_global, 
	ValType* valsDest_global, Comp comp, CudaContext& context, bool verbose) {	

	int2 launch = Tuning::GetLaunchParams(context);
	int NV = launch.x * launch.y;

	const int NT2 = 64;
	int numPartitions = numBlocks + 1;
	int numPartBlocks = MGPU_DIV_UP(numPartitions, NT2 - 1);
	int numCTAs = min(numBlocks, 16 * 6);
	int numBlocks2 = MGPU_DIV_UP(numBlocks, 2);
	
	SegSortPassInfo info(numBlocks);
	for(int pass = 0; pass < numPasses; ++pass) {
		if(0 == pass) {
			KernelSegSortPartitionBase<NT2, Segments>
				<<<numPartBlocks, NT2, 0, context.Stream()>>>(keysSource_global,
				support, count, NV, numPartitions, comp);
			MGPU_SYNC_CHECK("KernelSegSortPartitionBase");
		} else {
			KernelSegSortPartitionDerived<NT2, Segments>
				<<<numPartBlocks, NT2, 0, context.Stream()>>>(keysSource_global, 
				support, count, numBlocks2, pass, NV, numPartitions, comp);
			MGPU_SYNC_CHECK("KernelSegSortPartitionDerived");

			support.ranges2_global += numBlocks2;
			numBlocks2 = MGPU_DIV_UP(numBlocks2, 2);
		}
		if(verbose) info.Pass(support, pass);
				
		KernelSegSortMerge<Tuning, Segments, HasValues>
			<<<numCTAs, launch.x, 0, context.Stream()>>>(keysSource_global,
			valsSource_global, support, count, pass, keysDest_global, 
			valsDest_global, comp);
		MGPU_SYNC_CHECK("KernelSegSortMerge");

		std::swap(keysDest_global, keysSource_global);
		std::swap(valsDest_global, valsSource_global);
		std::swap(support.queueCounters_global, support.nextCounters_global);
	}
	if(verbose) info.Final(numPasses);
}

////////////////////////////////////////////////////////////////////////////////
// Segmented sort from head flags in a bit field.

template<typename T, typename Comp>
MGPU_HOST void SegSortKeysFromFlags(T* data_global, int count,
	const uint* flags_global, CudaContext& context, Comp comp,
	bool verbose) {

	MGPU_VAR_UNUSED const bool Stable = true;

    typedef LaunchBoxVT<
		128, 11, 0,
		128, 11, 0,
		128, (sizeof(T) > 4) ? 7 : 11, 0
	> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;
	
	int numBlocks = MGPU_DIV_UP(count, NV);
	int numPasses = FindLog2(numBlocks, true);
	
	SegSortSupport support;
	MGPU_MEM(byte) mem = AllocSegSortBuffers(count, NV, support, true, context);

	MGPU_MEM(T) destDevice = context.Malloc<T>(count);
	T* source = data_global;
	T* dest = destDevice->get();

	KernelSegBlocksortFlags<Tuning, Stable, false>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(source, (const int*)0,
		count, flags_global, (1 & numPasses) ? dest : source, (int*)0,
		support.ranges_global, comp);
	MGPU_SYNC_CHECK("KernelSegBlocksortFlags");

	if(1 & numPasses) std::swap(source, dest);

	SegSortPasses<Tuning, true, false>(support, source, (int*)0, count, 
		numBlocks, numPasses, dest, (int*)0, comp, context, verbose);
}

template<typename T>
MGPU_HOST void SegSortKeysFromFlags(T* data_global, int count,
	const uint* flags_global, CudaContext& context, bool verbose) {

	SegSortKeysFromFlags(data_global, count, flags_global, context, 
		mgpu::less<T>(), verbose);
}

template<typename KeyType, typename ValType, typename Comp>
MGPU_HOST void SegSortPairsFromFlags(KeyType* keys_global,
	ValType* values_global, int count, const uint* flags_global,
	CudaContext& context, Comp comp, bool verbose) {

	MGPU_VAR_UNUSED const bool Stable = true;

    typedef LaunchBoxVT<
		128, 11, 0,
		128, 7, 0,
		128, 7, 0
	> Tuning;
	int2 launch = Tuning::GetLaunchParams(context); 
	const int NV = launch.x * launch.y;
	
	int numBlocks = MGPU_DIV_UP(count, NV);
	int numPasses = FindLog2(numBlocks, true);

	SegSortSupport support;
	MGPU_MEM(byte) mem = AllocSegSortBuffers(count, NV, support, true, context);

	MGPU_MEM(KeyType) keysDestDevice = context.Malloc<KeyType>(count);
	MGPU_MEM(ValType) valsDestDevice = context.Malloc<ValType>(count);

	KeyType* keysSource = keys_global;
	KeyType* keysDest = keysDestDevice->get();
	ValType* valsSource = values_global;
	ValType* valsDest = valsDestDevice->get();

	KernelSegBlocksortFlags<Tuning, Stable, true>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(keysSource, valsSource,
		count, flags_global, (1 & numPasses) ? keysDest : keysSource,
		(1 & numPasses) ? valsDest : valsSource, support.ranges_global, 
		comp);
	MGPU_SYNC_CHECK("KernelSegBlocksortFlags");

	std::vector<int> rangesHost(numBlocks);
	copyDtoH(&rangesHost[0], support.ranges_global, numBlocks);

	if(1 & numPasses) {
		std::swap(keysSource, keysDest);
		std::swap(valsSource, valsDest);
	}

	SegSortPasses<Tuning, true, true>(support, keysSource, valsSource, count, 
		numBlocks, numPasses, keysDest, valsDest, comp, context, verbose);
}
template<typename KeyType, typename ValType>
MGPU_HOST void SegSortPairsFromFlags(KeyType* keys_global, 
	ValType* values_global, int count, const uint* flags_global, 
	CudaContext& context, bool verbose) {

	SegSortPairsFromFlags(keys_global, values_global, count, flags_global,
		context, mgpu::less<KeyType>(), verbose);
}

////////////////////////////////////////////////////////////////////////////////
// Segmented sort from head flags passed as indices

template<typename T, typename Comp>
MGPU_HOST void SegSortKeysFromIndices(T* data_global, int count,
	const int* indices_global, int indicesCount, CudaContext& context,
	Comp comp, bool verbose) {

	MGPU_VAR_UNUSED const bool Stable = true;

    typedef LaunchBoxVT<
		128, 11, 0,
		128, 11, 0,
		128, (sizeof(T) > 4) ? 7 : 11, 0
	> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;
	
	int numBlocks = MGPU_DIV_UP(count, NV);
	int numPasses = FindLog2(numBlocks, true);

	SegSortSupport support;
	MGPU_MEM(byte) mem = AllocSegSortBuffers(count, NV, support, true, context);

	MGPU_MEM(T) destDevice = context.Malloc<T>(count);
	T* source = data_global;
	T* dest = destDevice->get();

	MGPU_MEM(int) partitionsDevice = BinarySearchPartitions<MgpuBoundsLower>(
		count, indices_global, indicesCount, NV, mgpu::less<int>(), context);

	KernelSegBlocksortIndices<Tuning, Stable, false>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(source, (const int*)0,
		count, indices_global, partitionsDevice->get(), 
		(1 & numPasses) ? dest : source, (int*)0, support.ranges_global, comp);
	MGPU_SYNC_CHECK("KernelSegBlocksortIndices");

	if(1 & numPasses) std::swap(source, dest);

	SegSortPasses<Tuning, true, false>(support, source, (int*)0, count, 
		numBlocks, numPasses, dest, (int*)0, comp, context, verbose);
}

template<typename T>
MGPU_HOST void SegSortKeysFromIndices(T* data_global, int count,
	const int* indices_global, int indicesCount, CudaContext& context,
	bool verbose) {

	SegSortKeysFromIndices(data_global, count, indices_global, indicesCount,
		context, mgpu::less<T>(), verbose);
}

template<typename KeyType, typename ValType, typename Comp>
MGPU_HOST void SegSortPairsFromIndices(KeyType* keys_global, 
	ValType* values_global, int count, const int* indices_global, 
	int indicesCount, CudaContext& context, Comp comp, bool verbose) {

	MGPU_VAR_UNUSED const bool Stable = true;

    typedef LaunchBoxVT<
		128, 11, 0,
		128, 7, 0,
		128, 7, 0
	> Tuning;
	int2 launch = Tuning::GetLaunchParams(context); 
	const int NV = launch.x * launch.y;
	
	int numBlocks = MGPU_DIV_UP(count, NV);
	int numPasses = FindLog2(numBlocks, true);
		
	SegSortSupport support;
	MGPU_MEM(byte) mem = AllocSegSortBuffers(count, NV, support, true, context);

	MGPU_MEM(KeyType) keysDestDevice = context.Malloc<KeyType>(count);
	MGPU_MEM(ValType) valsDestDevice = context.Malloc<ValType>(count);

	KeyType* keysSource = keys_global;
	KeyType* keysDest = keysDestDevice->get();
	ValType* valsSource = values_global;
	ValType* valsDest = valsDestDevice->get();

	MGPU_MEM(int) partitionsDevice = BinarySearchPartitions<MgpuBoundsLower>(
		count, indices_global, indicesCount, NV, mgpu::less<int>(), context);

	KernelSegBlocksortIndices<Tuning, Stable, true>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(keysSource, valsSource,
		count, indices_global, partitionsDevice->get(), 
		(1 & numPasses) ? keysDest : keysSource, 
		(1 & numPasses) ? valsDest : valsSource, support.ranges_global, comp);
	MGPU_SYNC_CHECK("KernelSegBlocksortIndices");

	if(1 & numPasses) {
		std::swap(keysSource, keysDest);
		std::swap(valsSource, valsDest);
	}

	SegSortPasses<Tuning, true, true>(support, keysSource, valsSource, count,
		numBlocks, numPasses, keysDest, valsDest, comp, context, verbose);
}
template<typename KeyType, typename ValType>
MGPU_HOST void SegSortPairsFromIndices(KeyType* keys_global, 
	ValType* values_global, int count, const int* indices_global, 
	int indicesCount, CudaContext& context, bool verbose) {

	SegSortPairsFromIndices(keys_global, values_global, count, indices_global, 
		indicesCount, context, mgpu::less<KeyType>(), verbose);
}

} // namespace mgpu
