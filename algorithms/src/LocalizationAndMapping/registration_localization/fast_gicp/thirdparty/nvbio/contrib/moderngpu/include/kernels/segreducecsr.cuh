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
#include "../kernels/segreduce.cuh"
#include "../kernels/bulkinsert.cuh"

namespace mgpu {

// SegReduceCSR - Normal
template<size_t size>
struct SegReduceNormalTuning {
	typedef LaunchBox<
		SegReduceTuning<128, 11, 0, false, false>,
		SegReduceTuning<128, 7, 0, true, false>,
		SegReduceTuning<128, (size > sizeof(int)) ? 11 : 7, 0, true, true>
	> Tuning;
};

// SegReduceCSR - Preprocess
template<size_t size>
struct SegReducePreprocessTuning {
	typedef LaunchBox<
		SegReduceTuning<128, 11, 0, false, false>,
		SegReduceTuning<128, 11, 0, true, false>,
		SegReduceTuning<128, 11, 0, true, (size > 4) ? false : true>
	> Tuning;
};

// SegReduceCSR - Indirect
template<size_t size>
struct SegReduceIndirectTuning {
	typedef LaunchBox<
		SegReduceTuning<128, 11, 0, false, false>,
		SegReduceTuning<128, 7, 0, true, false>,
		SegReduceTuning<128, 7, 0, true, true>
	> Tuning;
};

////////////////////////////////////////////////////////////////////////////////
// CTAIntervalSegReduceGather
// Storage and logic for segmented reduce and interval reduce.
// Pass the Reduce function data in thread order.

template<int NT, int VT, bool HalfCapacity, bool LdgTranspose, typename T>
struct CTASegReduceLoad {
	enum {
		NV = NT * VT,
		Capacity = HalfCapacity ? (NV / 2) : NV
	};
	
	union Storage {
		int sources[NV];
		T data[Capacity];
	};
	
	// Load elements from multiple segments and store in thread order.
	template<typename InputIt>
	MGPU_DEVICE static void LoadDirect(int count2, int tid, int gid, 
		InputIt data_global, T identity, T data[VT], Storage& storage) {

		if(LdgTranspose) {
			// Load data in thread order from data_global + gid.
			DeviceGlobalToThreadDefault<NT, VT>(count2, data_global + gid,
				tid, data, identity);
		} else {
			// Load data in strided order from data_global + gid.
			T stridedData[VT];
			DeviceGlobalToRegDefault<NT, VT>(count2, data_global + gid, tid, 
				stridedData, identity);

			if(HalfCapacity)
				HalfSmemTranspose<NT, VT>(stridedData, tid, storage.data, data);
			else {
				DeviceRegToShared<NT, VT>(stridedData, tid, storage.data);
				DeviceSharedToThread<VT>(storage.data, tid, data);
			}
		}
	}

	// Load elements from multiple segments and store in thread order.
	template<typename InputIt, typename SourcesIt>
	MGPU_DEVICE static void LoadIndirect(int count2, int tid, int gid, 
		int numSegments, int startSeg, const int segs[VT + 1], 
		const int segStarts[VT], InputIt data_global, SourcesIt sources_global,
		T identity, T data[VT], Storage& storage) {

		T stridedData[VT];
		
		// Load source offsets from sources_global into smem.
		DeviceGlobalToSharedLoop<NT, VT>(numSegments, sources_global + startSeg,
			tid, storage.sources);

		// Compute the offset of each element within its segment.
		int indices[VT];
		#pragma unroll
		for(int i = 0; i < VT; ++i) {
			int index = VT * tid + i;
			int segOffset = gid + index - segStarts[i];
			int source = storage.sources[segs[i]];
			indices[i] = (index < count2) ? (source + segOffset) : 0;
		}
		__syncthreads();

		if(LdgTranspose) {
			// Directly load in thread order.
			#pragma unroll
			for(int i = 0; i < VT; ++i)
				data[i] = ldg(data_global + indices[i]);
		} else {
			// Transpose indices through shared memory.
			DeviceThreadToShared<VT>(indices, tid, storage.sources);
			DeviceSharedToReg<NT, VT>(storage.sources, tid, indices);

			// Cooperatively load all data elements.
			#pragma unroll
			for(int i = 0; i < VT; ++i) {
				int index = NT * i + tid;
				stridedData[i] = (index < count2) ? 
					data_global[indices[i]] :
					identity;
			}
			if(HalfCapacity)
				HalfSmemTranspose<NT, VT>(stridedData, tid, storage.data, data);
			else {
				DeviceRegToShared<NT, VT>(stridedData, tid, storage.data);
				DeviceSharedToThread<VT>(storage.data, tid, data);
			}
		}
	}

	// Load elements from a single segment and store in thread order.
	template<typename CsrIt, typename InputIt, typename SourcesIt>
	MGPU_DEVICE static void LoadIndirectFast(int tid, int gid, 
		int startSeg, CsrIt csr_global, InputIt data_global,
		SourcesIt sources_global, T data[VT], Storage& storage) {

		int source = sources_global[startSeg];
		int csr = csr_global[startSeg];

		int offset = source + gid - csr;
					
		// Round down to a multiple of NT. This guarantees all loads are
		// cache-line aligned.
		int mod = offset % NT;
		int start = offset - mod;

		data_global += start;
		data[0] = data_global[(tid < mod) ? (NV + tid) : tid];
		#pragma unroll
		for(int i = 1; i < VT; ++i)
			data[i] = data_global[NT * i + tid];
	}
};


////////////////////////////////////////////////////////////////////////////////
// KernelSegReduceCsr

template<typename Tuning, bool Indirect, typename CsrIt, typename SourcesIt,
	typename InputIt, typename DestIt, typename T, typename Op>
MGPU_LAUNCH_BOUNDS void KernelSegReduceCsr(CsrIt csr_global, 
	SourcesIt sources_global, int count, const int* limits_global, 
	InputIt data_global, T identity, Op op, DestIt dest_global,
	T* carryOut_global) {

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;
	const bool HalfCapacity = (sizeof(T) > sizeof(int)) && Params::HalfCapacity;
	const bool LdgTranspose = Params::LdgTranspose;

	typedef CTAReduce<NT, Op> FastReduce;
	typedef CTASegReduce<NT, VT, HalfCapacity, T, Op> SegReduce;
	typedef CTASegReduceLoad<NT, VT, HalfCapacity, LdgTranspose, T>
		SegReduceLoad;

	union Shared {
		int csr[NV + 1];
		typename FastReduce::Storage reduceStorage;
		typename SegReduce::Storage segReduceStorage;
		typename SegReduceLoad::Storage loadStorage;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = NV * block;
	int count2 = min(NV, count - gid);

	int limit0 = limits_global[block];
	int limit1 = limits_global[block + 1];

	SegReduceRange range;
	SegReduceTerms terms;
	int segs[VT + 1], segStarts[VT];
	T data[VT];
	if(Indirect) {
		// Indirect load. We need to load the CSR terms before loading any data.
		range = DeviceShiftRange(limit0, limit1);
		int numSegments = range.end - range.begin;

		if(range.total) {

			// Load the CSR interval.
			DeviceGlobalToSharedLoop<NT, VT>(numSegments, 
				csr_global + range.begin, tid, shared.csr);
	
			// Compute the segmented scan terms.
			terms = DeviceSegReducePrepare<NT, VT>(shared.csr, numSegments,
				tid, gid, range.flushLast, segs, segStarts);

			// Load tile of data in thread order from segment IDs.
			SegReduceLoad::LoadIndirect(count2, tid, gid, numSegments,
				range.begin, segs, segStarts, data_global, sources_global,
				identity, data, shared.loadStorage);
		} else {
			SegReduceLoad::LoadIndirectFast(tid, gid, range.begin, csr_global, 
				data_global, sources_global, data, shared.loadStorage);
		}

	} else {
		// Direct load. It is more efficient to load the full tile before
		// dealing with data dependencies.
		SegReduceLoad::LoadDirect(count2, tid, gid, data_global, identity,
			data, shared.loadStorage);

		range = DeviceShiftRange(limit0, limit1);
		int numSegments = range.end - range.begin;

		if(range.total) {
			// Load the CSR interval.
			DeviceGlobalToSharedLoop<NT, VT>(numSegments, 
				csr_global + range.begin, tid, shared.csr);
	
			// Compute the segmented scan terms.
			terms = DeviceSegReducePrepare<NT, VT>(shared.csr, numSegments,
				tid, gid, range.flushLast, segs, segStarts);
		}
	}

	if(range.total) {
		// Reduce tile data and store to dest_global. Write tile's carry-out
		// term to carryOut_global.
		SegReduce::ReduceToGlobal(segs, range.total, terms.tidDelta, 
			range.begin, block, tid, data, dest_global, carryOut_global,
			identity, op, shared.segReduceStorage);
	} else {
		T x;
		#pragma unroll
		for(int i = 0; i < VT; ++i)
		x = i ? op(x, data[i]) : data[i];
		x = FastReduce::Reduce(tid, x, shared.reduceStorage, op);
		if(!tid)
			carryOut_global[block] = x;
	}
}

////////////////////////////////////////////////////////////////////////////////
// SegReduceHost
// Generic host implementation for seg-reduce and interval-reduce.

template<typename Tuning, bool Indirect, typename InputIt,
	typename CsrIt, typename SourcesIt, typename DestIt, typename T,
	typename Op>
MGPU_HOST void SegReduceInner(InputIt data_global, CsrIt csr_global,
	SourcesIt sources_global, int count, int numSegments,
	const int* numSegments2_global, DestIt dest_global, T identity, Op op,
	CudaContext& context) {

	int2 launch = Tuning::GetLaunchParams(context);
	int NV = launch.x * launch.y;

	int numBlocks = MGPU_DIV_UP(count, NV);

	// Use upper-bound binary search to partition the CSR structure into tiles.
	MGPU_MEM(int) limitsDevice = PartitionCsrSegReduce(count, NV, csr_global,
		numSegments, numSegments2_global, numBlocks + 1, context);

	// Segmented reduction without source intervals.
	MGPU_MEM(T) carryOutDevice = context.Malloc<T>(numBlocks);
	KernelSegReduceCsr<Tuning, Indirect>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(csr_global,
		sources_global, count, limitsDevice->get(), data_global, identity, op, 
		dest_global, carryOutDevice->get());
	MGPU_SYNC_CHECK("KernelSegReduceCsr");

	// Add carry-in values.
	SegReduceSpine(limitsDevice->get(), numBlocks, dest_global, 
		carryOutDevice->get(), identity, op, context);
}

template<typename Tuning, bool Indirect, typename InputIt,
	typename CsrIt, typename SourcesIt, typename DestIt, typename T,
	typename Op>
MGPU_HOST void SegReduceHost(InputIt data_global, CsrIt csr_global,
	SourcesIt sources_global, int count, int numSegments, bool supportEmpty, 
	DestIt dest_global, T identity, Op op, CudaContext& context) {

	if(supportEmpty) {
		// Allocate space for Csr2 and Sources2.
		MGPU_MEM(int) csr2Device = context.Malloc<int>(numSegments + 1);
		MGPU_MEM(int) sources2Device;
		if(Indirect) sources2Device = context.Malloc<int>(numSegments);

		// Strip the empties from Csr and store in Csr2.
		CsrStripEmpties<Indirect>(count, csr_global, sources_global, numSegments,
			csr2Device->get(), Indirect ? sources2Device->get() : (int*)0, 
			(int*)0, context);
		
		// Run the segmented reduction in the Csr2 coordinate space.
		MGPU_MEM(T) destDevice = context.Malloc<T>(numSegments);
		SegReduceInner<Tuning, Indirect>(data_global, csr2Device->get(),
			Indirect ? sources2Device->get() : (const int*)0, count, -1,
			csr2Device->get() + numSegments, destDevice->get(), identity, op, 
			context);

		// Transform into the Csr space with BulkInsert.
		CsrBulkInsert(csr2Device->get(), numSegments, destDevice->get(), identity,
			dest_global, context);

	} else {
		// Evaluate the reduction directly into dest_global.
		SegReduceInner<Tuning, Indirect>(data_global, csr_global, 
			sources_global, count, numSegments, (const int*)0, dest_global,
			identity, op, context);
	}
}

template<typename InputIt, typename CsrIt, typename OutputIt, typename T,
	typename Op>
MGPU_HOST void SegReduceCsr(InputIt data_global, CsrIt csr_global, int count,
	int numSegments, bool supportEmpty, OutputIt dest_global, T identity, Op op, 
	CudaContext& context) {

	typedef typename SegReduceNormalTuning<sizeof(T)>::Tuning Tuning;

	SegReduceHost<Tuning, false>(data_global, csr_global, (const int*)0,
		count, numSegments, supportEmpty, dest_global, identity, op, context);
}

template<typename InputIt, typename CsrIt, typename SourcesIt, 
	typename OutputIt, typename T, typename Op>
MGPU_HOST void IndirectReduceCsr(InputIt data_global, CsrIt csr_global,
	SourcesIt sources_global, int count, int numSegments, bool supportEmpty,
	OutputIt dest_global, T identity, Op op, CudaContext& context) {

	typedef typename SegReduceIndirectTuning<sizeof(T)>::Tuning Tuning;

	SegReduceHost<Tuning, true>(data_global, csr_global, sources_global, count,
		numSegments, supportEmpty, dest_global, identity, op, context);
}

////////////////////////////////////////////////////////////////////////////////
// seg-reduce-csr preprocessed format.

template<typename T, typename CsrIt>
MGPU_HOST void SegReduceCsrPreprocess(int count, CsrIt csr_global, int numSegments,
	bool supportEmpty, std::auto_ptr<SegReducePreprocessData>* ppData,
	CudaContext& context) {

	typedef typename SegReducePreprocessTuning<sizeof(T)>::Tuning Tuning;
	SegReducePreprocess<Tuning>(count, csr_global, numSegments, supportEmpty, 
		ppData, context);
}

template<typename Tuning, typename InputIt, typename DestIt, typename T,
	typename Op>
MGPU_LAUNCH_BOUNDS void KernelSegReduceApply(const int* threadCodes_global,
	int count, const int* limits_global, InputIt data_global, T identity,
	Op op, DestIt dest_global, T* carryOut_global) {

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;
	const bool HalfCapacity = (sizeof(T) > sizeof(int)) && Params::HalfCapacity;
	const bool LdgTranspose = Params::LdgTranspose;

	typedef CTAReduce<NT, Op> FastReduce;
	typedef CTASegReduce<NT, VT, HalfCapacity, T, Op> SegReduce;
	typedef CTASegReduceLoad<NT, VT, HalfCapacity, LdgTranspose, T>
		SegReduceLoad;

	union Shared {
		int csr[NV];
		typename FastReduce::Storage reduceStorage;
		typename SegReduce::Storage segReduceStorage;
		typename SegReduceLoad::Storage loadStorage;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = NV * block;
	int count2 = min(NV, count - gid);

	int limit0 = limits_global[block];
	int limit1 = limits_global[block + 1];
	int threadCodes = threadCodes_global[NT * block + tid];

	// Load the data and transpose into thread order.
	T data[VT];
	SegReduceLoad::LoadDirect(count2, tid, gid, data_global, identity, data,
		shared.loadStorage);

	// Compute the range.
	SegReduceRange range = DeviceShiftRange(limit0, limit1);

	if(range.total) {
		// Expand the segment indices.
		int segs[VT + 1];
		DeviceExpandFlagsToRows<VT>(threadCodes>> 20, threadCodes, segs);

		// Reduce tile data and store to dest_global. Write tile's carry-out
		// term to carryOut_global.
		int tidDelta = 0x7f & (threadCodes>> 13);
		SegReduce::ReduceToGlobal(segs, range.total, tidDelta, range.begin,
			block, tid, data, dest_global, carryOut_global, identity, op, 
			shared.segReduceStorage);
	} else {
		// If there are no end flags in this CTA, use a fast reduction.
		T x;
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			x = i ? op(x, data[i]) : data[i];
		x = FastReduce::Reduce(tid, x, shared.reduceStorage, op);
		if(!tid)
			carryOut_global[block] = x;
	}
}

template<typename InputIt, typename DestIt, typename T, typename Op>
MGPU_HOST void SegReduceApply(const SegReducePreprocessData& preprocess, 
	InputIt data_global, T identity, Op op, DestIt dest_global,
	CudaContext& context) {

	typedef typename SegReducePreprocessTuning<sizeof(T)>::Tuning Tuning;
	int2 launch = Tuning::GetLaunchParams(context);

	if(preprocess.csr2Device.get()) {
		// Support empties.
		MGPU_MEM(T) tempOutDevice = context.Malloc<T>(preprocess.numSegments2);
		MGPU_MEM(T) carryOutDevice = context.Malloc<T>(preprocess.numBlocks);
		KernelSegReduceApply<Tuning>
			<<<preprocess.numBlocks, launch.x, 0, context.Stream()>>>(
			preprocess.threadCodesDevice->get(), preprocess.count, 
			preprocess.limitsDevice->get(), data_global, identity, op, 
			tempOutDevice->get(), carryOutDevice->get());
		MGPU_SYNC_CHECK("KernelSegReduceApply");

		// Add the carry-in values.
		SegReduceSpine(preprocess.limitsDevice->get(), preprocess.numBlocks, 
			tempOutDevice->get(), carryOutDevice->get(), identity, op, context);

		// Insert identity into the empty segments and stream into dest_global.
		BulkInsert(mgpu::constant_iterator<T>(identity), 
			preprocess.csr2Device->get() + preprocess.numSegments2,
			preprocess.numSegments - preprocess.numSegments2, 
			tempOutDevice->get(), preprocess.numSegments2, dest_global,
			context);
	} else {
		// No empties.
		MGPU_MEM(T) carryOutDevice = context.Malloc<T>(preprocess.numBlocks);
		KernelSegReduceApply<Tuning>
			<<<preprocess.numBlocks, launch.x, 0, context.Stream()>>>(
			preprocess.threadCodesDevice->get(), preprocess.count, 
			preprocess.limitsDevice->get(), data_global, identity, op, 
			dest_global, carryOutDevice->get());
		MGPU_SYNC_CHECK("KernelSegReduceApply");

		// Add the carry-in values.
		SegReduceSpine(preprocess.limitsDevice->get(), preprocess.numBlocks, 
			dest_global, carryOutDevice->get(), identity, op, context);
	}
}

} // namespace mgpu
