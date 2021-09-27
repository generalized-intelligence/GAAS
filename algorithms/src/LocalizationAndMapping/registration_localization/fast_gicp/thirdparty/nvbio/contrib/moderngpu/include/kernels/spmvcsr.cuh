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

template<size_t Size, bool LoadLeft>
struct SpmvTuningNormal {
	enum { Indirect = false };
	typedef LaunchBox<
		SegReduceTuning<128, 11, 0, false, false>,
		SegReduceTuning<128, 11, 0, true, false>,
		SegReduceTuning<128, 7, 0, true, false>
	> Tuning;
};

template<size_t Size, bool LoadLeft>
struct SpmvTuningIndirect {
	enum { Indirect = true };
	typedef LaunchBox<
		SegReduceTuning<128, 11, 0, false, false>,
		SegReduceTuning<128, 11, 0, true, false>,
		SegReduceTuning<128, 7, 0, true, false>
	> Tuning;
};

template<size_t Size, bool LoadLeft>
struct SpmvTuningPreprocess {
	enum { Indirect = false };
	typedef LaunchBox<
		SegReduceTuning<128, 11, 0, false, false>,
		SegReduceTuning<128, 11, 0, true, false>,
		SegReduceTuning<128, (Size > 4) ? 11 : 7, 0, true, false>
	> Tuning;
};


////////////////////////////////////////////////////////////////////////////////
// CTASpmvLoad
// Loads matrix values and column indices and gathers vector values. Finds 
// products and transposes terms into register output in thread order.

template<int NT, int VT, bool LoadLeft, bool HalfCapacity, typename T,
	typename MulOp>
struct CTASpmvLoad {
	enum {
		NV = NT * VT,
		Capacity = HalfCapacity ? (NV / 2) : NV
	};
	typedef CTASegReduce<NT, VT, HalfCapacity, T, MulOp> SegReduce;
	
	union Storage {
		int sources[NV];
		T data[Capacity];
		typename SegReduce::Storage segReduceStorage;
	};

	template<typename MatrixIt, typename ColumnsIt, typename VecIt>
	MGPU_DEVICE static void LoadDirect(int count2, int tid, int gid, 
		MatrixIt matrix_global, ColumnsIt cols_global, VecIt vec_global, 
		T identity, MulOp mulOp, T data[VT], Storage& storage) {

		// Load columns directly from cols_global.
		int columns[VT];
		DeviceGlobalToRegDefault<NT, VT>(count2, cols_global + gid, tid,
			columns, 0);

		// Load data into stridedData.
		T matrixData[VT];
		if(LoadLeft)
			DeviceGlobalToRegDefault<NT, VT>(count2, matrix_global + gid,
				tid, matrixData, identity);
		
		// Use ldg to load vector data in strided order.
		T vecData[VT];
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			vecData[i] = ldg(vec_global + columns[i]);
		
		// Clear out the out-of-range inputs. 
		if(count2 < NV) {
			#pragma unroll
			for(int i = 0; i < VT; ++i)
				if(NT * i + tid >= count2)
					vecData[i] = identity;
		}	

		// Multiply matrix and vector values together.
		T stridedData[VT];
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			stridedData[i] = LoadLeft ? 
				mulOp(matrixData[i], vecData[i]) : vecData[i];

		// Transpose from strided to thread order.
		if(HalfCapacity)
			HalfSmemTranspose<NT, VT>(stridedData, tid, storage.data, data);
		else {
			DeviceRegToShared<NT, VT>(stridedData, tid, storage.data);
			DeviceSharedToThread<VT>(storage.data, tid, data);
		}
	}

	template<typename SourcesIt, typename MatrixIt, typename ColumnsIt,
		typename VecIt>
	MGPU_DEVICE static void LoadIndirect(int count2, int tid, int gid, 
		int numRows, int startRow, const int rows[VT], const int rowStarts[VT],
		SourcesIt sources_global, MatrixIt matrix_global, 
		ColumnsIt cols_global, VecIt vec_global, T identity, MulOp mulOp,
		T data[VT], Storage& storage) {
			
		// Load source offsets from sources_global into smem.
		DeviceGlobalToSharedLoop<NT, VT>(numRows, sources_global + startRow,
			tid, storage.sources);

		// Compute the offset of each element within its row.
		int indices[VT];
		#pragma unroll
		for(int i = 0; i < VT; ++i) {
			int index = VT * tid + i;
			int rowOffset = gid + index - rowStarts[i];
			int source = storage.sources[rows[i]];
			indices[i] = source + rowOffset;
		}
		__syncthreads();

		// Transpose indices through shared memory into strided order.
		DeviceThreadToShared<VT>(indices, tid, storage.sources);
		DeviceSharedToReg<NT, VT>(storage.sources, tid, indices);

		// Gather columns from cols_global.
		int columns[VT];
		DeviceGatherDefault<NT, VT>(count2, cols_global, indices, tid, 
			columns, 0);

		// Gather data into stridedData.
		T matrixData[VT];
		if(LoadLeft)
			DeviceGatherDefault<NT, VT>(count2, matrix_global, indices, 
				tid, matrixData, identity);						
		
		// Use ldg to load vector data in strided order.
		T vecData[VT];
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			vecData[i] = ldg(vec_global + columns[i]);

		// Multiply matrix and vector values together.
		T stridedData[VT];
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			stridedData[i] = LoadLeft ? 
				mulOp(matrixData[i], vecData[i]) : vecData[i];

		// Transpose from strided to thread order.
		if(HalfCapacity)
			HalfSmemTranspose<NT, VT>(stridedData, tid, storage.data, data);
		else {
			DeviceRegToShared<NT, VT>(stridedData, tid, storage.data);
			DeviceSharedToThread<VT>(storage.data, tid, data);
		}
	}
};

////////////////////////////////////////////////////////////////////////////////
// KernelSpmvCsr

template<typename Tuning, bool Indirect, bool LoadLeft, typename MatrixIt, 
	typename ColsIt, typename CsrIt, typename SourcesIt, typename VecIt, 
	typename DestIt, typename T, typename MulOp, typename AddOp>
MGPU_LAUNCH_BOUNDS void KernelSpmvCsr(MatrixIt matrix_global,
	ColsIt cols_global, int nz, CsrIt csr_global, SourcesIt sources_global, 
	VecIt vec_global, const int* limits_global, DestIt dest_global,
	T* carryOut_global, T identity, MulOp mulOp, AddOp addOp) {

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;
	const bool HalfCapacity = (sizeof(T) > sizeof(int)) && Params::HalfCapacity;

	typedef CTAReduce<NT, AddOp> FastReduce;
	typedef CTASegReduce<NT, VT, HalfCapacity, T, AddOp> SegReduce;
	typedef CTASpmvLoad<NT, VT, LoadLeft, HalfCapacity, T, MulOp> SpmvLoad;

	union Shared {
		int csr[NV + 1];
		typename SegReduce::Storage segReduceStorage;
		typename SpmvLoad::Storage spmvLoadStorage;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = NV * block;
	int count2 = min(NV, nz - gid);

	// Retrieve the left and right row limits.
	int limit0 = limits_global[block];
	int limit1 = limits_global[block + 1];

	SegReduceRange range;
	SegReduceTerms terms;
	int rows[VT + 1], rowStarts[VT];
	T data[VT];

	if(Indirect) {
		// Transform the row limits into ranges.
		range = DeviceShiftRange(limit0, limit1);
		int numRows = range.end - range.begin;

		// Load the CSR interval.
		DeviceGlobalToSharedLoop<NT, VT>(numRows, csr_global + range.begin, tid, 
			shared.csr);

		// Flatten CSR->COO and return the segmented scan terms.
		terms = DeviceSegReducePrepare<NT, VT>(shared.csr, numRows, tid, gid, 
			range.flushLast, rows, rowStarts);

		// Load tile of data in thread order from row IDs.
		SpmvLoad::LoadIndirect(count2, tid, gid, numRows, range.begin, rows, 
			rowStarts, sources_global, matrix_global, cols_global, vec_global,
			identity, mulOp, data, shared.spmvLoadStorage);
	} else {
		// This is a direct load so we don't have a data-dependency on the
		// limits.
		SpmvLoad::LoadDirect(count2, tid, gid, matrix_global, cols_global,
			vec_global, identity, mulOp, data, shared.spmvLoadStorage);

		// Transform the row limits into ranges.
		range = DeviceShiftRange(limit0, limit1);
		int numRows = range.end - range.begin;

		// Load the CSR interval.
		DeviceGlobalToSharedLoop<NT, VT>(numRows, csr_global + range.begin, tid, 
			shared.csr);

		// Flatten CSR->COO and return the segmented scan terms.
		terms = DeviceSegReducePrepare<NT, VT>(shared.csr, numRows, tid, gid,
			range.flushLast, rows, rowStarts);
	}

	// Reduce tile data and store to dest_global. Write tile's carry-out
	// term to carryOut_global.
	SegReduce::ReduceToGlobal(rows, range.total, terms.tidDelta, 
		range.begin, block, tid, data, dest_global, carryOut_global,
		identity, addOp, shared.segReduceStorage);
}

////////////////////////////////////////////////////////////////////////////////
// SpmvCsrHost

template<typename Tuning, bool Indirect, bool LoadLeft, typename MatrixIt, 
	typename ColsIt, typename CsrIt, typename SourcesIt, typename VecIt,
	typename DestIt, typename T, typename MulOp, typename AddOp>
MGPU_HOST void SpmvCsrInner(MatrixIt matrix_global, ColsIt cols_global, int nz,
	CsrIt csr_global, SourcesIt sources_global, int numRows, 
	const int* numRows2_global, VecIt vec_global, DestIt dest_global,
	T identity, MulOp mulOp, AddOp addOp, CudaContext& context) {

	int2 launch = Tuning::GetLaunchParams(context);
	int NV = launch.x * launch.y;

	int numBlocks = MGPU_DIV_UP(nz, NV);

	// Use upper-bound binary search to partition the CSR structure into tiles.
	MGPU_MEM(int) limitsDevice = PartitionCsrSegReduce(nz, NV, csr_global,
		numRows, numRows2_global, numBlocks + 1, context);
		
	// Evaluate the Spmv product.
	MGPU_MEM(T) carryOutDevice = context.Malloc<T>(numBlocks);
	KernelSpmvCsr<Tuning, Indirect, LoadLeft>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(matrix_global,
		cols_global, nz, csr_global, sources_global, vec_global, 
		limitsDevice->get(), dest_global, carryOutDevice->get(), identity, 
		mulOp, addOp);
	MGPU_SYNC_CHECK("KernelSpmvCsr");

	// Add the carry-in values.
	SegReduceSpine(limitsDevice->get(), numBlocks, dest_global,
		carryOutDevice->get(), identity, addOp, context);
}


template<typename Tuning, bool Indirect, bool LoadLeft, typename MatrixIt,
	typename ColsIt, typename CsrIt, typename SourcesIt, typename VecIt, 
	typename DestIt, typename T, typename MulOp, typename AddOp>
MGPU_HOST void SpmvCsrHost(MatrixIt matrix_global, ColsIt cols_global, int nz,
	CsrIt csr_global, SourcesIt sources_global, int numRows, VecIt vec_global,
	bool supportEmpty, DestIt dest_global, T identity, MulOp mulOp, AddOp addOp,
	CudaContext& context) {
		
	if(supportEmpty) {
		// Allocate space for CSR2 and Sources2.
		MGPU_MEM(int) csr2Device = context.Malloc<int>(numRows + 1);
		MGPU_MEM(int) sources2Device;
		if(Indirect) sources2Device = context.Malloc<int>(numRows);

		// Strip the empties from CSR and store in CSR2.
		CsrStripEmpties<Indirect>(nz, csr_global, sources_global, numRows,
			csr2Device->get(), Indirect ? sources2Device->get() : (int*)0, 
			(int*)0, context);

		// Run the Spmv in the CSR2 coordinate space.
		MGPU_MEM(T) destDevice = context.Malloc<T>(numRows);
		SpmvCsrInner<Tuning, Indirect, LoadLeft>(matrix_global, cols_global, nz,
			csr2Device->get(), Indirect ? sources2Device->get() : (const int*)0,
			-1, csr2Device->get() + numRows, vec_global,
			destDevice->get(), identity, mulOp, addOp, context);
		
		// Transform into the CSR space with BulkInsert.
		CsrBulkInsert(csr2Device->get(), numRows, destDevice->get(), identity,
			dest_global, context);

	} else {
		SpmvCsrInner<Tuning, Indirect, LoadLeft>(matrix_global, cols_global, nz,
			csr_global, sources_global, numRows, (const int*)0, vec_global, 
			dest_global, identity, mulOp, addOp, context);
	}
}

////////////////////////////////////////////////////////////////////////////////
// Spmv host functions

template<typename T>
struct spmv_pass_through : public std::binary_function<T, T, T> {
	MGPU_HOST_DEVICE T operator()(T a, T b) { return a; }
};

template<typename ColsIt, typename CsrIt, typename VecIt, typename DestIt,
	typename T, typename AddOp>
MGPU_HOST void SpmvCsrUnary( ColsIt cols_global, int nz, CsrIt csr_global,
	int numRows, VecIt vec_global, bool supportEmpty, DestIt dest_global,
	T identity, AddOp addOp, CudaContext& context) {

	typedef typename SpmvTuningNormal<sizeof(T), false>::Tuning Tuning;
	SpmvCsrHost<Tuning, false, false>((const T*)0, cols_global, nz, csr_global,
		(const int*)0, numRows, vec_global, supportEmpty, dest_global, 
		identity, spmv_pass_through<T>(), addOp, context);
}

template<typename MatrixIt, typename ColsIt, typename CsrIt, typename VecIt,
	typename DestIt, typename T, typename MulOp, typename AddOp>
MGPU_HOST void SpmvCsrBinary(MatrixIt matrix_global, ColsIt cols_global, int nz,
	CsrIt csr_global, int numRows, VecIt vec_global, bool supportEmpty, 
	DestIt dest_global, T identity, MulOp mulOp, AddOp addOp, 
	CudaContext& context) {
			
	typedef typename SpmvTuningNormal<sizeof(T), true>::Tuning Tuning;
	SpmvCsrHost<Tuning, false, true>(matrix_global, cols_global, nz, csr_global,
		(const int*)0, numRows, vec_global, supportEmpty, dest_global, 
		identity, mulOp, addOp, context);
}

template<typename ColsIt, typename CsrIt, typename SourcesIt, typename VecIt,
	typename DestIt, typename T, typename AddOp>
MGPU_HOST void SpmvCsrIndirectUnary(ColsIt cols_global, int nz,
	CsrIt csr_global, SourcesIt sources_global, int numRows, VecIt vec_global,
	bool supportEmpty, DestIt dest_global, T identity, AddOp addOp, 
	CudaContext& context) {

	typedef typename SpmvTuningIndirect<sizeof(T), false>::Tuning Tuning;
	SpmvCsrHost<Tuning, true, false>((const T*)0, cols_global, nz, csr_global,
		sources_global, numRows, vec_global, supportEmpty, dest_global, 
		identity, spmv_pass_through<T>(), addOp, context);
}

template<typename MatrixIt, typename ColsIt, typename CsrIt, typename SourcesIt,
	typename VecIt, typename DestIt, typename T, typename MulOp, typename AddOp>
MGPU_HOST void SpmvCsrIndirectBinary(MatrixIt matrix_global, ColsIt cols_global,
	int nz, CsrIt csr_global, SourcesIt sources_global, int numRows,
	VecIt vec_global, bool supportEmpty, DestIt dest_global, T identity,
	MulOp mulOp, AddOp addOp, CudaContext& context) {

	typedef typename SpmvTuningIndirect<sizeof(T), true>::Tuning Tuning;
	SpmvCsrHost<Tuning, true, true>(matrix_global, cols_global, nz, csr_global,
		sources_global, numRows, vec_global, supportEmpty, dest_global, 
		identity, mulOp, addOp, context);
}

////////////////////////////////////////////////////////////////////////////////
// Spmv preprocessing

template<typename T, typename CsrIt>
MGPU_HOST void SpmvPreprocessUnary(int nz, CsrIt csr_global, int numRows,
	bool supportEmpty, std::auto_ptr<SpmvPreprocessData>* ppData, 
	CudaContext& context) {

	typedef typename SpmvTuningPreprocess<sizeof(T), false>::Tuning Tuning;
	SegReducePreprocess<Tuning>(nz, csr_global, numRows, supportEmpty, ppData, 
		context);
}

template<typename T, typename CsrIt>
MGPU_HOST void SpmvPreprocessBinary(int nz, CsrIt csr_global, int numRows,
	bool supportEmpty, std::auto_ptr<SpmvPreprocessData>* ppData,
	CudaContext& context) {

	typedef typename SpmvTuningPreprocess<sizeof(T), true>::Tuning Tuning;
	SegReducePreprocess<Tuning>(nz, csr_global, numRows, supportEmpty, ppData, 
		context);
}

////////////////////////////////////////////////////////////////////////////////
// KernelSpmvApply

template<typename Tuning, bool LoadLeft, typename MatrixIt, typename ColsIt,
	typename VecIt, typename DestIt, typename T, typename MulOp, typename AddOp>
MGPU_LAUNCH_BOUNDS void KernelSpmvApply(const int* threadCodes_global,
	MatrixIt matrix_global, ColsIt cols_global, int nz, VecIt vec_global, 
	const int* limits_global, DestIt dest_global, T* carryOut_global, 
	T identity, MulOp mulOp, AddOp addOp) {

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;
	const bool HalfCapacity = (sizeof(T) > sizeof(int)) && Params::HalfCapacity;

	typedef CTASegReduce<NT, VT, HalfCapacity, T, AddOp> SegReduce;
	typedef CTASpmvLoad<NT, VT, LoadLeft, HalfCapacity, T, MulOp> SpmvLoad;

	union Shared {
		typename SegReduce::Storage segReduceStorage;
		typename SpmvLoad::Storage spmvLoadStorage;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = NV * block;
	int count2 = min(NV, nz - gid);

	// Retrieve the left and right row limits and thread codes..
	int limit0 = limits_global[block];
	int limit1 = limits_global[block + 1];
	int threadCodes = threadCodes_global[NT * block + tid];
	
	// Load the tile's data before dereferencing limit0/limit1.
	T data[VT];
	SpmvLoad::LoadDirect(count2, tid, gid, matrix_global, cols_global,
		vec_global, identity, mulOp, data, shared.spmvLoadStorage);

	// Transform the row limits into ranges.
	SegReduceRange range = DeviceShiftRange(limit0, limit1);

	// Expand the row indices.
	int rows[VT + 1];
	DeviceExpandFlagsToRows<VT>(threadCodes>> 20, threadCodes, rows);

	// Reduce tile data and store to dest_global. Write tile's carry-out
	// term to carryOut_global.
	int tidDelta = 0x7f & (threadCodes>> 13);
	SegReduce::ReduceToGlobal(rows, range.total, tidDelta, range.begin,	
		block, tid, data, dest_global, carryOut_global, identity, addOp, 
		shared.segReduceStorage);
}

template<bool LoadLeft, typename MatrixIt, typename ColsIt, typename VecIt, 
	typename DestIt, typename T, typename MulOp, typename AddOp>
MGPU_HOST void SpmvApplyHost(const SpmvPreprocessData& preprocess,
	MatrixIt matrix_global, ColsIt cols_global, VecIt vec_global, 
	DestIt dest_global, T identity, MulOp mulOp, AddOp addOp,
	CudaContext& context) {

	typedef typename SpmvTuningPreprocess<sizeof(T), LoadLeft>::Tuning Tuning;
	int2 launch = Tuning::GetLaunchParams(context);

	if(preprocess.csr2Device.get()) {
		// Support empties.
		MGPU_MEM(T) destDevice = context.Malloc<T>(preprocess.numSegments2);
		MGPU_MEM(T) carryOutDevice = context.Malloc<T>(preprocess.numBlocks);
		KernelSpmvApply<Tuning, LoadLeft>
			<<<preprocess.numBlocks, launch.x, 0, context.Stream()>>>(
			preprocess.threadCodesDevice->get(), matrix_global, cols_global,
			preprocess.count, vec_global, preprocess.limitsDevice->get(),
			destDevice->get(), carryOutDevice->get(), identity, mulOp,
			addOp);

		// Add the carry-in values.
		SegReduceSpine(preprocess.limitsDevice->get(), preprocess.numBlocks, 
			destDevice->get(), carryOutDevice->get(), identity, addOp,
			context);

		// Transform into the CSR space with BulkInsert.
		CsrBulkInsert(preprocess.csr2Device->get(), preprocess.numSegments, 
			destDevice->get(), identity, dest_global, context);

	} else {
		// No empties.

		// Evaluate the Spmv product.
		MGPU_MEM(T) carryOutDevice = context.Malloc<T>(preprocess.numBlocks);
		KernelSpmvApply<Tuning, LoadLeft>
			<<<preprocess.numBlocks, launch.x, 0, context.Stream()>>>(
			preprocess.threadCodesDevice->get(), matrix_global, cols_global,
			preprocess.count, vec_global, preprocess.limitsDevice->get(),
			dest_global, carryOutDevice->get(), identity, mulOp, addOp);
		MGPU_SYNC_CHECK("KernelSpmvApply");

		// Add the carry-in values.
		SegReduceSpine(preprocess.limitsDevice->get(), preprocess.numBlocks, 
			dest_global, carryOutDevice->get(), identity, addOp, context);
	}
}

template<typename ColsIt, typename VecIt, typename DestIt, typename T,
	typename MulOp, typename AddOp>
MGPU_HOST void SpmvUnaryApply(const SpmvPreprocessData& preprocess,
	ColsIt cols_global, VecIt vec_global, DestIt dest_global, T identity, 
	AddOp addOp, CudaContext& context) {

	SpmvApplyHost<false>(preprocess, (const T*)0, cols_global, vec_global,
		dest_global, identity, spmv_pass_through<T>(), addOp, context);
}

template<typename MatrixIt, typename ColsIt, typename VecIt, typename DestIt, 
	typename T, typename MulOp, typename AddOp>
MGPU_HOST void SpmvBinaryApply(const SpmvPreprocessData& preprocess,
	MatrixIt matrix_global, ColsIt cols_global, VecIt vec_global, 
	DestIt dest_global, T identity, MulOp mulOp, AddOp addOp,
	CudaContext& context) {

	SpmvApplyHost<true>(preprocess, matrix_global, cols_global, vec_global,
		dest_global, identity, mulOp, addOp, context);
}

} // namespace mgpu
