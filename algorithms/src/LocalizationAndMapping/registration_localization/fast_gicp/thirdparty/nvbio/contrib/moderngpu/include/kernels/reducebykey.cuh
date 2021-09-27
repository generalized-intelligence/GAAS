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

#include "../kernels/segreducecsr.cuh"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// KernelReduceByKeyPreprocess
// Stream through keys and find discontinuities. Compress these into a bitfield
// for each thread in the reduction CTA. Emit a count of discontinuities.
// These are scanned to provide limits.
template<typename Tuning, typename KeysIt, typename Comp>
MGPU_LAUNCH_BOUNDS void KernelReduceByKeyPreprocess(KeysIt keys_global,
	int count, int* threadCodes_global, int* counts_global, Comp comp) {

	typedef typename std::iterator_traits<KeysIt>::value_type T;
	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	
	const int VT = Params::VT;
	const int NV = NT * VT;

	union Shared {
		T keys[NT * (VT + 1)];
		int indices[NT];
		typename CTAScan<NT>::Storage scanStorage;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = NV * block;
	int count2 = min(NV + 1, count - gid);

	// Load the keys for this tile with one following element. This allows us 
	// to determine end flags for all segments.
	DeviceGlobalToShared2<NT, VT, VT + 1>(count2, keys_global + gid, tid,
		shared.keys);

	// Compare adjacent keys in each thread and mark discontinuities in 
	// endFlags bits.
	int endFlags = 0;
	if(count2 > NV) {
		T key = shared.keys[VT * tid];	
		#pragma unroll
		for(int i = 0; i < VT; ++i) {
			T next = shared.keys[VT * tid + 1 + i];
			if(!comp(key, next)) endFlags |= 1<< i;
			key = next;
		}
	} else {
		T key = shared.keys[VT * tid];	
		#pragma unroll
		for(int i = 0; i < VT; ++i) {
			int index = VT * tid + 1 + i;
			T next = shared.keys[index];
			if(index == count2 || (index < count2 && !comp(key, next)))
				endFlags |= 1<< i;
			key = next;
		}
	}
	__syncthreads();
		
	// Count the number of encountered end flags.
	int total;
	int scan = CTAScan<NT>::Scan(tid, popc(endFlags), shared.scanStorage,
		&total);

	if(!tid)
		counts_global[block] = total;

	if(total) {
		// Find the segmented scan start for this thread.
		int tidDelta = DeviceFindSegScanDelta<NT>(tid, 0 != endFlags, 
			shared.indices);

		// threadCodes:
		// 12:0 - end flags for up to 13 values per thread.
		// 19:13 - tid delta for up to 128 threads.
		// 30:20 - scan offset for streaming partials.
		int threadCodes = endFlags | (tidDelta<< 13) | (scan<< 20);
		threadCodes_global[NT * block + tid] = threadCodes;
	}
}

////////////////////////////////////////////////////////////////////////////////
// KernelReduceByKeyEmit

template<typename Tuning, typename KeysIt, typename KeyType>
MGPU_LAUNCH_BOUNDS void KernelReduceByKeyEmit(KeysIt keys_global,
	int count, const int* threadCodes_global, const int* limits_global,
	KeyType* keysDest_global) {

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;

	union Shared {
		int indices[NV];
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = NV * block;

	int limit0 = limits_global[block];
	int limit1 = limits_global[block + 1];
	int threadCodes = threadCodes_global[NT * block + tid];

	int total = limit1 - limit0;
	if(total) {
		// Reconstruct row IDs from thread codes and the starting row offset.
		int rows[VT + 1];
		DeviceExpandFlagsToRows<VT>(threadCodes>> 20, threadCodes, rows);
		
		// Compact the location of the last element in each segment.
		int index = gid + VT * tid;
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			if(rows[i] != rows[i + 1])
				shared.indices[rows[i]] = index + i;
		__syncthreads();

		// Copy a key from the start of each segment.
		for(int i = tid; i < total; i += NT) {
			int pos = shared.indices[i] + 1;
			int seg = limit0 + i + 1;
			if(pos >= count) {
				pos = 0;
				seg = 0;
			}
			keysDest_global[seg] = keys_global[pos];
		}
	}
}

////////////////////////////////////////////////////////////////////////////////
// ReduceByKeyPreprocess

template<typename ValType, typename KeyType, typename KeysIt, typename Comp>
MGPU_HOST void ReduceByKeyPreprocess(int count, KeysIt keys_global, 
	KeyType* keysDest_global, Comp comp, int* count_host, int* count_global,
	std::auto_ptr<ReduceByKeyPreprocessData>* ppData, CudaContext& context) {

	typedef typename SegReducePreprocessTuning<sizeof(ValType)>::Tuning Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	int NV = launch.x * launch.y;

	const bool AsyncTransfer = true;

	std::auto_ptr<ReduceByKeyPreprocessData> 
		data(new ReduceByKeyPreprocessData);

	int numBlocks = MGPU_DIV_UP(count, NV);
	data->count = count;
	data->numBlocks = numBlocks;
	data->limitsDevice = context.Malloc<int>(numBlocks + 1);
	data->threadCodesDevice = context.Malloc<int>(numBlocks * launch.x);

	// Fill out thread codes for each thread in the processing CTAs.
	KernelReduceByKeyPreprocess<Tuning>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(keys_global, count, 
		data->threadCodesDevice->get(), data->limitsDevice->get(), comp);
	MGPU_SYNC_CHECK("KernelReduceByKeyPreprocess");

	// Scan the output counts.
	Scan<MgpuScanTypeExc>(data->limitsDevice->get(), numBlocks, 0,
		mgpu::plus<int>(), data->limitsDevice->get() + numBlocks, (int*)0,
		data->limitsDevice->get(), context);

	// Queue up a transfer of the row total.
	if(count_global)
		copyDtoD(count_global, data->limitsDevice->get() + numBlocks, 1,
			context.Stream());
	if(count_host) {
		if(AsyncTransfer) {
			cudaError_t error = cudaMemcpyAsync(context.PageLocked(), 
				data->limitsDevice->get() + numBlocks, sizeof(int),
				cudaMemcpyDeviceToHost, context.Stream());
			error = cudaEventRecord(context.Event(), context.Stream());
		} else
			copyDtoH(count_host, data->limitsDevice->get() + numBlocks, 1);
	}

	// Output one key per segment.
	if(keysDest_global) {
		KernelReduceByKeyEmit<Tuning>
			<<<numBlocks, launch.x, 0, context.Stream()>>>(keys_global,
			count, data->threadCodesDevice->get(), data->limitsDevice->get(),
			keysDest_global);
		MGPU_SYNC_CHECK("KernelReduceByKeyEmit");
	}

	// Retrieve the number of rows.
	if(AsyncTransfer && count_host) {
		cudaError_t error = cudaEventSynchronize(context.Event());
		*count_host = *context.PageLocked();
	}
	data->numSegments = count_host ? *count_host : -1;

	*ppData = data;
}

////////////////////////////////////////////////////////////////////////////////
// ReduceByKey host function.

template<typename KeysIt, typename InputIt, typename DestIt,
	typename KeyType, typename ValType, typename Op, typename Comp>
MGPU_HOST void ReduceByKey(KeysIt keys_global, InputIt data_global, int count,
	ValType identity, Op op, Comp comp, KeyType* keysDest_global, 
	DestIt dest_global, int* count_host, int* count_global, 
	CudaContext& context) {
		
	std::auto_ptr<ReduceByKeyPreprocessData> data;
	MGPU_MEM(int) countsDevice = context.Malloc<int>(1);
	if(count_host && !count_global) count_global = countsDevice->get();

	// Preprocess the keys and emit the first key in each segment.
	ReduceByKeyPreprocess<ValType>(count, keys_global, keysDest_global, comp, 
		(int*)0, count_global, &data, context);
	
	const bool AsyncTransfer = true;
	if(count_host) {
		if(AsyncTransfer) {
			cudaError_t error = cudaMemcpyAsync(context.PageLocked(), 
				count_global, sizeof(int), cudaMemcpyDeviceToHost, 
				context.Stream());
			error = cudaEventRecord(context.Event(), context.Stream());
		} else
			copyDtoH(count_host, count_global, 1);
	}

	// Evaluate the segmented reduction.
	SegReduceApply(*data, data_global, identity, op, dest_global, context);

	// Retrieve the number of segments.
	if(AsyncTransfer && count_host) {
		cudaError_t error = cudaEventSynchronize(context.Event());
		*count_host = *context.PageLocked();
	}
}

////////////////////////////////////////////////////////////////////////////////
// ReduceByKeyApply

template<typename InputIt, typename DestIt, typename T, typename Op>
MGPU_HOST void ReduceByKeyApply(const ReduceByKeyPreprocessData& preprocess, 
	InputIt data_global, T identity, Op op, DestIt dest_global,
	CudaContext& context) {

	return SegReduceApply(preprocess, data_global, identity, op, dest_global,
		context);
}

} // namespace mgpu
