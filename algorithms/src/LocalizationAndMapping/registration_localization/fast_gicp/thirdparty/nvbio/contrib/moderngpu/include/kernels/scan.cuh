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

#include "../mgpudevice.cuh"
#include "../kernels/reduce.cuh"

namespace mgpu {


////////////////////////////////////////////////////////////////////////////////

template<int NT, int VT, typename T, typename Op>
struct CTATileScan {
	enum { NV = NT * VT };
	typedef CTAScan<NT, Op> S;

	union Storage {
		T data[NV];
		typename S::Storage scanStorage;
	};

	template<MgpuScanType Type, typename DataIt, typename DestIt>
	MGPU_DEVICE static T DeviceScanTile(DataIt data_global, int count, int tid,
		T identity, Op op, T start, DestIt dest_global, Storage& storage) {

		T data[VT];
		if(VT > 1) {
			DeviceGlobalToSharedDefault<NT, VT>(count, data_global, tid,
				storage.data, identity);
			DeviceSharedToThread<VT>(storage.data, tid, data);
		} else 
			DeviceGlobalToRegDefault<NT, VT>(count, data_global, tid, data, 
				identity);

		// Reduce within each thread for thread totals.
		T x;
		#pragma unroll
		for(int i = 0; i < VT; ++i)
			x = i ? op(x, data[i]) : data[i];

		// Reduce thread totals across the tile.
		T total;
		x = S::Scan(tid, x, storage.scanStorage, &total, MgpuScanTypeExc,
			identity, op);

		// Add the scan as carry-in and scan into shared memory.
		T localScan[VT];
		x = op(x, start);

		#pragma unroll
		for(int i = 0; i < VT; ++i) {
			if(MgpuScanTypeExc == Type) 
				localScan[i] = x;
			x = op(x, data[i]);
			if(MgpuScanTypeInc == Type)
				localScan[i] = x;
		}

		if(VT > 1) {
			// Transpose into shared memory and store to dest_global.
			DeviceThreadToShared<VT>(localScan, tid, storage.data);
			DeviceSharedToGlobal<NT, VT>(count, storage.data, tid, dest_global);
		} else
			DeviceRegToGlobal<NT, VT>(count, localScan, tid, dest_global);

		// Return the reduction of all elements in this tile.
		return total;
	}
};

// KernelScanParallel.
// Scan all inputs in a single CTA.
template<typename Tuning, MgpuScanType Type, typename DataIt, typename OutputIt,
	typename T, typename Op>
MGPU_LAUNCH_BOUNDS void KernelScanParallel(DataIt data_global, int count,
	T identity, Op op, T* total_global, OutputIt dest_global) {

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	
	typedef CTATileScan<NT, VT, T, Op> TileScan;
	union Shared {
		typename TileScan::Storage tileScanStorage;
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	
	// Scan from data_global into dest_global.
	T total = TileScan::DeviceScanTile<Type>(data_global, count, tid, identity, 
		op, identity, dest_global, shared.tileScanStorage);

	if(!tid && total_global)
		*total_global = total;
}

template<typename Tuning, MgpuScanType Type, typename DataIt, typename OutputIt,
	typename T, typename Op>
MGPU_LAUNCH_BOUNDS void KernelScanDownsweep(DataIt data_global, int count, 
	const T* reduction_global, T identity, Op op, OutputIt dest_global) {

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;

	typedef CTATileScan<NT, VT, T, Op> TileScan;
	__shared__ typename TileScan::Storage tileScanStorage;

	int tid = threadIdx.x;
	int block = blockIdx.x;
	int gid = NV * block;
	int count2 = min(NV, count - gid);

	// Load the reduction of the previous tiles.
	T start = reduction_global[block];

	// Scan from data_global into dest_global.
	TileScan::DeviceScanTile<Type>(data_global + gid, count2, tid, 
		identity, op, start, dest_global + gid, tileScanStorage);
}

////////////////////////////////////////////////////////////////////////////////

template<MgpuScanType Type, typename DataIt, typename T, typename Op,
	typename DestIt>
MGPU_HOST void Scan(DataIt data_global, int count, T identity, Op op,
	T* reduce_global, T* reduce_host, DestIt dest_global, 
	CudaContext& context) {
		
	MGPU_MEM(T) totalDevice;
	if(reduce_host && !reduce_global) {
		totalDevice = context.Malloc<T>(1);
		reduce_global = totalDevice->get();
	}

	if(count <= 256) {
		typedef LaunchBoxVT<256, 1> Tuning;
		KernelScanParallel<Tuning, Type><<<1, 256, 0, context.Stream()>>>(
			data_global, count, identity, op, reduce_global, dest_global);
		MGPU_SYNC_CHECK("KernelScanParallel");

	} else if(count <= 768) {
		typedef LaunchBoxVT<256, 3> Tuning;
		KernelScanParallel<Tuning, Type><<<1, 256, 0, context.Stream()>>>(
			data_global, count, identity, op, reduce_global, dest_global);
		MGPU_SYNC_CHECK("KernelScanParallel");

	} else if (count <= 512 * 5) {
		typedef LaunchBoxVT<512, 5> Tuning;
		KernelScanParallel<Tuning, Type><<<1, 512, 0, context.Stream()>>>(
			data_global, count, identity, op, reduce_global, dest_global);
		MGPU_SYNC_CHECK("KernelScanParallel");

	} else {
		typedef LaunchBoxVT<
			128, (sizeof(T) > 4) ? 7 : 15, 0,
			128, 7, 0,
			128, 7, 0
		> Tuning;
		int2 launch = Tuning::GetLaunchParams(context);
		int NV = launch.x * launch.y;
		int numBlocks = MGPU_DIV_UP(count, NV);
		MGPU_MEM(T) reduceDevice = context.Malloc<T>(numBlocks + 1);

		// Reduce tiles into reduceDevice.
		KernelReduce<Tuning><<<numBlocks, launch.x, 0, context.Stream()>>>(
			data_global, count, identity, op, reduceDevice->get());
		MGPU_SYNC_CHECK("KernelReduce");

		// Recurse to scan the reductions.
		Scan<MgpuScanTypeExc>(reduceDevice->get(), numBlocks, identity, op,
			 reduce_global, (T*)0, reduceDevice->get(), context);

		// Add scanned reductions back into output and scan.
		KernelScanDownsweep<Tuning, Type>
			<<<numBlocks, launch.x, 0, context.Stream()>>>(data_global, count,
			reduceDevice->get(), identity, op, dest_global);
		MGPU_SYNC_CHECK("KernelScanDownsweep");
	}
	if(reduce_host)
		copyDtoH(reduce_host, reduce_global, 1);
}

template<typename InputIt, typename T>
MGPU_HOST void ScanExc(InputIt data_global, int count, T* total,
	CudaContext& context) {

	Scan<MgpuScanTypeExc>(data_global, count, (T)0, mgpu::plus<T>(), (T*)0, 
		total, data_global,context);
}

template<typename InputIt>
MGPU_HOST void ScanExc(InputIt data_global, int count, CudaContext& context) {
	typedef typename std::iterator_traits<InputIt>::value_type T;
	Scan<MgpuScanTypeExc>(data_global, count, (T)0, mgpu::plus<T>(), (T*)0,
		(T*)0, data_global, context);
}

} // namespace mgpu
