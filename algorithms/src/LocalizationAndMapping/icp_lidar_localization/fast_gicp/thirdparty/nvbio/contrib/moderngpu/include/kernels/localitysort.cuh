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
#include "../kernels/mergesort.cuh"
#include "../kernels/segmentedsort.cuh"

namespace mgpu {

template<typename T, typename Comp>
MGPU_HOST void LocalitySortKeys(T* data_global, int count, CudaContext& context,
	Comp comp, bool verbose) {

	const int NT = 128;
	const int VT = 11;
	typedef LaunchBoxVT<NT, VT> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;

	int numBlocks = MGPU_DIV_UP(count, NV);
	int numPasses = FindLog2(numBlocks, true);

	SegSortSupport support;
	MGPU_MEM(byte) mem = AllocSegSortBuffers(count, NV, support, false,
		context);
	
	MGPU_MEM(T) destDevice = context.Malloc<T>(count);
	T* source = data_global;
	T* dest = destDevice->get(); 
	
	KernelBlocksort<Tuning, false>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(source, (const int*)0,
		count, (1 & numPasses) ? dest : source, (int*)0, comp);
	MGPU_SYNC_CHECK("KernelBlocksort");

	if(1 & numPasses) std::swap(source, dest);

	SegSortPasses<Tuning, false, false>(support, source, (int*)0, count, 
		numBlocks, numPasses, dest, (int*)0, comp, context, verbose);
} 
template<typename T>
MGPU_HOST void LocalitySortKeys(T* data_global, int count, CudaContext& context,
	bool verbose) {
	LocalitySortKeys(data_global, count, context, mgpu::less<T>(), verbose);
}

template<typename KeyType, typename ValType, typename Comp>
MGPU_HOST void LocalitySortPairs(KeyType* keys_global, ValType* values_global,
	int count, CudaContext& context, Comp comp, bool verbose) {

	const int NT = 128;
	const int VT = 7;
	typedef LaunchBoxVT<NT, VT> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;

	int numBlocks = MGPU_DIV_UP(count, NV);
	int numPasses = FindLog2(numBlocks, true);

	SegSortSupport support;
	MGPU_MEM(byte) mem = AllocSegSortBuffers(count, NV, support, false,
		context);
	
	MGPU_MEM(KeyType) keysDestDevice = context.Malloc<KeyType>(count);
	MGPU_MEM(ValType) valsDestDevice = context.Malloc<ValType>(count);

	KeyType* keysSource = keys_global;
	KeyType* keysDest = keysDestDevice->get();
	ValType* valsSource = values_global;
	ValType* valsDest = valsDestDevice->get();

	KernelBlocksort<Tuning, true><<<numBlocks, launch.x, 0, context.Stream()>>>(
		keysSource, valsSource, count, (1 & numPasses) ? keysDest : keysSource,
		(1 & numPasses) ? valsDest : valsSource, comp);
	MGPU_SYNC_CHECK("KernelBlocksort");

	if(1 & numPasses) {
		std::swap(keysSource, keysDest);
		std::swap(valsSource, valsDest);
	}

	SegSortPasses<Tuning, false, true>(support, keysSource, valsSource, count,
		numBlocks, numPasses, keysDest, valsDest, comp, context, verbose);
} 
template<typename KeyType, typename ValType>
MGPU_HOST void LocalitySortPairs(KeyType* keys_global, ValType* values_global,
	int count, CudaContext& context, bool verbose) {
	LocalitySortPairs(keys_global, values_global, count, context,
		mgpu::less<KeyType>(), verbose);
}

} // namespace mgpu
