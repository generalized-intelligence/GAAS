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
#include "../kernels/search.cuh"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// KernelMerge

template<typename Tuning, bool HasValues, bool LoadExtended, typename KeysIt1,
	typename KeysIt2, typename KeysIt3, typename ValsIt1, typename ValsIt2,
	typename ValsIt3, typename Comp>
MGPU_LAUNCH_BOUNDS void KernelMerge(KeysIt1 aKeys_global, ValsIt1 aVals_global,
	int aCount, KeysIt2 bKeys_global, ValsIt2 bVals_global, int bCount,
	const int* mp_global, int coop, KeysIt3 keys_global, ValsIt3 vals_global,
	Comp comp) {

	typedef MGPU_LAUNCH_PARAMS Params;
	typedef typename std::iterator_traits<KeysIt1>::value_type KeyType;
	typedef typename std::iterator_traits<ValsIt1>::value_type ValType;

	const int NT = Params::NT;
	const int VT = Params::VT;
	const int NV = NT * VT;
	union Shared {
		KeyType keys[NT * (VT + 1)];
		int indices[NV];
	};
	__shared__ Shared shared;

	int tid = threadIdx.x;
	int block = blockIdx.x;

	int4 range = ComputeMergeRange(aCount, bCount, block, coop, NT * VT, 
		mp_global);

	DeviceMerge<NT, VT, HasValues, LoadExtended>(aKeys_global, aVals_global, 
		aCount, bKeys_global, bVals_global, bCount, tid, block, range, 
		shared.keys, shared.indices, keys_global, vals_global, comp);
}

////////////////////////////////////////////////////////////////////////////////
// MergeKeys

template<typename KeysIt1, typename KeysIt2, typename KeysIt3, typename Comp>
MGPU_HOST void MergeKeys(KeysIt1 aKeys_global, int aCount, KeysIt2 bKeys_global,
	int bCount, KeysIt3 keys_global, Comp comp, CudaContext& context) {
	
	typedef typename std::iterator_traits<KeysIt1>::value_type T;
	typedef LaunchBoxVT<
		128, 23, 0,
		128, 11, 0,
		128, (sizeof(T) > 4) ? 7 : 11, 0
	> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);

	const int NV = launch.x * launch.y;
	MGPU_MEM(int) partitionsDevice = MergePathPartitions<MgpuBoundsLower>(
		aKeys_global, aCount, bKeys_global, bCount, NV, 0, comp, context);

	int numBlocks = MGPU_DIV_UP(aCount + bCount, NV);
	KernelMerge<Tuning, false, true>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(aKeys_global, 
		(const int*)0, aCount, bKeys_global, (const int*)0, bCount, 
		partitionsDevice->get(), 0, keys_global, (int*)0, comp);
	MGPU_SYNC_CHECK("KernelMerge");
}
template<typename KeysIt1, typename KeysIt2, typename KeysIt3>
MGPU_HOST void MergeKeys(KeysIt1 aKeys_global, int aCount, KeysIt2 bKeys_global,
	int bCount, KeysIt3 keys_global, CudaContext& context) {

	typedef mgpu::less<typename std::iterator_traits<KeysIt1>::value_type> Comp;
	return MergeKeys(aKeys_global, aCount, bKeys_global, bCount, keys_global,
		Comp(), context);
}

////////////////////////////////////////////////////////////////////////////////
// MergePairs

template<typename KeysIt1, typename KeysIt2, typename KeysIt3, typename ValsIt1,
	typename ValsIt2, typename ValsIt3, typename Comp>
MGPU_HOST void MergePairs(KeysIt1 aKeys_global, ValsIt1 aVals_global, 
	int aCount, KeysIt2 bKeys_global, ValsIt2 bVals_global, int bCount,
	KeysIt3 keys_global, ValsIt3 vals_global, Comp comp, CudaContext& context) {

	typedef typename std::iterator_traits<KeysIt1>::value_type T;
	typedef LaunchBoxVT<
		128, 11, 0,
		128, 11, 0,
		128, (sizeof(T) > 4) ? 7 : 11, 0
	> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);

	const int NV = launch.x * launch.y;
	MGPU_MEM(int) partitionsDevice = MergePathPartitions<MgpuBoundsLower>(
		aKeys_global, aCount, bKeys_global, bCount, NV, 0, comp, context);

	int numBlocks = MGPU_DIV_UP(aCount + bCount, NV);
	KernelMerge<Tuning, true, false>
		<<<numBlocks, launch.x, 0, context.Stream()>>>(aKeys_global,
		aVals_global, aCount, bKeys_global, bVals_global, bCount, 
		partitionsDevice->get(), 0, keys_global, vals_global, comp);
	MGPU_SYNC_CHECK("KernelMerge");
}
template<typename KeysIt1, typename KeysIt2, typename KeysIt3, typename ValsIt1,
	typename ValsIt2, typename ValsIt3>
MGPU_HOST void MergePairs(KeysIt1 aKeys_global, ValsIt1 aVals_global, 
	int aCount, KeysIt2 bKeys_global, ValsIt2 bVals_global, int bCount,
	KeysIt3 keys_global, ValsIt3 vals_global, CudaContext& context) {

	typedef mgpu::less<typename std::iterator_traits<KeysIt1>::value_type> Comp;
	return MergePairs(aKeys_global, aVals_global, aCount, bKeys_global, 
		bVals_global, bCount, keys_global, vals_global, Comp(), context);
}

} // namespace mgpu
