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
#include "../device/ctaloadbalance.cuh"
#include "../kernels/search.cuh"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// KernelLoadBalance

template<typename Tuning, typename InputIt>
MGPU_LAUNCH_BOUNDS void KernelLoadBalance(int aCount, InputIt b_global,
	int bCount, const int* mp_global, int* indices_global) {

	typedef MGPU_LAUNCH_PARAMS Params;
	const int NT = Params::NT;
	const int VT = Params::VT;
	__shared__ int indices_shared[NT * (VT + 1)];
	
	int tid = threadIdx.x;
	int block = blockIdx.x;
	int4 range = CTALoadBalance<NT, VT>(aCount, b_global, bCount, block, tid,
		mp_global, indices_shared, false);
	aCount = range.y - range.x;

	DeviceSharedToGlobal<NT, VT>(aCount, indices_shared, tid, 
		indices_global + range.x, false);
}

////////////////////////////////////////////////////////////////////////////////
// LoadBalanceSearch

template<typename InputIt>
MGPU_HOST void LoadBalanceSearch(int aCount, InputIt b_global, int bCount,
	int* indices_global, CudaContext& context) {

	const int NT = 128;
	const int VT = 7;
	typedef LaunchBoxVT<NT, VT> Tuning;
	int2 launch = Tuning::GetLaunchParams(context);
	const int NV = launch.x * launch.y;
	  
	MGPU_MEM(int) partitionsDevice = MergePathPartitions<MgpuBoundsUpper>(
		mgpu::counting_iterator<int>(0), aCount, b_global, bCount, NV, 0,
		mgpu::less<int>(), context);

	int numBlocks = MGPU_DIV_UP(aCount + bCount, NV);
	KernelLoadBalance<Tuning><<<numBlocks, launch.x, 0, context.Stream()>>>(
		aCount, b_global, bCount, partitionsDevice->get(), indices_global);
	MGPU_SYNC_CHECK("KernelLoadBalance");
}

} // namespace mgpu
