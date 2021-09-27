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

#include "../util/mgpucontext.h"

namespace mgpu {
	
#if __CUDA_ARCH__ >= 350
	#define MGPU_SM_TAG Sm35
#elif __CUDA_ARCH__ >= 300
	#define MGPU_SM_TAG Sm30
#elif __CUDA_ARCH__ >= 200
	#define MGPU_SM_TAG Sm20
#else
	#define MGPU_SM_TAG Sm20
#endif

#define MGPU_LAUNCH_PARAMS typename Tuning::MGPU_SM_TAG
#define MGPU_LAUNCH_BOUNDS __global__ \
	__launch_bounds__(Tuning::MGPU_SM_TAG::NT, Tuning::MGPU_SM_TAG::OCC)

// Returns (NT, VT) from the sm version.
template<typename Derived>
struct LaunchBoxRuntime {
	static int2 GetLaunchParams(CudaContext& context) {
		return GetLaunchParams(context.PTXVersion());
	}

	static int2 GetLaunchParams(int sm) {
		if(sm >= 350) 
			return make_int2(Derived::Sm35::NT, Derived::Sm35::VT);
		else if(sm >= 300) 
			return make_int2(Derived::Sm30::NT, Derived::Sm30::VT);
		else
			return make_int2(Derived::Sm20::NT, Derived::Sm20::VT);
	}
};

// General LaunchBox for any param types.
template<
	typename Sm20_, 
	typename Sm30_ = Sm20_,
	typename Sm35_ = Sm30_>
struct LaunchBox : LaunchBoxRuntime<LaunchBox<Sm20_, Sm30_, Sm35_> > {
	typedef Sm20_ Sm20;
	typedef Sm30_ Sm30;
	typedef Sm35_ Sm35;	
};

// LaunchBox over (NT, VT, NumBlocks)
template<int NT_, int VT_, int OCC_>
struct LaunchParamsVT {
	enum { NT = NT_, VT = VT_, OCC = OCC_ };
};
template<
	int NT_SM20,           int VT_SM20,           int OCC_SM20 = 0,
	int NT_SM30 = NT_SM20, int VT_SM30 = VT_SM20, int OCC_SM30 = OCC_SM20,
	int NT_SM35 = NT_SM30, int VT_SM35 = VT_SM30, int OCC_SM35 = OCC_SM30>
struct LaunchBoxVT : LaunchBox<
	LaunchParamsVT<NT_SM20, VT_SM20, OCC_SM20>,
	LaunchParamsVT<NT_SM30, VT_SM30, OCC_SM30>,
	LaunchParamsVT<NT_SM35, VT_SM35, OCC_SM35> > { };

} // namespace mgpu
