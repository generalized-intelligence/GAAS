/*
 * nvbio
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the NVIDIA CORPORATION nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <nvbio/basic/types.h>
#ifdef __CUDACC__
#include <nvbio/basic/cuda/simd_functions.h>
#endif
#include <cmath>
#include <limits>

namespace nvbio {

///
/// A 4-way uint8 SIMD type
///
struct simd4u8
{
    struct base_rep_tag {};

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    simd4u8() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    explicit simd4u8(const uint32 op, const base_rep_tag) { m = op; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    explicit simd4u8(const uint4 v);

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    explicit simd4u8(const uint8 v);

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    simd4u8(const uint8 v1, const uint8 v2, const uint8 v3, const uint8 v4);

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    simd4u8& operator= (const uint4 v);

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    simd4u8& operator= (const uchar4 v);

    uint32 m;
};

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool any(const simd4u8 op) { return op.m != 0; }

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator~(const simd4u8 op) { return simd4u8( ~op.m ); }

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator== (const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator!= (const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator>= (const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator> (const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator<= (const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator< (const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator+ (const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8& operator+= (simd4u8& op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator- (const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8& operator-= (simd4u8& op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 max(const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 min(const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 and_op(const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 or_op(const simd4u8 op1, const simd4u8 op2);

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 ternary_op(const simd4u8 mask, const simd4u8 op1, const simd4u8 op2);

template <uint32 I>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint8 get(const simd4u8 op);

} // namespace nvbio

#include <nvbio/basic/simd_inl.h>
