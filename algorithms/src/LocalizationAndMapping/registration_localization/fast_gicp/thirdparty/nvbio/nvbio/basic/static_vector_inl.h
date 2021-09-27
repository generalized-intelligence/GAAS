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

namespace nvbio {

#if 0
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>::StaticVector(const T* v)
{
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        this->operator[](d) = v[d];
}
#endif

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>::StaticVector(const T v)
{
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        this->operator[](d) = v;
}

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>& StaticVector<T,DIM>::operator= (const StaticVectorBase<T,DIM>& op)
{
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        this->operator[](d) = op[d];

    return *this;
}

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> operator+ (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    StaticVector<T,DIM> r;
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        r[d] = op1[d] + op2[d];
    return r;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>& operator+= (StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        op1[d] = op1[d] + op2[d];
    return op1;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> operator- (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    StaticVector<T,DIM> r;
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        r[d] = op1[d] - op2[d];
    return r;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>& operator-= (StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        op1[d] = op1[d] - op2[d];
    return op1;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> operator* (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    StaticVector<T,DIM> r;
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        r[d] = op1[d] - op2[d];
    return r;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>& operator*= (StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        op1[d] = op1[d] * op2[d];
    return op1;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> operator/ (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    StaticVector<T,DIM> r;
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        r[d] = op1[d] / op2[d];
    return r;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>& operator/= (StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        op1[d] = op1[d] / op2[d];
    return op1;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> min(const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    StaticVector<T,DIM> r;
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        r[d] = nvbio::min( op1[d], op2[d] );
    return r;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> max(const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    StaticVector<T,DIM> r;
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        r[d] = nvbio::max( op1[d], op2[d] );
    return r;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool any(const StaticVector<T,DIM>& op)
{
    bool r = false;
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        r = r || (op[d] != 0);
    return r;
}
template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool all(const StaticVector<T,DIM>& op)
{
    bool r = true;
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        r = r && (op[d] != 0);
    return r;
}

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator== (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    bool r = true;
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        r = r && (op1[d] == op2[d]);
    return r;
}

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator!= (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2)
{
    bool r = false;
    #pragma unroll
    for (uint32 d = 0; d < DIM; ++d)
        r = r || (op1[d] != op2[d]);
    return r;
}

} // namespace nvbio
