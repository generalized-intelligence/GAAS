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
#include <nvbio/basic/numbers.h>
#include <cmath>
#include <limits>

namespace nvbio {

///
/// A generic small vector class with the dimension set at compile-time
///
template <typename T, uint32 DIM>
struct StaticVectorBase
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const T& operator[] (const uint32 i) const { return data[i]; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
          T& operator[] (const uint32 i)       { return data[i]; }

    T data[DIM];
};

///
/// A generic small vector class with the dimension set at compile-time
///
template <typename T>
struct StaticVectorBase<T,2>
{
    typedef typename vector_type<T,2>::type    base_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StaticVectorBase<T,2>() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StaticVectorBase<T,2>(const base_type v) : data(v) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const T& operator[] (const uint32 i) const { return (&data.x)[i]; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
          T& operator[] (const uint32 i)       { return (&data.x)[i]; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    operator base_type() const { return data; }

    base_type data;
};

///
/// A generic small vector class with the dimension set at compile-time
///
template <typename T>
struct StaticVectorBase<T,3>
{
    typedef typename vector_type<T,3>::type    base_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StaticVectorBase<T,3>() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StaticVectorBase<T,3>(const base_type v) : data(v) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const T& operator[] (const uint32 i) const { return (&data.x)[i]; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
          T& operator[] (const uint32 i)       { return (&data.x)[i]; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    operator base_type() const { return data; }

    base_type data;
};

///
/// A generic small vector class with the dimension set at compile-time
///
template <typename T>
struct StaticVectorBase<T,4>
{
    typedef typename vector_type<T,4>::type    base_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StaticVectorBase<T,4>() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StaticVectorBase<T,4>(const base_type v) : data(v) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const T& operator[] (const uint32 i) const { return (&data.x)[i]; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
          T& operator[] (const uint32 i)       { return (&data.x)[i]; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    operator base_type() const { return data; }

    base_type data;
};

///
/// A generic small vector class with the dimension set at compile-time
///
template <typename T, uint32 DIM>
struct StaticVector : public StaticVectorBase<T,DIM>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StaticVector() {}

    //NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    //StaticVector(const T* v);

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    explicit StaticVector(const T v);

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    StaticVector<T,DIM>& operator= (const StaticVectorBase<T,DIM>& op);
};

template <typename T,uint32 DIM_T> struct vector_traits< StaticVectorBase<T,DIM_T> > { typedef T value_type; const static uint32 DIM = DIM_T; };

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> operator+ (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>& operator+= (StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> operator- (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>& operator-= (StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> operator* (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>& operator*= (StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> operator/ (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM>& operator/= (StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> min(const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
StaticVector<T,DIM> max(const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool any(const StaticVector<T,DIM>& op);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool all(const StaticVector<T,DIM>& op);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator== (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

template <typename T, uint32 DIM>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator!= (const StaticVector<T,DIM>& op1, const StaticVector<T,DIM>& op2);

} // namespace nvbio

#include <nvbio/basic/static_vector_inl.h>
