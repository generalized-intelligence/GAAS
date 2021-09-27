/******************************************************************************
 * Copyright (c) 2011, Duane Merrill.  All rights reserved.
 * Copyright (c) 2011-2014, NVIDIA CORPORATION.  All rights reserved.
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
 *
 ******************************************************************************/

/**
 * \file
 * Simple binary operator functor types
 */

/******************************************************************************
 * Simple functor operators
 ******************************************************************************/

#pragma once

#include "../util_macro.cuh"
#include "../util_type.cuh"
#include "../util_namespace.cuh"

/// Optional outer namespace(s)
CUB_NS_PREFIX

/// CUB namespace
namespace cub {


/**
 * \addtogroup UtilModule
 * @{
 */

/**
 * \brief Default equality functor
 */
struct Equality
{
    /// Boolean equality operator, returns <tt>(a == b)</tt>
    template <typename T>
    __host__ __device__ __forceinline__ bool operator()(const T &a, const T &b) const
    {
        return a == b;
    }
};


/**
 * \brief Default inequality functor
 */
struct Inequality
{
    /// Boolean inequality operator, returns <tt>(a != b)</tt>
    template <typename T>
    __host__ __device__ __forceinline__ bool operator()(const T &a, const T &b) const
    {
        return a != b;
    }
};


/**
 * \brief Inequality functor (wraps equality functor)
 */
template <typename EqualityOp>
struct InequalityWrapper
{
    /// Wrapped equality operator
    EqualityOp op;

    /// Constructor
    __host__ __device__ __forceinline__
    InequalityWrapper(EqualityOp op) : op(op) {}

    /// Boolean inequality operator, returns <tt>(a != b)</tt>
    template <typename T>
    __host__ __device__ __forceinline__ bool operator()(const T &a, const T &b) const
    {
        return !op(a, b);
    }
};


/**
 * \brief Default sum functor
 */
struct Sum
{
    /// Boolean sum operator, returns <tt>a + b</tt>
    template <typename T>
    __host__ __device__ __forceinline__ T operator()(const T &a, const T &b) const
    {
        return a + b;
    }
};


/**
 * \brief Default max functor
 */
struct Max
{
    /// Boolean max operator, returns <tt>(a > b) ? a : b</tt>
    template <typename T>
    __host__ __device__ __forceinline__ T operator()(const T &a, const T &b) const
    {
        return CUB_MAX(a, b);
    }
};


/**
 * \brief Arg max functor (keeps the value and offset of the first occurrence of the l item)
 */
struct ArgMax
{
    /// Boolean max operator, preferring the item having the smaller offset in case of ties
    template <typename T, typename Offset>
    __host__ __device__ __forceinline__ ItemOffsetPair<T, Offset> operator()(
        const ItemOffsetPair<T, Offset> &a,
        const ItemOffsetPair<T, Offset> &b) const
    {
        if (a.value == b.value)
            return (b.offset < a.offset) ? b : a;

        return (b.value > a.value) ? b : a;
    }
};


/**
 * \brief Default min functor
 */
struct Min
{
    /// Boolean min operator, returns <tt>(a < b) ? a : b</tt>
    template <typename T>
    __host__ __device__ __forceinline__ T operator()(const T &a, const T &b) const
    {
        return CUB_MIN(a, b);
    }
};


/**
 * \brief Arg min functor (keeps the value and offset of the first occurrence of the smallest item)
 */
struct ArgMin
{
    /// Boolean min operator, preferring the item having the smaller offset in case of ties
    template <typename T, typename Offset>
    __host__ __device__ __forceinline__ ItemOffsetPair<T, Offset> operator()(
        const ItemOffsetPair<T, Offset> &a,
        const ItemOffsetPair<T, Offset> &b) const
    {
        if (a.value == b.value)
            return (b.offset < a.offset) ? b : a;

        return (b.value < a.value) ? b : a;
    }
};


/**
 * \brief Default cast functor
 */
template <typename B>
struct Cast
{
    /// Boolean max operator, returns <tt>(a > b) ? a : b</tt>
    template <typename A>
    __host__ __device__ __forceinline__ B operator()(const A &a) const
    {
        return (B) a;
    }
};



/** @} */       // end group UtilModule


}               // CUB namespace
CUB_NS_POSTFIX  // Optional outer namespace(s)
