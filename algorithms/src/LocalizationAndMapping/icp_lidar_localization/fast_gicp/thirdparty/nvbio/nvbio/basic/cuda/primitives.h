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
#include <nvbio/basic/console.h>
#include <nvbio/basic/thrust_view.h>
#include <thrust/device_vector.h>
#include <cub/cub.cuh>

/// \page cuda_primitives_page Parallel Primitives
///
/// This module provides a set of convenience wrappers to invoke device-wide
/// CUB's parallel primitives without worrying about the memory management.
/// All temporary storage is in fact allocated within a single thrust::device_vector
/// passed by the user, which can be safely reused across function calls.
///
/// - cuda::any()
/// - cuda::all()
/// - cuda::is_sorted()
/// - cuda::is_segment_sorted()
/// - cuda::reduce()
/// - cuda::inclusive_scan()
/// - cuda::exclusive_scan()
/// - cuda::copy_flagged()
/// - cuda::copy_if()
/// - cuda::runlength_encode()
///

namespace nvbio {
namespace cuda {

///@addtogroup Basic
///@{

///@addtogroup CUDA
///@{

///@defgroup CUDAPrimitives Parallel Primitives
/// This module provides a set of convenience wrappers to invoke device-wide
/// CUB's parallel primitives without worrying about the memory management.
/// All temporary storage is in fact allocated within a single thrust::device_vector
/// passed by the user, which can be safely reused across function calls.
///@{

/// make sure a given buffer is as big as size;
/// <b>note:</b> upon reallocations, the contents of the buffer are invalidated
///
template <typename VectorType>
void alloc_temp_storage(VectorType& vec, const uint64 size);

/// return true if any item in the range [0,n) evaluates to true
///
template <typename PredicateIterator>
bool any(
    const uint32            n,
    const PredicateIterator pred);

/// return true if all items in the range [0,n) evaluate to true
///
template <typename PredicateIterator>
bool all(
    const uint32            n,
    const PredicateIterator pred);

/// return true if the items in the range [0,n) are sorted
///
template <typename Iterator>
bool is_sorted(
    const uint32            n,
    const Iterator          values);

/// return true if the items in the range [0,n) are sorted by segment, where
/// the beginning of each segment is identified by a set head flag
///
template <typename Iterator, typename Headflags>
bool is_segment_sorted(
    const uint32            n,
    const Iterator          values,
    const Headflags         flags);

/// device-wide reduce
///
/// \param n                    number of items to reduce
/// \param d_in                 a device iterator
/// \param op                   the binary reduction operator
/// \param d_temp_storage       some temporary storage
///
template <typename InputIterator, typename BinaryOp>
typename std::iterator_traits<InputIterator>::value_type reduce(
    const uint32                  n,
    InputIterator                 d_in,
    BinaryOp                      op,
    thrust::device_vector<uint8>& d_temp_storage);

/// device-wide inclusive scan
///
/// \param n                    number of items to reduce
/// \param d_in                 a device input iterator
/// \param d_out                a device output iterator
/// \param op                   the binary reduction operator
/// \param d_temp_storage       some temporary storage
///
template <typename InputIterator, typename OutputIterator, typename BinaryOp>
void inclusive_scan(
    const uint32                  n,
    InputIterator                 d_in,
    OutputIterator                d_out,
    BinaryOp                      op,
    thrust::device_vector<uint8>& d_temp_storage);

/// device-wide exclusive scan
///
/// \param n                    number of items to reduce
/// \param d_in                 a device input iterator
/// \param d_out                a device output iterator
/// \param op                   the binary reduction operator
/// \param identity             the identity element
/// \param d_temp_storage       some temporary storage
///
template <typename InputIterator, typename OutputIterator, typename BinaryOp, typename Identity>
void exclusive_scan(
    const uint32                  n,
    InputIterator                 d_in,
    OutputIterator                d_out,
    BinaryOp                      op,
    Identity                      identity,
    thrust::device_vector<uint8>& d_temp_storage);

/// device-wide copy of flagged items
///
/// \param n                    number of input items
/// \param d_in                 a device input iterator
/// \param d_flags              a device flags iterator
/// \param d_out                a device output iterator
/// \param d_temp_storage       some temporary storage
///
/// \return                     the number of copied items
///
template <typename InputIterator, typename FlagsIterator, typename OutputIterator>
uint32 copy_flagged(
    const uint32                  n,
    InputIterator                 d_in,
    FlagsIterator                 d_flags,
    OutputIterator                d_out,
    thrust::device_vector<uint8>& d_temp_storage);

/// device-wide copy of predicated items
///
/// \param n                    number of input items
/// \param d_in                 a device input iterator
/// \param d_out                a device output iterator
/// \param pred                 a unary predicate functor
/// \param d_temp_storage       some temporary storage
///
/// \return                     the number of copied items
///
template <typename InputIterator, typename OutputIterator, typename Predicate>
uint32 copy_if(
    const uint32                  n,
    InputIterator                 d_in,
    OutputIterator                d_out,
    const Predicate               pred,
    thrust::device_vector<uint8>& d_temp_storage);

/// device-wide run-length encode
///
/// \param n                    number of input items
/// \param d_in                 a device input iterator
/// \param d_out                a device output iterator
/// \param d_counts             a device output count iterator
/// \param d_temp_storage       some temporary storage
///
/// \return                     the number of copied items
///
template <typename InputIterator, typename OutputIterator, typename CountIterator>
uint32 runlength_encode(
    const uint32                  n,
    InputIterator                 d_in,
    OutputIterator                d_out,
    CountIterator                 d_counts,
    thrust::device_vector<uint8>& d_temp_storage);

/// device-wide run-length encode
///
/// \param n                    number of input items
/// \param d_keys_in            a device input iterator
/// \param d_values_in          a device input iterator
/// \param d_keys_out           a device output iterator
/// \param d_values_out         a device output iterator
/// \param reduction_op         a reduction operator
/// \param d_temp_storage       some temporary storage
///
/// \return                     the number of copied items
///
template <typename KeyIterator, typename ValueIterator, typename OutputKeyIterator, typename OutputValueIterator, typename ReductionOp>
uint32 reduce_by_key(
    const uint32                  n,
    KeyIterator                   d_keys_in,
    ValueIterator                 d_values_in,
    OutputKeyIterator             d_keys_out,
    OutputValueIterator           d_values_out,
    ReductionOp                   reduction_op,
    thrust::device_vector<uint8>& d_temp_storage);

///@} // end of the CUDAPrimitives group
///@} // end of the CUDA group
///@} // end of the Basic group

} // namespace cuda
} // namespace nvbio

#include <nvbio/basic/cuda/primitives_inl.h>
