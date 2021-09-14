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
#include <nvbio/basic/console.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/algorithms.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/timer.h>
#include <thrust/reduce.h>
#include <thrust/scan.h>
#include <thrust/copy.h>
#include <thrust/sort.h>
#include <thrust/binary_search.h>
#include <thrust/merge.h>
#include <thrust/iterator/constant_iterator.h>

#if defined(__CUDACC__)
#include <nvbio/basic/cuda/primitives.h>
#include <nvbio/basic/cuda/sort.h>
#endif

#if defined (_OPENMP)
#include <omp.h>
#endif

/// \page primitives_page Parallel Primitives
///
/// This module provides a set of system-wide parallel primitives.
/// The backend system is specified at compile-time by a \ref SystemTags "system_tag".
/// All temporary storage is allocated within a single nvbio::vector
/// passed by the user, which can be safely reused across function calls.
///
/// - nvbio::any()
/// - nvbio::all()
/// - nvbio::is_sorted()
/// - nvbio::is_segment_sorted()
/// - nvbio::for_each()
/// - nvbio::for_each_enactor
/// - nvbio::transform()
/// - nvbio::reduce()
/// - nvbio::inclusive_scan()
/// - nvbio::exclusive_scan()
/// - nvbio::copy_flagged()
/// - nvbio::copy_if()
/// - nvbio::runlength_encode()
/// - nvbio::upper_bound()
/// - nvbio::lower_bound()
/// - nvbio::radix_sort()
/// - nvbio::merge_by_key()
///
///\par
/// The complete list can be found in the \ref Primitives module documentation.
///

namespace nvbio {

///@addtogroup Basic
///@{

///@defgroup Primitives Parallel Primitives
/// This module provides a set of convenience wrappers to invoke system-wide
/// CUB's parallel primitives without worrying about the memory management.
/// All temporary storage is in fact allocated within a single thrust::system_vector
/// passed by the user, which can be safely reused across function calls.
///@{

/// return true if any item in the range [0,n) evaluates to true
///
template <typename system_tag, typename PredicateIterator>
bool any(
    const uint32            n,
    const PredicateIterator pred);

/// return true if all items in the range [0,n) evaluate to true
///
template <typename system_tag, typename PredicateIterator>
bool all(
    const uint32            n,
    const PredicateIterator pred);

/// return true if the items in the range [0,n) are sorted
///
template <typename system_tag, typename Iterator>
bool is_sorted(
    const uint32            n,
    const Iterator          values);

/// return true if the items in the range [0,n) are sorted by segment, where
/// the beginning of each segment is identified by a set head flag
///
template <typename system_tag, typename Iterator, typename Headflags>
bool is_segment_sorted(
    const uint32            n,
    const Iterator          values,
    const Headflags         flags);


/// invoke a functor for each element of the given sequence
///
template <typename system_tag, typename Iterator, typename Functor>
void for_each(
    const uint64            n,
    const Iterator          in,
          Functor           functor);

/// apply a functor to each element of the given sequence
///
template <typename system_tag, typename Iterator, typename Output, typename Functor>
void transform(
    const uint32            n,
    const Iterator          in,
    const Output            out,
    const Functor           functor);

/// apply a binary functor to each pair of elements of the given sequences
///
template <typename system_tag, typename Iterator1, typename Iterator2, typename Output, typename Functor>
void transform(
    const uint32            n,
    const Iterator1         in1,
    const Iterator2         in2,
    const Output            out,
    const Functor           functor);

/// system-wide reduce
///
/// \param n                    number of items to reduce
/// \param in                   a system iterator
/// \param op                   the binary reduction operator
/// \param temp_storage         some temporary storage
///
template <typename system_tag, typename InputIterator, typename BinaryOp>
typename std::iterator_traits<InputIterator>::value_type reduce(
    const uint32                        n,
    InputIterator                       in,
    BinaryOp                            op,
    nvbio::vector<system_tag,uint8>&    temp_storage);

/// system-wide inclusive scan
///
/// \param n                    number of items to reduce
/// \param in                   a system input iterator
/// \param out                  a system output iterator
/// \param op                   the binary reduction operator
/// \param temp_storage         some temporary storage
///
template <typename system_tag, typename InputIterator, typename OutputIterator, typename BinaryOp>
void inclusive_scan(
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    BinaryOp                            op,
    nvbio::vector<system_tag,uint8>&    temp_storage);

/// system-wide exclusive scan
///
/// \param n                    number of items to reduce
/// \param in                   a system input iterator
/// \param out                  a system output iterator
/// \param op                   the binary reduction operator
/// \param identity             the identity element
/// \param temp_storage         some temporary storage
///
template <typename system_tag, typename InputIterator, typename OutputIterator, typename BinaryOp, typename Identity>
void exclusive_scan(
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    BinaryOp                            op,
    Identity                            identity,
    nvbio::vector<system_tag,uint8>&    temp_storage);

/// system-wide copy of flagged items
///
/// \param n                    number of input items
/// \param in                   a system input iterator
/// \param flags                a system flags iterator
/// \param out                  a system output iterator
/// \param temp_storage         some temporary storage
///
/// \return                     the number of copied items
///
template <typename system_tag, typename InputIterator, typename FlagsIterator, typename OutputIterator>
uint32 copy_flagged(
    const uint32                        n,
    InputIterator                       in,
    FlagsIterator                       flags,
    OutputIterator                      out,
    nvbio::vector<system_tag,uint8>&    temp_storage);

/// system-wide copy of predicated items
///
/// \param n                    number of input items
/// \param in                   a system input iterator
/// \param out                  a system output iterator
/// \param pred                 a unary predicate functor
/// \param temp_storage         some temporary storage
///
/// \return                     the number of copied items
///
template <typename system_tag, typename InputIterator, typename OutputIterator, typename Predicate>
uint32 copy_if(
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    const Predicate                     pred,
    nvbio::vector<system_tag,uint8>&    temp_storage);

/// system-wide run-length encode
///
/// \param n                    number of input items
/// \param in                   a system input iterator
/// \param out                  a system output iterator
/// \param counts               a system output count iterator
/// \param temp_storage         some temporary storage
///
/// \return                     the number of copied items
///
template <typename system_tag, typename InputIterator, typename OutputIterator, typename CountIterator>
uint32 runlength_encode(
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    CountIterator                       counts,
    nvbio::vector<system_tag,uint8>&    temp_storage);

/// system-wide run-length encode
///
/// \param n                    number of input items
/// \param keys_in              a system input iterator
/// \param values_in            a system input iterator
/// \param keys_out             a system output iterator
/// \param values_out           a system output iterator
/// \param reduction_op         a reduction operator
/// \param temp_storage         some temporary storage
///
/// \return                     the number of copied items
///
template <typename system_tag, typename KeyIterator, typename ValueIterator, typename OutputKeyIterator, typename OutputValueIterator, typename ReductionOp>
uint32 reduce_by_key(
    const uint32                        n,
    KeyIterator                         keys_in,
    ValueIterator                       values_in,
    OutputKeyIterator                   keys_out,
    OutputValueIterator                 values_out,
    ReductionOp                         reduction_op,
    nvbio::vector<system_tag,uint8>&    temp_storage);

/// system-wide lower_bound
///
/// \param n                    number of input items
/// \param values               a system input iterator of values to be searched
/// \param n_keys               number of sorted keys
/// \param keys                 a system input iterator of sorted keys
/// \param indices              a system output iterator
///
template <typename system_tag, typename KeyIterator, typename ValueIterator, typename OutputIterator>
void lower_bound(
    const uint32                        n,
    ValueIterator                       values,
    const uint32                        n_keys,
    KeyIterator                         keys,
    OutputIterator                      indices);

/// system-wide upper_bound
///
/// \param n                    number of input items
/// \param values               a system input iterator of values to be searched
/// \param n_keys               number of sorted keys
/// \param keys                 a system input iterator of sorted keys
/// \param indices              a system output iterator
///
template <typename system_tag, typename KeyIterator, typename ValueIterator, typename OutputIterator>
void upper_bound(
    const uint32                        n,
    ValueIterator                       values,
    const uint32                        n_keys,
    KeyIterator                         keys,
    OutputIterator                      indices);

/// system-wide radix-sort
///
/// \param n                    number of input items
/// \param keys                 a system input iterator of keys to be sorted
/// \param temp_storage         some temporary storage
///
template <typename system_tag, typename KeyIterator>
void radix_sort(
    const uint32                        n,
    KeyIterator                         keys,
    nvbio::vector<system_tag,uint8>&    temp_storage);

/// system-wide radix-sort by key
///
/// \param n                    number of input items
/// \param keys                 a system input iterator of keys to be sorted
/// \param values               a system input iterator of values to be sorted
/// \param temp_storage         some temporary storage
///
template <typename system_tag, typename KeyIterator, typename ValueIterator>
void radix_sort(
    const uint32                        n,
    KeyIterator                         keys,
    ValueIterator                       values,
    nvbio::vector<system_tag,uint8>&    temp_storage);

/// merge two sequences by key
///
/// \param A_len                number of input items in the first sequence
/// \param B_len                number of input items in the second sequence
/// \param A_keys               a system input iterator of keys to be merged from the first sequence
/// \param B_keys               a system input iterator of keys to be merged from the second sequence
/// \param A_values             a system input iterator of values to be merged from the first sequence
/// \param B_values             a system input iterator of values to be merged from the second sequence
/// \param C_keys               a system output iterator to the final merged keys
/// \param C_values             a system output iterator of the final merged values
/// \param temp_storage         some temporary storage
///
template <
    typename system_tag,
    typename key_iterator1,
    typename key_iterator2,
    typename value_iterator1,
    typename value_iterator2,
    typename key_output,
    typename value_output>
void merge_by_key(
    const uint32                        A_len,
    const uint32                        B_len,
    const key_iterator1                 A_keys,
    const key_iterator2                 B_keys,
    const value_iterator1               A_values,
    const value_iterator2               B_values,
          key_output                    C_keys,
          value_output                  C_values,
    nvbio::vector<system_tag,uint8>&    temp_storage);

/// A stateful for_each enactor class that optimizes the kernel launches at run-time,
/// using per-launch statistics
///
template <typename system_tag>
struct for_each_enactor
{
    /// enact the for_each
    ///
    template <typename Iterator, typename Functor>
    void operator () (
        const uint64    n,
        const Iterator  in,
              Functor   functor)
    {
        for_each<system_tag>( n, in, functor );
    }

    /// enact the for_each
    ///
    template <typename Functor>
    void operator () (
        const uint64    n,
              Functor   functor)
    {
        for_each<system_tag>( n, thrust::make_counting_iterator<uint64>(0), functor );
    }
};

/// A stateful for_each enactor class that optimizes the kernel launches at run-time,
/// using per-launch statistics
///
template <>
struct for_each_enactor<device_tag>
{
    /// constructor
    ///
    for_each_enactor() :
        m_blocks_lo( 0 ),
        m_blocks_hi( 0 ),
        m_speed_lo( 0.0f ),
        m_speed_hi( 0.0f ) {}

    /// enact the for_each
    ///
    template <typename Iterator, typename Functor>
    void operator () (
        const uint64            n,
        const Iterator          in,
              Functor           functor);

    /// enact the for_each
    ///
    template <typename Functor>
    void operator () (
        const uint64    n,
              Functor   functor)
    {
        this->operator()( n, thrust::make_counting_iterator<uint64>(0), functor );
    }

  private:

    /// ask the optimizer how many blocks we should try using next
    ///
    template <typename KernelFunction>
    uint32 suggested_blocks(KernelFunction kernel, const uint32 cta_size) const;

    /// update the optimizer's internal state with the latest speed data-point
    ///
    void update(const uint32 n_blocks, const float speed);

    uint32  m_blocks_lo;
    uint32  m_blocks_hi;
    float   m_speed_lo;
    float   m_speed_hi;
};

///@} // end of the Primitives group
///@} // end of the Basic group

} // namespace nvbio

#include <nvbio/basic/primitives_inl.h>
