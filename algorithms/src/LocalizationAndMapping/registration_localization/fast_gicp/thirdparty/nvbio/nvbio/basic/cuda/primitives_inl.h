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

#include <nvbio/basic/numbers.h>

namespace nvbio {
namespace cuda {

// make sure a given buffer is big enough
//
template <typename VectorType>
void alloc_temp_storage(VectorType& vec, const uint64 size)
{
    if (vec.size() < size)
    {
        try
        {
            vec.clear();
            vec.resize( size );
        }
        catch (...)
        {
            log_error(stderr,"alloc_temp_storage() : allocation failed! (%llu entries / %llu bytes)\n", size, size * sizeof(typename VectorType::value_type));
            throw;
        }
    }
}

// any kernel
//
template <typename PredicateIterator>
__global__
void any_kernel(
    const uint32            n,
    const PredicateIterator pred,
          uint32*           r)
{
    const uint32 i = threadIdx.x + blockIdx.x * blockDim.x;

    const bool p_i = (i < n ? pred[i] : false);
    const bool p = __syncthreads_or( p_i );

    // TODO: this could be made faster by using persistent blocks, and early-exiting a
    // block if this condition is true
    if (p)
        *r = 1u;
}

// all kernel
//
template <typename PredicateIterator>
__global__
void all_kernel(
    const uint32            n,
    const PredicateIterator pred,
          uint32*           r)
{
    const uint32 i = threadIdx.x + blockIdx.x * blockDim.x;

    const bool p_i = (i < n ? pred[i] : true);
    const bool p = __syncthreads_and( p_i );

    // TODO: this could be made faster by using persistent blocks, and early-exiting a
    // block if this condition is true
    if (p == false)
        *r = 0u;
}

// return true if any item in the range [0,n) evaluates to true
//
template <typename PredicateIterator>
bool any(
    const uint32            n,
    const PredicateIterator pred)
{
    const uint32 block_dim = 256;
    const uint32 n_blocks = util::divide_ri( n, block_dim );

    thrust::device_vector<uint32> r( 1u, 0u );

    any_kernel<<<n_blocks,block_dim>>>( n, pred, nvbio::plain_view( r ) );
    return r[0] != 0u;
}

// return true if all items in the range [0,n) evaluate to true
//
template <typename PredicateIterator>
bool all(
    const uint32            n,
    const PredicateIterator pred)
{
    const uint32 block_dim = 256;
    const uint32 n_blocks = util::divide_ri( n, block_dim );

    thrust::device_vector<uint32> r( 1u, 1u );

    all_kernel<<<n_blocks,block_dim>>>( n, pred, nvbio::plain_view( r ) );
    return r[0] != 0u;
}

// a pseudo-iterator to evaluate the predicate (it1[i] <= it2[i]) for arbitrary iterator pairs
//
template <typename Iterator1, typename Iterator2>
struct is_sorted_iterator
{
    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    is_sorted_iterator(const Iterator1 _it1, const Iterator2 _it2) : it1( _it1 ), it2( _it2 ) {}

    // dereference operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator[] (const uint32 i) const { return it1[i] <= it2[i]; }

    const Iterator1 it1;
    const Iterator2 it2;
};

// a pseudo-iterator to evaluate the predicate (hd[i] || (it1[i] <= it2[i])) for arbitrary iterator pairs
//
template <typename Iterator1, typename Iterator2, typename Headflags>
struct is_segment_sorted_iterator
{
    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    is_segment_sorted_iterator(const Iterator1 _it1, const Iterator2 _it2, const Headflags _hd) : it1( _it1 ), it2( _it2 ), hd(_hd) {}

    // dereference operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator[] (const uint32 i) const { return hd[i] || (it1[i] <= it2[i]); }

    const Iterator1 it1;
    const Iterator2 it2;
    const Headflags hd;
};

// return true if the items in the range [0,n) are sorted
//
template <typename Iterator>
bool is_sorted(
    const uint32    n,
    const Iterator  values)
{
    return all( n-1, is_sorted_iterator<Iterator,Iterator>( values, values+1 ) );
}

// return true if the items in the range [0,n) are sorted by segment, where
// the beginning of each segment is identified by a set head flag
//
template <typename Iterator, typename Headflags>
bool is_segment_sorted(
    const uint32            n,
    const Iterator          values,
    const Headflags         flags)
{
    return all( n-1, is_segment_sorted_iterator<Iterator,Iterator,Headflags>( values, values+1, flags+1 ) );
}

// device-wide reduce
//
// \param n                    number of items to reduce
// \param d_in                 a device iterator
// \param op                   the binary reduction operator
// \param d_temp_storage       some temporary storage
//
template <typename InputIterator, typename BinaryOp>
typename std::iterator_traits<InputIterator>::value_type reduce(
    const uint32                  n,
    InputIterator                 d_in,
    BinaryOp                      op,
    thrust::device_vector<uint8>& d_temp_storage)
{
    typedef typename std::iterator_traits<InputIterator>::value_type value_type;

    thrust::device_vector<value_type> d_out(1);

    size_t temp_bytes = 0;

    cub::DeviceReduce::Reduce(
        (void*)NULL, temp_bytes,
        d_in,
        d_out.begin(),
        int(n),
        op );

    temp_bytes = nvbio::max( uint64(temp_bytes), uint64(16) );
    alloc_temp_storage( d_temp_storage, temp_bytes );

    cub::DeviceReduce::Reduce(
        (void*)nvbio::plain_view( d_temp_storage ), temp_bytes,
        d_in,
        d_out.begin(),
        int(n),
        op );

    return d_out[0];
}

// device-wide inclusive scan
//
// \param n                    number of items to reduce
// \param d_in                 a device input iterator
// \param d_out                a device output iterator
// \param op                   the binary reduction operator
// \param d_temp_storage       some temporary storage
//
template <typename InputIterator, typename OutputIterator, typename BinaryOp>
void inclusive_scan(
    const uint32                  n,
    InputIterator                 d_in,
    OutputIterator                d_out,
    BinaryOp                      op,
    thrust::device_vector<uint8>& d_temp_storage)
{
    size_t temp_bytes = 0;

    cub::DeviceScan::InclusiveScan(
        (void*)NULL, temp_bytes,
        d_in,
        d_out,
        op,
        int(n) );

    temp_bytes = nvbio::max( uint64(temp_bytes), uint64(16) );
    alloc_temp_storage( d_temp_storage, temp_bytes );

    cub::DeviceScan::InclusiveScan(
        (void*)nvbio::plain_view( d_temp_storage ), temp_bytes,
        d_in,
        d_out,
        op,
        int(n) );
}

// device-wide exclusive scan
//
// \param n                    number of items to reduce
// \param d_in                 a device input iterator
// \param d_out                a device output iterator
// \param op                   the binary reduction operator
// \param identity             the identity element
// \param d_temp_storage       some temporary storage
//
template <typename InputIterator, typename OutputIterator, typename BinaryOp, typename Identity>
void exclusive_scan(
    const uint32                  n,
    InputIterator                 d_in,
    OutputIterator                d_out,
    BinaryOp                      op,
    Identity                      identity,
    thrust::device_vector<uint8>& d_temp_storage)
{
    size_t temp_bytes = 0;

    cub::DeviceScan::ExclusiveScan(
        (void*)NULL, temp_bytes,
        d_in,
        d_out,
        op,
        identity,
        int(n) );

    temp_bytes = nvbio::max( uint64(temp_bytes), uint64(16) );
    alloc_temp_storage( d_temp_storage, temp_bytes );

    cub::DeviceScan::ExclusiveScan(
        (void*)nvbio::plain_view( d_temp_storage ), temp_bytes,
        d_in,
        d_out,
        op,
        identity,
        int(n) );
}

// device-wide copy of flagged items
//
// \param n                    number of input items
// \param d_in                 a device input iterator
// \param d_flags              a device flags iterator
// \param d_out                a device output iterator
// \param d_temp_storage       some temporary storage
//
// \return                     the number of copied items
//
template <typename InputIterator, typename FlagsIterator, typename OutputIterator>
uint32 copy_flagged(
    const uint32                  n,
    InputIterator                 d_in,
    FlagsIterator                 d_flags,
    OutputIterator                d_out,
    thrust::device_vector<uint8>& d_temp_storage)
{
    size_t                         temp_bytes = 0;
    thrust::device_vector<int>     d_num_selected(1);

    cub::DeviceSelect::Flagged(
        (void*)NULL, temp_bytes,
        d_in,
        d_flags,
        d_out,
        nvbio::plain_view( d_num_selected ),
        int(n) );

    temp_bytes = nvbio::max( uint64(temp_bytes), uint64(16) );
    alloc_temp_storage( d_temp_storage, temp_bytes );

    cub::DeviceSelect::Flagged(
        (void*)nvbio::plain_view( d_temp_storage ), temp_bytes,
        d_in,
        d_flags,
        d_out,
        nvbio::plain_view( d_num_selected ),
        int(n) );

    return uint32( d_num_selected[0] );
};

// device-wide copy of predicated items
//
// \param n                    number of input items
// \param d_in                 a device input iterator
// \param d_out                a device output iterator
// \param pred                 a unary predicate functor
// \param d_temp_storage       some temporary storage
//
// \return                     the number of copied items
//
template <typename InputIterator, typename OutputIterator, typename Predicate>
uint32 copy_if(
    const uint32                  n,
    InputIterator                 d_in,
    OutputIterator                d_out,
    const Predicate               pred,
    thrust::device_vector<uint8>& d_temp_storage)
{
    size_t                         temp_bytes = 0;
    thrust::device_vector<int>     d_num_selected(1);

    cub::DeviceSelect::If(
        (void*)NULL, temp_bytes,
        d_in,
        d_out,
        nvbio::plain_view( d_num_selected ),
        int(n),
        pred );

    temp_bytes = nvbio::max( uint64(temp_bytes), uint64(16) );
    alloc_temp_storage( d_temp_storage, temp_bytes );

    cub::DeviceSelect::If(
        (void*)nvbio::plain_view( d_temp_storage ), temp_bytes,
        d_in,
        d_out,
        nvbio::plain_view( d_num_selected ),
        int(n),
        pred );

    return uint32( d_num_selected[0] );
};

// device-wide run-length encode
//
// \param n                    number of input items
// \param d_in                 a device input iterator
// \param d_out                a device output iterator
// \param d_counts             a device output count iterator
// \param d_temp_storage       some temporary storage
//
// \return                     the number of copied items
//
template <typename InputIterator, typename OutputIterator, typename CountIterator>
uint32 runlength_encode(
    const uint32                  n,
    InputIterator                 d_in,
    OutputIterator                d_out,
    CountIterator                 d_counts,
    thrust::device_vector<uint8>& d_temp_storage)
{
    size_t                         temp_bytes = 0;
    thrust::device_vector<int>     d_num_selected(1);

    cub::DeviceReduce::RunLengthEncode(
        (void*)NULL, temp_bytes,
        d_in,
        d_out,
        d_counts,
        nvbio::plain_view( d_num_selected ),
        int(n) );

    temp_bytes = nvbio::max( uint64(temp_bytes), uint64(16) );
    alloc_temp_storage( d_temp_storage, temp_bytes );

    cub::DeviceReduce::RunLengthEncode(
        (void*)nvbio::plain_view( d_temp_storage ), temp_bytes,
        d_in,
        d_out,
        d_counts,
        nvbio::plain_view( d_num_selected ),
        int(n) );

    return uint32( d_num_selected[0] );
};


// device-wide run-length encode
//
// \param n                    number of input items
// \param d_keys_in            a device input iterator
// \param d_values_in          a device input iterator
// \param d_keys_out           a device output iterator
// \param d_values_out         a device output iterator
// \param reduction_op         a reduction operator
// \param d_temp_storage       some temporary storage
//
// \return                     the number of copied items
//
template <typename KeyIterator, typename ValueIterator, typename OutputKeyIterator, typename OutputValueIterator, typename ReductionOp>
uint32 reduce_by_key(
    const uint32                  n,
    KeyIterator                   d_keys_in,
    ValueIterator                 d_values_in,
    OutputKeyIterator             d_keys_out,
    OutputValueIterator           d_values_out,
    ReductionOp                   reduction_op,
    thrust::device_vector<uint8>& d_temp_storage)
{
    size_t                         temp_bytes = 0;
    thrust::device_vector<int>     d_num_selected(1);

    cub::DeviceReduce::ReduceByKey(
        (void*)NULL, temp_bytes,
        d_keys_in,
        d_keys_out,
        d_values_in,
        d_values_out,
        nvbio::plain_view( d_num_selected ),
        reduction_op,
        int(n) );

    temp_bytes = nvbio::max( uint64(temp_bytes), uint64(16) );
    alloc_temp_storage( d_temp_storage, temp_bytes );

    cub::DeviceReduce::ReduceByKey(
        (void*)nvbio::plain_view( d_temp_storage ), temp_bytes,
        d_keys_in,
        d_keys_out,
        d_values_in,
        d_values_out,
        nvbio::plain_view( d_num_selected ),
        reduction_op,
        int(n) );

    return uint32( d_num_selected[0] );
}

} // namespace cuda
} // namespace nvbio
