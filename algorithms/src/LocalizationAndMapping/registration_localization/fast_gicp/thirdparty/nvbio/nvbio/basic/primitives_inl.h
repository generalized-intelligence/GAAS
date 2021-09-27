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

// return true if any item in the range [0,n) evaluates to true
//
template <typename PredicateIterator>
bool any(
    const host_tag          tag,
    const uint32            n,
    const PredicateIterator pred)
{
    return thrust::reduce(
        pred,
        pred + n,
        false,
        thrust::logical_or<bool>() );
}

// return true if all items in the range [0,n) evaluate to true
//
template <typename PredicateIterator>
bool all(
    const host_tag          tag,
    const uint32            n,
    const PredicateIterator pred)
{
    return thrust::reduce(
        pred,
        pred + n,
        true,
        thrust::logical_and<bool>() );
}

#if defined(__CUDACC__)

// return true if any item in the range [0,n) evaluates to true
//
template <typename PredicateIterator>
bool any(
    const device_tag        tag,
    const uint32            n,
    const PredicateIterator pred)
{
    return cuda::any( n, pred );
}

// return true if any item in the range [0,n) evaluates to true
//
template <typename PredicateIterator>
bool all(
    const device_tag        tag,
    const uint32            n,
    const PredicateIterator pred)
{
    return cuda::all( n, pred );
}

#endif

// return true if any item in the range [0,n) evaluates to true
//
template <typename system_tag, typename PredicateIterator>
bool any(
    const uint32            n,
    const PredicateIterator pred)
{
    return any( system_tag(), n, pred );
}

// return true if all items in the range [0,n) evaluate to true
//
template <typename system_tag, typename PredicateIterator>
bool all(
    const uint32            n,
    const PredicateIterator pred)
{
    return all( system_tag(), n, pred );
}

// a pseudo-iterator to evaluate the predicate (it1[i] <= it2[i]) for arbitrary iterator pairs
//
template <typename Iterator1, typename Iterator2>
struct is_sorted_iterator
{
    typedef bool                                                        value_type;
    typedef value_type&                                                 reference;
    typedef value_type                                                  const_reference;
    typedef value_type*                                                 pointer;
    typedef typename std::iterator_traits<Iterator1>::difference_type   difference_type;
    typedef typename std::iterator_traits<Iterator1>::iterator_category iterator_category;

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    is_sorted_iterator(const Iterator1 _it1, const Iterator2 _it2) : it1( _it1 ), it2( _it2 ) {}

    // dereference operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator[] (const uint64 i) const { return it1[i] <= it2[i]; }

    // dereference operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator* () const { return it1[0] <= it2[0]; }

    // dereference operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    is_sorted_iterator& operator++ () { ++it1; ++it2; return *this; }

    Iterator1 it1;
    Iterator2 it2;
};

// operator+
template <typename T1, typename T2>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
is_sorted_iterator<T1,T2> operator+ (const is_sorted_iterator<T1,T2> it, const int64 i)
{
    return is_sorted_iterator<T1,T2>( it.it1 + i, it.it2 + i );
}
// operator-
template <typename T1, typename T2>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
int64 operator- (const is_sorted_iterator<T1,T2> it1, const is_sorted_iterator<T1,T2> it2)
{
    return it1.it1 - it2.it1;
}
// operator!=
template <typename T1, typename T2>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator!= (const is_sorted_iterator<T1,T2> it1, const is_sorted_iterator<T1,T2> it2)
{
    return it1.it1 != it2.it1;
}
// operator==
template <typename T1, typename T2>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator== (const is_sorted_iterator<T1,T2> it1, const is_sorted_iterator<T1,T2> it2)
{
    return it1.it1 == it2.it1;
}

// a pseudo-iterator to evaluate the predicate (hd[i] || (it1[i] <= it2[i])) for arbitrary iterator pairs
//
template <typename Iterator1, typename Iterator2, typename Headflags>
struct is_segment_sorted_iterator
{
    typedef bool                                                        value_type;
    typedef value_type&                                                 reference;
    typedef value_type                                                  const_reference;
    typedef value_type*                                                 pointer;
    typedef typename std::iterator_traits<Iterator1>::difference_type   difference_type;
    typedef typename std::iterator_traits<Iterator1>::iterator_category iterator_category;

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    is_segment_sorted_iterator(const Iterator1 _it1, const Iterator2 _it2, const Headflags _hd) : it1( _it1 ), it2( _it2 ), hd(_hd) {}

    // dereference operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator[] (const uint64 i) const { return hd[i] || (it1[i] <= it2[i]); }

    // dereference operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator* () const { return hd[0] || (it1[0] <= it2[0]); }

    // dereference operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    is_segment_sorted_iterator& operator++ () { ++it1; ++it2; ++hd; return *this; }

    Iterator1 it1;
    Iterator2 it2;
    Headflags hd;
};

// operator+
template <typename T1, typename T2, typename H>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
is_segment_sorted_iterator<T1,T2,H> operator+ (const is_segment_sorted_iterator<T1,T2,H> it, const int64 i)
{
    return is_segment_sorted_iterator<T1,T2,H>( it.it1 + i, it.it2 + i, it.hd + i );
}
// operator-
template <typename T1, typename T2, typename H>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
int64 operator- (const is_segment_sorted_iterator<T1,T2,H> it1, const is_segment_sorted_iterator<T1,T2,H> it2)
{
    return it1.it1 - it2.it1;
}
// operator!=
template <typename T1, typename T2, typename H>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator!= (const is_segment_sorted_iterator<T1,T2,H> it1, const is_segment_sorted_iterator<T1,T2,H> it2)
{
    return it1.it1 != it2.it1;
}
// operator==
template <typename T1, typename T2, typename H>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator== (const is_segment_sorted_iterator<T1,T2,H> it1, const is_segment_sorted_iterator<T1,T2,H> it2)
{
    return it1.it1 == it2.it1;
}

// return true if the items in the range [0,n) are sorted
//
template <typename system_tag, typename Iterator>
bool is_sorted(
    const uint32    n,
    const Iterator  values)
{
    return all<system_tag>( n-1, is_sorted_iterator<Iterator,Iterator>( values, values+1 ) );
}

// return true if the items in the range [0,n) are sorted by segment, where
// the beginning of each segment is identified by a set head flag
//
template <typename system_tag, typename Iterator, typename Headflags>
bool is_segment_sorted(
    const uint32            n,
    const Iterator          values,
    const Headflags         flags)
{
    return all<system_tag>( n-1, is_segment_sorted_iterator<Iterator,Iterator,Headflags>( values, values+1, flags+1 ) );
}

// invoke a functor for each element of the given sequence
//
template <typename Iterator, typename Functor>
void for_each(
    const host_tag          tag,
    const uint64            n,
    const Iterator          in,
          Functor           functor)
{
    #if defined(_OPENMP)
    #pragma omp parallel for if (n >= 256)
    #endif
    for (int64 i = 0; i < int64(n); ++i)
        functor( in[i] );
}

// invoke a functor for each element of the given sequence
//
template <typename Iterator, typename Functor>
void for_each(
    const device_tag        tag,
    const uint64            n,
    const Iterator          in,
          Functor           functor)
{
    thrust::for_each( in, in + n, functor );
}

// invoke a functor for each element of the given sequence
//
template <typename system_tag, typename Iterator, typename Functor>
void for_each(
    const uint64            n,
    const Iterator          in,
          Functor           functor)
{
    return for_each( system_tag(), n, in, functor );
}

// apply a functor to each element of the given sequence
//
template <typename Iterator, typename Output, typename Functor>
void transform(
    const device_tag        tag,
    const uint64            n,
    const Iterator          in,
    const Output            out,
    const Functor           functor)
{
    thrust::transform( in, in + n, out, functor );
}

// apply a functor to each element of the given sequence
//
template <typename Iterator, typename Output, typename Functor>
void transform(
    const host_tag          tag,
    const uint32            n,
    const Iterator          in,
    const Output            out,
    const Functor           functor)
{
    #if defined(_OPENMP)
    #pragma omp parallel for if (n >= 256)
    #endif
    for (int64 i = 0; i < int64(n); ++i)
        out[i] = functor( in[i] );
}

// apply a binary functor to each pair of elements of the given sequences
//
template <typename Iterator1, typename Iterator2, typename Output, typename Functor>
void transform(
    const device_tag        tag,
    const uint32            n,
    const Iterator1         in1,
    const Iterator2         in2,
    const Output            out,
    const Functor           functor)
{
    thrust::transform( in1, in1 + n, in2, out, functor );
}

// apply a binary functor to each pair of elements of the given sequences
//
template <typename Iterator1, typename Iterator2, typename Output, typename Functor>
void transform(
    const host_tag          tag,
    const uint32            n,
    const Iterator1         in1,
    const Iterator2         in2,
    const Output            out,
    const Functor           functor)
{
    #if defined(_OPENMP)
    #pragma omp parallel for if (n >= 256)
    #endif
    for (int64 i = 0; i < int64(n); ++i)
        out[i] = functor( in1[i], in2[i] );
}

// apply a functor to each element of the given sequence
//
template <typename system_tag, typename Iterator, typename Output, typename Functor>
void transform(
    const uint32            n,
    const Iterator          in,
    const Output            out,
    const Functor           functor)
{
    transform( system_tag(), n, in, out, functor );
}

// apply a binary functor to each pair of elements of the given sequences
//
template <typename system_tag, typename Iterator1, typename Iterator2, typename Output, typename Functor>
void transform(
    const uint32            n,
    const Iterator1         in1,
    const Iterator2         in2,
    const Output            out,
    const Functor           functor)
{
    transform( system_tag(), n, in1, in2, out, functor );
}

// host-wide reduce
//
// \param n                    number of items to reduce
// \param in                   a system iterator
// \param op                   the binary reduction operator
// \param temp_storage         some temporary storage
//
template <typename InputIterator, typename BinaryOp>
typename std::iterator_traits<InputIterator>::value_type reduce(
    host_tag                            tag,
    const uint32                        n,
    InputIterator                       in,
    BinaryOp                            op,
    nvbio::vector<host_tag,uint8>&      temp_storage)
{
    return thrust::reduce( in, in + n, 0u, op );
}

// host-wide inclusive scan
//
// \param n                    number of items to reduce
// \param in                 a device input iterator
// \param out                a device output iterator
// \param op                   the binary reduction operator
// \param temp_storage       some temporary storage
//
template <typename InputIterator, typename OutputIterator, typename BinaryOp>
void inclusive_scan(
    host_tag                            tag,
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    BinaryOp                            op,
    nvbio::vector<host_tag,uint8>&      temp_storage)
{
    thrust::inclusive_scan(
        in,
        in + n,
        out,
        op );
}

// host-wide exclusive scan
//
// \param n                    number of items to reduce
// \param in                 a device input iterator
// \param out                a device output iterator
// \param op                   the binary reduction operator
// \param identity             the identity element
// \param temp_storage       some temporary storage
//
template <typename InputIterator, typename OutputIterator, typename BinaryOp, typename Identity>
void exclusive_scan(
    host_tag                            tag,
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    BinaryOp                            op,
    Identity                            identity,
    nvbio::vector<host_tag,uint8>&      temp_storage)
{
    thrust::exclusive_scan(
        in,
        in + n,
        out,
        identity,
        op );
}

#if defined(__CUDACC__)

// system-wide reduce
//
// \param n                    number of items to reduce
// \param in                   a system iterator
// \param op                   the binary reduction operator
// \param temp_storage         some temporary storage
//
template <typename InputIterator, typename BinaryOp>
typename std::iterator_traits<InputIterator>::value_type reduce(
    device_tag                          tag,
    const uint32                        n,
    InputIterator                       in,
    BinaryOp                            op,
    nvbio::vector<device_tag,uint8>&    temp_storage)
{
    return cuda::reduce( n, in, op, temp_storage );
}

// device-wide inclusive scan
//
// \param n                    number of items to reduce
// \param in                 a device input iterator
// \param out                a device output iterator
// \param op                   the binary reduction operator
// \param temp_storage       some temporary storage
//
template <typename InputIterator, typename OutputIterator, typename BinaryOp>
void inclusive_scan(
    device_tag                          tag,
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    BinaryOp                            op,
    nvbio::vector<device_tag,uint8>&    temp_storage)
{
    cuda::inclusive_scan( n, in, out, op, temp_storage );
}

// device-wide exclusive scan
//
// \param n                    number of items to reduce
// \param in                 a device input iterator
// \param out                a device output iterator
// \param op                   the binary reduction operator
// \param identity             the identity element
// \param temp_storage       some temporary storage
//
template <typename InputIterator, typename OutputIterator, typename BinaryOp, typename Identity>
void exclusive_scan(
    device_tag                          tag,
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    BinaryOp                            op,
    Identity                            identity,
    nvbio::vector<device_tag,uint8>&    temp_storage)
{
    cuda::exclusive_scan( n, in, out, op, identity, temp_storage );
}

#endif

// system-wide reduce
//
// \param n                    number of items to reduce
// \param in                   a system iterator
// \param op                   the binary reduction operator
// \param temp_storage         some temporary storage
//
template <typename system_tag, typename InputIterator, typename BinaryOp>
typename std::iterator_traits<InputIterator>::value_type reduce(
    const uint32                        n,
    InputIterator                       in,
    BinaryOp                            op,
    nvbio::vector<system_tag,uint8>&    temp_storage)
{
    return reduce(
        system_tag(),
        n,
        in,
        op,
        temp_storage );
}

// device-wide inclusive scan
//
// \param n                    number of items to reduce
// \param in                 a device input iterator
// \param out                a device output iterator
// \param op                   the binary reduction operator
// \param temp_storage       some temporary storage
//
template <typename system_tag, typename InputIterator, typename OutputIterator, typename BinaryOp>
void inclusive_scan(
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    BinaryOp                            op,
    nvbio::vector<system_tag,uint8>&    temp_storage)
{
    inclusive_scan(
        system_tag(),
        n,
        in,
        out,
        op,
        temp_storage );
}

// device-wide exclusive scan
//
// \param n                    number of items to reduce
// \param in                 a device input iterator
// \param out                a device output iterator
// \param op                   the binary reduction operator
// \param identity             the identity element
// \param temp_storage       some temporary storage
//
template <typename system_tag, typename InputIterator, typename OutputIterator, typename BinaryOp, typename Identity>
void exclusive_scan(
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    BinaryOp                            op,
    Identity                            identity,
    nvbio::vector<system_tag,uint8>&    temp_storage)
{
    exclusive_scan(
        system_tag(),
        n,
        in,
        out,
        op,
        identity,
        temp_storage );
}

// host-wide copy of flagged items
//
// \param n                    number of input items
// \param in                    a input iterator
// \param flags                 a flags iterator
// \param out                   a output iterator
// \param temp_storage          some temporary storage
//
// \return                     the number of copied items
//
template <typename InputIterator, typename FlagsIterator, typename OutputIterator>
uint32 copy_flagged(
    const host_tag                  tag,
    const uint32                    n,
    InputIterator                   in,
    FlagsIterator                   flags,
    OutputIterator                  out,
    nvbio::vector<host_tag,uint8>&  temp_storage)
{
    return uint32( thrust::copy_if(
        in,
        in + n,
        flags,
        out,
        nvbio::is_true_functor<bool>() ) - out );
}

// host-wide copy of predicated items
//
// \param n                    number of input items
// \param in                   a input iterator
// \param flags                a flags iterator
// \param out                  a output iterator
// \param temp_storage         some temporary storage
//
// \return                     the number of copied items
//
template <typename InputIterator, typename OutputIterator, typename Predicate>
uint32 copy_if(
    const host_tag                      tag,
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    const Predicate                     pred,
    nvbio::vector<host_tag,uint8>&      temp_storage)
{
    return uint32( thrust::copy_if(
        in,
        in + n,
        out,
        pred ) - out );
}

// system-wide run-length encode
//
// \param n                     number of input items
// \param in                    a system input iterator
// \param out                   a system output iterator
// \param counts                a system output count iterator
// \param temp_storage          some temporary storage
//
// \return                     the number of copied items
//
template <typename InputIterator, typename OutputIterator, typename CountIterator>
uint32 runlength_encode(
    const host_tag                      tag,
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    CountIterator                       counts,
    nvbio::vector<host_tag,uint8>&      temp_storage)
{
    return uint32( thrust::reduce_by_key(
        in,
        in + n,
        thrust::make_constant_iterator<uint32>( 1u ),
        out,
        counts ).first - out );
};


// system-wide run-length encode
//
// \param n                     number of input items
// \param keys_in               a system input iterator
// \param values_in             a system input iterator
// \param keys_out              a system output iterator
// \param values_out            a system output iterator
// \param reduction_op          a reduction operator
// \param temp_storage          some temporary storage
//
// \return                      the number of copied items
//
template <typename KeyIterator, typename ValueIterator, typename OutputKeyIterator, typename OutputValueIterator, typename ReductionOp>
uint32 reduce_by_key(
    const host_tag                      tag,
    const uint32                        n,
    KeyIterator                         keys_in,
    ValueIterator                       values_in,
    OutputKeyIterator                   keys_out,
    OutputValueIterator                 values_out,
    ReductionOp                         reduction_op,
    nvbio::vector<host_tag,uint8>&      temp_storage)
{
    typedef typename std::iterator_traits<KeyIterator>::value_type key_type;

    return uint32( thrust::reduce_by_key(
        keys_in,
        keys_in + n,
        values_in,
        keys_out,
        values_out,
        nvbio::equal_functor<key_type>(),
        reduction_op ).first - keys_out );
}

#if defined(__CUDACC__)

// device-wide copy of flagged items
//
// \param n                    number of input items
// \param in                   a input iterator
// \param flags                a flags iterator
// \param out                  a output iterator
// \param temp_storage         some temporary storage
//
// \return                     the number of copied items
//
template <typename InputIterator, typename FlagsIterator, typename OutputIterator>
uint32 copy_flagged(
    const device_tag                    tag,
    const uint32                        n,
    InputIterator                       in,
    FlagsIterator                       flags,
    OutputIterator                      out,
    nvbio::vector<device_tag,uint8>&    temp_storage)
{
    return cuda::copy_flagged( n, in, flags, out, temp_storage );
}

// device-wide copy of predicated items
//
// \param n                    number of input items
// \param in                   a input iterator
// \param flags                a flags iterator
// \param out                  a output iterator
// \param temp_storage         some temporary storage
//
// \return                     the number of copied items
//
template <typename InputIterator, typename OutputIterator, typename Predicate>
uint32 copy_if(
    const device_tag                    tag,
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    const Predicate                     pred,
    nvbio::vector<device_tag,uint8>&    temp_storage)
{
    return cuda::copy_if( n, in, out, pred, temp_storage );
}

// system-wide run-length encode
//
// \param n                     number of input items
// \param in                    a device input iterator
// \param out                   a device output iterator
// \param counts                a device output count iterator
// \param temp_storage          some temporary storage
//
// \return                     the number of copied items
//
template <typename InputIterator, typename OutputIterator, typename CountIterator>
uint32 runlength_encode(
    const device_tag                    tag,
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    CountIterator                       counts,
    nvbio::vector<device_tag,uint8>&    temp_storage)
{
    return cuda::runlength_encode( n, in, out, counts, temp_storage );
};

// device-wide run-length encode
//
// \param n                     number of input items
// \param keys_in               a device input iterator
// \param values_in             a device input iterator
// \param keys_out              a device output iterator
// \param values_out            a device output iterator
// \param reduction_op          a reduction operator
// \param temp_storage          some temporary storage
//
// \return                      the number of copied items
//
template <typename KeyIterator, typename ValueIterator, typename OutputKeyIterator, typename OutputValueIterator, typename ReductionOp>
uint32 reduce_by_key(
    const device_tag                    tag,
    const uint32                        n,
    KeyIterator                         keys_in,
    ValueIterator                       values_in,
    OutputKeyIterator                   keys_out,
    OutputValueIterator                 values_out,
    ReductionOp                         reduction_op,
    nvbio::vector<device_tag,uint8>&    temp_storage)
{
    return cuda::reduce_by_key(
        n,
        keys_in,
        values_in,
        keys_out,
        values_out,
        reduction_op,
        temp_storage );
}

#endif

// system-wide copy of flagged items
//
// \param n                    number of input items
// \param in                 a device input iterator
// \param flags              a device flags iterator
// \param out                a device output iterator
// \param temp_storage       some temporary storage
//
// \return                     the number of copied items
//
template <typename system_tag, typename InputIterator, typename FlagsIterator, typename OutputIterator>
uint32 copy_flagged(
    const uint32                        n,
    InputIterator                       in,
    FlagsIterator                       flags,
    OutputIterator                      out,
    nvbio::vector<system_tag,uint8>&    temp_storage)
{
    return copy_flagged( system_tag(), n, in, flags, out, temp_storage );
};

// system-wide copy of predicated items
//
// \param n                    number of input items
// \param in                 a device input iterator
// \param out                a device output iterator
// \param pred                 a unary predicate functor
// \param temp_storage       some temporary storage
//
// \return                     the number of copied items
//
template <typename system_tag, typename InputIterator, typename OutputIterator, typename Predicate>
uint32 copy_if(
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    const Predicate                     pred,
    nvbio::vector<system_tag,uint8>&    temp_storage)
{
    return copy_if( system_tag(), n, in, out, pred, temp_storage );
};

// system-wide run-length encode
//
// \param n                     number of input items
// \param in                    a system input iterator
// \param out                   a system output iterator
// \param counts                a system output count iterator
// \param temp_storage          some temporary storage
//
// \return                     the number of copied items
//
template <typename system_tag, typename InputIterator, typename OutputIterator, typename CountIterator>
uint32 runlength_encode(
    const uint32                        n,
    InputIterator                       in,
    OutputIterator                      out,
    CountIterator                       counts,
    nvbio::vector<system_tag,uint8>&    temp_storage)
{
    return runlength_encode( system_tag(), n, in, out, counts, temp_storage );
};

// system-wide run-length encode
//
// \param n                     number of input items
// \param keys_in               a system input iterator
// \param values_in             a system input iterator
// \param keys_out              a system output iterator
// \param values_out            a system output iterator
// \param reduction_op          a reduction operator
// \param temp_storage          some temporary storage
//
// \return                      the number of copied items
//
template <typename system_tag, typename KeyIterator, typename ValueIterator, typename OutputKeyIterator, typename OutputValueIterator, typename ReductionOp>
uint32 reduce_by_key(
    const uint32                        n,
    KeyIterator                         keys_in,
    ValueIterator                       values_in,
    OutputKeyIterator                   keys_out,
    OutputValueIterator                 values_out,
    ReductionOp                         reduction_op,
    nvbio::vector<system_tag,uint8>&    temp_storage)
{
    return reduce_by_key(
        system_tag(),
        n,
        keys_in,
        values_in,
        keys_out,
        values_out,
        reduction_op,
        temp_storage );
}

// device-wide lower_bound
//
// \param n                    number of input items
// \param values               a system input iterator of values to be searched
// \param keys                 a system input iterator of sorted keys
// \param indices              a system output iterator
//
template <typename KeyIterator, typename ValueIterator, typename OutputIterator>
void lower_bound(
    const device_tag                    tag,
    const uint32                        n,
    ValueIterator                       values,
    const uint32                        n_keys,
    KeyIterator                         keys,
    OutputIterator                      indices)
{
    thrust::lower_bound(
        keys, keys + n_keys,
        values, values + n,
        indices );
}

// host-wide lower_bound
//
// \param n                    number of input items
// \param values               a system input iterator of values to be searched
// \param keys                 a system input iterator of sorted keys
// \param indices              a system output iterator
//
template <typename KeyIterator, typename ValueIterator, typename OutputIterator>
void lower_bound(
    const host_tag                      tag,
    const uint32                        n,
    ValueIterator                       values,
    const uint32                        n_keys,
    KeyIterator                         keys,
    OutputIterator                      indices)
{
    #pragma omp parallel for
    for (long i = 0; i < long(n); ++i)
        indices[i] = uint32( lower_bound( values[i], keys, n_keys ) - keys );
}

// system-wide lower_bound
//
// \param n                    number of input items
// \param values               a system input iterator of values to be searched
// \param keys                 a system input iterator of sorted keys
// \param indices              a system output iterator
//
template <typename system_tag, typename KeyIterator, typename ValueIterator, typename OutputIterator>
void lower_bound(
    const uint32                        n,
    ValueIterator                       values,
    const uint32                        n_keys,
    KeyIterator                         keys,
    OutputIterator                      indices)
{
    lower_bound(
        system_tag(),
        n,
        values,
        n_keys,
        keys,
        indices );
}

// device-wide upper_bound
//
// \param n                    number of input items
// \param values               a system input iterator of values to be searched
// \param keys                 a system input iterator of sorted keys
// \param indices              a system output iterator
//
template <typename KeyIterator, typename ValueIterator, typename OutputIterator>
void upper_bound(
    const device_tag                    tag,
    const uint32                        n,
    ValueIterator                       values,
    const uint32                        n_keys,
    KeyIterator                         keys,
    OutputIterator                      indices)
{
    thrust::upper_bound(
        keys, keys + n_keys,
        values, values + n,
        indices );
}

// host-wide upper_bound
//
// \param n                    number of input items
// \param values               a system input iterator of values to be searched
// \param keys                 a system input iterator of sorted keys
// \param indices              a system output iterator
//
template <typename KeyIterator, typename ValueIterator, typename OutputIterator>
void upper_bound(
    const host_tag                      tag,
    const uint32                        n,
    ValueIterator                       values,
    const uint32                        n_keys,
    KeyIterator                         keys,
    OutputIterator                      indices)
{
    #pragma omp parallel for
    for (long i = 0; i < long(n); ++i)
        indices[i] = uint32( upper_bound( values[i], keys, n_keys ) - keys );
}

// system-wide upper_bound
//
// \param n                    number of input items
// \param values               a system input iterator of values to be searched
// \param keys                 a system input iterator of sorted keys
// \param indices              a system output iterator
//
template <typename system_tag, typename KeyIterator, typename ValueIterator, typename OutputIterator>
void upper_bound(
    const uint32                        n,
    ValueIterator                       values,
    const uint32                        n_keys,
    KeyIterator                         keys,
    OutputIterator                      indices)
{
    upper_bound(
        system_tag(),
        n,
        values,
        n_keys,
        keys,
        indices );
}

#if defined(__CUDACC__)

// device-wide sort
//
// \param n                    number of input items
// \param keys                 a system input iterator of keys to be sorted
//
template <typename KeyIterator>
void radix_sort(
    const device_tag                    tag,
    const uint32                        n,
    KeyIterator                         keys,
    nvbio::vector<device_tag,uint8>&    temp_storage)
{
    typedef typename std::iterator_traits<KeyIterator>::value_type key_type;

    cuda::alloc_temp_storage( temp_storage, 2 * n * sizeof(key_type) );

    key_type* keys_ptr = reinterpret_cast<key_type*>( raw_pointer( temp_storage ) );

    thrust::device_ptr<key_type> keys_buf( keys_ptr );

    thrust::copy( keys, keys + n, keys_buf );

    cuda::SortBuffers<key_type*> sort_buffers;
    sort_buffers.keys[0] = keys_ptr;
    sort_buffers.keys[1] = keys_ptr + n;

    cuda::SortEnactor sort_enactor;
    sort_enactor.sort( n, sort_buffers );

    thrust::copy(
        keys_buf + sort_buffers.selector * n,
        keys_buf + sort_buffers.selector * n + n,
        keys );
}

// device-wide sort by key
//
// \param n                    number of input items
// \param keys                 a system input iterator of keys to be sorted
// \param values               a system input iterator of values to be sorted
//
template <typename KeyIterator, typename ValueIterator>
void radix_sort(
    const device_tag                    tag,
    const uint32                        n,
    KeyIterator                         keys,
    ValueIterator                       values,
    nvbio::vector<device_tag,uint8>&    temp_storage)
{
    typedef typename std::iterator_traits<KeyIterator>::value_type   key_type;
    typedef typename std::iterator_traits<ValueIterator>::value_type value_type;

    const uint32 aligned_key_bytes = align<16>( 2 * n * sizeof(key_type) );
    const uint32 aligned_val_bytes =            2 * n * sizeof(value_type);
    cuda::alloc_temp_storage( temp_storage, aligned_key_bytes + aligned_val_bytes );

    key_type*     keys_ptr = reinterpret_cast<key_type*>( raw_pointer( temp_storage ) );
    value_type* values_ptr = reinterpret_cast<value_type*>( raw_pointer( temp_storage ) + aligned_key_bytes );

    thrust::device_ptr<key_type> keys_buf( keys_ptr );
    thrust::device_ptr<key_type> values_buf( values_ptr );

    thrust::copy( keys,     keys + n,   keys_buf );
    thrust::copy( values,   values + n, values_buf );

    cuda::SortBuffers<key_type*, value_type*> sort_buffers;
    sort_buffers.keys[0]   = keys_ptr;
    sort_buffers.keys[1]   = keys_ptr + n;
    sort_buffers.values[0] = values_ptr;
    sort_buffers.values[1] = values_ptr + n;

    cuda::SortEnactor sort_enactor;
    sort_enactor.sort( n, sort_buffers );

    thrust::copy(
        keys_buf + sort_buffers.selector * n,
        keys_buf + sort_buffers.selector * n + n,
        keys );

    thrust::copy(
        values_buf + sort_buffers.selector * n,
        values_buf + sort_buffers.selector * n + n,
        values );
}

#endif

// host-wide sort
//
// \param n                    number of input items
// \param keys                 a system input iterator of keys to be sorted
//
template <typename KeyIterator>
void radix_sort(
    const host_tag                      tag,
    const uint32                        n,
    KeyIterator                         keys,
    nvbio::vector<host_tag,uint8>&      temp_storage)
{
    thrust::sort( keys, keys + n );
}

// system-wide sort
//
// \param n                    number of input items
// \param keys                 a system input iterator of keys to be sorted
//
template <typename system_tag, typename KeyIterator>
void radix_sort(
    const uint32                        n,
    KeyIterator                         keys,
    nvbio::vector<system_tag,uint8>&    temp_storage)
{
    radix_sort( system_tag(), n, keys, temp_storage );
}

// host-wide sort by key
//
// \param n                    number of input items
// \param keys                 a system input iterator of keys to be sorted
// \param values               a system input iterator of values to be sorted
//
template <typename KeyIterator, typename ValueIterator>
void radix_sort(
    const host_tag                      tag,
    const uint32                        n,
    KeyIterator                         keys,
    ValueIterator                       values,
    nvbio::vector<host_tag,uint8>&      temp_storage)
{
    thrust::sort_by_key( keys, keys + n, values, temp_storage );
}

// system-wide sort by key
//
// \param n                    number of input items
// \param keys                 a system input iterator of keys to be sorted
// \param values               a system input iterator of values to be sorted
//
template <typename system_tag, typename KeyIterator, typename ValueIterator>
void radix_sort(
    const uint32                        n,
    KeyIterator                         keys,
    ValueIterator                       values,
    nvbio::vector<system_tag,uint8>&    temp_storage)
{
    radix_sort( system_tag(), n, keys, values, temp_storage );
}

template <
    typename key_iterator1,
    typename key_iterator2>
uint2 corank(
    const int32         i,
    const key_iterator1 A,
    const int32         m,
    const key_iterator2 B,
    const int32         n)
{
    int32 j = min( i, m );
    int32 k = i - j;

    int32 j_lo = i >= n ? i - n : 0;
    int32 k_lo = 0;

    while (1)
    {
        if ((j > 0 || k < n) && A[j-1] > B[k])
        {
            // decrease j
            const int32 delta = util::divide_ri( j - j_lo, 2 );
            k_lo = k;
            j -= delta;
            k += delta;
            assert( j + k == i );
        }
        else if ((k > 0 || j < m) && B[k-1] >= A[j])
        {
            // decrease k
            const int32 delta = util::divide_ri( k - k_lo, 2 );
            j_lo = j;
            j += delta;
            k -= delta;
            assert( j + k == i );
        }
        else
            break;
    }
    return make_uint2( uint32(j), uint32(k) );
}

template <
    typename key_iterator1,
    typename key_iterator2,
    typename value_iterator1,
    typename value_iterator2,
    typename key_output,
    typename value_output>
void merge_by_key(
    const host_tag          tag,
    const uint32            A_len,
    const uint32            B_len,
    const key_iterator1     A_keys,
    const key_iterator2     B_keys,
    const value_iterator1   A_values,
    const value_iterator2   B_values,
          key_output        C_keys,
          value_output      C_values)
{
    if (A_len == 0)
    {
        #pragma omp parallel for
        for (int32 i = 0; i < int32( B_len ); ++i)
        {
            C_keys[i]   = A_keys[i];
            C_values[i] = A_values[i];
        }
    }
    else if (B_len == 0)
    {
        #pragma omp parallel for
        for (int32 i = 0; i < int32( A_len ); ++i)
        {
            C_keys[i]   = A_keys[i];
            C_values[i] = A_values[i];
        }
    }

    const uint32 n_threads = (uint32)omp_get_num_procs();

    nvbio::vector<host_tag,uint32> A_diag( n_threads+1 );
    nvbio::vector<host_tag,uint32> B_diag( n_threads+1 );

    const uint32 C_len = A_len + B_len;

    A_diag[ n_threads ] = 0;
    B_diag[ n_threads ] = 0;
    A_diag[ n_threads ] = A_len;
    B_diag[ n_threads ] = B_len;

    const uint32 n_partition = util::divide_ri( C_len, n_threads );

    #pragma omp parallel for num_threads(n_threads)
    for (int32 i = 1; i < int32( n_threads ); ++i)
    {
        const int32 index = i * n_partition;

        const uint2 jk = corank( index, A_keys, A_len, B_keys, B_len );

        A_diag[i] = jk.x;
        B_diag[i] = jk.y;
    }

    #pragma omp parallel for num_threads(n_threads)
    for (int32 i = 0; i < int32( n_threads ); ++i)
    {
        nvbio::merge_by_key(
            A_keys   + A_diag[i],
            A_keys   + A_diag[i+1],
            B_keys   + B_diag[i],
            B_keys   + B_diag[i+1],
            A_values + A_diag[i],
            B_values + B_diag[i],
            C_keys   + i * n_partition,
            C_values + i * n_partition );
    }
/*  for (uint32 i = 1; i < C_len; ++i)
    {
        if (C_keys[i-1] > C_keys[i])
        {
            fprintf(stderr, "merging error at %u: %llu, %llu\n", i, C_keys[i-1], C_keys[i] );
            exit(1);
        }
    }*/
}

template <
    typename key_iterator1,
    typename key_iterator2,
    typename value_iterator1,
    typename value_iterator2,
    typename key_output,
    typename value_output>
void merge_by_key(
    const device_tag        tag,
    const uint32            A_len,
    const uint32            B_len,
    const key_iterator1     A_keys,
    const key_iterator2     B_keys,
    const value_iterator1   A_values,
    const value_iterator2   B_values,
          key_output        C_keys,
          value_output      C_values)
{
    thrust::merge_by_key(
        A_keys,
        A_keys + A_len,
        B_keys,
        B_keys + A_len,
        A_values,
        B_values,
        C_keys,
        C_values );
}

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
    nvbio::vector<system_tag,uint8>&    temp_storage)
{
    merge_by_key(
        system_tag(),
        A_len,
        B_len,
        A_keys,
        B_keys,
        A_values,
        B_values,
        C_keys,
        C_values );
}

#if defined(__CUDACC__)

/// A very simple for_each CUDA kernel
///
template <typename iterator_type, typename functor_type>
__global__
void for_each_kernel(const uint64 n, const iterator_type in, const functor_type f)
{
    const uint32 grid_size = blockDim.x * gridDim.x;

    for (uint64 i = threadIdx.x + blockIdx.x * blockDim.x; i < n; i += grid_size)
        f( in[i] );
};

#endif

// ask the optimizer how many blocks we should try using next
//
template <typename KernelFunction>
uint32 for_each_enactor<device_tag>::suggested_blocks(KernelFunction kernel, const uint32 cta_size) const
{
#if defined(__CUDACC__)
    if (m_blocks_hi == 0)
        return cuda::multiprocessor_count() * cuda::max_active_blocks_per_multiprocessor( kernel, cta_size, 0u );
    else if (m_blocks_lo == 0)
        return cuda::multiprocessor_count();
    else
        return cuda::multiprocessor_count() * (m_blocks_lo + m_blocks_hi) / 2;
#else
    return 0u;
#endif
}

// update the optimizer's internal state with the latest speed data-point
//
inline
void for_each_enactor<device_tag>::update(const uint32 n_blocks, const float speed)
{
#if defined(__CUDACC__)
    // carry out a little binary search over the best number of blocks/SM
    if (m_blocks_hi == 0)
    {
        m_blocks_hi = n_blocks / cuda::multiprocessor_count();
        m_speed_hi  = speed;
    }
    else if (m_blocks_lo == 0)
    {
        m_blocks_lo = n_blocks / cuda::multiprocessor_count();
        m_speed_lo  = speed;
    }
    else if (m_speed_lo > m_speed_hi)
    {
        m_blocks_hi = n_blocks / cuda::multiprocessor_count();
        m_speed_hi  = speed;
    }
    else 
    {
        m_blocks_lo = n_blocks / cuda::multiprocessor_count();
        m_speed_lo  = speed;
    }
    // TODO: once the optimizer settles to a given value, it will never change:
    // we should explore using occasional "mutations" to adapt to possibly
    // changing conditions...
#endif
}

// enact the for_each
//
template <typename Iterator, typename Functor>
void for_each_enactor<device_tag>::operator () (
    const uint64            n,
    const Iterator          in,
          Functor           functor)
{
#if defined(__CUDACC__)
    const uint32 blockdim = 128;
    const uint32 n_blocks = suggested_blocks( for_each_kernel<Iterator,Functor>, blockdim );

    cuda::Timer timer;
    timer.start();

    for_each_kernel<<<n_blocks,blockdim>>>( n, in, functor );

    timer.stop();

    update( n_blocks, float(n) / timer.seconds() );
#endif
}

} // namespace nvbio
