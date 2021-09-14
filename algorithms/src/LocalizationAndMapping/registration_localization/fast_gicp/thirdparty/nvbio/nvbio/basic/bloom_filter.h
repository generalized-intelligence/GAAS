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
#include <nvbio/basic/atomics.h>
#include <nvbio/basic/static_vector.h>

namespace nvbio {

///\page bloom_filter_page Bloom Filters
///\par
/// <a href=http://en.wikipedia.org/wiki/Bloom_filter>Bloom filters</a> are a probabilistic data-structure
/// useful to <i>conservatively</i> represent membership in a set using less than 1 bit per set item.
///\par
/// NVBIO provides two simple generic classes to incrementally build and work with standard and <i>blocked</i>
/// Bloom filters both on the host and the device:
///
/// - bloom_filter
/// - blocked_bloom_filter
///
/// \section ExampleSection Example
///
///\code
/// // populate a filter in parallel
/// template <typename bloom_filter_type, typename T>
/// __global__ void populate_kernel(const uint32 N, const T* vector, bloom_filter_type filter)
/// {
///     const uint32 i = threadIdx.x + blockIdx.x * blockDim.x;
///     if (i < N)
///         filter.insert( vector[i] );
/// }
///
/// bool bloom_test()
/// {
///     // build a set of 1M random integers
///     const uint32 N = 1000000;
///     nvbio::vector<host_tag,uint32> h_vector( N );
///
///     // fill it up
///     for (uint32 i = 0; i < N; ++i)
///         h_vector[i] = rand();
///
///     // copy it to the device
///     nvbio::vector<device_tag,uint32> d_vector = h_vector;
///
///     // construct an empty Bloom filter
///     typedef bloom_filter<2,hash_functor1,hash_functor2,uint32*> bloom_filter_type;
///
///     const uint32 filter_words = N;  // let's use 32-bits per input item;
///                                     // NOTE: this is still a lot less than 1-bit for each
///                                     // of the 4+ billion integers out there...
///
///     nvbio::vector<device_tag,uint32> d_filter_storage( filter_words, 0u );
///     bloom_filter_type d_filter(
///        filter_words * 32,
///        plain_view( d_filter_storage ) );
///
///     // and populate it
///     populate_kernel<<<util::divide_ri(N,128),128>>>( N, plain_view( d_vector ), d_filter );
///
///     // copy the filter back on the host
///     nvbio::vector<host_tag,uint32> h_filter_storage = filter_storage;
///     bloom_filter_type h_filter(
///        filter_words * 32,
///        plain_view( h_filter_storage ) );
///
///     // and now ask The Question: is 42 in there?
///     return h_filter.has( 42 );
/// }
///\endcode
///

///@addtogroup Basic
///@{

///\defgroup BloomFilterModule Bloom Filters
///
/// <a href=http://en.wikipedia.org/wiki/Bloom_filter>Bloom filters</a> are a probabilistic data-structure
/// useful to <i>conservatively</i> represent membership in a set using less than 1 bit per item.
///

///@addtogroup BloomFilterModule
///@{

///
/// atomic in-place OR binary functor used to construct a bloom_filter
///
template <typename T>
struct inplace_or
{
};

///
/// atomic in-place OR binary functor used to construct a bloom_filter
///
template <>
struct inplace_or<uint32>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint32* word, const uint32 mask) const
    {
        atomic_or( word, mask );
    }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint32& block, const uint32 word, const uint32 mask) const
    {
        block |= mask;
    }
};

///
/// atomic in-place OR binary functor used to construct a bloom_filter
///
template <>
struct inplace_or<uint2>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint2* word, const uint2 mask) const
    {
        atomic_or( &(word->x), mask.x );
        atomic_or( &(word->y), mask.y );
    }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint2& block, const uint32 word, const uint32 mask) const
    {
        if (word == 0) block.x |= mask;
        else           block.y |= mask;
    }
};

///
/// atomic in-place OR binary functor used to construct a bloom_filter
///
template <>
struct inplace_or<uint4>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint4* word, const uint4 mask) const
    {
        atomic_or( &(word->x), mask.x );
        atomic_or( &(word->y), mask.y );
        atomic_or( &(word->z), mask.z );
        atomic_or( &(word->w), mask.w );
    }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint4& block, const uint32 word, const uint32 mask) const
    {
        if      (word == 0) block.x |= mask;
        else if (word == 1) block.y |= mask;
        else if (word == 2) block.z |= mask;
        else                block.w |= mask;
    }
};

///
/// atomic in-place OR binary functor used to construct a bloom_filter
///
template <>
struct inplace_or<uint64>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint64* word, const uint64 mask) const
    {
        atomic_or( word, mask );
    }
};

///
/// atomic in-place OR binary functor used to construct a bloom_filter
///
template <>
struct inplace_or<uint64_2>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint64_2* word, const uint64_2 mask) const
    {
        atomic_or( &(word->x), mask.x );
        atomic_or( &(word->y), mask.y );
    }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint64_2& block, const uint32 word, const uint64 mask) const
    {
        if (word == 0) block.x |= mask;
        else           block.y |= mask;
    }
};

///
/// atomic in-place OR binary functor used to construct a bloom_filter
///
template <>
struct inplace_or<uint64_4>
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint64_4* word, const uint64_4 mask) const
    {
        atomic_or( &(word->x), mask.x );
        atomic_or( &(word->y), mask.y );
        atomic_or( &(word->z), mask.z );
        atomic_or( &(word->w), mask.w );
    }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void operator() (uint64_4& block, const uint32 word, const uint64 mask) const
    {
        if      (word == 0) block.x |= mask;
        else if (word == 1) block.y |= mask;
        else if (word == 2) block.z |= mask;
        else                block.w |= mask;
    }
};

///
/// A Bloom filter implementation.
/// This class is <i>storage-free</i>, and can used both from the host and the device.
/// Constructing a Bloom filter can be done incrementally calling insert(), either sequentially
/// or in parallel.
///
/// \tparam  K              the number of hash functions, obtained as { Hash1 + i * Hash2 | i : 0, ..., K-1 }
/// \tparam  Hash1          the first hash function
/// \tparam  Hash2          the second hash function
/// \tparam  Iterator       the iterator to the internal filter storage, iterator_traits<iterator>::value_type
///                         must be a uint32
/// \tparam  OrOperator     the binary functor used to OR the filter's words with the inserted keys;
///                         NOTE: this operation must be performed atomically if the filter is constructed
///                         in parallel
///
template <
    uint32   K,                         // number of hash functions { Hash1 + i * Hash2 | i : 0, ..., K-1 }
    typename Hash1,                     // first hash generator function
    typename Hash2,                     // second hash generator function
    typename Iterator,                  // storage iterator - must dereference to uint32
    typename OrOperator = inplace_or<uint32> >   // OR binary operator
struct bloom_filter
{
    /// constructor
    ///
    /// \param size         the Bloom filter's storage size, in bits
    /// \param storage      the Bloom filter's internal storage
    /// \param hash1        the first hashing function
    /// \param hash2        the second hashing function
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    bloom_filter(
        const uint64    size,
        Iterator        storage,
        const Hash1     hash1 = Hash1(),
        const Hash2     hash2 = Hash2());

    /// insert a key
    ///
    template <typename Key>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void insert(const Key key, const OrOperator or_op = OrOperator());

    /// check for a key
    ///
    template <typename Key>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    bool has(const Key key) const;

    /// check for a key
    ///
    template <typename Key>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator[] (const Key key) const { return has( key ); }

    uint64      m_size;
    Iterator    m_storage;
    Hash1       m_hash1;
    Hash2       m_hash2;
};


///
/// A blocked Bloom filter implementation.
/// This class is <i>storage-free</i>, and can used both from the host and the device.
/// Constructing a Bloom filter can be done incrementally calling insert(), either sequentially
/// or in parallel.
/// The block size is determined by the value_type of the storage Iterator: e.g. a uint4 or a uint64_2
/// will cause the Bloom filter to use 128-bit blocks, whereas a uint64_4 will cause it to use
/// 256-bit blocks.
///
/// \tparam  Hash1              the first hash function
/// \tparam  Hash2              the second hash function
/// \tparam  Iterator           the iterator to the internal filter storage, iterator_traits<iterator>::value_type
///                             must be one of {uint32|uint2|uint4|uint64|uint64_2|uint64_4}
/// \tparam  OrOperator         the binary functor used to OR the filter's words with the inserted keys;
///                             NOTE: this operation must be performed atomically if the filter is constructed
///                             in parallel
///
template <
    typename Hash1,                     // first hash generator function
    typename Hash2,                     // second hash generator function
    typename Iterator,                  // storage iterator - must be one of {uint32|uint2|uint4|uint64|uint64_2|uint64_4}
    typename OrOperator = inplace_or<typename std::iterator_traits<Iterator>::value_type> >   // OR binary operator
struct blocked_bloom_filter
{
    typedef typename std::iterator_traits<Iterator>::value_type block_type;
    typedef typename vector_traits<block_type>::value_type      word_type;

    static const uint32 BLOCK_DIM  = vector_traits<block_type>::DIM;
    static const uint32 BLOCK_SIZE = sizeof(block_type)*8u;
    static const uint32 WORD_SIZE  = sizeof(word_type)*8u;

    typedef StaticVector<word_type,BLOCK_DIM>                   vector_type;

    /// constructor
    ///
    /// \param size         the Bloom filter's storage size, in bits
    /// \param storage      the Bloom filter's internal storage
    /// \param hash1        the first hashing function
    /// \param hash2        the second hashing function
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    blocked_bloom_filter(
        const uint32    k,
        const uint64    size,
        Iterator        storage,
        const Hash1     hash1 = Hash1(),
        const Hash2     hash2 = Hash2());

    /// insert a key
    ///
    template <typename Key>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    void insert(const Key key, const OrOperator or_op = OrOperator());

    /// check for a key
    ///
    template <typename Key>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    bool has(const Key key) const;

    /// check for a key
    ///
    template <typename Key>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool operator[] (const Key key) const { return has( key ); }

    uint32          m_k;
    uint64          m_size;
    Iterator        m_storage;
    Hash1           m_hash1;
    Hash2           m_hash2;
};

/// compute the optimal number of Bloom filter hash functions given the
/// number of bits per key
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 optimal_bloom_filter_hashes(const float bits_per_key)
{
    return uint32( bits_per_key * logf(2.0) ) + 1u;
}

/// compute the optimal Bloom filter size for a given target false positive rate,
/// assuming an optimal number of hash functions
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
float optimal_bloom_filter_bits_per_key(const float fp_rate)
{
    return -logf(fp_rate) / (logf(2.0f)*logf(2.0f));
}

/// compute the optimal Bloom filter size for a given number of hash functions
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
float optimal_bloom_filter_bits_per_key(const uint32 K)
{
    return float(K) / logf(2.0f);
}

/// compute the optimal Bloom filter parameters for a given target false positive rate,
/// specifically the number of hash functions K, and the number of bits per key
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void optimal_bloom_filter_parameters(const float fp_rate, uint32* K, float* bits_per_key)
{
    *bits_per_key = optimal_bloom_filter_bits_per_key( fp_rate );

    *K = optimal_bloom_filter_hashes( *bits_per_key );
}

/// compute the achievable false positive rate of a Bloom filter with a given number
/// of hash functions and bits per key
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
float optimal_bloom_filter_fp_rate(const uint32 K, const float bits_per_key)
{
    return powf( 1.0f - expf( -float(K) / bits_per_key ), float(K) );
}

///@} BloomFilterModule
///@} Basic

} // namespace nvbio

#include <nvbio/basic/bloom_filter_inl.h>
