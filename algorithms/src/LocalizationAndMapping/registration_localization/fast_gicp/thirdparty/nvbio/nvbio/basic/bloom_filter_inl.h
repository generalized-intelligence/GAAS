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

namespace nvbio {

template <
    uint32   K,         // number of hash functions { Hash1 + i * Hash2 | i : 0, ..., K-1 }
    typename Hash1,     // first hash generator function
    typename Hash2,     // second hash generator function
    typename Iterator,  // storage iterator - must dereference to uint32
    typename OrOperator>
bloom_filter<K,Hash1,Hash2,Iterator,OrOperator>::bloom_filter(
    const uint64    size,
    Iterator        storage,
    const Hash1     hash1,
    const Hash2     hash2) :
    m_size( size ),
    m_storage( storage ),
    m_hash1( hash1 ),
    m_hash2( hash2 ) {}

template <
    uint32   K,         // number of hash functions { Hash1 + i * Hash2 | i : 0, ..., K-1 }
    typename Hash1,     // first hash generator function
    typename Hash2,     // second hash generator function
    typename Iterator,  // storage iterator - must dereference to uint32
    typename OrOperator>
template <typename Key>
void bloom_filter<K,Hash1,Hash2,Iterator,OrOperator>::insert(const Key key, const OrOperator or_op)
{
    const uint64 h0 = m_hash1( key );
    const uint64 h1 = m_hash2( key );

    #if defined(__CUDA_ARCH__)
    #pragma unroll
    #endif
    for (uint64 i = 0; i < K; ++i)
    {
        const uint64 r = (h0 + i * h1) % m_size;

        const uint32 word = uint32(r >> 5);
        const uint32 bit  = uint32(r) & 31u;

        or_op( &m_storage[word], (1u << bit) );
    }
}

template <
    uint32   K,         // number of hash functions { Hash1 + i * Hash2 | i : 0, ..., K-1 }
    typename Hash1,     // first hash generator function
    typename Hash2,     // second hash generator function
    typename Iterator,  // storage iterator - must dereference to uint32
    typename OrOperator>
template <typename Key>
bool bloom_filter<K,Hash1,Hash2,Iterator,OrOperator>::has(const Key key) const
{
    const uint64 h0 = m_hash1( key );
    const uint64 h1 = m_hash2( key );

    #if defined(__CUDA_ARCH__)
    #pragma unroll
    #endif
    for (uint64 i = 0; i < K; ++i)
    {
        const uint64 r = (h0 + i * h1) % m_size;

        const uint32 word = uint32(r >> 5);
        const uint32 bit  = uint32(r) & 31u;

        if ((m_storage[word] & (1u << bit)) == 0u)
            return false;
    }
    return true;
}


template <
    typename Hash1,             // first hash generator function
    typename Hash2,             // second hash generator function
    typename Iterator,          // storage iterator - must be one of {uint32|uint2|uint4|uint64|uint64_2|uint64_4}
    typename OrOperator>
blocked_bloom_filter<Hash1,Hash2,Iterator,OrOperator>::blocked_bloom_filter(
    const uint32    k,
    const uint64    size,
    Iterator        storage,
    const Hash1     hash1,
    const Hash2     hash2) :
    m_k( k ),
    m_size( size ),
    m_storage( storage ),
    m_hash1( hash1 ),
    m_hash2( hash2 ) {}

template <
    typename Hash1,     // first hash generator function
    typename Hash2,     // second hash generator function
    typename Iterator,  // storage iterator - must be one of {uint32|uint2|uint4|uint64|uint64_2|uint64_4}
    typename OrOperator>
template <typename Key>
void blocked_bloom_filter<Hash1,Hash2,Iterator,OrOperator>::insert(const Key key, const OrOperator or_op)
{
    const uint64 h0 = m_hash1( key );
    const uint64 h1 = m_hash2( key );

    const uint64 block_idx = (h0 % m_size) / BLOCK_SIZE;
          block_type block = vector_type( 0u );

    #if defined(__CUDA_ARCH__)
    #pragma unroll
    #endif
    for (uint64 i = 1; i <= m_k; ++i)
    {
        const uint32 r = uint32(h0 + i * h1) % BLOCK_SIZE;

        const uint32 word = r / WORD_SIZE;
        const uint32 bit  = r & (WORD_SIZE-1);

        const word_type pattern = (word_type(1u) << bit);

        or_op( block, word, pattern );
    }
    or_op( &m_storage[block_idx], block );
}

template <
    typename Hash1,     // first hash generator function
    typename Hash2,     // second hash generator function
    typename Iterator,  // storage iterator - must be one of {uint32|uint2|uint4|uint64|uint64_2|uint64_4}
    typename OrOperator>
template <typename Key>
bool blocked_bloom_filter<Hash1,Hash2,Iterator,OrOperator>::has(const Key key) const
{
    const uint64 h0 = m_hash1( key );
    const uint64 h1 = m_hash2( key );

    const uint64 block_idx = (h0 % m_size) / BLOCK_SIZE;
    const block_type block = m_storage[block_idx];

    #if defined(__CUDA_ARCH__)
    #pragma unroll
    #endif
    for (uint64 i = 1; i <= m_k; ++i)
    {
        const uint32 r = uint32(h0 + i * h1) % BLOCK_SIZE;

        const uint32 word = r / WORD_SIZE;
        const uint32 bit  = r & (WORD_SIZE-1);

        const word_type pattern = (word_type(1u) << bit);

        if ((comp( block, word ) & pattern) == 0u)
            return false;
    }
    return true;
}

} // namespace nvbio
