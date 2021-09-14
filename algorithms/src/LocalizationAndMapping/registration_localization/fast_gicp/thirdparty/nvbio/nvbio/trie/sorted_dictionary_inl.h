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
#include <nvbio/basic/transform_iterator.h>
#include <nvbio/basic/algorithms.h>

namespace nvbio {

// constructor
//
template <uint32 ALPHABET_SIZE_T, typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
SortedDictionarySuffixTrie<ALPHABET_SIZE_T,Iterator>::SortedDictionarySuffixTrie(const Iterator seq, const uint32 size) :
    m_seq( seq ), m_size( size )
{}

// return the root node of the dictionary seen as a trie
//
template <uint32 ALPHABET_SIZE_T, typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename SortedDictionarySuffixTrie<ALPHABET_SIZE_T,Iterator>::node_type
SortedDictionarySuffixTrie<ALPHABET_SIZE_T,Iterator>::root() const
{
    return node_type( 0u, m_size, length( m_seq[m_size-1] ) );
}

// visit the children of a given node
//
template <uint32 ALPHABET_SIZE_T, typename Iterator>
template <typename Visitor>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void SortedDictionarySuffixTrie<ALPHABET_SIZE_T,Iterator>::children(const node_type node, Visitor& visitor) const
{
    // check subranges
    const uint8 lc = m_seq[ node.begin ][ node.level-1u ];
    const uint8 rc = m_seq[ node.end-1 ][ node.level-1u ];

    uint32 l_boundary = node.begin;
    uint8  c = lc;
    while (c < rc)
    {
        const uint32 r_boundary = uint32( upper_bound(
            c,
            make_transform_iterator( m_seq, get_char_functor<string_type>( node.level-1u ) ) + l_boundary,
            node.end - l_boundary ) - make_transform_iterator( m_seq, get_char_functor<string_type>( node.level-1u ) ) );

        visitor.visit( c, node_type( l_boundary, r_boundary, node.level-1u ) );

        l_boundary = r_boundary;
        c = m_seq[r_boundary][node.level-1u];
    }
    // last child
    visitor.visit( rc, node_type( l_boundary, node.end, node.level-1u ) );
}

// return true if the node is a leaf
//
template <uint32 ALPHABET_SIZE_T, typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool SortedDictionarySuffixTrie<ALPHABET_SIZE_T,Iterator>::is_leaf(const node_type node) const
{
    return node.level == 0u;
}

// return the size of a node
//
template <uint32 ALPHABET_SIZE_T, typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 SortedDictionarySuffixTrie<ALPHABET_SIZE_T,Iterator>::size(const node_type node) const
{
    return node.end - node.begin;
}

} // namespace nvbio
