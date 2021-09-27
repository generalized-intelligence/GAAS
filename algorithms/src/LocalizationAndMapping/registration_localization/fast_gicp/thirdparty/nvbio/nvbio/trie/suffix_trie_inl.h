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
#include <nvbio/basic/popcount.h>

namespace nvbio {

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
TrieNode<ALPHABET_SIZE_T,TYPE_T>::TrieNode() : m_child(invalid_node), m_mask(0u), m_size(1u) {}

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
TrieNode<ALPHABET_SIZE_T,TYPE_T>::TrieNode(const uint32 _child, const uint32 _mask) :
    m_child( _child ), m_mask( _mask ), m_size(1u)
{
    assert( (_child <= invalid_node) && (m_child == _child) );
}

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool TrieNode<ALPHABET_SIZE_T,TYPE_T>::is_leaf() const { return m_mask == 0u; }

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode<ALPHABET_SIZE_T,TYPE_T>::child() const { return m_child; }

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode<ALPHABET_SIZE_T,TYPE_T>::child(const uint32 c) const
{
    if (TYPE_T == CompressedTrie)
        return m_child + nvbio::popc( m_mask & ((1u << c)-1u) );
    else
        return m_child + c;
}

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode<ALPHABET_SIZE_T,TYPE_T>::nth_child(const uint32 c) const
{
    if (TYPE_T == CompressedTrie)
        return m_child + c;
    else
        return m_child + find_nthbit( m_mask, c+1u );
}

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode<ALPHABET_SIZE_T,TYPE_T>::first_child() const
{
    if (TYPE_T == CompressedTrie)
        return m_child;
    else
        return m_child + ffs( m_mask );
}

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode<ALPHABET_SIZE_T,TYPE_T>::mask()  const { return m_mask; }

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void TrieNode<ALPHABET_SIZE_T,TYPE_T>::set_child_bit(const uint32 c) { m_mask |= (1u << c); }

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode<ALPHABET_SIZE_T,TYPE_T>::child_bit(const uint32 c) const { return m_mask & (1u << c); }

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void TrieNode<ALPHABET_SIZE_T,TYPE_T>::set_size(const uint32 size) { m_size = size; }

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode<ALPHABET_SIZE_T,TYPE_T>::size() const { return m_size; }



template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
TrieNode5<TYPE_T>::TrieNode5() : m_child(invalid_node), m_mask(0u), m_size(1u) {}

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
TrieNode5<TYPE_T>::TrieNode5(const uint32 _child, const uint32 _mask) :
    m_child( _child ), m_mask( _mask ), m_size(1u)
{
    assert( (_child <= invalid_node) && (m_child == _child) );
}

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool TrieNode5<TYPE_T>::is_leaf() const { return m_mask == 0u; }

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode5<TYPE_T>::child() const { return m_child; }

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode5<TYPE_T>::child(const uint32 c) const
{
    if (TYPE_T == CompressedTrie)
        return m_child + nvbio::popc( m_mask & ((1u << c)-1u) );
    else
        return m_child + c;
}

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode5<TYPE_T>::nth_child(const uint32 c) const
{
    if (TYPE_T == CompressedTrie)
        return m_child + c;
    else
        return m_child + find_nthbit8( m_mask, c+1u );
}

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode5<TYPE_T>::first_child() const
{
    if (TYPE_T == CompressedTrie)
        return m_child;
    else
        return m_child + ffs( m_mask );
}

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode5<TYPE_T>::mask()  const { return m_mask; }

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void TrieNode5<TYPE_T>::set_child_bit(const uint32 c) { m_mask |= (1u << c); }

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode5<TYPE_T>::child_bit(const uint32 c) const { return m_mask & (1u << c); }

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void TrieNode5<TYPE_T>::set_size(const uint32 size) { m_size = size; }

template <TrieType TYPE_T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 TrieNode5<TYPE_T>::size() const { return m_size; }



// constructor
//
template <uint32 ALPHABET_SIZE_T, typename NodeIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
SuffixTrie<ALPHABET_SIZE_T,NodeIterator>::SuffixTrie(const NodeIterator seq) :
    m_seq( seq )
{}

// return the root node of the dictionary seen as a trie
//
template <uint32 ALPHABET_SIZE_T, typename NodeIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename SuffixTrie<ALPHABET_SIZE_T,NodeIterator>::node_type
SuffixTrie<ALPHABET_SIZE_T,NodeIterator>::root() const
{
    return m_seq[0];
}

// visit the children of a given node
//
template <uint32 ALPHABET_SIZE_T, typename NodeIterator>
template <typename Visitor>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void SuffixTrie<ALPHABET_SIZE_T,NodeIterator>::children(const node_type node, Visitor& visitor) const
{
    for (uint8 c = 0; c < ALPHABET_SIZE; ++c)
    {
        if (node.child_bit(c))
            visitor.visit( c, node.child(c) );
    }
}

template <uint32 ALPHABET_SIZE_T, typename NodeType>
struct Collector
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    Collector() : count(0), mask(0) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void visit(const uint8 c, const NodeType node)
    {
        nodes[count++] = node;
        mask |= (1u << c);
    }

    uint32      count;
    uint32      mask;
    NodeType    nodes[ALPHABET_SIZE_T];
};

template <uint32 ALPHABET_SIZE_T, TrieType TYPE_T>
struct trie_copy {};

template <uint32 ALPHABET_SIZE_T>
struct trie_copy<ALPHABET_SIZE_T, CompressedTrie>
{
    template <typename InTrieType, typename OutVectorType>
    static void enact(
        const InTrieType&                    in_trie,
        const typename InTrieType::node_type in_node,
              OutVectorType&                 out_nodes,
        const uint32                         out_node_index)
    {
        typedef typename InTrieType::node_type in_node_type;

        // check if the input node is a leaf
        if (in_trie.is_leaf( in_node ))
        {
            out_nodes[ out_node_index ].set_size( in_trie.size( in_node ) );
            return;
        }

        Collector<ALPHABET_SIZE_T,in_node_type> children;

        in_trie.children( in_node, children );

        const uint32 child_offset = uint32( out_nodes.size() );
        out_nodes.resize( child_offset + children.count );

        out_nodes[ out_node_index ] = TrieNode<ALPHABET_SIZE_T,CompressedTrie>(
            child_offset,
            children.mask );

        for (uint32 i = 0; i < children.count; ++i)
        {
            enact(
                in_trie,
                children.nodes[i],
                out_nodes,
                child_offset + i );
        }
    }
};

template <typename TrieType, typename NodeVector>
void build_suffix_trie(
    const TrieType&     in_trie,
    NodeVector&         out_nodes)
{
    typedef typename NodeVector::value_type out_node_type;

    // alloc the root node
    out_nodes.resize(1u);

    trie_copy<TrieType::ALPHABET_SIZE, out_node_type::trie_type>::enact(
        in_trie,
        in_trie.root(),
        out_nodes,
        0u );
}

} // namespace nvbio
