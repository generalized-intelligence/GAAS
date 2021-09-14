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
#include <nvbio/basic/iterator.h>

namespace nvbio {

///@addtogroup TriesModule
///@{

///@addtogroup SortedDictionarySuffixTriesModule Sorted Dictionary Suffix Tries
///@{

///
/// A node of a SortedDictionarySuffixTrie
///
struct SortedDictionaryNode
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SortedDictionaryNode() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SortedDictionaryNode(const uint32 _begin, const uint32 _end, const uint32 _level) :
        begin( _begin ), end( _end ), level( _level ) {}

    uint32 begin;
    uint32 end;
    uint32 level;
};

///
/// A suffix trie type built on a generic dictionary of sorted strings
///
/// \tparam ALPHABET_SIZE_T     the size of the alphabet
/// \tparam Iterator            an iterator to the sorted string dictionary
///
template <uint32 ALPHABET_SIZE_T, typename Iterator>
struct SortedDictionarySuffixTrie
{
    const static uint32 ALPHABET_SIZE = ALPHABET_SIZE_T;

    typedef typename std::iterator_traits<Iterator>::value_type string_type;
    typedef SortedDictionaryNode                                node_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SortedDictionarySuffixTrie(const Iterator seq, const uint32 size);

    /// return the root node of the dictionary seen as a trie
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    node_type root() const;

    /// visit the children of a given node
    ///
    /// \tparam Visitor     a visitor implementing the following interface:
    /// \code
    /// struct Visitor
    /// {
    ///     // do something with the node corresponding to character c
    ///     void visit(const uint8 c, const NodeType node);
    /// }
    /// \endcode
    ///
    template <typename Visitor>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void children(const node_type node, Visitor& visitor) const;

    /// return true if the node is a leaf
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool is_leaf(const node_type node) const;

    /// return the size of a node
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size(const node_type node) const;

private:
    Iterator m_seq;
    uint32   m_size;
};

///@} // SortedDictionarySuffixTriesModule
///@} // TriesModule

} // namespace nvbio

#include <nvbio/trie/sorted_dictionary_inl.h>
