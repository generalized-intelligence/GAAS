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
#include <iterator>

namespace nvbio {

/// \page sum_trees_page Sum Trees
///
/// This module implements a class to hold a Sum Tree, an object similar to a Haar Wavelet
/// tree, with the difference that each node encodes the plain sum of the children's coefficients.
/// This data-structure allows O(log(N)) modifications to the values associated with the leaves
/// of the tree.\n\n
/// An example usage of this class is to sample CDFs that are continuously
/// updated over time.\n\n
/// This data structure is <i>storage-free</i> and relies on the user to provide the combined
/// storage for the leaves and the internal nodes using a template iterator.
/// It can be used both in host and device code.
///
/// <img src="tree.png" style="position:relative; bottom:-10px; border:0px;"/>
///
/// See:
///
/// - SumTree
/// - uint32 sample<Iterator>(const SumTree<Iterator>& tree, const float value)
///
/// \section SumTreeExample Example
///
/// \code
/// // build a PDF, assigning each slot a value between 0 and 10.
/// // NOTE: we need to alloc space for the leaves as well as the inner nodes.
/// const uint32 n_leaves = 100;
/// const uint32 n_nodes  = SumTree<uint32*>::node_count( n_leaves );
/// std::vector<float> probs( n_nodes );
/// for (uint32 i = 0; i < n_leaves; ++i)
///     probs[i] = uint32( drand48() * 10 );
///
/// // build the sum tree
/// SumTree<float*> prob_tree( n_leaves, &probs[0] );
/// prob_tree.setup();
///
/// // pick its cells in random order with probability proportional to their value,
/// // and every time a cell is sampled decrease its probability by 1.
/// while (prob_tree.sum() > 0)
/// {
///     // sample a cell using our probability tree
///     const uint32 cell = sample( prob_tree, drand48() );
///
///     // decrease by one this cell's probability
///     prob_tree.add( cell, -1 );
///
///     // and do something with the selected cell...
///     record_event( cell );
/// }
/// \endcode
///

///@addtogroup Basic
///@{

///@defgroup SumTrees Sum Trees
/// This module implements a class to hold a Sum Tree, an object similar to a Haar Wavelet
/// tree, with the difference that each node encodes the plain sum of the children's coefficients.
/// This data-structure allows O(log(N)) modifications to the values associated with the leaves
/// of the tree.
///@{

///
/// Given an array of length N, it's possible to build a binary tree on top of it
/// with 2*N-1 nodes, where each internal node encodes the sum of the values associated
/// with its leaves. The original N cells will be the leaves of such a tree.
/// This class encodes such a tree, allowing incremental modifications of the
/// values associated with the leaves.
/// If N is not a power of 2, the base array is conceptually padded with zeros.
///
/// An example usage of this class is to sample CDFs that are continuously
/// updated over time.
///
template <typename Iterator>
struct SumTree
{
    typedef Iterator                                              iterator_type;
    typedef typename std::iterator_traits<Iterator>::value_type   value_type;

    /// return the number of nodes corresponding to a given number of leaves
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static uint32 node_count(const uint32 size);

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SumTree(const uint32 size, iterator_type cells);

    /// return the number of leaves
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_size; }

    /// return the number of leaves, padded to the nearest power of 2
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 padded_size() const { return m_padded_size; }

    /// return the total node count
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 nodes() const { return m_padded_size * 2u - 1u; }

    /// setup the tree structure
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void setup(const value_type zero = value_type(0));

    /// increment a cell's value
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void add(const uint32 i, const value_type v);

    /// reset a cell's value
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void set(const uint32 i, const value_type v);

    /// return the total tree sum
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    value_type sum() const { return m_cells[ m_padded_size * 2u - 2u ]; }

    /// return a cell's value
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    value_type cell(const uint32 i) const { return m_cells[i]; }

private:
    iterator_type m_cells;
    uint32        m_size;
    uint32        m_padded_size;
};

/// sample a cell from a linear SumTree, returning a leaf with probability proportional to its value in the SumTree
///
/// \param value        a value in the range [0,1]
/// \return             the sampled cell
///
template <typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 sample(const SumTree<Iterator>& tree, const float value);

///@} SumTrees
///@} Basic

} // namespace nvbio

#include <nvbio/basic/sum_tree_inl.h>
