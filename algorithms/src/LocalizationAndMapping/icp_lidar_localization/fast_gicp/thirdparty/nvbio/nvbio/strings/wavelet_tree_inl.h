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

template <typename symbol_type>
struct select_bit_functor
{
    typedef symbol_type   argument_type;
    typedef symbol_type   result_type;

    NVBIO_HOST_DEVICE
    select_bit_functor(const uint32 _i) : i(_i) {}

    NVBIO_HOST_DEVICE
    symbol_type operator() (const symbol_type c) const { return (c >> i) & 1u; }

    const uint32 i;
};

// a private class used for ranking 1's at global indices, exclusively
//
template <typename WaveletTreeType, typename OccIterator>
struct wavelet_ranker
{
    typedef typename WaveletTreeType::index_type index_type;

    typedef index_type  argument_type;
    typedef index_type  result_type;

    // constructor
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    wavelet_ranker(
        const WaveletTreeType   _tree,
        const OccIterator       _occ) :
        tree( _tree ), occ( _occ ) {}

    // unary transform operator
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_type operator() (const index_type r) const
    {
        if (r == 0)
            return 0;
        else
        {
            const uint32     i = r-1;                                           // return the number of ones at i = r-1
            const index_type base_index = i / 32u;                              // the index of this occurrence counter block
            const index_type base_occ   = occ[ base_index ];                    // the base occurrence counter for this block

            const uint32     word       = tree.bits().stream()[ base_index ];   // the block's corresponding word
            const uint32     word_occ   = popc_nbit<1u>( word, 1u, ~i & 31u );  // the delta popcount in this word

            return base_occ + word_occ;                                         // the final result
        }
    }

    const WaveletTreeType   tree;
    const OccIterator       occ;
};

namespace priv {
namespace wtree {

typedef std::pair<uint64,uint64> index_range;
typedef std::pair<uint64,uint64> symbol_range;

struct node_type
{
    node_type() {}
    node_type(
        const uint32        _level,
        const uint32        _index,
        const symbol_range  _s_range,
        const index_range   _i_range) :
        level( _level ),
        index( _index ),
        s_range( _s_range ),
        i_range( _i_range ) {}

    uint32       level;
    uint32       index;
    symbol_range s_range;
    index_range  i_range;
};

} // namespace wtree
} // namespace priv

//
// build a Wavelet Tree out of a string: the output consists of a bit-string representing
// the different bit-planes of the output WaveleTree, and an array of offsets to the
// leaves, containing the sorted string symbols.
//
// \tparam string_iterator     the string type: must provide a random access iterator
//                             interface as well as define a proper stream_traits<string_iterator>
//                             expansion; particularly, stream_traits<string_iterator>::SYMBOL_SIZE
//                             is used to infer the number of bits needed to represent the
//                             symbols in the string's alphabet
//
template <typename system_tag, typename string_iterator, typename index_type, typename symbol_type>
void setup(
    const index_type                                        string_len,
    const string_iterator&                                  string,
    WaveletTreeStorage<system_tag,index_type,symbol_type>&  out_tree)
{
    typedef typename WaveletTreeStorage<system_tag,index_type>::plain_view_type     wavelet_tree_view_type;
    typedef typename WaveletTreeStorage<system_tag,index_type>::bit_iterator        bit_iterator;
    typedef typename WaveletTreeStorage<system_tag,index_type>::index_iterator      occ_iterator;

    const uint32 symbol_size = stream_traits<string_iterator>::SYMBOL_SIZE;

    // resize the output tree
    out_tree.resize( string_len, symbol_size );

    // allocate a temporary string used for sorting
    nvbio::vector<system_tag,symbol_type> sorted_string( string_len * 2 );

    typename nvbio::vector<system_tag,symbol_type>::iterator sorted_keys;

    if (equal<system_tag,device_tag>())
    {
        // copy the input to the temporary sorting string
        thrust::copy( string, string + string_len, sorted_string.begin() );

        cuda::SortBuffers<symbol_type*> sort_buffers;
        sort_buffers.keys[0] = raw_pointer( sorted_string );
        sort_buffers.keys[1] = raw_pointer( sorted_string ) + string_len;

        cuda::SortEnactor sort_enactor;

        // loop through all bit planes in the range [0,symbol_size)
        for (uint32 i = 0; i < symbol_size; ++i)
        {
            const uint32 bit = symbol_size - i - 1u;

            // copy the i-th bit-plane to the output
            priv::device_assign(
                string_len,
                thrust::make_transform_iterator(
                    thrust::device_ptr<symbol_type>( sort_buffers.current_keys() ),
                    select_bit_functor<symbol_type>(bit) ),
                out_tree.bits() + string_len * i );

            // sort by the leading i+1 bits
            sort_enactor.sort( string_len, sort_buffers, bit, symbol_size );
        }

        // setup the pointer to the fully sorted string
        sorted_keys = sort_buffers.selector ? 
            sorted_string.begin() + string_len :
            sorted_string.begin();
    }
    else
    {
        // copy the input to the temporary sorting string
        thrust::copy( string, string + string_len, sorted_string.begin() );

        // loop through all bit planes in the range [0,symbol_size)
        for (uint32 i = 0; i < symbol_size; ++i)
        {
            const uint32 bit = symbol_size - i - 1u;

            // copy the i-th bit-plane to the output
            nvbio::assign(
                string_len,
                thrust::make_transform_iterator(
                    sorted_string.begin(),
                    select_bit_functor<symbol_type>(bit) ),
                out_tree.bits() + string_len * i );

            // extract the leading bits
            nvbio::transform<system_tag>(
                string_len,
                sorted_string.begin(),
                sorted_string.begin() + string_len,
                leading_bits<symbol_type>( i + 1u ) );

            // sort by the leading i+1 bits
            thrust::sort_by_key( sorted_string.begin() + string_len, sorted_string.begin() + string_len, sorted_string.begin() );
        }

        // setup the pointer to the fully sorted string
        sorted_keys = sorted_string.begin();
    }

    //
    // now compute the Wavelet Tree's leaf offsets, representing integer
    // offsets to the symbol runs in the lexicographically sorted string:
    // in practice, this is equivalent to the prefix-sum of the symbol frequencies;
    // for example, if the input string is '0301133' over a 2-bit alphabet, the
    // sorted string will be '0011333', and the output offsets will be [0,2,4,4,7],
    // as there are two 0's, two 1's, zero 2's, and three 3's.
    //

    const uint32 n_symbols = 1u << symbol_size;

    nvbio::vector<system_tag,index_type> leaves( n_symbols + 1u );

    // the offset to the first node is always zero
    leaves[0] = 0;

    // now find the offset to each symbol greater than zero
    nvbio::upper_bound<system_tag>(
        n_symbols,
        thrust::make_counting_iterator<index_type>(0u),
        string_len,
        sorted_keys,
        leaves.begin() + 1u );

    // build the binary heap tree of split points
    //
    // e.g. in the example above, this will be: [4, 2, 4]
    //  corresponding to the splits at symbols:  2, 1, 3
    //  i.e:
    //    nodes[0] = leaves[ 2 = split[0,4] ]; // root
    //    nodes[1] = leaves[ 1 = split[0,2] ]; // 1st child
    //    nodes[2] = leaves[ 3 = split[2,4] ]; // 2nd child
    //

    typedef priv::wtree::index_range  index_range;
    typedef priv::wtree::symbol_range symbol_range;
    typedef priv::wtree::node_type    node_type;

    std::stack<node_type> stack;
    stack.push( node_type( 0u, 0u, std::make_pair( 0u, n_symbols ), std::make_pair( 0u, string_len ) ) );

    nvbio::vector<host_tag,index_type> h_splits( n_symbols - 1u );
    nvbio::vector<host_tag,index_type> h_lookups( n_symbols - 1u );
    nvbio::vector<host_tag,index_type> h_leaves( leaves );

    // visit all the nodes in the tree
    while (!stack.empty())
    {
        // fetch the top of the stack
        node_type node = stack.top();
        stack.pop();

        index_range  i_range = node.i_range;
        symbol_range s_range = node.s_range;

        // check whether this is a leaf node
        if (s_range.second - s_range.first == 1)
            continue;

        // calculate the left and right child indices
        const uint32 l_node_index = node.index*2u + 1u;
        const uint32 r_node_index = node.index*2u + 2u;
        const uint32 s_split = (s_range.second + s_range.first)/2u;
        const uint32 i_split = h_leaves[ s_split ]; // # symbols preceeding 's_split

        // write out the split point for this node
        h_splits[ node.index ] = index_type( i_split );

        // store the this node's beginning index in the global bit vector
        h_lookups[ node.index ] = index_type( i_range.first + node.level * string_len );

        // push the children onto the stack
        stack.push( node_type( node.level + 1u, r_node_index, std::make_pair( s_split, s_range.second ), std::make_pair( i_split, i_range.second ) ) );
        stack.push( node_type( node.level + 1u, l_node_index, std::make_pair( s_range.first, s_split ),  std::make_pair( i_range.first, i_split ) ) );
    }

    // and copy it to the output
    thrust::copy( h_splits.begin(), h_splits.begin() + n_symbols - 1u, out_tree.splits() );

    // build the rank dictionary structure
    {
        // copy the nodes to the device
        nvbio::vector<system_tag,index_type> lookups( h_lookups );
        nvbio::vector<system_tag,uint8>      temp_storage;

        typedef typename bit_iterator::storage_iterator words_iterator;

        const words_iterator words = out_tree.bits().stream();
        const uint32 n_words = util::divide_ri( string_len * symbol_size, 32u );

        // compute the exclusive sum of the popcount of all the words in the bit-stream
        nvbio::exclusive_scan(
            n_words,
            thrust::make_transform_iterator(
                words,
                popc_functor<uint32>() ),
            out_tree.occ() + n_symbols,
            thrust::plus<index_type>(),
            0u,
            temp_storage );

        const wavelet_ranker<wavelet_tree_view_type,occ_iterator> ranker(
            plain_view( out_tree ),
            out_tree.occ() + n_symbols );

        // and now build the tree structure by doing simple lookups
        nvbio::transform<system_tag>(
            n_symbols - 1u,             // n
            lookups.begin(),            // input
            out_tree.occ(),             // output
            ranker );                   // functor

        out_tree.occ()[ n_symbols ] = 0u;
    }
}

// return the i-th symbol
//
template <typename BitStreamIterator, typename IndexIterator, typename SymbolType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
SymbolType WaveletTree<BitStreamIterator,IndexIterator,SymbolType>::operator[] (const index_type i) const
{
    return text( *this, i );
}

// return the number of bits set to b in the range [0,r] within node n at level l
//
template <typename BitStreamIterator, typename IndexIterator, typename SymbolType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename WaveletTree<BitStreamIterator,IndexIterator,SymbolType>::index_type
WaveletTree<BitStreamIterator,IndexIterator,SymbolType>::rank(const uint32 l, const uint32 node, const index_type node_begin, const index_type r, const uint8 b) const
{
    const uint32 n_nodes = 1u << symbol_size();

    // the global index of the beginning of the node is given by its local index into its level,
    // plus the global offset of the level into the bit string, which contains size() symbols
    // per level
    const uint32 global_node_begin = node_begin + l * size();

    const index_type ones  = m_occ[ node ];                         // # of occurrences of 1's preceding the node's beginning
    const index_type zeros = global_node_begin - ones;              // # of occurrences of 0's preceding the node's beginning
    const index_type offset = b ? ones : zeros;                     // number of occurrences of b at the node's beginning


    const index_type global_index = nvbio::min(
        global_node_begin + r,                                      // the global position of r in the bit-string
        size() * (l+1u) - 1u );                                     // maximum index for this level
    const index_type global_index_mod = ~global_index & 31u;

    const index_type base_index = global_index / 32u;               // the index of this occurrence counter block
    const index_type base_occ   = b ?                  m_occ[ base_index + n_nodes ] :
                                      base_index*32u - m_occ[ base_index + n_nodes ];

    const uint32     word       = bits().stream()[ base_index ];             // the block's corresponding word
    const uint32     word_occ   = popc_nbit<1u>( word, b, global_index_mod );// the inclusive popcount

    return base_occ + word_occ - offset;
}

// \relates WaveletTree
// fetch the number of occurrences of character c in the substring [0,i]
//
// \param dict         the rank dictionary
// \param i            the end of the query range [0,i]
// \param c            the query character
//
template <typename BitStreamIterator, typename IndexIterator, typename SymbolType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename WaveletTree<BitStreamIterator,IndexIterator,SymbolType>::index_type
rank(
    const          WaveletTree<BitStreamIterator,IndexIterator,SymbolType>&             tree,
    const typename WaveletTree<BitStreamIterator,IndexIterator,SymbolType>::index_type  i,
    const uint32                                                                        c)
{
    typedef typename WaveletTree<BitStreamIterator,IndexIterator,SymbolType>::index_type index_type;

    const uint32 symbol_size = tree.symbol_size();

    // traverse the tree from the root node down to the leaf containing c
    uint32     node     = 0u;
    index_type range_lo = 0u;
    index_type range_hi = tree.size();

    index_type r = i+1;
    
    for (uint32 l = 0; l < symbol_size; ++l)
    {
        // we got to an empty node, the rank must be zero
        if (range_lo == range_hi)
            return 0u;

        // select the l-th level bit of c
        const uint32 b = (c >> (symbol_size - l - 1u)) & 1u;

        // r is the new relative rank of c within the child node
        r = r ? tree.rank( l, node, range_lo, r-1, b ) : 0u;

        // compute the base (i.e. left) child node
        const uint32 child = node*2u + 1u;

        const uint32 split = tree.splits()[ node ];

        if (b == 1)
        {
            // descend into the right node
            range_lo = split;
            node = child + 1u;
        }
        else
        {
            //  descend into the left node
            range_hi = split;
            node     = child;
        }
    }
    return r;
}

// \relates WaveletTree
// fetch the number of occurrences of character c in the substring [0,i]
//
// \param dict         the rank dictionary
// \param i            the end of the query range [0,i]
// \param c            the query character
//
template <typename BitStreamIterator, typename IndexIterator, typename SymbolType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename WaveletTree<BitStreamIterator,IndexIterator,SymbolType>::range_type
rank(
    const          WaveletTree<BitStreamIterator,IndexIterator,SymbolType>&             tree,
    const typename WaveletTree<BitStreamIterator,IndexIterator,SymbolType>::range_type  range,
    const uint32                                                                        c)
{
    return make_vector(
        rank( tree, range.x, c ),
        rank( tree, range.y, c ) );
}

// \relates WaveletTree
// fetch the text character at position i in the rank dictionary
//
template <typename BitStreamIterator, typename IndexIterator, typename IndexType, typename SymbolType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
SymbolType text(const WaveletTree<BitStreamIterator,IndexIterator,SymbolType>& tree, const IndexType i)
{
    const uint32 symbol_size = tree.symbol_size();
    const uint32 string_len  = tree.size();

    // traverse the tree from the root node down to the leaf containing c
    uint32    node     = 0u;
    IndexType range_lo = 0u;

    IndexType r = i;

    SymbolType c = 0;

    for (uint32 l = 0; l < symbol_size; ++l)
    {
        // read the character in position r at level l
        const uint32 b = tree.bits()[ r + range_lo + string_len*l ];

        // insert b at the proper level in c
        c |= b << (symbol_size - l - 1u);

        // r is the new relative rank of c within the child node
        r = r ? tree.rank( l, node, range_lo, r-1, b ) : 0u;

        // compute the base (i.e. left) child node
        const uint32 child = node*2u + 1u;

        if (b == 1)
        {
            // descend into the right node
            range_lo = tree.splits()[ node ];
            node = child + 1u;
        }
        else
        {
            //  descend into the left node
            node = child;
        }
    }
    return c;
}

} // namespace nvbio
