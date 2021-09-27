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

// return the number of nodes corresponding to a given number of leaves
//
template <typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 SumTree<Iterator>::node_count(const uint32 size)
{
    const uint32 log_size = nvbio::log2( size );
    const uint32 padded_size =
        (1u << log_size) < size ?
            1u << (log_size+1u) :
            1u <<  log_size;

    return padded_size * 2u - 1u;
}

// constructor
//
template <typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
SumTree<Iterator>::SumTree(const uint32 size, iterator_type cells) :
    m_cells( cells ),
    m_size( size ),
    m_padded_size(
        (1u << nvbio::log2( size )) < size ?
            1u << (nvbio::log2( size )+1u) :
            1u <<  nvbio::log2( size ) )
{}

// setup the tree structure
//
template <typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void SumTree<Iterator>::setup(const value_type zero)
{
    // zero out padding cells
    for (uint32 i = m_size; i < m_padded_size; ++i)
        m_cells[i] = zero;

    uint32 src = 0;

    for (uint32 n = m_padded_size; n >= 2; n >>= 1)
    {
        const uint32 dst = src + n;

        const uint32 m = n >> 1;
        for (uint32 i = 0; i < m; ++i)
            m_cells[ dst + i ] = (m_cells[ src + i*2 ] + m_cells[ src + i*2 + 1u ]);

        src += n;
    }
}

// increment a cell's value
//
template <typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void SumTree<Iterator>::add(const uint32 i, const value_type v)
{
    uint32 dst = 0;
    uint32 j   = i;
    for (uint32 m = m_padded_size; m >= 2; m >>= 1, j >>= 1)
    {
        m_cells[ dst + j ] += v;

        dst += m;
    }
}

// reset a cell's value
//
template <typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void SumTree<Iterator>::set(const uint32 i, const value_type v)
{
    // sum the left & right values of each ancestor bottom-up
    m_cells[ i ] = v;

    uint32 prev_base   = 0u;
    uint32 parent_base = m_padded_size;
    uint32 parent      = i >> 1;

    for (uint32 m = m_padded_size >> 1; parent_base + parent < nodes(); m >>= 1)
    {
        m_cells[ parent_base + parent ] =
            m_cells[ prev_base + parent*2   ] +
            m_cells[ prev_base + parent*2+1 ];

        prev_base     = parent_base;
        parent_base  += m;
        parent      >>= 1;
    }
}

// sample a cell from a linear tree
//
template <typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint32 sample(const SumTree<Iterator>& tree, const float value)
{
    const uint32 padded_size = tree.padded_size();

    // compute the offset
    uint32 node_base  = padded_size*2u - 4u;
    uint32 node_index = 0;

    float v = value;

    // choose the proper child of each internal node, from the root down to the leaves.
    for (uint32 m = 2; m < padded_size; m *= 2)
    {
        // choose the proper node among the selected pair.
        const float l = float(tree.cell( node_base + node_index      ));
        const float r = float(tree.cell( node_base + node_index + 1u ));
        const float sum = float( l + r );

        if (sum == 0.0f)
            node_index *= 2;
        else
        {
            if (v * sum < l || r == 0.0f)
            {
                node_index = node_index * 2u;
                v = nvbio::min( v * sum / l, 1.0f );
            }
            else
            {
                node_index = (node_index + 1u) * 2u;
                v = nvbio::min( (v*sum - l) / r, 1.0f );
            }
        }

        node_base -= m*2;
    }

    // level 0
    const uint32 size = tree.size();
    {
        // choose the proper leaf among the selected pair.
        const float l = node_index      < size ? float(tree.cell( node_index ))      : 0.0f;
        const float r = node_index + 1u < size ? float(tree.cell( node_index + 1u )) : 0.0f;
        const float sum = float( l + r );

        node_index = (v * sum < l || r == 0.0f) ? node_index : node_index + 1u;
    }

    // clamp the leaf index to the tree size
    return node_index < size ? node_index : size - 1u;
}

} // namespace nvbio
