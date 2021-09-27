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

#include <nvBowtie/bowtie2/cuda/seed_hit_deque_array.h>
#include <nvbio/basic/sum_tree.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

// resize the arena
//
// \return     # of allocated bytes
uint64 SeedHitDequeArray::resize(const uint32 n_reads, const uint32 max_hits, const bool do_alloc)
{
    uint64 bytes = 0u;
    uint32 max_nodes = SumTree<float*>::node_count( max_hits );
    if (do_alloc) m_counts.resize( n_reads );               bytes += n_reads * sizeof(uint32);
    if (do_alloc) m_index.resize( n_reads );                bytes += n_reads * sizeof(uint32);
    if (do_alloc) m_hits.resize( n_reads * max_hits );      bytes += n_reads * max_hits * sizeof(SeedHit);
    if (do_alloc) m_probs.resize( n_reads * max_nodes );    bytes += n_reads * max_nodes * sizeof(float);
    if (do_alloc) m_probs_index.resize( n_reads );          bytes += n_reads * sizeof(uint32);
    if (do_alloc) m_pool.resize( 1, 0u );                   bytes += sizeof(uint32);
    if (do_alloc) m_probs_pool.resize( 1, 0u );             bytes += sizeof(uint32);

    if (do_alloc) thrust::fill( m_counts.begin(),       m_counts.end(),       uint32(0) );
    if (do_alloc) thrust::fill( m_index.begin(),        m_index.end(),        uint32(0) );
    if (do_alloc) thrust::fill( m_probs_index.begin(),  m_probs_index.end(),  uint32(0) );
    return bytes;
}

/// clear all deques
///
void SeedHitDequeArray::clear_deques()
{
    // reset deque size counters
    thrust::fill( m_counts.begin(),       m_counts.end(),       uint32(0) );
    thrust::fill( m_index.begin(),        m_index.end(),        uint32(0) );
    thrust::fill( m_probs_index.begin(),  m_probs_index.end(),  uint32(0) );

    m_pool[0] = 0; // reset the arena
    m_probs_pool[0] = 0; // reset the arena
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
