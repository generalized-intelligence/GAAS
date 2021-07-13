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
namespace bowtie2 {
namespace cuda {

// return the device view
//
inline
SeedHitDequeArrayDeviceView SeedHitDequeArray::device_view()
{
    return SeedHitDequeArrayDeviceView(
        nvbio::device_view( m_counts ),
        nvbio::device_view( m_index ),
        nvbio::device_view( m_hits ),
        nvbio::device_view( m_probs_index ),
        nvbio::device_view( m_probs ),
        nvbio::device_view( m_pool ),
        nvbio::device_view( m_probs_pool ) );
}

// constructor
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
SeedHitDequeArrayDeviceView::SeedHitDequeArrayDeviceView(
    index_storage_type  counts,
    index_storage_type  index,
    hits_storage_type   hits,
    index_storage_type  probs_index,
    prob_storage_type   probs,
    index_storage_type  pool,
    index_storage_type  probs_pool) :
    m_counts( counts ),
    m_hits( hits ),
    m_probs( probs ),
    m_index( index ),
    m_probs_index( probs_index ),
    m_pool( pool ),
    m_probs_pool( probs_pool )
{}

// return a reference to the given deque
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
SeedHitDequeArrayDeviceView::reference SeedHitDequeArrayDeviceView::operator[] (const uint32 read_id)
{
    return reference( *this, read_id );
}

// allocate some storage for the deque bound to a given read
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
SeedHit* SeedHitDequeArrayDeviceView::alloc_deque(const uint32 read_id, const uint32 size)
{
#if defined(__CUDA_ARCH__) && __CUDA_ARCH__ > 0
    m_counts[read_id] = 0u;
    const uint32 index       = size ? atomicAdd( m_pool, size ) : 0u;
    const uint32 probs_index = size ? atomicAdd( m_probs_pool, SumTree<float*>::node_count( size ) ) : 0u;
    if (index < m_hits.size())
    {
        m_index[read_id]       = index;
        m_probs_index[read_id] = probs_index;
        return m_hits + m_index[read_id];
    }
    else
    {
        // allocation failed
        m_index[read_id]       = 0;
        m_probs_index[read_id] = 0;
        return NULL;
    }
#else
    return NULL;
#endif
}

// return the deque bound to a given read
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename SeedHitDequeArrayDeviceView::hit_deque_type SeedHitDequeArrayDeviceView::get_deque(const uint32 read_id, bool build_heap) const
{
    hit_vector_type hit_vector( m_counts[read_id], get_data( read_id ) );
    return hit_deque_type( hit_vector, build_heap );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
