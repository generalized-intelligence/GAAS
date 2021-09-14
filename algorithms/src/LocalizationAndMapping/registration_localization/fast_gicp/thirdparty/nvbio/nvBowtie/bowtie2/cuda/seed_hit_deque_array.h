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

///
///\file seed_hit_deque_array.h
///

#pragma once

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/seed_hit.h>
#include <nvbio/basic/cuda/pingpong_queues.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/basic/priority_deque.h>
#include <nvbio/basic/sum_tree.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/vector.h>
#include <algorithm>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

///@addtogroup nvBowtie
///@{

///@addtogroup SeedHits
///@{

struct SeedHitDequeArrayDeviceView;

template <typename SeedHitDequeArrayType> struct SeedHitDequeReference;

///
/// The SeedHitDequeArray device storage class
///
struct SeedHitDequeArrayDeviceStorage
{
    typedef thrust::device_vector<SeedHit>  hits_storage_type;
    typedef thrust::device_vector<uint32>   index_storage_type;
    typedef thrust::device_vector<float>    prob_storage_type;

    hits_storage_type    m_hits;        ///< global arena of seed hit SA ranges
    index_storage_type   m_counts;      ///< per-read seed hit counters
    prob_storage_type    m_probs;       ///< global arena of SA probabilities
    index_storage_type   m_index;       ///< per-read index marking the beginning of the read's hits vector in the global arena
    index_storage_type   m_probs_index; ///< per-read index marking the beginning of the read's hits vector in the global arena
    index_storage_type   m_pool;        ///< pool counter for the global arena
    index_storage_type   m_probs_pool;  ///< pool counter for the global arena
};

///
/// The SeedHitDequeArray host storage class
///
struct SeedHitDequeArrayHostStorage
{
    typedef thrust::host_vector<SeedHit>  hits_storage_type;
    typedef thrust::host_vector<uint32>   index_storage_type;
    typedef thrust::host_vector<float>    prob_storage_type;

    /// copy constructor
    ///
    SeedHitDequeArrayHostStorage(const SeedHitDequeArrayDeviceStorage& other) { (*this)= other; }

    /// copy from a device object
    ///
    SeedHitDequeArrayHostStorage& operator=(const SeedHitDequeArrayDeviceStorage& other)
    {
        nvbio::cuda::thrust_copy_vector( m_hits,        other.m_hits );
        nvbio::cuda::thrust_copy_vector( m_counts,      other.m_counts );
        nvbio::cuda::thrust_copy_vector( m_probs,       other.m_probs );
        nvbio::cuda::thrust_copy_vector( m_index,       other.m_index );
        nvbio::cuda::thrust_copy_vector( m_probs_index, other.m_probs_index );
        nvbio::cuda::thrust_copy_vector( m_pool,        other.m_pool );
        nvbio::cuda::thrust_copy_vector( m_probs_pool,  other.m_probs_pool );
        return *this;
    }

    hits_storage_type    m_hits;        ///< global arena of seed hit SA ranges
    index_storage_type   m_counts;      ///< per-read seed hit counters
    prob_storage_type    m_probs;       ///< global arena of SA probabilities
    index_storage_type   m_index;       ///< per-read index marking the beginning of the read's hits vector in the global arena
    index_storage_type   m_probs_index; ///< per-read index marking the beginning of the read's hits vector in the global arena
    index_storage_type   m_pool;        ///< pool counter for the global arena
    index_storage_type   m_probs_pool;        ///< pool counter for the global arena
};

///
/// An array containing a priority deque of SeedHit's per read
///
struct SeedHitDequeArray : public SeedHitDequeArrayDeviceStorage
{
    typedef SeedHitDequeArrayDeviceStorage::hits_storage_type  hits_storage_type;
    typedef SeedHitDequeArrayDeviceStorage::index_storage_type index_storage_type;
    typedef SeedHitDequeArrayDeviceStorage::prob_storage_type  prob_storage_type;

    typedef SeedHitDequeArrayDeviceView     device_view_type;

    /// resize the arena
    ///
    /// \return     # of allocated bytes
    uint64 resize(const uint32 n_reads, const uint32 max_hits, const bool do_alloc = true);

    /// clear all deques
    ///
    void clear_deques();

    /// return the device view
    ///
    SeedHitDequeArrayDeviceView device_view();

    /// return the hits vector
    ///
    hits_storage_type& hits() { return m_hits; }

    /// return the counts vector
    ///
    index_storage_type& counts() { return m_counts; }

    /// return the index vector
    ///
    index_storage_type& index() { return m_index; }

    /// return the probs vector
    ///
    prob_storage_type& probs() { return m_probs; }
};

///
/// An array containing a priority deque of SeedHit's per read
///
struct SeedHitDequeArrayDeviceView
{
    typedef typename device_view_subtype< thrust::device_vector<SeedHit> >::type  hits_storage_type;
    typedef typename device_view_subtype< thrust::device_vector<uint32> >::type   index_storage_type;
    typedef typename device_view_subtype< thrust::device_vector<float> >::type    prob_storage_type;

    typedef vector_view<SeedHit*>                                 hit_vector_type;
    typedef priority_deque<SeedHit, hit_vector_type, hit_compare> hit_deque_type;

    typedef SeedHitDequeReference<SeedHitDequeArrayDeviceView>    reference;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SeedHitDequeArrayDeviceView(
        index_storage_type  counts      = index_storage_type(),
        index_storage_type  index       = index_storage_type(),
        hits_storage_type   hits        = hits_storage_type(),
        index_storage_type  probs_index = index_storage_type(),
        prob_storage_type   probs       = prob_storage_type(),
        index_storage_type  pool        = index_storage_type(),
        index_storage_type  probs_pool  = index_storage_type());

    /// return a reference to the given deque
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    reference operator[] (const uint32 read_id);

    /// allocate some storage for the deque bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SeedHit* alloc_deque(const uint32 read_id, const uint32 size);

    /// resize the deque bound to a given read
    /// NOTE: this method doesn't alloc or release any memory! alloc_deque() must have been previously called.
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void resize_deque(const uint32 read_id, const uint32 size) { m_counts[read_id] = size; }

    /// get the size of the hit vector bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 get_size(const uint32 read_id) const { return m_counts[read_id]; }

    /// get the storage for the hit vector bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SeedHit* get_data(const uint32 read_id) const { return m_hits.base() + m_index[read_id]; }

    /// get the storage for the hit vector bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    float* get_probs(const uint32 read_id) const { return m_probs.base() + m_probs_index[read_id]; }

    /// erase the set of hits bound to a read.
    /// NOTE: this method doesn't release the previously allocated memory!
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void erase(const uint32 read_id) { m_counts[read_id] = 0; }

    /// return the deque bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    hit_deque_type get_deque(const uint32 read_id, bool build_heap = false) const;

//private:
    hits_storage_type    m_hits;        ///< global arena of seed hit SA ranges
    index_storage_type   m_counts;      ///< per-read seed hit counters
    prob_storage_type    m_probs;       ///< global arena of SA probabilities
    index_storage_type   m_index;       ///< per-read index marking the beginning of the read's hits vector in the global arena
    index_storage_type   m_probs_index; ///< per-read index marking the beginning of the read's hits vector in the global arena
    index_storage_type   m_pool;        ///< pool counter for the global arena
    index_storage_type   m_probs_pool;  ///< pool counter for the global arena
};

///
/// Implements a reference to a hit deque bound to a read.
/// Provides methods for atomically allocating the deque's storage, as well as
/// a deque and plain array interface.
///
template <typename SeedHitDequeArrayType>
struct SeedHitDequeReference
{
    typedef vector_view<SeedHit*>                                 hit_vector_type;
    typedef priority_deque<SeedHit, hit_vector_type, hit_compare> hit_deque_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SeedHitDequeReference(SeedHitDequeArrayType& deques, const uint32 read_id) :
        m_deques( deques ),
        m_read_id( read_id ),
        m_deque( deques.get_deque( read_id ) ) {}

    /// destructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ~SeedHitDequeReference()
    {
        // update the size of the deque if dirty
        m_deques.resize_deque( m_read_id, m_deque.size() );
    }

    /// allocate some storage for the deque bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SeedHit* alloc(const uint32 size)
    {
        SeedHit* storage = m_deques.alloc_deque( m_read_id, size );
        m_deque = m_deques.get_deque( m_read_id ); // re-assign the deque
        return storage;
    }

    /// allocate some storage for the deque bound to a given read
    /// NOTE: this method doesn't expand or release the previously allocated memory!
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void resize(const uint32 size)
    {
        m_deques.resize_deque( m_read_id, size );
        m_deque = m_deques.get_deque( m_read_id ); // re-assign the deque
    }

    /// get the size of the hit vector bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_deque.size(); }

    /// get the storage for the hit vector bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SeedHit* get_data() const { return m_deques.get_data( m_read_id ); }

    /// get the storage for the hit vector bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    float* get_probs() const { return m_deques.get_probs( m_read_id ); }

    /// erase the set of hits bound to a read.
    /// NOTE: this method doesn't release the previously allocated memory!
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void erase() { m_deques.erase( m_read_id ); m_deque = get_deque( m_read_id ); }

    /// return the deque bound to a given read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    hit_deque_type get_deque(bool build_heap = false) const { return m_deques.get_deque( m_read_id, build_heap ); }

    /// push a seed-hit
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void push(const SeedHit hit)  { m_deque.push( hit ); }

    /// pop from the front
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void pop_top() { m_deque.pop_top(); }

    /// pop from the front
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void pop_bottom() { m_deque.pop_bottom(); }

    /// pop from the top
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void pop() { m_deque.pop(); }

    /// top
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const SeedHit& top() const { return m_deque.top(); }

    /// bottom
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const SeedHit& bottom() const { return m_deque.bottom(); }

    /// return the i-th seed-hit (unsorted)
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SeedHit& operator[] (const uint32 i) { return get_data()[i]; }

private:
    SeedHitDequeArrayType&  m_deques;       ///< reference to deque array
    uint32                  m_read_id;      ///< read id
    hit_deque_type          m_deque;        ///< local deque object for this read
};

///@}  // group SeedHits
///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio

#include <nvBowtie/bowtie2/cuda/seed_hit_deque_array_inl.h>
