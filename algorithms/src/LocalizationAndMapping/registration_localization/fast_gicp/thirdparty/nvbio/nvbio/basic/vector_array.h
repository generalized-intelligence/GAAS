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
#include <nvbio/basic/thrust_view.h>
#include <nvbio/basic/atomics.h>
#include <nvbio/basic/vector.h>   // thrust_copy_vector

namespace nvbio {

/// \page vector_arrays_page Vector Arrays
///
/// This module implements the notion of <i>vector array</i>, i.e. an array of dynamically-allocated vectors.
/// The ideas is that one can allocate some shared arena, and then carve N vectors from the arena
/// in parallel.
/// This data-structure provides methods to perform the carving and remember the binding between the i-th
/// array and its slot in the arena.
///
/// \section AtAGlanceSection At a Glance
///
/// There's two flavors of this class, one for the host and one for the device:
///
/// - HostVectorArray
/// - DeviceVectorArray
///
/// The DeviceVectorArray is a container meant to be used from the host; the corresponding view:
///
/// - VectorArrayView
///
/// can be obtained with a call to the plain_view() function.
///
/// \section Example
///
///\code
/// __global__ void my_alloc_kernel(VectorArrayView<uint32> vector_array)
/// {
///     const uint32 idx = threadIdx.x + blockIdx.x * blockDim.x;
///     const uint32 size = threadIdx.x+1;
///
///     // alloc this thread's vector
///     vector_array.alloc(
///         idx,           // vector to allocate
///         size );        // vector size
/// }
/// __global__ void my_other_kernel(VectorArrayView<uint32> vector_array)
/// {
///     const uint32 idx = threadIdx.x + blockIdx.x * blockDim.x;
///
///     // and do something with it
///     do_something( vector_array[i] );
/// }
///
/// DeviceVectorArray<uint32> vector_array( 32, 32*32 );
///
/// my_alloc_kernel<<<1,32>>>( plain_view( vector_array ) );
/// my_other_kernel<<<1,32>>>( plain_view( vector_array ) );
///\endcode
///
/// \section TechnicalOverviewSection Technical Overview
///
/// See the \ref VectorArrayModule module documentation.
///

///@addtogroup Basic
///@{

///\defgroup VectorArrayModule Vector Arrays
///
/// This module implements the notion of <i>vector array</i>, i.e. an array of dynamically-allocated vectors.
/// See \ref vector_arrays_page.
///

///@addtogroup VectorArrayModule
///@{

///
/// A utility class to manage a vector of dynamically-allocated arrays
///
template <typename T>
struct VectorArrayView
{
    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    VectorArrayView(
        T*      arena = NULL,
        uint32* index = NULL,
        uint32* sizes = NULL,
        uint32* pool  = NULL,
        uint32  size  = 0u)
        : m_arena(arena), m_index(index), m_sizes(sizes), m_pool(pool), m_size(size) {}

    /// alloc the vector bound to the given index
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    T* alloc(const uint32 index, const uint32 size)
    {
        const uint32 slot = atomic_add( m_pool, size );
        if (slot + size >= m_size)
        {
            // mark an out-of-bounds allocation
            m_index[index] = m_size;
            m_sizes[index] = 0u;
            return NULL;
        }
        m_index[index] = slot;
        m_sizes[index] = size;
        return m_arena + slot;
    }

    /// return the vector corresponding to the given index
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    T* operator[](const uint32 index) const
    {
        // for unsuccessful allocations m_index is set to m_size - in that case we return NULL
        return (m_index[index] < m_size) ? m_arena + m_index[index] : NULL;
    }

    /// return the slot corresponding to the given index
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 slot(const uint32 index) const { return m_index[index]; }

    /// return the size of the given array
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size(const uint32 index) const { return m_sizes[index]; }

public:
    T*      m_arena;        ///< memory arena
    uint32* m_index;        ///< index of the allocated arrays
    uint32* m_sizes;        ///< sizes of the allocated arrays
    uint32* m_pool;         ///< pool counter
    uint32  m_size;         ///< size of the arena
};

///
/// A utility class to manage a vector of dynamically-allocated arrays
///
template <typename T>
struct DeviceVectorArray
{
    typedef device_tag            system_tag;
    typedef VectorArrayView<T>    device_view_type;  ///< this object's plain view type
    typedef VectorArrayView<T>    plain_view_type;  ///< this object's plain view type

    /// constructor
    ///
    DeviceVectorArray() : m_pool(1,0) {}

    /// resize the arena
    ///
    /// \param size      size of the array (i.e. number of vectors)
    /// \param arena     size of the memory arena
    /// \param do_alloc  a flag to indicate whether to really perform allocations;
    ///                  if false, the function will just return the amount of memory
    ///                  needed
    ///
    /// \return          amount of memory allocated/needed
    ///
    uint64 resize(const uint32 size, const uint32 arena, const bool do_alloc = true)
    {
        uint64 bytes = 0;
        if (do_alloc) m_arena.resize( arena ); bytes += sizeof(T)*arena;
        if (do_alloc) m_index.resize( size );  bytes += sizeof(uint32)*size;
        if (do_alloc) m_sizes.resize( size );  bytes += sizeof(uint32)*size;
        if (do_alloc)
        {
            // initialize all slots
            thrust::fill(
                m_index.begin(),
                m_index.begin() + size,
                arena );

            // initialize all slots
            thrust::fill(
                m_sizes.begin(),
                m_sizes.begin() + size,
                uint32(0) );
        }
        return bytes;
    }

    /// check for overlows
    ///
    bool has_overflown() { return (m_pool[0] > m_arena.size()); }

    /// clear the pool
    ///
    void clear() { m_pool[0] = 0; }

    /// return number of vectors
    ///
    uint32 size() const { return m_index.size(); }

    /// return allocated size
    ///
    uint32 allocated_size() const { return m_pool[0]; }

    /// return allocated size
    ///
    uint32 arena_size() const { return m_arena.size(); }

    /// copy operator
    ///
    DeviceVectorArray& operator=(const DeviceVectorArray<T>& vec)
    {
        cuda::thrust_copy_vector( m_arena, vec.m_arena );
        cuda::thrust_copy_vector( m_index, vec.m_index );
        cuda::thrust_copy_vector( m_sizes, vec.m_sizes );
        cuda::thrust_copy_vector( m_pool,  vec.m_pool );
        return *this;
    }

    /// swap
    ///
    DeviceVectorArray& swap(DeviceVectorArray<T>& vec)
    {
        m_arena.swap( vec.m_arena );
        m_index.swap( vec.m_index );
        m_sizes.swap( vec.m_sizes );
        m_pool.swap( vec.m_pool );
        return *this;
    }

    /// return the device view of this object
    ///
    device_view_type device_view()
    {
        return VectorArrayView<T>(
            nvbio::device_view( m_arena ),
            nvbio::device_view( m_index ),
            nvbio::device_view( m_sizes ),
            nvbio::device_view( m_pool ),
            uint32( m_arena.size() ) );
    }

    /// return the plain view of this object
    ///
    plain_view_type plain_view()
    {
        return VectorArrayView<T>(
            nvbio::plain_view( m_arena ),
            nvbio::plain_view( m_index ),
            nvbio::plain_view( m_sizes ),
            nvbio::plain_view( m_pool ),
            uint32( m_arena.size() ) );
    }

    thrust::device_vector<T>        m_arena;        ///< memory arena
    thrust::device_vector<uint32>   m_index;        ///< index of the allocated arrays
    thrust::device_vector<uint32>   m_sizes;        ///< sizes of the allocated arrays
    thrust::device_vector<uint32>   m_pool;         ///< pool counter
};

///
/// A utility class to manage a vector of dynamically-allocated arrays
///
template <typename T>
struct HostVectorArray
{
    typedef device_tag            system_tag;
    typedef VectorArrayView<T>    plain_view_type;  ///< this object's plain view type

    /// constructor
    ///
    HostVectorArray() : m_pool(1,0) {}

    /// resize the arena
    ///
    /// \param size      size of the array (i.e. number of vectors)
    /// \param arena     size of the memory arena
    /// \param do_alloc  a flag to indicate whether to really perform allocations;
    ///                  if false, the function will just return the amount of memory
    ///                  needed
    ///
    /// \return          amount of memory allocated/needed
    ///
    uint64 resize(const uint32 size, const uint32 arena, const bool do_alloc = true)
    {
        uint64 bytes = 0;
        if (do_alloc) m_arena.resize( arena ); bytes += sizeof(T)*arena;
        if (do_alloc) m_index.resize( size );  bytes += sizeof(uint32)*size;
        if (do_alloc) m_sizes.resize( size );  bytes += sizeof(uint32)*size;
        if (do_alloc)
        {
            // initialize all slots
            thrust::fill(
                m_index.begin(),
                m_index.begin() + size,
                arena );

            // initialize all slots
            thrust::fill(
                m_sizes.begin(),
                m_sizes.begin() + size,
                uint32(0) );
        }
        return bytes;
    }

    /// check for overlows
    ///
    bool has_overflown() { return (m_pool[0] > m_arena.size()); }

    /// clear the pool
    ///
    void clear() { m_pool[0] = 0; }

    /// return number of vectors
    ///
    uint32 size() const { return m_index.size(); }

    /// return allocated size
    ///
    uint32 allocated_size() const { return m_pool[0]; }

    /// copy operator
    ///
    HostVectorArray& operator=(const DeviceVectorArray<T>& vec)
    {
        cuda::thrust_copy_vector( m_arena, vec.m_arena );
        cuda::thrust_copy_vector( m_index, vec.m_index );
        cuda::thrust_copy_vector( m_sizes, vec.m_sizes );
        cuda::thrust_copy_vector( m_pool,  vec.m_pool );
        return *this;
    }

    /// swap
    ///
    HostVectorArray& swap(HostVectorArray<T>& vec)
    {
        m_arena.swap( vec.m_arena );
        m_index.swap( vec.m_index );
        m_sizes.swap( vec.m_sizes );
        m_pool.swap( vec.m_pool );
        return *this;
    }

    /// return the vector corresponding to the given index
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const T* operator[](const uint32 index) const
    {
        // for unsuccessful allocations m_index is set to m_size - in that case we return NULL
        return (m_index[index] < m_arena.size()) ? &m_arena[0] + m_index[index] : NULL;
    }

    /// return the slot corresponding to the given index
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 slot(const uint32 index) const { return m_index[index]; }

    /// return the plain view of this object
    ///
    plain_view_type plain_view()
    {
        return VectorArrayView<T>(
            nvbio::plain_view( m_arena ),
            nvbio::plain_view( m_index ),
            nvbio::plain_view( m_sizes ),
            nvbio::plain_view( m_pool ),
            uint32( m_arena.size() ) );
    }

    thrust::host_vector<T>        m_arena;        ///< memory arena
    thrust::host_vector<uint32>   m_index;        ///< index of the allocated arrays
    thrust::host_vector<uint32>   m_sizes;        ///< sizes of the allocated arrays
    thrust::host_vector<uint32>   m_pool;         ///< pool counter
};

///\relates DeviceVectorArray
/// return a view of the queues
///
template <typename T>
VectorArrayView<T> device_view(DeviceVectorArray<T>& vec) { return vec.device_view(); }

///\relates DeviceVectorArray
/// return a view of the queues
///
template <typename T>
VectorArrayView<T> plain_view(DeviceVectorArray<T>& vec) { return vec.device_view(); }

///\relates DeviceVectorArray
/// return a view of the queues
///
template <typename T>
VectorArrayView<T> plain_view(HostVectorArray<T>& vec) { return vec.plain_view(); }

///@} // VectorArrayModule
///@} Basic

} // namespace nvbio
