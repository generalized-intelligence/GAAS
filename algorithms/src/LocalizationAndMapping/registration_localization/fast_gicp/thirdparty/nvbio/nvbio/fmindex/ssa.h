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
#include <nvbio/basic/popcount.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/ldg.h>
#include <vector_types.h>
#include <vector_functions.h>
#include <cuda_runtime.h>
#include <vector>
#include <stdexcept>
#include <thrust/device_vector.h>

namespace nvbio {

///@addtogroup FMIndex
///@{

///\defgroup SSAModule Sampled Suffix Arrays
///
/// A <i>Sampled Suffix Array</i> is a succint suffix array which has been sampled at a subset of
/// its indices. Ferragina & Manzini showed that given such a data structure and the BWT of the
/// original text it is possible to reconstruct the missing locations.
/// Two such data structures have been proposed, with different tradeoffs:
///
///  - one storing only the entries that are a multiple of K, { SA[i] : (SA[i] = 0 mod K) }
///  - one storing only the entries whose index is a multiple of K, { SA[i] : (i = 0 mod K) }
///
/// NVBIO provides both:
///
///  - SSA_value_multiple
///  - SSA_index_multiple
///
/// Unlike for the rank_dictionary, which is a storage-free class, these classes own the (internal) storage
/// needed to represent the underlying data structures, which resides on the host.
/// Similarly, the module provides some counterparts that hold the corresponding storage for the device:
///
///  - SSA_value_multiple_device
///  - SSA_index_multiple_device
///
/// While these classes hold device data, they are meant to be used from the host and cannot be directly
/// passed to CUDA kernels.
/// Plain views (see \ref host_device_page), or <i>contexts</i>, can be obtained with the usual plain_view() function.
///
/// The contexts expose the following SSA interface:
/// \anchor SSAInterface
///
/// \code
/// struct SSA
/// {
///     // return the i-th suffix array entry, if present
///     bool fetch(const uint32 i, uint32& r) const
///
///     // return whether the i-th suffix array is present
///     bool has(const uint32 i) const
/// }
/// \endcode
///

///@addtogroup SSAModule
///@{

struct SSA_value_multiple_device;

template <uint32 K, typename index_type = uint32>
struct SSA_index_multiple_device;

///
/// A simple context to access a SSA_value_multiple structure - a model of \ref SSAInterface.
///
template <typename SSAIterator, typename BitmaskIterator, typename BlockIterator>
struct SSA_value_multiple_context
{
    /// empty constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE SSA_value_multiple_context() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE SSA_value_multiple_context(
        SSAIterator     ssa,
        BitmaskIterator bitmask,
        BlockIterator   blocks) : m_ssa( ssa ), m_bitmask( bitmask ), m_blocks( blocks ) {}

    /// fetch the i-th value, if stored, return false otherwise.
    ///
    /// \param i        requested entry
    /// \param r        result value
    /// \return         true if present, false otherwise
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool fetch(const uint32 i, uint32& r) const;

    /// check if the i-th value is present
    ///
    /// \param i        requested entry
    /// \return         true if present, false otherwise
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool has(const uint32 i) const;

    SSAIterator     m_ssa;
    BitmaskIterator m_bitmask;
    BlockIterator   m_blocks;
};

///
/// Build a sampled suffix array storing only the values which
/// are a multiple of K, i.e. { SA[i] | SA[i] % K = 0 }
///
struct SSA_value_multiple
{
    typedef SSA_value_multiple_context<
        const uint32*,
        const uint32*,
        const uint32*>                  context_type;
    typedef SSA_value_multiple_device   device_type;
    typedef context_type                device_view_type;
    typedef context_type                plain_view_type;

    /// empty constructor
    ///
    SSA_value_multiple() : m_n(0), m_stored(0) {}

    /// constructor
    ///
    /// \param n        number of entries in the SA
    /// \param sa       suffix array
    /// \param K        compression factor
    SSA_value_multiple(
        const uint32  n,
        const int32*  sa,
        const uint32  K);

    /// constructor
    ///
    /// \param fmi      FM index
    /// \param K        compression factor
    template <typename FMIndexType>
    SSA_value_multiple(
        const FMIndexType& fmi,
        const uint32  K);

    /// get a context
    ///
    context_type get_context() const
    {
        return context_type( &m_ssa[0], &m_bitmask[0], &m_blocks[0] );
    }

    uint32  m_n;
    uint32  m_stored;
    std::vector<uint32> m_ssa;
    std::vector<uint32> m_bitmask;
    std::vector<uint32> m_blocks;

private:
    uint32 index(const uint32 i) const;
};

struct SSA_value_multiple_device
{
    typedef SSA_value_multiple_context<
        cuda::ldg_pointer<uint32>,
        cuda::ldg_pointer<uint32>,
        cuda::ldg_pointer<uint32> >     context_type;
    typedef context_type                device_view_type;
    typedef context_type                plain_view_type;

    /// constructor
    ///
    /// \param ssa      host sampled suffix array
    SSA_value_multiple_device(const SSA_value_multiple& ssa);

    /// destructor
    ///
    ~SSA_value_multiple_device();

    /// get a context
    ///
    context_type get_context() const
    {
        return context_type( cuda::ldg_pointer<uint32>(m_ssa), cuda::ldg_pointer<uint32>(m_bitmask), cuda::ldg_pointer<uint32>(m_blocks) );
    }

    uint32  m_n;
    uint32  m_stored;
    uint32* m_ssa;
    uint32* m_bitmask;
    uint32* m_blocks;
};

///
/// A simple context to access a SSA_index_multiple structure - a model of \ref SSAInterface.
///
template <uint32 K, typename Iterator = const uint32*>
struct SSA_index_multiple_context
{
    typedef typename std::iterator_traits<Iterator>::value_type index_type;
    typedef index_type                                          value_type;

    /// empty constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE SSA_index_multiple_context() {}

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE SSA_index_multiple_context(
        const Iterator ssa) : m_ssa( ssa ) {}

    /// fetch the i-th value, if stored, return false otherwise.
    ///
    /// \param i        requested entry
    /// \param r        result value
    /// \return         true if present, false otherwise
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool fetch(const index_type i, index_type& r) const;

    /// check if the i-th value is present
    ///
    /// \param i        requested entry
    /// \return         true if present, false otherwise
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool has(const index_type i) const;

    Iterator m_ssa;
};

///
/// Build a sampled suffix array storing only the values at positions
/// which are a multiple of K, i.e. { SA[i] | i % K = 0 }
///
template <uint32 K, typename index_type = uint32>
struct SSA_index_multiple
{
    typedef index_type                                          value_type;
    typedef SSA_index_multiple_context<K, const index_type*>    context_type;
    typedef SSA_index_multiple_device<K,index_type>             device_type;
    typedef context_type                                        device_view_type;
    typedef context_type                                        plain_view_type;

    /// empty constructor
    ///
    SSA_index_multiple() : m_n(0) {}

    /// constructor
    ///
    /// \param n        number of entries in the SA
    /// \param sa       suffix array
    SSA_index_multiple(
        const index_type  n,
        const index_type* sa);

    /// constructor
    ///
    /// \param fmi      FM index
    /// \param K        compression factor
    template <typename FMIndexType>
    SSA_index_multiple(
        const FMIndexType& fmi);

    /// constructor
    ///
    /// \param ssa      device-side ssa
    SSA_index_multiple(
        const SSA_index_multiple_device<K,index_type>& ssa);

    /// copy operator
    ///
    /// \param ssa      device-side ssa
    SSA_index_multiple& operator=(
        const SSA_index_multiple_device<K,index_type>& ssa);

    /// get a context
    ///
    context_type get_context() const { return context_type( &m_ssa[0] ); }

    index_type              m_n;
    std::vector<index_type> m_ssa;
};

template <uint32 K, typename index_type>
struct SSA_index_multiple_device
{
    typedef index_type                                          value_type;
    typedef SSA_index_multiple_context<K, const index_type*>    context_type;
    typedef context_type                                        device_view_type;
    typedef context_type                                        plain_view_type;

    /// empty constructor
    ///
    SSA_index_multiple_device() : m_n(0) {}

    /// constructor
    ///
    /// \param ssa      host sampled suffix array
    SSA_index_multiple_device(const SSA_index_multiple<K, index_type>& ssa);

#ifdef __CUDACC__
    /// constructor
    ///
    /// \param fmi      FM-index
    template <typename FMIndexType>
    SSA_index_multiple_device(
        const FMIndexType& fmi);
    #endif

    /// destructor
    ///
    ~SSA_index_multiple_device() {}

    #ifdef __CUDACC__
    template <typename FMIndexType>
    void init(const FMIndexType& fmi);
    #endif

    /// get a context
    ///
    context_type get_context() const { return context_type( thrust::raw_pointer_cast(&m_ssa[0]) ); }

    index_type                        m_n;
    thrust::device_vector<index_type> m_ssa;
};

/// return the plain view of a SSA_value_multiple
///
inline
SSA_value_multiple::plain_view_type plain_view(const SSA_value_multiple& vec) { return vec.get_context(); }

/// return the plain view of a SSA_value_multiple_device
///
inline
SSA_value_multiple_device::plain_view_type plain_view(const SSA_value_multiple_device& vec) { return vec.get_context(); }

/// return the plain view of a SSA_index_multiple
///
template <uint32 K, typename index_type>
typename SSA_index_multiple<K,index_type>::plain_view_type plain_view(const SSA_index_multiple<K,index_type>& vec) { return vec.get_context(); }


///@} SSAModule
///@} FMIndex

} // namespace nvbio

#include <nvbio/fmindex/ssa_inl.h>
