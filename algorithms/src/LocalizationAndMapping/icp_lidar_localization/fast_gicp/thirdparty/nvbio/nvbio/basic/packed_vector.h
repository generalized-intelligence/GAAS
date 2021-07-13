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

#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/vector.h>


namespace nvbio {

///@addtogroup Basic
///@{

///@addtogroup PackedStreams
///@{

///
/// A class to represent a vector of packed symbols, where the size of the symbol is specified at compile-time
/// as a template parameter.
/// The sequence is packed on top of an underlying stream of 32-bit words.
///
/// \tparam SYMBOL_SIZE_T       the number of bits needed for each symbol
/// \tparam BIG_ENDIAN_T        the "endianness" of the words: if true, symbols will be packed from right to left within each word
/// \tparam IndexType           the type of integer used to address the stream (e.g. uint32, uint64)
///
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T = false, typename IndexType = uint32>
struct PackedVector
{
    static const uint32 SYMBOL_SIZE = SYMBOL_SIZE_T;
    static const uint32 BIG_ENDIAN  = BIG_ENDIAN_T;

    static const uint32 SYMBOLS_PER_WORD = 32 / SYMBOL_SIZE;
    static const uint32 VECTOR_WIDTH     = SYMBOLS_PER_WORD;

    typedef SystemTag   system_tag;
    typedef IndexType   index_type;

    typedef typename nvbio::vector<system_tag,uint32>::pointer                                pointer;
    typedef typename nvbio::vector<system_tag,uint32>::const_pointer                    const_pointer;

    typedef PackedStream<      pointer,uint8,SYMBOL_SIZE,BIG_ENDIAN,IndexType>                stream_type;
    typedef PackedStream<const_pointer,uint8,SYMBOL_SIZE,BIG_ENDIAN,IndexType>          const_stream_type;
    typedef typename stream_type::iterator                                                    iterator;
    typedef typename const_stream_type::iterator                                        const_iterator;
    typedef ForwardPackedStream<const_pointer,uint8,SYMBOL_SIZE,BIG_ENDIAN,IndexType>   forward_iterator;

    typedef uint8                                                                             value_type;
    typedef PackedStreamRef<stream_type>                                                      reference;
    typedef PackedStreamRef<const_stream_type>                                          const_reference;

    typedef stream_type                                                                       plain_view_type;
    typedef const_stream_type                                                           const_plain_view_type;

    /// constructor
    ///
    PackedVector(const index_type size = 0);

    /// copy constructor
    ///
    template <typename other_tag>
    PackedVector(const PackedVector<other_tag,SYMBOL_SIZE,BIG_ENDIAN,IndexType>& other) :
        m_storage( other.m_storage ), m_size( other.m_size ) {}

    /// reserve
    ///
    void reserve(const index_type size);

    /// resize
    ///
    void resize(const index_type size);

    /// clear
    ///
    void clear(void);

    /// size
    ///
    index_type size() const { return m_size; }

    /// length
    ///
    index_type length() const { return m_size; }

    index_type capacity() const { return m_storage.size(); }

    /// return the begin iterator
    ///
    iterator begin();

    /// return the end iterator
    ///
    iterator end();

    /// return the begin iterator
    ///
    const_iterator begin() const;

    /// return the end iterator
    ///
    const_iterator end() const;

    /// push back a symbol
    ///
    void push_back(const uint8 s);

    /// get the memory address of a symbol
    void *addrof(const index_type i);

    /// get the i-th symbol
    /// note: no device implementation for this as PackedVectors are never used on device
    /// (only their plain view, which is a PackedStream)
    NVBIO_FORCEINLINE NVBIO_HOST const typename stream_type::symbol_type operator[] (const index_type i) const
    {
        const_stream_type stream( &m_storage[0] );
        return stream[i];
    }

    /// get the i-th symbol
    /// note: no device implementation for this as PackedVectors are never used on device
    /// (only their plain view, which is a PackedStream)
    NVBIO_FORCEINLINE NVBIO_HOST typename stream_type::reference operator[] (const index_type i)
    {
        stream_type stream( &m_storage[0] );
        return stream[i];
    }

    nvbio::vector<system_tag,uint32> m_storage;
    index_type                       m_size;
};

/// return a plain view of a PackedVector object
///
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
inline
typename PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::plain_view_type
plain_view(PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& vec)
{
    typedef typename PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::plain_view_type stream;
    return stream( &vec.m_storage.front() );
}

/// return a plain view of a const PackedVector object
///
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
inline
typename PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::const_plain_view_type
plain_view(const PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& vec)
{
    typedef typename PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::const_plain_view_type stream;
    return stream( &vec.m_storage.front() );
}

///@} PackedStreams
///@} Basic

} // namespace nvbio

#include <nvbio/basic/packed_vector_inl.h>
