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

// constructor
//
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::PackedVector(const index_type size) :
    m_storage( util::divide_ri( size, SYMBOLS_PER_WORD ) ), m_size( size )
{}

// reserve
//
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
void PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::reserve(const index_type size)
{
    if (m_storage.size() < util::divide_ri( m_size, SYMBOLS_PER_WORD ))
        m_storage.resize( util::divide_ri( m_size, SYMBOLS_PER_WORD ) );
}

// resize
//
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
void PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::resize(const index_type size)
{
    m_size = size;
    reserve(size);
}


// clear
//
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
void PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::clear(void)
{
    resize(0);
}

// return the begin iterator
//
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
typename PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::iterator
PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::begin()
{
    stream_type stream( &m_storage.front() );
    return stream.begin();
}

// return the end iterator
//
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
typename PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::iterator
PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::end()
{
    stream_type stream( &m_storage.front() );
    return stream.begin() + m_size;
}

// return the begin iterator
//
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
typename PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::const_iterator
PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::begin() const
{
    const_stream_type stream( &m_storage.front() );
    return stream.begin();
}

// return the end iterator
//
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
typename PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::const_iterator
PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::end() const
{
    const_stream_type stream( &m_storage.front() );
    return stream.begin() + m_size;
}

// push back a symbol
//
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
void PackedVector<SystemTag,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>::push_back(const uint8 s)
{
    if (m_storage.size() < util::divide_ri( m_size+1, SYMBOLS_PER_WORD ))
        m_storage.resize( util::divide_ri( m_size+1, SYMBOLS_PER_WORD ) );

    begin()[ m_size++ ] = s;
}

// return the base address of a symbol in the stream
// note that several symbols may share the same base address
template <typename SystemTag, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
void *PackedVector<SystemTag, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::addrof(const index_type i)
{
    index_type off = i / SYMBOLS_PER_WORD;
    return &m_storage[off];
}

} // namespace nvbio
