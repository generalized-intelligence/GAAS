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

#include <nvbio/strings/alphabet.h>
#include <nvbio/io/sequence/sequence_traits.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/strings/string_set.h>

namespace nvbio {
namespace io {

///@addtogroup IO
///@{

///@addtogroup SequenceIO
///@{

///
///\par
/// An interface class to access a referenced sequence data object.
///\par
/// This class is templated over the a SequenceDataT type which needs to provide the
/// core iterators to access itself
///
/// \tparam SEQUENCE_ALPHABET_T         the alphabet used to access the data
/// \tparam SequenceDataT               the type of the underlying sequence data;
///                                     must provide the following interface:
///
///\code
/// interface SequenceDataT : public SequenceDataInfo
/// {
///     typedef ...                   index_iterator;               // the index iterator
///     typedef ...         sequence_storage_iterator;              // the read storage iterator
///     typedef ...             qual_storage_iterator;              // the qualities iterator
///     typedef ...             name_storage_iterator;              // the names string iterator
///
///     typedef ...                    const_index_iterator;        // the index iterator
///     typedef ...         const_sequence_storage_iterator;        // the read storage iterator
///     typedef ...             const_qual_storage_iterator;        // the qualities iterator
///     typedef ...             const_name_storage_iterator;        // the names string iterator
///
///     NVBIO_HOST_DEVICE const_index_iterator            name_index()          const;
///     NVBIO_HOST_DEVICE const_index_iterator            sequence_index()      const;
///     NVBIO_HOST_DEVICE const_name_storage_iterator     name_stream()         const;
///     NVBIO_HOST_DEVICE const_sequence_storage_iterator sequence_storage()    const;
///     NVBIO_HOST_DEVICE const_qual_storage_iterator     qual_stream()         const;
/// };
///\endcode
///
template <
    Alphabet  SEQUENCE_ALPHABET_T,
    typename  SequenceDataT = ConstSequenceDataView>
struct SequenceDataAccess
{
    static const Alphabet SEQUENCE_ALPHABET = SEQUENCE_ALPHABET_T;                                                        ///< alphabet type
    static const uint32   SEQUENCE_BITS               = SequenceDataTraits<SEQUENCE_ALPHABET>::SEQUENCE_BITS;             ///< symbol size
    static const bool     SEQUENCE_BIG_ENDIAN         = SequenceDataTraits<SEQUENCE_ALPHABET>::SEQUENCE_BIG_ENDIAN;       ///< endianness
    static const uint32   SEQUENCE_SYMBOLS_PER_WORD   = SequenceDataTraits<SEQUENCE_ALPHABET>::SEQUENCE_SYMBOLS_PER_WORD; ///< number of symbols per word

    typedef typename SequenceDataT::const_index_iterator                              index_iterator;               ///< the index iterator
    typedef typename SequenceDataT::const_sequence_storage_iterator                   sequence_storage_iterator;    ///< the read storage iterator
    typedef typename SequenceDataT::const_qual_storage_iterator                       qual_storage_iterator;        ///< the qualities iterator
    typedef typename SequenceDataT::const_name_storage_iterator                       name_storage_iterator;        ///< the names string iterator

    typedef SequenceDataViewCore<
        index_iterator,
        sequence_storage_iterator,
        qual_storage_iterator,
        name_storage_iterator>                                                        sequence_reference;           ///< the sequence reference

    typedef PackedStream<
        sequence_storage_iterator,uint8,SEQUENCE_BITS,SEQUENCE_BIG_ENDIAN>            sequence_stream_type;         ///< the packed read-stream type

    typedef vector_view<sequence_stream_type>                                         sequence_string;              ///< the read string type
    typedef vector_view<qual_storage_iterator>                                        qual_string;                  ///< the quality string type
    typedef vector_view<name_storage_iterator>                                        name_string;                  ///< the name string type

    typedef ConcatenatedStringSet<
        sequence_stream_type,
        index_iterator>                                                         sequence_string_set_type;   ///< string-set type

    typedef ConcatenatedStringSet<
        qual_storage_iterator,
        index_iterator>                                                         qual_string_set_type;   ///< quality string-set type

    typedef ConcatenatedStringSet<
        name_storage_iterator,
        index_iterator>                                                         name_string_set_type;   ///< name string-set type

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  size()                    const { return m_data.size();  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  bps()                     const { return m_data.bps(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  words()                   const { return m_data.words(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  name_stream_len()         const { return m_data.name_stream_len(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  max_sequence_len()        const { return m_data.max_sequence_len(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  min_sequence_len()        const { return m_data.min_sequence_len(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  avg_sequence_len()        const { return m_data.avg_sequence_len(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE index_iterator                  name_index()                const { return m_data.name_index();  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE index_iterator                  sequence_index()            const { return m_data.sequence_index();  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE name_storage_iterator           name_stream()               const { return m_data.name_stream(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_storage_iterator       sequence_storage()          const { return m_data.sequence_storage(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE qual_storage_iterator           qual_stream()               const { return m_data.qual_stream(); }

    /// constructor
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE SequenceDataAccess(const SequenceDataT& data) : m_data( data )
    {
      #if !defined(NVBIO_DEVICE_COMPILATION) || defined(NVBIO_CUDA_DEBUG)
        assert( m_data.m_alphabet == SEQUENCE_ALPHABET );
      #endif
    }

    /// get the range of a read in the sequence stream
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint2 get_range(const uint32 i) const { return make_uint2(sequence_index()[i],sequence_index()[i+1]); }

    /// return a sequence stream object
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_stream_type sequence_stream() const { return sequence_stream_type( sequence_storage() ); }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_string_set_type sequence_string_set() const
    {
        return sequence_string_set_type(
            size(),
            sequence_stream().begin(),
            sequence_index() );
    }

    /// return the i-th read as a string
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_string get_read(const uint32 i) const
    {
        const uint2 sequence_range = get_range( i );
        return sequence_string( sequence_range.y - sequence_range.x, sequence_stream().begin() + sequence_range.x );
    }

    /// return the i-th quality read as a string
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE qual_string get_quals(const uint32 i) const
    {
        const uint2 sequence_range = get_range( i );
        return qual_string( sequence_range.y - sequence_range.x, qual_stream() + sequence_range.x );
    }

    /// return the i-th read name as a string
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE name_string get_name(const uint32 i) const
    {
        const uint2 name_range = make_uint2( name_index()[i], name_index()[i+1] );
        return name_string( name_range.y - name_range.x - 1u, name_stream() + name_range.x );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE qual_string_set_type qual_string_set() const
    {
        return qual_string_set_type(
            size(),
            qual_stream(),
            sequence_index() );
    }


    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE name_string_set_type name_string_set() const
    {
        return name_string_set_type(
            size(),
            name_stream(),
            name_index() );
    }

    const sequence_reference m_data;
};

template <Alphabet         ALPHABET, typename SequenceDataT>
SequenceDataAccess<ALPHABET,SequenceDataT> make_access(const SequenceDataT& data)
{
    return SequenceDataAccess<ALPHABET,SequenceDataT>( data );
}

///
/// An interface class to access a referenced sequence data object.
///
/// This class is templated over the a SequenceDataT type which needs to provide the
/// core iterators to access itself
///
/// \tparam SEQUENCE_ALPHABET_T         the alphabet used to access the data
/// \tparam SequenceDataT               the type of the underlying sequence data
///
template <
    Alphabet  SEQUENCE_ALPHABET_T,
    typename  SequenceDataT>
struct SequenceDataEdit
{
    static const Alphabet SEQUENCE_ALPHABET = SEQUENCE_ALPHABET_T;                                                        ///< alphabet type
    static const uint32   SEQUENCE_BITS               = SequenceDataTraits<SEQUENCE_ALPHABET>::SEQUENCE_BITS;             ///< symbol size
    static const bool     SEQUENCE_BIG_ENDIAN         = SequenceDataTraits<SEQUENCE_ALPHABET>::SEQUENCE_BIG_ENDIAN;       ///< endianness
    static const uint32   SEQUENCE_SYMBOLS_PER_WORD   = SequenceDataTraits<SEQUENCE_ALPHABET>::SEQUENCE_SYMBOLS_PER_WORD; ///< number of symbols per word

    typedef typename SequenceDataT::index_iterator                                    index_iterator;               ///< the index iterator
    typedef typename SequenceDataT::sequence_storage_iterator                         sequence_storage_iterator;    ///< the read storage iterator
    typedef typename SequenceDataT::qual_storage_iterator                             qual_storage_iterator;        ///< the qualities iterator
    typedef typename SequenceDataT::name_storage_iterator                             name_storage_iterator;        ///< the names string iterator

    typedef SequenceDataViewCore<
        index_iterator,
        sequence_storage_iterator,
        qual_storage_iterator,
        name_storage_iterator>                                                        sequence_reference;           ///< the sequence reference

    typedef PackedStream<
        sequence_storage_iterator,uint8,SEQUENCE_BITS,SEQUENCE_BIG_ENDIAN>            sequence_stream_type;         ///< the packed read-stream type

    typedef vector_view<sequence_stream_type>                                         sequence_string;              ///< the read string type

    typedef ConcatenatedStringSet<
        sequence_stream_type,
        index_iterator>                                                         sequence_string_set_type;   ///< string-set type

    typedef ConcatenatedStringSet<
        qual_storage_iterator,
        index_iterator>                                                         qual_string_set_type;   ///< quality string-set type

    typedef ConcatenatedStringSet<
        name_storage_iterator,
        index_iterator>                                                         name_string_set_type;   ///< name string-set type

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  size()                    const { return m_data.size();  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  bps()                     const { return m_data.bps(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  words()                   const { return m_data.words(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  name_stream_len()         const { return m_data.name_stream_len(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  max_sequence_len()        const { return m_data.max_sequence_len(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  min_sequence_len()        const { return m_data.min_sequence_len(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  avg_sequence_len()        const { return m_data.avg_sequence_len(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE index_iterator                  name_index()                const { return m_data.name_index();  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE index_iterator                  sequence_index()            const { return m_data.sequence_index();  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE name_storage_iterator           name_stream()               const { return m_data.name_stream(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_storage_iterator       sequence_storage()          const { return m_data.sequence_storage(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE qual_storage_iterator           qual_stream()               const { return m_data.qual_stream(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE index_iterator                  name_index()                      { return m_data.name_index();  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE index_iterator                  sequence_index()                  { return m_data.sequence_index();  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE name_storage_iterator           name_stream()                     { return m_data.name_stream(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_storage_iterator       sequence_storage()                { return m_data.sequence_storage(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE qual_storage_iterator           qual_stream()                     { return m_data.qual_stream(); }

    /// constructor
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE SequenceDataEdit(SequenceDataT& data) : m_data( data )
    {
      #if !defined(NVBIO_DEVICE_COMPILATION) || defined(NVBIO_CUDA_DEBUG)
        assert( m_data.m_alphabet == SEQUENCE_ALPHABET );
      #endif
    }

    /// get the range of a read in the sequence stream
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint2 get_range(const uint32 i) const { return make_uint2(sequence_index()[i],sequence_index()[i+1]); }

    /// return a sequence stream object
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_stream_type sequence_stream() const { return sequence_stream_type( sequence_storage() ); }

    /// return a sequence stream object
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_stream_type sequence_stream() { return sequence_stream_type( sequence_storage() ); }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_string_set_type sequence_string_set() const
    {
        return sequence_string_set_type(
            size(),
            sequence_stream().begin(),
            sequence_index() );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_string_set_type sequence_string_set()
    {
        return sequence_string_set_type(
            size(),
            sequence_stream().begin(),
            sequence_index() );
    }

    /// return the i-th read as a string
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_string get_read(const uint32 i) const
    {
        const uint2 sequence_range = get_range( i );
        return sequence_string( sequence_range.y - sequence_range.x, sequence_stream().begin() + sequence_range.x );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE qual_string_set_type qual_string_set() const
    {
        return qual_string_set_type(
            size(),
            qual_stream(),
            sequence_index() );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE qual_string_set_type qual_string_set()
    {
        return qual_string_set_type(
            size(),
            qual_stream(),
            sequence_index() );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE name_string_set_type name_string_set() const
    {
        return name_string_set_type(
            size(),
            name_stream(),
            name_index() );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE name_string_set_type name_string_set()
    {
        return name_string_set_type(
            size(),
            name_stream(),
            name_index() );
    }

    mutable sequence_reference m_data;
};

///@} // SequenceIO
///@} // IO

} // namespace io
} // namespace nvbio
