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

#include <nvbio/basic/strided_iterator.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/strings/string_set.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

namespace nvbio {
namespace io {

///
/// \page reads_io_page Read data input
/// This module contains a series of classes to load and represent read streams.
/// The idea is that a read stream is an object implementing a simple interface, \ref ReadDataStream,
/// which allows to stream through a file or other set of reads in batches, which are represented in memory
/// with an object inheriting from ReadData.
/// There are several kinds of ReadData containers to keep the reads in the host RAM, or in CUDA device memory.
/// Additionally, the same container can be viewed with different ReadDataView's, in order to allow reinterpreting
/// the base arrays as arrays of different types, e.g. to perform vector loads or use LDG.
///
/// Specifically, it exposes the following classes and methods:
///
/// - ReadData
/// - ReadDataView
/// - ReadDataStream
/// - open_read_file()
///

///@addtogroup IO
///@{

///
///@defgroup ReadsIO Read data input
/// This module contains a series of classes to load and represent read streams.
/// The idea is that a read stream is an object implementing a simple interface, \ref ReadDataStream,
/// which allows to stream through a file or other set of reads in batches, which are represented in memory
/// with an object inheriting from ReadData.
/// There are several kinds of ReadData containers to keep the reads in the host RAM, or in CUDA device memory.
/// Additionally, the same container can be viewed with different ReadDataView's, in order to allow reinterpreting
/// the base arrays as arrays of different types, e.g. to perform vector loads or use LDG.
///@{
///

// describes the quality encoding for a given read file
enum QualityEncoding
{
    // phred quality
    Phred = 0,
    // phred quality + 33
    Phred33 = 1,
    // phred quality + 64
    Phred64 = 2,
    Solexa = 3,
};

// a set of flags describing the types of supported read strands
enum ReadEncoding
{
    FORWARD            = 0x0001,
    REVERSE            = 0x0002,
    FORWARD_COMPLEMENT = 0x0004,
    REVERSE_COMPLEMENT = 0x0008,
};

// how mates of a paired-end read are encoded
// F = forward, R = reverse
enum PairedEndPolicy
{
    PE_POLICY_FF = 0,
    PE_POLICY_FR = 1,
    PE_POLICY_RF = 2,
    PE_POLICY_RR = 3,
};

///
/// Encodes a (storage-less) plain-data view of a read batch.
/// This class is templated over the iterators pointing to the actual storage, so as to allow
/// them being both raw (const or non-const) pointers or fancier iterators (e.g. cuda::load_pointer
/// or nvbio::vector<system_tag>::iterator's)
///
/// \tparam IndexIterator               the type of the iterator to the reads index
/// \tparam ReadStorageIterator         the type of the iterator to the reads storage
/// \tparam QualStorageIterator         the type of the iterator to the qualities storage
/// \tparam NameStorageIterator         the type of the iterator to the names storage
///
template <
    typename IndexIterator,
    typename ReadStorageIterator,
    typename QualStorageIterator,
    typename NameStorageIterator>
struct ReadDataView
{
    typedef IndexIterator                                                     index_iterator;           ///< the index iterator
    typedef typename to_const<index_iterator>::type                     const_index_iterator;           ///< the const index iterator

    typedef ReadStorageIterator                                               read_storage_iterator;    ///< the read storage iterator
    typedef typename to_const<read_storage_iterator>::type              const_read_storage_iterator;    ///< the const read storage iterator

    typedef QualStorageIterator                                               qual_storage_iterator;    ///< the qualities iterator
    typedef typename to_const<qual_storage_iterator>::type              const_qual_storage_iterator;    ///< the const qualities iterator

    typedef NameStorageIterator                                               name_storage_iterator;    ///< the names string iterator
    typedef typename to_const<name_storage_iterator>::type              const_name_storage_iterator;    ///< the names string iterator

    // symbol size for reads
    static const uint32 READ_BITS = 4;
    // big endian?
    static const bool   HI_BITS   = false; // deprecated
    // big endian?
    static const bool   READ_BIG_ENDIAN = false;
    // symbols per word
    static const uint32 READ_SYMBOLS_PER_WORD = (4*sizeof(uint32))/READ_BITS;

    typedef PackedStream<
        read_storage_iterator,uint8,READ_BITS,READ_BIG_ENDIAN>                      read_stream_type;       ///< the packed read-stream type
    typedef PackedStream<
        const_read_storage_iterator,uint8,READ_BITS,READ_BIG_ENDIAN>          const_read_stream_type;       ///< the const packed read-stream type

    typedef typename       read_stream_type::iterator                               read_stream_iterator;   ///< the read-stream iterator
    typedef typename const_read_stream_type::iterator                         const_read_stream_iterator;   ///< the const read-stream iterator

    typedef vector_view<read_stream_iterator>                                       read_string;            ///< the read string type
    typedef vector_view<const_read_stream_iterator>                           const_read_string;            ///< the const read string type

    typedef ConcatenatedStringSet<
        read_stream_iterator,
        index_iterator>                                                     read_string_set_type;   ///< string-set type

    typedef ConcatenatedStringSet<
        const_read_stream_iterator,
        const_index_iterator>                                         const_read_string_set_type;   ///< const string-set type

    typedef ConcatenatedStringSet<
        qual_storage_iterator,
        index_iterator>                                                     qual_string_set_type;   ///< quality string-set type

    typedef ConcatenatedStringSet<
        const_qual_storage_iterator,
        const_index_iterator>                                         const_qual_string_set_type;   ///< const quality string-set type

    typedef ConcatenatedStringSet<
        name_storage_iterator,
        index_iterator>                                                     name_string_set_type;   ///< name string-set type

    typedef ConcatenatedStringSet<
        const_name_storage_iterator,
        const_index_iterator>                                         const_name_string_set_type;   ///< const name string-set type

    /// empty constructor
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    ReadDataView()
      : m_n_reads(0),
        m_name_stream_len(0),
        m_read_stream_len(0),
        m_read_stream_words(0),
        m_min_read_len(uint32(-1)),
        m_max_read_len(0),
        m_avg_read_len(0)
    {};

    /// copy constructor
    ///
    template <
        typename InIndexIterator,
        typename InReadIterator,
        typename InQualIterator,
        typename InNameIterator>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    ReadDataView(const ReadDataView<InIndexIterator,InReadIterator,InQualIterator,InNameIterator>& in)
      : m_n_reads           (in.m_n_reads),
        m_name_stream       (NameStorageIterator(in.m_name_stream)),
        m_name_stream_len   (in.m_name_stream_len),
        m_name_index        (IndexIterator(in.m_name_index)),
        m_read_stream       (ReadStorageIterator(in.m_read_stream)),
        m_read_stream_len   (in.m_read_stream_len),
        m_read_stream_words (in.m_read_stream_words),
        m_read_index        (IndexIterator(in.m_read_index)),
        m_qual_stream       (QualStorageIterator(in.m_qual_stream)),
        m_min_read_len      (in.m_min_read_len),
        m_max_read_len      (in.m_max_read_len),
        m_avg_read_len      (in.m_avg_read_len)
    {}

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE index_iterator         name_index()             { return m_name_index;  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE index_iterator         read_index()             { return m_read_index;  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE name_storage_iterator  name_stream()            { return m_name_stream; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE read_storage_iterator  read_stream_storage()    { return m_read_stream; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE read_stream_type       read_stream()            { return read_stream_type( m_read_stream ); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE qual_storage_iterator  qual_stream()            { return m_qual_stream; }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_index_iterator         const_name_index()          const { return m_name_index;  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_index_iterator         const_read_index()          const { return m_read_index;  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_name_storage_iterator  const_name_stream()         const { return m_name_stream; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_read_storage_iterator  const_read_stream_storage() const { return m_read_stream; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_read_stream_type       const_read_stream()         const { return const_read_stream_type( m_read_stream ); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_qual_storage_iterator  const_qual_stream()         const { return m_qual_stream; }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_index_iterator         name_index()                const { return const_name_index();  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_index_iterator         read_index()                const { return const_read_index();  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_name_storage_iterator  name_stream()               const { return const_name_stream(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_read_storage_iterator  read_stream_storage()       const { return const_read_stream_storage(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_read_stream_type       read_stream()               const { return const_read_stream(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_qual_storage_iterator  qual_stream()               const { return const_qual_stream(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  size()                    const { return m_n_reads; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  bps()                     const { return m_read_stream_len; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  words()                   const { return m_read_stream_words; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  name_stream_len()         const { return m_name_stream_len; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  max_read_len()            const { return m_max_read_len; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  min_read_len()            const { return m_min_read_len; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32  avg_read_len()            const { return m_avg_read_len; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint2   get_range(const uint32 i) const { return make_uint2(m_read_index[i],m_read_index[i+1]); }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE read_string_set_type read_string_set()
    {
        return read_string_set_type(
            size(),
            read_stream().begin(),
            read_index() );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_read_string_set_type read_string_set() const
    {
        return const_read_string_set_type(
            size(),
            read_stream().begin(),
            read_index() );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_read_string_set_type const_read_string_set() const
    {
        return const_read_string_set_type(
            size(),
            read_stream().begin(),
            read_index() );
    }

    /// return the i-th read as a string
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE read_string get_read(const uint32 i)
    {
        const uint2            read_range = get_range( i );
        return read_string( read_range.y - read_range.x, read_stream().begin() + read_range.x );
    }

    /// return the i-th read as a string
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE read_string get_read(const uint32 i) const
    {
        const uint2            read_range = get_range( i );
        return const_read_string( read_range.y - read_range.x, read_stream().begin() + read_range.x );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE qual_string_set_type qual_string_set()
    {
        return qual_string_set_type(
            size(),
            qual_stream(),
            read_index() );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_qual_string_set_type qual_string_set() const
    {
        return const_qual_string_set_type(
            size(),
            qual_stream(),
            read_index() );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_qual_string_set_type const_qual_string_set() const
    {
        return const_qual_string_set_type(
            size(),
            qual_stream(),
            read_index() );
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

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_name_string_set_type name_string_set() const
    {
        return const_name_string_set_type(
            size(),
            name_stream(),
            name_index() );
    }

    /// return the a string-set view of this set of reads
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_name_string_set_type const_name_string_set() const
    {
        return const_name_string_set_type(
            size(),
            name_stream(),
            name_index() );
    }

public:
    // number of reads in this struct
    uint32                  m_n_reads;

    // a pointer to a buffer containing the names of all the reads in this batch
    name_storage_iterator   m_name_stream;
    // the length (in bytes) of the name_stream buffer
    uint32                  m_name_stream_len;
    // an array of uint32 with the byte indices of the starting locations of each name in name_stream
    index_iterator          m_name_index;

    // a pointer to a buffer containing the read data
    // note that this could point at either host or device memory
    read_storage_iterator   m_read_stream;
    // the length of read_stream in base pairs
    uint32                  m_read_stream_len;
    // the number of words in read_stream
    uint32                  m_read_stream_words;
    // an array of uint32 with the indices of the starting locations of each read in read_stream (in base pairs)
    index_iterator          m_read_index;

    // a pointer to a buffer containing quality data
    // (the indices in m_read_index are also valid for this buffer)
    qual_storage_iterator   m_qual_stream;

    // statistics on the reads: minimum size, maximum size, average size
    uint32                  m_min_read_len;
    uint32                  m_max_read_len;
    uint32                  m_avg_read_len;
};

///
/// The core ReadData class
///
typedef ReadDataView<uint32*,uint32*,char*,char*> ReadDataCore;

///
/// Base abstract class to encode a host-side read batch.
/// This has no storage, it's meant to be a base for either host or device memory objects
///
struct ReadData : public ReadDataCore
{
    typedef ReadDataView<uint32*,uint32*,char*,char*>                               plain_view_type;
    typedef ReadDataView<const uint32*,const uint32*,const char*,const char*> const_plain_view_type;

    /// empty constructor
    ///
    ReadData() : ReadDataCore()
    {
        m_name_stream   = NULL;
        m_name_index    = NULL;
        m_read_stream   = NULL;
        m_read_index    = NULL;
        m_qual_stream   = NULL;
    }

    /// virtual destructor
    ///
    virtual ~ReadData() {}
};

///
/// a read batch in host memory
///
struct ReadDataRAM : public ReadData
{
    /// a set of flags describing the operators to apply to a given strand
    ///
    enum StrandOp
    {
        NO_OP                 = 0x0000,     ///< default, no operator applied
        REVERSE_OP            = 0x0001,     ///< reverse operator
        COMPLEMENT_OP         = 0x0002,     ///< complement operator
        REVERSE_COMPLEMENT_OP = 0x0003,     ///< convenience definition, same as StrandOp( REVERSE_OP | COMPLEMENT_OP )
    };

    ReadDataRAM();

    /// reserve enough storage for a given number of reads and bps
    ///
    void reserve(const uint32 n_reads, const uint32 n_bps);

    /// add a read to the end of this batch
    ///
    /// \param read_len                     input read length
    /// \param name                         read name
    /// \param base_pairs                   list of base pairs
    /// \param quality                      list of base qualities
    /// \param quality_encoding             quality encoding scheme
    /// \param truncate_read_len            truncate the read if longer than this
    /// \param conversion_flags             conversion operators applied to the strand
    ///
    void push_back(uint32                   read_len,
                   const char*              name,
                   const uint8*             base_pairs,
                   const uint8*             quality,
                   const QualityEncoding    quality_encoding,
                   const uint32             truncate_read_len,
                   const StrandOp           conversion_flags);

    /// signals that the batch is complete
    ///
    void end_batch(void);

    std::vector<uint32> m_read_vec;
    std::vector<uint32> m_read_index_vec;
    std::vector<char>   m_qual_vec;
    std::vector<char>   m_name_vec;
    std::vector<uint32> m_name_index_vec;
};

///
/// a read in device memory
///
struct ReadDataDevice : public ReadData
{
    enum {
        READS = 0x01,
        QUALS = 0x02,
    };

    /// constructor
    ///
     ReadDataDevice(const ReadData& host_data, const uint32 flags = READS);

    /// destructor
    ///
    ~ReadDataDevice();

    uint64 allocated() const { return m_allocated; }

private:
    uint64 m_allocated;
};

typedef ReadDataRAM     ReadDataHost;
typedef ReadDataDevice  ReadDataCUDA;

///
/// A stream of ReadData, allowing to process the associated
/// reads in batches.
///
struct ReadDataStream
{
    ReadDataStream(uint32 truncate_read_len = uint32(-1))
      : m_truncate_read_len(truncate_read_len)
    {
    };

    /// virtual destructor
    ///
    virtual ~ReadDataStream() {}

    /// next batch
    ///
    virtual ReadData* next(const uint32 batch_size, const uint32 batch_bps = uint32(-1)) = 0;

    /// is the stream ok?
    ///
    virtual bool is_ok() = 0;

    // maximum length of a read; longer reads are truncated to this size
    uint32             m_truncate_read_len;
};


/// factory method to open a read file
///
/// \param read_file_name       the file to open
/// \param qualities            the encoding of the qualities
/// \param max_reads            maximum number of reads to input
/// \param max_read_len         maximum read length - reads will be truncated
/// \param flags                a set of flags indicating which strands to encode
///                             in the batch for each read.
///                             For example, passing FORWARD | REVERSE_COMPLEMENT
///                             will result in a stream containing BOTH the forward
///                             and reverse-complemented strands.
///
ReadDataStream *open_read_file(const char *          read_file_name,
                               const QualityEncoding qualities,
                               const uint32          max_reads = uint32(-1),
                               const uint32          max_read_len = uint32(-1),
                               const ReadEncoding    flags = REVERSE);

///@} // ReadsIO
///@} // IO

} // namespace io

/// return a plain view of a ReadData object
///
inline
io::ReadData::plain_view_type plain_view(io::ReadData& read_data)
{
    return io::ReadData::plain_view_type( read_data );
}

/// return a plain view of a const ReadData object
///
inline
io::ReadData::const_plain_view_type plain_view(const io::ReadData& read_data)
{
    return io::ReadData::const_plain_view_type( read_data );
}

} // namespace nvbio
