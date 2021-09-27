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
#include <nvbio/basic/vector.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/strings/string_set.h>

namespace nvbio {
namespace io {

///
/// \page sequence_io_page Sequence Data Input
///\par
/// This module contains a series of classes to load and represent sequence streams.
/// The idea is that a sequence stream is an object implementing a simple interface, \ref SequenceDataInputStream,
/// which allows to stream through a file or other set of reads in batches, which are represented in memory
/// with an object inheriting from SequenceData.
/// There are several kinds of SequenceData containers to keep the reads in the host or in CUDA device memory.
/// Additionally, the same containers can be \ref PlainViewsSection "viewed" with different \ref SequenceDataViews,
/// in order to allow reinterpreting the base arrays with iterators of different types, e.g. to perform vector loads
/// or use LDG.
///\par
/// Specifically, it exposes the following core classes and methods:
///\par
/// - io::SequenceData
/// - io::SequenceDataHost
/// - io::SequenceDataDevice
/// - io::SequenceDataMMAP
/// - io::SequenceDataMMAPServer
/// - io::SequenceDataInputStream
/// - io::SequenceDataOutputStream
/// - io::open_sequence_file()
/// - io::load_sequence_file()
/// - io::map_sequence_file()
///\par
/// as well as some additional accessors:
///\par
/// - io::SequenceDataViewCore
/// - io::SequenceDataView
/// - io::ConstSequenceDataView
/// - io::SequenceDataAccess
///
///\section SequenceDataSection Sequence Data
///\par
/// The SequenceData class is the base class for all containers holding storage of sequence data.
/// These containers are:
///\par
/// - io::SequenceDataHost
/// - io::SequenceDataDevice
/// - io::SequenceDataMMAP
///\par
/// Each SequenceData object might contain several distinct sequences, which are represented
/// as a packed string-set of sequence symbols accompanied by corresponding string-sets of
/// sequence quality scores and sequence names.
/// Internally, all the string-sets are stored as \ref ConcatenatedStringSet's, with an index
/// specifying the position of the i-th sequence in the concatenated arrays.
/// The packed sequences can in turn be encoded with a user-specified \ref Alphabet "alphabet".
/// However, SequenceData has only runtime knowledge of the alphabet encoding, and hence does not provide
/// any method to perform decoding - rather, it only exposes methods to obtain \ref SequenceDataViews "plain-views"
/// of the underlying sequence storage.
/// However, by providing compile-time knowledge of the alphabet, one can construct a SequenceDataAccess wrapper
/// around any SequenceData (or SequenceDataView) object and access the decoded string-sets transparently.
/// The following example shows how to load a sequence file and access it at compile-time:
///\code
/// typedef io::SequenceDataAccess<DNA>::sequence_string_set_type sequence_string_set_type;
///
/// // load a SequenceData object
/// SharedPointer<io::SequenceDataHost> genome = io::load_sequence_data( DNA, "drosophila.fa" );
///
/// // access it specifying the alphabet at compile-time
/// const io::SequenceDataAccess<DNA> genome_access( genome.get() );
///
/// // fetch the decoding string-set
/// const sequence_string_set_type genome_string_set = genome_access.sequence_string_set();
/// for (uint32 i = 0; i < n; ++i)
/// {
///     // fetch the i-th sequence
///     const sequence_string_set_type::string_type gene = genome_string_set[i];
///
///     // and do something with it...
///     printf("gene %u contains %u bps:\n", i, length( gene ) );
///     for (uint32 j = 0; j < length( gene ); ++j)
///         printf("%c", to_char<DNA>( gene[j] ));
///     printf("\n");
/// }
///\endcode
///
///\section SequenceDataStreamSection Sequence Data Streams
///\par
/// Sometimes it is convenient to stream through sequences in batches.
/// SequenceDataStream provides an abstract interface for doing just this:
///\code
/// // open a sequence file
/// SharedPointer<io::SequenceDataInputStream> reads_file = io::open_sequence_file( "reads.fastq" );
///
/// // instantiate a host SequenceData object
/// io::SequenceDataHost reads;
///
/// // declare how much sequence data we want to load in each batch
/// const uint32  seqs_per_batch = 128*1024;        // the maximum number of sequences
/// const uint32   bps_per_batch = 128*1024*100;    // the maximum number of base pairs
///
/// // loop through the stream in batches
/// while (io::next( DNA_N, &reads, reads_file.get(), seqs_per_batch, bps_per_batch ))
/// {
///     // copy the loaded batch on the device
///     const io::SequenceDataDevice device_reads( reads );
///     ...
/// }
///\endcode
///
///\section SequenceDataTechnicalSection Technical Documentation
///\par
/// More documentation is available in the \ref SequenceIO module.
///

///@addtogroup IO
///@{

///
///@defgroup SequenceIO Sequence Data Input
/// This module contains a series of classes to load and represent read streams.
/// The idea is that a read stream is an object implementing a simple interface, \ref SequenceDataStream,
/// which allows to stream through a file or other set of reads in batches, which are represented in memory
/// with an object inheriting from SequenceData.
/// There are several kinds of SequenceData containers to keep the reads in the host RAM, or in CUDA device memory.
/// Additionally, the same container can be viewed with different \ref SequenceDataViews, in order to allow reinterpreting
/// the base arrays as arrays of different types, e.g. to perform vector loads or use LDG.
///@{
///

// describes the quality encoding for a given read file
enum QualityEncoding
{
    Phred   = 0,    ///< phred quality
    Phred33 = 1,    ///< phred quality + 33
    Phred64 = 2,    ///< phred quality + 64
    Solexa  = 3,    ///< Solexa quality
};

// a set of flags describing the types of supported read strands
enum SequenceEncoding
{
    FORWARD            = 0x0001,
    REVERSE            = 0x0002,
    FORWARD_COMPLEMENT = 0x0004,
    REVERSE_COMPLEMENT = 0x0008,
};

// a set of flags describing what to load
enum SequenceFlags
{
    SEQUENCE_DATA   = 0x0001,
    SEQUENCE_QUALS  = 0x0002,
    SEQUENCE_NAMES  = 0x0004,
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
/// A POD type encapsulating basic sequence information
///
struct SequenceDataInfo
{
    /// empty constructor
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    SequenceDataInfo()
      : m_alphabet(PROTEIN),
        m_n_seqs(0),
        m_name_stream_len(0),
        m_sequence_stream_len(0),
        m_sequence_stream_words(0),
        m_has_qualities(0),
        m_min_sequence_len(uint32(-1)),
        m_max_sequence_len(0),
        m_avg_sequence_len(0)
    {};

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE Alphabet         alphabet()         const { return m_alphabet; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32           size()             const { return m_n_seqs; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32           bps()              const { return m_sequence_stream_len; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32           words()            const { return m_sequence_stream_words; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32           qs()               const { return m_has_qualities ? m_sequence_stream_len : 0u; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32           name_stream_len()  const { return m_name_stream_len; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool             has_qualities()    const { return m_has_qualities ? true : false; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32           max_sequence_len() const { return m_max_sequence_len; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32           min_sequence_len() const { return m_min_sequence_len; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32           avg_sequence_len() const { return m_avg_sequence_len; }

    Alphabet            m_alphabet;                 ///< the alphabet
    uint32              m_n_seqs;                   ///< number of reads in this struct
    uint32              m_name_stream_len;          ///< the length (in bytes) of the name_stream buffer
    uint32              m_sequence_stream_len;      ///< the length of sequence_stream in base pairs
    uint32              m_sequence_stream_words;    ///< the number of words in sequence_stream
    uint32              m_has_qualities;            ///< has qualities

    uint32              m_min_sequence_len;         ///< statistics on the reads
    uint32              m_max_sequence_len;         ///< statistics on the reads
    uint32              m_avg_sequence_len;         ///< statistics on the reads
};

/// comparison operator for SequenceDataInfo
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator== (
    const SequenceDataInfo& op1,
    const SequenceDataInfo& op2)
{
    return
        op1.m_alphabet              == op2.m_alphabet               &&
        op1.m_n_seqs                == op2.m_n_seqs                 &&
        op1.m_name_stream_len       == op2.m_name_stream_len        &&
        op1.m_sequence_stream_len   == op2.m_sequence_stream_len    &&
        op1.m_sequence_stream_words == op2.m_sequence_stream_words  &&
        op1.m_has_qualities         == op2.m_has_qualities          &&
        op1.m_min_sequence_len      == op2.m_min_sequence_len       &&
        op1.m_max_sequence_len      == op2.m_max_sequence_len       &&
        op1.m_avg_sequence_len      == op2.m_avg_sequence_len;
}

/// comparison operator for SequenceDataInfo
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool operator!= (
    const SequenceDataInfo& op1,
    const SequenceDataInfo& op2)
{
    return !(op1 == op2);
}

///@defgroup SequenceDataViews SequenceData Views
///\par
/// This module contains \ref \ref PlainViewsSection "plain-view" classes for \ref SequenceData objects
/// that can be adapted to use iterators of a user-specified type, so as to allow reinterpreting
/// the base arrays, e.g. using LDG loads.
///@{

///
///\par
/// A storage-less plain-view class to represent the core sequence data iterators.
/// Notice that this class only has runtime knowledge of the underlying alphabet encoding,
/// and as such does provide any direct access to the stored sequences (except for pointers
/// to underlying storage).
/// In order to access the decoded sequences, one needs to provide compile-time knowledge
/// of the alphabet, and create a SequenceDataAccess wrapper.
///
///\par
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
    typename IndexIterator           = uint32*,
    typename SequenceStorageIterator = uint32*,
    typename QualStorageIterator     = char*,
    typename NameStorageIterator     = char*>
struct SequenceDataViewCore : public SequenceDataInfo
{
    typedef IndexIterator                   index_iterator;               ///< the index iterator
    typedef SequenceStorageIterator         sequence_storage_iterator;    ///< the read storage iterator
    typedef QualStorageIterator             qual_storage_iterator;        ///< the qualities iterator
    typedef NameStorageIterator             name_storage_iterator;        ///< the names string iterator

    typedef typename to_const<IndexIterator>::type                   const_index_iterator;               ///< the index iterator
    typedef typename to_const<SequenceStorageIterator>::type         const_sequence_storage_iterator;    ///< the read storage iterator
    typedef typename to_const<QualStorageIterator>::type             const_qual_storage_iterator;        ///< the qualities iterator
    typedef typename to_const<NameStorageIterator>::type             const_name_storage_iterator;        ///< the names string iterator

    /// empty constructor
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    SequenceDataViewCore() : SequenceDataInfo() {}

    /// constructor
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    SequenceDataViewCore(
        const SequenceDataInfo&         info,
        const SequenceStorageIterator   sequence_stream,
        const IndexIterator             sequence_index,
        const QualStorageIterator       qual_stream,
        const NameStorageIterator       name_stream,
        const IndexIterator             name_index)
      : SequenceDataInfo        ( info ),
        m_name_stream           (NameStorageIterator( name_stream )),
        m_name_index            (IndexIterator( name_index )),
        m_sequence_stream       (SequenceStorageIterator( sequence_stream )),
        m_sequence_index        (IndexIterator( sequence_index )),
        m_qual_stream           (QualStorageIterator( qual_stream ))
    {}

    /// copy constructor
    ///
    template <
        typename InIndexIterator,
        typename InSequenceIterator,
        typename InQualIterator,
        typename InNameIterator>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    SequenceDataViewCore(const SequenceDataViewCore<InIndexIterator,InSequenceIterator,InQualIterator,InNameIterator>& in)
      : SequenceDataInfo        ( in ),
        m_name_stream           (NameStorageIterator( in.m_name_stream )),
        m_name_index            (IndexIterator( in.m_name_index )),
        m_sequence_stream       (SequenceStorageIterator( in.m_sequence_stream )),
        m_sequence_index        (IndexIterator( in.m_sequence_index )),
        m_qual_stream           (QualStorageIterator( in.m_qual_stream ))
    {}

    /// assignment operator
    ///
    template <
        typename InIndexIterator,
        typename InSequenceIterator,
        typename InQualIterator,
        typename InNameIterator>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    SequenceDataViewCore& operator=(const SequenceDataViewCore<InIndexIterator,InSequenceIterator,InQualIterator,InNameIterator>& in)
    {
        // copy the info
        this->SequenceDataInfo::operator=( in );

        // copy the iterators
        m_name_stream       = NameStorageIterator( in.m_name_stream );
        m_name_index        = IndexIterator( in.m_name_index );
        m_sequence_stream   = SequenceStorageIterator( in.m_sequence_stream );
        m_sequence_index    = IndexIterator( in.m_sequence_index );
        m_qual_stream       = QualStorageIterator( in.m_qual_stream );
        return *this;
    }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE index_iterator                  name_index()                { return m_name_index;  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE index_iterator                  sequence_index()            { return m_sequence_index;  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE name_storage_iterator           name_stream()               { return m_name_stream; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE sequence_storage_iterator       sequence_storage()          { return m_sequence_stream; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE qual_storage_iterator           qual_stream()               { return m_qual_stream; }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_index_iterator            name_index()          const { return m_name_index;  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_index_iterator            sequence_index()      const { return m_sequence_index;  }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_name_storage_iterator     name_stream()         const { return m_name_stream; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_sequence_storage_iterator sequence_storage()    const { return m_sequence_stream; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const_qual_storage_iterator     qual_stream()         const { return m_qual_stream; }

    /// get the range of a read in the sequence stream
    ///
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint2 get_range(const uint32 i) const { return make_uint2( sequence_index()[i], sequence_index()[i+1] ); }

    name_storage_iterator       m_name_stream;      ///< a pointer to a buffer containing the names of all the reads in this batch
    index_iterator              m_name_index;       ///< an array of uint32 with the byte indices of the starting locations of each name in name_stream
    sequence_storage_iterator   m_sequence_stream;  ///< a pointer to a buffer containing the read data
                                                    ///< note that this could point at either host or device memory

    index_iterator              m_sequence_index;   ///< an array of uint32 with the indices of the starting locations of each read in sequence_stream (in base pairs)
    qual_storage_iterator       m_qual_stream;      ///< a pointer to a buffer containing quality data

};

typedef SequenceDataViewCore<uint32*,uint32*,char*,char*>                                                   SequenceDataView;           ///< \n A non-const SequenceData view
typedef SequenceDataViewCore<const uint32*,const uint32*,const char*,const char*>                           ConstSequenceDataView;      ///< \n A const SequenceData view
typedef SequenceDataViewCore<cuda::ldg_pointer<uint32>,cuda::ldg_pointer<uint32>,const char*,const char*>   LdgSequenceDataView;        ///< \n An LDG-based SequenceData view

///@} // SequenceDataViews

///
///\par
/// Base abstract class to encapsulate a sequence data object.
/// This class is meant to be a base for either host, shared or device memory objects, and
/// provides almost no interface, except for virtual methods to obtain a \ref SequenceDataView "plain view" of the
/// class itself.
///
struct SequenceData : public SequenceDataInfo
{
    typedef      SequenceDataView             plain_view_type;
    typedef ConstSequenceDataView       const_plain_view_type;

    /// virtual destructor
    ///
    virtual ~SequenceData() {}

    /// convert to a plain_view
    ///
    virtual operator       plain_view_type() { return plain_view_type(); }

    /// convert to a const plain_view
    ///
    virtual operator const_plain_view_type() const { return const_plain_view_type(); }
};

///
/// A concrete SequenceData storage implementation in host/device memory
///
template <typename system_tag>
struct SequenceDataStorage : public SequenceData
{
    typedef SequenceData                                                SequenceDataBase;

    typedef      SequenceDataView                                       plain_view_type;
    typedef ConstSequenceDataView                                 const_plain_view_type;

    typedef typename nvbio::vector<system_tag,uint32>::iterator                  index_iterator;
    typedef typename nvbio::vector<system_tag,uint32>::iterator                  sequence_storage_iterator;
    typedef typename nvbio::vector<system_tag,char>::iterator                    qual_storage_iterator;
    typedef typename nvbio::vector<system_tag,char>::iterator                    name_storage_iterator;

    typedef typename nvbio::vector<system_tag,uint32>::const_iterator            const_index_iterator;
    typedef typename nvbio::vector<system_tag,uint32>::const_iterator            const_sequence_storage_iterator;
    typedef typename nvbio::vector<system_tag,char>::const_iterator              const_qual_storage_iterator;
    typedef typename nvbio::vector<system_tag,char>::const_iterator              const_name_storage_iterator;

    /// constructor
    ///
    SequenceDataStorage() {}

    /// copy constructor
    ///
    template <typename other_tag>
    SequenceDataStorage(const SequenceDataStorage<other_tag>& other)
    {
        // copy
        this->operator=( other );
    }

    /// copy constructor
    ///
    template <
        typename IndexIterator,
        typename SequenceStorageIterator,
        typename QualStorageIterator,
        typename NameStorageIterator>
    SequenceDataStorage(const SequenceDataViewCore<IndexIterator,SequenceStorageIterator,QualStorageIterator,NameStorageIterator>& other)
    {
        // copy
        this->operator=( other );
    }

    /// copy constructor
    ///
    SequenceDataStorage(const SequenceData& other);

    /// assignment operator
    ///
    template <typename other_tag>
    SequenceDataStorage& operator= (const SequenceDataStorage<other_tag>& other)
    {
        // copy the info
        this->SequenceDataInfo::operator=( other );

        // copy the vectors
        m_sequence_vec       = other.m_sequence_vec;
        m_sequence_index_vec = other.m_sequence_index_vec;
        m_qual_vec           = other.m_qual_vec;
        m_name_vec           = other.m_name_vec;
        m_name_index_vec     = other.m_name_index_vec;
        return *this;
    }

    /// assignment operator, from a view object
    /// NOTE: if the view contains raw pointers, this method works if only if the pointers refer to host data.
    /// If the view refers to device data instead, it must provide proper device iterators.
    ///
    template <
        typename IndexIterator,
        typename SequenceStorageIterator,
        typename QualStorageIterator,
        typename NameStorageIterator>
    SequenceDataStorage& operator= (const SequenceDataViewCore<IndexIterator,SequenceStorageIterator,QualStorageIterator,NameStorageIterator>& other)
    {
        // copy the info
        this->SequenceDataInfo::operator=( other );

        // resize the vectors
        m_sequence_vec.resize( m_sequence_stream_words );
        m_sequence_index_vec.resize( m_n_seqs + 1u );
        m_name_vec.resize( m_name_stream_len );
        m_name_index_vec.resize( m_n_seqs + 1u );
        if (m_has_qualities)
            m_qual_vec.resize( m_sequence_stream_len );

        // and copy the contents
        thrust::copy( other.sequence_storage(), other.sequence_storage() + m_sequence_stream_words, m_sequence_vec.begin() );
        thrust::copy( other.sequence_index(),   other.sequence_index()   + m_n_seqs + 1u,           m_sequence_index_vec.begin() );
        thrust::copy( other.name_stream(),      other.name_stream()      + m_name_stream_len,       m_name_vec.begin() );
        thrust::copy( other.name_index(),       other.name_index()       + m_n_seqs + 1u,           m_name_index_vec.begin() );
        if (m_has_qualities)
            thrust::copy( other.qual_stream(),  other.qual_stream()      + m_sequence_stream_len,   m_qual_vec.begin() );

        return *this;
    }
    /// convert to a plain_view
    ///
    operator plain_view_type()
    {
        return plain_view_type(
            static_cast<const SequenceDataInfo&>( *this ),
            nvbio::raw_pointer( m_sequence_vec ),
            nvbio::raw_pointer( m_sequence_index_vec ),
            nvbio::raw_pointer( m_qual_vec ),
            nvbio::raw_pointer( m_name_vec ),
            nvbio::raw_pointer( m_name_index_vec ) );
    }
    /// convert to a const plain_view
    ///
    operator const_plain_view_type() const
    {
        return const_plain_view_type(
            static_cast<const SequenceDataInfo&>( *this ),
            nvbio::raw_pointer( m_sequence_vec ),
            nvbio::raw_pointer( m_sequence_index_vec ),
            nvbio::raw_pointer( m_qual_vec ),
            nvbio::raw_pointer( m_name_vec ),
            nvbio::raw_pointer( m_name_index_vec ) );
    }

    // reserve enough storage for a given number of reads and bps
    //
    void reserve(const uint32 n_seqs, const uint32 n_bps)
    {
        // a default read id length used to reserve enough space upfront and avoid frequent allocations
        const uint32 AVG_NAME_LENGTH = 250;

        const uint32 bps_per_word = 32u / bits_per_symbol( SequenceDataInfo::m_alphabet );

        m_sequence_index_vec.reserve( n_seqs+1 );
        m_sequence_vec.reserve( n_bps / bps_per_word );
        m_qual_vec.reserve( n_bps );
        m_name_index_vec.reserve( AVG_NAME_LENGTH * n_seqs );
        m_name_index_vec.reserve( n_seqs+1 );
    }

    NVBIO_FORCEINLINE index_iterator                  name_index()                { return m_name_index_vec.begin();  }
    NVBIO_FORCEINLINE index_iterator                  sequence_index()            { return m_sequence_index_vec.begin();  }
    NVBIO_FORCEINLINE name_storage_iterator           name_stream()               { return m_name_vec.begin(); }
    NVBIO_FORCEINLINE sequence_storage_iterator       sequence_storage()          { return m_sequence_vec.begin(); }
    NVBIO_FORCEINLINE qual_storage_iterator           qual_stream()               { return m_qual_vec.begin(); }

    NVBIO_FORCEINLINE const_index_iterator            name_index()          const { return m_name_index_vec.begin();  }
    NVBIO_FORCEINLINE const_index_iterator            sequence_index()      const { return m_sequence_index_vec.begin();  }
    NVBIO_FORCEINLINE const_name_storage_iterator     name_stream()         const { return m_name_vec.begin(); }
    NVBIO_FORCEINLINE const_sequence_storage_iterator sequence_storage()    const { return m_sequence_vec.begin(); }
    NVBIO_FORCEINLINE const_qual_storage_iterator     qual_stream()         const { return m_qual_vec.begin(); }

    nvbio::vector<system_tag,uint32> m_sequence_vec;
    nvbio::vector<system_tag,uint32> m_sequence_index_vec;
    nvbio::vector<system_tag,char>   m_qual_vec;
    nvbio::vector<system_tag,char>   m_name_vec;
    nvbio::vector<system_tag,uint32> m_name_index_vec;
};

typedef SequenceDataStorage<host_tag>   SequenceDataHost;           ///< a SequenceData object stored in host memory
typedef SequenceDataStorage<device_tag> SequenceDataDevice;         ///< a SequenceData object stored in device memory

///
/// A stream of SequenceData, allowing to process the associated reads in batches.
///
struct SequenceDataInputStream
{
    /// virtual destructor
    ///
    virtual ~SequenceDataInputStream() {}

    /// next batch
    ///
    virtual int next(struct SequenceDataEncoder* encoder, const uint32 batch_size, const uint32 batch_bps = uint32(-1)) = 0;

    /// is the stream ok?
    ///
    virtual bool is_ok() = 0;

    /// rewind
    ///
    virtual bool rewind() = 0;
};

/// legacy typedef
///
typedef SequenceDataInputStream SequenceDataStream;

///\relates SequenceDataInputStream
/// utility method to get the next batch from a SequenceDataInputStream
///
int next(const Alphabet alphabet, SequenceDataHost* data, SequenceDataInputStream* stream, const uint32 batch_size, const uint32 batch_bps = uint32(-1));

///\relates SequenceDataInputStream
/// utility method to append the next batch from a SequenceDataInputStream
///
int append(const Alphabet alphabet, SequenceDataHost* data, SequenceDataInputStream* stream, const uint32 batch_size, const uint32 batch_bps = uint32(-1));

///\relates SequenceDataInputStream
/// utility method to skip a batch from a SequenceDataInputStream
///
int skip(SequenceDataInputStream* stream, const uint32 batch_size);

///\relates SequenceDataInputStream
/// factory method to open a read file
///
/// \param sequence_file_name   the file to open
/// \param qualities            the encoding of the qualities
/// \param max_seqs             maximum number of reads to input
/// \param max_sequence_len     maximum read length - reads will be truncated
/// \param flags                a set of flags indicating which strands to encode
///                             in the batch for each read.
///                             For example, passing FORWARD | REVERSE_COMPLEMENT
///                             will result in a stream containing BOTH the forward
///                             and reverse-complemented strands.
///
SequenceDataInputStream* open_sequence_file(
    const char*              sequence_file_name,
    const QualityEncoding    qualities        = Phred33,
    const uint32             max_seqs         = uint32(-1),
    const uint32             max_sequence_len = uint32(-1),
    const SequenceEncoding   flags            = FORWARD,
    const uint32             trim3            = 0,
    const uint32             trim5            = 0);

///\relates SequenceDataHost
/// load a sequence file
///
/// \param alphabet             the alphabet used to encode the sequence data
/// \param sequence_data        the output sequence data
/// \param sequence_file_name   the file to open
/// \param load_flags           a set of flags indicating what to load
/// \param qualities            the encoding of the qualities
///
bool load_sequence_file(
    const Alphabet              alphabet,
    SequenceDataHost*           sequence_data,
    const char*                 sequence_file_name,
    const SequenceFlags         load_flags  = io::SequenceFlags( io::SEQUENCE_DATA | io::SEQUENCE_QUALS | io::SEQUENCE_NAMES ),
    const QualityEncoding       qualities   = Phred33);

///\relates SequenceDataHost
/// load a sequence file
///
/// \param alphabet             the alphabet used to encode the sequence data
/// \param sequence_file_name   the file to open
/// \param load_flags           a set of flags indicating what to load
/// \param qualities            the encoding of the qualities
///
SequenceDataHost* load_sequence_file(
    const Alphabet              alphabet,
    const char*                 sequence_file_name,
    const SequenceFlags         load_flags  = io::SequenceFlags( io::SEQUENCE_DATA | io::SEQUENCE_QUALS | io::SEQUENCE_NAMES ),
    const QualityEncoding       qualities   = Phred33);

///
/// An output stream of SequenceData, allowing to write reads in batches.
///
struct SequenceDataOutputStream
{
    /// virtual destructor
    ///
    virtual ~SequenceDataOutputStream() {}

    /// next batch
    ///
    virtual void next(const SequenceDataHost& sequence_data) = 0;

    /// is the stream ok?
    ///
    virtual bool is_ok() = 0;
};

///\relates SequenceDataOutputStream
/// factory method to open a read file for writing
///
/// \param sequence_file_name   the file to open
/// \param compression          compression options
///
SequenceDataOutputStream* open_output_sequence_file(
    const char* sequence_file_name,
    const char* compression);

///@} // SequenceIO
///@} // IO

} // namespace io

// return a plain view of a SequenceData object
//
inline
io::SequenceData::plain_view_type plain_view(io::SequenceData& sequence_data)
{
    return io::SequenceData::plain_view_type( sequence_data );
}

// return a plain view of a const SequenceData object
//
inline
io::SequenceData::const_plain_view_type plain_view(const io::SequenceData& sequence_data)
{
    return io::SequenceData::const_plain_view_type( sequence_data );
}

/// return a plain view of a SequenceData object
///
template <typename system_tag>
typename io::SequenceDataStorage<system_tag>::plain_view_type
plain_view(io::SequenceDataStorage<system_tag>& sequence_data)
{
    return typename io::SequenceDataStorage<system_tag>::plain_view_type( sequence_data );
}

/// return a plain view of a const SequenceData object
///
template <typename system_tag>
typename io::SequenceDataStorage<system_tag>::const_plain_view_type
plain_view(const io::SequenceDataStorage<system_tag>& sequence_data)
{
    return typename io::SequenceDataStorage<system_tag>::const_plain_view_type( sequence_data );
}

namespace io {

// copy constructor
//
template <typename system_tag>
SequenceDataStorage<system_tag>::SequenceDataStorage(const SequenceData& other)
{
    // copy
    this->operator=( plain_view( other ) );
}

} // namespace io

} // namespace nvbio

#include <nvbio/io/sequence/sequence_access.h>
