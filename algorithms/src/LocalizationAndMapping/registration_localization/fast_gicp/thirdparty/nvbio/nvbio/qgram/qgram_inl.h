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

// build a q-group index from a given string
//
// \param q                the q parameter
// \param string_len       the size of the string
// \param string           the string iterator
//
template <typename string_type>
void QGramIndexDevice::build(
    const uint32        q,
    const uint32        symbol_sz,
    const uint32        string_len,
    const string_type   string,
    const uint32        qlut)
{
    thrust::device_vector<uint8> d_temp_storage;

    symbol_size = symbol_sz;
    Q           = q;
    QL          = qlut;
    QLS         = (Q - QL) * symbol_size;

    n_qgrams = string_len;

    qgrams.resize( string_len );
    index.resize( string_len );

    thrust::device_vector<qgram_type> d_all_qgrams( align<32>( string_len ) * 2u );
    thrust::device_vector<uint32>     d_temp_index( string_len );

    // build the list of q-grams
    thrust::transform(
        thrust::make_counting_iterator<uint32>(0u),
        thrust::make_counting_iterator<uint32>(0u) + string_len,
        d_all_qgrams.begin(),
        string_qgram_functor<string_type>( Q, symbol_size, string_len, string ) );

    // build the list of q-gram indices
    thrust::copy(
        thrust::make_counting_iterator<uint32>(0u),
        thrust::make_counting_iterator<uint32>(0u) + string_len,
        index.begin() );

    // create the ping-pong sorting buffers
    cub::DoubleBuffer<qgram_type>  key_buffers;
    cub::DoubleBuffer<uint32>      value_buffers;

    key_buffers.selector       = 0;
    value_buffers.selector     = 0;
    key_buffers.d_buffers[0]   = nvbio::raw_pointer( d_all_qgrams );
    key_buffers.d_buffers[1]   = nvbio::raw_pointer( d_all_qgrams ) + align<32>( string_len );
    value_buffers.d_buffers[0] = nvbio::raw_pointer( index );
    value_buffers.d_buffers[1] = nvbio::raw_pointer( d_temp_index );

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortPairs( NULL, temp_storage_bytes, key_buffers, value_buffers, string_len, 0u, Q * symbol_size );

    // resize the temp storage vector
    d_temp_storage.clear();
    d_temp_storage.resize( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortPairs( nvbio::raw_pointer( d_temp_storage ), temp_storage_bytes, key_buffers, value_buffers, string_len, 0u, Q * symbol_size );

    // swap the index vector if needed
    if (value_buffers.selector)
        index.swap( d_temp_index );

    // copy only the unique q-grams and count them
    thrust::device_vector<uint32> d_counts( string_len + 1u );

    n_unique_qgrams = cuda::runlength_encode(
        string_len,
        key_buffers.d_buffers[ key_buffers.selector ],
        qgrams.begin(),
        d_counts.begin(),
        d_temp_storage );

    // now we know how many unique q-grams there are
    slots.resize( n_unique_qgrams + 1u );

    // scan the counts to get the slots
    cuda::exclusive_scan(
        n_unique_qgrams + 1u,
        d_counts.begin(),
        slots.begin(),
        thrust::plus<uint32>(),
        uint32(0),
        d_temp_storage );

    // shrink the q-gram vector
    qgrams.resize( n_unique_qgrams );

    const uint32 n_slots = slots[ n_unique_qgrams ];
    if (n_slots != string_len)
        throw runtime_error( "mismatching number of q-grams: inserted %u q-grams, got: %u\n" );

    //
    // build a LUT
    //

    if (QL)
    {
        const uint32 ALPHABET_SIZE = 1u << symbol_size;

        uint64 lut_size = 1;
        for (uint32 i = 0; i < QL; ++i)
            lut_size *= ALPHABET_SIZE;

        // and now search them
        lut.resize( lut_size+1 );

        thrust::lower_bound(
            qgrams.begin(),
            qgrams.begin() + n_unique_qgrams,
            thrust::make_transform_iterator( thrust::make_counting_iterator<uint32>(0), shift_left<qgram_type>( QLS ) ),
            thrust::make_transform_iterator( thrust::make_counting_iterator<uint32>(0), shift_left<qgram_type>( QLS ) ) + lut_size,
            lut.begin() );

        // and write a sentinel value
        lut[ lut_size ] = n_unique_qgrams;
    }
    else
        lut.resize(0);
}

// A functor to localize a string-set index
//
template <typename string_set_type>
struct localize_functor
{
    typedef uint32 argument_type;
    typedef uint2  result_type;

    // constructor
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    localize_functor(const string_set_type _string_set, const uint32* _cum_lengths) :
        string_set(_string_set), cum_lengths(_cum_lengths) {}

    // return the length of the i-th string
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint2 operator() (const uint32 global_idx) const
    {
        const uint32 string_id = uint32( upper_bound( global_idx, cum_lengths, string_set.size() ) - cum_lengths );

        const uint32 base_offset = string_id ? cum_lengths[ string_id-1 ] : 0u;

        return make_uint2( string_id, global_idx - base_offset );
    }

    const string_set_type   string_set;
    const uint32*           cum_lengths;
};

// build a q-group index from a given string set
//
// \param q                the q parameter
// \param string-set       the string-set
//
template <typename string_set_type, typename seed_functor>
void QGramSetIndexDevice::build(
    const uint32            q,
    const uint32            symbol_sz,
    const string_set_type   string_set,
    const seed_functor      seeder,
    const uint32            qlut)
{
    thrust::device_vector<uint8> d_temp_storage;

    symbol_size = symbol_sz;
    Q           = q;
    QL          = qlut;
    QLS         = (Q - QL) * symbol_size;

    // extract the list of q-gram coordinates
    n_qgrams = (uint32)enumerate_string_set_seeds(
        string_set,
        seeder,
        index );

    thrust::device_vector<qgram_type> d_all_qgrams( align<32>( n_qgrams ) * 2u );
    thrust::device_vector<uint2>      d_temp_index( n_qgrams );

    // build the list of q-grams
    thrust::transform(
        index.begin(),
        index.begin() + n_qgrams,
        d_all_qgrams.begin(),
        string_set_qgram_functor<string_set_type>( Q, symbol_size, string_set ) );

    // create the ping-pong sorting buffers
    cub::DoubleBuffer<qgram_type>  key_buffers;
    cub::DoubleBuffer<uint64>      value_buffers;

    key_buffers.selector       = 0;
    value_buffers.selector     = 0;
    key_buffers.d_buffers[0]   = nvbio::raw_pointer( d_all_qgrams );
    key_buffers.d_buffers[1]   = nvbio::raw_pointer( d_all_qgrams ) + align<32>( n_qgrams );
    value_buffers.d_buffers[0] = (uint64*)nvbio::raw_pointer( index );
    value_buffers.d_buffers[1] = (uint64*)nvbio::raw_pointer( d_temp_index );

    size_t temp_storage_bytes = 0;

    // gauge the amount of temp storage we need
    cub::DeviceRadixSort::SortPairs( NULL, temp_storage_bytes, key_buffers, value_buffers, n_qgrams, 0u, Q * symbol_size );

    // resize the temp storage vector
    d_temp_storage.clear();
    d_temp_storage.resize( temp_storage_bytes );

    // do the real run
    cub::DeviceRadixSort::SortPairs( nvbio::raw_pointer( d_temp_storage ), temp_storage_bytes, key_buffers, value_buffers, n_qgrams, 0u, Q * symbol_size );

    // swap the index vector if needed
    if (value_buffers.selector)
        index.swap( d_temp_index );

    // reserve enough storage for the output q-grams
    qgrams.resize( n_qgrams );

    // copy only the unique q-grams and count them
    thrust::device_vector<uint32> d_counts( n_qgrams + 1u );

    n_unique_qgrams = cuda::runlength_encode(
        n_qgrams,
        key_buffers.d_buffers[ key_buffers.selector ],
        qgrams.begin(),
        d_counts.begin(),
        d_temp_storage );

    // now we know how many unique q-grams there are
    slots.resize( n_unique_qgrams + 1u );

    // scan the counts to get the slots
    cuda::exclusive_scan(
        n_unique_qgrams + 1u,
        d_counts.begin(),
        slots.begin(),
        thrust::plus<uint32>(),
        uint32(0),
        d_temp_storage );

    // shrink the q-gram vector
    qgrams.resize( n_unique_qgrams );

    const uint32 n_slots = slots[ n_unique_qgrams ];
    if (n_slots != n_qgrams)
        throw runtime_error( "mismatching number of q-grams: inserted %u q-grams, got: %u\n" );

    //
    // build a LUT
    //

    if (QL)
    {
        const uint32 ALPHABET_SIZE = 1u << symbol_size;

        uint64 lut_size = 1;
        for (uint32 i = 0; i < QL; ++i)
            lut_size *= ALPHABET_SIZE;

        // build a set of spaced q-grams and search them
        lut.resize( lut_size+1 );

        thrust::lower_bound(
            qgrams.begin(),
            qgrams.begin() + n_unique_qgrams,
            thrust::make_transform_iterator( thrust::make_counting_iterator<uint32>(0), shift_left<qgram_type>( QLS ) ),
            thrust::make_transform_iterator( thrust::make_counting_iterator<uint32>(0), shift_left<qgram_type>( QLS ) ) + lut_size,
            lut.begin() );

        // and write a sentinel value
        lut[ lut_size ] = n_unique_qgrams;
    }
    else
        lut.resize(0);
}

// build a q-group index from a given string set
//
// \param q                the q parameter
// \param string-set       the string-set
//
template <typename string_set_type>
void QGramSetIndexDevice::build(
    const uint32            q,
    const uint32            symbol_sz,
    const string_set_type   string_set,
    const uint32            qlut)
{
    build(
        q,
        symbol_sz,
        string_set,
        uniform_seeds_functor<>( q, 1u ),
        qlut );
}

// copy operator
//
template <typename SystemTag>
QGramIndexHost& QGramIndexHost::operator= (const QGramIndexCore<SystemTag,uint64,uint32,uint32>& src)
{
    Q               = src.Q;
    symbol_size     = src.symbol_size;
    n_qgrams        = src.n_qgrams;
    n_unique_qgrams = src.n_unique_qgrams;
    qgrams          = src.qgrams;
    slots           = src.slots;
    index           = src.index;
    QL              = src.QL;
    QLS             = src.QLS;
    lut             = src.lut;
    return *this;
}

// copy operator
//
template <typename SystemTag>
QGramIndexDevice& QGramIndexDevice::operator= (const QGramIndexCore<SystemTag,uint64,uint32,uint32>& src)
{
    Q               = src.Q;
    symbol_size     = src.symbol_size;
    n_qgrams        = src.n_qgrams;
    n_unique_qgrams = src.n_unique_qgrams;
    qgrams          = src.qgrams;
    slots           = src.slots;
    index           = src.index;
    QL              = src.QL;
    QLS             = src.QLS;
    lut             = src.lut;
    return *this;
}

// copy operator
//
template <typename SystemTag>
QGramSetIndexHost& QGramSetIndexHost::operator= (const QGramIndexCore<SystemTag,uint64,uint32,uint2>& src)
{
    Q               = src.Q;
    symbol_size     = src.symbol_size;
    n_unique_qgrams = src.n_unique_qgrams;
    qgrams          = src.qgrams;
    slots           = src.slots;
    index           = src.index;
    QL              = src.QL;
    QLS             = src.QLS;
    lut             = src.lut;
    return *this;
}

// copy operator
//
template <typename SystemTag>
QGramSetIndexDevice& QGramSetIndexDevice::operator= (const QGramIndexCore<SystemTag,uint64,uint32,uint2>& src)
{
    Q               = src.Q;
    symbol_size     = src.symbol_size;
    n_unique_qgrams = src.n_unique_qgrams;
    qgrams          = src.qgrams;
    slots           = src.slots;
    index           = src.index;
    QL              = src.QL;
    QLS             = src.QLS;
    lut             = src.lut;
    return *this;
}

// generate the q-grams corresponding to a list of q-gram coordinates
//
// \tparam string_type         a string iterator
// \tparam index_iterator      a q-gram coordinate iterator
// \tparam qgram_iterator      a q-gram iterator
//
// \param q                    the q-gram length
// \param symbol_size          the symbol size, in bits
// \param string_len           the input string length
// \param string               the input string
// \param n_qgrams             the number of q-grams to generate
// \param indices              the input q-gram coordinates
// \param indices              the output q-grams
//
template <typename string_type, typename index_iterator, typename qgram_iterator>
void generate_qgrams(
    const uint32                q,
    const uint32                symbol_size,
    const uint32                string_len,
    const string_type           string,
    const uint32                n_qgrams,
    const index_iterator        indices,
          qgram_iterator        qgrams)
{
    thrust::transform(
        indices,
        indices + n_qgrams,
        qgrams,
        string_qgram_functor<string_type>( q, symbol_size, string_len, string ) );
}

// generate the q-grams corresponding to a list of q-gram coordinates
//
// \tparam string_type         a string iterator
// \tparam index_iterator      a q-gram coordinate iterator
// \tparam qgram_iterator      a q-gram iterator
//
// \param q                    the q-gram length
// \param symbol_size          the symbol size, in bits
// \param string_set           the input string-set
// \param n_qgrams             the number of q-grams to generate
// \param indices              the input q-gram coordinates
// \param indices              the output q-grams
//
template <typename string_set_type, typename index_iterator, typename qgram_iterator>
void generate_qgrams(
    const uint32                q,
    const uint32                symbol_size,
    const string_set_type       string_set,
    const uint32                n_qgrams,
    const index_iterator        indices,
          qgram_iterator        qgrams)
{
    thrust::transform(
        indices,
        indices + n_qgrams,
        qgrams,
        string_set_qgram_functor<string_set_type>( q, symbol_size, string_set ) );
}

} // namespace nvbio
