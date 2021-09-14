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

// setup an internal namespace to avoid polluting the global environment
namespace qgroup {

// a functor to set the q-group's I vector
//
template <typename string_type>
struct qgroup_setup_I
{
    typedef string_qgram_functor<string_type>   qgram_functor_type;

    // constructor
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    qgroup_setup_I(
        const QGroupIndexView   _qgroup,
        const uint32            _string_len,
        const string_type       _string)
    : qgroup        ( _qgroup ),
      string_len    ( _string_len ),
      string        ( _string ) {}

    // operator functor
    //
    NVBIO_FORCEINLINE NVBIO_DEVICE
    void operator() (const uint32 p) const
    {
        const qgram_functor_type qgram( qgroup.Q, qgroup.symbol_size, string_len, string );

        // set the bit corresponding to the i-th qgram
        const uint64 g = qgram(p);

        const uint32 word = g / 32u;
        const uint32 bit  = g & 31u;

        atomicOr( qgroup.I + word, 1u << bit );
    }

    const QGroupIndexView   qgroup;
    const uint32            string_len;
    const string_type       string;
};

// a functor to set the q-group's SS vector
//
template <typename string_type>
struct qgroup_setup_SS
{
    typedef string_qgram_functor<string_type>   qgram_functor_type;

    static const uint32 WORD_SIZE = 32;

    // constructor
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    qgroup_setup_SS(
        const QGroupIndexView   _qgroup,
        const uint32            _string_len,
        const string_type       _string)
    : qgroup        ( _qgroup ),
      string_len    ( _string_len ),
      string        ( _string ) {}

    // operator functor
    //
    NVBIO_FORCEINLINE NVBIO_DEVICE
    void operator() (const uint32 p) const
    {
        const qgram_functor_type qgram( qgroup.Q, qgroup.symbol_size, string_len, string );

        // compute the qgram g
        const uint64 g = qgram(p);

        // compute (i,j) from g
        const uint32 i = uint32( g / WORD_SIZE );
        const uint32 j = uint32( g % WORD_SIZE );

        // compute j' such that bit j is the j'-th set bit in I[i]
        const uint32 j_prime = popc( qgroup.I[i] & ((1u << j) - 1u) );

        // atomically increase the appropriate counter in SS
        atomicAdd( qgroup.SS + qgroup.S[i] + j_prime, 1u );
    }

    const QGroupIndexView   qgroup;
    const uint32            string_len;
    const string_type       string;
};

// a functor to set the q-group's SS vector
//
template <typename string_type>
struct qgroup_setup_P
{
    typedef string_qgram_functor<string_type>   qgram_functor_type;

    static const uint32 WORD_SIZE = 32;

    // constructor
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    qgroup_setup_P(
        const QGroupIndexView   _qgroup,
        const uint32            _string_len,
        const string_type       _string)
    : qgroup        ( _qgroup ),
      string_len    ( _string_len ),
      string        ( _string ) {}

    // operator functor
    //
    NVBIO_FORCEINLINE NVBIO_DEVICE
    void operator() (const uint32 p) const
    {
        const qgram_functor_type qgram( qgroup.Q, qgroup.symbol_size, string_len, string );

        // compute the qgram g
        const uint64 g = qgram(p);

        // compute (i,j) from g
        const uint32 i = uint32( g / WORD_SIZE );
        const uint32 j = uint32( g % WORD_SIZE );

        // compute j' such that bit j is the j'-th set bit in I[i]
        const uint32 j_prime = popc( qgroup.I[i] & ((1u << j) - 1u) );

        // atomically increase the appropriate counter in SS to get the next free slot
        const uint32 slot = atomicAdd( qgroup.SS + qgroup.S[i] + j_prime, 1u );

        // and fill the corresponding slot of P
        qgroup.P[ slot ] = p;
    }

    const QGroupIndexView   qgroup;
    const uint32            string_len;
    const string_type       string;
};

} // namespace qgroup

// build a q-group index from a given string
//
// \param q                the q parameter
// \param string_len       the size of the string
// \param string           the string iterator
//
template <typename string_type>
void QGroupIndexDevice::build(
    const uint32        q,
    const uint32        symbol_sz,
    const uint32        string_len,
    const string_type   string)
{
    typedef qgroup::qgroup_setup_I<string_type>     setup_I_type;
    typedef qgroup::qgroup_setup_SS<string_type>    setup_SS_type;
    typedef qgroup::qgroup_setup_P<string_type>     setup_P_type;

    thrust::device_vector<uint8> d_temp_storage;

    Q           = q;
    symbol_size = symbol_sz;
    n_qgrams    = string_len;

    const uint32 ALPHABET_SIZE = 1u << symbol_size;

    uint64 n_max_qgrams = 1;
    for (uint32 i = 0; i < q; ++i)
        n_max_qgrams *= ALPHABET_SIZE;

    const uint32 n_qblocks = uint32( n_max_qgrams / WORD_SIZE );

    I.resize( n_qblocks+1 );
    S.resize( n_qblocks+1 );

    //
    // setup I
    //

    // fill I with zeros
    thrust::fill(
        I.begin(),
        I.begin() + n_qblocks + 1u,
        uint32(0) );

    const setup_I_type setup_I( nvbio::plain_view( *this ), string_len, string );

    // set the bits in I corresponding to the used qgram slots
    thrust::for_each(
        thrust::make_counting_iterator<uint32>(0),
        thrust::make_counting_iterator<uint32>(0) + string_len,
        setup_I );

    //
    // setup S
    //

    // compute the exclusive prefix sum of the popcount of the words in I
    cuda::exclusive_scan(
        n_qblocks + 1u,
        thrust::make_transform_iterator( I.begin(), popc_functor<uint32>() ),
        S.begin(),
        thrust::plus<uint32>(),
        uint32(0),
        d_temp_storage );

    // fetch the number of used qgrams
    n_unique_qgrams = S[ n_qblocks ];

    //
    // setup SS
    //
    SS.resize( n_unique_qgrams + 1u );

    thrust::fill(
        SS.begin(),
        SS.begin() + n_unique_qgrams + 1u,
        uint32(0) );

    const setup_SS_type setup_SS( nvbio::plain_view( *this ), string_len, string );

    thrust::for_each(
        thrust::make_counting_iterator<uint32>(0),
        thrust::make_counting_iterator<uint32>(0) + string_len,
        setup_SS );

    // compute the exclusive prefix sum of SS
    cuda::exclusive_scan(
        n_unique_qgrams + 1u,
        SS.begin(),
        SS.begin(),
        thrust::plus<uint32>(),
        uint32(0),
        d_temp_storage );

    //
    // setup P
    //
    P.resize( string_len );

    // copy SS into a temporary vector for the purpose of slot allocation
    thrust::device_vector<uint32> SS_copy( SS );

    const setup_P_type setup_P( nvbio::plain_view( *this ), string_len, string );

    thrust::for_each(
        thrust::make_counting_iterator<uint32>(0),
        thrust::make_counting_iterator<uint32>(0) + string_len,
        setup_P );

    // and swap the slots with their previous copy
    SS.swap( SS_copy );

    const uint32 n_slots = SS[ n_unique_qgrams ];
    if (n_slots != string_len)
        throw runtime_error( "mismatching number of q-grams: inserted %u q-grams, got: %u\n" );
}

} // namespace nvbio
