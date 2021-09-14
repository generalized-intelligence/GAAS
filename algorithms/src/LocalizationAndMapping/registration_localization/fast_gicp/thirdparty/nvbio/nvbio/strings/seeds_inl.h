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

template <typename in_coord_type, typename out_coord_type>
struct project_coords_functor {};

template <>
struct project_coords_functor<uint32,uint32>
{
    typedef uint32 argument_type;
    typedef uint32 result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type i) const { return i; }
};
template <>
struct project_coords_functor<uint64,uint64>
{
    typedef uint64 argument_type;
    typedef uint64 result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type i) const { return i; }
};
template <typename in_coord_type>
struct project_coords_functor<in_coord_type,uint32>
{
    typedef in_coord_type argument_type;
    typedef uint32        result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type i) const { return i.x; }
};
template <typename in_coord_type>
struct project_coords_functor<in_coord_type,uint2>
{
    typedef in_coord_type argument_type;
    typedef uint2         result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type i) const { return make_uint2( i.x, i.y ); }
};
template <typename in_coord_type>
struct project_coords_functor<in_coord_type,uint3>
{
    typedef in_coord_type argument_type;
    typedef uint3         result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type i) const { return make_uint3( i.x, i.y, i.z ); }
};
template <typename in_coord_type>
struct project_coords_functor<in_coord_type,uint4>
{
    typedef in_coord_type argument_type;
    typedef uint4         result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type i) const { return make_uint4( i.x, i.y, i.z, i.w ); }
};

/// project a given set of coordinates to a lower-dimensional object
///
template <typename out_coord_type, typename in_coord_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
out_coord_type project_coords(const in_coord_type i)
{
    const project_coords_functor<in_coord_type,out_coord_type> p;
    return p(i);
}

// A functor to return the coordinates given by a seed_functor
//
template <typename index_type, typename seed_functor, typename coord_type>
struct string_seed_functor
{
    typedef index_type  argument_type;
    typedef coord_type  result_type;

    // constructor
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_seed_functor(const index_type _string_len, const seed_functor _seeder) :
        string_len(_string_len), seeder(_seeder) {}

    // return the coordinate of the i-th seed
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    coord_type operator() (const index_type idx) const
    {
        return project_coords<coord_type>( seeder.seed( string_len, idx ) );
    }

    const index_type       string_len;
    const seed_functor      seeder;
};

// A functor to return the localized coordinates given by a seed_functor
//
template <typename string_set_type, typename seed_functor, typename coord_type>
struct localized_seed_functor
{
    typedef uint32      argument_type;
    typedef coord_type  result_type;

    // constructor
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    localized_seed_functor(const string_set_type _string_set, const seed_functor _seeder, const uint64* _cum_seeds) :
        string_set(_string_set), seeder(_seeder), cum_seeds(_cum_seeds) {}

    // return the localized coordinate of the i-th seed
    //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    coord_type operator() (const uint64 global_idx) const
    {
        // compute the string index
        const uint32 string_id = uint32( upper_bound( global_idx, cum_seeds, string_set.size() ) - cum_seeds );

        // fetch the string length
        const uint32 string_len = string_set[ string_id ].length();

        // compute the local string coordinate
        const uint64 base_offset = string_id ? cum_seeds[ string_id-1 ] : 0u;
        const uint32 seed_idx    = uint32( global_idx - base_offset );

        const uint2 seed = seeder.seed( string_len, seed_idx );
        return project_coords<coord_type>( make_uint4( string_id, seed.x, seed.y, 0u ) );
    }

    const string_set_type   string_set;
    const seed_functor      seeder;
    const uint64*           cum_seeds;
};

// extract a set of seed coordinates out of a string, according to a given seeding functor
//
template <typename index_type, typename seed_functor, typename index_vector_type>
index_type enumerate_string_seeds(
    const index_type            string_len,
    const seed_functor          seeder,
          index_vector_type&    indices)
{
    typedef typename index_vector_type::value_type   coord_type;

    // fetch the total number of output q-grams
    const index_type n_seeds = seeder( string_len );

    // reserve enough storage
    indices.resize( n_seeds );

    // build the list of q-gram indices
    thrust::transform(
        thrust::make_counting_iterator<index_type>(0u),
        thrust::make_counting_iterator<index_type>(0u) + n_seeds,
        indices.begin(),
        string_seed_functor<index_type,seed_functor,coord_type>( string_len, seeder ) );

    return n_seeds;
}

// extract a set of seed coordinates out of a string-set, according to a given seeding functor
//
template <typename string_set_type, typename seed_functor, typename index_vector_type>
uint64 enumerate_string_set_seeds(
    const string_set_type       string_set,
    const seed_functor          seeder,
          index_vector_type&    indices)
{
    // TODO: use some vector traits...
    typedef typename index_vector_type::system_tag   system_tag;
    typedef typename index_vector_type::value_type   coord_type;

    const uint32 n_strings = string_set.size();

    nvbio::vector<system_tag,uint64> cum_seeds( n_strings );

    // scan the number of q-grams produced per string
    thrust::inclusive_scan(
        thrust::make_transform_iterator(
            thrust::make_transform_iterator( thrust::make_counting_iterator<uint64>(0u), string_set_length_functor<string_set_type>( string_set ) ),
            seeder ),
        thrust::make_transform_iterator(
            thrust::make_transform_iterator( thrust::make_counting_iterator<uint64>(0u), string_set_length_functor<string_set_type>( string_set ) ),
            seeder ) + n_strings,
        cum_seeds.begin() );

    // fetch the total nunber of q-grams to output
    const uint64 n_seeds = cum_seeds[ n_strings-1 ];

    // reserve enough storage
    indices.resize( n_seeds );

    // build the list of q-gram indices
    thrust::transform(
        thrust::make_counting_iterator<uint32>(0u),
        thrust::make_counting_iterator<uint32>(0u) + n_seeds,
        indices.begin(),
        localized_seed_functor<string_set_type,seed_functor,coord_type>( string_set, seeder, nvbio::plain_view( cum_seeds ) ) );

    return n_seeds;
}

} // namespace nvbio
