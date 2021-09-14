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

#include <nvbio/strings/string_set.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/vector.h>
#include <thrust/transform.h>
#include <thrust/scan.h>


namespace nvbio {

///@addtogroup Strings
///@{

///@addtogroup StringSetsModule
///@{

/// extract a set of seed coordinates out of a string, according to a given seeding functor
///
/// \tparam index_type          the type of string indices: uint32 or uint64
/// \tparam seed_functor        a class providing the following interface:
///\anchor SeedFunctor
///\code
/// struct seed_functor
/// {
///     typedef uint32  argument_type;
///     typedef uint32  result_type;
///
///     // return the number of seeds produced for a given string length
///     uint32 operator() (const uint32 length) const;
///
///     // return the coordinate of the i-th seed produced for a given string length
///     uint2 seed(const uint32 length, const uint32 i) const;
/// };
///\endcode
/// \tparam index_vector_type   a dynamic vector of uint2 coordinates
///
/// \param seeder               the seeding functor
/// \param indices              the vector of output uint32 or uint2 indices:
///                             in the first case, only the starting seed coordinates
///                             will be retained
///
template <typename index_type, typename seed_functor, typename index_vector_type>
index_type enumerate_string_seeds(
    const index_type            string_len,
    const seed_functor          seeder,
          index_vector_type&    indices);

/// extract a set of seed coordinates out of a string-set, according to a given seeding functor
///
/// \tparam string_set_type     the string-set type
/// \tparam seed_functor        a \ref SeedFunctor "Seeding Functor"
/// \tparam index_vector_type   a dynamic vector of uint2 coordinates
///
/// \param string_set           the string set to seed
/// \param seeder               the seeding functor
/// \param indices              the vector of output uint2 or uint4 indices:
///                             in the first case, only the starting seed coordinates
///                             will be retained
///
template <typename string_set_type, typename seed_functor, typename index_vector_type>
uint64 enumerate_string_set_seeds(
    const string_set_type       string_set,
    const seed_functor          seeder,
          index_vector_type&    indices);

/// a \ref SeedFunctor "Seeding Functor" returning seeds sampled at regular intervals
///
template <typename index_type = uint32>
struct uniform_seeds_functor
{
    typedef index_type                                  argument_type;
    typedef index_type                                  result_type;
    typedef typename vector_type<index_type,2u>::type   range_type;

    /// constructor
    ///
    /// \param _interval        the sampling interval
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uniform_seeds_functor(const uint32 _len, const uint32 _interval) : len(_len), interval(_interval) {}

    /// return the number of seeds for a given string length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    index_type operator() (const index_type length) const
    {
        uint32 n = 0;
        for (uint32 pos = 0; pos + len <= length; pos += interval)
            ++n;
        return n;
    }

    /// return the coordinate of the i-th seed
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    range_type seed(const uint32 length, const index_type i) const { return make_vector( i * interval, i * interval + len ); }

    const uint32 len;       ///< the seed length
    const uint32 interval;  ///< the sampling interval
};

///@} StringSetsModule
///@} Strings

} // namespace nvbio

#include <nvbio/strings/seeds_inl.h>
