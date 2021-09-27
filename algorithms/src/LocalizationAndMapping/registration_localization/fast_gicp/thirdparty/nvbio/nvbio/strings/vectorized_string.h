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
#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/vector_loads.h>
#include <nvbio/strings/string_traits.h>
#include <nvbio/strings/string.h>
#include <string.h>


namespace nvbio {

///@addtogroup Strings
///@{

///\defgroup VectorizedStringLoadingModule Vectorized String Loading
///\par
/// The functions in this module define a vector loading API for strings:
/// strings, are usually made up of tiny characters, often a byte each, or
/// just a few bits.
/// Loading them byte by byte stresses the memory bus, and often results in
/// bad memory utilization.
/// Loading them in bigger chunks, when possible (e.g. when the string is
/// properly aligned), can be very favourable.
/// This API defines:
///
///\code
///  // the size of the vector which can be used to load many symbols at once
///  vectorized_string<string_type>::VECTOR_WIDTH;
///
///  // the range of the string which is amenable to vectorized loading
///  uint2 vectorized_string_range(const string_type& string);
///
///  // a function to retrieve a vector
///  void  vectorized_string_load(const string_type& string, const uint32 i, value_type* v);
///\endcode
///\par
/// The following example shows how the API could be used:
///
///\code
/// const char* string = "this is a long enough ASCII string - let's see if this stuff works...";
///
/// // determine the vectorization width
/// const uint32 VECW = vectorized_string<const char*>::VECTOR_WIDTH;
/// printf("w = %u\n", VECW);
/// 
/// // determine the vectorized string range
/// const uint2 vec_range = vectorized_string_range( string );
/// printf("range(%u,%u)\n", vec_range.x, vec_range.y);
///
/// // loop through the scalar range
/// for (uint32 i = 0; i < vec_range.x; ++i)
///     printf("%c", string[i]);
/// 
/// // loop through the vectorized range
/// for (uint32 i = vec_range.x; i < vec_range.y; i += VECW)
/// {
///     char v[VECW];
/// 
///     // load a vector
///     vectorized_string_load( string, i, v );
/// 
///     for (uint32 j = 0; j < VECW; ++j)
///         printf("%c", v[j]);
/// }
///
/// // loop through the scalar range
/// for (uint32 i = vec_range.y; i < length( string ); ++i)
///     printf("%c", string[i]);
/// printf("\n");
///\endcode
///

///@addtogroup VectorizedStringLoadingModule
///@{

/// \ref VectorizedStringLoadingModule interface
///
template <typename string_type>
struct vectorized_string
{
    typedef typename string_traits<string_type>::value_type value_type;
    
    static const uint32 VECTOR_WIDTH = 1u;  ///< the intrinsic vector width - this is somewhat arbitrarily defined, as for generic types we just manually load the blocks

    /// determine the vectorized range of a string with a specified vector width
    ///
    template <uint32 VECTOR_WIDTH_T>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static uint2 range(const string_type& string)
    {
        const uint32 rounded_length = (uint32)util::round_z( length( string ), VECTOR_WIDTH_T );
        return make_uint2( 0, rounded_length );
    }

    /// load a vector with a specified vector width
    ///
    template <uint32 VECTOR_WIDTH_T>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void load(
        const string_type&  string,
        const uint32        offset,
        value_type*         vector)
    {
        // fill the vector with a simple loop
        #pragma unroll
        for (uint32 j = 0; j < VECTOR_WIDTH_T; ++j)
            vector[j] = string[offset + j];
    }

    /// determine the vectorized range of a string using the maximum vector width
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static uint2 range(const string_type& string)
    {
        return range<VECTOR_WIDTH>( string );
    }

    /// load a vector using the maximum vector width
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void load(
        const string_type&  string,
        const uint32        offset,
        value_type*         vector)
    {
        load<VECTOR_WIDTH>( string, offset, vector );
    }
};

/// determine the vectorized range of a string using the maximum vector width
///
template <typename string_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint2 vectorized_string_range(const string_type& string)
{
    return vectorized_string<string_type>::range( string );
}

/// load a vector using the maximum vector width
///
template <typename string_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void vectorized_string_load(const string_type& string, const uint32 i, typename string_traits<string_type>::value_type* vector)
{
    vectorized_string<string_type>::load( string, i, vector );
}

//
// --- const char* specialization------------------------------------------------------------------------
//

/// const char* specialization of the \ref VectorizedStringLoadingModule interface
///
template <>
struct vectorized_string<const char*>
{
    typedef const char*                    string_type;
    typedef char                           value_type;
    typedef uint64                         index_type;

    static const uint32 VECTOR_WIDTH = maximum_vector_width<char>::VALUE; ///< the intrinsic vector width

    /// determine the vectorized range of a string with a specified vector width
    ///
    template <uint32 VECTOR_WIDTH_T>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static uint2 range(const char* string)
    {
        const uint64 offset = uint64( string ) % VECTOR_WIDTH_T;
        const uint64 len    = length( string );

        const uint64 aligned_begin = util::round_i( offset,                 VECTOR_WIDTH );
        const uint64 aligned_end   = util::round_z( offset + len, VECTOR_WIDTH );

        return aligned_begin < aligned_end ?
            make_uint2( aligned_begin - offset, aligned_end - offset ) :
            make_uint2( len, len );
    }

    /// load a vector with a specified vector width
    ///
    template <uint32 VECTOR_WIDTH_T>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void load(
        const char*  string,
        const uint32 offset,
        char*        vector)
    {
        const char* base_ptr = string + offset;
        vector_load<VECTOR_WIDTH_T>( base_ptr, vector );
    }

    /// determine the vectorized range of a string using the maximum vector width
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static uint2 range(const char* string)
    {
        return range<VECTOR_WIDTH>( string );
    }

    /// load a vector using the maximum vector width
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void load(
        const char*  string,
        const uint32 offset,
        char*        vector)
    {
        load<VECTOR_WIDTH>( string, offset, vector );
    }
};

//
// --- vector_view<const T*> specialization------------------------------------------------------------------------
//

/// vector_view specialization of the \ref VectorizedStringLoadingModule interface
///
template <typename T, typename IndexType>
struct vectorized_string< vector_view<const T*,IndexType> >
{
    typedef vector_view<const T*,IndexType> string_type;
    typedef T                               value_type;
    typedef IndexType                       index_type;

    static const uint32 VECTOR_WIDTH = maximum_vector_width<T>::VALUE; ///< the intrinsic vector width

    /// determine the vectorized range of a string with a specified vector width
    ///
    template <uint32 VECTOR_WIDTH_T>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static uint2 range(const string_type& string)
    {
        const uint64 offset = (uint64(string.base()) / sizeof(T)) % VECTOR_WIDTH_T;

        const uint64 aligned_begin = util::round_i( offset,                 VECTOR_WIDTH );
        const uint64 aligned_end   = util::round_z( offset + string.size(), VECTOR_WIDTH );

        return aligned_begin < aligned_end ?
            make_uint2( aligned_begin - offset, aligned_end - offset ) :
            make_uint2( string.size(), string.size() );
    }

    /// load a vector with a specified vector width
    ///
    template <uint32 VECTOR_WIDTH_T>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void load(
        const string_type& string,
        const uint32 offset,
        T*           vector)
    {
        const T* base_ptr = string.base() + offset;
        vector_load<VECTOR_WIDTH_T>( base_ptr, vector );
    }

    /// determine the vectorized range of a string using the maximum vector width
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static uint2 range(const string_type& string)
    {
        return range<VECTOR_WIDTH>( string );
    }

    /// load a vector using the maximum vector width
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void load(
        const string_type& string,
        const uint32 offset,
        T*           vector)
    {
        load<VECTOR_WIDTH>( string, offset, vector );
    }
};

//
// --- vector_view< PackedStream > specialization---------------------------------------------------------------------
//

/// PackedStream specialization of the \ref VectorizedStringLoadingModule interface
///
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
struct vectorized_string< vector_view< PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType> > >
{
    typedef vector_view< PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType> >            string_type;
    typedef Symbol                                                                                          value_type;
    typedef IndexType                                                                                       index_type;
    typedef PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>                           packed_stream_type;
    typedef InputStream                                                                                     storage_iterator;
    typedef typename packed_stream_type::storage_type                                                       storage_type;

    static const uint32 SYMBOL_SIZE  = SYMBOL_SIZE_T;
    static const uint32 SYMBOL_COUNT = 1u << SYMBOL_SIZE_T;
    static const uint32 VECTOR_WIDTH = packed_stream_type::VECTOR_WIDTH; ///< the intrinsic vector width

    /// determine the vectorized range of a string using the intrinsic vector width
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static uint2 range(const string_type& string)
    {
        const index_type offset = string.base().index();

        const index_type aligned_begin = util::round_i( offset,                 VECTOR_WIDTH );
        const index_type aligned_end   = util::round_z( offset + string.size(), VECTOR_WIDTH );

        return aligned_begin < aligned_end ?
            make_uint2( aligned_begin - offset, aligned_end - offset ) :
            make_uint2( string.size(), string.size() );
    }

    /// load a vector with the intrinsic vector width
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void load(
        const string_type&  string,
        const uint32        offset,
        value_type*         vector)
    {
        const storage_iterator storage = string.base().stream();

        // find the proper word containing the vector we are after
        const index_type word_idx = (string.base().index() + offset) / VECTOR_WIDTH;

        const storage_type word = storage[ word_idx ];

        // unpack the entire word
        if (BIG_ENDIAN_T)
        {
            const uint32 SYMBOL_OFFSET = (VECTOR_WIDTH-1) * SYMBOL_SIZE;

            #pragma unroll
            for (uint32 j = 0; j < VECTOR_WIDTH; ++j)
                vector[j] = (word >> (SYMBOL_OFFSET - SYMBOL_SIZE * j)) & (SYMBOL_COUNT-1);
        }
        else
        {
            #pragma unroll
            for (uint32 j = 0; j < VECTOR_WIDTH; ++j)
                vector[j] = (word >> (SYMBOL_SIZE * j)) & (SYMBOL_COUNT-1);
        }
    }
};

///@} VectorizedStringLoadingModule
///@} Strings

} // namespace nvbio
