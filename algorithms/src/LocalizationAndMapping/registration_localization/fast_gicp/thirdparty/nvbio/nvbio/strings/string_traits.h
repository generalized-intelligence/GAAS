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

namespace nvbio {

///@addtogroup Strings
///@{

/// a string-traits class needed to reason about proper strings
///
template <typename T>
struct string_traits
{
    static const uint32 SYMBOL_SIZE = T::SYMBOL_SIZE;

    typedef typename T::value_type          value_type;
    typedef typename T::reference           reference;
    typedef typename T::index_type          index_type;
    typedef typename T::iterator            iterator;
    typedef typename T::const_iterator      const_iterator;
    typedef typename T::forward_iterator    forward_iterator;
};

/// a string-traits specialization for plain arrays (like C strings)
///
template <typename T>
struct string_traits<T*>
{
    static const uint32 SYMBOL_SIZE = 8u * uint32( sizeof(T) );

    typedef T           value_type;
    typedef T&          reference;
    typedef uint64      index_type;
    typedef T*          iterator;
    typedef const T*    const_iterator;
    typedef const T*    forward_iterator;
};

/// a string-traits specialization for plain arrays (like C strings)
///
template <typename T>
struct string_traits<const T*>
{
    static const uint32 SYMBOL_SIZE = 8u * uint32( sizeof(T) );

    typedef T           value_type;
    typedef const T&    reference;
    typedef uint64      index_type;
    typedef const T*    iterator;
    typedef const T*    const_iterator;
    typedef const T*    forward_iterator;
};

///@} Strings

} // namespace nvbio
