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

/*! \file cached_iterator.h
 *   \brief CUDA-compatible iterator wrappers allowing to cache the dereferenced
 *   value of generic iterators
 */

#pragma once

#include <iterator>
#include <nvbio/basic/types.h>
#include <thrust/iterator/iterator_categories.h>

#if defined(__CUDACC__)

namespace std
{

// extend the std::iterator_traits with support for CUDA's __restrict__ pointers
template<class _Ty>
struct iterator_traits<const _Ty * __restrict__>
{	// get traits from const pointer
    typedef random_access_iterator_tag  iterator_category;
    typedef _Ty                         value_type;
    typedef ptrdiff_t                   difference_type;
    //typedef ptrdiff_t                   distance_type;	// retained
    typedef const _Ty* __restrict__     pointer;
    typedef const _Ty&                  reference;
};

} // namespace std

#endif // __CUDACC__

namespace nvbio {

///@addtogroup Basic
///@{

typedef std::input_iterator_tag                     input_host_iterator_tag;
typedef std::output_iterator_tag                    output_host_iterator_tag;
typedef std::forward_iterator_tag                   forward_host_iterator_tag;
typedef std::bidirectional_iterator_tag             bidirectional_host_iterator_tag;
typedef std::random_access_iterator_tag             random_access_host_iterator_tag;

typedef thrust::input_device_iterator_tag           input_device_iterator_tag;
typedef thrust::output_device_iterator_tag          output_device_iterator_tag;
typedef thrust::forward_device_iterator_tag         forward_device_iterator_tag;
typedef thrust::bidirectional_device_iterator_tag   bidirectional_device_iterator_tag;
typedef thrust::random_access_device_iterator_tag   random_access_device_iterator_tag;

typedef thrust::input_universal_iterator_tag           input_universal_iterator_tag;
typedef thrust::output_universal_iterator_tag          output_universal_iterator_tag;
typedef thrust::forward_universal_iterator_tag         forward_universal_iterator_tag;
typedef thrust::bidirectional_universal_iterator_tag   bidirectional_universal_iterator_tag;
typedef thrust::random_access_universal_iterator_tag   random_access_universal_iterator_tag;

template <typename iterator_category> struct iterator_category_system {};
template <>                           struct iterator_category_system<input_host_iterator_tag>              { typedef host_tag   type; };
template <>                           struct iterator_category_system<output_host_iterator_tag>             { typedef host_tag   type; };
template <>                           struct iterator_category_system<forward_host_iterator_tag>            { typedef host_tag   type; };
template <>                           struct iterator_category_system<bidirectional_host_iterator_tag>      { typedef host_tag   type; };
template <>                           struct iterator_category_system<random_access_host_iterator_tag>      { typedef host_tag   type; };
template <>                           struct iterator_category_system<input_device_iterator_tag>            { typedef device_tag type; };
template <>                           struct iterator_category_system<output_device_iterator_tag>           { typedef device_tag type; };
template <>                           struct iterator_category_system<forward_device_iterator_tag>          { typedef device_tag type; };
template <>                           struct iterator_category_system<bidirectional_device_iterator_tag>    { typedef device_tag type; };
template <>                           struct iterator_category_system<random_access_device_iterator_tag>    { typedef device_tag type; };
template <>                           struct iterator_category_system<input_universal_iterator_tag>         { typedef device_tag type; };
template <>                           struct iterator_category_system<output_universal_iterator_tag>        { typedef device_tag type; };
template <>                           struct iterator_category_system<forward_universal_iterator_tag>       { typedef device_tag type; };
template <>                           struct iterator_category_system<bidirectional_universal_iterator_tag> { typedef device_tag type; };
template <>                           struct iterator_category_system<random_access_universal_iterator_tag> { typedef device_tag type; };

template <typename iterator>
struct iterator_system
{
    typedef typename std::iterator_traits<iterator>::iterator_category  iterator_category;
    typedef typename iterator_category_system<iterator_category>::type  type;
};

/// extend the std::iterator_traits class
///
template <typename T>
struct iterator_traits : public std::iterator_traits<T>
{
    typedef typename std::iterator_traits<T>::iterator_category  iterator_category;
    typedef typename std::iterator_traits<T>::value_type         value_type;
    typedef typename std::iterator_traits<T>::difference_type    difference_type;
    typedef typename std::iterator_traits<T>::pointer            pointer;
    typedef typename std::iterator_traits<T>::reference          reference;
    typedef T                                                    forward_iterator;       ///< add forward iterator conversion
};

///@} Basic

} // namespace nvbio
