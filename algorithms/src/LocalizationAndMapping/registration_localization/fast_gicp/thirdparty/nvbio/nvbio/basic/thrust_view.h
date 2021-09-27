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

/*! \file utils.h
 *   \brief Define CUDA utilities.
 */

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/vector_view.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

namespace nvbio {

template <typename T> struct device_view_subtype< thrust::device_vector<T> >        { typedef vector_view<T*,uint64> type; };
template <typename T> struct plain_view_subtype< thrust::host_vector<T> >           { typedef vector_view<T*,uint64> type; };
template <typename T> struct plain_view_subtype< thrust::device_vector<T> >         { typedef vector_view<T*,uint64> type; };
template <typename T> struct plain_view_subtype< const thrust::host_vector<T> >     { typedef vector_view<const T*,uint64> type; };
template <typename T> struct plain_view_subtype< const thrust::device_vector<T> >   { typedef vector_view<const T*,uint64> type; };

/// return the plain view of a device vector
///
template <typename T>
vector_view<T*,uint64> device_view(thrust::device_vector<T>& vec) { return vector_view<T*,uint64>( vec.size(), vec.size() ? thrust::raw_pointer_cast( &vec.front() ) : NULL ); }

/// return the plain view of a device vector
///
template <typename T>
vector_view<const T*,uint64> device_view(const thrust::device_vector<T>& vec) { return vector_view<const T*,uint64>( vec.size(), vec.size() ? thrust::raw_pointer_cast( &vec.front() ) : NULL ); }

/// return the plain view of a device vector
///
template <typename T>
vector_view<T*,uint64> plain_view(thrust::device_vector<T>& vec) { return vector_view<T*,uint64>( vec.size(), vec.size() ? thrust::raw_pointer_cast( &vec.front() ) : NULL ); }

/// return the plain view of a device vector
///
template <typename T>
vector_view<const T*,uint64> plain_view(const thrust::device_vector<T>& vec) { return vector_view<const T*,uint64>( vec.size(), vec.size() ? thrust::raw_pointer_cast( &vec.front() ) : NULL ); }

/// return the plain view of a device vector
///
template <typename T>
vector_view<T*,uint64> plain_view(thrust::host_vector<T>& vec) { return vector_view<T*,uint64>( vec.size(), vec.size() ? thrust::raw_pointer_cast( &vec.front() ) : NULL ); }

/// return the plain view of a device vector
///
template <typename T>
vector_view<const T*,uint64> plain_view(const thrust::host_vector<T>& vec) { return vector_view<const T*,uint64>( vec.size(), vec.size() ? thrust::raw_pointer_cast( &vec.front() ) : NULL ); }

/// return the raw pointer of a device vector
///
template <typename T>
T* raw_pointer(thrust::device_vector<T>& vec) { return vec.size() ? thrust::raw_pointer_cast( &vec.front() ) : NULL; }

/// return the raw pointer of a device vector
///
template <typename T>
const T* raw_pointer(const thrust::device_vector<T>& vec) { return vec.size() ? thrust::raw_pointer_cast( &vec.front() ) : NULL; }

/// return the raw pointer of a device vector
///
template <typename T>
T* raw_pointer(thrust::host_vector<T>& vec) { return vec.size() ? thrust::raw_pointer_cast( &vec.front() ) : NULL; }

/// return the plain view of a device vector
///
template <typename T>
const T* raw_pointer(const thrust::host_vector<T>& vec) { return vec.size(), vec.size() ? thrust::raw_pointer_cast( &vec.front() ) : NULL; }

/// return the begin iterator of a device vector
///
template <typename T>
typename thrust::device_vector<T>::iterator begin(thrust::device_vector<T>& vec) { return vec.begin; }

/// return the begin iterator of a device vector
///
template <typename T>
typename thrust::device_vector<T>::const_iterator begin(const thrust::device_vector<T>& vec) { return vec.begin; }

/// return the begin iterator of a host vector
///
template <typename T>
typename thrust::host_vector<T>::iterator begin(thrust::host_vector<T>& vec) { return vec.begin; }

/// return the begin iterator of a host vector
///
template <typename T>
typename thrust::host_vector<T>::const_iterator begin(const thrust::host_vector<T>& vec) { return vec.begin; }

} // namespace nvbio
