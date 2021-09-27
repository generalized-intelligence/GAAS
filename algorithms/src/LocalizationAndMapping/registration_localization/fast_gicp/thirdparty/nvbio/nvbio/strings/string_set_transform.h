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

#include <nvbio/strings/string.h>
#include <nvbio/strings/string_set.h>

namespace nvbio {

///
/// A transformed string set
///
/// \tparam StringSet           the base string set type
/// \tparam StringTransform     the string transformation, a functor taking StringSet::string_type
///                             as input and returning a StringTransform::result_type as output
///
template <typename StringSet, typename StringTransform>
struct TransformStringSet
{
    typedef StringSet                                                   string_set_type;
    typedef StringTransform                                             transform_type;
    typedef typename transform_type::result_type                        string_type;
    typedef typename string_type::symbol_type                           symbol_type;
    typedef typename StringSet::system_tag                              system_tag;

    typedef StringSetIterator< TransformedStringSet<StringSet,StringTransform> >       iterator;
    typedef StringSetIterator< TransformedStringSet<StringSet,StringTransform> > const_iterator;

    /// default constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    TransformStringSet() {}

    /// constructor
    ///
    /// \param string_set       the base string set
    /// \param transform        the string transfrom
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    TransformStringSet(
        const StringSet         string_set,
        const StringTransform   transform) :
        m_string_set( string_set ),
        m_transform( transform ) {}

    /// set size
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_string_set.size(); }

    /// indexing operator: access the i-th string
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    string_type operator[] (const uint32 i) const
    {
        return m_transform( m_string_set[i] );
    }

    /// begin iterator
    ///
    const_iterator begin() const { return const_iterator(*this,0u); }

    /// begin iterator
    ///
    const_iterator end() const { return const_iterator(*this,size()); }

    /// begin iterator
    ///
    iterator begin() { return iterator(*this,0u); }

    /// begin iterator
    ///
    iterator end() { return iterator(*this,size()); }

private:
    StringSet       m_string_set;
    StringTransform m_transform;
};

///\relates TransformStringSet
///
/// A utility function to make a ConcatenatedStringSet
///
/// \param string_set       the base string set
/// \param transform        the string transfrom
///
template <typename StringSet, typename StringTransform>
TransformStringSet<StringSet,StringTransform> make_transform_string_set(
    const StringSet         string_set,
    const StringTransform   transform)
{
    return TransformStringSet<StringSet,StringTransform>(
        string_set,
        transform );
}

} // namespace nvbio
