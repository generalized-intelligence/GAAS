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

#include <nvbio/qgram/qgram.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/cuda/primitives.h>
#include <nvbio/basic/thrust_view.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/scatter.h>
#include <thrust/for_each.h>
#include <thrust/iterator/constant_iterator.h>

///\page qgroup_page Q-Group Index Module
///\htmlonly
/// <img src="nvidia_cubes.png" style="position:relative; bottom:-10px; border:0px;"/>
///\endhtmlonly
///\par
/// This module contains a series of functions to build a Q-Group Index, as described
/// in:
///\par
/// <i>Massively parallel read mapping on GPUs with PEANUT</i> \n
/// Johannes Koester and Sven Rahmann \n
/// http://arxiv.org/pdf/1403.1706v1.pdf
///
///
/// \section TechnicalOverviewSection Technical Overview
///\par
/// A complete list of the classes and functions in this module is given in the \ref QGroupIndex documentation.
///

namespace nvbio {

///@addtogroup QGram
///@{

///
///@defgroup QGroupIndex Q-Group Index Module
/// This module contains a series of functions to build a Q-Group Index, as described
/// in:
///\par
/// <i>Massively parallel read mapping on GPUs with PEANUT</i> \n
/// Johannes Koester and Sven Rahmann \n
/// http://arxiv.org/pdf/1403.1706v1.pdf \n
/// this data-structure requires O(A^q) bits of storage in the alphabet-size <i>A</i> and the q-gram length <i>q</i>
/// (to be precise, 2*A^q bits + (min(A^q,|T|) + |T|) words), and provides O(1) query time;
///@{
///

/// A plain view of a q-group index (see \ref QGroupIndex)
///
template <typename index_iterator>
struct QGroupIndexViewCore
{
    static const uint32 WORD_SIZE = 32;

    // class typedefs
    typedef index_iterator                                      vector_type;
    typedef uint32                                              coord_type;

    // plain view typedefs
    typedef QGroupIndexViewCore<index_iterator>                 plain_view_type;
    typedef QGroupIndexViewCore<index_iterator>                 const_plain_view_type;

    // unary functor typedefs
    typedef uint64                                              argument_type;
    typedef uint2                                               result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    QGroupIndexViewCore(
        const uint32 _Q                 = 0,
        const uint32 _symbol_size       = 0,
        const uint32 _n_qgrams          = 0,
        const uint32 _n_unique_qgrams   = 0,
        vector_type  _I                 = NULL,
        vector_type  _S                 = NULL,
        vector_type  _SS                = NULL,
        vector_type  _P                 = NULL) :
        Q               (_Q),
        symbol_size     (_symbol_size),
        n_qgrams        (_n_qgrams),
        n_unique_qgrams (_n_unique_qgrams),
        I               (_I),
        S               (_S),
        SS              (_SS),
        P               (_P)    {}

    /// return the slots of P corresponding to the given qgram g
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint2 range(const uint64 g) const
    {
        const uint32 i = uint32( g / WORD_SIZE );
        const uint32 j = uint32( g % WORD_SIZE );

        // check whether the j-th bit of I[i] is set
        if ((I[i] & (1u << j)) == 0u)
            return make_uint2( 0u, 0u );

        // compute j' such that bit j is the j'-th set bit in I[i]
        const uint32 j_prime = popc( I[i] & ((1u << j) - 1u) );

        return make_uint2(
            SS[ S[i] + j_prime ],
            SS[ S[i] + j_prime + 1u ] );
    }

    /// functor operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint2 operator() (const uint64 g) const { return range( g ); }

    /// locate a given occurrence of a q-gram
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 locate(const uint32 i) const { return P[i]; }


    uint32        Q;
    uint32        symbol_size;
    uint32        n_qgrams;
    uint32        n_unique_qgrams;
    vector_type   I;
    vector_type   S;
    vector_type   SS;
    vector_type   P;
};

typedef QGroupIndexViewCore<uint32*>       QGroupIndexView;
typedef QGroupIndexViewCore<const uint32*> ConstQGroupIndexView;

/// A host-side q-group index (see \ref QGroupIndex)
///
struct QGroupIndexHost
{
    static const uint32 WORD_SIZE = 32;

    typedef host_tag                                            system_tag;

    typedef thrust::host_vector<uint32>                         vector_type;
    typedef uint32                                              coord_type;
    typedef QGroupIndexView                                     plain_view_type;
    typedef ConstQGroupIndexView                                const_plain_view_type;

    /// return the amount of device memory used
    ///
    uint64 used_host_memory() const
    {
        return I.size() * sizeof(uint32) +
               S.size() * sizeof(uint32) +
               SS.size() * sizeof(uint32) +
               P.size() * sizeof(uint32);
    }

    /// return the amount of device memory used
    ///
    uint64 used_device_memory() const { return 0u; }

    uint32        Q;
    uint32        n_qgrams;
    uint32        n_unique_qgrams;
    vector_type   I;
    vector_type   S;
    vector_type   SS;
    vector_type   P;
};

/// A device-side q-group index (see \ref QGroupIndex)
///
struct QGroupIndexDevice
{
    static const uint32 WORD_SIZE = 32;

    typedef device_tag                                          system_tag;

    typedef thrust::device_vector<uint32>                       vector_type;
    typedef uint32                                              coord_type;
    typedef QGroupIndexView                                     view_type;

    /// build a q-group index from a given string T; the amount of storage required
    /// is basically O( A^q + |T|*32 ) bits, where A is the alphabet size.
    ///
    /// \tparam string_type     the string iterator type
    ///
    /// \param q                the q parameter
    /// \param symbol_sz        the size of the symbols, in bits
    /// \param string_len       the size of the string
    /// \param string           the string iterator
    ///
    template <typename string_type>
    void build(
        const uint32        q,
        const uint32        symbol_sz,
        const uint32        string_len,
        const string_type   string);

    /// return the amount of device memory used
    ///
    uint64 used_host_memory() const { return 0u; }

    /// return the amount of device memory used
    ///
    uint64 used_device_memory() const
    {
        return I.size() * sizeof(uint32) +
               S.size() * sizeof(uint32) +
               SS.size() * sizeof(uint32) +
               P.size() * sizeof(uint32);
    }

    uint32        Q;
    uint32        symbol_size;
    uint32        n_qgrams;
    uint32        n_unique_qgrams;
    vector_type   I;
    vector_type   S;
    vector_type   SS;
    vector_type   P;
};

/// return the plain view of a QGroupIndexView, i.e. the object itself
///
inline
QGroupIndexView plain_view(const QGroupIndexView qgram) { return qgram; }

/// return the plain view of a QGroupIndexView, i.e. the object itself
///
inline
ConstQGroupIndexView plain_view(const ConstQGroupIndexView qgram) { return qgram; }

/// return the plain view of a QGroupIndex
///
inline
QGroupIndexView plain_view(QGroupIndexDevice& qgroup)
{
    return QGroupIndexView(
        qgroup.Q,
        qgroup.symbol_size,
        qgroup.n_qgrams,
        qgroup.n_unique_qgrams,
        nvbio::plain_view( qgroup.I ),
        nvbio::plain_view( qgroup.S ),
        nvbio::plain_view( qgroup.SS ),
        nvbio::plain_view( qgroup.P ) );
}

/// return the plain view of a QGroupIndex
///
inline
ConstQGroupIndexView plain_view(const QGroupIndexDevice& qgroup)
{
    return ConstQGroupIndexView(
        qgroup.Q,
        qgroup.symbol_size,
        qgroup.n_qgrams,
        qgroup.n_unique_qgrams,
        nvbio::plain_view( qgroup.I ),
        nvbio::plain_view( qgroup.S ),
        nvbio::plain_view( qgroup.SS ),
        nvbio::plain_view( qgroup.P ) );
}

template<> struct plain_view_subtype<QGroupIndexHost>         { typedef QGroupIndexView type; };
template<> struct plain_view_subtype<QGroupIndexDevice>       { typedef QGroupIndexView type; };
template<> struct plain_view_subtype<const QGroupIndexHost>   { typedef ConstQGroupIndexView type; };
template<> struct plain_view_subtype<const QGroupIndexDevice> { typedef ConstQGroupIndexView type; };

///@} // end of the QGroupIndex group
///@} // end of the QGram group

} // namespace nvbio

#include <nvbio/qgram/qgroup_inl.h>
