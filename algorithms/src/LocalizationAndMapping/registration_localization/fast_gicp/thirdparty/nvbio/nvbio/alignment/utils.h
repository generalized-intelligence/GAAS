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

#include <nvbio/basic/packedstream.h>
#include <nvbio/alignment/sink.h>

namespace nvbio {
namespace aln {

///@addtogroup Alignment
///@{

///@defgroup Utilities
///@{

/// A meta-function returning the type of the checkpoint cells for a given \ref Aligner "Aligner"
///
/// \tparam aligner_type        the queries \ref Aligner "Aligner" type
///
template <typename aligner_type> struct checkpoint_storage_type {
    typedef null_type type;    ///< the type of the checkpoint cells
};
template <AlignmentType TYPE, typename algorithm_tag>                        struct checkpoint_storage_type< EditDistanceAligner<TYPE,algorithm_tag> >                  { typedef  int16 type; };
template <AlignmentType TYPE, typename scoring_type, typename algorithm_tag> struct checkpoint_storage_type< HammingDistanceAligner<TYPE,scoring_type,algorithm_tag> >  { typedef int16 type; };
template <AlignmentType TYPE, typename scoring_type, typename algorithm_tag> struct checkpoint_storage_type< SmithWatermanAligner<TYPE,scoring_type,algorithm_tag> >    { typedef  int16 type; };
template <AlignmentType TYPE, typename scoring_type, typename algorithm_tag> struct checkpoint_storage_type< GotohAligner<TYPE,scoring_type,algorithm_tag> >            { typedef short2 type; };

/// A meta-function returning the type of the column cells for a given \ref Aligner "Aligner"
///
/// \tparam aligner_type        the queries \ref Aligner "Aligner" type
///
template <typename aligner_type> struct column_storage_type {
    typedef null_type type;    ///< the type of the column cells
};
template <AlignmentType TYPE, typename algorithm_tag>                        struct column_storage_type< EditDistanceAligner<TYPE,algorithm_tag> >                  { typedef  int16 type; };
template <AlignmentType TYPE, typename scoring_type, typename algorithm_tag> struct column_storage_type< HammingDistanceAligner<TYPE,scoring_type,algorithm_tag> >  { typedef  int16 type; };
template <AlignmentType TYPE, typename scoring_type, typename algorithm_tag> struct column_storage_type< SmithWatermanAligner<TYPE,scoring_type,algorithm_tag> >    { typedef  int16 type; };
template <AlignmentType TYPE, typename scoring_type, typename algorithm_tag> struct column_storage_type< GotohAligner<TYPE,scoring_type,algorithm_tag> >            { typedef short2 type; };

/// A meta-function returning the number of bits required to represent the direction vectors
/// for a given \ref Aligner "Aligner"
///
/// \tparam aligner_type        the queries \ref Aligner "Aligner" type
///
template <typename aligner_type> struct direction_vector_traits {
    static const uint32 BITS = 2;   ///< the number of bits needed to encode direction vectors
};

/// A meta-function returning the number of bits required to represent the direction vectors
/// for a GotohAligner.
///
/// \tparam aligner_type        the queries \ref Aligner "Aligner" type
///
template <AlignmentType TYPE, typename scoring_type, typename algorithm_tag> struct direction_vector_traits<GotohAligner<TYPE,scoring_type,algorithm_tag> > {
    static const uint32 BITS = 4;   ///< the number of bits needed to encode direction vectors
};

///@}

///@defgroup Utilities
///@{

///
/// A simple implementation of the \ref SmithWatermanScoringScheme model
///
struct SimpleSmithWatermanScheme
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE SimpleSmithWatermanScheme() {}
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE SimpleSmithWatermanScheme(
        const int32 match, const int32 mm, const int32 del, const int32 ins) :
        m_match(match), m_mismatch(mm), m_deletion(del), m_insertion(ins) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 match(const uint8 q = 0)      const { return m_match; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 mismatch(const uint8 q = 0)   const { return m_mismatch; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 mismatch(const uint8 a, const uint8 b, const uint8 q = 0)   const { return m_mismatch; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 deletion()                    const { return m_deletion; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 insertion()                   const { return m_insertion; };

    int32 m_match;
    int32 m_mismatch;
    int32 m_deletion;
    int32 m_insertion;
};

///
/// A simple implementation of the \ref GotohScoringScheme model
///
struct SimpleGotohScheme
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE SimpleGotohScheme() {}
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE SimpleGotohScheme(
        const int32 match, const int32 mm, const int32 gap_open, const int32 gap_ext) :
        m_match(match), m_mismatch(mm), m_gap_open(gap_open), m_gap_ext(gap_ext) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 match(const uint8 q = 0)      const { return m_match; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 mismatch(const uint8 q = 0)   const { return m_mismatch; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 mismatch(const uint8 a, const uint8 b, const uint8 q = 0)   const { return m_mismatch; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 substitution(const uint32 r_i, const uint32 q_j, const uint8 r, const uint8 q, const uint8 qq = 0) const { return q == r ? m_match : m_mismatch; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 pattern_gap_open()            const { return m_gap_open; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 pattern_gap_extension()       const { return m_gap_ext; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 text_gap_open()               const { return m_gap_open; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 text_gap_extension()          const { return m_gap_ext; };

    int32 m_match;
    int32 m_mismatch;
    int32 m_gap_open;
    int32 m_gap_ext;
};

///
/// Calculate the maximum possible number of pattern gaps that could occur in a
/// given score boundary
///
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_pattern_gaps(
    const HammingDistanceAligner<TYPE,scoring_scheme_type,algorithm_tag>&   aligner,
	int32                                                                   min_score,
    int32                                                                   pattern_len);

///
/// Calculate the maximum possible number of reference gaps that could occur in a
/// given score boundary
///
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_text_gaps(
    const HammingDistanceAligner<TYPE,scoring_scheme_type, algorithm_tag>&  aligner,
	int32                                                                   min_score,
    int32                                                                   pattern_len);

///
/// Calculate the maximum possible number of pattern gaps that could occur in a
/// given score boundary
///
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_pattern_gaps(
    const SmithWatermanAligner<TYPE,scoring_scheme_type,algorithm_tag>& aligner,
	int32                                                               min_score,
    int32                                                               pattern_len);

///
/// Calculate the maximum possible number of reference gaps that could occur in a
/// given score boundary
///
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_text_gaps(
    const SmithWatermanAligner<TYPE,scoring_scheme_type, algorithm_tag>& aligner,
	int32                                                                min_score,
    int32                                                                pattern_len);

///
/// Calculate the maximum possible number of pattern gaps that could occur in a
/// given score boundary
///
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_pattern_gaps(
    const GotohAligner<TYPE,scoring_scheme_type,algorithm_tag>& aligner,
	int32                                                       min_score,
    int32                                                       pattern_len);

///
/// Calculate the maximum possible number of reference gaps that could occur in a
/// given score boundary
///
template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_text_gaps(
    const GotohAligner<TYPE,scoring_scheme_type,algorithm_tag>& aligner,
	int32                                                       min_score,
    int32                                                       pattern_len);

///
/// Calculate the maximum possible number of pattern gaps that could occur in a
/// given score boundary
///
template <AlignmentType TYPE, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_pattern_gaps(
    const EditDistanceAligner<TYPE,algorithm_tag>&  aligner,
	int32                                           min_score,
    int32                                           pattern_len);

///
/// Calculate the maximum possible number of reference gaps that could occur in a
/// given score boundary
///
template <AlignmentType TYPE, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
uint32 max_text_gaps(
    const EditDistanceAligner<TYPE,algorithm_tag>&  aligner,
	int32                                           min_score,
    int32                                           pattern_len);

///
/// A trivial implementation of a quality string, constantly zero;
/// This class has both a string and an iterator interface, as it is used as its own iterator.
///
struct trivial_quality_string
{
    static const uint32 SYMBOL_SIZE = 8u;

    typedef random_access_universal_iterator_tag    iterator_category;
    typedef uint8                                   value_type;
    typedef uint8                                   reference;
    typedef const uint8*                            pointer;
    typedef  int32                                  difference_type;
    typedef uint32                                  index_type;
    typedef trivial_quality_string                  iterator;
    typedef trivial_quality_string                  const_iterator;
    typedef trivial_quality_string                  forward_iterator;

    /// indexing operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    uint8 operator[] (const index_type i) const { return 0u; }

    /// dereference operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    uint8 operator* () const { return 0u; }

    /// pre-decrement
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    trivial_quality_string& operator++ (){ return *this; }

    /// pre-increment
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    trivial_quality_string operator++ (int dummy) { return *this; }

    /// pre-decrement
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    trivial_quality_string& operator-- (){ return *this; }

    /// post-decrement
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    trivial_quality_string operator-- (int dummy) { return *this; }

    /// return beginning iterator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    iterator begin() const { return *this; }
};

///
/// A trivial implementation of a quality string set, constantly zero
///
struct trivial_quality_string_set
{
    typedef trivial_quality_string string_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    string_type operator[] (const uint32 i) const { return string_type(); }
};

///@} // end of Utilities group
///@} // end of Alignment group

} // namespace aln
} // namespace nvbio

#include <nvbio/alignment/utils_inl.h>
