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

///@addtogroup Alignment
///@{
namespace aln {
///@}

///
///@defgroup Alignment Alignment Module
/// This module contains a series of functions to perform string alignment.
/// The the kind of alignment performed is selected according to an
/// \ref Aligner "Aligner".
///@{
///

///@defgroup AlignmentTypeModule Alignment Type
/// The alignment type specifies how to penalize gaps at the beginning and end of the
/// pattern and text.
///@{

/// alignment type specifier
///
enum AlignmentType { GLOBAL, LOCAL, SEMI_GLOBAL };

///@} // end of AlignerTagModule group

///@defgroup AlgorithmTag Algorithm Tags
/// Algorithm tags are used to specify a DP algorithm.
/// In order to avoid expensive memory transactions, the algorithms we designed block the
/// matrix in stripes that are processed in registers. The blocking can happen either along
/// the pattern or along the text. To communicate the values across stripes, the right-most
/// column of each stripe must be stored in memory - hence the two algorithms differ for
/// the amount of temporary storage needed (for long texts, text-blocking is preferred).
///\par
/// Additionally, for \ref EditDistanceAligner "edit distance", the Myers bit-vector algorithm
/// is available.
///@{

/// an algorithm that blocks the DP matrix along the pattern, in stripes parallel to the text
///
///\anchor PatternBlockingTag
struct PatternBlockingTag {};  ///< block along the pattern

/// an algorithm that blocks the DP matrix along the text, in stripes parallel to the pattern
/// (at the moment, only supported for scoring)
///
///\anchor TextBlockingTag
struct TextBlockingTag {};     ///< block along the text (at the moment, this is only supported for scoring)

/// Myers bit-vector algorithm, only supported for the EditDistanceAligner
///
///\tparam ALPHABET_SIZE_T      the size of the alphabet, in symbols; currently there are fast
///                             specializations for alphabets of 2, 4 and 5 symbols.
///
///\anchor MyersTag
template <uint32 ALPHABET_SIZE_T> struct MyersTag { static const uint32 ALPHABET_SIZE = ALPHABET_SIZE_T; }; ///< Myers bit-vector algorithm

template <typename T> struct transpose_tag {};
template <>           struct transpose_tag<PatternBlockingTag> { typedef TextBlockingTag type; };
template <>           struct transpose_tag<TextBlockingTag>    { typedef PatternBlockingTag type; };

template <typename T> struct transpose_aligner {};

///@} // end of AlgorithmTag group

///@defgroup AlignerTag Aligner Tags
/// Aligner tags are used to specify the algorithmic family of an \ref Aligner "Aligner" (which can
/// in general be a parametric object).
/// The aligner tag of a given aligner can be retrived with the aligner_tag meta-function.
/// For example the SmithWatermanAligner <TYPE,scoring_scheme_type> has an
/// aligner tag of type SmithWatermanTag.
///@{

struct SmithWatermanTag {}; ///< the Smith-Waterman aligner tag
struct GotohTag {};         ///< the Gotoh aligner tag
struct EditDistanceTag {};  ///< the Edit Distance aligner tag
struct HammingDistanceTag {};  ///< the Hamming Distance aligner tag

/// A meta-function specifying the aligner tag of an \ref Aligner "Aligner"
///
/// \tparam aligner_type        the queried aligner type
///
template <typename aligner_type>
struct aligner_tag {
    typedef typename aligner_type::aligner_tag type; ///< the type of the aligner's \ref AlignerTag "tag"
};

///@} // end of AlignerTag group

/// A compact representation of an alignment result, specifying the score and the start/terminal cells
///
template <typename ScoreType>
struct Alignment
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    Alignment() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    Alignment(const ScoreType _score, const uint2 _source, const uint2 _sink) : score( _score ), source( _source ), sink(_sink) {}

    ScoreType score;    ///< alignment score
    uint2     source;   ///< alignment start cells in the pattern & text
    uint2     sink;     ///< alignment terminal cells in the pattern & text
};

/// A representation of the DP direction flow vectors
///
enum DirectionVector {
    SUBSTITUTION   = 0u,
    INSERTION      = 1u,
    DELETION       = 2u,
    SINK           = 3u,
    INSERTION_EXT  = 4u,
    DELETION_EXT   = 8u,
    // MASKS
    HMASK          = 3u,
    EMASK          = 4u,
    FMASK          = 8u,
};

/// A representation of the Gotoh traceback matrix states
///
enum State
{
    HSTATE = 0,
    ESTATE = 1,
    FSTATE = 2
};

///
///@defgroup Aligner Aligners
/// An Aligner is an object representing a specific alignment algorithm and its parameters,
/// passed to \ref alignment_page "Alignment" functions to determine which algorithm to invoke.
/// Three aligners are currently available:
///
///     - EditDistanceAligner
///     - SmithWatermanAligner
///     - GotohAligner
///@{
///

/// An edit distance alignment algorithm, see \ref Aligner
/// \anchor EditDistanceAligner
///
/// \tparam T_TYPE                  specifies whether the alignment is SEMI_GLOBAL/GLOBAL
/// \tparam AlgorithmType           specifies the \ref AlgorithmTag "Algorithm Tag"
///
template <AlignmentType T_TYPE, typename AlgorithmType = PatternBlockingTag>
struct EditDistanceAligner
{
    static const AlignmentType TYPE =   T_TYPE;         ///< the AlignmentType

    typedef EditDistanceTag             aligner_tag;    ///< the \ref AlignerTag "Aligner Tag"
    typedef AlgorithmType               algorithm_tag;  ///< the \ref AlgorithmTag "Algorithm Tag"
};

template <AlignmentType T_TYPE, typename AlgorithmTag>
struct transpose_aligner< EditDistanceAligner<T_TYPE,AlgorithmTag> >
{
    typedef EditDistanceAligner<T_TYPE, typename transpose_tag<AlgorithmTag>::type> type;
};

template <AlignmentType TYPE>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
EditDistanceAligner<TYPE> make_edit_distance_aligner()
{
    return EditDistanceAligner<TYPE>();
}

template <AlignmentType TYPE, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
EditDistanceAligner<TYPE,algorithm_tag> make_edit_distance_aligner()
{
    return EditDistanceAligner<TYPE,algorithm_tag>();
}

template <AlignmentType TYPE>
EditDistanceAligner<TYPE,TextBlockingTag> transpose(const EditDistanceAligner<TYPE,PatternBlockingTag>& aligner)
{
    return EditDistanceAligner<TYPE,TextBlockingTag>();
}
template <AlignmentType TYPE, typename scoring_scheme_type>
EditDistanceAligner<TYPE,PatternBlockingTag> transpose(const EditDistanceAligner<TYPE,TextBlockingTag>& aligner)
{
    return EditDistanceAligner<TYPE,PatternBlockingTag>();
}

/// A Gotoh alignment algorithm, see \ref Aligner
/// \anchor GotohAligner
///
/// \tparam T_TYPE                  specifies whether the alignment is LOCAL/SEMI_GLOBAL/GLOBAL
/// \tparam scoring_scheme_type     specifies the scoring scheme, a model of \ref GotohScoringScheme
/// \tparam AlgorithmType           specifies the \ref AlgorithmTag "Algorithm Tag"
///
///\section GotohScoringScheme Gotoh Scoring Scheme
/// A Gotoh scoring scheme is a class exposing the following interface:
///
/// \code
/// struct GotohScoringScheme
/// {
///     // the maximum match bonus at a given quality q 
///     int32 match(const uint8 q = 0) const;
///
///     // the substitution score of text and pattern bases (t,p),
///     // at positions (t_i,p_j), with a given pattern quality q
///     int32 substitution(
///         const uint32 t_i, const uint32 p_j,
///         const uint8  t,   const uint8  p,   const uint8 q = 0) const;
///
///     // the pattern gap open cost
///     int32 pattern_gap_open()       const;
///
///     // the pattern gap extension cost
///     int32 pattern_gap_extension()  const;
///
///     // the text gap open cost
///     int32 text_gap_open()          const;
///
///     // the text gap extension cost
///     int32 text_gap_extension()     const;
/// };
/// \endcode
///
template <AlignmentType T_TYPE, typename scoring_scheme_type, typename AlgorithmType = PatternBlockingTag>
struct GotohAligner
{
    static const AlignmentType TYPE =   T_TYPE;         ///< the AlignmentType

    typedef GotohTag                    aligner_tag;    ///< the \ref AlignerTag "Aligner Tag"
    typedef AlgorithmType               algorithm_tag;  ///< the \ref AlgorithmTag "Algorithm Tag"
 
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    GotohAligner(const scoring_scheme_type _scheme) : scheme(_scheme) {}

    scoring_scheme_type scheme;
};

template <AlignmentType T_TYPE, typename scoring_scheme_type, typename AlgorithmTag>
struct transpose_aligner< GotohAligner<T_TYPE, scoring_scheme_type, AlgorithmTag> >
{
    typedef GotohAligner<T_TYPE, scoring_scheme_type, typename transpose_tag<AlgorithmTag>::type> type;
};

template <AlignmentType TYPE, typename scoring_scheme_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
GotohAligner<TYPE,scoring_scheme_type> make_gotoh_aligner(const scoring_scheme_type& scheme)
{
    return GotohAligner<TYPE,scoring_scheme_type>( scheme );
}

template <AlignmentType TYPE, typename algorithm_tag, typename scoring_scheme_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
GotohAligner<TYPE,scoring_scheme_type,algorithm_tag> make_gotoh_aligner(const scoring_scheme_type& scheme)
{
    return GotohAligner<TYPE,scoring_scheme_type,algorithm_tag>( scheme );
}

template <AlignmentType TYPE, typename scoring_scheme_type>
GotohAligner<TYPE,scoring_scheme_type,TextBlockingTag> transpose(const GotohAligner<TYPE,scoring_scheme_type,PatternBlockingTag>& aligner)
{
    return GotohAligner<TYPE,scoring_scheme_type,TextBlockingTag>( aligner.scheme );
}
template <AlignmentType TYPE, typename scoring_scheme_type>
GotohAligner<TYPE,scoring_scheme_type,PatternBlockingTag> transpose(const GotohAligner<TYPE,scoring_scheme_type,TextBlockingTag>& aligner)
{
    return GotohAligner<TYPE,scoring_scheme_type,PatternBlockingTag>( aligner.scheme );
}

/// A Smith-Waterman alignment algorithm, see \ref Aligner
/// \anchor SmithWatermanAligner
///
/// \tparam TYPE                    specifies whether the alignment is LOCAL/SEMI_GLOBAL/GLOBAL
/// \tparam scoring_scheme_type     specifies the scoring scheme, a model of \ref SmithWatermanScoringScheme
/// \tparam AlgorithmType           specifies the \ref AlgorithmTag "Algorithm Tag"
///
///\section SmithWatermanScoringScheme Smith-Waterman Scoring Scheme
/// A Smith-Waterman scoring scheme is a class exposing the following interface:
/// \code
/// struct SmithWatermanScoringScheme
/// {
///     // the match bonus at a given quality q 
///     int32 match(const uint8 q = 0)      const;
///
///     // the mismatch penalty at a given quality q 
///     int32 mismatch(const uint8 q = 0)   const;
///
///     // the text deletion penalty
///     int32 deletion()                    const;
///
///     // the text insertion penalty
///     int32 insertion()                   const;
/// };
/// \endcode
///
template <AlignmentType T_TYPE, typename scoring_scheme_type, typename AlgorithmType = PatternBlockingTag>
struct SmithWatermanAligner
{
    static const AlignmentType TYPE =   T_TYPE;         ///< the AlignmentType

    typedef SmithWatermanTag            aligner_tag;    ///< the \ref AlignerTag "Aligner Tag"
    typedef AlgorithmType               algorithm_tag;  ///< the \ref AlgorithmTag "Algorithm Tag"

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SmithWatermanAligner(const scoring_scheme_type _scheme) : scheme(_scheme) {}

    scoring_scheme_type scheme;
};

template <AlignmentType T_TYPE, typename scoring_scheme_type, typename AlgorithmTag>
struct transpose_aligner< SmithWatermanAligner<T_TYPE, scoring_scheme_type, AlgorithmTag> >
{
    typedef SmithWatermanAligner<T_TYPE, scoring_scheme_type, typename transpose_tag<AlgorithmTag>::type> type;
};

template <AlignmentType TYPE, typename scoring_scheme_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
SmithWatermanAligner<TYPE,scoring_scheme_type> make_smith_waterman_aligner(const scoring_scheme_type& scheme)
{
    return SmithWatermanAligner<TYPE,scoring_scheme_type>( scheme );
}

template <AlignmentType TYPE, typename scoring_scheme_type>
SmithWatermanAligner<TYPE,scoring_scheme_type,TextBlockingTag> transpose(const SmithWatermanAligner<TYPE,scoring_scheme_type,PatternBlockingTag>& aligner)
{
    return SmithWatermanAligner<TYPE,scoring_scheme_type,TextBlockingTag>( aligner.scheme );
}
template <AlignmentType TYPE, typename scoring_scheme_type>
SmithWatermanAligner<TYPE,scoring_scheme_type,PatternBlockingTag> transpose(const SmithWatermanAligner<TYPE,scoring_scheme_type,TextBlockingTag>& aligner)
{
    return SmithWatermanAligner<TYPE,scoring_scheme_type,PatternBlockingTag>( aligner.scheme );
}

/// An edit distance alignment algorithm, see \ref Aligner
/// \anchor HammingDistanceAligner
///
/// \tparam T_TYPE                    specifies whether the alignment is SEMI_GLOBAL/GLOBAL
///
template <AlignmentType T_TYPE, typename scoring_scheme_type, typename AlgorithmType = PatternBlockingTag>
struct HammingDistanceAligner
{
    static const AlignmentType TYPE =   T_TYPE;         ///< the AlignmentType

    typedef HammingDistanceTag            aligner_tag;  ///< the \ref AlignerTag "Aligner Tag"
    typedef AlgorithmType               algorithm_tag;  ///< the \ref AlgorithmTag "Algorithm Tag"

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    HammingDistanceAligner(const scoring_scheme_type _scheme) : scheme(_scheme) {}

    scoring_scheme_type scheme;
};

template <AlignmentType T_TYPE, typename scoring_scheme_type, typename AlgorithmTag>
struct transpose_aligner< HammingDistanceAligner<T_TYPE,scoring_scheme_type,AlgorithmTag> >
{
    typedef HammingDistanceAligner<T_TYPE,scoring_scheme_type,typename transpose_tag<AlgorithmTag>::type> type;
};

template <AlignmentType TYPE, typename scoring_scheme_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
HammingDistanceAligner<TYPE,scoring_scheme_type> make_hamming_distance_aligner(const scoring_scheme_type& scheme)
{
    return HammingDistanceAligner<TYPE,scoring_scheme_type>( scheme );
}

template <AlignmentType TYPE, typename scoring_scheme_type, typename algorithm_tag>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
HammingDistanceAligner<TYPE,scoring_scheme_type,algorithm_tag> make_hamming_distance_aligner(const scoring_scheme_type& scheme)
{
    return HammingDistanceAligner<TYPE,scoring_scheme_type,algorithm_tag>( scheme );
}

template <AlignmentType TYPE, typename scoring_scheme_type>
HammingDistanceAligner<TYPE,scoring_scheme_type,TextBlockingTag> transpose(const HammingDistanceAligner<TYPE,scoring_scheme_type,PatternBlockingTag>& aligner)
{
    return HammingDistanceAligner<TYPE,scoring_scheme_type,TextBlockingTag>( aligner.scheme );
}
template <AlignmentType TYPE, typename scoring_scheme_type>
HammingDistanceAligner<TYPE,scoring_scheme_type,PatternBlockingTag> transpose(const HammingDistanceAligner<TYPE,scoring_scheme_type,TextBlockingTag>& aligner)
{
    return HammingDistanceAligner<TYPE,scoring_scheme_type,PatternBlockingTag>( aligner.scheme );
}

///@} // end of the Aligner group

///@} // end of the Alignment group

} // namespace aln
} // namespace nvbio

#include <nvbio/alignment/utils.h>
