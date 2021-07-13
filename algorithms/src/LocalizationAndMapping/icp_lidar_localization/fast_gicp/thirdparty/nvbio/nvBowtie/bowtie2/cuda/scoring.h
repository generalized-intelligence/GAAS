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

///
///\file scoring.h
///

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/options.h>
#include <nvbio/alignment/alignment_base.h>
#include <nvBowtie/bowtie2/quality_coeffs.h>
#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/func.h>
#include <string>
#include <map>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

///@addtogroup nvBowtie
///@{

///@addtogroup Alignment
///@{

enum AlignmentType {
    EndToEndAlignment = 0,
    LocalAlignment    = 1,
};

enum CostType {
	ROUNDED_QUAL_COST   = 1,
	QUAL_COST           = 2,
	CONSTANT_COST       = 3
};

///
/// Rounded quality cost function
///
template <typename T>
struct RoundedQualCost
{
    static const CostType COST_TYPE = ROUNDED_QUAL_COST;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    RoundedQualCost() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    RoundedQualCost(const T min_val, const T max_val) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    T operator() (const int i) const { return T( phred_to_maq(i) ); }
};

///
/// Simple quality cost function
///
template <typename T>
struct QualCost
{
    static const CostType COST_TYPE = QUAL_COST;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    QualCost() : m_min_val(0), m_max_val(0) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    QualCost(const T min_val, const T max_val) : m_min_val( min_val ), m_max_val( max_val ) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    T operator() (const int i) const
    {
        const float frac = (float)(nvbio::min(i, 40) / 40.0f);
        return m_min_val + T( frac * (m_max_val - m_min_val) );
    }

    T m_min_val;
    T m_max_val;
};

///
/// Constant cost function
///
template <typename T>
struct ConstantCost
{
    static const CostType COST_TYPE = CONSTANT_COST;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ConstantCost() : m_val(0) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ConstantCost(const T min_val, const T max_val) : m_val( max_val ) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    T operator() (const int i) const { return T( m_val ); }

    T m_val;
};

///
/// This class implements an edit-distance based scoring scheme for short read alignment.
/// While the original bowtie2 code base always performs alignment using Gotoh's algorithm
/// (i.e. Smith-Waterman with affine gap penalties), nvBowtie also supports edit-distance,
/// which is sometimes better suited for very short reads (e.g. ~100bps).
///
struct EditDistanceScoringScheme
{
    typedef EditDistanceScoringScheme    scheme_type;
    typedef SimpleFunc                   threshold_score_type;
    typedef aln::EditDistanceTag         aligner_tag;

    typedef aln::EditDistanceAligner<aln::LOCAL>        local_aligner_type;
    typedef aln::EditDistanceAligner<aln::SEMI_GLOBAL>  end_to_end_aligner_type;

    typedef aln::EditDistanceAligner<aln::LOCAL>        ungapped_local_aligner_type;
    typedef aln::EditDistanceAligner<aln::SEMI_GLOBAL>  ungapped_end_to_end_aligner_type;

    static const int32 inf_score     =  0;
    static const int32 worst_score   = -(1 << 8);

    /// return the local aligner
    ///
    local_aligner_type local_aligner() const { return local_aligner_type(); }

    /// return the end_to_end aligner
    ///
    end_to_end_aligner_type end_to_end_aligner() const { return end_to_end_aligner_type(); }

    /// return the local aligner
    ///
    ungapped_local_aligner_type ungapped_local_aligner() const { return ungapped_local_aligner_type(); }

    /// return the end_to_end aligner
    ///
    ungapped_end_to_end_aligner_type ungapped_end_to_end_aligner() const { return ungapped_end_to_end_aligner_type(); }

    // ---------- constructors -------------------------------------------------------------------- //

    /// default constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    EditDistanceScoringScheme() : m_score_min( SimpleFunc::LinearFunc, -5.0f, 0.0f ) {}

    // ---------- begin: limits ------------------------------------------------------------------- //

    /// best achievable score for a read of given length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    int32 perfect_score(const uint32 read_len) const { return 0; }

    /// min score
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    int32 min_score(const uint32 len) const { return m_score_min( len ); }

    /// min score function
    ///
    SimpleFunc threshold_score() const
    {
        return m_score_min;
    }

    // ---------- end: limits ---------------------------------------------------------------------- //

public:
    SimpleFunc      m_score_min;            // minimum read score function
};

///
/// This class implements a scoring scheme for short read alignment,
/// taking into account matches, mismatches, insertions, deletions,
/// ambiguous bases and base qualities.
/// In practice, it reproduces exactly bowtie2's original scoring
/// scheme employing a Gotoh aligner.
///
template <
    typename MMCost = QualCost<int>,
    typename NCost  = ConstantCost<int> >
struct SmithWatermanScoringScheme
{
    typedef SmithWatermanScoringScheme<MMCost,NCost>    scheme_type;
    typedef aln::GotohTag                               aligner_tag;

    typedef aln::GotohAligner<aln::LOCAL,scheme_type>        local_aligner_type;
    typedef aln::GotohAligner<aln::SEMI_GLOBAL,scheme_type>  end_to_end_aligner_type;

    typedef aln::HammingDistanceAligner<aln::LOCAL,scheme_type>        ungapped_local_aligner_type;
    typedef aln::HammingDistanceAligner<aln::SEMI_GLOBAL,scheme_type>  ungapped_end_to_end_aligner_type;

    typedef ConstantCost<int> MatchCost;
    typedef MMCost            MismatchCost;

    typedef MatchCost         match_cost_function;
    typedef MismatchCost      mismatch_cost_function;
    typedef NCost             N_cost_function;

    typedef SimpleFunc        threshold_score_type;

    static const int32 inf_score     = -(1 << 16);
    static const int32 worst_score   =  inf_score;

    /// return the local aligner
    ///
    local_aligner_type local_aligner() const { return local_aligner_type(*this); }

    /// return the end_to_end aligner
    ///
    end_to_end_aligner_type end_to_end_aligner() const { return end_to_end_aligner_type(*this); }

    /// return the local aligner
    ///
    ungapped_local_aligner_type ungapped_local_aligner() const { return ungapped_local_aligner_type(*this); }

    /// return the end_to_end aligner
    ///
    ungapped_end_to_end_aligner_type ungapped_end_to_end_aligner() const { return ungapped_end_to_end_aligner_type(*this); }

    // ---------- constructors -------------------------------------------------------------------- //

    /// return a preconfigured scoring scheme
    ///
    static SmithWatermanScoringScheme base1();

    /// return preconfigured scoring scheme for local mapping
    ///
    static SmithWatermanScoringScheme local();

    /// default constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SmithWatermanScoringScheme();

    /// constructor
    ///
    /// \param options          key/value string options
    SmithWatermanScoringScheme(
        const std::map<std::string,std::string>& options,
        const AlignmentType type = LocalAlignment);

    // ---------- begin: limits ------------------------------------------------------------------- //

    /// min score
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    int32 min_score(const uint32 len) const { return m_score_min( len ); }

    /// min score function
    ///
    SimpleFunc threshold_score() const { return m_score_min; }

	/// best achievable score for a read of given length
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	int32 perfect_score(const uint32 read_len) const { return int32(read_len) * match( 0 ); }

    // ---------- end: limits ---------------------------------------------------------------------- //

    // -------- begin: aln::GotohAligner interface ----------------------------------------------------------------- //
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 match(const uint8 q = 0)      const { return  m_match(q); }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 mismatch(const uint8 q = 0)   const { return -m_mmp(q); }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 mismatch(const uint8 r, const uint8 q, const uint8 qq = 0)   const { return -m_mmp(qq); }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 substitution(const uint32 r_i, const uint32 q_j, const uint8 r, const uint8 q, const uint8 qq = 0)   const { return r == q ? m_match(qq) : -m_mmp(qq); }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 pattern_gap_open()            const { return -m_read_gap_const - m_read_gap_coeff; }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 pattern_gap_extension()       const { return -m_read_gap_coeff; }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 text_gap_open()               const { return -m_ref_gap_const - m_ref_gap_coeff; }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 text_gap_extension()          const { return -m_ref_gap_coeff; }
    // -------- end: aln::GotohAligner interface ------------------------------------------------------------------- //

    // -------- begin: helper methods to perform final scoring --------------------------------------------------- //
    // NOTE: technically we could use the same interface used by the SW module, but wheareas that is geared
    // towards dynamic programming, this one is simpler to use when just linearly scanning through a given
    // alignment

    /// score of a given alignment
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	int score(const uint8 read, const uint8 ref_mask, int q) const
    {
        return (read > 3 || ref_mask > 15) ?    // is there an N?
            -m_np(q) :                              // N score
            ((ref_mask & (1u << read)) != 0) ?   // is there a match ?
                 m_match(q) :                       // match score
                -m_mmp(q);                          // mismatch score
	}

	/// marginal penalty for an N with given quality
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	int n(int q) const { return m_np( q < 255 ? q : 255 ); }

	/// cumulative penalty of a gap of length 'i' in the read
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	int cumulative_insertion(const uint32 i) const
    {
        return m_read_gap_const + m_read_gap_coeff * i;
	}

	/// cumulative penalty of a gap of length 'i' in the reference
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	int cumulative_deletion(const uint32 i) const
    {
        return m_ref_gap_const + m_ref_gap_coeff * i;
	}
    // -------- end: helper methods to perform final scoring --------------------------------------------------- //

public:
    SimpleFunc      m_score_min;            // minimum read score function
    float           m_n_ceil_const;         // constant for the function governing the ceiling on the number of N's, depending on the read-length
    float           m_n_ceil_coeff;         // coefficient for the function governing the ceiling on the number of N's, depending on the read-length
    int             m_read_gap_const;       // reap gap open penalty
    int             m_read_gap_coeff;       // reap gap extension penalty
    int             m_ref_gap_const;        // reference gap open penalty
    int             m_ref_gap_coeff;        // reference gap extension penalty
    int             m_gap_free;             // length of the gap-free area at the beginning of each read
    MatchCost       m_match;                // match bonus function (function of the quality score)
    MMCost          m_mmp;                  // mismatch penalty function (function of the quality score)
    NCost           m_np;                   // N-penalty function (function of the quality score)
    bool            m_monotone;             // is this scoring scheme monotone? (i.e. match bonus == 0)
    bool            m_local;                // are we doing local alignment?

private:
    static SimpleFunc::Type func_type(const std::string& type);
    static SimpleFunc min_score_function(const std::map<std::string,std::string>& options);
    static MatchCost match_cost(const std::map<std::string,std::string>& options);
    static MMCost mm_cost(const std::map<std::string,std::string>& options);
    static NCost n_cost(const std::map<std::string,std::string>& options);
};

/// a helper struct to make an aligner given the AlignmentType (LocalAlignment|EndToEndAlignment) and the scheme (EditDistanceScoringScheme|SmithWatermanScoringScheme)
///
template <AlignmentType TYPE, typename scheme_type> struct make_aligner_dispatch {};

/// a helper struct to make an aligner given the AlignmentType (LocalAlignment|EndToEndAlignment) and the scheme (EditDistanceScoringScheme|SmithWatermanScoringScheme)
///
template <typename scheme_type> struct make_aligner_dispatch<LocalAlignment,scheme_type>    { typedef typename scheme_type::local_aligner_type type; static type make(const scheme_type& scheme) { return scheme.local_aligner(); } };

/// a helper struct to make an aligner given the AlignmentType (LocalAlignment|EndToEndAlignment) and the scheme (EditDistanceScoringScheme|SmithWatermanScoringScheme)
///
template <typename scheme_type> struct make_aligner_dispatch<EndToEndAlignment,scheme_type> { typedef typename scheme_type::local_aligner_type type; static type make(const scheme_type& scheme) { return scheme.end_to_end_aligner(); } };

/// a helper function to make an aligner given the AlignmentType (LocalAlignment|EndToEndAlignment) and the scheme (EditDistanceScoringScheme|SmithWatermanScoringScheme)
///
template <AlignmentType TYPE, typename scheme_type>
typename make_aligner_dispatch<TYPE,scheme_type>::type
make_aligner(const scheme_type& scheme) { return make_aligner_dispatch<TYPE,scheme_type>::make(); }

/// load a Smith-Waterman scoring scheme from disk
///
SmithWatermanScoringScheme<> load_scoring_scheme(const char* name, const AlignmentType type);

///
/// An uber class containing both an EditDistanceScoringScheme and a SmithWatermanScoringScheme,
/// needed to do option parsing, *before* it is known which of the two we are going to use 
///
struct UberScoringScheme
{
    EditDistanceScoringScheme       ed;
    SmithWatermanScoringScheme<>    sw;
};

///
/// select which scoring system (edit distance/smith waterman) to use based on the scoring tag
///
template <typename ScoringTagType>
struct ScoringSchemeSelector
{};

/// edit_distance_scoring_tag specialization of ScoringSchemeSelector
///
template <>
struct ScoringSchemeSelector<edit_distance_scoring_tag>
{
    typedef EditDistanceScoringScheme      type;

    static EditDistanceScoringScheme scheme(const UberScoringScheme& _scheme) { return _scheme.ed; }
};

/// smith_waterman_scoring_tag specialization of ScoringSchemeSelector
///
template <>
struct ScoringSchemeSelector<smith_waterman_scoring_tag>
{
    typedef SmithWatermanScoringScheme<>   type;

    static SmithWatermanScoringScheme<> scheme(const UberScoringScheme& _scheme) { return _scheme.sw; }
};

///
/// return the scoring scheme tag, given the scoring scheme
///
template <typename ScoringSchemeType>
struct ScoringSchemeTag {};

/// EditDistanceScoringScheme specialization of ScoringSchemeTag
///
template <>
struct ScoringSchemeTag< EditDistanceScoringScheme > { typedef edit_distance_scoring_tag type; };

/// SmithWatermanScoringScheme specialization of ScoringSchemeTag
///
template <
    typename MMCost,
    typename NCost >
struct ScoringSchemeTag< SmithWatermanScoringScheme<MMCost,NCost> > { typedef smith_waterman_scoring_tag type; };

///@}  // group Alignment
///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio

#include <nvBowtie/bowtie2/cuda/scoring_inl.h>
