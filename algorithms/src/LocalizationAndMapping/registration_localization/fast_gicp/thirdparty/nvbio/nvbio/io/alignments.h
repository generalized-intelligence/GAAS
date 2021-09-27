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

//#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cached_iterator.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/pod.h>
#include <algorithm>

#pragma once

namespace nvbio {
namespace io {

///@addtogroup IO
///@{

///
/// The list of MD string operations
///
enum MDS_OP {
    MDS_MATCH       = 0,
    MDS_MISMATCH    = 1,
    MDS_INSERTION   = 2,
    MDS_DELETION    = 3,
    MDS_INVALID     = 4
};

///
/// A tight 16-bit CIGAR element representation
///
struct Cigar
{
    /// The list of CIGAR operations
    ///
    enum Operation {
        SUBSTITUTION  = 0,
        INSERTION     = 1,
        DELETION      = 2,
        SOFT_CLIPPING = 3,
    };

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    Cigar() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    Cigar(const uint8 type, const uint16 len) : m_type( type ), m_len( len ) {}

    uint16 m_type:2, m_len:14;
};

///
/// A simple data-structure to represent an alignment
///
struct Alignment
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    static uint32 max_ed() { return 255u; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    static int32 max_score() { return (1 << 17) - 1; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
    static int32 min_score() { return -((1 << 17) - 1); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE Alignment() {}
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE Alignment(
        const uint32 pos,
        const uint32 ed,
        const  int32 score,
        const uint32 rc,
        const uint32 mate = 0,
        const bool   paired = false,
        const bool   discordant = false)
    {
        //NVBIO_CUDA_ASSERT( ed < 1024u );
        //NVBIO_CUDA_ASSERT( score >= min_score() && score <= max_score() );
        m_align      = pos;
        m_ed         = ed;
        m_score      = score < 0 ? uint32( -score ) : uint32( score );
        m_score_sgn  = score < 0 ? 1u               : 0u;
        m_rc         = rc;
        m_mate       = mate;
        m_paired     = paired ? 1u : 0u;
        m_discordant = discordant ? 1u : 0u;
    }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE int32  score()         const { return m_score_sgn ? -int32( m_score ) : int32( m_score ); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_aligned()    const { return m_align != uint32(-1); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 alignment()     const { return m_align; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_rc()         const { return m_rc; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 ed()            const { return m_ed; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 sink()          const { return m_ed; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 mate()          const { return m_mate; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_paired()     const { return (m_paired == true)  && is_aligned(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_unpaired()   const { return (m_paired == false) && is_aligned(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_concordant() const { return (m_paired == true)  && (m_discordant == false); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_discordant() const { return (m_paired == true)  && (m_discordant == true); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    static Alignment invalid() { return Alignment( uint32(-1), max_ed(), max_score(), 0u, 0u, false ); }

    uint32 m_score_sgn : 1, m_score:17, m_ed:10, m_rc:1, m_mate:1, m_paired:1, m_discordant:1;
    uint32 m_align;
};

///
/// A simple functor to compare alignments
///
struct AlignmentCompare
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const Alignment f, const Alignment s) const { return (f.m_score > s.m_score); }
};

///
/// A simple data-structure to hold the best 2 alignments
///
struct BestAlignments
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    BestAlignments() {}

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    BestAlignments(const Alignment& a1, const Alignment& a2) : m_a1( a1 ), m_a2( a2 ) {}

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_aligned()           const { return m_a1.is_aligned(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   has_second()           const { return m_a2.is_aligned(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE int32  best_score()           const { return m_a1.score(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 best_ed()              const { return m_a1.ed(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 best_alignment_pos()   const { return m_a1.alignment(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE int32  second_score()         const { return m_a2.score(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 second_ed()            const { return m_a2.ed(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 second_alignment_pos() const { return m_a2.alignment(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const Alignment& best()         const { return m_a1; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE const Alignment& second_best()  const { return m_a2; }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    const Alignment& alignment() const { return I == 0 ? m_a1 : m_a2; }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    Alignment& alignment() { return I == 0 ? m_a1 : m_a2; }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    uint32 alignment_pos() const { return I == 0 ? best_alignment_pos() : second_alignment_pos(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    int32 score() const { return I == 0 ? best_score() : second_score(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    uint32 ed() const { return I  == 0? best_ed() : second_ed(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    bool is_aligned() const { return I == 0 ? m_a1.is_aligned() : m_a2.is_aligned(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    bool is_rc() const { return I == 0 ? m_a1.is_rc() : m_a2.is_rc(); }

    Alignment m_a1;
    Alignment m_a2;
};

///
/// A simple data-structure to hold paired alignments
///
struct PairedAlignments
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    PairedAlignments() {}

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    PairedAlignments(const Alignment& a, const Alignment& o) : m_a( a ), m_o( o ) {}

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_aligned()           const { return m_a.is_aligned() && m_o.is_aligned(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_paired()            const { return m_a.is_paired(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_concordant()        const { return m_a.is_concordant(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_discordant()        const { return m_a.is_discordant(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE int32  score()                const { return m_a.score() + m_o.score(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE int32  ed()                   const { return m_a.ed()    + m_o.ed(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    Alignment& mate(const uint32 m) { return m == m_a.mate() ? m_a : m_o; }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    Alignment mate(const uint32 m) const { return m == m_a.mate() ? m_a : m_o; }

    Alignment m_a;
    Alignment m_o;
};

///
/// A simple data-structure to hold the best 2 paired alignments
///
struct BestPairedAlignments
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    BestPairedAlignments() {}

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    BestPairedAlignments(const BestAlignments& a, const BestAlignments& o) : m_a1( a.m_a1 ), m_a2( a.m_a2 ), m_o1( o.m_a1 ), m_o2( o.m_a2 ) {}

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    BestPairedAlignments(const BestAlignments& a) : m_a1( a.m_a1 ), m_a2( a.m_a2 ), m_o1( Alignment::invalid() ), m_o2( Alignment::invalid() ) {}

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_paired()            const { return m_a1.is_paired(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   is_aligned()           const { return m_a1.is_aligned() /*&& m_o1.is_aligned()*/; }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   has_second_paired()    const { return m_a2.is_paired(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   has_second_unpaired()  const { return m_a2.is_unpaired(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE bool   has_second()           const { return is_paired() ? has_second_paired() : m_a2.is_aligned(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE int32  best_score()           const { return m_a1.score() + (is_paired() ? m_o1.score() : 0); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 best_ed()              const { return m_a1.ed()    + (is_paired() ? m_o1.ed()    : 0u); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 best_alignment_pos()   const { return m_a1.alignment(); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE int32  second_score()         const { return m_a2.score() + (has_second_paired() ? m_o2.score() : 0); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 second_ed()            const { return m_a2.ed()    + (has_second_paired() ? m_o2.ed()    : 0u); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE uint32 second_alignment_pos() const { return m_a2.alignment(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE BestAlignments best_anchor()   const { return BestAlignments( m_a1, m_a2 ); }
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE BestAlignments best_opposite() const { return BestAlignments( m_o1, m_o2 ); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    PairedAlignments pair() const { return I == 0 ? PairedAlignments( m_a1, m_o1 ) : PairedAlignments( m_a2, m_o2 ); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    const Alignment& alignment() const { return I == 0 ? m_a1 : m_a2; }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    Alignment& alignment() { return I == 0 ? m_a1 : m_a2; }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    const Alignment& opposite_alignment() const { return I == 0 ? m_o1 : m_o2; }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    Alignment& opposite_alignment() { return I == 0 ? m_o1 : m_o2; }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    uint32 alignment_pos() const { return I == 0 ? best_alignment_pos() : second_alignment_pos(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    uint32 opposite_alignment_pos() const { return I == 0 ? m_o1.alignment() : m_o2.alignment(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    int32 score() const { return I == 0 ? best_score() : second_score(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    uint32 ed() const { return I  == 0? best_ed() : second_ed(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    bool is_aligned() const { return I == 0 ? m_a1.is_aligned() : m_a2.is_aligned(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    bool is_rc() const { return I == 0 ? m_a1.is_rc() : m_a2.is_rc(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    bool is_opposite_rc() const { return I == 0 ? m_o1.is_rc() : m_o2.is_rc(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    uint32 anchor_mate() const { return I == 0 ? m_a1.mate() : m_a2.mate(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    uint32 opposite_mate() const { return I == 0 ? m_o1.mate() : m_o2.mate(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    int32 anchor_score() const { return I == 0 ? m_a1.score() : m_a2.score(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    uint32 anchor_ed() const { return I  == 0? m_a1.ed() : m_a2.ed(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    int32 opposite_score() const { return I == 0 ? m_o1.score() : m_o2.score(); }

    template <uint32 I>
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    uint32 opposite_ed() const { return I == 0 ? m_o1.ed() : m_o2.ed(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    BestAlignments anchor() const { return BestAlignments( m_a1, m_a2 ); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    BestAlignments opposite() const { return BestAlignments( m_o1, m_o2 ); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE
    BestAlignments mate(const uint32 m) const
    {
        return BestAlignments(
            m == m_a1.mate() ? m_a1 : m_o1,
            m == m_a2.mate() ? m_a2 : m_o2 );
    }

    Alignment m_a1;
    Alignment m_a2;
    Alignment m_o1;
    Alignment m_o2;
};

struct has_second
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestAlignments& op) { return op.has_second(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestPairedAlignments& op) { return op.has_second(); }
};

struct has_second_paired
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestAlignments& op) { return op.has_second() && op.alignment<1>().is_paired(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestPairedAlignments& op) { return op.has_second_paired(); }
};

struct has_second_unpaired
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestAlignments& op) { return op.has_second() && op.alignment<1>().is_unpaired() ; }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestPairedAlignments& op) { return op.has_second_unpaired(); }
};

struct is_paired
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestAlignments& op) { return op.alignment<0>().is_paired(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const Alignment& op) { return op.is_paired(); }
};
struct is_unpaired
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestAlignments& op) { return op.alignment<0>().is_unpaired(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const Alignment& op) { return op.is_unpaired(); }
};
struct is_concordant
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestAlignments& op) { return op.alignment<0>().is_concordant(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const Alignment& op) { return op.is_concordant(); }
};
struct is_discordant
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestAlignments& op) { return op.alignment<0>().is_discordant(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const Alignment& op) { return op.is_discordant(); }
};
struct is_not_concordant // note: !concordant = discordant | unpaired - i.e. this is different from is_discordant
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestAlignments& op) { return op.alignment<0>().is_concordant() == false; }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const Alignment& op) { return op.is_concordant() == false; }
};
struct is_aligned
{
    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const BestAlignments& op) { return op.alignment<0>().is_aligned(); }

    NVBIO_HOST_DEVICE NVBIO_FORCEINLINE 
    bool operator() (const Alignment& op) { return op.is_aligned(); }
};

// check whether two alignments are distinct
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const uint32 pos1,
    const bool   rc1,
    const uint32 pos2,
    const bool   rc2,
    const uint32 dist);

// check whether two paired-end alignments are distinct
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const uint32 apos1,
    const uint32 opos1,
    const bool   arc1,
    const bool   orc1,
    const uint32 apos2,
    const uint32 opos2,
    const bool   arc2,
    const bool   orc2,
    const uint32 dist);

// check whether two alignments are distinct
//
//   NOTE: this uses Alignment::sink() which is only valid during alignment,
//   as sink() is unioned with ed().
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const Alignment& p1,
    const Alignment& p2,
    const uint32     dist = 1);

// check whether two paired-end alignments are distinct
//
//   NOTE: this uses Alignment::sink() which is only valid during alignment,
//   as sink() is unioned with ed().
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const PairedAlignments& p1,
    const PairedAlignments& p2);

// check whether two paired-end alignments are distinct
//
//   NOTE: this uses Alignment::sink() which is only valid during alignment,
//   as sink() is unioned with ed().
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const PairedAlignments& p1,
    const PairedAlignments& p2,
    const uint32            dist);

///@} // IO

} // namespace io

// pod_type specializations
template <> struct pod_type<io::Alignment>      { typedef uint2 type; };
template <> struct pod_type<io::BestAlignments> { typedef uint4 type; };

} // namespace nvbio

#include <nvbio/io/alignments_inl.h>
