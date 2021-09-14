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

#include <nvbio-aln-diff/stats.h>
#include <nvbio-aln-diff/alignment.h>
#include <nvbio-aln-diff/filter.h>

namespace nvbio {
namespace alndiff {

struct PEAnalyzer
{
    PEAnalyzer(Filter& filter, const bool id_check);

    void push(
        const AlignmentPair& alnL,
        const AlignmentPair& alnR);

    void generate_report(const char* aln_file_nameL, const char* aln_file_nameR, const char* report);

    // flush any open files
    //
    void flush() { m_filter.flush(); }

    float mismatched() const { return float(n_mismatched)/float(n + n_mismatched); }

    float different_ref() const { return float(n_different_ref.count)/float(n); }
    float distant() const { return float(n_distant.count)/float(n); }
    float discordant() const { return float(n_discordant.count)/float(n); }
    uint32 filtered() const { return m_filter.filtered(); }

    Filter& m_filter;
    bool    m_id_check;

    BooleanStats mapped;
    BooleanStats paired;
    BooleanStats unique;
    BooleanStats ambiguous;
    BooleanStats not_ambiguous;

    Histogram<8> paired_L_not_R_by_mapQ;
    Histogram<8> paired_R_not_L_by_mapQ;
    Histogram<8> unique_L_not_R_by_mapQ;
    Histogram<8> unique_R_not_L_by_mapQ;
    Histogram<8> ambiguous_L_not_R_by_mapQ;
    Histogram<8> ambiguous_R_not_L_by_mapQ;

    uint32 n;
    uint32 n_mismatched;

    Histogram<8> n_different_ref12;
    Histogram<8> n_different_ref1;
    Histogram<8> n_different_ref2;
    Histogram<8> n_different_ref;
    Histogram<8> n_different_ref_unique;
    Histogram<8> n_different_ref_not_ambiguous;

    Histogram<8> n_distant12;
    Histogram<8> n_distant1;
    Histogram<8> n_distant2;
    Histogram<8> n_distant;
    Histogram<8> n_distant_unique;
    Histogram<8> n_distant_not_ambiguous;

    Histogram<8> n_discordant12;
    Histogram<8> n_discordant1;
    Histogram<8> n_discordant2;
    Histogram<8> n_discordant;
    Histogram<8> n_discordant_unique;
    Histogram<8> n_discordant_not_ambiguous;

    AlignmentStats al_stats;
    AlignmentStats distant_stats;
    AlignmentStats discordant_stats;

    Histogram2d<12,12> sec_score_by_score_l;
    Histogram2d<12,12> sec_score_by_score_r;

    Histogram2d<16,16> sec_ed_by_ed_l;
    Histogram2d<16,16> sec_ed_by_ed_r;
};

} // namespace alndiff
} // namespace nvbio