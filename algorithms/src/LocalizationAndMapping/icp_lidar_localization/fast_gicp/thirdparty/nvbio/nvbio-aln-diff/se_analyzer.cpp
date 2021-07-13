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

#include <nvbio-aln-diff/se_analyzer.h>
#include <nvbio-aln-diff/html.h>
#include <nvbio-aln-diff/utils.h>

namespace nvbio {
namespace alndiff {

SEAnalyzer::SEAnalyzer(Filter& filter) :
    m_filter( filter )
{
    n = 0;
    n_mismatched = 0;
}

void SEAnalyzer::push(
    const Alignment& alnL,
    const Alignment& alnR)
{
    if (alnL.read_id  != alnR.read_id ||
        alnL.read_len != alnR.read_len)
    {
        n_mismatched++;
        return;
    }

    mapped.push( alnL.is_mapped(), alnR.is_mapped() );
    unique.push( alnL.is_unique(), alnR.is_unique() );
    ambiguous.push( alnL.is_ambiguous(), alnR.is_ambiguous() );

    if ((alnL.is_mapped()    == true) && (alnR.is_mapped()    == false)) mapped_L_not_R_by_mapQ.push( log_bin( alnL.mapQ ) );
    if ((alnR.is_mapped()    == true) && (alnL.is_mapped()    == false)) mapped_R_not_L_by_mapQ.push( log_bin( alnR.mapQ ) );
    if ((alnL.is_unique()    == true) && (alnR.is_unique()    == false)) unique_L_not_R_by_mapQ.push( log_bin( alnL.mapQ ) );
    if ((alnR.is_unique()    == true) && (alnL.is_unique()    == false)) unique_R_not_L_by_mapQ.push( log_bin( alnR.mapQ ) );
    if ((alnL.is_ambiguous() == true) && (alnR.is_ambiguous() == false)) ambiguous_L_not_R_by_mapQ.push( log_bin( alnL.mapQ ) );
    if ((alnR.is_ambiguous() == true) && (alnL.is_ambiguous() == false)) ambiguous_R_not_L_by_mapQ.push( log_bin( alnR.mapQ ) );

    if (alnL.is_mapped() && alnR.is_mapped())
    {
        const uint32 mapQ_bin = log_bin( alnR.mapQ );

        uint32 read_flags = 0u;

        if (alnL.ref_id != alnR.ref_id)
        {
            n_different_ref.push( mapQ_bin );
            n_distant.push( mapQ_bin );
            read_flags |= Filter::DISTANT;
            read_flags |= Filter::DIFFERENT_REF;
        }
        else if (alndiff::distant( alnL, alnR ))
        {
            n_distant.push( mapQ_bin );
            read_flags |= Filter::DISTANT;
        }

        if (alnL.is_rc() != alnR.is_rc())
        {
            n_discordant.push( mapQ_bin );
            read_flags |= Filter::DISCORDANT;
        }

        const uint32 length_bin = read_length_bin( alnL.read_len );

        // generic stats
        {
            // ed
            m_filter( al_stats.lower_ed.push( alnL.ed, alnR.ed, length_bin, alnL.mapQ, alnR.mapQ ), read_flags, Filter::ED, alnL.read_id );

            // mapQ
            m_filter( al_stats.higher_mapQ.push( alnL.mapQ, alnR.mapQ, length_bin, alnL.mapQ, alnR.mapQ ), read_flags, Filter::MAPQ, alnL.read_id );

            // longer mapping
            al_stats.longer_mapping.push( alnL.mapped_read_bases(), alnR.mapped_read_bases(), length_bin, alnL.mapQ, alnR.mapQ );

            al_stats.lower_subs.push( alnL.subs, alnR.subs, length_bin, alnL.mapQ, alnR.mapQ );
            m_filter( al_stats.lower_mms.push(  alnL.n_mm, alnR.n_mm, length_bin, alnL.mapQ, alnR.mapQ ), read_flags, Filter::MMS,  alnL.read_id );
            m_filter( al_stats.lower_ins.push(  alnL.ins,  alnR.ins,  length_bin, alnL.mapQ, alnR.mapQ ), read_flags, Filter::INS,  alnL.read_id );
            m_filter( al_stats.lower_dels.push( alnL.dels, alnR.dels, length_bin, alnL.mapQ, alnR.mapQ ), read_flags, Filter::DELS, alnL.read_id );

            al_stats.higher_pos.push( alnL.pos, alnR.pos, length_bin, alnL.mapQ, alnR.mapQ );
        }
        if (read_flags & Filter::DISTANT)
        {
            // ed
            distant_stats.lower_ed.push( alnL.ed, alnR.ed, length_bin, alnL.mapQ, alnR.mapQ );

            // mapQ
            distant_stats.higher_mapQ.push( alnL.mapQ, alnR.mapQ, length_bin, alnL.mapQ, alnR.mapQ );

            // longer mapping
            distant_stats.longer_mapping.push( alnL.mapped_read_bases(), alnR.mapped_read_bases(), length_bin, alnL.mapQ, alnR.mapQ );

            distant_stats.lower_subs.push( alnL.subs, alnR.subs, length_bin, alnL.mapQ, alnR.mapQ );
            distant_stats.lower_mms.push(  alnL.n_mm, alnR.n_mm, length_bin, alnL.mapQ, alnR.mapQ );
            distant_stats.lower_ins.push(  alnL.ins,  alnR.ins,  length_bin, alnL.mapQ, alnR.mapQ );
            distant_stats.lower_dels.push( alnL.dels, alnR.dels, length_bin, alnL.mapQ, alnR.mapQ );
            distant_stats.higher_pos.push( alnL.pos,  alnR.pos,  length_bin, alnL.mapQ, alnR.mapQ );
        }
        if (read_flags & Filter::DISCORDANT)
        {
            // ed
            discordant_stats.lower_ed.push( alnL.ed, alnR.ed, length_bin, alnL.mapQ, alnR.mapQ );

            // mapQ
            discordant_stats.higher_mapQ.push( alnL.mapQ, alnR.mapQ, length_bin, alnL.mapQ, alnR.mapQ );

            // longer mapping
            discordant_stats.longer_mapping.push( alnL.mapped_read_bases(), alnR.mapped_read_bases(), length_bin, alnL.mapQ, alnR.mapQ );

            discordant_stats.lower_subs.push( alnL.subs, alnR.subs, length_bin, alnL.mapQ, alnR.mapQ );
            discordant_stats.lower_mms.push(  alnL.n_mm, alnR.n_mm, length_bin, alnL.mapQ, alnR.mapQ );
            discordant_stats.lower_ins.push(  alnL.ins,  alnR.ins,  length_bin, alnL.mapQ, alnR.mapQ );
            discordant_stats.lower_dels.push( alnL.dels, alnR.dels, length_bin, alnL.mapQ, alnR.mapQ );
            discordant_stats.higher_pos.push( alnL.pos,  alnR.pos,  length_bin, alnL.mapQ, alnR.mapQ );
        }
    }

    ++n;
}

namespace {

void generate_summary_header(FILE* html_output)
{
    html::tr_object tr( html_output, NULL );
    html::th_object( html_output, html::FORMATTED, NULL, "" );
    html::th_object( html_output, html::FORMATTED, NULL, "better" );
    html::th_object( html_output, html::FORMATTED, "class", "red", NULL, "worse" );
}
template <typename StatsType>
void generate_summary_cell(FILE* html_output, const std::string file_name, const char* type, const StatsType& stats, const uint32 n)
{
    char link_name[1024];
    sprintf( link_name, "<a href=\"%s\">%s</a>", local_file( file_name ), type );
    html::tr_object tr( html_output, "class", "alt", NULL );
    html::th_object( html_output, html::FORMATTED, NULL, link_name );
    html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%", 100.0f * float(stats.l.diff_hist.all_but(0))/float(n) );
    html::td_object( html_output, html::FORMATTED, "class", "pink", NULL, "%5.2f %%", 100.0f * float(stats.r.diff_hist.all_but(0))/float(n) );
}

} // anonymous namespace

void SEAnalyzer::generate_report(const char* aln_file_name1, const char* aln_file_name2, const char* report)
{
    if (report == NULL)
        return;

    const std::string mapped_bps_name     = generate_file_name( report, "mapped-bps" );
    const std::string ed_name             = generate_file_name( report, "ed" );
    const std::string mapQ_name           = generate_file_name( report, "mapQ" );
    const std::string mms_name            = generate_file_name( report, "mms" );
    const std::string ins_name            = generate_file_name( report, "ins" );
    const std::string dels_name           = generate_file_name( report, "dels" );
    const std::string pos_name            = generate_file_name( report, "pos" );

    const std::string distant_mapped_bps_name     = generate_file_name( report, "distant_stats.mapped-bps" );
    const std::string distant_ed_name             = generate_file_name( report, "distant_stats.ed" );
    const std::string distant_mapQ_name           = generate_file_name( report, "distant_stats.mapQ" );
    const std::string distant_mms_name            = generate_file_name( report, "distant_stats.mms" );
    const std::string distant_ins_name            = generate_file_name( report, "distant_stats.ins" );
    const std::string distant_dels_name           = generate_file_name( report, "distant_stats.dels" );
    const std::string distant_pos_name            = generate_file_name( report, "distant_stats.pos" );

    const std::string discordant_mapped_bps_name     = generate_file_name( report, "discordant_stats.mapped-bps" );
    const std::string discordant_ed_name             = generate_file_name( report, "discordant_stats.ed" );
    const std::string discordant_mapQ_name           = generate_file_name( report, "discordant_stats.mapQ" );
    const std::string discordant_mms_name            = generate_file_name( report, "discordant_stats.mms" );
    const std::string discordant_ins_name            = generate_file_name( report, "discordant_stats.ins" );
    const std::string discordant_dels_name           = generate_file_name( report, "discordant_stats.dels" );
    const std::string discordant_pos_name            = generate_file_name( report, "discordant_stats.pos" );

    generate_table( mapped_bps_name.c_str(),    "mapped L & R", "mapped bps",    "mapped bps diff",     al_stats.longer_mapping.bin_type(), al_stats.longer_mapping.l, al_stats.longer_mapping.r,   n, mapped.L_and_R );
    generate_table( ed_name.c_str(),            "mapped L & R", "edit distance", "edit distance diff",  al_stats.lower_ed.bin_type(),       al_stats.lower_ed.l,       al_stats.lower_ed.r,         n, mapped.L_and_R );
    generate_table( mapQ_name.c_str(),          "mapped L & R", "mapQ",          "mapQ diff",           al_stats.higher_mapQ.bin_type(),    al_stats.higher_mapQ.l,    al_stats.higher_mapQ.r,      n, mapped.L_and_R );
    generate_table( mms_name.c_str(),           "mapped L & R", "mms",           "mms diff",            al_stats.lower_mms.bin_type(),      al_stats.lower_mms.l,      al_stats.lower_mms.r,        n, mapped.L_and_R );
    generate_table( ins_name.c_str(),           "mapped L & R", "ins",           "ins diff",            al_stats.lower_ins.bin_type(),      al_stats.lower_ins.l,      al_stats.lower_ins.r,        n, mapped.L_and_R );
    generate_table( dels_name.c_str(),          "mapped L & R", "dels",          "dels diff",           al_stats.lower_dels.bin_type(),     al_stats.lower_dels.l,     al_stats.lower_dels.r,       n, mapped.L_and_R );
    generate_table( pos_name.c_str(),           "mapped L & R", "position",      "distance",            al_stats.higher_pos.bin_type(),     al_stats.higher_pos.l,     al_stats.higher_pos.r,       n, mapped.L_and_R, false );

    generate_table( distant_mapped_bps_name.c_str(),    "distant", "mapped bps",    "mapped bps diff",      distant_stats.longer_mapping.bin_type(), distant_stats.longer_mapping.l, distant_stats.longer_mapping.r,   n, n_distant.count );
    generate_table( distant_ed_name.c_str(),            "distant", "edit distance", "edit distance diff",   distant_stats.lower_ed.bin_type(),       distant_stats.lower_ed.l,       distant_stats.lower_ed.r,         n, n_distant.count );
    generate_table( distant_mapQ_name.c_str(),          "distant", "mapQ",          "mapQ diff",            distant_stats.higher_mapQ.bin_type(),    distant_stats.higher_mapQ.l,    distant_stats.higher_mapQ.r,      n, n_distant.count );
    generate_table( distant_mms_name.c_str(),           "distant", "mms",           "mms diff",             distant_stats.lower_mms.bin_type(),      distant_stats.lower_mms.l,      distant_stats.lower_mms.r,        n, n_distant.count );
    generate_table( distant_ins_name.c_str(),           "distant", "ins",           "ins diff",             distant_stats.lower_ins.bin_type(),      distant_stats.lower_ins.l,      distant_stats.lower_ins.r,        n, n_distant.count );
    generate_table( distant_dels_name.c_str(),          "distant", "dels",          "dels diff",            distant_stats.lower_dels.bin_type(),     distant_stats.lower_dels.l,     distant_stats.lower_dels.r,       n, n_distant.count );
    generate_table( distant_pos_name.c_str(),           "distant", "position",      "distance",             distant_stats.higher_pos.bin_type(),     distant_stats.higher_pos.l,     distant_stats.higher_pos.r,       n, n_distant.count, false );

    generate_table( discordant_mapped_bps_name.c_str(),    "discordant", "mapped bps",      "mapped bps diff",      discordant_stats.longer_mapping.bin_type(), discordant_stats.longer_mapping.l, discordant_stats.longer_mapping.r,   n, n_discordant.count );
    generate_table( discordant_ed_name.c_str(),            "discordant", "edit distance",   "edit distance diff",   discordant_stats.lower_ed.bin_type(),       discordant_stats.lower_ed.l,       discordant_stats.lower_ed.r,         n, n_discordant.count );
    generate_table( discordant_mapQ_name.c_str(),          "discordant", "mapQ",            "mapQ diff",            discordant_stats.higher_mapQ.bin_type(),    discordant_stats.higher_mapQ.l,    discordant_stats.higher_mapQ.r,      n, n_discordant.count );
    generate_table( discordant_mms_name.c_str(),           "discordant", "mms",             "mms diff",             discordant_stats.lower_mms.bin_type(),      discordant_stats.lower_mms.l,      discordant_stats.lower_mms.r,        n, n_discordant.count );
    generate_table( discordant_ins_name.c_str(),           "discordant", "ins",             "ins diff",             discordant_stats.lower_ins.bin_type(),      discordant_stats.lower_ins.l,      discordant_stats.lower_ins.r,        n, n_discordant.count );
    generate_table( discordant_dels_name.c_str(),          "discordant", "dels",            "dels diff",            discordant_stats.lower_dels.bin_type(),     discordant_stats.lower_dels.l,     discordant_stats.lower_dels.r,       n, n_discordant.count );
    generate_table( discordant_pos_name.c_str(),           "discordant", "position",        "distance",             discordant_stats.higher_pos.bin_type(),     discordant_stats.higher_pos.l,     discordant_stats.higher_pos.r,       n, n_discordant.count, false );

    FILE* html_output = fopen( report, "w" );
    if (html_output == NULL)
    {
        log_warning( stderr, "unable to write HTML report \"%s\"\n", report );
        return;
    }

    const Histogram<8> cum_different_ref   = reverse_cumulative( n_different_ref );
    const Histogram<8> cum_distant   = reverse_cumulative( n_distant );
    const Histogram<8> cum_discordant   = reverse_cumulative( n_discordant );

    const uint32 HI_MAPQ_BIN = 6; // >= 32
    {
        html::html_object html( html_output );
        {
            const char* meta_list = "<meta http-equiv=\"refresh\" content=\"5\" />";

            html::header_object hd( html_output, "nv-aln-diff report", html::style(), meta_list );
            {
                html::body_object body( html_output );

                //
                // alignment stats
                //
                {
                    html::table_object table( html_output, "alignment-stats", "stats", "alignment stats" );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                        html::th_object( html_output, html::FORMATTED, NULL, "L = %s", aln_file_name1 );
                        html::th_object( html_output, html::FORMATTED, NULL, "R = %s", aln_file_name2 );
                        html::th_object( html_output, html::FORMATTED, NULL, "L & R" );
                        html::th_object( html_output, html::FORMATTED, NULL, "L \\ R" );
                        html::th_object( html_output, html::FORMATTED, NULL, "R \\ L" );
                    }
                    {
                        html::tr_object tr( html_output, "class", "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "mapped");
                        html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%", 100.0f * float(mapped.L) / float(n) );
                        html::td_object( html_output, html::FORMATTED, "class", "pink",  NULL, "%5.2f %%", 100.0f * float(mapped.R) / float(n) );
                        html::td_object( html_output, html::FORMATTED, "class", "green", NULL, "%5.2f %%", 100.0f * float(mapped.L_and_R) / float(n) );
                        html::td_object( html_output, html::FORMATTED,                   NULL, "%5.2f %%   (%.3f %%)", 100.0f * float(mapped.L_not_R) / float(n), 100.0f * float(mapped_L_not_R_by_mapQ[HI_MAPQ_BIN]) / float(n) );
                        html::td_object( html_output, html::FORMATTED, "class", "pink",  NULL, "%5.2f %%   (%.3f %%)", 100.0f * float(mapped.R_not_L) / float(n), 100.0f * float(mapped_R_not_L_by_mapQ[HI_MAPQ_BIN]) / float(n) );
                    }
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "unique");
                        html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%", 100.0f * float(unique.L) / float(n) );
                        html::td_object( html_output, html::FORMATTED, "class", "pink",  NULL, "%5.2f %%", 100.0f * float(unique.R) / float(n) );
                        html::td_object( html_output, html::FORMATTED, "class", "green", NULL, "%5.2f %%", 100.0f * float(unique.L_and_R) / float(n) );
                        html::td_object( html_output, html::FORMATTED, NULL,                   "%5.2f %%   (%.3f %%)", 100.0f * float(unique.L_not_R) / float(n), 100.0f * float(unique_L_not_R_by_mapQ[HI_MAPQ_BIN]) / float(n) );
                        html::td_object( html_output, html::FORMATTED, "class", "pink",  NULL, "%5.2f %%   (%.3f %%)", 100.0f * float(unique.R_not_L) / float(n), 100.0f * float(unique_R_not_L_by_mapQ[HI_MAPQ_BIN]) / float(n) );
                    }
                    {
                        html::tr_object tr( html_output, "class", "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "ambiguous");
                        html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%", 100.0f * float(ambiguous.L) / float(n) );
                        html::td_object( html_output, html::FORMATTED, "class", "pink",  NULL, "%5.2f %%", 100.0f * float(ambiguous.R) / float(n) );
                        html::td_object( html_output, html::FORMATTED, "class", "green", NULL, "%5.2f %%", 100.0f * float(ambiguous.L_and_R) / float(n) );
                        html::td_object( html_output, html::FORMATTED,                   NULL, "%5.2f %%   (%.3f %%)", 100.0f * float(ambiguous.L_not_R) / float(n), 100.0f * float(ambiguous_L_not_R_by_mapQ[HI_MAPQ_BIN]) / float(n) );
                        html::td_object( html_output, html::FORMATTED, "class", "pink",  NULL, "%5.2f %%   (%.3f %%)", 100.0f * float(ambiguous.R_not_L) / float(n), 100.0f * float(ambiguous_R_not_L_by_mapQ[HI_MAPQ_BIN]) / float(n) );
                    }
                }
                //
                // discordance stats
                //
                {
                    html::table_object table( html_output, "discordance-stats", "stats", "discordance stats" );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                        html::th_object( html_output, html::FORMATTED, NULL, "items" );
                        html::th_object( html_output, html::FORMATTED, NULL, "%% of total" );
                    }
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "different reference" );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.2f M   (%.2f M)",             float(cum_different_ref[0]) * 1.0e-6f,           float(cum_different_ref[HI_MAPQ_BIN]) * 1.0e-6f );
                        html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%   (%.3f %%)", 100.0f * float(cum_different_ref[0]) / float(n), 100.0f * float(cum_different_ref[HI_MAPQ_BIN]) / float(n) );
                    }
                    {
                        html::tr_object tr( html_output, "class", "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "distant" );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.2f M    (%.2f M)", float(cum_distant[0]) * 1.0e-6f,                      float(cum_distant[HI_MAPQ_BIN]) * 1.0e-6f );
                        html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%   (%.3f %%)", 100.0f * float(cum_distant[0]) / float(n), 100.0f * float(cum_distant[HI_MAPQ_BIN]) / float(n) );
                    }
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "discordant" );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.2f M   (%.2f M)",             float(cum_discordant[0]) * 1.0e-6f,           float(cum_discordant[HI_MAPQ_BIN]) * 1.0e-6f );
                        html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%   (%.3f %%)", 100.0f * float(cum_discordant[0]) / float(n), 100.0f * float(cum_discordant[HI_MAPQ_BIN]) / float(n) );
                    }
                }
                //
                // summary stats
                //
                {
                    html::table_object table( html_output, "summary-stats", "stats", "summary stats" );
                    generate_summary_header( html_output );
                    generate_summary_cell( html_output, mapped_bps_name, "mapped bases", al_stats.longer_mapping,   n );
                    generate_summary_cell( html_output, ed_name,         "edit distance",al_stats.lower_ed,         n );
                    generate_summary_cell( html_output, mapQ_name,       "mapQ",         al_stats.higher_mapQ,      n );
                    generate_summary_cell( html_output, mms_name,        "mismatches",   al_stats.lower_mms,        n );
                    generate_summary_cell( html_output, ins_name,        "insertions",   al_stats.lower_ins,        n );
                    generate_summary_cell( html_output, dels_name,       "deletions",    al_stats.lower_dels,       n );
                    generate_summary_cell( html_output, pos_name,        "distance",     al_stats.higher_pos,       n );
                    // ------------------------------------------- distant -------------------------------------------------- //
                    generate_summary_header( html_output );
                    generate_summary_cell( html_output, distant_mapped_bps_name, "mapped bases [distant]", distant_stats.longer_mapping,    n );
                    generate_summary_cell( html_output, distant_ed_name,         "edit distance [distant]",distant_stats.lower_ed,          n );
                    generate_summary_cell( html_output, distant_mapQ_name,       "mapQ [distant]",         distant_stats.higher_mapQ,       n );
                    generate_summary_cell( html_output, distant_mms_name,        "mismatches [distant]",   distant_stats.lower_mms,         n );
                    generate_summary_cell( html_output, distant_ins_name,        "insertions [distant]",   distant_stats.lower_ins,         n );
                    generate_summary_cell( html_output, distant_dels_name,       "deletions [distant]",    distant_stats.lower_dels,        n );
                    generate_summary_cell( html_output, distant_pos_name,        "distance [distant]",     distant_stats.higher_pos,        n );
                    // ------------------------------------------- discordant ---------------------------------------------- //
                    generate_summary_header( html_output );
                    generate_summary_cell( html_output, discordant_mapped_bps_name, "mapped bases [discordant]", discordant_stats.longer_mapping,   n );
                    generate_summary_cell( html_output, discordant_ed_name,         "edit distance [discordant]",discordant_stats.lower_ed,         n );
                    generate_summary_cell( html_output, discordant_mapQ_name,       "mapQ [discordant]",         discordant_stats.higher_mapQ,      n );
                    generate_summary_cell( html_output, discordant_mms_name,        "mismatches [discordant]",   discordant_stats.lower_mms,        n );
                    generate_summary_cell( html_output, discordant_ins_name,        "insertions [discordant]",   discordant_stats.lower_ins,        n );
                    generate_summary_cell( html_output, discordant_dels_name,       "deletions [discordant]",    discordant_stats.lower_dels,       n );
                    generate_summary_cell( html_output, discordant_pos_name,        "distance [discordant]",     discordant_stats.higher_pos,       n );
                }

                // mapped L not R
                generate_table(
                    html_output,
                    "by mapQ",
                    "mapped (L \\ R) vs (R \\ L)",
                    LOG,
                    mapped_L_not_R_by_mapQ,
                    mapped_R_not_L_by_mapQ,
                    n,
                    n );

                // unique L not R
                generate_table(
                    html_output,
                    "by mapQ",
                    "unique (L \\ R) vs (R \\ L)",
                    LOG,
                    unique_L_not_R_by_mapQ,
                    unique_R_not_L_by_mapQ,
                    n,
                    n );

                // unique L not R
                generate_table(
                    html_output,
                    "by mapQ",
                    "ambiguous (L \\ R) vs (R \\ L)",
                    LOG,
                    ambiguous_L_not_R_by_mapQ,
                    ambiguous_R_not_L_by_mapQ,
                    n,
                    n );

                // different reference by mapQ
                generate_table(
                    html_output,
                    "mapped L & R",
                    "different reference by mapQ",
                    LOG,
                    n_different_ref,
                    mapped.L_and_R );

                // different reference by mapQ
                generate_table(
                    html_output,
                    "mapped L & R",
                    "distant by mapQ",
                    LOG,
                    n_distant,
                    mapped.L_and_R );

                // discordant reference by mapQ
                generate_table(
                    html_output,
                    "mapped L & R",
                    "discordant by mapQ",
                    LOG,
                    n_discordant,
                    mapped.L_and_R );
            }
        }
    }
    fclose( html_output );
}

} // namespace alndiff
} // namespace nvbio
