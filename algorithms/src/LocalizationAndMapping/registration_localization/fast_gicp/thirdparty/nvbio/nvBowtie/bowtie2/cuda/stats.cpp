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

#include <nvBowtie/bowtie2/cuda/stats.h>
#include <nvbio/basic/html.h>
#include <nvbio/io/alignments.h>
#include <functional>
#include <algorithm>
#include <numeric>

#ifndef WIN32
#include <string>
#endif


namespace nvbio {
namespace bowtie2 {
namespace cuda {

void generate_kernel_table(const uint32 id, const char* report, const KernelStats& stats);

Stats::Stats(const Params _params) :
    n_reads(0),
    params( _params )
{
    global_time = 0.0f;

    hits_total        = 0u;
    hits_ranges       = 0u;
    hits_max          = 0u;
    hits_max_range    = 0u;
    hits_top_total    = 0u;
    hits_top_max      = 0u;
    hits_stats        = 0u;

    for (uint32 i = 0; i < 28; ++i)
        hits_bins[i] = hits_top_bins[i] = 0;

    map.name                = "map";                map.units                   = "reads";
    select.name             = "select";             select.units                = "seeds";
    sort.name               = "sort";               sort.units                  = "seeds";
    locate.name             = "locate";             locate.units                = "seeds";
    score.name              = "score";              score.units                 = "seeds";
    opposite_score.name     = "score-opposite";     opposite_score.units        = "seeds";
    backtrack.name          = "backtrack";          backtrack.units             = "reads";
    backtrack_opposite.name = "backtrack-opposite"; backtrack_opposite.units    = "reads";
    finalize.name           = "finalize";           finalize.units              = "reads";
    alignments_DtoH.name    = "alignments-DtoH";    alignments_DtoH.units       = "reads";
    read_HtoD.name          = "reads-HtoD";         read_HtoD.units             = "reads";
    read_io.name            = "reads-IO";           read_io.units               = "reads";
    io.name                 = "IO";                 io.units                    = "reads";

    opposite_score.user_names[0] = "queue::get utilization"; opposite_score.user_avg[0] = true;
    opposite_score.user_names[1] = "queue::run utilization"; opposite_score.user_avg[1] = true;
    opposite_score.user_names[2] = "queue::run T_avg";       opposite_score.user_avg[2] = true;
    opposite_score.user_names[3] = "queue::run T_sigma";     opposite_score.user_avg[3] = false;
}

void Stats::track_alignment_statistics(
    const io::BestAlignments& alignment1,
    const io::BestAlignments& alignment2,
    const uint8 mapq)
{
    n_reads++;

    const uint32 log_mapq = mapq ? nvbio::log2(mapq) + 1u : 0u;

    if (alignment1.best().is_concordant())
    {
        // keep track of mapping quality histogram
        concordant.mapq_bins[mapq]++;
        concordant.mapq_log_bins[log_mapq]++;

        // count this read as mapped
        concordant.n_mapped++;

        if (!alignment1.second_best().is_paired())
        {
            // we only have one score; count as a unique alignment
            concordant.n_unique++;
        }
        else
        {
            // we have two best scores, which implies two (or more) alignment positions
            // count as multiple alignment
            concordant.n_multiple++;
        }

        // compute final alignment score
        const int32 first  = alignment1.best().score()        + alignment2.best().score();
        const int32 second = alignment1.second_best().is_paired() ?
                             alignment1.second_best().score() + alignment2.second_best().score() :
                             Field_traits<int32>::min();

        // if the two best scores are equal, count as ambiguous
        if (first == second)
            concordant.n_ambiguous++;
        else {
            // else, the first score must be higher...
            NVBIO_CUDA_ASSERT(first > second);
            /// ... which counts as a nonambiguous alignment
            concordant.n_unambiguous++;
        }

        // compute edit distance scores
        const uint32 first_ed  = alignment1.best().ed()        + alignment2.best().ed();
        const uint32 second_ed = alignment1.second_best().is_paired() ?
                                 alignment1.second_best().ed() + alignment2.second_best().ed() : 255u;

        // update best edit-distance histograms
        if (first_ed < concordant.mapped_ed_histogram.size())
        {
            concordant.mapped_ed_histogram[first_ed]++;
            if (alignment1.best().is_rc())
                concordant.mapped_ed_histogram_fwd[first_ed]++;
            else
                concordant.mapped_ed_histogram_rev[first_ed]++;

            const uint32 log_first_ed = first_ed ? nvbio::log2(first_ed) + 1 : 0;
            concordant.mapped_log_ed_histogram[log_first_ed]++;
        }

        // track edit-distance correlation
        if (first_ed + 1 < 64)
        {
            if (second_ed + 1 < 64)
                concordant.mapped_ed_correlation[first_ed + 1][second_ed + 1]++;
            else
                concordant.mapped_ed_correlation[first_ed + 1][0]++;
        }
    }
    else if (alignment1.best().is_discordant())
    {
        // keep track of mapping quality histogram
        discordant.mapq_bins[mapq]++;
        discordant.mapq_log_bins[log_mapq]++;

        // count this read as mapped
        discordant.n_mapped++;

        if (!alignment1.second_best().is_paired())
        {
            // we only have one score; count as a unique alignment
            discordant.n_unique++;
        }
        else
        {
            // we have two best scores, which implies two (or more) alignment positions
            // count as multiple alignment
            discordant.n_multiple++;
        }

        // compute final alignment score
        const int32 first  = alignment1.best().score()        + alignment2.best().score();
        const int32 second = alignment1.second_best().is_paired() ?
                             alignment1.second_best().score() + alignment2.second_best().score() :
                             Field_traits<int32>::min();

        // if the two best scores are equal, count as ambiguous
        if (first == second)
            discordant.n_ambiguous++;
        else {
            // else, the first score must be higher...
            NVBIO_CUDA_ASSERT(first > second);
            /// ... which counts as a nonambiguous alignment
            discordant.n_unambiguous++;
        }

        // compute edit distance scores
        const uint32 first_ed  = alignment1.best().ed()        + alignment2.best().ed();
        const uint32 second_ed = alignment1.second_best().is_paired() ?
                                 alignment1.second_best().ed() + alignment2.second_best().ed() : 255u;

        // update best edit-distance histograms
        if (first_ed < discordant.mapped_ed_histogram.size())
        {
            discordant.mapped_ed_histogram[first_ed]++;
            if (alignment1.best().is_rc())
                discordant.mapped_ed_histogram_fwd[first_ed]++;
            else
                discordant.mapped_ed_histogram_rev[first_ed]++;

            const uint32 log_first_ed = first_ed ? nvbio::log2(first_ed) + 1 : 0;
            discordant.mapped_log_ed_histogram[log_first_ed]++;
        }

        // track edit-distance correlation
        if (first_ed + 1 < 64)
        {
            if (second_ed + 1 < 64)
                discordant.mapped_ed_correlation[first_ed + 1][second_ed + 1]++;
            else
                discordant.mapped_ed_correlation[first_ed + 1][0]++;
        }
    }
    else
    {
        //
        // track discordand alignments separately for each mate
        //

        const io::BestAlignments& aln1 = alignment1.best().mate() == 0 ? alignment1 : alignment2;
        const io::BestAlignments& aln2 = alignment1.best().mate() == 0 ? alignment2 : alignment1;

        track_alignment_statistics( &mate1, aln1, mapq );
        track_alignment_statistics( &mate2, aln2, mapq );
    }
}

void Stats::track_alignment_statistics(
    AlignmentStats*             mate,
    const io::BestAlignments&   alignment,
    const uint8                 mapq)
{
    // check if the mate is aligned
    if (!alignment.best().is_aligned())
    {
        mate->mapped_ed_correlation[0][0]++;
        return;
    }

    // count this read as mapped
    mate->n_mapped++;

    const uint32 log_mapq = mapq ? nvbio::log2(mapq) + 1u : 0u;

    // keep track of mapping quality histogram
    mate->mapq_bins[mapq]++;
    mate->mapq_log_bins[log_mapq]++;

    if (!alignment.second_best().is_aligned())
    {
        // we only have one score; count as a unique alignment
        mate->n_unique++;
    }
    else
    {
        // we have two best scores, which implies two (or more) alignment positions
        // count as multiple alignment
        mate->n_multiple++;
    }

    // compute final alignment score
    const int32 first  = alignment.best().score();
    const int32 second = alignment.second_best().score();

    // if the two best scores are equal, count as ambiguous
    if (first == second)
        mate->n_ambiguous++;
    else {
        // else, the first score must be higher...
        NVBIO_CUDA_ASSERT(first > second);
        /// ... which counts as a nonambiguous alignment
        mate->n_unambiguous++;
    }

    // compute edit distance scores
    const uint32 first_ed  = alignment.best().ed();
    const uint32 second_ed = alignment.second_best().ed();

    // update best edit-distance histograms
    if (first_ed < mate->mapped_ed_histogram.size())
    {
        mate->mapped_ed_histogram[first_ed]++;
        if (alignment.best().is_rc())
            mate->mapped_ed_histogram_fwd[first_ed]++;
        else
            mate->mapped_ed_histogram_rev[first_ed]++;

        const uint32 log_first_ed = first_ed ? nvbio::log2(first_ed) + 1 : 0;
        mate->mapped_log_ed_histogram[log_first_ed]++;
    }

    // track edit-distance correlation
    if (first_ed + 1 < 64)
    {
        if (second_ed + 1 < 64)
            mate->mapped_ed_correlation[first_ed + 1][second_ed + 1]++;
        else
            mate->mapped_ed_correlation[first_ed + 1][0]++;
    }
}

namespace { // anonymous

std::string generate_file_name(const char* report, const uint32 id, const char* name)
{
    std::string file_name = report;
    char id_name[2048]; sprintf( id_name, "%u.%s", id, name );
    {
        const size_t offset = file_name.find(".html");
        file_name.replace( offset+1, file_name.length() - offset - 1, id_name );
        file_name.append( ".html" );
    }
    return file_name;
}

std::string device_file_name(const char* report, const uint32 i)
{
    std::string file_name = report;
    char name[32]; sprintf( name, "%u", i );
    {
        const size_t offset = file_name.find(".html");
        file_name.replace( offset+1, file_name.length() - offset - 1, name );
        file_name.append( ".html" );
    }
    return file_name;
}

// return the local file name from a path
//
const char* local_file(const std::string& file_name)
{
  #if WIN32
    const size_t pos = file_name.find_last_of("/\\");
  #else
    const size_t pos = file_name.rfind('/');
  #endif

    if (pos == std::string::npos)
        return file_name.c_str();
    else
        return file_name.c_str() + pos + 1u;
}

void add_param(FILE* html_output, const char* name, const uint32 val, bool alt)
{
    html::tr_object tr( html_output, "class", alt ? "alt" : "none", NULL );
    html::th_object( html_output, html::FORMATTED, NULL, name );
    html::td_object( html_output, html::FORMATTED, NULL, "%u", val );
}
void add_param(FILE* html_output, const char* name, const float val, bool alt)
{
    html::tr_object tr( html_output, "class", alt ? "alt" : "none", NULL );
    html::th_object( html_output, html::FORMATTED, NULL, name );
    html::td_object( html_output, html::FORMATTED, NULL, "%f", val );
}
void add_param(FILE* html_output, const char* name, const char* val, bool alt)
{
    html::tr_object tr( html_output, "class", alt ? "alt" : "none", NULL );
    html::th_object( html_output, html::FORMATTED, NULL, name );
    html::td_object( html_output, html::FORMATTED, NULL, "%s", val );
}

void stats_string(char* buffer, const uint32 px, const char* units, const float v, const float p, const float range)
{
    sprintf(buffer,"<span><statnum style=\"width:%upx;\">%.1f %s</statnum> <statbar style=\"width:%.1f%%%%\">\'</statbar></span>", px, v, units, 2.0f + range * p);
}

} // anonymous namespace

void generate_report_header(const uint32 n_reads, const Params& params, AlignmentStats& aln_stats, const uint32 n_devices, const Stats* device_stats, const char* report)
{
    if (report == NULL)
        return;

    FILE* html_output = fopen( report, "w" );
    if (html_output == NULL)
    {
        log_warning( stderr, "unable to write HTML report \"%s\"\n", report );
        return;
    }

    {
        const uint32 n_mapped = aln_stats.n_mapped;

        html::html_object html( html_output );
        {
            const char* meta_list = "<meta http-equiv=\"refresh\" content=\"2\" />";

            html::header_object hd( html_output, "Bowtie2 Report", html::style(), meta_list );
            {
                html::body_object body( html_output );

                //
                // params
                //
                {
                    html::table_object table( html_output, "params", "params", "parameters" );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "parameter name" );
                        html::th_object( html_output, html::FORMATTED, NULL, "value" );
                    }
                    add_param( html_output, "randomized",  params.randomized ? "yes" : "no", true );
                    add_param( html_output, "N",           params.allow_sub,                 false );
                    add_param( html_output, "seed-len",    params.seed_len,                  true );
                    add_param( html_output, "subseed-len", params.subseed_len,               false );
                    add_param( html_output, "seed-freq",   params.seed_freq.type_symbol(),   true );
                    add_param( html_output, "seed-freq",   params.seed_freq.k,               true );
                    add_param( html_output, "seed-freq",   params.seed_freq.m,               true );
                    add_param( html_output, "max-reseed",  params.max_reseed,                false );
                    add_param( html_output, "rep-seeds",   params.rep_seeds,                 true );
                    add_param( html_output, "max-hits",    params.max_hits,                  false );
                    add_param( html_output, "max-dist",    params.max_dist,                  true );
                    add_param( html_output, "max-effort",  params.max_effort,                false );
                    add_param( html_output, "min-ext",     params.min_ext,                   true );
                    add_param( html_output, "max-ext",     params.max_ext,                   false );
                    add_param( html_output, "mapQ-filter", params.mapq_filter,               true );
                    add_param( html_output, "scoring",     params.scoring_file.c_str(),      false );
                    add_param( html_output, "report",      params.report.c_str(),            true );
                }

                //
                // per-device stats
                //
                {
                    char span_string[1024];

                    html::table_object table( html_output, "device-stats", "stats", "device stats" );

                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                        html::th_object( html_output, html::FORMATTED, NULL, "time" );
                        html::th_object( html_output, html::FORMATTED, NULL, "avg speed" );
                        html::th_object( html_output, html::FORMATTED, NULL, "reads" );
                    }
                    float global_time = 0.0f;
                    for (uint32 i = 0; i < n_devices; ++i)
                        global_time += device_stats[i].global_time;

                    for (uint32 i = 0; i < n_devices; ++i)
                    {
                        char link_name[1024];
                        char device_name[64];
                        sprintf( device_name, "device %u", i );
                        sprintf( link_name, "<a href=\"%s\">%s</a>", local_file( device_file_name( report, i ) ), device_name );
                        const char* cls = "none";
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, link_name );
                        stats_string( span_string, 40, "s", device_stats[i].global_time, device_stats[i].global_time / global_time, 75.0f );
                        html::td_object( html_output, html::FORMATTED, "class", cls, NULL, span_string );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.2f K reads/s", 1.0e-3f * float(device_stats[i].n_reads) / device_stats[i].global_time );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.2f M", float(device_stats[i].n_reads) * 1.0e-6f );
                    }
                }

                //
                // mapping stats
                //
                {
                    html::table_object table( html_output, "mapping-stats", "stats", "mapping stats" );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                        html::th_object( html_output, html::FORMATTED, NULL, "mapped" );
                        html::th_object( html_output, html::FORMATTED, NULL, "ambiguous" );
                        html::th_object( html_output, html::FORMATTED, NULL, "multiple" );
                    }
                    {
                        html::tr_object tr( html_output, "class", "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "reads" );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * float(n_mapped)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * float(aln_stats.n_ambiguous)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * float(aln_stats.n_multiple)/float(n_reads) );
                    }
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "edit distance" );
                        html::th_object( html_output, html::FORMATTED, NULL, "total" );
                        html::th_object( html_output, html::FORMATTED, NULL, "forward" );
                        html::th_object( html_output, html::FORMATTED, NULL, "reverse" );
                    }
                    uint32 best_bin[2]     = {0};
                    uint32 best_bin_val[2] = {0};
                    for (uint32 i = 0; i < aln_stats.mapped_ed_histogram.size(); ++i)
                    {
                        const uint32 v = aln_stats.mapped_ed_histogram[i];

                        if (best_bin_val[0] < v)
                        {
                            best_bin_val[1] = best_bin_val[0];
                            best_bin[1]     = best_bin[0];
                            best_bin_val[0] = v;
                            best_bin[0]     = i;
                        }
                        else if (best_bin_val[1] < v)
                        {
                            best_bin_val[1] = v;
                            best_bin[1]     = i;
                        }
                    }
                    for (uint32 i = 0; i < aln_stats.mapped_ed_histogram.size(); ++i)
                    {
                        const uint32 v = aln_stats.mapped_ed_histogram[i];
                        const uint32 vf = aln_stats.mapped_ed_histogram_fwd[i];
                        const uint32 vr = aln_stats.mapped_ed_histogram_rev[i];

                        if (float(v)/float(n_reads) < 1.0e-3f)
                            continue;

                        html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "%u", i );
                        const char* cls = i == best_bin[0] ? "yellow" : i == best_bin[1] ? "orange" : "none";
                        html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "%.1f %%", 100.0f * float(v)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * float(vf)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * float(vr)/float(n_reads) );
                    }
                }

                //
                // mapping quality stats
                //
                {
                    html::table_object table( html_output, "mapping-quality-stats", "stats", "mapping quality stats" );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "mapQ" );
                        html::th_object( html_output, html::FORMATTED, NULL, "percentage" );
                    }

                    // rebin to a logarithmic scale
                    uint64 bins[7] = {0};
                    for (uint32 i = 0; i < 64; ++i)
                    {
                        const uint32 log_mapq = i ? nvbio::log2(i) + 1 : 0;
                        bins[log_mapq] += aln_stats.mapq_bins[i];
                    }

                    // compute best bins
                    uint32 best_bin[2]     = {0};
                    uint64 best_bin_val[2] = {0};
                    for (uint32 i = 0; i < 7; ++i)
                    {
                        if (best_bin_val[0] < bins[i])
                        {
                            best_bin_val[1] = best_bin_val[0];
                            best_bin[1]     = best_bin[0];
                            best_bin_val[0] = bins[i];
                            best_bin[0]     = i;
                        }
                        else if (best_bin_val[1] < bins[i])
                        {
                            best_bin_val[1] = bins[i];
                            best_bin[1]     = i;
                        }
                    }

                    // output html table
                    for (uint32 i = 0; i < 7; ++i)
                    {
                        const uint32 bin_size = 1u << (i-1);

                        char buffer[1024];
                        if (i <= 1)
                            sprintf( buffer, "%u", i );
                        else if (bin_size < 1024)
                            sprintf( buffer, "%u - %u", bin_size, bin_size*2-1 );

                        html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, buffer );
                        const char* cls = i == best_bin[0] ? "yellow" : i == best_bin[1] ? "orange" : "none";
                        html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "%.1f %%", 100.0f * float(bins[i])/float(n_reads) );
                    }
                }
                //
                // best2-mapping stats
                //
                {
                    // compute best 2 entries among double alignments
                    uint2  best_bin2[2] = { make_uint2(0,0) };
                    uint32 best_bin2_val[2] = { 0 };

                    for (uint32 i = 1; i <= 16; ++i)
                    {
                        for (uint32 j = 1; j <= 16; ++j)
                        {
                            if (best_bin2_val[0] < aln_stats.mapped_ed_correlation[i][j])
                            {
                                best_bin2_val[1] = best_bin2_val[0];
                                best_bin2[1]     = best_bin2[0];
                                best_bin2_val[0] = aln_stats.mapped_ed_correlation[i][j];
                                best_bin2[0]     = make_uint2(i,j);
                            }
                            else if (best_bin2_val[1] < aln_stats.mapped_ed_correlation[i][j])
                            {
                                best_bin2_val[1] = aln_stats.mapped_ed_correlation[i][j];
                                best_bin2[1]     = make_uint2(i,j);
                            }
                        }
                    }

                    // compute best 2 entries among single alignments
                    uint2  best_bin1[2] = { make_uint2(0,0) };
                    uint32 best_bin1_val[2] = { 0 };

                    for (uint32 i = 0; i <= 16; ++i)
                    {
                        if (best_bin1_val[0] < aln_stats.mapped_ed_correlation[i][0])
                        {
                            best_bin1_val[1] = best_bin1_val[0];
                            best_bin1[1]     = best_bin1[0];
                            best_bin1_val[0] = aln_stats.mapped_ed_correlation[i][0];
                            best_bin1[0]     = make_uint2(i,0);
                        }
                        else if (best_bin1_val[1] < aln_stats.mapped_ed_correlation[i][0])
                        {
                            best_bin1_val[1] = aln_stats.mapped_ed_correlation[i][0];
                            best_bin1[1]     = make_uint2(i,0);
                        }
                    }

                    html::table_object table( html_output, "best2-mapping-stats", "stats", "best2 mapping stats" );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                        for (uint32 i = 0; i <= 16; ++i)
                            html::th_object( html_output, html::FORMATTED, NULL, (i == 0 ? "-" : "%u"), i-1 );
                    }
                    for (uint32 i = 0; i <= 16; ++i)
                    {
                        html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, (i == 0 ? "-" : "%u"), i-1 );

                        for (uint32 j = 0; j <= 16; ++j)
                        {
                            const uint32 v = aln_stats.mapped_ed_correlation[i][j];

                            if (100.0f * float(v)/float(n_reads) >= 0.1f)
                            {
                                const char* cls = ((i == best_bin1[0].x && j == best_bin1[0].y) ||
                                                   (i == best_bin2[0].x && j == best_bin2[0].y)) ? "yellow" :
                                                  ((i == best_bin1[1].x && j == best_bin1[1].y) ||
                                                   (i == best_bin2[1].x && j == best_bin2[1].y)) ? "orange" :
                                                  (i   == j) ? "pink" :
                                                  (i+1 == j) ? "azure" : "none";
                                html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "%.1f %%", 100.0f * float(v)/float(n_reads) );
                            }
                            else if (100.0f * float(v)/float(n_reads) >= 0.01f)
                                html::td_object( html_output, html::FORMATTED, "class", "small", NULL, "%.2f %%", 100.0f * float(v)/float(n_reads) );
                            else
                            {
                                const char* cls = (i > params.max_dist+1 || j > params.max_dist+1) ? "gray" : "none";
                                html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "-" );
                            }
                        }
                    }
                }
            }
        }
    }

    fclose( html_output );
}

void generate_device_report(const uint32 id, Stats& stats, AlignmentStats& aln_stats, const char* report)
{
    if (report == NULL)
        return;

    std::vector<KernelStats*> kernel_stats;
    kernel_stats.push_back( &stats.map );
    kernel_stats.push_back( &stats.select );
    kernel_stats.push_back( &stats.sort );
    kernel_stats.push_back( &stats.locate );
    kernel_stats.push_back( &stats.score );
    kernel_stats.push_back( &stats.opposite_score );
    kernel_stats.push_back( &stats.backtrack );
    kernel_stats.push_back( &stats.backtrack_opposite );
    kernel_stats.push_back( &stats.finalize );
    kernel_stats.push_back( &stats.alignments_DtoH );
    kernel_stats.push_back( &stats.read_HtoD );
    kernel_stats.push_back( &stats.read_io );

    //if (stats.params.keep_stats)
    {
        for (uint32 i = 0; i < kernel_stats.size(); ++i)
            generate_kernel_table( id, report, *kernel_stats[i] );
    }

    const std::string device_report = device_file_name( report, id );

    FILE* html_output = fopen( device_report.c_str(), "w" );
    if (html_output == NULL)
    {
        log_warning( stderr, "unable to write HTML report \"%s\"\n", device_report.c_str() );
        return;
    }

    {
        const uint32 n_reads  = stats.n_reads;
        const uint32 n_mapped = aln_stats.n_mapped;

        html::html_object html( html_output );
        {
            const char* meta_list = "<meta http-equiv=\"refresh\" content=\"2\" />";

            html::header_object hd( html_output, "Bowtie2 Report", html::style(), meta_list );
            {
                html::body_object body( html_output );

                //
                // speed stats
                //
                {
                    char span_string[1024];

                    html::table_object table( html_output, "speed-stats", "stats", "speed stats" );
                    float  worst_time[2] = {0};
                    uint32 worst[2]      = {0};

                    for (uint32 i = 0; i < kernel_stats.size(); ++i)
                    {
                        const float time = kernel_stats[i]->time;

                        if (worst_time[0] < time)
                        {
                            worst_time[1] = worst_time[0];
                            worst[1]      = worst[0];
                            worst_time[0] = time;
                            worst[0]      = i;
                        }
                        else if (worst_time[1] < time)
                        {
                            worst_time[1] = time;
                            worst[1]      = i;
                        }
                    }
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                        html::th_object( html_output, html::FORMATTED, NULL, "time" );
                        html::th_object( html_output, html::FORMATTED, NULL, "avg speed" );
                        html::th_object( html_output, html::FORMATTED, NULL, "max speed" );
                    }
                    {
                        html::tr_object tr( html_output, "class", "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "total" );
                        html::td_object( html_output, html::FORMATTED, "class", "red", NULL, "%.1f s", stats.global_time );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f K reads/s", 1.0e-3f * float(n_reads)/stats.global_time );
                        html::td_object( html_output, "-", NULL );
                    }
                    for (uint32 i = 0; i < kernel_stats.size(); ++i)
                    {
                        const KernelStats&     kstats = *kernel_stats[i];
                        const char*            name = kstats.name.c_str();
                        const char*            units = kstats.units.c_str();
                        const std::string file_name = generate_file_name( report, id, name );

                        char link_name[1024];
                        sprintf( link_name, "<a href=\"%s\">%s</a>", local_file( file_name ), name );
                        const char* cls = worst[0] == i ? "yellow" : worst[1] == i ? "orange" : "none";
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, link_name );
                        stats_string( span_string, 40, "s", kstats.time, kstats.time / stats.global_time, 75.0f );
                        html::td_object( html_output, html::FORMATTED, "class", cls, NULL, span_string );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.2f M %s/s", 1.0e-6f * float(kstats.calls)/kstats.time, units );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.2f M %s/s", 1.0e-6f * kstats.max_speed, units );
                    }
                }
                //
                // mapping stats
                //
                {
                    html::table_object table( html_output, "mapping-stats", "stats", "mapping stats" );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                        html::th_object( html_output, html::FORMATTED, NULL, "mapped" );
                        html::th_object( html_output, html::FORMATTED, NULL, "ambiguous" );
                        html::th_object( html_output, html::FORMATTED, NULL, "multiple" );
                    }
                    {
                        html::tr_object tr( html_output, "class", "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "reads" );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * float(n_mapped)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * float(aln_stats.n_ambiguous)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * float(aln_stats.n_multiple)/float(n_reads) );
                    }
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "edit distance" );
                        html::th_object( html_output, html::FORMATTED, NULL, "total" );
                        html::th_object( html_output, html::FORMATTED, NULL, "forward" );
                        html::th_object( html_output, html::FORMATTED, NULL, "reverse" );
                    }
                    uint32 best_bin[2]     = {0};
                    uint32 best_bin_val[2] = {0};
                    for (uint32 i = 0; i < aln_stats.mapped_ed_histogram.size(); ++i)
                    {
                        const uint32 v = aln_stats.mapped_ed_histogram[i];

                        if (best_bin_val[0] < v)
                        {
                            best_bin_val[1] = best_bin_val[0];
                            best_bin[1]     = best_bin[0];
                            best_bin_val[0] = v;
                            best_bin[0]     = i;
                        }
                        else if (best_bin_val[1] < v)
                        {
                            best_bin_val[1] = v;
                            best_bin[1]     = i;
                        }
                    }
                    for (uint32 i = 0; i < aln_stats.mapped_ed_histogram.size(); ++i)
                    {
                        const uint32 v = aln_stats.mapped_ed_histogram[i];
                        const uint32 vf = aln_stats.mapped_ed_histogram_fwd[i];
                        const uint32 vr = aln_stats.mapped_ed_histogram_rev[i];

                        if (float(v)/float(n_reads) < 1.0e-3f)
                            continue;

                        html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "%u", i );
                        const char* cls = i == best_bin[0] ? "yellow" : i == best_bin[1] ? "orange" : "none";
                        html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "%.1f %%", 100.0f * float(v)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * float(vf)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * float(vr)/float(n_reads) );
                    }
                }
                //
                // mapping quality stats
                //
                {
                    html::table_object table( html_output, "mapping-quality-stats", "stats", "mapping quality stats" );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "mapQ" );
                        html::th_object( html_output, html::FORMATTED, NULL, "percentage" );
                    }

                    // rebin to a logarithmic scale
                    uint64 bins[7] = {0};
                    for (uint32 i = 0; i < 64; ++i)
                    {
                        const uint32 log_mapq = i ? nvbio::log2(i) + 1 : 0;
                        bins[log_mapq] += aln_stats.mapq_bins[i];
                    }

                    // compute best bins
                    uint32 best_bin[2]     = {0};
                    uint64 best_bin_val[2] = {0};
                    for (uint32 i = 0; i < 7; ++i)
                    {
                        if (best_bin_val[0] < bins[i])
                        {
                            best_bin_val[1] = best_bin_val[0];
                            best_bin[1]     = best_bin[0];
                            best_bin_val[0] = bins[i];
                            best_bin[0]     = i;
                        }
                        else if (best_bin_val[1] < bins[i])
                        {
                            best_bin_val[1] = bins[i];
                            best_bin[1]     = i;
                        }
                    }

                    // output html table
                    for (uint32 i = 0; i < 7; ++i)
                    {
                        const uint32 bin_size = 1u << (i-1);

                        char buffer[1024];
                        if (i <= 1)
                            sprintf( buffer, "%u", i );
                        else if (bin_size < 1024)
                            sprintf( buffer, "%u - %u", bin_size, bin_size*2-1 );

                        html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, buffer );
                        const char* cls = i == best_bin[0] ? "yellow" : i == best_bin[1] ? "orange" : "none";
                        html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "%.1f %%", 100.0f * float(bins[i])/float(n_reads) );
                    }
                }
                //
                // best2-mapping stats
                //
                {
                    // compute best 2 entries among double alignments
                    uint2  best_bin2[2] = { make_uint2(0,0) };
                    uint32 best_bin2_val[2] = { 0 };

                    for (uint32 i = 1; i <= 16; ++i)
                    {
                        for (uint32 j = 1; j <= 16; ++j)
                        {
                            if (best_bin2_val[0] < aln_stats.mapped_ed_correlation[i][j])
                            {
                                best_bin2_val[1] = best_bin2_val[0];
                                best_bin2[1]     = best_bin2[0];
                                best_bin2_val[0] = aln_stats.mapped_ed_correlation[i][j];
                                best_bin2[0]     = make_uint2(i,j);
                            }
                            else if (best_bin2_val[1] < aln_stats.mapped_ed_correlation[i][j])
                            {
                                best_bin2_val[1] = aln_stats.mapped_ed_correlation[i][j];
                                best_bin2[1]     = make_uint2(i,j);
                            }
                        }
                    }

                    // compute best 2 entries among single alignments
                    uint2  best_bin1[2] = { make_uint2(0,0) };
                    uint32 best_bin1_val[2] = { 0 };

                    for (uint32 i = 0; i <= 16; ++i)
                    {
                        if (best_bin1_val[0] < aln_stats.mapped_ed_correlation[i][0])
                        {
                            best_bin1_val[1] = best_bin1_val[0];
                            best_bin1[1]     = best_bin1[0];
                            best_bin1_val[0] = aln_stats.mapped_ed_correlation[i][0];
                            best_bin1[0]     = make_uint2(i,0);
                        }
                        else if (best_bin1_val[1] < aln_stats.mapped_ed_correlation[i][0])
                        {
                            best_bin1_val[1] = aln_stats.mapped_ed_correlation[i][0];
                            best_bin1[1]     = make_uint2(i,0);
                        }
                    }

                    html::table_object table( html_output, "best2-mapping-stats", "stats", "best2 mapping stats" );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                        for (uint32 i = 0; i <= 16; ++i)
                            html::th_object( html_output, html::FORMATTED, NULL, (i == 0 ? "-" : "%u"), i-1 );
                    }
                    for (uint32 i = 0; i <= 16; ++i)
                    {
                        html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, (i == 0 ? "-" : "%u"), i-1 );

                        for (uint32 j = 0; j <= 16; ++j)
                        {
                            const uint32 v = aln_stats.mapped_ed_correlation[i][j];

                            if (100.0f * float(v)/float(n_reads) >= 0.1f)
                            {
                                const char* cls = ((i == best_bin1[0].x && j == best_bin1[0].y) ||
                                                   (i == best_bin2[0].x && j == best_bin2[0].y)) ? "yellow" :
                                                  ((i == best_bin1[1].x && j == best_bin1[1].y) ||
                                                   (i == best_bin2[1].x && j == best_bin2[1].y)) ? "orange" :
                                                  (i   == j) ? "pink" :
                                                  (i+1 == j) ? "azure" : "none";
                                html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "%.1f %%", 100.0f * float(v)/float(n_reads) );
                            }
                            else if (100.0f * float(v)/float(n_reads) >= 0.01f)
                                html::td_object( html_output, html::FORMATTED, "class", "small", NULL, "%.2f %%", 100.0f * float(v)/float(n_reads) );
                            else
                            {
                                const char* cls = (i > stats.params.max_dist+1 || j > stats.params.max_dist+1) ? "gray" : "none";
                                html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "-" );
                            }
                        }
                    }
                }
                //
                // seeding stats
                //
                if (stats.params.keep_stats)
                {
                    // copy stats locally
                    uint64 hits_total     = stats.hits_total;
                    uint64 hits_ranges    = stats.hits_ranges;
                    uint32 hits_max       = stats.hits_max;
                    uint32 hits_max_range = stats.hits_max_range;
                    uint64 hits_top_total = stats.hits_top_total;
                    uint32 hits_top_max   = stats.hits_top_max;
                    uint64 hits_bins[28];
                    uint64 hits_bins_sum = 0;
                    uint64 hits_top_bins[28];
                    uint64 hits_top_bins_sum = 0;
                    for (uint32 i = 0; i < 28; ++i)
                    {
                         hits_bins_sum     += hits_bins[i]     = stats.hits_bins[i];
                         hits_top_bins_sum += hits_top_bins[i] = stats.hits_top_bins[i];
                    }

                    html::table_object table( html_output, "seeding-stats", "stats", "seeding stats" );
                    char buffer[1024];
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                        html::th_object( html_output, html::FORMATTED, NULL, "seed hits" );
                        html::th_object( html_output, html::FORMATTED, NULL, "top-seed hits" );
                        html::th_object( html_output, html::FORMATTED, NULL, "seed ranges" );
                        html::th_object( html_output, html::FORMATTED, NULL, "range size" );
                    }
                    {
                        html::tr_object tr( html_output, "class", "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "avg" );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f",  float(hits_total)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f",  float(hits_top_total)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f",  float(hits_ranges)/float(n_reads) );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f",  float(hits_total)/float(hits_ranges) );
                    }
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "max" );
                        html::td_object( html_output, html::FORMATTED, NULL, "%u", hits_max );
                        html::td_object( html_output, html::FORMATTED, NULL, "%u", hits_top_max );
                        html::td_object( html_output, html::FORMATTED, NULL, "" );
                        html::td_object( html_output, html::FORMATTED, NULL, "%u", hits_max_range );
                    }
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "# hits" );
                        html::th_object( html_output, html::FORMATTED, NULL, "%% of seeds" );
                        html::th_object( html_output, html::FORMATTED, NULL, "%% of top-seeds" );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                        html::th_object( html_output, html::FORMATTED, NULL, "" );
                    }
                    uint32 max_bin             = 0;
                    uint32 best_bin[2]         = {0};
                    uint64 best_bin_val[2]     = {0};
                    uint32 best_top_bin[2]     = {0};
                    uint64 best_top_bin_val[2] = {0};
                    for (uint32 i = 0; i < 28; ++i)
                    {
                        max_bin = float(hits_bins[i]) / float(hits_bins_sum) > 0.001f ? i : max_bin;
                        max_bin = float(hits_top_bins[i]) / float(hits_top_bins_sum) > 0.001f ? i : max_bin;

                        if (best_bin_val[0] < hits_bins[i])
                        {
                            best_bin_val[1] = best_bin_val[0];
                            best_bin[1]     = best_bin[0];
                            best_bin_val[0] = hits_bins[i];
                            best_bin[0]     = i;
                        }
                        else if (best_bin_val[1] < hits_bins[i])
                        {
                            best_bin_val[1] = hits_bins[i];
                            best_bin[1]     = i;
                        }

                        if (best_top_bin_val[0] < hits_top_bins[i])
                        {
                            best_top_bin_val[1] = best_top_bin_val[0];
                            best_top_bin[1]     = best_top_bin[0];
                            best_top_bin_val[0] = hits_top_bins[i];
                            best_top_bin[0]     = i;
                        }
                        else if (best_top_bin_val[1] < hits_top_bins[i])
                        {
                            best_top_bin_val[1] = hits_top_bins[i];
                            best_top_bin[1]     = i;
                        }
                    }
                    for (uint32 i = 0; i < max_bin; ++i)
                    {
                        const uint32 bin_size = 1u << (i-1);

                        html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
                        if (i <= 1)
                            sprintf( buffer, "%u", i );
                        else if (bin_size < 512)
                            sprintf( buffer, "%u - %u", bin_size, bin_size*2-1 );
                        else if (bin_size == 512)
                            sprintf( buffer, "0.5K - 1K" );
                        else
                            sprintf( buffer, "%uK - %uK", bin_size/1024, bin_size*2/1024 );

                        const char* cls     = i == best_bin[0]     ? "yellow" : i == best_bin[1]     ? "orange" : "none";
                        const char* cls_top = i == best_top_bin[0] ? "yellow" : i == best_top_bin[1] ? "orange" : "none";

                        html::th_object( html_output, html::FORMATTED, NULL, buffer );
                        html::td_object( html_output, html::FORMATTED, "class", cls,     NULL, "%4.1f %%", 100.0f * float(hits_bins[i]) / float(hits_bins_sum) );
                        html::td_object( html_output, html::FORMATTED, "class", cls_top, NULL, "%4.1f %%", 100.0f * float(hits_top_bins[i]) / float(hits_top_bins_sum) );
                        html::td_object( html_output, html::FORMATTED, NULL, "" );
                        html::td_object( html_output, html::FORMATTED, NULL, "" );
                    }
                }
            }
        }
    }
    fclose( html_output );
}

template <typename T>
void find_gt2(const uint32 n, const T* table, uint32 best_bin[2])
{
    best_bin[0] = best_bin[1] = 0;
    T best_bin_val[2] = {0};
    for (uint32 i = 0; i < n; ++i)
    {
        if (best_bin_val[0] < table[i])
        {
            best_bin_val[1] = best_bin_val[0];
            best_bin[1]     = best_bin[0];
            best_bin_val[0] = table[i];
            best_bin[0]     = i;
        }
        else if (best_bin_val[1] < table[i])
        {
            best_bin_val[1] = table[i];
            best_bin[1]     = i;
        }
    }
}

void generate_kernel_table(const uint32 id, const char* report, const KernelStats& stats)
{
    const char* name            = stats.name.c_str();
    const char* units           = stats.units.c_str();
    const std::string file_name = generate_file_name( report, id, name );

    const std::deque< std::pair<uint32,float> >& table = stats.info;

    FILE* html_output = fopen( file_name.c_str(), "w" );
    if (html_output == NULL)
    {
        log_warning( stderr, "unable to write HTML report \"%s\"\n", file_name.c_str() );
        return;
    }

    {
        html::html_object html( html_output );
        {
            const char* meta_list = "<meta http-equiv=\"refresh\" content=\"2\" />";

            html::header_object hd( html_output, "Bowtie2 Report", html::style(), meta_list );
            {
                html::body_object body( html_output );

                //
                // kernel summary stats
                //
                {
                    uint32 bin_calls[32]    = {0};
                    float  bin_sum_time[32] = {0.0f};
                    float  bin_avg_time[32] = {0.0f};
                    float  bin_speed[32]    = {0.0f};

                    float total_time = stats.time;
                    float avg_speed  = total_time ? float(double(stats.calls) / double(stats.time)) : 0.0f;
                    float max_speed  = 0.0f;

                    for (uint32 bin = 0; bin < 32; ++bin)
                    {
                        bin_calls[bin]    = stats.bin_calls[bin];
                        bin_sum_time[bin] = stats.bin_time[bin];
                        bin_avg_time[bin] = stats.bin_calls[bin] ? stats.bin_time[bin] / stats.bin_calls[bin] : 0.0f;
                        bin_speed[bin]    = stats.bin_time[bin]  ? float(double(stats.bin_items[bin]) / double(stats.bin_time[bin])) : 0.0f;
                        //bin_speed[bin]    = stats.bin_calls[bin] ? stats.bin_speed[bin] / stats.bin_calls[bin] : 0.0f;

                        max_speed = std::max( max_speed, bin_speed[bin] );
                    }

                    char buffer1[1024];
                    char buffer2[1024];
                    sprintf( buffer1, "%s-summary-stats", name );
                    sprintf( buffer2, "%s summary stats", name );
                    html::table_object tab( html_output, buffer1, "stats", buffer2 );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "items" );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.2f M", float(stats.calls) * 1.0e-6f );
                    }
                    // write "user" stats
                    for (uint32 i = 0; i < 32; ++i)
                    {
                        if (stats.user_names[i] == NULL)
                            break;

                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, stats.user_names[i] );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.3f %s",
                            stats.user_avg[i] ? stats.user[i]/float(stats.num) :
                                                stats.user[i],
                            stats.user_units[i] );
                    }
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "batch size (%s)", units );
                        html::th_object( html_output, html::FORMATTED, NULL, "calls" );
                        html::th_object( html_output, html::FORMATTED, NULL, "avg time" );
                        html::th_object( html_output, html::FORMATTED, NULL, "sum time" );
                        html::th_object( html_output, html::FORMATTED, NULL, "cumul. time" );
                        html::th_object( html_output, html::FORMATTED, NULL, "speed" );
                    }

                    uint32 best_avg_bin[2];
                    uint32 best_sum_bin[2];
                    find_gt2( 32, bin_avg_time, best_avg_bin );
                    find_gt2( 32, bin_sum_time, best_sum_bin );

                    float cum_time = 0.0f;

                    float max_avg_time = 0.0f;
                    float max_sum_time = 0.0f;
                    for (uint32 i = 0; i < 32; ++i)
                    {
                        max_avg_time = nvbio::max( bin_avg_time[i], max_avg_time );
                        max_sum_time = nvbio::max( bin_sum_time[i], max_sum_time );
                    }

                    char span_string[1024];
                    for (uint32 i = 0; i < 32; ++i)
                    {
                        if (bin_calls[i] == 0)
                            continue;

                        const float speed = bin_speed[i];

                        cum_time += bin_sum_time[i];

                        const uint32 bin_size = 1u << i;
                        html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
                        if (bin_size == 1)
                            html::th_object( html_output, html::FORMATTED, NULL, "1" );
                        else if (bin_size < 512)
                            html::th_object( html_output, html::FORMATTED, NULL, "%u - %u", bin_size, bin_size*2-1 );
                        else if (bin_size == 512) 
                            html::th_object( html_output, html::FORMATTED, NULL, "512 - 1K" );
                        else if (bin_size < 512*1024)
                            html::th_object( html_output, html::FORMATTED, NULL, "%uK- %uK", bin_size/1024, (bin_size*2)/1024 );
                        else if (bin_size == 512*1024)
                            html::th_object( html_output, html::FORMATTED, NULL, "512K - 1M" );

                        const char* avg_cls = i == best_avg_bin[0] ? "yellow" : i == best_avg_bin[1] ? "orange" : "none";
                        const char* sum_cls = i == best_sum_bin[0] ? "yellow" : i == best_sum_bin[1] ? "orange" : "none";
                        const char* spd_cls = speed == max_speed ? "yellow" : speed < avg_speed * 0.1f ? "red" : speed < max_speed * 0.1f ? "pink" : "none";

                        html::td_object( html_output, html::FORMATTED, NULL, "%u", bin_calls[i] );
                        stats_string( span_string, 60, "ms", 1000.0f * bin_avg_time[i], bin_avg_time[i] / max_avg_time, 50.0f );
                        html::td_object( html_output, html::FORMATTED, "class", avg_cls, NULL, span_string );
                        stats_string( span_string, 60, "ms", 1000.0f * bin_sum_time[i], bin_sum_time[i] / max_sum_time, 50.0f );
                        html::td_object( html_output, html::FORMATTED, "class", sum_cls, NULL, span_string );
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %%", 100.0f * cum_time / total_time );
                        html::td_object( html_output, html::FORMATTED, "class", spd_cls, NULL, "%.1f %c %s/s", speed * (speed >= 1.0e6f ?  1.0e-6f : 1.0e-3f), speed >= 1.0e6f ? 'M' : 'K', units );
                    }
                }
                //
                // kernel table stats
                //
                {
                    char buffer1[1024];
                    char buffer2[1024];
                    sprintf( buffer1, "%s-stats", name );
                    sprintf( buffer2, "%s stats", name );
                    html::table_object tab( html_output, buffer1, "stats", buffer2 );
                    {
                        html::tr_object tr( html_output, NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "launch" );
                        html::th_object( html_output, html::FORMATTED, NULL, "batch size (%s)", units );
                        html::th_object( html_output, html::FORMATTED, NULL, "time" );
                        html::th_object( html_output, html::FORMATTED, NULL, "speed" );
                    }
                    uint32 best_bin[2]     = {0};
                    float  best_bin_val[2] = {0};
                    for (uint32 i = 0; i < table.size(); ++i)
                    {
                        if (best_bin_val[0] < table[i].second)
                        {
                            best_bin_val[1] = best_bin_val[0];
                            best_bin[1]     = best_bin[0];
                            best_bin_val[0] = table[i].second;
                            best_bin[0]     = i;
                        }
                        else if (best_bin_val[1] < table[i].second)
                        {
                            best_bin_val[1] = table[i].second;
                            best_bin[1]     = i;
                        }
                    }

                    float max_time  = 0.0f;
                    float max_speed = 0.0f;
                    for (uint32 i = 0; i < table.size(); ++i)
                    {
                        const float speed = float(table[i].first) / table[i].second;
                        max_time = nvbio::max( float(table[i].second), max_time );
                        max_speed = nvbio::max( speed, max_speed );
                    }

                    char span_string[1024];
                    char units_string[1024];
                    for (uint32 i = 0; i < table.size(); ++i)
                    {
                        const float speed = float(table[i].first) / table[i].second;
                        html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
                        html::th_object( html_output, html::FORMATTED, NULL, "%u", i );
                        const char* cls = i == best_bin[0] ? "yellow" : i == best_bin[1] ? "orange" : "none";
                        html::td_object( html_output, html::FORMATTED, NULL, "%.1f %c", float(table[i].first) * (table[i].first > 1000000 ?  1.0e-6f : 1.0e-3f), table[i].first > 1000000 ? 'M' : 'K' );
                        stats_string( span_string, 50, "ms", 1000.0f * float(table[i].second), float(table[i].second) / max_time, 50.0f );
                        html::td_object( html_output, html::FORMATTED, "class", cls, NULL, span_string );
                        sprintf(units_string, "%c %s/s", speed > 1000000 ? 'M' : 'K', units );
                        stats_string( span_string, 100, units_string, speed * (speed >= 1.0e6f ? 1.0e-6f : 1.0e-3f), speed / max_speed, 50.0f );
                        html::td_object( html_output, html::FORMATTED, NULL, span_string );
                    }
                }
            }
        }
    }
    fclose( html_output );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
