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

#include <nvbio-aln-diff/utils.h>
#include <nvbio-aln-diff/alignment.h>
#include <nvbio/basic/html.h>
#include <string>
#include <vector>

namespace nvbio {
namespace alndiff {

inline
std::string generate_file_name(const char* report, const char* name)
{
    std::string file_name = report;
    {
        const size_t offset = file_name.find(".html");
        file_name.replace( offset+1, file_name.length() - offset - 1, name );
        file_name.append( ".html" );
    }
    return file_name;
}
inline 
void add_param(FILE* html_output, const char* name, const uint32 val, bool alt)
{
    html::tr_object tr( html_output, "class", alt ? "alt" : "none", NULL );
    html::th_object( html_output, html::FORMATTED, NULL, name );
    html::td_object( html_output, html::FORMATTED, NULL, "%u", val );
}
inline 
void add_param(FILE* html_output, const char* name, const float val, bool alt)
{
    html::tr_object tr( html_output, "class", alt ? "alt" : "none", NULL );
    html::th_object( html_output, html::FORMATTED, NULL, name );
    html::td_object( html_output, html::FORMATTED, NULL, "%f", val );
}
inline 
void add_param(FILE* html_output, const char* name, const char* val, bool alt)
{
    html::tr_object tr( html_output, "class", alt ? "alt" : "none", NULL );
    html::th_object( html_output, html::FORMATTED, NULL, name );
    html::td_object( html_output, html::FORMATTED, NULL, "%s", val );
}

// find greatest two values in a table
//
template <typename T>
void find_gt2(const int32 first_row, const int32 last_row, const T& table, int32 best_bin[2])
{
    best_bin[0] = 0;
    best_bin[1] = 1;
    uint32 best_bin_val[2] = {0};
    for (int32 i = first_row; i <= last_row; ++i)
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

// compute first row with a non-zero display value
//
template <uint32 XX>
int32 first_display_row(const Histogram<XX>& table, const uint32 total, const int32 last_row = 0)
{
    const int32 X = XX;

    int32 first_row = last_row;
    for (int32 i = last_row; i > -X; --i)
    {
        if (100.0f * float(table[i])/float(total) > 0.01f)
            first_row = i;
    }
    return first_row;
}

// compute last row with a non-zero display value
//
template <uint32 XX>
int32 last_display_row(const Histogram<XX>& table, const uint32 total, const int32 first_row = 0)
{
    const int32 X = XX;

    int32 last_row = first_row;
    for (int32 i = first_row; i < X; ++i)
    {
        if (100.0f * float(table[i])/float(total) > 0.01f)
            last_row = i;
    }
    return last_row;
}

// compute first and last rows with a non-zero display value
//
template <uint32 XX>
int2 display_range(const Histogram<XX>& table, const uint32 total)
{
    return make_int2(
        first_display_row( table, total ),
        last_display_row( table, total ) );
}

inline 
void stats_string(char* buffer, const float v, const float range)
{
    sprintf(buffer,"<span><statnum>%5.2f %%%%</statnum> <statbar style=\"width:%.1f%%%%\">\'</statbar></span>", v, 2.0f + range * v/100.0f);
}

template <uint32 XX>
void generate_diff_table(
    FILE*                  html_output,
    const char*            stat_by,
    const char*            name,
    const Bins             bin_type,
    const Histogram<XX>&   stats_l,
    const Histogram<XX>&   stats_r,
    const uint32           total,
    const uint32           n_mapped_both)
{
    const int32 X = XX;

    //
    // kernel summary stats
    //
    {
        char buffer1[1024];
        char buffer2[1024];
        sprintf( buffer1, "%s-summary-stats", name );
        sprintf( buffer2, "%s [%s] summary stats", name, stat_by );
        html::table_object tab( html_output, buffer1, "stats", buffer2 );
        {
            html::tr_object tr(html_output, NULL);
            html::th_object(html_output, html::FORMATTED, NULL, "file names");
            html::td_object(html_output, html::FORMATTED, "colspan", "2", "align", "center", NULL, "[L]"/*, aln_file_name1*/);
            html::td_object(html_output, html::FORMATTED, "colspan", "2", "align", "center", NULL, "[R]"/*, aln_file_name2*/);
        }
        {
            html::tr_object tr( html_output, NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "" );
            html::th_object( html_output, html::FORMATTED, NULL, "better/equal" );
            html::th_object( html_output, html::FORMATTED, NULL, "equal" );
            html::th_object( html_output, html::FORMATTED, NULL, "better" );
            html::th_object( html_output, html::FORMATTED, NULL, "worse" );
        }
        {
            html::tr_object tr( html_output, NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "items" );
            html::td_object( html_output, html::FORMATTED, NULL, "%.2f M", float(stats_l.count) * 1.0e-6f );
            html::td_object( html_output, html::FORMATTED, NULL, "%.2f M", float(stats_l[0]) * 1.0e-6f );
            html::td_object( html_output, html::FORMATTED, "class", "yellow", NULL, "%.2f M", float(stats_l.all_but(0)) * 1.0e-6f );
            html::td_object( html_output, html::FORMATTED, "class", "pink", NULL, "%.2f M", float(stats_r.all_but(0)) * 1.0e-6f );
        }
        {
            html::tr_object tr( html_output, "class", "alt", NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "%% of total" );
            html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%", 100.0f * float(stats_l.count)/float(total) );
            html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%", 100.0f * float(stats_l[0])/float(total) );
            html::td_object( html_output, html::FORMATTED, "class", "yellow", NULL, "%5.2f %%", 100.0f * float(stats_l.all_but(0))/float(total) );
            html::td_object( html_output, html::FORMATTED, "class", "pink", NULL, "%5.2f %%", 100.0f * float(stats_r.all_but(0))/float(total) );
        }
        {
            html::tr_object tr( html_output, NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "%% of %s", stat_by );
            html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%", 100.0f * float(stats_l.count)/float(n_mapped_both) );
            html::td_object( html_output, html::FORMATTED, NULL, "%5.2f %%", 100.0f * float(stats_l[0])/float(n_mapped_both) );
            html::td_object( html_output, html::FORMATTED, "class", "yellow", NULL, "%5.2f %%", 100.0f * float(stats_l.all_but(0))/float(n_mapped_both) );
            html::td_object( html_output, html::FORMATTED, "class", "pink", NULL, "%5.2f %%", 100.0f * float(stats_r.all_but(0))/float(n_mapped_both) );
        }
    }

    //
    // table stats
    //
    {
        char buffer1[1024];
        char buffer2[1024];
        sprintf( buffer1, "%s-stats", name );
        sprintf( buffer2, "%s [%s] stats", name, stat_by );
        html::table_object tab( html_output, buffer1, "stats", buffer2 );
        {
            html::tr_object tr( html_output, NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "bin" );
            html::th_object( html_output, html::FORMATTED, NULL, "%% of total" );
            html::th_object( html_output, html::FORMATTED, NULL, "%% of %s", stat_by );
            html::th_object( html_output, html::FORMATTED, NULL, "%% of diff" );
            html::th_object( html_output, html::FORMATTED, NULL, "cumulative (+)" );
            html::th_object( html_output, html::FORMATTED, NULL, "cumulative (-)" );
        }

        const Histogram<X>& table     = stats_l;
        const Histogram<X>& table_neg = stats_r;

        int32 best_bin[2] = {0};
        find_gt2( 0, X-1, table, best_bin );

        const int32 last_row     = last_display_row( stats_l, n_mapped_both );
        const int32 last_row_neg = last_display_row( stats_r, n_mapped_both );

        // compute cumulative values, from best to worst
        float cum_pos[X];
        float cum_neg[X];
        cum_pos[X-1] = 100.0f * float(table[X-1])/float(n_mapped_both);
        for (int32 i = X-2; i >= 0; --i)
            cum_pos[i] = 100.0f * float(table[i])/float(n_mapped_both) + cum_pos[i+1];
        cum_neg[0] = cum_pos[0];
        for (int32 i = 1; i < X; ++i)
            cum_neg[i] = 100.0f * float(table_neg[i])/float(n_mapped_both) + cum_neg[i-1];

        const float max_perc = 48.0f;

        char span_string[1024];
        for (int32 i = last_row_neg; i > 0; --i)
        {
            html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "%d", -int32(bin_type == LOG ? log_bin_range(i) : i) );
            stats_string( span_string, 100.0f * float(table_neg[i]) / float(total), max_perc );
            html::td_object( html_output, html::FORMATTED, "class", "pink", NULL,  span_string );
            stats_string( span_string, 100.0f * float(table_neg[i]) / float(n_mapped_both), max_perc );
            html::td_object( html_output, html::FORMATTED, "class", "pink", NULL,  span_string );
            stats_string( span_string, stats_r.count ? 100.0f * float(table_neg[i]) / float(stats_r.count) : 0.0f, max_perc );
            html::td_object( html_output, html::FORMATTED, "class", "pink", NULL,  span_string );
            stats_string( span_string, cum_neg[i], max_perc );
            html::td_object( html_output, html::FORMATTED, "class", "pink", NULL,  span_string );
            stats_string( span_string, 100.0f - cum_neg[i] + 100.0f * float(table_neg[i])/float(n_mapped_both), max_perc );
            html::td_object( html_output, html::FORMATTED, "class", "pink", NULL,  span_string );
        }
        for (int32 i = 0; i <= last_row; ++i)
        {
            html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "%d", (bin_type == LOG ? log_bin_range(i) : i) );
            const char* cls = i == best_bin[0] ? "yellow" : i == best_bin[1] ? "orange" : "none";
            stats_string( span_string,  100.0f * float(table_neg[i]) / float(total), max_perc );
            html::td_object( html_output, html::FORMATTED, "class", cls, NULL,  span_string );
            stats_string( span_string,  100.0f * float(table[i]) / float(n_mapped_both), max_perc );
            html::td_object( html_output, html::FORMATTED, "class", cls, NULL,  span_string );
            stats_string( span_string, stats_l.count ? 100.0f * float(table[i]) / float(stats_l.count) : 0.0f, max_perc );
            html::td_object( html_output, html::FORMATTED, "class", cls, NULL,  span_string );
            stats_string( span_string, cum_pos[i], max_perc );
            html::td_object( html_output, html::FORMATTED, "class", cls, NULL,  span_string );
            stats_string( span_string, 100.0f - cum_pos[i] + 100.0f * float(table[i])/float(n_mapped_both), max_perc );
            html::td_object( html_output, html::FORMATTED, "class", cls, NULL,  span_string );
        }
    }
}

template <uint32 XX>
void generate_table(
    FILE*                   html_output,
    const char*             stat_by,
    const char*             name,
    const Bins              bin_type,
    const Histogram<XX>&    stats,
    const uint32            total)
{
    //
    // table stats
    //

    const int32 X = XX;

    const Histogram<X>& table = stats;

    // find range, negative values
    const int2 row_range = display_range( stats, total );

    // compute best 2 entries
    int32 best_bin[2] = { 0, 1 };

    find_gt2( row_range.x, row_range.y, stats, best_bin );

    // write HTML table
    {
        char buffer1[1024];
        char buffer2[1024];
        sprintf( buffer1, "%s-stats", name );
        sprintf( buffer2, "%s [%s] stats", name, stat_by );
        html::table_object tab( html_output, buffer1, "stats", buffer2 );
        {
            html::tr_object tr( html_output, NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "bin" );
            html::th_object( html_output, html::FORMATTED, NULL, "%%" );
        }

        char span_string[1024];
        const float max_perc = 70.0f;
        for (int32 i = row_range.x; i <= row_range.y; ++i)
        {
            html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );

            const int32 c = bin_type == LOG ? log_bin_range(i >= 0 ? i : -i) : i >= 0 ? i : -i;

            const char* cls = (i == best_bin[0]) ? "yellow" :
                              (i == best_bin[1]) ? "orange" :
                              "none";

            html::th_object( html_output, html::FORMATTED, NULL, "%d", i >= 0 ? c : -c  );
            stats_string( span_string, total ? 100.0f * float(table[i]) / float(total) : 0.0f, max_perc );
            html::td_object( html_output, html::FORMATTED, "class", cls, NULL,  span_string );
        }
    }
}

template <uint32 XX>
void generate_table(
    FILE*                   html_output,
    const char*             stat_by,
    const char*             name,
    const Bins              bin_type,
    const Histogram<XX>&    stats_l,
    const Histogram<XX>&    stats_r,
    const uint32            l_count,
    const uint32            r_count)
{
    //
    // table stats
    //

    const int2 row_range_l = display_range( stats_l, l_count );
    const int2 row_range_r = display_range( stats_r, r_count );
    const int2 row_range = make_int2( nvbio::min( row_range_l.x, row_range_r.x ),
                                      nvbio::max( row_range_l.y, row_range_r.y ) );

    // compute best 2 entries
    int32 best_bin[2]  = { 0, 1 };
    int32 worst_bin[2] = { 0, 1 };

    find_gt2( row_range.x, row_range.y, stats_l, best_bin );
    find_gt2( row_range.x, row_range.y, stats_r, worst_bin );

    // write table
    {
        char buffer1[1024];
        char buffer2[1024];
        sprintf( buffer1, "%s-stats", name );
        sprintf( buffer2, "%s [%s] stats", name, stat_by );
        html::table_object tab( html_output, buffer1, "stats", buffer2 );
        {
            html::tr_object tr( html_output, NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "bin" );
            html::th_object( html_output, html::FORMATTED, NULL, "[L]" );
            html::th_object( html_output, html::FORMATTED, NULL, "[R]" );
        }

        char span_string[1024];
        const float max_perc = 70.0f;
        for (int32 i = row_range.x; i <= row_range.y; ++i)
        {
            html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );

            const int32 c = bin_type == LOG ? log_bin_range(i >= 0 ? i : -i) : i >= 0 ? i : -i;

            const char* cls_l = (i == best_bin[0]) ? "yellow" :
                                (i == best_bin[1]) ? "orange" :
                                "none";
            const char* cls_r = (i == worst_bin[0]) ? "red" :
                                (i == worst_bin[1]) ? "orange" :
                                "pink";

            html::th_object( html_output, html::FORMATTED, NULL, "%d", i >= 0 ? c : -c );
            stats_string( span_string, l_count ? 100.0f * float(stats_l[i])     / float(l_count) : 0.0f, max_perc );
            html::td_object( html_output, html::FORMATTED, "class", cls_l, NULL,  span_string );
            stats_string( span_string, r_count ? 100.0f * float(stats_r[i]) / float(r_count) : 0.0f, max_perc );
            html::td_object( html_output, html::FORMATTED, "class", cls_r, NULL,  span_string );
        }
    }
}

template <uint32 XX, uint32 YY>
void generate_table2d(
    FILE*                           html_output,
    const char*                     stat_by,
    const char*                     name,
    const Histogram2d<XX,YY>&       stats_l,
    const Histogram2d<XX,YY>&       stats_r,
    const char*                     bin_name,
    const std::vector<std::string>& X_bins,
    const std::vector<std::string>& Y_bins,
    const uint32                    total,
    const uint32                    n_mapped_both,
    const bool                      diff_table)
{
    const int32 X = int32(XX);
    const int32 Y = int32(YY);

    //
    // table stats
    //
    {
        char buffer1[1024];
        char buffer2[1024];
        sprintf( buffer1, "%s-stats-%s", name, bin_name );
        sprintf( buffer2, "%s [%s] stats by %s", name, stat_by, bin_name );

        // determine the first and last columns
        int32 first_col = 0;
        int32 last_col  = 0;
        if (diff_table)
        {
            first_col = 0;
            last_col  = Y-1;
        }
        else
        {
            for (int32 i = -X+1; i < X; ++i)
            {
                for (int32 j = 0; j > -Y; --j)
                {
                    if (stats_l(i,j) > 0 ||
                        stats_r(i,j) > 0)
                        first_col = j;
                }
                for (int32 j = 0; j < Y; ++j)
                {
                    if (stats_l(i,j) > 0 ||
                        stats_r(i,j) > 0)
                        last_col = j;
                }
            }
        }

        // determine the first and last row
        int32 first_row = 0;
        for (int32 i = 0; i > -X; --i)
        {
            for (int32 j = last_col; j >= first_col; --j)
            {
                if (stats_l(i,j) > 0 ||
                    stats_r(i,j) > 0)
                    first_row = i;
            }
        }
        int32 last_row = 0;
        for (int32 i = 0; i < X; ++i)
        {
            for (int32 j = last_col; j >= first_col; --j)
            {
                if (stats_l(i,j) > 0 ||
                    stats_r(i,j) > 0)
                    last_row = i;
            }
        }

        // compute best 2 entries
         int2  best_bin[2] = { make_int2(0,0) };
        uint32 best_bin_val[2] = { 0 };

        for (int32 i = first_row; i <= last_row; ++i)
        {
            for (int32 j = first_col; j <= last_col; ++j)
            {
                if (best_bin_val[0] < stats_l(i,j))
                {
                    best_bin_val[1] = best_bin_val[0];
                    best_bin[1]     = best_bin[0];
                    best_bin_val[0] = stats_l(i,j);
                    best_bin[0]     = make_int2(i,j);
                }
                else if (best_bin_val[1] < stats_l(i,j))
                {
                    best_bin_val[1] = stats_l(i,j);
                    best_bin[1]     = make_int2(i,j);
                }
            }
        }

        // compute worst 2 entries
         int2  worst_bin[2] = { make_int2(0,0) };
        uint32 worst_bin_val[2] = { 0 };

        for (int32 i = first_row; i <= last_row; ++i)
        {
            for (int32 j = (diff_table ? 1 : first_col); j <= last_col; ++j)
            {
                if (worst_bin_val[0] < stats_r(i,j))
                {
                    worst_bin_val[1] = worst_bin_val[0];
                    worst_bin[1]     = worst_bin[0];
                    worst_bin_val[0] = stats_r(i,j);
                    worst_bin[0]     = make_int2(i,j);
                }
                else if (worst_bin_val[1] < stats_l(i,j))
                {
                    worst_bin_val[1] = stats_r(i,j);
                    worst_bin[1]     = make_int2(i,j);
                }
            }
        }

        html::table_object table( html_output, buffer1, "stats", buffer2 );
        {
            html::tr_object tr( html_output, NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "" );
            for (int32 j = last_col; j >= (diff_table ? 1 : first_col); --j)
                html::th_object( html_output, html::FORMATTED, NULL, "%s%s", j >= 0 ? "" : "-", Y_bins[j >= 0 ? j : -j].c_str() );

            for (int32 j = first_col; j <= last_col; ++j)
                html::th_object( html_output, html::FORMATTED, NULL, "%s%s", j >= 0 ? "" : "-", Y_bins[j >= 0 ? j : -j].c_str() );

            html::th_object( html_output, html::FORMATTED, NULL, "%s", name );
        }

        for (int32 i = first_row; i <= last_row; ++i)
        {
            html::tr_object tr( html_output, "class", i % 2 ? "none" : "alt", NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "%s%s", i >= 0 ? "" : "-", X_bins[i >= 0 ? i : -i].c_str() );

            for (int32 j = last_col; j >= (diff_table ? 1 : first_col); --j)
            {
                const float sval = 100.0f * float(stats_r(i,j))/float(n_mapped_both);
                if (sval >= 0.1f)
                {
                    const char* cls = (i == worst_bin[0].x && j == worst_bin[0].y) ? "red" :
                                      (i == worst_bin[1].x && j == worst_bin[1].y) ? "orange" :
                                      "pink";
                    html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "%.1f %%", sval );
                }
                else if (sval >= 0.01f)
                    html::td_object( html_output, html::FORMATTED, "class", "smallpink", NULL, "%.2f %%", sval );
                else
                {
                    const char* cls = sval == 0.0f ? "gray" : "pink";
                    html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "-" );
                }
            }
            for (int32 j = first_col; j <= last_col; ++j)
            {
                const float sval = 100.0f * float(stats_l(i,j))/float(n_mapped_both);
                if (sval >= 0.1f)
                {
                    const char* cls = (i == best_bin[0].x && j == best_bin[0].y) ? "yellow" :
                                      (i == best_bin[1].x && j == best_bin[1].y) ? "orange" :
                                      "none";
                    html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "%.1f %%", sval );
                }
                else if (sval >= 0.01f)
                    html::td_object( html_output, html::FORMATTED, "class", "small", NULL, "%.2f %%", sval );
                else
                {
                    const char* cls = sval == 0.0f ? "gray" : "none";
                    html::td_object( html_output, html::FORMATTED, "class", cls, NULL, "-" );
                }
            }
            html::td_object( html_output, html::FORMATTED, "class", "gray", NULL, "" );
        }
        {
            html::tr_object tr( html_output, NULL );
            html::th_object( html_output, html::FORMATTED, NULL, "%s", bin_name );
            for (int32 i = last_col; i >= (diff_table ? 1 : first_col); --i)
                html::td_object( html_output, html::FORMATTED, "class", "gray", NULL, "" );
            for (int32 i = first_col; i <= last_col; ++i)
                html::td_object( html_output, html::FORMATTED, "class", "gray", NULL, "" );
            html::td_object( html_output, html::FORMATTED, "class", "gray", NULL, "" );
        }
    }
}

inline 
void generate_table(const char*            file_name,
                    const char*            stat_by,
                    const char*            name,
                    const char*            diff_name,
                    const Bins             bin_type,
                    const StatsPartition&  stats_l,
                    const StatsPartition&  stats_r,
                    const uint32           total,
                    const uint32           n_mapped_both,
                    const bool             absolute_values = true)
{
    FILE* html_output = fopen( file_name, "w" );
    if (html_output == NULL)
    {
        log_warning( stderr, "unable to write HTML report \"%s\"\n", file_name );
        return;
    }

    std::vector<std::string> read_bins;
    read_bins.push_back( "0" );
    for (uint32 x = 0; x < 12; ++x)
    {
        char buffer[16];
        sprintf(buffer, "%d", read_length_bin_range(x));
        read_bins.push_back( buffer );
    }

    std::vector<std::string> log_bins;
    for (uint32 y = 0; y < 32; ++y)
    {
        if (y == 0)
            log_bins.push_back( "0" );
        else if (y == 1)
            log_bins.push_back( "1" );
        else if (y == 2)
            log_bins.push_back( "2" );
        else
        {
            char buffer[16];
            sprintf(buffer, "2^%u", y-1);
            log_bins.push_back( buffer );
        }
    }

    {
        html::html_object html( html_output );
        {
            const char* meta_list = "<meta http-equiv=\"refresh\" content=\"5\" />";

            html::header_object hd( html_output, "nv-aln-diff report", html::style(), meta_list );
            {
                html::body_object body( html_output );

                generate_diff_table( html_output,
                                     stat_by,
                                     diff_name,
                                     bin_type,
                                     stats_l.diff_hist,
                                     stats_r.diff_hist,
                                     total,
                                     n_mapped_both );

                generate_table2d( html_output,
                                  stat_by,
                                  diff_name,
                                  stats_l.diff_hist_by_length,
                                  stats_r.diff_hist_by_length,
                                  "read length",
                                  read_bins,
                                  log_bins,
                                  total,
                                  n_mapped_both,
                                  true );

                generate_table2d( html_output,
                                  stat_by,
                                  diff_name,
                                  stats_l.diff_hist_by_value_pos,
                                  stats_r.diff_hist_by_value_pos,
                                  "best value",
                                  log_bins,
                                  log_bins,
                                  total,
                                  n_mapped_both,
                                  true );

                generate_table2d( html_output,
                                  stat_by,
                                  diff_name,
                                  stats_l.diff_hist_by_mapQ1,
                                  stats_r.diff_hist_by_mapQ1,
                                  "mapQ [L]",
                                  log_bins,
                                  log_bins,
                                  total,
                                  n_mapped_both,
                                  true );

                generate_table2d( html_output,
                                  stat_by,
                                  diff_name,
                                  stats_l.diff_hist_by_mapQ2,
                                  stats_r.diff_hist_by_mapQ2,
                                  "mapQ [R]",
                                  log_bins,
                                  log_bins,
                                  total,
                                  n_mapped_both,
                                  true );

                if (absolute_values)
                {
                    generate_table( html_output,
                                    stat_by,
                                    name,
                                    bin_type,
                                    stats_l.hist,
                                    stats_r.hist,
                                    stats_l.hist.count,
                                    stats_r.hist.count );

                    generate_table2d( html_output,
                                      stat_by,
                                      name,
                                      stats_l.hist_by_length,
                                      stats_r.hist_by_length,
                                      "read length",
                                      read_bins,
                                      log_bins,
                                      total,
                                      n_mapped_both,
                                      false );

                    generate_table2d( html_output,
                                      stat_by,
                                      name,
                                      stats_l.hist_by_mapQ,
                                      stats_r.hist_by_mapQ,
                                      "mapQ",
                                      log_bins,
                                      log_bins,
                                      total,
                                      n_mapped_both,
                                      false );
                }
            }
        }
    }
    fclose( html_output );
}

} // namespace alndiff
} // namespace nvbio
