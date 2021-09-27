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

namespace nvbio {
namespace alndiff {

enum Type { LOWER  = 0, HIGHER = 1 };
enum Bins { LINEAR = 0, LOG    = 1 };

struct StatsPartition
{
    Histogram<256>      hist;
    Histogram2d<12,10>  hist_by_length;
    Histogram2d<12,10>  hist_by_mapQ;
    Histogram<256>      diff_hist;
    Histogram2d<12,10>  diff_hist_by_length;
    Histogram2d<32,10>  diff_hist_by_value_neg;
    Histogram2d<32,10>  diff_hist_by_value_pos;
    Histogram2d<7,12>   diff_hist_by_mapQ1;
    Histogram2d<7,12>   diff_hist_by_mapQ2;
};

template <Type TYPE_T, Bins BINS_T>
struct Stats
{
    static const Type TYPE = TYPE_T;
    static const Bins BINS = BINS_T;

    typedef StatsPartition Partition;

    Bins bin_type() const { return BINS; }

    int32 push(const int32 val1, const int32 val2, const uint32 length_bin, const uint32 mapQ1, const uint32 mapQ2)
    {
        l.hist.push( (BINS == LOG) ? log_bin(val1) : val1 );
        l.hist_by_length.push( length_bin, log_bin( val1 ) );
        l.hist_by_mapQ.push( log_bin( mapQ1 ), log_bin( val1 ) );

        r.hist.push( (BINS == LOG) ? log_bin(val2) : val2 );
        r.hist_by_length.push( length_bin, log_bin( val2 ) );
        r.hist_by_mapQ.push( log_bin( mapQ2 ), log_bin( val2) );

        if (TYPE == LOWER)
        {
            const int32 diff = int32(val2) - int32(val1);
            if (diff >= 0)
            {
                const int32 log_diff = log_bin( diff );
                l.diff_hist.push( (BINS == LOG) ? log_diff : diff );
                l.diff_hist_by_length.push( length_bin,   log_diff );
                l.diff_hist_by_mapQ1.push( log_bin(mapQ1),  log_diff );
                l.diff_hist_by_mapQ2.push( log_bin(mapQ2),  log_diff );
                if (val1 < 0)
                    l.diff_hist_by_value_neg.push( log_bin(-val1), log_diff );
                else
                    l.diff_hist_by_value_pos.push( log_bin(val1), log_diff );
            }
            if (diff <= 0)
            {
                const int32 log_diff = log_bin( -diff );
                r.diff_hist.push( (BINS == LOG) ? log_diff : -diff );
                r.diff_hist_by_length.push( length_bin,   log_diff );
                r.diff_hist_by_mapQ1.push( log_bin(mapQ1),  log_diff );
                r.diff_hist_by_mapQ2.push( log_bin(mapQ2),  log_diff );
                if (val2 < 0)
                    r.diff_hist_by_value_neg.push( log_bin(-val2), log_diff );
                else
                    r.diff_hist_by_value_pos.push( log_bin(val2), log_diff );
            }
            return diff;
        }
        else
        {
            const int32 diff = int32(val1) - int32(val2);
            if (diff >= 0)
            {
                const int32 log_diff = log_bin( diff );
                l.diff_hist.push( (BINS == LOG) ? log_diff : diff );
                l.diff_hist_by_length.push( length_bin,   log_diff );
                l.diff_hist_by_mapQ1.push( log_bin(mapQ1),  log_diff );
                l.diff_hist_by_mapQ2.push( log_bin(mapQ2),  log_diff );
                if (val2 < 0)
                    l.diff_hist_by_value_neg.push( log_bin(-val2), log_diff );
                else
                    l.diff_hist_by_value_pos.push( log_bin(val2), log_diff );
            }
            if (diff <= 0)
            {
                const int32 log_diff = log_bin( -diff );
                r.diff_hist.push( (BINS == LOG) ? log_diff : -diff );
                r.diff_hist_by_length.push( length_bin,   log_diff );
                r.diff_hist_by_mapQ1.push( log_bin(mapQ1),  log_diff );
                r.diff_hist_by_mapQ2.push( log_bin(mapQ2),  log_diff );
                if (val1 < 0)
                    r.diff_hist_by_value_neg.push( log_bin(-val1), log_diff );
                else
                    r.diff_hist_by_value_pos.push( log_bin(val1), log_diff );
            }
            return diff;
        }
    }

    Partition   l;
    Partition   r;
};

struct AlignmentStats
{
    Stats<HIGHER,LOG>          higher_score;
    Stats<LOWER,LINEAR>        lower_ed;
    Stats<HIGHER,LOG>          higher_mapQ;
    Stats<HIGHER,LOG>          longer_mapping;
    Stats<HIGHER,LOG>          higher_pos;
    Stats<LOWER,LINEAR>        lower_subs;
    Stats<LOWER,LINEAR>        lower_ins;
    Stats<LOWER,LINEAR>        lower_dels;
    Stats<LOWER,LINEAR>        lower_mms;
};

} // namespace alndiff
} // namespace nvbio
