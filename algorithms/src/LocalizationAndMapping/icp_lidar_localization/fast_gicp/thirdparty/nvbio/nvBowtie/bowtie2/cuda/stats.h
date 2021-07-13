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

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvbio/basic/timer.h>
#include <vector>
#include <deque>

namespace nvbio { namespace io { struct BestAlignments; } }

namespace nvbio {
namespace bowtie2 {
namespace cuda {

typedef nvbio::TimeSeries KernelStats;


struct AlignmentStats
{
    AlignmentStats()
        : n_mapped(0),
          n_ambiguous(0),
          n_unambiguous(0),
          n_unique(0),
          n_multiple(0),
          mapped_ed_histogram(4096, 0),
          mapped_ed_histogram_fwd(4096, 0),
          mapped_ed_histogram_rev(4096, 0),
          mapped_log_ed_histogram(12, 0)
    {
        for(uint32 c = 0; c < 64; c++)
            mapq_bins[c] = 0;

        for(uint32 c = 0; c < 8; c++)
            mapq_log_bins[c] = 0;

        for(uint32 c = 0; c < 64; c++)
            for(uint32 d = 0; d < 64; d++)
                mapped_ed_correlation[c][d] = 0;
    }

    // mapping quality stats
    uint64 mapq_bins[64];
    uint64 mapq_log_bins[8];

    // mapping stats
    uint32          n_mapped;       // number of mapped reads

    uint32          n_ambiguous;    // number of reads mapped to more than one place with the same score
    uint32          n_unambiguous;  // number of reads with a single best score (even if mapped to multiple locations)

    uint32          n_unique;       // number of reads mapped to a single location
    uint32          n_multiple;     // number of reads mapped to more than one location

    // edit distance scoring histograms
    std::vector<uint32> mapped_ed_histogram;        // aggregate histogram of edit-distance scores per read
    std::vector<uint32> mapped_ed_histogram_fwd;    // histogram of edit-distance scores for reads mapped to the forward sequence
    std::vector<uint32> mapped_ed_histogram_rev;    // histogram of edit-distance scores for reads mapped to the reverse-complemented sequence
    std::vector<uint32> mapped_log_ed_histogram;    // aggregate histogram of log edit-distance scores per read
    std::vector<uint32> mapped_log_mapq_histogram;  // aggregate histogram of log mapq scores per read

    // edit distance correlation (xxxnsubtil: what exactly does this measure?)
    uint32  mapped_ed_correlation[64][64];

    // merge stats
    void merge(const AlignmentStats& stats)
    {
        n_mapped      += stats.n_mapped;
        n_ambiguous   += stats.n_ambiguous;
        n_unambiguous += stats.n_unambiguous;
        n_unique      += stats.n_unique;
        n_multiple    += stats.n_multiple;

        for (uint32 i = 0; i < mapped_ed_histogram.size(); ++i)
        {
            mapped_ed_histogram[i]     += stats.mapped_ed_histogram[i];
            mapped_ed_histogram_fwd[i] += stats.mapped_ed_histogram_fwd[i];
            mapped_ed_histogram_rev[i] += stats.mapped_ed_histogram_rev[i];
        }
        for (uint32 i = 0; i < 12; ++i)
            mapped_log_ed_histogram[i] += stats.mapped_log_ed_histogram[i];

        for(uint32 c = 0; c < 64; c++)
            mapq_bins[c] += stats.mapq_bins[c];

        for(uint32 c = 0; c < 8; c++)
            mapq_log_bins[c] += stats.mapq_log_bins[c];

        for(uint32 c = 0; c < 64; c++)
            for(uint32 d = 0; d < 64; d++)
                mapped_ed_correlation[c][d] += stats.mapped_ed_correlation[c][d];
    }
};

//
// Global statistics
//
struct Stats
{
    // constructor
    Stats(const Params _params = Params());

    // timing stats
    float       global_time;
    KernelStats map;
    KernelStats select;
    KernelStats sort;
    KernelStats locate;
    KernelStats score;
    KernelStats opposite_score;
    KernelStats backtrack;
    KernelStats backtrack_opposite;
    KernelStats finalize;
    KernelStats alignments_DtoH;
    KernelStats read_HtoD;
    KernelStats read_io;
    KernelStats io;
    KernelStats scoring_pipe;

    // detailed mapping stats
    AlignmentStats  concordant;
    AlignmentStats  discordant;
    AlignmentStats  mate1;
    AlignmentStats  mate2;

    // mapping stats
    uint32 n_reads;

    // extensive (seeding) stats
    uint64 hits_total;
    uint64 hits_ranges;
    uint32 hits_max;
    uint32 hits_max_range;
    uint64 hits_top_total;
    uint32 hits_top_max;
    uint64 hits_bins[28];
    uint64 hits_top_bins[28];
    uint32 hits_stats;

    Params params;

    void track_alignment_statistics(
        const io::BestAlignments&   alignment1,
        const io::BestAlignments&   alignment2,
        const uint8                 mapq);

    void track_alignment_statistics(
        AlignmentStats*             mate,
        const io::BestAlignments&   alignment,
        const uint8                 mapq);
};

void generate_report_header(const uint32 n_reads, const Params& params, AlignmentStats& aln_stats, const uint32 n_devices, const Stats* device_stats, const char* report);
void generate_device_report(const uint32 id, Stats& stats, AlignmentStats& aln_stats, const char* report);

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
