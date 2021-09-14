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

#include <nvBowtie/bowtie2/cuda/scoring.h>
#include <string.h>
#include <string>
#include <map>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

///@addtogroup nvBowtie
///@{

enum MappingMode {
    BestMappingApprox = 0,
    BestMappingExact  = 1,
    AllMapping        = 2
};

enum ScoringMode {
    EditDistanceMode  = 0,
    SmithWatermanMode = 1,
};

static const char* s_mapping_mode[] = {
    "best",
    "best-exact",
    "all"
};
inline const char* mapping_mode(const uint32 mode)
{
    return s_mapping_mode[ mode ];
}

inline uint32 mapping_mode(const char* str)
{
    if (strcmp( str, "best" ) == 0)
        return BestMappingApprox;
    else if (strcmp( str, "best-exact" ) == 0)
        return BestMappingExact;
    else
        return AllMapping;
}

static const char* s_scoring_mode[] = {
    "ed",
    "sw"
};
inline const char* scoring_mode(const uint32 mode)
{
    return s_scoring_mode[ mode ];
}

inline uint32 scoring_mode(const char* str)
{
    if (strcmp( str, "ed" ) == 0)
        return EditDistanceMode;
    else if (strcmp( str, "sw" ) == 0)
        return SmithWatermanMode;
    else
        return EditDistanceMode;
}

///
/// A POD structure holding all of nvBowtie's parameters
///
struct ParamsPOD
{
    bool          keep_stats;
    bool          randomized;
    bool          fw;
    bool          rc;
    uint32        mode;
    uint32        scoring_mode;
    uint32        alignment_type;
    uint32        top_seed;
    uint32        seed_len;
    SimpleFunc    seed_freq;
    uint32        max_hits;
    uint32        max_dist;
    uint32        max_effort_init;
    uint32        max_effort;
    uint32        min_ext;
    uint32        max_ext;
    uint32        max_reseed;
    uint32        rep_seeds;
    uint32        allow_sub;
    uint32        subseed_len;
    uint32        mapq_filter;
    uint32        min_read_len;
    uint32        max_batch_size;
    uint32        avg_read_length;
    bool          ungapped_mates;

    // paired-end options
    uint32        pe_policy;
    bool          pe_overlap;
    bool          pe_dovetail;
    bool          pe_unpaired;
    bool          pe_discordant;
    uint32        min_frag_len;
    uint32        max_frag_len;

    // Scoring scheme
    UberScoringScheme scoring_scheme;

    // Internal fields
    uint32        scoring_window;
    DebugState    debug;
};

///
/// A non-POD structure holding all of nvBowtie's parameters plus a few extra string options
///
struct Params : public ParamsPOD
{
    std::string   report;
    std::string   scoring_file;

    int32         persist_batch;
    int32         persist_seeding;
    int32         persist_extension;
    std::string   persist_file;

    int32         no_multi_hits;
};

///
/// load options from a file
///
std::map<std::string,std::string> load_options(const char* name);

///
/// parse options
///
void parse_options(Params& params, const std::map<std::string,std::string>& options, bool init);

///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
