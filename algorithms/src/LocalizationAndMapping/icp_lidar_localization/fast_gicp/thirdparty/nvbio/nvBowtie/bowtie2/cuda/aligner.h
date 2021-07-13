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
#include <nvBowtie/bowtie2/cuda/reads_def.h>
#include <nvBowtie/bowtie2/cuda/fmindex_def.h>
#include <nvBowtie/bowtie2/cuda/seed_hit.h>
#include <nvBowtie/bowtie2/cuda/seed_hit_deque_array.h>
#include <nvBowtie/bowtie2/cuda/scoring_queues.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvBowtie/bowtie2/cuda/stats.h>
#include <nvBowtie/bowtie2/cuda/mapping.h>
#include <nvbio/io/alignments.h>
#include <nvbio/io/output/output_file.h>
#include <nvbio/io/output/output_batch.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvBowtie/bowtie2/cuda/scoring.h>
#include <nvBowtie/bowtie2/cuda/mapq.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/sort.h>
#include <nvbio/basic/cuda/host_device_buffer.h>
#include <nvbio/basic/cuda/work_queue.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/options.h>
#include <nvbio/basic/threads.h>
#include <nvbio/basic/html.h>
#include <nvbio/basic/dna.h>
#include <nvbio/basic/vector_array.h>
#include <nvbio/fmindex/bwt.h>
#include <nvbio/fmindex/ssa.h>
#include <nvbio/fmindex/fmindex.h>
#include <nvbio/fmindex/fmindex_device.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/scan.h>
#include <thrust/sort.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <functional>



namespace nvbio {
namespace bowtie2 {
namespace cuda {

struct Aligner
{
    static const uint32 MAX_READ_LEN = MAXIMUM_READ_LENGTH;

    typedef FMIndexDef::type                                                                                                 fmi_type;
    typedef FMIndexDef::type                                                                                                 rfmi_type;

    typedef ReadsDef::read_storage_type                                                                                      read_storage_type;
    typedef ReadsDef::read_base_type                                                                                         read_base_type;
    typedef ReadsDef::read_qual_type                                                                                         read_qual_type;
    typedef ReadsDef::read_view_type                                                                                         read_view_type;
    typedef ReadsDef::type                                                                                                   read_batch_type;

    typedef io::LdgSequenceDataView                                                                                          genome_view_type;
    typedef io::SequenceDataAccess<DNA,genome_view_type>                                                                     genome_access_type;
    typedef genome_access_type::sequence_stream_type                                                                         genome_iterator;


    uint32                              ID;
    uint32                              BATCH_SIZE;

    thrust::device_vector<uint8>        dp_buffer_dvec;
    uint8*                              dp_buffer_dptr;

    SeedHitDequeArray                   hit_deques;

    nvbio::cuda::PingPongQueues<uint32> seed_queues;
    ScoringQueues                       scoring_queues;

    thrust::device_vector<uint32>       idx_queue_dvec;
    uint32*                             idx_queue_dptr;
    thrust::device_vector<uint16>       sorting_queue_dvec;

    thrust::device_vector<uint8>        reseed_dvec;
    uint8*                              reseed_dptr;
    thrust::device_vector<uint32>       trys_dvec;
    uint32*                             trys_dptr;
    thrust::device_vector<uint32>       rseeds_dvec;
    uint32*                             rseeds_dptr;
    thrust::device_vector<uint8>        flags_dvec;
    uint8*                              flags_dptr;
    nvbio::vector<device_tag,uint8>     temp_dvec;

    thrust::device_vector<io::Alignment>    best_data_dvec;
    thrust::device_vector<io::Alignment>    best_data_dvec_o;
    io::Alignment*                          best_data_dptr;
    io::Alignment*                          best_data_dptr_o;
    thrust::device_vector<uint8>            mapq_dvec;
    uint8*                                  mapq_dptr;

    // --- paired-end vectors --------------------------------- //
    thrust::device_vector<uint32>   opposite_queue_dvec;
    uint32*                         opposite_queue_dptr;

    // --- all-mapping vectors -------------------------------- //
    thrust::device_vector<io::Alignment>  buffer_alignments_dvec;
    io::Alignment*                        buffer_alignments_dptr;
    thrust::device_vector<uint32>         buffer_read_info_dvec;
    uint32*                               buffer_read_info_dptr;
    thrust::device_vector<io::Alignment>  output_alignments_dvec;
    io::Alignment*                        output_alignments_dptr;
    thrust::device_vector<uint32>         output_read_info_dvec;
    uint32*                               output_read_info_dptr;

    thrust::device_vector<uint32>         hits_count_scan_dvec;
    uint32*                               hits_count_scan_dptr;
    thrust::device_vector<uint64>         hits_range_scan_dvec;
    uint64*                               hits_range_scan_dptr;
    // -------------------------------------------------------- //

    nvbio::DeviceVectorArray<uint8>       mds;
    nvbio::DeviceVectorArray<io::Cigar>   cigar;
    thrust::device_vector<uint2>          cigar_coords_dvec;
    uint2*                                cigar_coords_dptr;

    thrust::device_vector<uint64>   hits_stats_dvec;
    thrust::host_vector<uint64>     hits_stats_hvec;
    uint64*                         hits_stats_dptr;

    uint32                          batch_number;

    nvbio::cuda::SortEnactor        sort_enactor;
    // --------------------------------------------------------------------------------------------- //

    // file object that we're writing into
    io::OutputFile *output_file;

    static uint32 band_length(const uint32 max_dist)
    {
        //return max_dist*2+1;
        // compute band length
        uint32 band_len = 4;
        while (band_len-1 < max_dist*2+1)
            band_len *= 2;
        band_len -= 1;
        return band_len;
    }

    Aligner() : output_file(NULL) {}

    bool init(const uint32 id, const uint32 batch_size, const Params& params, const EndType type);

    void keep_stats(const uint32 count, Stats& stats);

    template <typename scoring_tag>
    void best_approx(
        const Params&                           params,
        const fmi_type                          fmi,
        const rfmi_type                         rfmi,
        const UberScoringScheme&                scoring_scheme,
        const io::SequenceDataDevice&           reference_data,
        const io::FMIndexDataDevice&            driver_data,
        const io::SequenceDataDevice&           read_data,
        io::HostOutputBatchSE&                  cpu_batch,
        Stats&                                  stats);

    template <
        typename scoring_tag,
        typename scoring_scheme_type>
    void best_approx_score(
        const Params&                           params,
        const fmi_type                          fmi,
        const rfmi_type                         rfmi,
        const scoring_scheme_type&              scoring_scheme,
        const io::SequenceDataDevice&           reference_data,
        const io::FMIndexDataDevice&            driver_data,
        const io::SequenceDataDevice&           read_data,
        const uint32                            seeding_pass,
        const uint32                            seed_queue_size,
        const uint32*                           seed_queue,
        Stats&                                  stats);

    template <typename scoring_tag>
    void best_approx(
        const Params&                           params,
        const FMIndexDef::type                  fmi,
        const FMIndexDef::type                  rfmi,
        const UberScoringScheme&                scoring_scheme,
        const io::SequenceDataDevice&           reference_data,
        const io::FMIndexDataDevice&            driver_data,
        const io::SequenceDataDevice&           read_data1,
        const io::SequenceDataDevice&           read_data2,
        io::HostOutputBatchPE&                  cpu_batch,
        Stats&                                  stats);

    template <
        typename scoring_tag,
        typename scoring_scheme_type>
    void best_approx_score(
        const Params&                           params,
        const fmi_type                          fmi,
        const rfmi_type                         rfmi,
        const scoring_scheme_type&              scoring_scheme,
        const io::SequenceDataDevice&           reference_data,
        const io::FMIndexDataDevice&            driver_data,
        const uint32                            anchor,
        const io::SequenceDataDevice&           read_data1,
        const io::SequenceDataDevice&           read_data2,
        const uint32                            seeding_pass,
        const uint32                            seed_queue_size,
        const uint32*                           seed_queue,
        Stats&                                  stats);

    template <typename scoring_tag>
    void all(
        const Params&                           params,
        const fmi_type                          fmi,
        const rfmi_type                         rfmi,
        const UberScoringScheme&                scoring_scheme,
        const io::SequenceDataDevice&           reference_data,
        const io::FMIndexDataDevice&            driver_data,
        const io::SequenceDataDevice&           read_data,
        io::HostOutputBatchSE&                  cpu_batch,
        Stats&                                  stats);

    template <typename scoring_scheme_type>
    void score_all(
        const Params&                           params,
        const fmi_type                          fmi,
        const rfmi_type                         rfmi,
        const UberScoringScheme&                input_scoring_scheme,
        const scoring_scheme_type&              scoring_scheme,
        const io::SequenceDataDevice&           reference_data,
        const io::FMIndexDataDevice&            driver_data,
        const io::SequenceDataDevice&           read_data,
        io::HostOutputBatchSE&                  cpu_batch,
        const uint32                            seed_queue_size,
        const uint32*                           seed_queue,
        Stats&                                  stats,
        uint64&                                 total_alignments);

  #if defined(__CUDACC__)
    // return a pointer to an "index" into the given sorted keys
    //
    template <typename iterator_type>
    std::pair<uint32*,uint64*> sort_64_bits(
        const uint32        count,
        const iterator_type keys)
    {
        thrust::copy(
            keys,
            keys + count,
            thrust::device_ptr<uint64>( (uint64*)raw_pointer( sorting_queue_dvec ) ) );

        return sort_64_bits( count );
    }
  #endif

    // return a pointer to an "index" into the given sorted keys
    //
    std::pair<uint32*,uint64*> sort_64_bits(
        const uint32 count);

    // return a pointer to an "index" into the given keys sorted by their hi bits
    //
    uint32* sort_hi_bits(
        const uint32    count,
        const uint32*   keys);

    // sort a set of keys in place
    //
    void sort_inplace(
        const uint32    count,
        uint32*         keys);

    bool init_alloc(const uint32 BATCH_SIZE, const Params& params, const EndType type, bool do_alloc, std::pair<uint64,uint64>* mem_stats = NULL);
};

// Compute the total number of matches found
void hits_stats(
    const uint32    batch_size,
    const SeedHit*  hit_data,
    const uint32*   hit_counts,
          uint64*   hit_stats);

void ring_buffer_to_plain_array(
    const uint32* buffer,
    const uint32  buffer_size,
    const uint32  begin,
    const uint32  end,
          uint32* output);

#if defined(__CUDACC__)

// initialize a set of alignments
//
template <typename ReadBatch, typename ScoreFunction>
__global__
void init_alignments_kernel(
    const ReadBatch         read_batch,
    const ScoreFunction     worst_score_fun,
    io::Alignment*          best_data,
    const uint32            best_stride,
    const uint32            mate)
{
    const uint32 thread_id = threadIdx.x + BLOCKDIM*blockIdx.x;
    if (thread_id >= read_batch.size()) return;

    // compute the read length
    const uint2 read_range = read_batch.get_range( thread_id );
    const uint32 read_len  = read_range.y - read_range.x;

    const int32 worst_score = worst_score_fun( read_len );

    io::Alignment a1 = io::Alignment( uint32(-1), io::Alignment::max_ed(), worst_score, mate );
    io::Alignment a2 = io::Alignment( uint32(-1), io::Alignment::max_ed(), worst_score, mate );
    best_data[ thread_id ]               = a1;
    best_data[ thread_id + best_stride ] = a2;
}

// initialize a set of alignments
//
template <typename ReadBatch, typename ScoreFunction>
void init_alignments(
    const ReadBatch         read_batch,
    const ScoreFunction     worst_score_fun,
    io::Alignment*          best_data,
    const uint32            best_stride,
    const uint32            mate = 0)
{
    const int blocks = (read_batch.size() + BLOCKDIM-1) / BLOCKDIM;

    init_alignments_kernel<<<blocks, BLOCKDIM>>>(
        read_batch,
        worst_score_fun,
        best_data,
        best_stride,
        mate );
}

#endif // defined(__CUDACC__)

// mark unaligned reads that need reseeding
//
void mark_unaligned(
    const uint32            n_active_reads,
    const uint32*           active_reads,
    const io::Alignment*    best_data,
    uint8*                  reseed);

// mark unique unaligned read pairs as discordant
//
void mark_discordant(
    const uint32            n_reads,
          io::Alignment*    anchor_data,
          io::Alignment*    opposite_data,
    const uint32            stride);

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
