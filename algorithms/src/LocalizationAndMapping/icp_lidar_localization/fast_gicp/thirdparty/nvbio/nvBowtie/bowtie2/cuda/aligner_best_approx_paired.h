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

#include <nvBowtie/bowtie2/cuda/utils.h>
#include <nvBowtie/bowtie2/cuda/checksums.h>
#include <nvBowtie/bowtie2/cuda/persist.h>
#include <nvBowtie/bowtie2/cuda/pipeline_states.h>
#include <nvBowtie/bowtie2/cuda/select.h>
#include <nvBowtie/bowtie2/cuda/locate.h>
#include <nvBowtie/bowtie2/cuda/score.h>
#include <nvBowtie/bowtie2/cuda/reduce.h>
#include <nvBowtie/bowtie2/cuda/traceback.h>
#include <nvbio/basic/cuda/pingpong_queues.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/basic/primitives.h>

#include <nvbio/io/output/output_types.h>
#include <nvbio/io/output/output_batch.h>
#include <nvbio/io/output/output_file.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

template <typename MapqCalculator, typename ReadData>
struct MapqFunctorPE
{
    MapqFunctorPE(
        const uint32         _mate,
        MapqCalculator       _mapq_eval,
        const io::Alignment* _best_data,
        const io::Alignment* _best_data_o,
        const uint32         _best_stride,
        const ReadData       _read_data1,
        const ReadData       _read_data2) :
        mapq_eval( _mapq_eval ),
        best_data( _best_data ),
        best_data_o( _best_data_o ),
        best_stride( _best_stride ),
        read_data1( _read_data1 ),
        read_data2( _read_data2 ),
        mate( _mate ) {}

    NVBIO_HOST_DEVICE
    uint8 operator() (const uint32 read_id)
    {
        io::BestAlignments best( best_data[ read_id ], best_data[ read_id + best_stride ] );
        io::BestAlignments best_o( best_data_o[ read_id ], best_data_o[ read_id + best_stride ] );
        io::BestPairedAlignments best_paired(
            (mate == 0) ? best   : best_o,
            (mate == 0) ? best_o : best );

        const uint32 read_offset1 = read_data1.sequence_index()[ read_id ];
        const uint32 read_len1    = read_data1.sequence_index()[ read_id + 1 ] - read_offset1;

        const uint32 read_offset2 = read_data2.sequence_index()[ read_id ];
        const uint32 read_len2    = read_data2.sequence_index()[ read_id + 1 ] - read_offset2;

        return uint8( mapq_eval(
            best_paired,
            best_paired.anchor_mate<0>() ? read_len2 : read_len1,
            best_paired.anchor_mate<0>() ? read_len1 : read_len2 ) );
    }

    const MapqCalculator  mapq_eval;
    const io::Alignment*  best_data;
    const io::Alignment*  best_data_o;
    const uint32          best_stride;
    const ReadData        read_data1;
    const ReadData        read_data2;
    const uint32          mate;
};

template <
    typename scoring_tag>
void Aligner::best_approx(
    const Params&                           params,
    const fmi_type                          fmi,
    const rfmi_type                         rfmi,
    const UberScoringScheme&                input_scoring_scheme,
    const io::SequenceDataDevice&           reference_data,
    const io::FMIndexDataDevice&            driver_data,
    const io::SequenceDataDevice&           read_data1,
    const io::SequenceDataDevice&           read_data2,
    io::HostOutputBatchPE&                  cpu_batch,
    Stats&                                  stats)
{
    // prepare the scoring system
    typedef typename ScoringSchemeSelector<scoring_tag>::type           scoring_scheme_type;
    typedef typename scoring_scheme_type::threshold_score_type          threshold_score_type;

    scoring_scheme_type scoring_scheme = ScoringSchemeSelector<scoring_tag>::scheme( input_scoring_scheme );

    threshold_score_type threshold_score = scoring_scheme.threshold_score();
    //const int32          score_limit     = scoring_scheme_type::worst_score;

    // start timing
    nvbio::Timer timer;
    nvbio::cuda::Timer device_timer;

    const uint32 count = read_data1.size();
    const uint32 band_len = band_length( params.max_dist );

    // cast the genome to use proper iterators
    const io::LdgSequenceDataView   genome_view( plain_view( reference_data ) );
    const genome_access_type        genome_access( genome_view );
    const uint32                    genome_len = genome_access.bps();
    const genome_iterator           genome     = genome_access.sequence_stream();

    // cast the reads to use proper iterators
    const read_view_type  reads_view1 = plain_view( read_data1 );
    const read_view_type  reads_view2 = plain_view( read_data2 );
    const read_batch_type reads1( reads_view1 );
    const read_batch_type reads2( reads_view2 );

    // initialize best-alignments
    init_alignments( reads1, threshold_score, best_data_dptr,   BATCH_SIZE, 0u );
    init_alignments( reads2, threshold_score, best_data_dptr_o, BATCH_SIZE, 1u );

    for (uint32 anchor = 0; anchor < 2; ++anchor)
    {
        log_debug(stderr, "[%u]   anchor mate: %u\n", ID, anchor);

        // start with a full seed queue
        seed_queues.in_size = count;

        thrust::copy(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + count,
            seed_queues.in_queue.begin() );

        //
        // Similarly to Bowtie2, we perform a number of seed & extension passes.
        // Whether a read is re-seeded is determined at run time based on seed hit and
        // alignment statistics.
        // Hence, the number of reads actively processed in each pass can vary substantially.
        // In order to keep the cores and all its lanes busy, we use a pair of input & output
        // queues to compact the set of active reads in each round, swapping them at each
        // iteration.
        //

        const bool fw_strand =
            (anchor == 0) ? (params.pe_policy == io::PE_POLICY_FF || // anchor = 0
                             params.pe_policy == io::PE_POLICY_FR) :
                            (params.pe_policy == io::PE_POLICY_FF || // anchor = 1
                             params.pe_policy == io::PE_POLICY_RF);
        const bool fw = fw_strand ? params.fw : params.rc;
        const bool rc = fw_strand ? params.rc : params.fw;

        for (uint32 seeding_pass = 0; seeding_pass < params.max_reseed+1; ++seeding_pass)
        {
            // check whether the input queue is empty
            if (seed_queues.in_size == 0)
                break;

            const uint32 n_active_reads = seed_queues.in_size;

            // initialize output seeding queue size
            seed_queues.clear_output();

            // check if we need to persist this seeding pass
            if (batch_number == (uint32) params.persist_batch &&
                seeding_pass == (uint32) params.persist_seeding)
                persist_reads( params.persist_file, "reads", anchor, n_active_reads, seed_queues.in_queue.begin() );

            //
            // perform mapping
            //
            {
                log_debug(stderr, "[%u]    mapping (%u active reads)\n", ID, n_active_reads);
                timer.start();
                device_timer.start();

                // initialize the seed hit counts
                hit_deques.clear_deques();

                SeedHitDequeArrayDeviceView hits = hit_deques.device_view();

                map(
                    anchor ? reads2 : reads1, fmi, rfmi,
                    seeding_pass, seed_queues.device_view(),
                    reseed_dptr,
                    hits,
                    params,
                    fw,
                    rc );

                optional_device_synchronize();
                nvbio::cuda::check_error("mapping kernel");

                device_timer.stop();
                timer.stop();
                stats.map.add( n_active_reads, timer.seconds(), device_timer.seconds() );
/*
                #if defined(NVBIO_CUDA_DEBUG)
                {
                    uint64 crc, sum;
                    hits_checksum( n_active_reads, hit_deques, crc, sum );

                    NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "[%u]      crc cnts: %llu\n", ID, device_checksum( hit_deques.counts().begin(), hit_deques.counts().begin() + count ) ) );
                    NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "[%u]      crc hits: %llu\n", ID, crc ) );
                    NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "[%u]      sum hits: %llu\n", ID, sum ) );
                }
                #endif
*/
                // check if we need to persist this seeding pass
                if (batch_number == (uint32) params.persist_batch &&
                    seeding_pass == (uint32) params.persist_seeding)
                    persist_hits( params.persist_file, "hits", anchor, count, hit_deques );
            }

            // take some stats on the hits we got
            if (seeding_pass == 0 && anchor == 0 && params.keep_stats)
                keep_stats( reads1.size(), stats );

            best_approx_score<scoring_tag>(
                params,
                fmi,
                rfmi,
                scoring_scheme,
                reference_data,
                driver_data,
                anchor,
                anchor ? read_data2 : read_data1,
                anchor ? read_data1 : read_data2,
                seeding_pass,
                seed_queues.in_size,
                seed_queues.raw_input_queue(),
                stats );

            // copy the reads that need reseeding
            seed_queues.out_size[0] = nvbio::copy_flagged(
                seed_queues.in_size,
                (uint32*)seed_queues.raw_input_queue(),
                reseed_dptr,
                (uint32*)seed_queues.raw_output_queue(),
                temp_dvec );

            // swap input & output queues
            seed_queues.swap();
        }
    }

    //
    // At this point, for each read we have the scores and rough alignment positions of the
    // best two alignments: to compute the final results we need to backtrack the DP extension,
    // and compute accessory CIGARs and MD strings.
    //

    thrust::device_vector<io::Alignment>::iterator best_anchor_iterator   = best_data_dvec.begin();
    thrust::device_vector<io::Alignment>::iterator best_opposite_iterator = best_data_dvec_o.begin();

    io::Alignment* best_anchor_ptr   = thrust::raw_pointer_cast( best_anchor_iterator.base() );
    io::Alignment* best_opposite_ptr = thrust::raw_pointer_cast( best_opposite_iterator.base() );

    // reclassify here "discordant" alignments as "paired" (if discordant alignments are allowed)
    if (params.pe_discordant)
    {
        mark_discordant(
            count,
            best_anchor_ptr,
            best_opposite_ptr,
            BATCH_SIZE );
    }

    // compute mapq for the first mate
    {
        log_debug(stderr, "[%u]    compute mapq\n", ID);
        typedef BowtieMapq2< SmithWatermanScoringScheme<> > mapq_evaluator_type;

        mapq_evaluator_type mapq_eval( input_scoring_scheme.sw );

        MapqFunctorPE<mapq_evaluator_type,read_view_type> mapq_functor( 0u, mapq_eval, best_anchor_ptr, best_opposite_ptr, BATCH_SIZE, reads_view1, reads_view2 );

        nvbio::transform<device_tag>(
            count,
            thrust::make_counting_iterator<uint32>(0),
            mapq_dvec.begin(),
            mapq_functor );
    }

    TracebackPipelineState<scoring_scheme_type> traceback_state(
        reads1,
        reads2,
        genome_len,
        genome,
        scoring_scheme,
        *this );

    //
    // perform backtracking and compute cigars for the best alignments
    //
    {
        // initialize cigars & MDS
        cigar.clear();
        mds.clear();

        timer.start();
        device_timer.start();

        log_debug(stderr, "[%u]    backtracking\n", ID);
        banded_traceback_best(
            0u,
            count,
            NULL,
            best_anchor_ptr,
            BATCH_SIZE,
            band_len,
            traceback_state,
            params );

        optional_device_synchronize();
        nvbio::cuda::check_error("backtracking kernel");
        if (cigar.has_overflown())
            throw nvbio::runtime_error("CIGAR vector overflow\n");

        device_timer.stop();
        timer.stop();
        stats.backtrack.add( count, timer.seconds(), device_timer.seconds() );

        timer.start();
        device_timer.start();

        log_debug(stderr, "[%u]    alignment\n", ID);
        finish_alignment_best(
            0u,
            count,
            NULL,
            best_anchor_ptr,
            BATCH_SIZE,
            band_len,
            traceback_state,
            input_scoring_scheme.sw,    // always use Smith-Waterman for the final scoring of the found alignments
            params );

        optional_device_synchronize();
        nvbio::cuda::check_error("alignment kernel");
        if (mds.has_overflown())
            throw nvbio::runtime_error("MDS vector overflow\n");

        device_timer.stop();
        timer.stop();
        stats.finalize.add( count, timer.seconds(), device_timer.seconds() );
    }

    // wrap the results in a DeviceOutputBatchSE and process it
    {
        timer.start();

        io::DeviceOutputBatchSE gpu_batch(
            count,
            best_data_dvec,
            io::DeviceCigarArray(cigar, cigar_coords_dvec),
            mds,
            mapq_dvec);

        cpu_batch.readback( gpu_batch, io::MATE_1 );

        timer.stop();
        stats.alignments_DtoH.add( count, timer.seconds() );
    }

    // compute mapq for the second mate
    {
        log_debug(stderr, "[%u]    compute mapq\n", ID);
        typedef BowtieMapq2< SmithWatermanScoringScheme<> > mapq_evaluator_type;

        mapq_evaluator_type mapq_eval( input_scoring_scheme.sw );

        MapqFunctorPE<mapq_evaluator_type,read_view_type> mapq_functor( 1u, mapq_eval, best_anchor_ptr, best_opposite_ptr, BATCH_SIZE, reads_view1, reads_view2 );

        nvbio::transform<device_tag>(
            count,
            thrust::make_counting_iterator<uint32>(0),
            mapq_dvec.begin(),
            mapq_functor );
    }

    //
    // perform backtracking and compute cigars for the opposite mates of the best paired alignments
    //
    {
        // initialize cigars & MDS
        cigar.clear();
        mds.clear();

        //
        // these alignments are of two kinds: concordant, or discordant: this requires some attention,
        // as true opposite paired alignments require full DP backtracking, while unpaired or discordant
        // alignments require the banded version.
        //
        timer.start();
        device_timer.start();

        // overlap the paired indices with the loc queue
        thrust::device_vector<uint32>::iterator paired_idx_begin = scoring_queues.hits.loc.begin();

        // compact the indices of the best concordant alignments
        const uint32 n_paired = uint32( thrust::copy_if(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + count,
            best_opposite_iterator,
            paired_idx_begin,
            io::is_concordant() ) - paired_idx_begin );

        if (n_paired)
        {
            log_debug(stderr, "[%u]    paired opposite: %u\n", ID, n_paired);
            const uint32* paired_idx = thrust::raw_pointer_cast( paired_idx_begin.base() );

            log_debug(stderr, "[%u]    paired opposite backtracking\n", ID);
            opposite_traceback_best(
                0u,
                n_paired,
                paired_idx,
                best_opposite_ptr,
                BATCH_SIZE,
                traceback_state,
                params );

            optional_device_synchronize();
            nvbio::cuda::check_error("paired opposite backtracking kernel");
            if (cigar.has_overflown())
                throw nvbio::runtime_error("CIGAR vector overflow\n");
        }

        // overlap the unpaired indices with the loc queue
        thrust::device_vector<uint32>::iterator unpaired_idx_begin = scoring_queues.hits.loc.begin();

        // compact the indices of the best unpaired or discordant alignments
        const uint32 n_unpaired = uint32( thrust::copy_if(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + count,
            best_opposite_iterator,
            unpaired_idx_begin,
            io::is_not_concordant() ) - unpaired_idx_begin );

        if (n_unpaired)
        {
            log_debug(stderr, "[%u]    unpaired opposite: %u\n", ID, n_unpaired);
            const uint32* unpaired_idx = thrust::raw_pointer_cast( unpaired_idx_begin.base() );

            log_debug(stderr, "[%u]    unpaired opposite backtracking\n", ID);
            banded_traceback_best(
                0u,
                n_unpaired,
                unpaired_idx,
                best_opposite_ptr,
                BATCH_SIZE,
                band_len,
                traceback_state,
                params );

            optional_device_synchronize();
            nvbio::cuda::check_error("unpaired opposite backtracking kernel");
            if (cigar.has_overflown())
                throw nvbio::runtime_error("CIGAR vector overflow\n");
        }

        device_timer.stop();
        timer.stop();
        stats.backtrack_opposite.add( n_paired + n_unpaired, timer.seconds(), device_timer.seconds() );

        timer.start();
        device_timer.start();

        thrust::device_vector<uint32>::iterator aligned_idx_begin = scoring_queues.hits.loc.begin();

        // compact the indices of the best opposite alignments (concordant or not)
        const uint32 n_aligned = uint32( thrust::copy_if(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + count,
            best_opposite_iterator,
            aligned_idx_begin,
            io::is_aligned() ) - aligned_idx_begin );

        if (n_aligned)
        {
            log_debug(stderr, "[%u]    opposite alignment: %u\n", ID, n_aligned);
            const uint32* aligned_idx = thrust::raw_pointer_cast( aligned_idx_begin.base() );

            finish_opposite_alignment_best(
                0u,
                n_aligned,
                aligned_idx,
                best_opposite_ptr,
                BATCH_SIZE,
                band_len,
                traceback_state,
                input_scoring_scheme.sw,    // always use Smith-Waterman for the final scoring of the found alignments
                params );

            optional_device_synchronize();
            nvbio::cuda::check_error("opposite alignment kernel");
            if (mds.has_overflown())
                throw nvbio::runtime_error("MDS vector overflow\n");
        }

        device_timer.stop();
        timer.stop();
        stats.finalize.add( count, timer.seconds(), device_timer.seconds() );

        // wrap the results in a DeviceOutputBatchSE and process it
        if (output_file)
        {
            timer.start();

            io::DeviceOutputBatchSE gpu_batch(
                count,
                best_data_dvec_o,
                io::DeviceCigarArray(cigar, cigar_coords_dvec),
                mds,
                mapq_dvec);

            cpu_batch.readback( gpu_batch, io::MATE_2 );

            timer.stop();
            stats.alignments_DtoH.add( count, timer.seconds() );

            timer.start();

            output_file->process( cpu_batch );

            timer.stop();
            stats.io.add( count, timer.seconds() );
        }
    }

    // TODO: decide whether to output second best alignments
    #if 0
    {
        // clear mapq's
        thrust::fill( mapq_dvec.begin(), mapq_dvec.begin() + count, uint8(255) );

        // overlap the second-best indices with the loc queue
        thrust::device_vector<uint32>::iterator second_idx_begin = scoring_queues.hits.loc.begin();

        // compact the indices of the second-best alignments
        const uint32 n_second = uint32( thrust::copy_if(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + count,
            best_anchor_iterator,
            second_idx_begin,
            io::has_second() ) - second_idx_begin );

        //
        // perform backtracking and compute cigars for the second-best alignments
        //
        if (n_second)
        {
            // initialize cigars & MDS
            cigar.clear();
            mds.clear();

            timer.start();
            device_timer.start();

            NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]    second-best: %u\n", ID, n_second) );
            const uint32* second_idx = thrust::raw_pointer_cast( second_idx_begin.base() );

            NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]    second-best backtracking\n", ID) );
            banded_traceback_best(
                1u,
                n_second,
                second_idx,
                best_anchor_ptr,
                BATCH_SIZE,
                band_len,
                traceback_state,
                params );

            optional_device_synchronize();
            nvbio::cuda::check_error("second-best backtracking kernel");
            if (cigar.has_overflown())
                throw nvbio::runtime_error("CIGAR vector overflow\n");

            device_timer.stop();
            timer.stop();
            stats.backtrack.add( n_second, timer.seconds(), device_timer.seconds() );

            timer.start();
            device_timer.start();

            NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]    second-best alignment\n", ID) );
            finish_alignment_best(
                1u,
                n_second,
                second_idx,
                best_anchor_ptr,
                BATCH_SIZE,
                band_len,
                traceback_state,
                input_scoring_scheme.sw,    // always use Smith-Waterman for the final scoring of the found alignments
                params );

            optional_device_synchronize();
            nvbio::cuda::check_error("second-best alignment kernel");
            if (mds.has_overflown())
                throw nvbio::runtime_error("MDS vector overflow\n");

            device_timer.stop();
            timer.stop();
            stats.finalize.add( n_second, timer.seconds(), device_timer.seconds() );
        }

        // wrap the results in a DeviceOutputBatchSE and process it
        {
            timer.start();

            io::DeviceOutputBatchSE gpu_batch(
                count,
                best_data_dvec,
                io::DeviceCigarArray(cigar, cigar_coords_dvec),
                mds,
                mapq_dvec);

            cpu_batch.readback( gpu_batch, io::MATE_1 );

            timer.stop();
            stats.alignments_DtoH.add( count, timer.seconds() );
        }

        //
        // perform backtracking and compute cigars for the opposite mates of the second-best paired alignments
        //
        {
            // initialize cigars & MDS pools
            cigar.clear();
            mds.clear();

            timer.start();
            device_timer.start();

            //
            // these alignments are of two kinds: paired, or unpaired: this requires some attention,
            // as true opposite paired alignments require full DP backtracking, while unpaired
            // alignments require the banded version.
            //

            // compact the indices of the second-best paired alignments
            const uint32 n_second_paired = uint32( thrust::copy_if(
                thrust::make_counting_iterator(0u),
                thrust::make_counting_iterator(0u) + count,
                best_opposite_iterator,
                second_idx_begin,
                io::has_second_paired() ) - second_idx_begin );

            const uint32* second_idx = thrust::raw_pointer_cast( second_idx_begin.base() );

            if (n_second_paired)
            {
                NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]    second-best paired: %u\n", ID, n_second_paired) );

                NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]    second-best paired opposite backtracking\n", ID) );
                opposite_traceback_best(
                    1u,
                    n_second_paired,
                    second_idx,
                    best_opposite_ptr,
                    BATCH_SIZE,
                    traceback_state,
                    params );

                optional_device_synchronize();
                nvbio::cuda::check_error("second-best paired opposite backtracking kernel");
                if (cigar.has_overflown())
                    throw nvbio::runtime_error("CIGAR vector overflow\n");
            }

            // compact the indices of the second-best unpaired alignments
            const uint32 n_second_unpaired = uint32( thrust::copy_if(
                thrust::make_counting_iterator(0u),
                thrust::make_counting_iterator(0u) + count,
                best_opposite_iterator,
                second_idx_begin,
                io::has_second_unpaired() ) - second_idx_begin );

            if (n_second_unpaired)
            {
                NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]    second-best unpaired: %u\n", ID, n_second_unpaired) );

                NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]    second-best unpaired opposite backtracking\n", ID) );
                banded_traceback_best(
                    1u,
                    n_second_unpaired,
                    second_idx,
                    best_opposite_ptr,
                    BATCH_SIZE,
                    band_len,
                    traceback_state,
                    params );

                optional_device_synchronize();
                nvbio::cuda::check_error("second-best unpaired opposite backtracking kernel");
                if (cigar.has_overflown())
                    throw nvbio::runtime_error("CIGAR vector overflow\n");
            }

            device_timer.stop();
            timer.stop();
            stats.backtrack_opposite.add( n_second_paired + n_second_unpaired, timer.seconds(), device_timer.seconds() );

            timer.start();
            device_timer.start();

            // compact the indices of the second-best alignments
            const uint32 n_second = uint32( thrust::copy_if(
                thrust::make_counting_iterator(0u),
                thrust::make_counting_iterator(0u) + count,
                best_opposite_iterator,
                second_idx_begin,
                io::has_second() ) - second_idx_begin );

            if (n_second)
            {
                // compute alignment only on the opposite mates with a second-best
                NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]    second-best opposite alignment\n", ID) );
                finish_opposite_alignment_best(
                        1u,
                        n_second,
                        second_idx,
                        best_opposite_ptr,
                        BATCH_SIZE,
                        band_len,
                        traceback_state,
                        input_scoring_scheme.sw,    // always use Smith-Waterman for the final scoring of the found alignments
                        params );

                optional_device_synchronize();
                nvbio::cuda::check_error("second-best opposite alignment kernel");
                if (mds.has_overflown())
                    throw nvbio::runtime_error("MDS vector overflow\n");
            }

            device_timer.stop();
            timer.stop();
            stats.finalize.add( n_second, timer.seconds(), device_timer.seconds() );

            // wrap the results in a DeviceOutputBatchSE and process it
            if (output_file)
            {
                timer.start();

                io::DeviceOutputBatchSE gpu_batch(
                    count,
                    best_data_dvec_o,
                    io::DeviceCigarArray(cigar, cigar_coords_dvec),
                    mds,
                    mapq_dvec);

                cpu_batch.readback( gpu_batch, io::MATE_2 );

                timer.stop();
                stats.alignments_DtoH.add( count, timer.seconds() );

                timer.start();

                output_file->process( cpu_batch );

                timer.stop();
                stats.io.add( count, timer.seconds() );
            }
        }
    }
    #endif

    // keep alignment stats
    {
        log_debug(stderr, "[%u]    track stats\n", ID);
        thrust::host_vector<io::Alignment> h_best_data( BATCH_SIZE*2 );
        thrust::host_vector<io::Alignment> h_best_data_o( BATCH_SIZE*2 );
        thrust::host_vector<uint8>         h_mapq( BATCH_SIZE );

        thrust::copy( best_data_dvec.begin(),                best_data_dvec.begin() + count,                h_best_data.begin() );
        thrust::copy( best_data_dvec.begin() + BATCH_SIZE,   best_data_dvec.begin() + count + BATCH_SIZE,   h_best_data.begin() + BATCH_SIZE );
        thrust::copy( best_data_dvec_o.begin(),              best_data_dvec_o.begin() + count,              h_best_data_o.begin() );
        thrust::copy( best_data_dvec_o.begin() + BATCH_SIZE, best_data_dvec_o.begin() + count + BATCH_SIZE, h_best_data_o.begin() + BATCH_SIZE );
        thrust::copy( mapq_dvec.begin(),                     mapq_dvec.begin()      + count,                h_mapq.begin() );

        for (uint32 i = 0; i < count; ++i)
        {
            const io::BestAlignments best( h_best_data[i], h_best_data[i + BATCH_SIZE] );
            const io::BestAlignments best_o( h_best_data_o[i], h_best_data_o[i + BATCH_SIZE] );
            const uint8 mapq = h_mapq[i];

            stats.track_alignment_statistics( best, best_o, mapq );
        }
    }

    // increase the batch number
    ++batch_number;
}

template <
    typename scoring_tag,
    typename scoring_scheme_type>
void Aligner::best_approx_score(
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
    Stats&                                  stats)
{
    log_debug(stderr, "[%u]    score\n", ID);
    // prepare the scoring system
    typedef typename scoring_scheme_type::threshold_score_type          threshold_score_type;

    //threshold_score_type threshold_score = scoring_scheme.threshold_score();
    const int32 score_limit = scoring_scheme_type::worst_score;

    // start processing
    Timer timer;
    Timer global_timer;
    nvbio::cuda::Timer device_timer;

    global_timer.start();

    const uint32 band_len = band_length( params.max_dist );

    const read_view_type  reads_view1 = plain_view( read_data1 );
    const read_view_type  reads_view2 = plain_view( read_data2 );
    const read_batch_type reads1( reads_view1 );
    const read_batch_type reads2( reads_view2 );

    const io::LdgSequenceDataView   genome_view( plain_view( reference_data ) );
    const genome_access_type        genome_access( genome_view );
    const uint32                    genome_len = genome_access.bps();
    const genome_iterator           genome     = genome_access.sequence_stream();

    NVBIO_VAR_UNUSED thrust::device_vector<uint32>::iterator          loc_queue_iterator   = scoring_queues.hits.loc.begin();
                     thrust::device_vector<int32>::iterator           score_queue_iterator = scoring_queues.hits.score.begin();
                     thrust::device_vector<int32>::iterator  opposite_score_queue_iterator = scoring_queues.hits.opposite_score.begin();
                     thrust::device_vector<uint32>::iterator opposite_queue_iterator       = opposite_queue_dvec.begin();

    //
    // At this point we have a queue full of reads, each with an associated set of
    // seed hits encoded as a (sorted) list of SA ranges.
    // For each read we need to:
    //      1. select some seed hit to process (i.e. a row in one of the SA ranges)
    //      2. locate it, i.e. converting from SA to linear coordinates
    //      3. and score it
    // until some search criteria are satisfied.
    // The output queue is then reused in the next round as the input queue, and
    // viceversa.
    //
    ScoringQueues::active_reads_storage_type& active_read_queues = scoring_queues.active_reads;

    active_read_queues.resize( seed_queue_size );

    thrust::transform(
        thrust::device_ptr<const uint32>( seed_queue ),
        thrust::device_ptr<const uint32>( seed_queue ) + seed_queue_size,
        active_read_queues.in_queue.begin(),
        pack_read( params.top_seed ) );

    // keep track of the number of extensions performed for each of the active reads
    uint32 n_ext = 0;

    typedef BestApproxScoringPipelineState<scoring_scheme_type>     pipeline_type;

    pipeline_type pipeline(
        anchor,
        reads1,
        reads2,
        genome_len,
        genome,
        fmi,
        rfmi,
        scoring_scheme,
        score_limit,
        *this );

    // initialize the hit selection & scoring pipeline
    select_init(
        pipeline,
        params );

    optional_device_synchronize();
    nvbio::cuda::check_error("select-init kernel");

    // prepeare the selection context
    SelectBestApproxContext select_context( trys_dptr );

    for (uint32 extension_pass = 0; active_read_queues.in_size && n_ext < params.max_ext; ++extension_pass)
    {
        log_debug(stderr, "[%u]    pass:\n[%u]      batch:[%u]          %u\n[%u]      seeding pass:   %u\n[%u]      extension pass: %u\n", ID, ID, batch_number, ID, seeding_pass, ID, extension_pass);

        // initialize all the scoring output queues
        scoring_queues.clear_output();

        // sort the active read infos
        #if 0
        {
            timer.start();
            device_timer.start();

            NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]    read sort\n", ID) );
            sort_inplace( active_read_queues.in_size, active_read_queues.raw_input_queue() );

            device_timer.stop();
            timer.stop();
            stats.sort.add( active_read_queues.in_size, timer.seconds(), device_timer.seconds() );
        }
        #endif
 
        timer.start();
        device_timer.start();

        // keep track of how many hits per read we are generating
        pipeline.n_hits_per_read = 1;

        if ((active_read_queues.in_size <= BATCH_SIZE/2) &&
            (params.no_multi_hits == false))
        {
            //
            // The queue of actively processed reads is very small: at this point
            // it's better to select multiple hits to process in each round.
            // This adds some book keeping overheads, but allows to make larger
            // kernel launches.
            //

            // the maximum number of extensions we can perform in one iteration
            // is at most 4096, as we use 12 bits to encode the extension index
            const uint32 max_ext = std::min( 4096u, params.max_ext - n_ext );

            // try to generate BATCH_SIZE items to process
            pipeline.n_hits_per_read = std::min(
                BATCH_SIZE / active_read_queues.in_size,
                max_ext );
        }
        // else
        //
        // The queue of actively processed reads is sufficiently large to allow
        // selecting & scoring one seed hit at a time and still have large kernel
        // launches. This is algorithmically the most efficient choice (because
        // it enables frequent early outs), so let's choose it.
        //

        // setup the hits queue according to whether we select multiple hits per read or not
        scoring_queues.hits_index.setup( pipeline.n_hits_per_read, active_read_queues.in_size );

        // update pipeline
        pipeline.scoring_queues = scoring_queues.device_view();

        log_debug(stderr, "[%u]    select\n", ID);
        select(
            select_context,
            pipeline,
            params );

        optional_device_synchronize();
        nvbio::cuda::check_error("select kernel");

        // this sync point seems very much needed: if we don't place it, we won't see
        // the right number of hits later on...
        cudaDeviceSynchronize();

        device_timer.stop();
        timer.stop();
        stats.select.add( active_read_queues.in_size * pipeline.n_hits_per_read, timer.seconds(), device_timer.seconds() );

        // swap input & output queues
        active_read_queues.swap();

        // update pipeline view
        pipeline.scoring_queues = scoring_queues.device_view();

        // fetch the new queue size
        if (active_read_queues.in_size == 0)
            break;

        // fetch the output queue size
        pipeline.hits_queue_size = pipeline.n_hits_per_read > 1 ? scoring_queues.hits_count() : active_read_queues.in_size;
        if (pipeline.hits_queue_size == 0)
            continue;

        // use the parent queue only if we chose the multiple-hits per read pipeline
        // check if we need to persist this seeding pass
        if (batch_number   == (uint32) params.persist_batch &&
            seeding_pass   == (uint32) params.persist_seeding &&
            extension_pass == (uint32) params.persist_extension)
            persist_selection( params.persist_file, "selection",
                anchor,
                active_read_queues.in_size,
                active_read_queues.raw_input_queue(),
                pipeline.n_hits_per_read,
                pipeline.hits_queue_size,
                scoring_queues.hits_index,
                scoring_queues.hits );

        log_debug(stderr, "[%u]    selected %u hits\n", ID, pipeline.hits_queue_size);

        timer.start();
        device_timer.start();

        // sort the selected hits by their SA coordinate
        log_debug(stderr, "[%u]    locate sort\n", ID);
        pipeline.idx_queue = sort_hi_bits( pipeline.hits_queue_size, pipeline.scoring_queues.hits.loc );

        device_timer.stop();
        timer.stop();
        stats.sort.add( pipeline.hits_queue_size, timer.seconds(), device_timer.seconds() );

        timer.start();
        device_timer.start();

        // NOTE: only 75-80% of these locations are unique.
        // It might pay off to do a compaction beforehand.

        // and locate their position in linear coordinates
        log_debug(stderr, "[%u]    locate init\n", ID);
        locate_init( pipeline, params );

        optional_device_synchronize();

        log_debug(stderr, "[%u]    locate lookup\n", ID);
        locate_lookup( pipeline, params );

        optional_device_synchronize();
        nvbio::cuda::check_error("locating kernel");

        device_timer.stop();
        timer.stop();
        stats.locate.add( pipeline.hits_queue_size, timer.seconds(), device_timer.seconds() );

        NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "[%u]      crc: %llu\n", ID, device_checksum( loc_queue_iterator, loc_queue_iterator + pipeline.hits_queue_size ) ) );

        //
        // Start the real scoring work...
        //

        timer.start();
        device_timer.start();

        // sort the selected hits by their linear genome coordinate
        // TODO: sub-sort by read position/RC flag so as to (1) get better coherence,
        // (2) allow removing duplicate extensions
        log_debug(stderr, "[%u]     score sort\n", ID);
        pipeline.idx_queue = sort_hi_bits( pipeline.hits_queue_size, pipeline.scoring_queues.hits.loc );

        device_timer.stop();
        timer.stop();
        stats.sort.add( pipeline.hits_queue_size, timer.seconds(), device_timer.seconds() );

        //
        // assign a score to all selected hits (currently in the output queue)
        //
        float score_time     = 0.0f;
        float dev_score_time = 0.0f;
        timer.start();
        device_timer.start();

        anchor_score_best(
            band_len,
            pipeline,
            params );

        optional_device_synchronize();
        nvbio::cuda::check_error("score kernel");

        device_timer.stop();
        timer.stop();
        score_time += timer.seconds();
        dev_score_time += device_timer.seconds();

        NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "[%u]       crc: %llu\n", ID, device_checksum( score_queue_iterator, score_queue_iterator + pipeline.hits_queue_size ) ) );

        //
        // compact the list of candidate hits (with a valid anchor mate score) and perform DP alignment
        // on the opposite mates
        //

        timer.start();
        device_timer.start();

        // NOTE:
        //  Here we want the output opposite_queue to contain the list of indices *into* idx_queue, so as to keep the
        //  original sorting order by genome coordintes used for scoring the anchor.
        //  Down the pipeline the scoring kernel will address the problems by idx_queue[ opposite_score_queue[ thread_id ] ].
        log_debug(stderr, "[%u]     compact opposite\n", ID);
        const  int32 worst_score = scoring_scheme_type::worst_score;

        pipeline.opposite_queue_size = uint32( thrust::copy_if(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + pipeline.hits_queue_size,
            thrust::make_permutation_iterator( score_queue_iterator, thrust::device_ptr<uint32>( pipeline.idx_queue ) ), // gather from the indexed score queue
            opposite_queue_iterator,
            bind_second_functor< not_equal_functor<int32> >( worst_score ) ) - opposite_queue_iterator );

        // make sure the reducer sees correct scores
        thrust::fill(
            opposite_score_queue_iterator,
            opposite_score_queue_iterator + pipeline.hits_queue_size,
            worst_score );

        if (pipeline.opposite_queue_size)
        {
            // perform DP alignment on the opposite mates
            log_debug(stderr, "[%u]     score opposite (%u)\n", ID, pipeline.opposite_queue_size );
            opposite_score_best(
                pipeline,
                params );

            NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "[%u]       crc: %llu\n", device_checksum( opposite_score_queue_iterator, opposite_score_queue_iterator + pipeline.hits_queue_size ) ) );

            // check if we need to persist this seeding pass
            if (batch_number   == (uint32) params.persist_batch &&
                seeding_pass   == (uint32) params.persist_seeding &&
                extension_pass == (uint32) params.persist_extension)
                persist_scores( params.persist_file, "opposite-score",
                    anchor,
                    active_read_queues.in_size,
                    pipeline.n_hits_per_read,
                    pipeline.hits_queue_size,
                    scoring_queues );
        }

        optional_device_synchronize();
        nvbio::cuda::check_error("opposite-score kernel");

        device_timer.stop();
        timer.stop();
        stats.opposite_score.add( pipeline.opposite_queue_size, timer.seconds(), device_timer.seconds() );
        //stats.opposite_score.user[0] += score_mate_work_queue_stats.utilization(nvbio::cuda::STREAM_EVENT);
        //stats.opposite_score.user[1] += score_mate_work_queue_stats.utilization(nvbio::cuda::RUN_EVENT);
        //stats.opposite_score.user[2] += score_mate_work_queue_stats.avg_iterations();
        //stats.opposite_score.user[3]  = nvbio::max( stats.opposite_score.user[3], score_mate_work_queue_stats.max_iterations() - score_mate_work_queue_stats.avg_iterations() );

        timer.start();
        device_timer.start();

        const ReduceBestApproxContext reduce_context( pipeline.trys, n_ext );

        // reduce the multiple scores to find the best two alignments
        // (one thread per active read).
        log_debug(stderr, "[%u]     score reduce\n", ID);
        score_reduce_paired(
            reduce_context,
            pipeline,
            params );

        optional_device_synchronize();
        nvbio::cuda::check_error("score-reduce kernel");

        // keep track of the number of extensions performed for each of the active reads
        n_ext += pipeline.n_hits_per_read;

        device_timer.stop();
        timer.stop();
        stats.score.add( pipeline.hits_queue_size, score_time + timer.seconds(), dev_score_time + device_timer.seconds() );
    }

    optional_device_synchronize();
    global_timer.stop();
    stats.scoring_pipe.add( seed_queue_size, global_timer.seconds(), global_timer.seconds() );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
