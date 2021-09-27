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

#include <nvbio/io/output/output_batch.h>
#include <nvbio/io/output/output_file.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

template <typename MapqCalculator, typename ReadData>
struct MapqFunctorSE
{
    typedef uint32  argument_type;
    typedef uint8   result_type;

    MapqFunctorSE(
        MapqCalculator       _mapq_eval,
        const io::Alignment* _best_data,
        const uint32         _best_stride,
        const ReadData       _read_data) :
        mapq_eval( _mapq_eval ),
        best_data( _best_data ),
        best_stride( _best_stride ),
        read_data( _read_data ) {}

    NVBIO_HOST_DEVICE
    uint8 operator() (const uint32 read_id) const
    {
        const io::BestAlignments best( best_data[ read_id ], best_data[ read_id + best_stride ] );

        const uint32 read_offset = read_data.sequence_index()[ read_id ];
        const uint32 read_len    = read_data.sequence_index()[ read_id + 1 ] - read_offset;

        return uint8( mapq_eval(
            io::BestPairedAlignments( best, io::BestAlignments() ),
            read_len,
            read_len ) );
    }

    const MapqCalculator mapq_eval;
    const io::Alignment* best_data;
    const uint32         best_stride;
    const ReadData       read_data;
};

template <typename scoring_tag>
void Aligner::best_approx(
    const Params&                           params,
    const fmi_type                          fmi,
    const rfmi_type                         rfmi,
    const UberScoringScheme&                input_scoring_scheme,
    const io::SequenceDataDevice&           reference_data,
    const io::FMIndexDataDevice&            driver_data,
    const io::SequenceDataDevice&           read_data,
    io::HostOutputBatchSE&                  cpu_batch,
    Stats&                                  stats)
{
    // cast the genome to use proper iterators
    const genome_view_type          genome_view( plain_view( reference_data ) );
    const genome_access_type        genome_access( genome_view );
    const uint32                    genome_len = genome_access.bps();
    const genome_iterator           genome     = genome_access.sequence_stream();

    // prepare the scoring system
    typedef typename ScoringSchemeSelector<scoring_tag>::type           scoring_scheme_type;
    typedef typename scoring_scheme_type::threshold_score_type          threshold_score_type;

    scoring_scheme_type scoring_scheme = ScoringSchemeSelector<scoring_tag>::scheme( input_scoring_scheme );

    threshold_score_type threshold_score = scoring_scheme.threshold_score();
    //const int32          score_limit     = scoring_scheme_type::worst_score;

    // start timing
    Timer timer;
    Timer global_timer;
    nvbio::cuda::Timer device_timer;

    const uint32 count = read_data.size();
    const uint32 band_len = band_length( params.max_dist );

    // create a device-side read batch
    const read_view_type  reads_view = plain_view( read_data );
    const read_batch_type reads( reads_view );

    // initialize best-alignments
    init_alignments( reads, threshold_score, best_data_dptr, BATCH_SIZE );

    seed_queues.resize( count );

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

    // filter away reads that map exactly
    if (0)
    {
        // initialize output seeding queue size
        seed_queues.clear_output();

        //
        // perform whole read mapping
        //
        {
            timer.start();
            device_timer.start();

            // initialize the seed hit counts
            hit_deques.clear_deques();

            SeedHitDequeArrayDeviceView hits = hit_deques.device_view();

            log_debug(stderr, "[%u]     map\n", ID);
            map_whole_read(
                reads, fmi, rfmi,
                seed_queues.device_view(),
                reseed_dptr,
                hits,
                params,
                params.fw,
                params.rc );

            optional_device_synchronize();
            nvbio::cuda::check_error("mapping kernel");

            device_timer.stop();
            timer.stop();
            stats.map.add( seed_queues.in_size, timer.seconds(), device_timer.seconds() );
        }

        best_approx_score<scoring_tag>(
            params,
            fmi,
            rfmi,
            scoring_scheme,
            reference_data,
            driver_data,
            read_data,
            uint32(-1),
            seed_queues.in_size,
            seed_queues.raw_input_queue(),
            stats );

        log_verbose( stderr, "[%u]     %.1f %% reads map exactly\n", ID, 100.0f * float(count - seed_queues.output_size())/float(count) );

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

    for (uint32 seeding_pass = 0; seeding_pass < params.max_reseed+1; ++seeding_pass)
    {
        // check whether the input queue is empty
        if (seed_queues.in_size == 0)
            break;

        // initialize output seeding queue size
        seed_queues.clear_output();

        //
        // perform mapping
        //
        {
            timer.start();
            device_timer.start();

            hit_deques.clear_deques();

            SeedHitDequeArrayDeviceView hits = hit_deques.device_view();

            log_debug(stderr, "[%u]     map\n", ID);
            map(
                reads, fmi, rfmi,
                seeding_pass, seed_queues.device_view(),
                reseed_dptr,
                hits,
                params,
                params.fw,
                params.rc );

            optional_device_synchronize();
            nvbio::cuda::check_error("mapping kernel");

            device_timer.stop();
            timer.stop();
            stats.map.add( seed_queues.in_size, timer.seconds(), device_timer.seconds() );

            // check if we need to persist this seeding pass
            if (params.persist_batch   == -1 || batch_number == (uint32) params.persist_batch &&
                params.persist_seeding == -1 || seeding_pass == (uint32) params.persist_seeding)
                persist_hits( params.persist_file, "hits", 0u, count, hit_deques );
        }

        // take some stats on the hits we got
        if (seeding_pass == 0 && params.keep_stats)
            keep_stats( reads.size(), stats );

        best_approx_score<scoring_tag>(
            params,
            fmi,
            rfmi,
            scoring_scheme,
            reference_data,
            driver_data,
            read_data,
            seeding_pass,
            seed_queues.in_size,
            seed_queues.raw_input_queue(),
            stats );

        // mark unaligned reads
        mark_unaligned(
            seed_queues.in_size,
            seed_queues.raw_input_queue(),
            best_data_dptr,
            reseed_dptr );

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

    //
    // At this point, for each read we have the scores and rough alignment positions of the
    // best two alignments: to compute the final results we need to backtrack the DP extension,
    // and compute accessory CIGARS and MD strings.
    //

    // compute mapq
    {
        log_debug(stderr, "[%u]     compute mapq\n", ID);
        typedef BowtieMapq2< SmithWatermanScoringScheme<> > mapq_evaluator_type;

        mapq_evaluator_type mapq_eval( input_scoring_scheme.sw );

        MapqFunctorSE<mapq_evaluator_type,read_view_type> mapq_functor( mapq_eval, best_data_dptr, BATCH_SIZE, reads_view );

        nvbio::transform<device_tag>(
            count,
            thrust::make_counting_iterator<uint32>(0),
            mapq_dvec.begin(),
            mapq_functor );
    }

    TracebackPipelineState<scoring_scheme_type> traceback_state(
        reads,
        reads,
        genome_len,
        genome,
        scoring_scheme,
        *this );

    //
    // perform backtracking and compute cigars for the best alignments
    //
    {
        // initialize cigars & MDS pools
        cigar.clear();
        mds.clear();

        timer.start();
        device_timer.start();

        log_debug(stderr, "[%u]     backtrack\n", ID);
        banded_traceback_best(
            0u,
            count,
            NULL,
            best_data_dptr,
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

        log_debug(stderr, "[%u]     alignment\n", ID);
        finish_alignment_best(
            0u,
            count,
            NULL,
            best_data_dptr,
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

    // wrap the results in a DeviceOutputBatchSE and process
    if (output_file)
    {
        timer.start();

        io::DeviceOutputBatchSE gpu_batch(
            count,
            best_data_dvec,
            io::DeviceCigarArray(cigar, cigar_coords_dvec),
            mds,
            mapq_dvec);

        cpu_batch.readback( gpu_batch );

        timer.stop();
        stats.alignments_DtoH.add( count, timer.seconds() );

        timer.start();

        log_debug(stderr, "[%u]     output\n", ID);
        output_file->process( cpu_batch );

        timer.stop();
        stats.io.add( count, timer.seconds() );
    }

    // TODO: decide whether to output second best alignments
    #if 0
    {
        // clear mapq's
        thrust::fill( mapq_dvec.begin(), mapq_dvec.begin() + count, uint8(255) );

        // overlap the second-best indices with the loc queue
        thrust::device_vector<uint32>& second_idx_dvec = scoring_queues.hits.loc;

        // compact the indices of the second-best alignments
        const uint32 n_second = uint32( thrust::copy_if(
            thrust::make_counting_iterator(0u),
            thrust::make_counting_iterator(0u) + count,
            best_data_dvec.begin(),
            second_idx_dvec.begin(),
            io::has_second() ) - second_idx_dvec.begin() ); // FIXME: this has to be fixed since best_data_dvec is now
                                                            // a strided vector of io::Alignment's rather than a vector of
                                                            // io::BestAlignment's

        //
        // perform backtracking and compute cigars for the second-best alignments
        //
        if (n_second)
        {
            // initialize cigars & MDS pools
            cigar.clear();
            mds.clear();

            timer.start();
            device_timer.start();

            uint32* second_idx = thrust::raw_pointer_cast( &second_idx_dvec[0] );

            NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]     second-best backtrack (%u)\n", ID, n_second) );
            banded_traceback_best(
                1u,
                n_second,
                second_idx,
                best_data_dptr,
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

            NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]     second-best alignment (%u)\n", ID, n_second) );
            finish_alignment_best(
                1u,
                n_second,
                second_idx,
                best_data_dptr,
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

        // wrap the results in a DeviceOutputBatchSE and process them
        if (output_file)
        {
            timer.start();

            io::DeviceOutputBatchSE gpu_batch(
                count,
                best_data_dvec,
                io::DeviceCigarArray(cigar, cigar_coords_dvec),
                mds,
                mapq_dvec);

            cpu_batch.readback( gpu_batch );

            timer.stop();
            stats.alignments_DtoH.add( count, timer.seconds() );

            timer.start();

            output_file->process( cpu_batch );

            timer.stop();
            stats.io.add( count, timer.seconds() );
        }
    }
    #endif

    // keep alignment stats
    {
        log_debug(stderr, "[%u]    track stats\n", ID);
        thrust::host_vector<io::Alignment> h_best_data( BATCH_SIZE*2 );
        thrust::host_vector<uint8>         h_mapq( BATCH_SIZE );

        thrust::copy( best_data_dvec.begin(),              best_data_dvec.begin() + count,              h_best_data.begin() );
        thrust::copy( best_data_dvec.begin() + BATCH_SIZE, best_data_dvec.begin() + count + BATCH_SIZE, h_best_data.begin() + BATCH_SIZE );
        thrust::copy( mapq_dvec.begin(),                   mapq_dvec.begin()      + count,              h_mapq.begin() );

        for (uint32 i = 0; i < count; ++i)
        {
            const io::BestAlignments best( h_best_data[i], h_best_data[i + BATCH_SIZE] );
            const uint8 mapq = h_mapq[i];

            stats.track_alignment_statistics( &stats.mate1, best, mapq );
        }
    }
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
    const io::SequenceDataDevice&           read_data,
    const uint32                            seeding_pass,
    const uint32                            seed_queue_size,
    const uint32*                           seed_queue,
    Stats&                                  stats)
{
    // prepare the scoring system
    typedef typename scoring_scheme_type::threshold_score_type          threshold_score_type;

    //threshold_score_type threshold_score = scoring_scheme.threshold_score();
    const int32 score_limit = scoring_scheme_type::worst_score;

    Timer timer;
    Timer global_timer;
    nvbio::cuda::Timer device_timer;

//    const uint32 count    = read_data.size();
    const uint32 band_len = band_length( params.max_dist );

    // cast the reads to use proper iterators
    const read_view_type  reads_view = plain_view( read_data );
    const read_batch_type reads( reads_view );

    // cast the genome to use proper iterators
    const genome_view_type          genome_view( plain_view( reference_data ) );
    const genome_access_type        genome_access( genome_view );
    const uint32                    genome_len = genome_access.bps();
    const genome_iterator           genome     = genome_access.sequence_stream();

    NVBIO_VAR_UNUSED thrust::device_vector<uint32>::iterator         hit_read_id_iterator = scoring_queues.hits.read_id.begin();
    NVBIO_VAR_UNUSED thrust::device_vector<uint32>::iterator         loc_queue_iterator   = scoring_queues.hits.loc.begin();
    NVBIO_VAR_UNUSED thrust::device_vector<int32>::iterator          score_queue_iterator = scoring_queues.hits.score.begin();

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
        0u,
        reads,
        reads,
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
        log_debug(stderr, "[%u]     pass:\n[%u]       batch:          %u\n[%u]       seeding pass:   %d\n[%u]       extension pass: %u\n", ID, ID, batch_number, ID, seeding_pass, ID, extension_pass);

        // check if we need to persist this seeding pass
        if (params.persist_batch     == -1 || batch_number == (uint32) params.persist_batch &&
            params.persist_seeding   == -1 || seeding_pass == (uint32) params.persist_seeding &&
            params.persist_extension == -1 || extension_pass == (uint32) params.persist_extension)
            persist_hits( params.persist_file, "hits", 0u, reads.size(), hit_deques );

        // initialize all the scoring output queues
        scoring_queues.clear_output();

        // sort the active read infos
        #if 0
        {
            timer.start();
            device_timer.start();

            NVBIO_CUDA_DEBUG_STATEMENT( log_debug(stderr, "[%u]     read sort\n", ID) );
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
            // This adds some book-keeping overheads, but allows to make larger
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

        log_debug(stderr, "[%u]     select (%u active reads)\n", ID, active_read_queues.in_size);
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
        if (params.persist_batch     == -1 || batch_number   == (uint32) params.persist_batch &&
            params.persist_seeding   == -1 || seeding_pass   == (uint32) params.persist_seeding &&
            params.persist_extension == -1 || extension_pass == (uint32) params.persist_extension)
            persist_selection( params.persist_file, "selection",
                0u,
                active_read_queues.in_size,
                active_read_queues.raw_input_queue(),
                pipeline.n_hits_per_read,
                pipeline.hits_queue_size,
                scoring_queues.hits_index,
                scoring_queues.hits );

        log_debug(stderr, "[%u]     selected %u hits\n", ID, pipeline.hits_queue_size);
        NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "[%u]       crc: %llu\n", ID, device_checksum( hit_read_id_iterator, hit_read_id_iterator + pipeline.hits_queue_size ) ) );
        NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "[%u]       crc: %llu\n", ID, device_checksum( loc_queue_iterator, loc_queue_iterator + pipeline.hits_queue_size ) ) );

        timer.start();
        device_timer.start();

        // sort the selected hits by their SA coordinate
        log_debug(stderr, "[%u]     locate sort\n", ID);
        pipeline.idx_queue = sort_hi_bits( pipeline.hits_queue_size, pipeline.scoring_queues.hits.loc );

        device_timer.stop();
        timer.stop();
        stats.sort.add( pipeline.hits_queue_size, timer.seconds(), device_timer.seconds() );

        timer.start();
        device_timer.start();

        // NOTE: only 75-80% of these locations are unique.
        // It might pay off to do a compaction beforehand.

        // and locate their position in linear coordinates
        log_debug(stderr, "[%u]     locate init\n", ID);
        locate_init( pipeline, params );

        optional_device_synchronize();

        log_debug(stderr, "[%u]     locate lookup\n", ID);
        locate_lookup( pipeline, params );

        optional_device_synchronize();
        nvbio::cuda::check_error("locating kernel");

        device_timer.stop();
        timer.stop();
        stats.locate.add( pipeline.hits_queue_size, timer.seconds(), device_timer.seconds() );

        NVBIO_CUDA_DEBUG_STATEMENT( log_debug( stderr, "[%u]       crc: %llu\n", ID, device_checksum( loc_queue_iterator, loc_queue_iterator + pipeline.hits_queue_size ) ) );

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

        score_best(
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

        // check if we need to persist this seeding pass
        if (params.persist_batch     == -1 || batch_number   == (uint32) params.persist_batch &&
            params.persist_seeding   == -1 || seeding_pass   == (uint32) params.persist_seeding &&
            params.persist_extension == -1 || extension_pass == (uint32) params.persist_extension)
            persist_scores( params.persist_file, "scores", 0u, reads.size(), pipeline.n_hits_per_read, pipeline.hits_queue_size, scoring_queues );

        timer.start();
        device_timer.start();

        const ReduceBestApproxContext reduce_context( pipeline.trys, n_ext );

        // reduce the multiple scores to find the best two alignments
        // (one thread per active read).
        log_debug(stderr, "[%u]     score reduce\n", ID);
        score_reduce(
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
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
