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

namespace nvbio {
namespace bowtie2 {
namespace cuda {

template <
    typename FMI,
    typename rFMI,
    typename scoring_scheme_type>
void Aligner::best_exact(
    const Params&               params,
    const FMI                    fmi,
    const rFMI                  rfmi,
    const scoring_scheme_type&  scoring_scheme,
    const io::FMIndexDataCUDA&  driver_data,
    io::ReadDataCUDA&           read_data,
    const uint32                output_set,
    Stats&                      stats)
{
    typedef uint32  read_storage_type;
    typedef uint4   read_storage_type4;
    typedef READ_TEX_SELECTOR( const uint32*, nvbio::cuda::ldg_pointer<uint32> )  read_base_type;
    typedef READ_TEX_SELECTOR( const uint4*,  nvbio::cuda::ldg_pointer<uint4> )   read_base_type4;

    typedef read_storage_type read_storage_type_A;
    typedef read_base_type    read_base_type_A;

  #if USE_UINT4_PACKING
    typedef read_storage_type4 read_storage_type_B;
    typedef read_base_type4    read_base_type_B;
  #else
    typedef read_storage_type  read_storage_type_B;
    typedef read_base_type     read_base_type_B;
  #endif

    Timer timer;
    Timer global_timer;

    const uint32 count = read_data.size();
    const uint32 band_len = band_length( params.max_dist );

    //// make sure the current output set is not currently being flushed
    //ScopedLock lock( &output_thread.m_lock[ output_set ] );
    thrust::copy(
        thrust::make_counting_iterator(0u),
        thrust::make_counting_iterator(0u) + count,
        seed_queue_dvec.begin() );

    seed_queue_size_dvec[0] = count;

    //
    // Similarly to Bowtie2, we perform a number of seed & extension passes, except
    // that here we extract just one seed per pass.
    // Since reads may have different length, and the search might for some might terminate
    // earlier than for others, the number of reads actively processed in each pass can vary
    // substantially.
    // In order to keep the cores and all its lanes busy, we use a pair of input & output
    // queues to compact the set of active reads in each round, swapping them at each
    // iteration.
    //

    // initialize seeding queues
    HostQueues<uint32> seed_queues;
    seed_queues.in_size   = count;
    seed_queues.in_queue  = seed_queue_dptr;
    seed_queues.out_size  = NULL; // for mapping, we initially setup a NULL output queue
    seed_queues.out_queue = NULL; // for mapping, we initially setup a NULL output queue

    // initialize the seed hit counts
    thrust::fill( cnts_dvec.begin(), cnts_dvec.end(), uint32(0) );

    Batch<read_base_type_A,const char*> reads_A(
        0, count, 
        read_base_type_A( (const read_storage_type_A*)read_data.read_stream() ),
        read_data.read_index(),
        read_data.qual_stream());

    //
    // perform mapping
    //
    {
        timer.start();

    #if 0
        bowtie2_map_exact(
            reads_A, fmi, rfmi,
            0u, seed_queues.device(),
            hits_dptr, cnts_dptr,
            params );
    #else
        bowtie2_map_exact(
            reads_A, fmi, rfmi,
            hits_dptr, cnts_dptr,
            params );
    #endif
        cudaThreadSynchronize();
        nvbio::cuda::check_error("mapping kernel");

        timer.stop();
        stats.map.add( seed_queues.in_size, timer.seconds() );
    }

    // take some stats on the hits we got
    if (params.keep_stats)
        keep_stats( reads_A.size(), stats );

    // setup output queue
    seed_queues.out_size  = seed_queue_size_dptr;
    seed_queues.out_queue = seed_queue_dptr + BATCH_SIZE;

    const uint32 max_seeds = read_data.m_max_read_len / params.seed_len;

    thrust::device_vector<SeedHit> hits_level_dvec( BATCH_SIZE );
    thrust::device_vector<uint32>  cnts_level_dvec( BATCH_SIZE );

    for (uint32 seed_idx = 0; seed_queues.in_size && seed_idx < max_seeds; ++seed_idx)
    {
        {
            const uint32 blocks = (read_data.size() + BLOCKDIM-1) / BLOCKDIM;

            thrust::fill( hits_stats_dvec.begin(), hits_stats_dvec.end(), 0u );
            bowtie2_hits_stats( read_data.size(), hits_dptr, cnts_dptr, hits_stats_dptr );

            cudaThreadSynchronize();
            nvbio::cuda::check_error("hit stats kernel");

            cudaThreadSynchronize();
            nvbio::cuda::check_error("hit stats kernel");

            hits_stats_hvec = hits_stats_dvec;

            const uint64 n_hits = hits_stats_hvec[ HIT_STATS_TOTAL ];
            fprintf(stderr, "\nseed idx: %u (%u active reads - %.2f M global hits)\n", seed_idx, seed_queues.in_size, float(n_hits) * 1.0e-6f);
        }

        extract_top_range(
            reads_A, hits_dptr, cnts_dptr,
            thrust::raw_pointer_cast( &hits_level_dvec.front() ),
            thrust::raw_pointer_cast( &cnts_level_dvec.front() ) );

        cudaThreadSynchronize();
        nvbio::cuda::check_error("extract-top-range kernel");

        best_exact_score(
            params,
            fmi,
            rfmi,
            scoring_scheme,
            driver_data,
            read_data,
            seed_idx,
            seed_queues,
            thrust::raw_pointer_cast( &hits_level_dvec.front() ),
            thrust::raw_pointer_cast( &cnts_level_dvec.front() ),
            stats );

        // initialize output seeding queue size
        seed_queues.clear_output();

        bowtie2_prune_search(
            seed_idx,
            seed_queues.device(),
            best_data_dptr );

        cudaThreadSynchronize();
        nvbio::cuda::check_error("pruning kernel");

        // swap input & output queues
        seed_queues.swap();
    }

    //
    // At this point, for each read we have the scores and rough alignment positions of the
    // best two alignments: to compute the final results we need to backtrack the DP extension,
    // and compute accessory CIGARS and MD strings.
    //

    Batch<read_base_type_B,const char*> reads_B(
        0, count, 
        read_base_type_B( (const read_storage_type_B*)read_data.read_stream() ),
        read_data.read_index(),
        read_data.qual_stream());

    //
    // perform backtracking and compute cigars for the best alignments
    //
    timer.start();
    {
        BacktrackBestContext<0,edit_distance_scoring> context(
            best_data_dptr,
            NULL );

        bowtie2_backtrack(
            read_data.max_read_len(),
            band_len,
            count,
            reads_B,
            scoring_scheme,
            driver_data.genome_length(),
            driver_data.genome_stream(),
            cigar_dptr,
            cigar_coords_dptr,
            mds_dptr,
            count,
            context,
            params );

        cudaThreadSynchronize();
        nvbio::cuda::check_error("backtracking kernel");
    }
    timer.stop();
    stats.backtrack.add( count, timer.seconds() );

    // copy best cigars back to the host
    cigar_coords_hvec1[output_set] = cigar_coords_dvec;
    nvbio::cuda::copy( cigar_dvec, cigar_hvec1[output_set] );
    nvbio::cuda::copy( mds_dvec,   mds_hvec1[output_set] );

    // overlap the second-best indices with the loc queue
    thrust::device_vector<uint32>& second_idx_dvec = loc_queue_dvec;

    // compact the indices of the second-best alignments
    const uint32 n_second = uint32( thrust::copy_if(
        thrust::make_counting_iterator(0u),
        thrust::make_counting_iterator(0u) + count,
        best_data_dvec.begin(),
        second_idx_dvec.begin(),
        has_second() ) - second_idx_dvec.begin() );

    //
    // perform backtracking and compute cigars for the second-best alignments
    //
    if (n_second)
    {
        timer.start();

        uint32* second_idx = thrust::raw_pointer_cast( &second_idx_dvec[0] );

        BacktrackBestContext<1,edit_distance_scoring> context(
            best_data_dptr,
            second_idx );

        bowtie2_backtrack(
            read_data.max_read_len(),
            band_len,
            n_second,
            reads_B,
            scoring_scheme,
            driver_data.genome_length(),
            driver_data.genome_stream(),
            cigar_dptr,
            cigar_coords_dptr,
            mds_dptr,
            count,
            context,
            params );

        cudaThreadSynchronize();
        nvbio::cuda::check_error("second-best backtracking kernel");

        timer.stop();
        stats.backtrack.add( n_second, timer.seconds() );
    }

    // copy second-best cigars back to the host
    cigar_coords_hvec2[output_set] = cigar_coords_dvec;
    nvbio::cuda::copy( cigar_dvec, cigar_hvec2[output_set] );
    nvbio::cuda::copy( mds_dvec,   mds_hvec2[output_set] );

    // copy results back to the host
    thrust::copy( best_data_dvec.begin(), best_data_dvec.begin() + count, best_data_hvec[output_set].begin() );
}

template <
    typename FMI,
    typename rFMI,
    typename scoring_scheme_type>
void Aligner::best_exact_score(
    const Params&               params,
    const FMI                    fmi,
    const rFMI                  rfmi,
    const scoring_scheme_type&  scoring_scheme,
    const io::FMIndexDataCUDA&  driver_data,
    io::ReadDataCUDA&           read_data,
    const uint32                seed_idx,
    HostQueues<uint32>&         seed_queues,
    SeedHit*                    hits_level_dptr,
    uint32*                     cnts_level_dptr,
    Stats&                      stats)
{
  #if USE_UINT4_PACKING
    typedef uint32 read_storage_type;
  #else
    typedef uint4  read_storage_type;
  #endif
    typedef READ_TEX_SELECTOR( const read_storage_type*, nvbio::cuda::ldg_pointer<read_storage_type> )  read_base_type;

    uint32 hits_to_process;
    {
        thrust::fill( hits_stats_dvec.begin(), hits_stats_dvec.end(), 0u );
        bowtie2_hits_stats( read_data.size(), hits_dptr, cnts_dptr, hits_stats_dptr );

        cudaThreadSynchronize();
        nvbio::cuda::check_error("hit stats kernel");

        hits_stats_hvec = hits_stats_dvec;

        hits_to_process = hits_stats_hvec[ HIT_STATS_TOTAL ];
    }

    Timer timer;
    Timer global_timer;

    const uint32 count = read_data.size();
    const uint32 band_len = band_length( seed_idx );

    Batch<read_base_type,const char*> reads(
        0, count, 
        read_base_type( (const read_storage_type*)read_data.read_stream() ),
        read_data.read_index(),
        read_data.qual_stream());

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
    HostQueues<uint32> hit_queues;
    hit_queues.in_size   = seed_queues.in_size;
    hit_queues.in_queue  = hits_queue_dptr;
    hit_queues.out_size  = hits_queue_size_dptr;
    hit_queues.out_queue = hits_queue_dptr + BATCH_SIZE;

    cudaMemcpy( hit_queues.in_queue, seed_queues.in_queue, sizeof(uint32) * hit_queues.in_size, cudaMemcpyDeviceToDevice );

    bool first_run = true;

    SelectBestExactContext select_context(
        best_data_dptr,
        params.max_dist,
        seed_idx,
        seed_idx == 0 ? true : false ); // init

    // use the score queue to store temporary packed seed data
    uint32* seed_data_dptr = (uint32*)score_queue_dptr;

    uint32 processed_hits = 0;

    while (hit_queues.in_size)
    {
        // sort the input queue
        thrust::sort(
            hits_queue_dvec.begin() + (hit_queues.in_queue - hits_queue_dptr),
            hits_queue_dvec.begin() + (hit_queues.in_queue - hits_queue_dptr) + hit_queues.in_size );

        fprintf(stderr,"\rcount: %u (%.2f / %.2f M hits processed)    ", hit_queues.in_size, float(processed_hits) * 1.0e-6f, float(hits_to_process) * 1.0e-6f);
        hit_queues.clear_output();

        /*if (hit_queues.in_size < read_data.size() / 500)
        {
            // we can safely ignore this small set of active reads, as we'll drop only
            // a given amount of sensitivity
            return;
        }
        else*/ if (hit_queues.in_size > 128u*1024u)
        {
            //
            // The queue of actively processed reads is sufficiently large to allow
            // selecting & scoring one seed hit at a time and still have large kernel
            // launches. This is algorithmically the most efficient choice (because
            // it enables frequent early outs), so let's choose it.
            //
            timer.start();

            //
            // select and locate the next round of hits for the reads in the input queue
            //
            bowtie2_select(
                first_run,
                reads, fmi, rfmi, hits_level_dptr, cnts_level_dptr,
                select_context,
                hit_queues.in_size,
                hit_queues.in_queue,
                hit_queues.out_size,
                hit_queues.out_queue,
                loc_queue_dptr,
                seed_data_dptr,
                params );

            first_run = false;

            cudaThreadSynchronize();
            nvbio::cuda::check_error("selecting kernel");

            timer.stop();
            stats.select.add( hit_queues.in_size, timer.seconds() );

            // fetch the new queue size
            hit_queues.swap();
            if (hit_queues.in_size == 0)
                break;

            processed_hits += hit_queues.in_size;

            timer.start();

            // sort the selected hits by their SA coordinate
            thrust::copy(
                thrust::make_counting_iterator(0u),
                thrust::make_counting_iterator(0u) + hit_queues.in_size,
                idx_queue_dvec.begin() );

            thrust::sort_by_key(
                loc_queue_dvec.begin(),
                loc_queue_dvec.begin() + hit_queues.in_size,
                idx_queue_dvec.begin() );

            timer.stop();
            stats.sort.add( hit_queues.in_size, timer.seconds() );

            // NOTE: only 75-80% of these locations are unique.
            // It might pay off to do a compaction beforehand.

            // and locate their position in linear coordinates
            bowtie2_locate(
                reads, fmi, rfmi,
                hit_queues.in_size,
                loc_queue_dptr,
                idx_queue_dptr,
                seed_data_dptr,
                params );

            cudaThreadSynchronize();
            nvbio::cuda::check_error("locating kernel");

            timer.stop();
            stats.locate.add( hit_queues.in_size, timer.seconds() );

            timer.start();

            // sort the selected hits by their linear genome coordinate
            // TODO: sub-sort by read position/RC flag so as to (1) get better coherence,
            // (2) allow removing duplicate extensions
            thrust::sort_by_key(
                loc_queue_dvec.begin(),
                loc_queue_dvec.begin() + hit_queues.in_size,
                idx_queue_dvec.begin() );

            timer.stop();
            stats.sort.add( hit_queues.in_size, timer.seconds() );

            //
            // assign a score to all selected hits (currently in the output queue)
            //
            if (hit_queues.in_size)
            {
                timer.start();

                ScoreBestContext<edit_distance_scoring> context(
                    NULL,
                    hit_queues.in_queue,
                    score_queue_dptr,
                    best_data_dptr,
                    seed_idx );

                // compute the score for each selected hit (one thread per hit).
                bowtie2_score(
                    band_len,
                    reads,
                    context,
                    hit_queues.in_size,
                    idx_queue_dptr,
                    loc_queue_dptr,
                    driver_data.genome_length(),
                    driver_data.genome_stream(),
                    params );

                cudaThreadSynchronize();
                nvbio::cuda::check_error("score kernel");

                const ReduceBestExactContext reduce_context;

                bowtie2_score_reduce<edit_distance_scoring>(
                    reads, reduce_context, hits_level_dptr, cnts_level_dptr,
                    best_data_dptr,
                    hit_queues.in_size,
                    hit_queues.in_queue,
                    loc_queue_dptr,
                    score_queue_dptr,
                    params );

                cudaThreadSynchronize();
                nvbio::cuda::check_error("score-reduce kernel");

                timer.stop();
                stats.score.add( hit_queues.in_size, timer.seconds() );
            }
        }
        else
        {
            //
            // The queue of actively processed reads is very small: at this point
            // it's better to select multiple hits to process in each round.
            // This adds some book keeping overheads, but allows to make larger
            // kernel launches.
            //
            timer.start();

            score_output_count_dvec[0] = 0;

            // the maximum number of extensions we can perform in one iteration
            // is at most 4096, as we use 12 bits to encode the extension index
            const uint32 max_ext = 4096;

            // try to generate 256K items to process
            const uint32 n_multi =
                std::min(
                    (256u*1024u) / hit_queues.in_size,
                    max_ext );

            const uint32 score_output_queue_stride = hit_queues.in_size;

            bowtie2_select_multi(
                first_run,
                reads, fmi, rfmi,  hits_level_dptr, cnts_level_dptr,
                select_context,
                hit_queues.in_size,
                hit_queues.in_queue,
                hit_queues.out_size,
                hit_queues.out_queue,
                score_output_count_dptr,
                loc_queue_dptr,
                seed_data_dptr,
                score_parent_queue_dptr,
                score_output_queue_dptr,
                score_output_queue_stride,
                n_multi,
                params );

            first_run = false;

            cudaThreadSynchronize();
            nvbio::cuda::check_error("select-multi kernel");

            timer.stop();
            stats.select.add( hit_queues.in_size * n_multi, timer.seconds() );

            // swap the input & output queues
            hit_queues.swap();

            // fetch the output queue size
            const uint32 score_queue_size = score_output_count_dvec[0];
            if (score_queue_size == 0)
                break;

            processed_hits += score_queue_size;

            timer.start();

            // sort the selected hits by their SA coordinate
            thrust::copy(
                thrust::make_counting_iterator(0u),
                thrust::make_counting_iterator(0u) + score_queue_size,
                idx_queue_dvec.begin() );

            thrust::sort_by_key(
                loc_queue_dvec.begin(),
                loc_queue_dvec.begin() + score_queue_size,
                idx_queue_dvec.begin() );

            timer.stop();
            stats.sort.add( score_queue_size, timer.seconds() );

            // and locate their position in linear coordinates
            bowtie2_locate(
                reads, fmi, rfmi,
                score_queue_size,
                loc_queue_dptr,
                idx_queue_dptr,
                seed_data_dptr,
                params );

            cudaThreadSynchronize();
            nvbio::cuda::check_error("locating kernel");

            timer.stop();
            stats.locate.add( score_queue_size, timer.seconds() );

            timer.start();

            // sort the selected hits by their linear genome coordinate
            // TODO: sub-sort by read_id/RC flag so as to (1) get better coherence,
            // (2) allow removing duplicate extensions
            thrust::sort_by_key(
                loc_queue_dvec.begin(),
                loc_queue_dvec.begin() + score_queue_size,
                idx_queue_dvec.begin() );

            timer.stop();
            stats.sort.add( hit_queues.in_size, timer.seconds() );

            //
            // assign a score to all selected hits (currently in the output queue)
            //
            if (score_queue_size)
            {
                timer.start();

                ScoreBestContext<edit_distance_scoring> context(
                    score_parent_queue_dptr,
                    hit_queues.in_queue,
                    score_queue_dptr,
                    best_data_dptr,
                    seed_idx );

                // compute the score for each selected hit (one thread per hit).
                bowtie2_score(
                    band_len,
                    reads,
                    context,
                    score_queue_size,
                    idx_queue_dptr,
                    loc_queue_dptr,
                    driver_data.genome_length(),
                    driver_data.genome_stream(),
                    params );

                cudaThreadSynchronize();
                nvbio::cuda::check_error("score-multi kernel");

                const ReduceBestExactContext reduce_context;

                // reduce the multiple scores to find the best two alignments
                // (one thread per active read).
                bowtie2_score_multi_reduce<edit_distance_scoring>(
                    reads, reduce_context, hits_level_dptr, cnts_level_dptr,
                    best_data_dptr,
                    hit_queues.in_size,
                    hit_queues.in_queue,
                    score_output_queue_stride,
                    score_output_queue_dptr,
                    score_parent_queue_dptr,
                    score_queue_dptr,
                    loc_queue_dptr,
                    params );

                cudaThreadSynchronize();
                nvbio::cuda::check_error("score-multi-reduce kernel");

                timer.stop();
                stats.score.add( score_queue_size, timer.seconds() );
            }
        }
    }
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
