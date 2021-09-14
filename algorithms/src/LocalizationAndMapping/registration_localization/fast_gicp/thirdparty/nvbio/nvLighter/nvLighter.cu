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

// nvLighter.cu
//

//#define NVBIO_CUDA_DEBUG

#include <cub/cub.cuh>
#include <zlib/zlib.h>
#include <nvbio/basic/omp.h>

#include "bloom_filters.h"
#include "input_thread.h"
#include "output_thread.h"
#include "sample_kmers.h"
#include "error_correct.h"
#include <nvbio/basic/pipeline.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/bloom_filter.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/shared_pointer.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/system.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/io/sequence/sequence.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

using namespace nvbio;


void cumulative_binomial_distribution(double F[], uint32 l, double p)
{
    // p is the probability of getting 1
    double coef = 1 ;
    double exp  = pow( 1 - p, l );

    F[0] = pow( 1 - p, l );
    for (uint32 i = 1 ; i <= l ; ++i)
    {
        coef = coef / i * (l - i + 1);
        exp  = exp / (1 - p) * p;
        F[i] = F[i-1] + coef * exp;
    }
}

char get_bad_quality(nvbio::io::SequenceDataStream* reads_file)
{
    int i;
    int histogram1[300], histogram2[300];

    // fetch some reads
    io::SequenceDataHost reads;

    memset( histogram1, 0, sizeof( histogram1 )) ;
    memset( histogram2, 0, sizeof( histogram2 ) ) ;

    int n_reads = 0;

    for (int batch = 0 ; batch < 100; ++batch)
    {
        const int ret = nvbio::io::next( ASCII, &reads, reads_file, 10000, 10000000 );
        if (ret == 0)
            break;

        const nvbio::io::SequenceDataAccess<ASCII> reads_view( reads );

        typedef nvbio::io::SequenceDataAccess<ASCII>::qual_string qual_string;

        for (uint32 i = 0; i < reads.size(); ++i)
        {
            const qual_string quals = reads_view.get_quals(i);

            ++histogram2[ (int)quals[ quals.length() - 1 ] ];
            ++histogram1[ (int)quals[0] ];
        }

        n_reads += reads.size();
    }

    // rewind the file
    if (reads_file->rewind() == false)
    {
        log_error(stderr, "    failed rewinding reads file\n");
        return 1;
    }

    int cnt = 0;
    for (i = 0 ; i < 300; ++i)
    {
        cnt += histogram1[i];
        if (cnt > n_reads * 0.05f)
            break;
    }
    const int t1 = i - 1;
    cnt = 0;
    for (i = 0; i < 300 ; ++i)
    {
        cnt += histogram2[i];
        if (cnt > n_reads * 0.05f)
            break;
    }
    const int t2 = i;
    return (char)nvbio::min( t1, t2 );
}

float infer_alpha(nvbio::io::SequenceDataStream* reads_file, const uint64 genome_size)
{
    log_info(stderr, "  inferring alpha... started\n" );

    nvbio::io::SequenceDataHost reads;

    uint64 n_reads = 0;
    uint64 n_bps   = 0;
    float  time    = 0.0f;

    while (1)
    {
        Timer timer;
        timer.start();

        const int ret = nvbio::io::next( ASCII, &reads, reads_file, 512*1024, 128*1024*1024 );
        if (ret == 0)
            break;

        timer.stop();
        time += timer.seconds();

        log_verbose(stderr, "\r    input: %llu reads, %.2f%c bps (%.1f Mbps/s)          ",
            n_reads,      n_bps >= 1.0e9f ? 'B' : 'M',
            float(n_bps)*(n_bps >= 1.0e-9f ? 1.0e-9f : 1.0e-6f),
            1.0e-6f * float(n_bps)/time );

        n_reads += reads.size();
        n_bps   += reads.bps();
    }
    log_verbose_cont(stderr, "\n");

    const float coverage = float( double( n_bps ) / double( genome_size ) );
    const float alpha = 7.0f / coverage;

    log_info(stderr, "  inferring alpha... done\n" );
    log_stats(stderr, "    input: %llu reads, %.2f%c bps, %.3fx coverage\n",
        n_reads,      n_bps >= 1.0e9f ? 'B' : 'M',
        float(n_bps)*(n_bps >= 1.0e-9f ? 1.0e-9f : 1.0e-6f),
        coverage );

    log_visible(stderr, "    inferred alpha: %f\n", alpha );

    // rewind the file
    if (reads_file->rewind() == false)
    {
        log_error(stderr, "    failed rewinding reads file\n");
        return 1;
    }

    return alpha;
}

int main(int argc, char* argv[])
{
    if ((argc < 3) || (strcmp( argv[1], "--help" ) == 0))
    {
        log_visible(stderr, "nvLighter - Copyright 2015, NVIDIA Corporation\n");
        log_info(stderr, "usage:\n");
        log_info(stderr, "  nvLighter [options] input_file output_file\n");
        log_info(stderr, "  options:\n");
        log_info(stderr, "   -v         int (0-6) [5]                # verbosity level\n");
        log_info(stderr, "   -zlib      string    [1R]               # e.g. \"1\", ..., \"9\", \"1R\"\n");
        log_info(stderr, "   -t         int       [auto]             # number of CPU threads\n");
        log_info(stderr, "   -d         int       [0]                # add the specified GPU device\n");
        log_info(stderr, "   -k         k-mer genome-size alpha      # error correction parameters\n");
        log_info(stderr, "   -K         k-mer genome-size            # error correction parameters\n");
        log_info(stderr, "   -maxcor    int       [4]                # maximum correction factor\n");
        log_info(stderr, "   -newQual   int       [disabled]         # new quality score value\n");
        log_info(stderr, "   -no-cpu                                 # disable CPU usage\n");
        log_info(stderr, "   -no-gpu                                 # disable GPU usage\n");
        return 0;
    }

    const char* reads_name        = argv[argc-2];
    const char* output_name       = argv[argc-1];
    const char* comp_level        = "1R";
    io::QualityEncoding qencoding = io::Phred;
    int   threads                 = 0;
    uint32 k                      = 11u;
    uint64 genome_size            = 0;
    float  alpha                  = 0.0;
    float  max_correction         = 4.0f;
    char   new_quality            = 0;
    float  bf_factor              = 1.0f; // original: 1.5
    bool   cpu                    = true;
    bool   gpu                    = true;

    std::vector<int> devices(0);

    for (int i = 0; i < argc - 2; ++i)
    {
        if ((strcmp( argv[i], "-v" )             == 0) ||
            (strcmp( argv[i], "-verbosity" )     == 0) ||
            (strcmp( argv[i], "--verbosity" )    == 0))
        {
            set_verbosity( Verbosity( atoi( argv[++i] ) ) );
        }
        else if ((strcmp( argv[i], "-c" )             == 0) ||
                 (strcmp( argv[i], "--compression" )  == 0) ||
                 (strcmp( argv[i], "-zlib" )          == 0))  // setup compression level
        {
            comp_level = argv[++i];
        }
        else if ((strcmp( argv[i], "-t" )             == 0) ||
                 (strcmp( argv[i], "-threads" )       == 0) ||
                 (strcmp( argv[i], "--threads" )      == 0))  // setup number of threads
        {
            threads = atoi( argv[++i] );
        }
        else if ((strcmp( argv[i], "-d" )             == 0) ||
                 (strcmp( argv[i], "-device" )        == 0) ||
                 (strcmp( argv[i], "--device" )       == 0))  // add a device
        {
            devices.push_back( atoi( argv[++i] ) );
        }
        else if ((strcmp( argv[i], "-no-cpu" )        == 0) ||  // remove CPU
                 (strcmp( argv[i], "--no-cpu" )       == 0))
            cpu = false;
        else if ((strcmp( argv[i], "-no-gpu" )        == 0) ||  // remove GPU
                 (strcmp( argv[i], "--no-gpu" )       == 0))
            gpu = false;
        else if (strcmp( argv[i], "-k" )              == 0)  // setup kmer length, genome size and sampling frequency
        {
            k           = atoi( argv[++i] );
            genome_size = atol( argv[++i] );
            alpha       = atof( argv[++i] );
        }
        else if (strcmp( argv[i], "-K" )              == 0)  // setup kmer length, genome size and sampling frequency
        {
            k           = atoi( argv[++i] );
            genome_size = atol( argv[++i] );
            alpha       = 0.0f;
        }
        else if (strcmp( argv[i], "-maxcor" )         == 0)  // setup max correction factor
            max_correction = (float)atoi( argv[++i] );
        else if (strcmp( argv[i], "-newQual" )        == 0)  // setup new quality
            new_quality = argv[++i][0];
        else if (strcmp( argv[i], "-bf" )             == 0)  // Bloom filter expansion factor
            bf_factor = atof( argv[++i] );
    }

    // if no devices were specified, and the gpu is enabled, pick GPU 0
    if (gpu && (devices.size() == 0))
        devices.push_back(0);

    uint32 device_count = uint32( devices.size() );

    // check whether the genome size has been specified
    if (genome_size == 0u)
    {
        log_error(stderr, "must specify the k-mer and genome size with the option: -k k genome-size alpha\n" );
        return 1;
    }

    try
    {
        log_visible(stderr,"nvLighter... started\n");

        // compute the optimal Bloom filter size scaling factors
        const float sampled_kmers_bf_factor = optimal_bloom_filter_bits_per_key( 0.01f );
        const float trusted_kmers_bf_factor = optimal_bloom_filter_bits_per_key( 0.0005f );

        log_verbose(stderr, "  optimal m(0.01f) = %.2f\n", sampled_kmers_bf_factor );
        log_verbose(stderr, "  optimal k(0.01f) = %u\n", optimal_bloom_filter_hashes( sampled_kmers_bf_factor ) );

        log_verbose(stderr, "  optimal m(0.0005f) = %.2f\n", trusted_kmers_bf_factor );
        log_verbose(stderr, "  optimal k(0.0005f) = %u\n", optimal_bloom_filter_hashes( trusted_kmers_bf_factor ) );

        // compute the Bloom filter sizes, in words
        const uint64 sampled_kmers_bf_words = align<8u>( uint64( float(genome_size) * bf_factor * (sampled_kmers_bf_factor / 32.0f) ) );
        const uint64 trusted_kmers_bf_words = align<8u>( uint64( float(genome_size) * bf_factor * (trusted_kmers_bf_factor / 32.0f) ) );

        const uint32 bits_per_word = 32u;

        // now set the number of CPU threads
        threads = threads > 0 ? threads : omp_get_num_procs();

        //omp_set_num_threads( threads );
        omp_set_num_threads( omp_get_num_procs() ); // use all threads for the merging steps...
        omp_set_nested(1);

        // setup the device Bloom filters
        BloomFilters<host_tag>   h_bloom_filters;
        BloomFilters<host_tag>*  h_bloom_filters_ptr = &h_bloom_filters;
        BloomFilters<device_tag> d_bloom_filters[16];

        for (int i = device_count-1; i >= 0; --i)
        {
            if (d_bloom_filters[i].setup( devices[i], sampled_kmers_bf_words, trusted_kmers_bf_words ) == false)
                devices.erase( devices.begin() + i );
        }
        device_count = uint32( devices.size() );

        if (gpu && (device_count == 0))
        {
            log_warning(stderr, "  no available GPU devices\n");

            // revert to using the CPU even if -no-cpu was specified
            if (cpu == false)
            {
                cpu = true;
                log_warning(stderr, "  switching the CPU on\n");
            }
        }

        if (cpu && h_bloom_filters.setup( -1, sampled_kmers_bf_words, trusted_kmers_bf_words ) == false)
            cpu = false;

        if (cpu == false)
        {
            h_bloom_filters_ptr = NULL;

            if (device_count == 0)
            {
                log_error(stderr, "  no available CPU or GPU devices\n");
                return 1;
            }
        }

        //
        // open the output file
        //

        log_info(stderr, "  opening output file \"%s\"... started\n", output_name);

        SharedPointer<nvbio::io::SequenceDataOutputStream> output_file(
            nvbio::io::open_output_sequence_file(
                output_name,
                comp_level ) );

        if (output_file == NULL || output_file->is_ok() == false)
        {
            log_error(stderr, "    failed opening output \"%s\"\n", output_name);
            return 1;
        }
        log_info(stderr, "  opening output file \"%s\"... done\n", output_name);

        //
        // open the reads file
        //

        uint32 max_block_strings = 512*1024;
        uint32 max_block_bps     = 64*1024*1024;

        log_info(stderr, "  opening read file \"%s\"... started\n", reads_name);

        SharedPointer<nvbio::io::SequenceDataInputStream> read_data_file(
            nvbio::io::open_sequence_file(
                reads_name,
                qencoding,
                uint32(-1),
                uint32(-1),
                io::FORWARD )
        );

        if (read_data_file == NULL || read_data_file->is_ok() == false)
        {
            log_error(stderr, "    failed opening file \"%s\"\n", reads_name);
            return 1;
        }
        log_info(stderr, "  opening read file \"%s\"... done\n", reads_name);

        // infer alpha if necessary
        if (alpha <= 0.0f)
        {
            alpha = infer_alpha( read_data_file.get(), genome_size );

            if (alpha >= 1.0f)
            {
                log_error(stderr, "  alpha cannot be greater than 1, coverage likely too low\n");
                exit(1);
            }
        }

        const char bad_quality = get_bad_quality( read_data_file.get() );
        log_info(stderr, "  bad quality threshold: '%c'\n", bad_quality);
        
        log_info(stderr,"  sample kmers... started\n");
        {
            //
            // The following code implements a parallel nvbio::Pipeline to sample kmers from the input
            // reads. The pipeline is composed several subpipelines, one per active device, each made
            // of two stages (which will be run in separate threads): an InputStage, and a
            // SampleKmersStage doing the actual sampling work.
            //

            log_debug(stderr, "  assemble pipeline\n");

            // build the input stage
            InputStageData input_stage_data( read_data_file.get(), max_block_strings, max_block_bps );
            InputStage       input_stage[16];

            // build the sink
            SampleKmersStage sample_stage[16];
            SequenceStats    sample_stats;

            // setup the pipeline stages
            if (cpu)
            {
                // host stages
                input_stage[device_count]  = InputStage( &input_stage_data );
                sample_stage[device_count] = SampleKmersStage(
                    -threads,
                    k,
                    alpha,
                    sampled_kmers_bf_words * bits_per_word,
                    raw_pointer( h_bloom_filters.sampled_kmers_storage ),
                    &sample_stats );
            }
            for (uint32 i = 0; i < device_count; ++i)
            {
                // device stages
                input_stage[i]  = InputStage( &input_stage_data );
                sample_stage[i] = SampleKmersStage(
                    devices[i],
                    k,
                    alpha,
                    sampled_kmers_bf_words * bits_per_word,
                    raw_pointer( d_bloom_filters[i].sampled_kmers_storage ),
                    &sample_stats );
            }

            // build the pipeline
            nvbio::Pipeline pipeline;
            for (uint32 i = 0; i < device_count + (cpu ? 1 : 0); ++i)
            {
                const uint32 in0 = pipeline.append_stage( &input_stage[i], 4u );
                const uint32 out = pipeline.append_sink( &sample_stage[i] );
                pipeline.add_dependency( in0, out );
            }
            log_debug(stderr, "  start pipeline\n");

            Timer timer;
            timer.start();

            // and run it!
            pipeline.run();

            log_info_cont(stderr, "\n");
            merge( h_bloom_filters_ptr, device_count, d_bloom_filters, SAMPLED_KMERS );

            timer.stop();
            const float time = timer.seconds();

            log_verbose(stderr,"  total time  : %.1fs\n", time);
            log_verbose(stderr,"  peak memory : %.1f GB\n", float( peak_resident_memory() ) / float(1024*1024*1024));
        }
        log_info(stderr,"  sample kmers... done\n");

        log_info(stderr,"  mark trusted kmers... started\n");
        {
            //
            // The following code implements a parallel nvbio::Pipeline to mark trusted kmers in the input
            // reads. The pipeline is composed several subpipelines, one per active device, each made
            // of two stages (which will be run in separate threads): an InputStage, and a
            // TrustedKmersStage doing the actual marking work.
            //

            // gather Bloom filter statistics
            nvbio::vector<host_tag,uint32> threshold( 100, 0u );
            {
                double untrustF[100][100];

                // compute the number of bits set
                float occupancy;
                float approx_size;
                float FP;

                if (device_count)
                {
                    compute_bloom_filter_stats(
                        d_bloom_filters[0],
                        SAMPLED_KMERS,
                        SAMPLED_KMERS_FILTER_K,
                        occupancy,
                        approx_size,
                        FP );
                }
                else
                {
                    compute_bloom_filter_stats(
                        h_bloom_filters,
                        SAMPLED_KMERS,
                        SAMPLED_KMERS_FILTER_K,
                        occupancy,
                        approx_size,
                        FP );
                }

                log_stats(stderr, "  sampled kmers:\n" );
                log_stats(stderr, "    occupancy       : %f\n", occupancy );
                log_stats(stderr, "    #kmers (approx) : %.1f\n", approx_size );
                log_stats(stderr, "    FP rate         : %f\n", FP );

                // compute the i-th untrustF table
                for (uint32 i = 1; i <= k; ++i)
                {
                    int d = (int)(0.1 / alpha * 2);
                    if (d < 2)
                        d = 2;

                    const double p = 1.0 - pow( (1.0 - alpha), d );

                    cumulative_binomial_distribution( untrustF[i], i, p + FP - p * FP );
                }

                // compute the threshold table
                for (uint32 i = 1; i <= k; ++i)
                {
                    for (uint32 j = 0; j <= i; ++j)
                    {
                        if (untrustF[i][j] >= 1 - 0.5 * 1e-2)
                        {
                            threshold[i] = j;
                            break;
                        }
                    }
                }
                log_verbose(stderr, "  thresholds = {");
                for (uint32 i = 1; i <= k; ++i)
                    log_verbose_cont(stderr, " %u,", threshold[i]);
                log_verbose_cont(stderr, " }\n");
            }
            h_bloom_filters.set_threshold( threshold );
            for (uint32 i = 0; i < device_count; ++i)
                d_bloom_filters[i].set_threshold( threshold );

            log_debug(stderr, "  rewind reads\n");
            if (read_data_file->rewind() == false)
            {
                log_error(stderr, "    failed rewinding file \"%s\"\n", reads_name);
                return 1;
            }
            log_debug(stderr, "  assemble pipeline\n");

            // build the input stages
            InputStageData    input_stage_data( read_data_file.get(), max_block_strings, max_block_bps );
            InputStage        input_stage[16];

            // build the trusted-kmer marking stages
            SequenceStats     marking_stats;
            TrustedKmersStage marking_stage[16];

            // setup the pipeline stages
            if (cpu)
            {
                // host stages
                input_stage[device_count]   = InputStage( &input_stage_data );
                marking_stage[device_count] = TrustedKmersStage(
                    -threads,
                    k,
                    sampled_kmers_bf_words * bits_per_word, raw_pointer( h_bloom_filters.sampled_kmers_storage ),
                    trusted_kmers_bf_words * bits_per_word, raw_pointer( h_bloom_filters.trusted_kmers_storage ),
                    raw_pointer( h_bloom_filters.threshold ),
                    &marking_stats );
            }
            for (uint32 i = 0; i < device_count; ++i)
            {
                // device stages
                input_stage[i]   = InputStage( &input_stage_data );
                marking_stage[i] = TrustedKmersStage(
                    devices[i],
                    k,
                    sampled_kmers_bf_words * bits_per_word, raw_pointer( d_bloom_filters[i].sampled_kmers_storage ),
                    trusted_kmers_bf_words * bits_per_word, raw_pointer( d_bloom_filters[i].trusted_kmers_storage ),
                    raw_pointer( d_bloom_filters[i].threshold ),
                    &marking_stats );
            }

            // build the pipeline
            nvbio::Pipeline pipeline;
            for (uint32 i = 0; i < device_count + (cpu ? 1 : 0); ++i)
            {
                const uint32 in0 = pipeline.append_stage( &input_stage[i], 4u );
                const uint32 out = pipeline.append_sink( &marking_stage[i] );
                pipeline.add_dependency( in0, out );
            }
            log_debug(stderr, "  start pipeline\n");

            Timer timer;
            timer.start();

            // and run it!
            pipeline.run();

            log_info_cont(stderr, "\n");
            merge( h_bloom_filters_ptr, device_count, d_bloom_filters, TRUSTED_KMERS );

            timer.stop();
            const float time = timer.seconds();

            log_verbose(stderr,"  total time  : %.1fs\n", time);
            log_verbose(stderr,"  peak memory : %.1f GB\n", float( peak_resident_memory() ) / float(1024*1024*1024));
        }
        log_info(stderr,"  mark trusted kmers... done\n");

        log_info(stderr,"  error correction... started\n");
        {
            //
            // The following code implements a parallel nvbio::Pipeline to error-correct the input
            // reads. The pipeline is composed several subpipelines, one per active device, each made
            // of three stages (which will be run in separate threads): an InputStage, an
            // ErrorCorrectStage doing the actual error correction work, and a final OutputStage.
            //

            // gather Bloom filter statistics
            {
                // compute the number of bits set
                float occupancy;
                float approx_size;
                float FP;

                if (device_count)
                {
                    compute_bloom_filter_stats(
                        d_bloom_filters[0],
                        TRUSTED_KMERS,
                        TRUSTED_KMERS_FILTER_K,
                        occupancy,
                        approx_size,
                        FP );
                }
                else
                {
                    compute_bloom_filter_stats(
                        h_bloom_filters,
                        TRUSTED_KMERS,
                        TRUSTED_KMERS_FILTER_K,
                        occupancy,
                        approx_size,
                        FP );
                }

                log_stats(stderr, "  trusted kmers:\n" );
                log_stats(stderr, "    occupancy       : %f\n", occupancy );
                log_stats(stderr, "    #kmers (approx) : %.1f\n", approx_size );
                log_stats(stderr, "    FP rate         : %f\n", FP );
            }

            log_debug(stderr, "  rewind reads\n");
            if (read_data_file->rewind() == false)
            {
                log_error(stderr, "    failed rewinding file \"%s\"\n", reads_name);
                return 1;
            }
            log_debug(stderr, "  assemble pipeline\n");

            // build the input stages
            InputStageData    input_stage_data( read_data_file.get(), max_block_strings, max_block_bps );
            InputStage        input_stage[16];

            // build the error correction stages
            SequenceStats     ec_stats;
            ErrorCorrectStage ec_stage[16];

            // build the output stages
            OutputStageData   output_stage_data( output_file.get() );
            OutputStage       output_stage[16];

            // setup the pipeline stages
            if (cpu)
            {
                // build the input stage
                input_stage[device_count] = InputStage( &input_stage_data );

                // build the sink
                ec_stage[device_count] = ErrorCorrectStage(
                    -threads,
                    k,
                    trusted_kmers_bf_words * bits_per_word, raw_pointer( h_bloom_filters.trusted_kmers_storage ),
                    raw_pointer( h_bloom_filters.stats ),
                    max_correction,
                    bad_quality,
                    new_quality,
                    &ec_stats );

                // build the input stage
                output_stage[device_count] = OutputStage( &output_stage_data );
            }
            for (uint32 i = 0; i < device_count; ++i)
            {
                // build the input stage
                input_stage[i] = InputStage( &input_stage_data );

                // build the sink
                ec_stage[i] = ErrorCorrectStage(
                    devices[i],
                    k,
                    trusted_kmers_bf_words * bits_per_word, raw_pointer( d_bloom_filters[i].trusted_kmers_storage ),
                    raw_pointer( d_bloom_filters[i].stats ),
                    max_correction,
                    bad_quality,
                    new_quality,
                    &ec_stats );

                // build the input stage
                output_stage[i] = OutputStage( &output_stage_data );
            }
            log_debug(stderr, "  start pipeline\n");

            // build the pipeline
            nvbio::Pipeline pipeline;
            for (uint32 i = 0; i < device_count + (cpu ? 1 : 0); ++i)
            {
                const uint32 in  = pipeline.append_stage( &input_stage[i], 4u );
                const uint32 ec  = pipeline.append_stage( &ec_stage[i] );
                const uint32 out = pipeline.append_sink( &output_stage[i] );
                pipeline.add_dependency( in, ec );
                pipeline.add_dependency( ec, out );
            }

            Timer timer;
            timer.start();

            // and run it!
            pipeline.run();

            timer.stop();
            const float time = timer.seconds();

            log_info_cont(stderr, "\n");

            nvbio::vector<host_tag,uint64> stats;
            merged_stats( h_bloom_filters_ptr, device_count, d_bloom_filters, stats );

            log_info(stderr,"  stats :\n");
            log_info(stderr,"    error free reads : %llu\n", uint64( stats[ERROR_FREE] ));
            log_info(stderr,"    unfixable reads  : %llu\n", uint64( stats[UNFIXABLE] ));
            log_info(stderr,"    corrected bases  : %llu\n", uint64( stats[CORRECTIONS] ));

            log_verbose(stderr,"  total time  : %.1fs\n", time);
            log_verbose(stderr,"  peak memory : %.1f GB\n", float( peak_resident_memory() ) / float(1024*1024*1024));
        }
        log_info(stderr,"  error correction... done\n");

        log_visible(stderr,"nvLighter... done\n");
    }
    catch (nvbio::cuda_error e)
    {
        log_error(stderr, "caught a nvbio::cuda_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (thrust::system::system_error e)
    {
        log_error(stderr, "caught a thrust::system_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (...)
    {
        log_error(stderr, "caught an unknown exception!\n");
        return 1;
    }
    return 0;
}
