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

//#define NVBIO_ENABLE_PROFILING

#define MOD_NAMESPACE
#define MOD_NAMESPACE_BEGIN namespace bowtie2 { namespace driver {
#define MOD_NAMESPACE_END   }}
#define MOD_NAMESPACE_NAME bowtie2::driver

#include <nvBowtie/bowtie2/cuda/compute_thread.h>
#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/fmindex_def.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvBowtie/bowtie2/cuda/stats.h>
#include <nvBowtie/bowtie2/cuda/persist.h>
#include <nvBowtie/bowtie2/cuda/scoring.h>
#include <nvBowtie/bowtie2/cuda/mapq.h>
#include <nvBowtie/bowtie2/cuda/aligner.h>
#include <nvBowtie/bowtie2/cuda/aligner_inst.h>
#include <nvBowtie/bowtie2/cuda/input_thread.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/options.h>
#include <nvbio/basic/threads.h>
#include <nvbio/basic/atomics.h>
#include <nvbio/basic/html.h>
#include <nvbio/basic/version.h>
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

ComputeThreadSE::ComputeThreadSE(
    const uint32                             _thread_id,
    const uint32                             _device_id,
    const io::SequenceData&                  _reference_data,
    const io::FMIndexData&                   _driver_data,
    const std::map<std::string,std::string>& _options,
    const Params&                            _params,
          Stats&                             _stats) :
    thread_id( _thread_id ),
    device_id( _device_id ),
    reference_data_host( _reference_data ),
    driver_data_host( _driver_data ),
    options( _options ),
    input_thread( NULL ),
    output_file( NULL ),
    params( _params ),
    stats( _stats )
{
    log_visible(stderr, "[%u] nvBowtie cuda driver created on device %u\n", thread_id, device_id);

    // initialize the selected device
    cudaSetDevice( device_id );
    cudaSetDeviceFlags( cudaDeviceMapHost | cudaDeviceLmemResizeToMax );

    aligner = SharedPointer<Aligner>( new Aligner() );

    Timer timer;
    timer.start();

    const bool need_reverse =
        (params.allow_sub == 0 && USE_REVERSE_INDEX) ||
        (params.allow_sub == 1 && params.subseed_len == 0 && params.mode == BestMappingApprox);

    reference_data_device.reset( new io::SequenceDataDevice( reference_data_host ) );

    driver_data_device.reset( new io::FMIndexDataDevice( driver_data_host,
                        io::FMIndexDataDevice::FORWARD |
        (need_reverse ? io::FMIndexDataDevice::REVERSE : 0u) |
                        io::FMIndexDataDevice::SA ) );

    timer.stop();

    log_stats(stderr, "[%u]   allocated device driver data (%.2f GB - %.1fs)\n", thread_id, float(driver_data_device->allocated()) / 1.0e9f, timer.seconds() );
}

// gauge the favourite batch size
//
uint32 ComputeThreadSE::gauge_batch_size()
{
    // switch to the selected device
    cudaSetDevice( device_id );

    uint32 BATCH_SIZE;

    for (BATCH_SIZE = params.max_batch_size*1024; BATCH_SIZE >= 16*1024; BATCH_SIZE /= 2)
    {
        std::pair<uint64,uint64> mem_stats;

        // gauge how much memory we'd need
        if (aligner->init_alloc( BATCH_SIZE, params, kSingleEnd, false, &mem_stats ) == true)
        {
            log_stats(stderr, "[%u]   estimated allocation sizes: HOST %lu MB, DEVICE %lu MB)\n",
                thread_id,
                mem_stats.first / (1024*1024),
                mem_stats.second / (1024*1024) );
            break;
        }
    }

    return BATCH_SIZE;
}

void ComputeThreadSE::do_run()
{
    log_visible(stderr, "[%u] nvBowtie cuda driver... started\n", thread_id);

    // switch to the selected device
    cudaSetDevice( device_id );

    // build an empty report
    FILE* html_output = (params.report != std::string("")) ? fopen( params.report.c_str(), "w" ) : NULL;
    if (html_output)
    {
        // encapsulate the document
        {
            html::html_object html( html_output );
            {
                const char* meta_list = "<meta http-equiv=\"refresh\" content=\"1\" />";

                { html::header_object hd( html_output, "Bowtie2 Report", html::style(), meta_list ); }
                { html::body_object body( html_output ); }
            }
        }
        fclose( html_output );
    }

    Timer timer;

    io::SequenceDataDevice& reference_data = *reference_data_device.get();
    io::FMIndexDataDevice&  driver_data    = *driver_data_device.get();

    log_stats(stderr, "[%u]   allocated device driver data (%.2f GB - %.1fs)\n", thread_id, float(driver_data.allocated()) / 1.0e9f, timer.seconds() );

    typedef FMIndexDef::type fm_index_type;

    fm_index_type fmi  = driver_data.index();
    fm_index_type rfmi = driver_data.rindex();

    size_t free, total;
    cudaMemGetInfo(&free, &total);
    log_stats(stderr, "[%u]   device has %ld of %ld MB free\n", thread_id, free/1024/1024, total/1024/1024);

    const uint32 BATCH_SIZE = input_thread->batch_size();

    log_stats(stderr, "[%u]   processing reads in batches of %uK\n", thread_id, BATCH_SIZE/1024);

    // setup the output file
    aligner->output_file = output_file;

    // initialize the aligner
    if (aligner->init( thread_id, BATCH_SIZE, params, kSingleEnd ) == false)
        return;

    nvbio::cuda::check_error("cuda initializations");

    cudaMemGetInfo(&free, &total);
    log_stats(stderr, "[%u]   ready to start processing: device has %ld MB free\n", thread_id, free/1024/1024);

    Timer global_timer;
    global_timer.start();

    UberScoringScheme& scoring_scheme = params.scoring_scheme;

    uint32 n_reads = 0;

    io::SequenceDataHost   local_read_data_host;
    io::HostOutputBatchSE  local_output_batch_host;

    // loop through the batches of reads
    while (1)
    {
        uint32 read_begin;

        Timer io_timer;
        io_timer.start();

        io::SequenceDataHost* read_data_host = input_thread->next( &read_begin );

        io_timer.stop();
        stats.read_io.add( read_data_host ? read_data_host->size() : 0u, io_timer.seconds() );

        if (read_data_host == NULL)
        {
            log_verbose(stderr, "[%u] end of input reached\n", thread_id);
            break;
        }

        if (read_data_host->max_sequence_len() > Aligner::MAX_READ_LEN)
        {
            log_error(stderr, "[%u] unsupported read length %u (maximum is %u)\n", thread_id,
                read_data_host->max_sequence_len(),
                Aligner::MAX_READ_LEN );
            break;
        }

        // make a local copy of the host batch
        local_read_data_host = *read_data_host;

        // mark this set as ready to be reused
        input_thread->release( read_data_host );

        Timer timer;
        timer.start();

        //aligner.output_file->start_batch( &local_read_data_host );
        local_output_batch_host.read_data = &local_read_data_host;

        io::SequenceDataDevice read_data( local_read_data_host );
        cudaThreadSynchronize();

        timer.stop();
        stats.read_HtoD.add( read_data.size(), timer.seconds() );

        const uint32 count = read_data.size();
        log_info(stderr, "[%u] aligning reads [%u, %u]\n", thread_id, read_begin, read_begin + count - 1u);
        log_verbose(stderr, "[%u]   %u reads\n", thread_id, count);
        log_verbose(stderr, "[%u]   %.3f M bps (%.1f MB)\n", thread_id, float(read_data.bps())/1.0e6f, float(read_data.words()*sizeof(uint32)+read_data.bps()*sizeof(char))/float(1024*1024));
        log_verbose(stderr, "[%u]   %.1f bps/read (min: %u, max: %u)\n", thread_id, float(read_data.bps())/float(read_data.size()), read_data.min_sequence_len(), read_data.max_sequence_len());

        if (params.mode == AllMapping)
        {
            if (params.scoring_mode == EditDistanceMode)
            {
                all_ed(
                    *aligner,
                    params,
                    fmi,
                    rfmi,
                    scoring_scheme,
                    reference_data,
                    driver_data,
                    read_data,
                    local_output_batch_host,
                    stats );
            }
            else
            {
                all_sw(
                    *aligner,
                    params,
                    fmi,
                    rfmi,
                    scoring_scheme,
                    reference_data,
                    driver_data,
                    read_data,
                    local_output_batch_host,
                    stats );
            }
        }
        else
        {
            if (params.scoring_mode == EditDistanceMode)
            {
                best_approx_ed(
                    *aligner,
                    params,
                    fmi,
                    rfmi,
                    scoring_scheme,
                    reference_data,
                    driver_data,
                    read_data,
                    local_output_batch_host,
                    stats );
            }
            else
            {
                best_approx_sw(
                    *aligner,
                    params,
                    fmi,
                    rfmi,
                    scoring_scheme,
                    reference_data,
                    driver_data,
                    read_data,
                    local_output_batch_host,
                    stats );
            }
        }

        global_timer.stop();
        stats.global_time += global_timer.seconds();
        global_timer.start();

        //aligner->output_file->end_batch();

        // increase the total reads counter
        n_reads += count;

        log_verbose(stderr, "[%u]   %.1f K reads/s\n", thread_id, 1.0e-3f * float(n_reads) / stats.global_time);
    }

    global_timer.stop();
    stats.global_time += global_timer.seconds();
    stats.n_reads = n_reads;

    if (params.report.length())
        nvbio::bowtie2::cuda::generate_device_report( thread_id, stats, stats.mate1, params.report.c_str() );

    log_visible(stderr, "[%u] nvBowtie cuda driver... done\n", thread_id);

    log_stats(stderr, "[%u]   total        : %.2f sec (avg: %.1fK reads/s).\n", thread_id, stats.global_time, 1.0e-3f * float(n_reads)/stats.global_time);
    log_stats(stderr, "[%u]   mapping      : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s, %.2f device sec).\n", thread_id, stats.map.time, 1.0e-6f * stats.map.avg_speed(), 1.0e-6f * stats.map.max_speed, stats.map.device_time);
    log_stats(stderr, "[%u]   selecting    : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s, %.2f device sec).\n", thread_id, stats.select.time, 1.0e-6f * stats.select.avg_speed(), 1.0e-6f * stats.select.max_speed, stats.select.device_time);
    log_stats(stderr, "[%u]   sorting      : %.2f sec (avg: %.3fM seeds/s, max: %.3fM seeds/s, %.2f device sec).\n", thread_id, stats.sort.time, 1.0e-6f * stats.sort.avg_speed(), 1.0e-6f * stats.sort.max_speed, stats.sort.device_time);
    log_stats(stderr, "[%u]   scoring      : %.2f sec (avg: %.3fM seeds/s, max: %.3fM seeds/s, %.2f device sec).\n", thread_id, stats.score.time, 1.0e-6f * stats.score.avg_speed(), 1.0e-6f * stats.score.max_speed, stats.score.device_time);
    log_stats(stderr, "[%u]   locating     : %.2f sec (avg: %.3fM seeds/s, max: %.3fM seeds/s, %.2f device sec).\n", thread_id, stats.locate.time, 1.0e-6f * stats.locate.avg_speed(), 1.0e-6f * stats.locate.max_speed, stats.locate.device_time);
    log_stats(stderr, "[%u]   backtracking : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s, %.2f device sec).\n", thread_id, stats.backtrack.time, 1.0e-6f * stats.backtrack.avg_speed(), 1.0e-6f * stats.backtrack.max_speed, stats.backtrack.device_time);
    log_stats(stderr, "[%u]   finalizing   : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s, %.2f device sec).\n", thread_id, stats.finalize.time, 1.0e-6f * stats.finalize.avg_speed(), 1.0e-6f * stats.finalize.max_speed, stats.finalize.device_time);
    log_stats(stderr, "[%u]   results DtoH : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", thread_id, stats.alignments_DtoH.time, 1.0e-6f * stats.alignments_DtoH.avg_speed(), 1.0e-6f * stats.alignments_DtoH.max_speed);
    log_stats(stderr, "[%u]   results I/O  : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", thread_id, stats.io.time, 1.0e-6f * stats.io.avg_speed(), 1.0e-6f * stats.io.max_speed);
    log_stats(stderr, "[%u]   reads HtoD   : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", thread_id, stats.read_HtoD.time, 1.0e-6f * stats.read_HtoD.avg_speed(), 1.0e-6f * stats.read_HtoD.max_speed);
    log_stats(stderr, "[%u]   reads I/O    : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", thread_id, stats.read_io.time, 1.0e-6f * stats.read_io.avg_speed(), 1.0e-6f * stats.read_io.max_speed);
}


void ComputeThreadSE::run()
{
    try {
        do_run();
    }
    catch (nvbio::cuda_error e)
    {
        log_error(stderr, "caught a nvbio::cuda_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (...)
    {
        log_error(stderr, "caught an unknown exception!\n");
    }
}

ComputeThreadPE::ComputeThreadPE(
    const uint32                             _thread_id,
    const uint32                             _device_id,
    const io::SequenceData&                  _reference_data,
    const io::FMIndexData&                   _driver_data,
    const std::map<std::string,std::string>& _options,
    const Params&                            _params,
          Stats&                             _stats) :
    thread_id( _thread_id ),
    device_id( _device_id ),
    reference_data_host( _reference_data ),
    driver_data_host( _driver_data ),
    options( _options ),
    input_thread( NULL ),
    output_file( NULL ),
    params( _params ),
    stats( _stats )
{
    log_visible(stderr, "[%u] nvBowtie cuda driver created on device %u\n", thread_id, device_id);

    // initialize the selected device
    cudaSetDevice( device_id );
    cudaSetDeviceFlags( cudaDeviceMapHost | cudaDeviceLmemResizeToMax );

    aligner = SharedPointer<Aligner>( new Aligner() );

    Timer timer;
    timer.start();

    const bool need_reverse =
        (params.allow_sub == 0 && USE_REVERSE_INDEX) ||
        (params.allow_sub == 1 && params.subseed_len == 0 && params.mode == BestMappingApprox);

    reference_data_device.reset( new io::SequenceDataDevice( reference_data_host ) );

    driver_data_device.reset( new io::FMIndexDataDevice( driver_data_host,
                        io::FMIndexDataDevice::FORWARD |
        (need_reverse ? io::FMIndexDataDevice::REVERSE : 0u) |
                        io::FMIndexDataDevice::SA ) );

    timer.stop();

    log_stats(stderr, "[%u]   allocated device driver data (%.2f GB - %.1fs)\n", thread_id, float(driver_data_device->allocated()) / 1.0e9f, timer.seconds() );
}

// gauge the favourite batch size
//
uint32 ComputeThreadPE::gauge_batch_size()
{
    // switch to the selected device
    cudaSetDevice( device_id );

    uint32 BATCH_SIZE;

    for (BATCH_SIZE = params.max_batch_size*1024; BATCH_SIZE >= 16*1024; BATCH_SIZE /= 2)
    {
        std::pair<uint64,uint64> mem_stats;

        // gauge how much memory we'd need
        if (aligner->init_alloc( BATCH_SIZE, params, kPairedEnds, false, &mem_stats ) == true)
        {
            log_stats(stderr, "[%u]   estimated allocation sizes: HOST %lu MB, DEVICE %lu MB)\n",
                thread_id,
                mem_stats.first / (1024*1024),
                mem_stats.second / (1024*1024) );
            break;
        }
    }

    return BATCH_SIZE;
}

void ComputeThreadPE::do_run()
{
    log_visible(stderr, "[%u] nvBowtie cuda driver... started\n", thread_id);

    // switch to the selected device
    cudaSetDevice( device_id );

    // build an empty report
    FILE* html_output = (params.report != std::string("")) ? fopen( params.report.c_str(), "w" ) : NULL;
    if (html_output)
    {
        // encapsulate the document
        {
            html::html_object html( html_output );
            {
                const char* meta_list = "<meta http-equiv=\"refresh\" content=\"1\" />";

                { html::header_object hd( html_output, "Bowtie2 Report", html::style(), meta_list ); }
                { html::body_object body( html_output ); }
            }
        }
        fclose( html_output );
    }

    Timer timer;

    io::SequenceDataDevice& reference_data = *reference_data_device.get();
    io::FMIndexDataDevice&  driver_data    = *driver_data_device.get();

    typedef FMIndexDef::type fm_index_type;

    fm_index_type fmi  = driver_data.index();
    fm_index_type rfmi = driver_data.rindex();

    size_t free, total;
    cudaMemGetInfo(&free, &total);
    log_stats(stderr, "[%u]   device has %ld of %ld MB free\n", thread_id, free/1024/1024, total/1024/1024);

    const uint32 BATCH_SIZE = input_thread->batch_size();

    log_stats(stderr, "[%u]   processing reads in batches of %uK\n", thread_id, BATCH_SIZE/1024);

    // setup the output file
    aligner->output_file = output_file;

    // initialize the aligner
    if (aligner->init( thread_id, BATCH_SIZE, params, kPairedEnds ) == false)
        return;

    nvbio::cuda::check_error("cuda initializations");

    cudaMemGetInfo(&free, &total);
    log_stats(stderr, "[%u]   ready to start processing: device has %ld MB free\n", thread_id, free/1024/1024);

    size_t stack_size_limit;
    cudaDeviceGetLimit( &stack_size_limit, cudaLimitStackSize );
    log_debug(stderr, "[%u]   max cuda stack size: %u\n", thread_id, stack_size_limit);

    Timer global_timer;
    global_timer.start();

    UberScoringScheme& scoring_scheme = params.scoring_scheme;

    uint32 n_reads = 0;

    io::SequenceDataHost    local_read_data_host1;
    io::SequenceDataHost    local_read_data_host2;
    io::HostOutputBatchPE   local_output_batch_host;

    // loop through the batches of reads
    while (1)
    {
        uint32 read_begin;

        Timer io_timer;
        io_timer.start();

        std::pair<io::SequenceDataHost*,io::SequenceDataHost*> read_data_host_pair = input_thread->next( &read_begin );

        io::SequenceDataHost* read_data_host1 = read_data_host_pair.first;
        io::SequenceDataHost* read_data_host2 = read_data_host_pair.second;

        io_timer.stop();
        stats.read_io.add( read_data_host1 ? read_data_host1->size() : 0u, io_timer.seconds() );

        if (read_data_host1 == NULL ||
            read_data_host2 == NULL)
        {
            log_verbose(stderr, "[%u] end of input reached\n", thread_id);
            break;
        }

        if ((read_data_host1->max_sequence_len() > Aligner::MAX_READ_LEN) ||
            (read_data_host2->max_sequence_len() > Aligner::MAX_READ_LEN))
        {
            log_error(stderr, "[%u] unsupported read length %u (maximum is %u)\n",
                thread_id,
                nvbio::max(read_data_host1->max_sequence_len(), read_data_host2->max_sequence_len()),
                Aligner::MAX_READ_LEN );
            break;
        }

        // make a local copy of the host batch
        local_read_data_host1 = *read_data_host1;
        local_read_data_host2 = *read_data_host2;

        // mark this set as ready to be reused
        input_thread->release( read_data_host_pair );

        Timer timer;
        timer.start();

        //aligner.output_file->start_batch( &local_read_data_host1, &local_read_data_host2 );
        local_output_batch_host.read_data[0] = &local_read_data_host1;
        local_output_batch_host.read_data[1] = &local_read_data_host2;

        io::SequenceDataDevice read_data1( local_read_data_host1/*, io::ReadDataDevice::READS | io::ReadDataDevice::QUALS*/ );
        io::SequenceDataDevice read_data2( local_read_data_host2/*, io::ReadDataDevice::READS | io::ReadDataDevice::QUALS*/ );

        timer.stop();
        stats.read_HtoD.add( read_data1.size(), timer.seconds() );

        const uint32 count = read_data1.size();
        log_info(stderr, "[%u] aligning reads [%u, %u]\n", thread_id, read_begin, read_begin + count - 1u);
        log_verbose(stderr, "[%u]   %u reads\n", thread_id, count);
        log_verbose(stderr, "[%u]   %.3f M bps (%.1f MB)\n", thread_id,
            float(read_data1.bps() + read_data2.bps())/1.0e6f,
            float(read_data1.words()*sizeof(uint32)+read_data1.bps()*sizeof(char))/float(1024*1024)+
            float(read_data2.words()*sizeof(uint32)+read_data2.bps()*sizeof(char))/float(1024*1024));
        log_verbose(stderr, "[%u]   %.1f bps/read (min: %u, max: %u)\n", thread_id,
            float(read_data1.bps()+read_data2.bps())/float(read_data1.size()+read_data2.size()),
            nvbio::min( read_data1.min_sequence_len(), read_data2.min_sequence_len() ),
            nvbio::max( read_data1.max_sequence_len(), read_data2.max_sequence_len() ));

        if (params.mode == AllMapping)
        {
            log_error(stderr, "[%u] paired-end all-mapping is not yet supported!\n", thread_id);
            exit(1);
        }
        else
        {
            if (params.scoring_mode == EditDistanceMode)
            {
                best_approx_ed(
                    *aligner,
                    params,
                    fmi,
                    rfmi,
                    scoring_scheme,
                    reference_data,
                    driver_data,
                    read_data1,
                    read_data2,
                    local_output_batch_host,
                    stats );
            }
            else
            {
                best_approx_sw(
                    *aligner,
                    params,
                    fmi,
                    rfmi,
                    scoring_scheme,
                    reference_data,
                    driver_data,
                    read_data1,
                    read_data2,
                    local_output_batch_host,
                    stats );
            }
        }

        global_timer.stop();
        stats.global_time += global_timer.seconds();
        global_timer.start();

        //aligner.output_file->end_batch();

        // increase the total reads counter
        n_reads += count;

        log_verbose(stderr, "[%u]   %.1f K reads/s\n", thread_id, 1.0e-3f * float(n_reads) / stats.global_time);
    }

    global_timer.stop();
    stats.global_time += global_timer.seconds();

    if (params.report.length())
        nvbio::bowtie2::cuda::generate_device_report( thread_id, stats, stats.concordant, params.report.c_str() );

    log_visible(stderr, "[%u] nvBowtie cuda driver... done\n", thread_id);

    log_stats(stderr, "[%u]   total          : %.2f sec (avg: %.1fK reads/s).\n", thread_id, stats.global_time, 1.0e-3f * float(n_reads)/stats.global_time);
    log_stats(stderr, "[%u]   mapping        : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s, %.2f device sec).\n", thread_id, stats.map.time, 1.0e-6f * stats.map.avg_speed(), 1.0e-6f * stats.map.max_speed, stats.map.device_time);
    log_stats(stderr, "[%u]   scoring        : %.2f sec (avg: %.1fM reads/s, max: %.3fM reads/s, %.2f device sec).).\n", thread_id, stats.scoring_pipe.time, 1.0e-6f * stats.scoring_pipe.avg_speed(), 1.0e-6f * stats.scoring_pipe.max_speed, stats.scoring_pipe.device_time);
    log_stats(stderr, "[%u]     selecting    : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s, %.2f device sec).\n", thread_id, stats.select.time, 1.0e-6f * stats.select.avg_speed(), 1.0e-6f * stats.select.max_speed, stats.select.device_time);
    log_stats(stderr, "[%u]     sorting      : %.2f sec (avg: %.3fM seeds/s, max: %.3fM seeds/s, %.2f device sec).\n", thread_id, stats.sort.time, 1.0e-6f * stats.sort.avg_speed(), 1.0e-6f * stats.sort.max_speed, stats.sort.device_time);
    log_stats(stderr, "[%u]     scoring(a)   : %.2f sec (avg: %.3fM seeds/s, max: %.3fM seeds/s, %.2f device sec).\n", thread_id, stats.score.time, 1.0e-6f * stats.score.avg_speed(), 1.0e-6f * stats.score.max_speed, stats.score.device_time);
    log_stats(stderr, "[%u]     scoring(o)   : %.2f sec (avg: %.3fM seeds/s, max: %.3fM seeds/s, %.2f device sec).\n", thread_id, stats.opposite_score.time, 1.0e-6f * stats.opposite_score.avg_speed(), 1.0e-6f * stats.opposite_score.max_speed, stats.opposite_score.device_time);
    log_stats(stderr, "[%u]     locating     : %.2f sec (avg: %.3fM seeds/s, max: %.3fM seeds/s, %.2f device sec).\n", thread_id, stats.locate.time, 1.0e-6f * stats.locate.avg_speed(), 1.0e-6f * stats.locate.max_speed, stats.locate.device_time);
    log_stats(stderr, "[%u]   backtracing(a) : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s, %.2f device sec).\n", thread_id, stats.backtrack.time, 1.0e-6f * stats.backtrack.avg_speed(), 1.0e-6f * stats.backtrack.max_speed, stats.backtrack.device_time);
    log_stats(stderr, "[%u]   backtracing(o) : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s, %.2f device sec).\n", thread_id, stats.backtrack_opposite.time, 1.0e-6f * stats.backtrack_opposite.avg_speed(), 1.0e-6f * stats.backtrack_opposite.max_speed, stats.backtrack_opposite.device_time);
    log_stats(stderr, "[%u]   finalizing     : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s, %.2f device sec).\n", thread_id, stats.finalize.time, 1.0e-6f * stats.finalize.avg_speed(), 1.0e-6f * stats.finalize.max_speed, stats.finalize.device_time);
    log_stats(stderr, "[%u]   results DtoH   : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", thread_id, stats.alignments_DtoH.time, 1.0e-6f * stats.alignments_DtoH.avg_speed(), 1.0e-6f * stats.alignments_DtoH.max_speed);
    log_stats(stderr, "[%u]   results I/O    : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", thread_id, stats.io.time, 1.0e-6f * stats.io.avg_speed(), 1.0e-6f * stats.io.max_speed);
    log_stats(stderr, "[%u]   reads HtoD     : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", thread_id, stats.read_HtoD.time, 1.0e-6f * stats.read_HtoD.avg_speed(), 1.0e-6f * stats.read_HtoD.max_speed);
    log_stats(stderr, "[%u]   reads I/O      : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", thread_id, stats.read_io.time, 1.0e-6f * stats.read_io.avg_speed(), 1.0e-6f * stats.read_io.max_speed);
}

void ComputeThreadPE::run()
{
    try {
        do_run();
    }
    catch (nvbio::cuda_error e)
    {
        log_error(stderr, "caught a nvbio::cuda_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (...)
    {
        log_error(stderr, "caught an unknown exception!\n");
    }
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
