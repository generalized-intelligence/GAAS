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

#include <nvbio/basic/console.h>
#include <nvbio/basic/shared_pointer.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/cuda/arch.h>          // cuda::check_error
#include <nvbio/io/fmindex/fmindex.h>
#include <nvbio/io/output/output_file.h>
#include <nvbio/io/sequence/sequence.h>
#ifdef _OPENMP
#include <omp.h>
#endif

#include "options.h"
#include "util.h"
#include "pipeline.h"
#include "mem-search.h"
#include "build-chains.h"
#include "filter-chains.h"
#include "align.h"

using namespace nvbio;

int run(int argc, char **argv)
{
    parse_command_line(argc, argv);
    gpu_init();

    #ifdef _OPENMP
    // now set the number of CPU threads
    omp_set_num_threads( omp_get_num_procs() );
    #pragma omp parallel
    {
        log_verbose(stderr, "  running on multiple threads (%d)\n", omp_get_thread_num());
    }
    #endif

    pipeline_state pipeline;

    // start the pipeline on the device
    pipeline.system = DEVICE;

    // load the fmindex and prepare the SMEM search
    mem_init(&pipeline);

    // open the input read file
    SharedPointer<io::SequenceDataStream> input = SharedPointer<io::SequenceDataStream>(
        io::open_sequence_file(
            command_line_options.input_file_name,
            io::Phred33,
            uint32(-1),
            uint32(-1),
            io::SequenceEncoding(io::FORWARD | io::REVERSE_COMPLEMENT) ) );

    if (input == NULL || input->is_ok() == false)
    {
        log_error(stderr, "failed to open read file %s\n", command_line_options.input_file_name);
        exit(1);
    }

    // open the output file
    pipeline.output = io::OutputFile::open(command_line_options.output_file_name,
            io::SINGLE_END,
            io::BNT(*pipeline.mem.reference_data_host));

    if (!pipeline.output)
    {
        log_error(stderr, "failed to open output file %s\n", command_line_options.output_file_name);
        exit(1);
    }

    Timer  global_timer;
    Timer  timer;

    io::SequenceDataHost host_reads;

    // go!
    for(;;)
    {
        global_timer.start();
        timer.start();

        // read the next batch
        if (io::next( DNA_N, &host_reads, input.get(), command_line_options.batch_size, uint32(-1) ) == 0)
            break;  // EOF

        log_info(stderr, "processing reads [%llu,%llu)\n", pipeline.stats.n_reads, pipeline.stats.n_reads + host_reads.size()/2);

        // copy batch to the device
        const io::SequenceDataDevice device_reads( host_reads );

        timer.stop();
        pipeline.stats.io_time += timer.seconds();

        // search for MEMs
        mem_search(&pipeline, &device_reads);

        // now start a loop where we break the read batch into smaller chunks for
        // which we can locate all MEMs and build all chains
        for (uint32 read_begin = 0; read_begin < host_reads.size(); read_begin = pipeline.chunk.read_end)
        {
            // determine the next chunk of reads to process
            fit_read_chunk(&pipeline, &device_reads, read_begin);

            log_verbose(stderr, "processing chunk\n");
            log_verbose(stderr, "  reads : [%u,%u)\n", pipeline.chunk.read_begin, pipeline.chunk.read_end);
            log_verbose(stderr, "  mems  : [%u,%u)\n", pipeline.chunk.mem_begin,  pipeline.chunk.mem_end);

            // locate all MEMs in the current chunk
            mem_locate(&pipeline, &device_reads);

            // build the chains
            build_chains(&pipeline, &device_reads);

            // filter the chains
            filter_chains(&pipeline, &device_reads);

            log_verbose(stderr, "  chains: %u\n", pipeline.chn.n_chains);

            // initialize the alignment sub-pipeline
            align_init(&pipeline, &device_reads);

            // and loop until there's work to do
            while (align(&pipeline, &host_reads, &device_reads))
            {
                log_verbose(stderr, "\r    active: %u", pipeline.aln.n_active);
            }
            log_verbose_cont(stderr, "\n");
        }
        global_timer.stop();
        pipeline.stats.n_reads += host_reads.size()/2;
        pipeline.stats.time    += global_timer.seconds();

        log_stats(stderr, "  time: %5.1fs (%.1f K reads/s)\n", pipeline.stats.time, 1.0e-3f * float(pipeline.stats.n_reads)/pipeline.stats.time);
        log_stats(stderr, "    io     : %6.2f %%\n", 100.0f * pipeline.stats.io_time/pipeline.stats.time);
        log_stats(stderr, "    search : %6.2f %%\n", 100.0f * pipeline.stats.search_time/pipeline.stats.time);
        log_stats(stderr, "    locate : %6.2f %%\n", 100.0f * pipeline.stats.locate_time/pipeline.stats.time);
        log_stats(stderr, "    chain  : %6.2f %%\n", 100.0f * pipeline.stats.chain_time/pipeline.stats.time);
    }

    pipeline.output->close();
    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        return run( argc, argv );
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
    return 0;
}
