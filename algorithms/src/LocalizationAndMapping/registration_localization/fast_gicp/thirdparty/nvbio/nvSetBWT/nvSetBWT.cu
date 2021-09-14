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

// nvSetBWT.cu
//

//#define NVBIO_CUDA_DEBUG

#include <cub/cub.cuh>
#include <nvbio/basic/omp.h>

#include "input_thread.h"
#include <nvbio/basic/pipeline.h>
#include <nvbio/sufsort/sufsort.h>
#include <nvbio/sufsort/sufsort_utils.h>
#include <nvbio/sufsort/file_bwt.h>
#include <nvbio/sufsort/bwte.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/shared_pointer.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/dna.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/system.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/io/sequence/sequence.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

using namespace nvbio;

static const uint32 SYMBOL_SIZE = io::SequenceDataAccess<DNA>::SEQUENCE_BITS;
static const bool   BIG_ENDIAN  = io::SequenceDataAccess<DNA>::SEQUENCE_BIG_ENDIAN;

typedef io::SequenceDataAccess<DNA>::sequence_storage_iterator  storage_iterator;
typedef io::SequenceDataAccess<DNA>::index_iterator             offsets_iterator;

typedef BWTEContext<SYMBOL_SIZE,BIG_ENDIAN,storage_iterator,offsets_iterator> BWTE_context_type;

///
/// A small class implementing a Pipeline stage reading sequence batches from a file
///
struct SortStage
{
    typedef io::SequenceDataHost   argument_type;
    typedef BWTEBlock              return_type;

    /// constructor
    ///
    ///\param file          input sequence file
    ///\param max_strings   maximum number of strings per batch
    ///\param max_bps       maximum number of base pairs per batch
    ///
    SortStage(BWTE_context_type& context) : m_context( context ) {}

    /// fill the next batch
    ///
    bool process(PipelineContext& context)
    {
        // fetch the input
        io::SequenceDataHost* h_read_data = context.input<io::SequenceDataHost>(0);

        // fetch the output
        BWTEBlock* block = context.output<BWTEBlock>();

        // build a view
        const io::SequenceDataAccess<DNA> h_read_view( *h_read_data );

        m_context.sort_block(
            0u,
            h_read_data->size(),
            h_read_view.sequence_string_set(),
            *block );

        return true;
    }

    BWTE_context_type&  m_context;
};

///
/// A small class implementing a Pipeline stage reading sequence batches from a file
///
struct SinkStage
{
    typedef io::SequenceDataHost   argument_type;

    /// constructor
    ///
    ///\param file          input sequence file
    ///\param max_strings   maximum number of strings per batch
    ///\param max_bps       maximum number of base pairs per batch
    ///
    SinkStage(
        BWTE_context_type&                  context,
        PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  bwt,
        SparseSymbolSet&                    dollars) :
        m_context( context ),
        m_bwt( bwt ),
        m_dollars( dollars ),
        n_reads( 0 ),
        m_time( 0.0f )
    {}

    /// fill the next batch
    ///
    bool process(PipelineContext& context)
    {
        const ScopedTimer<float> timer( &m_time );

        // fetch the input
        io::SequenceDataHost* h_read_data = context.input<io::SequenceDataHost>( 0 );

        // build a view
        const io::SequenceDataAccess<DNA> h_read_view( *h_read_data );

        log_info(stderr, "  block [%u, %u] (%u / %.2fG bps, %.1f M suffixes/s)\n",
            n_reads, n_reads + h_read_data->size(), h_read_data->bps(),
            1.0e-9f * m_bwt.size(),
            m_time ? (1.0e-6f * m_bwt.size()) / m_time : 0.0f );
        log_debug(stderr,"  peak memory : %.1f GB\n", float( peak_resident_memory() ) / float(1024*1024*1024));

        /*
        m_context.append_block(
            0u,
            h_read_data->size(),
            h_read_view.sequence_string_set(),
            m_bwt,
            m_dollars,
            true );
            */

        // fetch the second input
        BWTEBlock* block = context.input<BWTEBlock>( 1 );

        m_context.merge_block(
            0u,
            h_read_data->size(),
            h_read_view.sequence_string_set(),
            *block,
            m_bwt,
            m_dollars,
            true );

        n_reads += h_read_data->size();
        return true;
    }

    BWTE_context_type&                  m_context;
    PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  m_bwt;
    SparseSymbolSet&                    m_dollars;
    uint32                              n_reads;
    float                               m_time;
};

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        log_visible(stderr, "nvSetBWT - Copyright 2013-2014, NVIDIA Corporation\n");
        log_info(stderr, "usage:\n");
        log_info(stderr, "  nvSetBWT [options] input_file output_file\n");
        log_info(stderr, "  options:\n");
        log_info(stderr, "   -v       | --verbosity     int (0-6) [5]\n");
        log_info(stderr, "   -c       | --compression   string    [1R]   (e.g. \"1\", ..., \"9\", \"1R\")\n");
        log_info(stderr, "   -t       | --threads       int       [auto]\n");
        log_info(stderr, "   -b       | --bucketing     int       [16]   (# of bits used for bucketing)\n");
        log_info(stderr, "   -F       | --skip-forward\n");
        log_info(stderr, "   -R       | --skip-reverse\n");
        log_info(stderr, "  output formats:\n");
        log_info(stderr, "    .txt      ASCII\n");
        log_info(stderr, "    .txt.gz   ASCII, gzip compressed\n");
        log_info(stderr, "    .txt.bgz  ASCII, block-gzip compressed\n");
        log_info(stderr, "    .bwt      2-bit packed binary\n");
        log_info(stderr, "    .bwt.gz   2-bit packed binary, gzip compressed\n");
        log_info(stderr, "    .bwt.bgz  2-bit packed binary, block-gzip compressed\n");
        log_info(stderr, "    .bwt4     4-bit packed binary\n");
        log_info(stderr, "    .bwt4.gz  4-bit packed binary, gzip compressed\n");
        log_info(stderr, "    .bwt4.bgz 4-bit packed binary, block-gzip compressed\n");
        return 0;
    }

    const char* reads_name        = argv[argc-2];
    const char* output_name       = argv[argc-1];
    bool  forward                 = true;
    bool  reverse                 = true;
    const char* comp_level        = "1R";
    io::QualityEncoding qencoding = io::Phred33;
    int   threads                 = 0;

    for (int i = 0; i < argc - 2; ++i)
    {
        if ((strcmp( argv[i], "-v" )             == 0) ||
            (strcmp( argv[i], "-verbosity" )     == 0) ||
            (strcmp( argv[i], "--verbosity" )    == 0))
        {
            set_verbosity( Verbosity( atoi( argv[++i] ) ) );
        }
        else if ((strcmp( argv[i], "-F" )             == 0) ||
                 (strcmp( argv[i], "--skip-forward" ) == 0))  // skip forward strand
        {
            forward = false;
        }
        else if ((strcmp( argv[i], "-R" )             == 0) ||
                 (strcmp( argv[i], "--skip-reverse" ) == 0))  // skip reverse strand
        {
            reverse = false;
        }
        else if ((strcmp( argv[i], "-c" )             == 0) ||
                 (strcmp( argv[i], "--compression" )  == 0))  // setup compression level
        {
            comp_level = argv[++i];
        }
        else if ((strcmp( argv[i], "-t" )             == 0) ||
                 (strcmp( argv[i], "--threads" )      == 0))  // setup number of threads
        {
            threads = atoi( argv[++i] );
        }
    }

    try
    {
        log_visible(stderr,"nvSetBWT... started\n");

        // build an output file
        SharedPointer<SetBWTHandler> output_handler = SharedPointer<SetBWTHandler>( open_bwt_file( output_name, comp_level ) );
        if (output_handler == NULL)
        {
            log_error(stderr, "  failed to create an output handler\n");
            return 1;
        }

        // gather device memory stats
        size_t free_device, total_device;
        cudaMemGetInfo(&free_device, &total_device);
        cuda::check_error("cuda-check");

        log_stats(stderr, "  device has %ld of %ld MB free\n", free_device/1024/1024, total_device/1024/1024);

    #ifdef _OPENMP
        // now set the number of CPU threads
        omp_set_num_threads( threads > 0 ? threads : omp_get_num_procs() );
        omp_set_nested(1);
        #pragma omp parallel
        {
            log_verbose(stderr, "  running on multiple threads (%d)\n", omp_get_thread_num());
        }
    #endif

        uint32       encoding_flags  = 0u;
        if (forward) encoding_flags |= io::FORWARD;
        if (reverse) encoding_flags |= io::REVERSE_COMPLEMENT;


        log_visible(stderr, "opening read file \"%s\"\n", reads_name);
        SharedPointer<nvbio::io::SequenceDataStream> read_data_file(
            nvbio::io::open_sequence_file(
                reads_name,
                qencoding,
                uint32(-1),
                uint32(-1),
                io::SequenceEncoding( encoding_flags ) )
        );

        if (read_data_file == NULL || read_data_file->is_ok() == false)
        {
            log_error(stderr, "    failed opening file \"%s\"\n", reads_name);
            return false;
        }

        // output vectors
        PagedText<SYMBOL_SIZE,BIG_ENDIAN> bwt;
        SparseSymbolSet                   dollars;

        // get the current device
        int current_device;
        cudaGetDevice( &current_device );

        // build a BWTEContext
        BWTE_context_type bwte_context( current_device );

        // find out how big a block can we alloc
        uint32 max_block_suffixes = 256*1024*1024;
        uint32 max_block_strings  =  16*1024*1024;

        while (bwte_context.needed_device_memory( max_block_strings, max_block_suffixes ) + 256u*1024u*1024u >= free_device)
            max_block_suffixes /= 2;

        log_verbose(stderr, "  block size: %u\n", max_block_suffixes);

        // reserve enough space for the block processing
        bwte_context.reserve( max_block_strings, max_block_suffixes );

        cudaMemGetInfo(&free_device, &total_device);
        log_stats(stderr, "  device has %ld of %ld MB free\n", free_device/1024/1024, total_device/1024/1024);

        // build the input stage
        InputStage input_stage( read_data_file.get(), max_block_strings, max_block_suffixes - max_block_strings );

        // build the sort stage
        SortStage sort_stage( bwte_context );

        // build the sink
        SinkStage sink_stage( bwte_context, bwt, dollars );

        // build the pipeline
        Pipeline pipeline;
        const uint32 in0 = pipeline.append_stage( &input_stage, 4u );
        const uint32 in1 = pipeline.append_stage( &sort_stage, 4u );
        const uint32 out = pipeline.append_sink( &sink_stage );
        pipeline.add_dependency( in0, out );
        pipeline.add_dependency( in0, in1 );
        pipeline.add_dependency( in1, out );

        Timer timer;
        timer.start();

        // and run it!
        pipeline.run();

        log_info(stderr,"  writing output... started\n");

        // write out the results
        for (uint32 i = 0; i < bwt.page_count(); ++i)
        {
            // find the dollars corresponding to this page
            const uint64 page_begin = bwt.get_page_offset(i);
            const uint64 page_end   = bwt.get_page_offset(i+1);

            const uint64 dollars_begin = nvbio::lower_bound_index(
                page_begin,
                dollars.pos(),
                dollars.size() );

            const uint64 dollars_end = nvbio::lower_bound_index(
                page_end,
                dollars.pos(),
                dollars.size() );

            //log_debug(stderr,"    page[%u] : %llu symbols (%llu,%llu), %llu dollars (%llu,%llu)\n", i, page_end - page_begin, page_begin, page_end, dollars_end - dollars_begin, dollars_begin, dollars_end);

            // and output the page
            output_handler->process(
                bwt.get_page_size(i),
                SYMBOL_SIZE,
                (const uint32*)bwt.get_page(i),
                dollars_end - dollars_begin,
                dollars.pos() + dollars_begin,
                dollars.ids() + dollars_begin );
        }

        log_info(stderr,"  writing output... done\n");

        timer.stop();
        const float time = timer.seconds();

        log_verbose(stderr,"  total time  : %.1fs\n", time);
        log_verbose(stderr,"  peak memory : %.1f GB\n", float( peak_resident_memory() ) / float(1024*1024*1024));
        log_visible(stderr,"nvSetBWT... done\n");
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
