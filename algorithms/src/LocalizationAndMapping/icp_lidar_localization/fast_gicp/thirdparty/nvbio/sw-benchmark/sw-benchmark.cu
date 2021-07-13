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

// sw-benchmark.cu
//

//
// This project supports comparisons against multi-core SSE-enabled CPUs using
// conditional compilation of the SSW library:
//
//  https://github.com/mengyao/Complete-Striped-Smith-Waterman-Library
//
// In order to perform these additional tests, the user must download ssw.h and ssw.c
// from the above repository, copy them in the sw-benchmark directory, and run cmake with
// the option -DSSWLIB=ON.
//
#if defined(SSWLIB)
#include "ssw.h"
#include <omp.h>
#endif

#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/packedstream_loader.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/shared_pointer.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/fasta/fasta.h>
#include <nvbio/basic/dna.h>
#include <nvbio/alignment/alignment.h>
#include <nvbio/alignment/batched.h>
#include <nvbio/alignment/sink.h>
#include <thrust/device_vector.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

enum { MAX_READ_LENGTH = 1024 };

using namespace nvbio;

enum { CACHE_SIZE = 64 };
typedef nvbio::lmem_cache_tag<CACHE_SIZE>                                       lmem_cache_tag_type;
typedef nvbio::uncached_tag                                                     uncached_tag_type;

enum { REF_BITS       = 2 };
enum { REF_BIG_ENDIAN = false };

//
// An alignment stream class to be used in conjunction with the BatchAlignmentScore class
//
template <typename t_aligner_type, typename cache_type = lmem_cache_tag_type>
struct AlignmentStream
{
    typedef t_aligner_type                                                          aligner_type;

    typedef nvbio::cuda::ldg_pointer<uint32>                                        storage_iterator;

    typedef nvbio::PackedStringLoader<
        storage_iterator,
        io::SequenceDataTraits<DNA_N>::SEQUENCE_BITS,
        io::SequenceDataTraits<DNA_N>::SEQUENCE_BIG_ENDIAN,cache_type>                          pattern_loader_type;
    typedef typename pattern_loader_type::input_iterator                                        uncached_pattern_iterator;
    typedef typename pattern_loader_type::iterator                                              pattern_iterator;
    typedef nvbio::vector_view<pattern_iterator>                                                pattern_string;

    typedef nvbio::PackedStringLoader<
        storage_iterator,
        REF_BITS,
        REF_BIG_ENDIAN,uncached_tag_type>                                                       text_loader_type;
    typedef typename text_loader_type::input_iterator                                           uncached_text_iterator;
    typedef typename text_loader_type::iterator                                                 text_iterator;
    typedef nvbio::vector_view<text_iterator>                                                   text_string;

    // an alignment context
    struct context_type
    {
        int32                   min_score;
        aln::BestSink<int32>    sink;
    };
    // a container for the strings to be aligned
    struct strings_type
    {
        pattern_loader_type         pattern_loader;
        text_loader_type            text_loader;
        pattern_string              pattern;
        aln::trivial_quality_string quals;
        text_string                 text;
    };

    // constructor
    AlignmentStream(
        aligner_type        _aligner,
        const uint32        _count,
        const uint32*       _offsets,
        const uint32*       _patterns,
        const uint32        _max_pattern_len,
        const uint32        _total_pattern_len,
        const uint32*       _text,
        const uint32        _text_len,
               int16*       _scores) :
        m_aligner           ( _aligner ),
        m_count             (_count),
        m_max_pattern_len   (_max_pattern_len),
        m_total_pattern_len (_total_pattern_len),
        m_text_len          (_text_len),
        m_offsets           (_offsets),
        m_patterns          (storage_iterator(_patterns)),
        m_text              (storage_iterator(_text)),
        m_scores            (_scores) {}

    // get the aligner
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const aligner_type& aligner() const { return m_aligner; };

    // return the maximum pattern length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_pattern_length() const { return m_max_pattern_len; }

    // return the maximum text length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_text_length() const { return m_text_len; }

    // return the stream size
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_count; }

    // return the i-th pattern's length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 pattern_length(const uint32 i, context_type* context) const { return m_offsets[i+1] - m_offsets[i]; }

    // return the i-th text's length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 text_length(const uint32 i, context_type* context) const { return m_text_len; }

    // return the total number of cells
    uint64 cells() const { return uint64( m_total_pattern_len ) * uint64( m_text_len ); }

    // initialize the i-th context
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool init_context(
        const uint32    i,
        context_type*   context) const
    {
        context->min_score = Field_traits<int32>::min();
        return true;
    }

    // initialize the i-th context
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void load_strings(
        const uint32        i,
        const uint32        window_begin,
        const uint32        window_end,
        const context_type* context,
              strings_type* strings) const
    {
        const uint32 offset = m_offsets[i];
        const uint32 length = m_offsets[i+1] - offset;

        strings->text = text_string( m_text_len,
            strings->text_loader.load(
                m_text,
                m_text_len,
                make_uint2( window_begin, window_end ),
                false ) );

        strings->pattern = pattern_string( length,
            strings->pattern_loader.load( m_patterns + offset, length ) );
    }

    // handle the output
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void output(
        const uint32        i,
        const context_type* context) const
    {
        // copy the output score
        m_scores[i] = context->sink.score;
    }

    aligner_type                m_aligner;
    uint32                      m_count;
    uint32                      m_max_pattern_len;
    uint32                      m_total_pattern_len;
    uint32                      m_text_len;
    const uint32*               m_offsets;
    uncached_pattern_iterator   m_patterns;
    uncached_text_iterator      m_text;
    int16*                      m_scores;
};

// A simple kernel to test the speed of alignment without the possible overheads of the BatchAlignmentScore interface
//
template <uint32 BLOCKDIM, uint32 MAX_PATTERN_LEN, typename aligner_type, typename score_type>
__global__ void alignment_test_kernel(
    const aligner_type aligner,
    const uint32        N_probs,
    const uint32*       offsets,
    const uint32*       pattern_ptr,
    const uint32        text_len,
    const uint32*       text_ptr,
    score_type*         score)
{
    const uint32 tid = blockIdx.x * BLOCKDIM + threadIdx.x;

    typedef lmem_cache_tag_type                                                 lmem_cache_type;
    typedef nvbio::cuda::ldg_pointer<uint32>                                    storage_iterator;

    typedef nvbio::PackedStringLoader<
        storage_iterator,
        io::SequenceDataTraits<DNA_N>::SEQUENCE_BITS,
        io::SequenceDataTraits<DNA_N>::SEQUENCE_BIG_ENDIAN,lmem_cache_type>                     pattern_loader_type;
    typedef typename pattern_loader_type::input_iterator                                        uncached_pattern_iterator;
    typedef typename pattern_loader_type::iterator                                              pattern_iterator;
    typedef nvbio::vector_view<pattern_iterator>                                                pattern_string;

    typedef nvbio::PackedStringLoader<
        storage_iterator,
        REF_BITS,
        REF_BIG_ENDIAN,uncached_tag_type>                                                       text_loader_type;
    typedef typename text_loader_type::input_iterator                                           uncached_text_iterator;
    typedef typename text_loader_type::iterator                                                 text_iterator;
    typedef nvbio::vector_view<text_iterator>                                                   text_string;

    if (tid >= N_probs)
        return;

    const uint32 pattern_off = offsets[tid];
    const uint32 pattern_len = offsets[tid+1] - pattern_off;

    pattern_loader_type pattern_loader;
    pattern_string pattern = pattern_string( pattern_len, pattern_loader.load( uncached_pattern_iterator( pattern_ptr ) + pattern_off, pattern_len ) );

    text_loader_type text_loader;
    text_string text = text_string( text_len, text_loader.load( uncached_text_iterator( text_ptr ), text_len ) );

    aln::BestSink<int32> sink;

    aln::alignment_score<MAX_PATTERN_LEN>(
        aligner,
        pattern,
        aln::trivial_quality_string(),
        text,
        Field_traits<int32>::min(),
        sink );

    score[tid] = sink.score;
}

unsigned char nst_nt4_table[256] = {
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 5 /*'-'*/, 4, 4,
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 0, 4, 1,  4, 4, 4, 2,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  3, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 0, 4, 1,  4, 4, 4, 2,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  3, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4
};

struct ReferenceCounter
{
    ReferenceCounter() : m_size(0) {}

    void begin_read() {}
    void end_read() {}

    void id(const uint8 c) {}
    void read(const uint8 c) { ++m_size; }

    uint32 m_size;
};
struct ReferenceCoder
{
    typedef PackedStream<uint32*,uint8,2,false> stream_type;

    ReferenceCoder(uint32* storage) :
        m_size(0), m_stream( storage )
    {}

    void begin_read() {}
    void end_read() {}

    void id(const uint8 c) {}

    void read(const uint8 s)
    {
        const uint8 c = nst_nt4_table[s];

        m_stream[ m_size++ ] = c < 4 ? c : 0;
    }

    uint32      m_size;
    stream_type m_stream;
};


// execute a given batch alignment type on a given stream
//
// \tparam batch_type               a \ref BatchAlignment "Batch Alignment"
// \tparam stream_type              a stream compatible to the given batch_type
//
// \return                          average time
//
template <typename batch_type, typename stream_type>
float enact_batch(
          batch_type&               batch,
    const stream_type&              stream)
{
    // allow to alloc all the needed temporary storage
    const uint64 temp_size = batch_type::max_temp_storage(
        stream.max_pattern_length(),
        stream.max_text_length(),
        stream.size() );

    Timer timer;
    timer.start();

    // enact the batch
    batch.enact( stream, temp_size, NULL );

    cudaDeviceSynchronize();

    timer.stop();

    return timer.seconds();
}

// execute and time a batch of full DP alignments using BatchAlignmentScore
//
template <typename scheduler_type, typename stream_type>
void batch_score_profile(
    const stream_type               stream)
{
    typedef aln::BatchedAlignmentScore<stream_type, scheduler_type> batch_type;  // our batch type

    // setup a batch
    batch_type batch;

    const float time = enact_batch(
        batch,
        stream );

    fprintf(stderr,"  %5.1f", 1.0e-9f * float(stream.cells())/time );
}

// execute and time the batch_score<scheduler> algorithm for all possible schedulers
//
template <typename aligner_type>
void batch_score_profile_all(
    const aligner_type                      aligner,
    const uint32                            n_tasks,
    const uint32*                           offsets_dvec,
    const uint32*                           pattern_dvec,
    const uint32                            max_pattern_len,
    const uint32                            total_pattern_len,
    const uint32*                           text_dvec,
    const uint32                            text_len,
    int16*                                  score_dvec)
{
    {
        typedef AlignmentStream<aligner_type> stream_type;

        // create a stream
        stream_type stream(
            aligner,
            n_tasks,
            offsets_dvec,
            pattern_dvec,
            max_pattern_len,
            total_pattern_len,
            text_dvec,
            text_len,
            score_dvec );

        // test the DeviceThreadScheduler
        batch_score_profile<aln::DeviceThreadScheduler>( stream );

        // test the DeviceStagedThreadScheduler
        //batch_score_profile<aln::DeviceStagedThreadScheduler>( stream );
    }
    {
        const uint32 BLOCKDIM = 128;
        const uint32 N_BLOCKS = (n_tasks + BLOCKDIM-1) / BLOCKDIM;

        Timer timer;
        timer.start();

        // enact the batch
        alignment_test_kernel<BLOCKDIM,MAX_READ_LENGTH> <<<N_BLOCKS,BLOCKDIM>>>(
            aligner,
            n_tasks,
            offsets_dvec,
            pattern_dvec,
            text_len,
            text_dvec,
            score_dvec );

        cudaDeviceSynchronize();

        timer.stop();

        const float time = timer.seconds();

        fprintf(stderr,"  %5.1f", 1.0e-9f * float(uint64(total_pattern_len)*uint64(text_len))/time );
    }
    fprintf(stderr, " GCUPS\n");
}

enum AlignmentTest
{
    ALL                 = 0xFFFFFFFFu,
    ED                  = 1u,
    SW                  = 2u,
    GOTOH               = 4u,
    ED_BANDED           = 8u,
    SW_BANDED           = 16u,
    GOTOH_BANDED        = 32u,
    SSW                 = 64u
};

int main(int argc, char* argv[])
{
    uint32 TEST_MASK        = 0xFFFFFFFFu;

    const char* reads_name  = argv[argc-2];
    const char* ref_name    = argv[argc-1];
    uint32      threads     = omp_get_num_procs();
    io::QualityEncoding qencoding = io::Phred33;

    for (int i = 0; i < argc-2; ++i)
    {
        if (strcmp( argv[i], "-tests" ) == 0)
        {
            const std::string tests_string( argv[++i] );

            char temp[256];
            const char* begin = tests_string.c_str();
            const char* end   = begin;

            TEST_MASK = 0u;

            while (1)
            {
                while (*end != ':' && *end != '\0')
                {
                    temp[end - begin] = *end;
                    end++;
                }

                temp[end - begin] = '\0';

                if (strcmp( temp, "ed" ) == 0)
                    TEST_MASK |= ED;
                else if (strcmp( temp, "sw" ) == 0)
                    TEST_MASK |= SW;
                else if (strcmp( temp, "gotoh" ) == 0)
                    TEST_MASK |= GOTOH;
                else if (strcmp( temp, "ssw" ) == 0)
                    TEST_MASK |= SSW;

                if (*end == '\0')
                    break;

                ++end; begin = end;
            }
        }
        else if (strcmp( argv[i], "-threads" ) == 0)
            threads = atoi( argv[++i] );
    }

    fprintf(stderr,"sw-benchmark... started\n");

    log_visible(stderr, "opening read file \"%s\"\n", reads_name);
    SharedPointer<nvbio::io::SequenceDataStream> read_data_file(
        nvbio::io::open_sequence_file(reads_name,
                                  qencoding)
    );

    log_visible(stderr, "reading reference file \"%s\"... started\n", ref_name);

    // read the reference
    thrust::host_vector<uint32> h_ref_storage;
    uint32                      ref_length;
    uint32                      ref_words;
    {
        ReferenceCounter counter;

        FASTA_inc_reader fasta( ref_name );
        if (fasta.valid() == false)
        {
            fprintf(stderr, "  error: unable to open reference file \"%s\"\n", ref_name);
            exit(1);
        }
        while (fasta.read( 1024, counter ) == 1024);

        ref_length = counter.m_size;
        ref_words  = (ref_length + 15)/16; // # of words at 2 bits per symbol
    }
    {
        h_ref_storage.resize( ref_words );
        ReferenceCoder coder( &h_ref_storage[0] );

        FASTA_inc_reader fasta( ref_name );
        if (fasta.valid() == false)
        {
            fprintf(stderr, "  error: unable to open reference file \"%s\"\n", ref_name);
            exit(1);
        }
        while (fasta.read( 1024, coder ) == 1024);
    }
    log_visible(stderr, "reading reference file \"%s\"... done (%u bps)\n", ref_name, ref_length);

    typedef PackedStream<uint32*,uint8,REF_BITS,REF_BIG_ENDIAN> ref_stream_type;

    thrust::device_vector<uint32> d_ref_storage( h_ref_storage );
    ref_stream_type d_ref_stream( nvbio::raw_pointer( d_ref_storage ) );

    const uint32 batch_size = 256*1024;

    thrust::device_vector<int16> score_dvec( batch_size, 0 );

  #if defined(SSWLIB)
    std::vector<int8_t> unpacked_ref( ref_length );
    {
        ref_stream_type h_ref_stream( nvbio::raw_pointer( h_ref_storage ) );
        for (uint32 i = 0; i < ref_length; ++i)
            unpacked_ref[i] = h_ref_stream[i];
    }

    // Now set the number of threads
    omp_set_num_threads( threads );

    #pragma omp parallel
    {
        fprintf(stderr, "  running on multiple threads\n");
    }
  #endif

    io::SequenceDataHost h_read_data;

    while (io::next( DNA_N, &h_read_data, read_data_file.get(), batch_size ))
    {
        // build the device side representation
        const io::SequenceDataDevice d_read_data( h_read_data );

        const uint32 n_read_symbols = h_read_data.bps();

        fprintf(stderr,"  %u reads, avg: %u bps, max: %u bps\n",
            h_read_data.size(),
            h_read_data.avg_sequence_len(),
            h_read_data.max_sequence_len());

        if (TEST_MASK & GOTOH)
        {
            aln::SimpleGotohScheme scoring;
            scoring.m_match    =  2;
            scoring.m_mismatch = -1;
            scoring.m_gap_open = -2;
            scoring.m_gap_ext  = -1;

            fprintf(stderr,"  testing Gotoh scoring speed...\n");
            fprintf(stderr,"    %15s : ", "global");
            {
                batch_score_profile_all(
                    aln::make_gotoh_aligner<aln::GLOBAL,aln::TextBlockingTag>( scoring ),
                    d_read_data.size(),
                    nvbio::plain_view( d_read_data ).sequence_index(),
                    nvbio::plain_view( d_read_data ).sequence_storage(),
                    d_read_data.max_sequence_len(),
                    n_read_symbols,
                    nvbio::raw_pointer( d_ref_storage ),
                    ref_length,
                    nvbio::raw_pointer( score_dvec ) );
            }

            fprintf(stderr,"    %15s : ", "semi-global");
            {
                batch_score_profile_all(
                    aln::make_gotoh_aligner<aln::SEMI_GLOBAL,aln::TextBlockingTag>( scoring ),
                    d_read_data.size(),
                    nvbio::plain_view( d_read_data ).sequence_index(),
                    nvbio::plain_view( d_read_data ).sequence_storage(),
                    d_read_data.max_sequence_len(),
                    n_read_symbols,
                    nvbio::raw_pointer( d_ref_storage ),
                    ref_length,
                    nvbio::raw_pointer( score_dvec ) );
            }
            fprintf(stderr,"    %15s : ", "local");
            {
                batch_score_profile_all(
                    aln::make_gotoh_aligner<aln::LOCAL,aln::TextBlockingTag>( scoring ),
                    d_read_data.size(),
                    nvbio::plain_view( d_read_data ).sequence_index(),
                    nvbio::plain_view( d_read_data ).sequence_storage(),
                    d_read_data.max_sequence_len(),
                    n_read_symbols,
                    nvbio::raw_pointer( d_ref_storage ),
                    ref_length,
                    nvbio::raw_pointer( score_dvec ) );
            }
        }
        if (TEST_MASK & ED)
        {
            fprintf(stderr,"  testing Edit Distance scoring speed...\n");
            fprintf(stderr,"    %15s : ", "semi-global");
            {
                batch_score_profile_all(
                    aln::make_edit_distance_aligner<aln::SEMI_GLOBAL,aln::TextBlockingTag>(),
                    d_read_data.size(),
                    nvbio::plain_view( d_read_data ).sequence_index(),
                    nvbio::plain_view( d_read_data ).sequence_storage(),
                    d_read_data.max_sequence_len(),
                    n_read_symbols,
                    nvbio::raw_pointer( d_ref_storage ),
                    ref_length,
                    nvbio::raw_pointer( score_dvec ) );
            }
        }

        #if defined(SSWLIB)
        if (TEST_MASK & SSW)
        {
            fprintf(stderr,"  testing SSW scoring speed...\n");
            fprintf(stderr,"    %15s : ", "local");

            const int8_t mat[4*4] = {2, -1, -1, -1, -1, 2, -1, -1, -1, -1, 2, -1, -1, -1, -1, 2};

            std::vector<int8_t> unpacked_reads( n_read_symbols );

            typedef io::SequenceDataAccess<DNA_N>           read_access_type;
            typedef read_access_type::sequence_stream_type  read_stream_type;

            const read_access_type reads_access( h_read_data );

            const read_stream_type packed_reads( reads_access.sequence_stream() );

            #pragma omp parallel for
            for (int i = 0; i < int( n_read_symbols ); ++i)
                unpacked_reads[i] = packed_reads[i];

            Timer timer;
            timer.start();

            #pragma omp parallel for
            for (int i = 0; i < int( h_read_data.size() ); ++i)
            {
                const uint32 read_off = reads_access.sequence_index()[i];
                const uint32 read_len = reads_access.sequence_index()[i+1] - read_off;

                s_profile* prof = ssw_init( &unpacked_reads[read_off], read_len, mat, 4, 2 );

                s_align* align = ssw_align(
                    prof, 
					&unpacked_ref[0], 
					ref_length, 
					2, 
					2,
					0u,	
					0u,
					0,
					15 );

                align_destroy( align );

                init_destroy( prof );
            }

            timer.stop();
            const float time = timer.seconds();

            fprintf(stderr,"  %5.1f", 1.0e-9f * float(uint64(n_read_symbols)*uint64(ref_length))/time );
            fprintf(stderr, " GCUPS\n");
        }
        #endif
    }
    fprintf(stderr,"sw-benchmark... done\n");
    return 0;
}
