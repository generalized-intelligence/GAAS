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

// alignment_test.cu
//

#include <nvbio-test/alignment_test_utils.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/basic/cached_iterator.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/packedstream_loader.h>
#include <nvbio/basic/vector_view.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/shared_pointer.h>
#include <nvbio/basic/dna.h>
#include <nvbio/alignment/alignment.h>
#include <nvbio/alignment/batched.h>
#include <nvbio/alignment/sink.h>
#include <thrust/device_vector.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

using namespace nvbio;

namespace nvbio {
namespace aln {

enum { CACHE_SIZE = 32 };
typedef nvbio::lmem_cache_tag<CACHE_SIZE>                                       lmem_cache_tag_type;
typedef nvbio::uncached_tag                                                     uncached_tag_type;

//
// An alignment stream class to be used in conjunction with the BatchAlignmentScore class
//
template <typename t_aligner_type, uint32 M, uint32 N, typename cache_type = lmem_cache_tag_type>
struct AlignmentStream
{
    typedef t_aligner_type                                                          aligner_type;

    typedef nvbio::cuda::ldg_pointer<uint32>                                        storage_iterator;

    typedef nvbio::PackedStringLoader<storage_iterator,4,false,cache_type>          pattern_loader_type;
    typedef typename pattern_loader_type::input_iterator                            uncached_pattern_iterator;
    typedef typename pattern_loader_type::iterator                                  pattern_iterator;
    typedef nvbio::vector_view<pattern_iterator>                                    pattern_string;

    typedef nvbio::PackedStringLoader<storage_iterator,2,false,cache_type>          text_loader_type;
    typedef typename text_loader_type::input_iterator                               uncached_text_iterator;
    typedef typename text_loader_type::iterator                                     text_iterator;
    typedef nvbio::vector_view<text_iterator>                                       text_string;

    // an alignment context
    struct context_type
    {
        int32                   min_score;
        aln::BestSink<int32>    sink;
    };
    // a container for the strings to be aligned
    struct strings_type
    {
        pattern_loader_type     pattern_loader;
        text_loader_type        text_loader;
        pattern_string          pattern;
        trivial_quality_string  quals;
        text_string             text;
    };

    // constructor
    AlignmentStream(
        aligner_type        _aligner,
        const uint32        _count,
        const uint32*       _patterns,
        const uint32*       _text,
               int16*       _scores) :
        m_aligner( _aligner ), m_count(_count), m_patterns(storage_iterator(_patterns)), m_text(storage_iterator(_text)), m_scores(_scores) {}

    // get the aligner
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    const aligner_type& aligner() const { return m_aligner; };

    // return the maximum pattern length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_pattern_length() const { return M; }

    // return the maximum text length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 max_text_length() const { return N; }

    // return the stream size
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_count; }

    // return the i-th pattern's length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 pattern_length(const uint32 i, context_type* context) const { return M; }

    // return the i-th text's length
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 text_length(const uint32 i, context_type* context) const { return N; }

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
        strings->pattern = pattern_string( M,
            strings->pattern_loader.load(
                m_patterns + i * M,
                M,
                make_uint2( window_begin, window_end ),
                false ) );

        strings->text = text_string( N, strings->text_loader.load( m_text + i * N, N ) );
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
    uncached_pattern_iterator   m_patterns;
    uncached_text_iterator      m_text;
    int16*                      m_scores;
};

// A simple kernel to test the speed of alignment without the possible overheads of the BatchAlignmentScore interface
//
template <uint32 BLOCKDIM, uint32 MAX_REF_LEN, typename aligner_type, typename score_type>
__global__ void alignment_test_kernel(const aligner_type aligner, const uint32 N_probs, const uint32 M, const uint32 N, const uint32* strptr, const uint32* refptr, score_type* score)
{
    const uint32 tid = blockIdx.x * BLOCKDIM + threadIdx.x;

    typedef lmem_cache_tag_type                                                 lmem_cache_type;
    typedef nvbio::cuda::ldg_pointer<uint32>                                    storage_iterator;

    typedef nvbio::PackedStringLoader<storage_iterator,4,false,lmem_cache_type>     pattern_loader_type;
    typedef typename pattern_loader_type::input_iterator                            uncached_pattern_iterator;
    typedef typename pattern_loader_type::iterator                                  pattern_iterator;
    typedef nvbio::vector_view<pattern_iterator>                                    pattern_string;

    typedef nvbio::PackedStringLoader<storage_iterator,2,false,lmem_cache_type>     text_loader_type;
    typedef typename text_loader_type::input_iterator                               uncached_text_iterator;
    typedef typename text_loader_type::iterator                                     text_iterator;
    typedef nvbio::vector_view<text_iterator>                                       text_string;

    pattern_loader_type pattern_loader;
    pattern_string pattern = pattern_string( M, pattern_loader.load( uncached_pattern_iterator( strptr ) + tid * M, tid < N_probs ? M : 0u ) );

    text_loader_type text_loader;
    text_string text = text_string( N, text_loader.load( uncached_text_iterator( refptr ) + tid * N, tid < N_probs ? N : 0u ) );

    aln::BestSink<int32> sink;

    aln::alignment_score<MAX_REF_LEN>(
        aligner,
        pattern,
        aln::trivial_quality_string(),
        text,
        Field_traits<int32>::min(),
        sink );

    score[tid] = sink.score;
}

//
// A class for making a single alignment test, testing both scoring and traceback
//
struct SingleTest
{
    thrust::host_vector<uint8>   str_hvec;
    thrust::host_vector<uint8>   ref_hvec;
    thrust::device_vector<uint8> str_dvec;
    thrust::device_vector<uint8> ref_dvec;
    thrust::device_vector<float> temp_dvec;
    thrust::device_vector<float> score_dvec;
    thrust::device_vector<uint2> sink_dvec;

    // test full DP alignment
    //
    // \param test              test name
    // \param aligner           alignment algorithm
    // \param ref_alignment     reference alignment string
    //
    template <uint32 BLOCKDIM, uint32 N, uint32 M, typename aligner_type>
    void full(const char* test, const aligner_type aligner, const char* ref_alignment)
    {
        NVBIO_VAR_UNUSED const uint32 CHECKPOINTS = 32u;

        typedef ScoreMatrices<N,M,typename aligner_type::aligner_tag> SWMatrices;

        SharedPointer<SWMatrices> mat = SharedPointer<SWMatrices>( new SWMatrices() );

        const uint8* str_hptr = nvbio::raw_pointer( str_hvec );
        const uint8* ref_hptr = nvbio::raw_pointer( ref_hvec );

        typename column_storage_type<aligner_type>::type column[N];

        const int32 ref_score = ref_sw<M,N>( str_hptr, ref_hptr, aligner, mat.get() );

        aln::BestSink<int32> sink;
        aln::alignment_score(
            aligner,
            vector_view<const uint8*>( M, str_hptr ),
            trivial_quality_string(),
            vector_view<const uint8*>( N, ref_hptr ),
            -1000,
            sink,
            column );

        const int32 cpu_score = sink.score;

        if (cpu_score != ref_score)
        {
            log_error(stderr, "    expected %s score %d, got: %d\n", test, ref_score, cpu_score);
            exit(1);
        }

        TestBacktracker backtracker;
        backtracker.clear();

        const Alignment<int32> aln = aln::alignment_traceback<1024u,1024u,CHECKPOINTS>(
            aligner,
            vector_view<const uint8*>( M, str_hptr ),
            trivial_quality_string(),
            vector_view<const uint8*>( N, ref_hptr ),
            -1000,
            backtracker );

        const int32 aln_score = backtracker.score( aligner, aln.source.x, str_hptr, ref_hptr );
        const std::string aln_string = rle( backtracker.aln ).c_str();
        if (aln_score != ref_score)
        {
            log_error(stderr, "    expected %s backtracking score %d, got %d\n", test, ref_score, aln_score);
            log_error(stderr, "    %s - %d - [%u, %u] x [%u, %u]\n", aln_string.c_str(), aln.score, aln.source.x, aln.sink.x, aln.source.y, aln.sink.y);
            //mat->print();
            exit(1);
        }
        fprintf(stderr, "    %15s : ", test);
        fprintf(stderr, "%d - %s - [%u:%u] x [%u:%u]\n", aln.score, aln_string.c_str(), aln.source.x, aln.sink.x, aln.source.y, aln.sink.y);
        if (strcmp( ref_alignment, aln_string.c_str() ) != 0)
        {
            log_error(stderr, "    expected %s, got %s\n", ref_alignment, aln_string.c_str());
            exit(1);
        }
    }

    // test banded alignment
    //
    // \param test              test name
    // \param aligner           alignment algorithm
    // \param ref_alignment     reference alignment string
    //
    template <uint32 BLOCKDIM, uint32 BAND_LEN, const uint32 N, const uint32 M, typename aligner_type>
    void banded(const char* test, const aligner_type aligner, const char* ref_alignment)
    {
        NVBIO_VAR_UNUSED const uint32 CHECKPOINTS = 32u;

        const uint8* str_hptr = nvbio::raw_pointer( str_hvec );
        const uint8* ref_hptr = nvbio::raw_pointer( ref_hvec );

        const int32 ref_score = ref_banded_sw<M,N,BAND_LEN>( str_hptr, ref_hptr, 0u, aligner );

        aln::BestSink<int32> sink;
        aln::banded_alignment_score<BAND_LEN>(
            aligner,
            vector_view<const uint8*>( M, str_hptr ),
            trivial_quality_string(),
            vector_view<const uint8*>( N, ref_hptr ),
            -1000,
            sink );

        const int32 cpu_score = sink.score;
        if (cpu_score != ref_score)
        {
            log_error(stderr, "    expected %s score %d, got: %d\n", test, ref_score, cpu_score);
            exit(1);
        }

        TestBacktracker backtracker;
        backtracker.clear();

        const Alignment<int32> aln = aln::banded_alignment_traceback<BAND_LEN,1024u,CHECKPOINTS>(
            aligner,
            vector_view<const uint8*>( M, str_hptr ),
            trivial_quality_string(),
            vector_view<const uint8*>( N, ref_hptr ),
            -1000,
            backtracker );

        const int32 aln_score = backtracker.score( aligner, aln.source.x, str_hptr, ref_hptr );
        const std::string aln_string = rle( backtracker.aln ).c_str();
        if (aln_score != ref_score)
        {
            log_error(stderr, "    expected %s backtracking score %d, got %d\n", ref_score, aln_score);
            log_error(stderr, "    %s - %d - [%u, %u] x [%u, %u]\n", aln_string.c_str(), aln.score, aln.source.x, aln.sink.x, aln.source.y, aln.sink.y);
            exit(1);
        }
        fprintf(stderr, "    %15s : ", test);
        fprintf(stderr, "%d - %s - [%u:%u] x [%u:%u]\n", aln.score, aln_string.c_str(), aln.source.x, aln.sink.x, aln.source.y, aln.sink.y);
        if (strcmp( ref_alignment, aln_string.c_str() ) != 0)
        {
            log_error(stderr, "    expected %s, got %s\n", ref_alignment, aln_string.c_str());
            exit(1);
        }
    }
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
    const stream_type&              stream,
    const uint32                    n_tests,
    const uint32                    n_tasks)
{
    // alloc all the needed temporary storage
    const uint64 temp_size = batch_type::max_temp_storage(
        stream.max_pattern_length(),
        stream.max_text_length(),
        stream.size() );

    thrust::device_vector<uint8> temp_dvec( temp_size );

    Timer timer;
    timer.start();

    for (uint32 i = 0; i < n_tests; ++i)
    {
        // enact the batch
        batch.enact( stream, temp_size, nvbio::raw_pointer( temp_dvec ) );

        cudaDeviceSynchronize();
    }

    timer.stop();

    return timer.seconds() / float(n_tests);
}

// execute and time a batch of full DP alignments using BatchAlignmentScore
//
template <bool supported, typename scheduler_type, uint32 N, uint32 M, typename stream_type>
struct batch_score_profile_dispatch
{
    static void run(
        const stream_type               stream,
        const uint32                    n_tests,
        const uint32                    n_tasks)
    {}
};

// execute and time a batch of full DP alignments using BatchAlignmentScore
//
template <typename scheduler_type, uint32 N, uint32 M, typename stream_type>
struct batch_score_profile_dispatch<true,scheduler_type,N,M,stream_type>
{
    static void run(
        const stream_type               stream,
        const uint32                    n_tests,
        const uint32                    n_tasks)
    {
        typedef aln::BatchedAlignmentScore<stream_type, scheduler_type> batch_type;  // our batch type

        // setup a batch
        batch_type batch;

        const float time = enact_batch(
            batch,
            stream,
            n_tests,
            n_tasks );

        fprintf(stderr,"  %5.1f", 1.0e-9f * float(n_tasks*uint64(N*M))/time );
    }
};

// execute and time a batch of full DP alignments using BatchAlignmentScore
//
template <typename scheduler_type, uint32 N, uint32 M, typename stream_type>
void batch_score_profile(
    const stream_type               stream,
    const uint32                    n_tests,
    const uint32                    n_tasks)
{
    NVBIO_VAR_UNUSED const bool is_supported = aln::supports_scheduler<typename stream_type::aligner_type,scheduler_type>::pred;

    batch_score_profile_dispatch<is_supported,scheduler_type,N,M,stream_type>::run(
        stream,
        n_tests,
        n_tasks );
}

// execute and time the batch_score<scheduler> algorithm for all possible schedulers
//
template <uint32 N, uint32 M, typename aligner_type>
void batch_score_profile_all(
    const aligner_type              aligner,
    const uint32                    n_tests,
    const uint32                    n_tasks,
    thrust::device_vector<uint32>&  pattern_dvec,
    thrust::device_vector<uint32>&  text_dvec,
    thrust::device_vector<int16>&   score_dvec)
{
    {
        typedef AlignmentStream<aligner_type,M,N> stream_type;

        // create a stream
        stream_type stream(
            aligner,
            n_tasks,
            nvbio::raw_pointer( pattern_dvec ),
            nvbio::raw_pointer( text_dvec ),
            nvbio::raw_pointer( score_dvec ) );

        // test the DeviceThreadScheduler
        batch_score_profile<DeviceThreadScheduler,N,M>(
            stream,
            n_tests,
            n_tasks );

        // test the DeviceStagedThreadScheduler
        batch_score_profile<DeviceStagedThreadScheduler,N,M>(
            stream,
            n_tests,
            n_tasks );
    }
    {
        typedef AlignmentStream<aligner_type,M,N,uncached_tag_type> stream_type;

        // create a stream
        stream_type stream(
            aligner,
            n_tasks,
            nvbio::raw_pointer( pattern_dvec ),
            nvbio::raw_pointer( text_dvec ),
            nvbio::raw_pointer( score_dvec ) );

        // test the DeviceWarpScheduler
        batch_score_profile<DeviceWarpScheduler,N,M>(
            stream,
            n_tests,
            n_tasks );
    }
    {
        const uint32 BLOCKDIM = 128;
        const uint32 N_BLOCKS = (n_tasks + BLOCKDIM-1) / BLOCKDIM;

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
        {
            // enact the batch
            alignment_test_kernel<BLOCKDIM,N> <<<N_BLOCKS,BLOCKDIM>>>(
                aligner,
                n_tasks,
                M,
                N,
                nvbio::raw_pointer( pattern_dvec ),
                nvbio::raw_pointer( text_dvec ),
                nvbio::raw_pointer( score_dvec ) );

            cudaDeviceSynchronize();
        }

        timer.stop();

        const float time = timer.seconds();

        fprintf(stderr,"  %5.1f", 1.0e-9f * float(n_tasks*uint64(N*M))*(float(n_tests)/time) );
    }
    fprintf(stderr, " GCUPS\n");
}

// execute and time a batch of banded alignments using BatchBandedAlignmentScore
//
template <uint32 BAND_LEN, typename scheduler_type, uint32 N, uint32 M, typename stream_type>
void batch_banded_score_profile(
    const stream_type               stream,
    const uint32                    n_tests,
    const uint32                    n_tasks)
{
    typedef aln::BatchedBandedAlignmentScore<BAND_LEN,stream_type, scheduler_type> batch_type;  // our batch type

    // setup a batch
    batch_type batch;

    const float time = enact_batch(
        batch,
        stream,
        n_tests,
        n_tasks );

    fprintf(stderr,"  %5.1f", 1.0e-9f * float(n_tasks*uint64(BAND_LEN*M))*(float(n_tests)/time) );
}
// execute and time the batch_banded_score<scheduler> algorithm for all possible schedulers
//
template <uint32 BAND_LEN, uint32 N, uint32 M, typename aligner_type>
void batch_banded_score_profile_all(
    const aligner_type              aligner,
    const uint32                    n_tests,
    const uint32                    n_tasks,
    thrust::device_vector<uint32>&  pattern_dvec,
    thrust::device_vector<uint32>&  text_dvec,
    thrust::device_vector<int16>&   score_dvec)
{
    typedef AlignmentStream<aligner_type,M,N> stream_type;

    // create a stream
    stream_type stream(
        aligner,
        n_tasks,
        nvbio::raw_pointer( pattern_dvec ),
        nvbio::raw_pointer( text_dvec ),
        nvbio::raw_pointer( score_dvec ) );

    // test the DeviceThreadScheduler
    batch_banded_score_profile<BAND_LEN,DeviceThreadScheduler,N,M>(
        stream,
        n_tests,
        n_tasks );

    // test the DeviceStagedThreadScheduler
    batch_banded_score_profile<BAND_LEN,DeviceStagedThreadScheduler,N,M>(
        stream,
        n_tests,
        n_tasks );

    // TODO: test DeviceWarpScheduler
    fprintf(stderr, " GCUPS\n");
}

// a simple banded edit distance test
//
template <typename string_type>
void banded_edit_distance_test(
    const uint32      test_id,
    const string_type pattern,
    const string_type text,
    const int32       ref_score)
{
    const int32 ed = banded_alignment_score<5>(
        make_edit_distance_aligner<aln::SEMI_GLOBAL>(),
        pattern,
        text,
        -255 );

    if (ed != ref_score)
    {
        log_error(stderr, "  synthetic Edit Distance test %u... failed\n", test_id);
        log_error(stderr, "    expected %d, got: %d - pattern: %s text: %s\n", ref_score, ed, pattern.begin(), text.begin());
        exit(1);
    }
    else
        fprintf(stderr, "  synthetic Edit Distance test %u... passed!\n", test_id);
}

void test(int argc, char* argv[])
{
                     uint32 n_tests          = 1;
    NVBIO_VAR_UNUSED uint32 N_WARP_TASKS     = 4096;
                     uint32 N_THREAD_TASKS   = 128*1024;
                     uint32 TEST_MASK        = 0xFFFFFFFFu;

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp( argv[i], "-N-thread-tasks" ) == 0)
            N_THREAD_TASKS = atoi( argv[++i] );
        else if (strcmp( argv[i], "-N-warp-tasks" ) == 0)
            N_WARP_TASKS = atoi( argv[++i] );
        else if (strcmp( argv[i], "-N-tests" ) == 0)
            n_tests = atoi( argv[++i] );
        else if (strcmp( argv[i], "-tests" ) == 0)
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

                if (strcmp( temp, "functional" ) == 0)
                    TEST_MASK |= FUNCTIONAL;
                else if (strcmp( temp, "ed" ) == 0)
                    TEST_MASK |= ED;
                else if (strcmp( temp, "ed-banded" ) == 0)
                    TEST_MASK |= ED_BANDED;
                else if (strcmp( temp, "sw" ) == 0)
                    TEST_MASK |= SW;
                else if (strcmp( temp, "sw-banded" ) == 0)
                    TEST_MASK |= SW_BANDED;
                else if (strcmp( temp, "sw-warp" ) == 0)
                    TEST_MASK |= SW_WARP;
                else if (strcmp( temp, "sw-striped" ) == 0)
                    TEST_MASK |= SW_STRIPED;
                else if (strcmp( temp, "gotoh" ) == 0)
                    TEST_MASK |= GOTOH;
                else if (strcmp( temp, "gotoh-banded" ) == 0)
                    TEST_MASK |= GOTOH_BANDED;

                if (*end == '\0')
                    break;

                ++end; begin = end;
            }
        }
    }

    fprintf(stderr,"testing alignment... started\n");

    if (TEST_MASK & FUNCTIONAL)
    {
        typedef vector_view<const char*> const_string;

        // right aligned, no gaps
        {
            const_string  text = make_string("AAAAGGGTGCTCAA");
            const_string  pattern  = make_string("GGGTGCTCAA");

            banded_edit_distance_test(
                1u,         // test id
                pattern,    // pattern
                text,       // text
                0 );        // expected score
        }
        // right aligned, 2 insertions
        {
            const_string  text = make_string("AAAAGGGTGCTCAA");
            const_string  pattern  = make_string("GGGTAAGCTC");

            banded_edit_distance_test(
                2u,         // test id
                pattern,    // pattern
                text,       // text
                -2 );       // expected score
        }
        // right aligned, 2 deletions
        {
            const_string   text = make_string("AAAAGGGTGCAATC");
            const_string pattern  = make_string("AAGGGTGCTC");

            banded_edit_distance_test(
                3u,         // test id
                pattern,    // pattern
                text,       // text
                -2 );       // expected score
        }
        // left aligned, zero gaps
        {
            const_string     text = make_string("AAAAGGGTGCTCAA");
            const_string pattern  = make_string("AAAAGGGTGC");

            banded_edit_distance_test(
                4u,         // test id
                pattern,    // pattern
                text,       // text
                0 );        // expected score
        }
        // left aligned, 2 deletions
        {
            const_string     text = make_string("AAAAGGAAGTGCTC");
            const_string pattern  = make_string("AAAAGGGTG");

            banded_edit_distance_test(
                5u,         // test id
                pattern,    // pattern
                text,       // text
                -2 );       // expected score
        }
        // centrally aligned, 2 insertions
        {
            const_string   text = make_string("AACAGGGTGCTC");
            const_string pattern  = make_string("CACCGGGT");

            banded_edit_distance_test(
                6u,         // test id
                pattern,    // pattern
                text,       // text
                -2 );       // expected score
        }
    }

    if (TEST_MASK & FUNCTIONAL)
    {
        NVBIO_VAR_UNUSED const uint32 BLOCKDIM = 128;
        const uint32 M = 7;
        const uint32 N = 20;

        thrust::host_vector<uint8> str_hvec( M );
        thrust::host_vector<uint8> ref_hvec( N );

        uint8* str_hptr = nvbio::raw_pointer( str_hvec );
        uint8* ref_hptr = nvbio::raw_pointer( ref_hvec );

        string_to_dna("ACAACTA", str_hptr);
        string_to_dna("AAACACCCTAACACACTAAA", ref_hptr);

        SingleTest test;
        nvbio::cuda::thrust_copy_vector(test.str_hvec, str_hvec);
        nvbio::cuda::thrust_copy_vector(test.ref_hvec, ref_hvec);
        nvbio::cuda::thrust_copy_vector(test.str_dvec, str_hvec);
        nvbio::cuda::thrust_copy_vector(test.ref_dvec, ref_hvec);

        {
            fprintf(stderr,"  testing Smith-Waterman scoring...\n");
            aln::SimpleSmithWatermanScheme scoring;
            scoring.m_match     =  2;
            scoring.m_mismatch  = -1;
            scoring.m_deletion  = -1;
            scoring.m_insertion = -1;

            test.full<BLOCKDIM,N,M>(      "global",  make_smith_waterman_aligner<aln::GLOBAL>( scoring ),      "1M2D3M1D3M10D" );
            test.full<BLOCKDIM,N,M>(       "local",  make_smith_waterman_aligner<aln::LOCAL>( scoring ),       "4M1D3M" );
            test.full<BLOCKDIM,N,M>( "semi-global",  make_smith_waterman_aligner<aln::SEMI_GLOBAL>( scoring ), "4M1D3M" );
        }
        {
            fprintf(stderr,"  testing Gotoh scoring...\n");
            aln::SimpleGotohScheme scoring;
            scoring.m_match    =  2;
            scoring.m_mismatch = -1;
            scoring.m_gap_open = -1;
            scoring.m_gap_ext  = -1;

            test.full<BLOCKDIM,N,M>(      "global", make_gotoh_aligner<aln::GLOBAL>( scoring ),      "1M2D3M1D3M10D" );
            test.full<BLOCKDIM,N,M>(       "local", make_gotoh_aligner<aln::LOCAL>( scoring ),       "4M1D3M" );
            test.full<BLOCKDIM,N,M>( "semi-global", make_gotoh_aligner<aln::SEMI_GLOBAL>( scoring ), "4M1D3M" );
            test.banded<BLOCKDIM, 7u, N, M>( "banded-semi-global", make_gotoh_aligner<aln::SEMI_GLOBAL>( scoring ), "4M1D3M" );
        }
    }

    if (TEST_MASK & FUNCTIONAL)
    {
        fprintf(stderr,"  testing real banded Gotoh problem...\n");
        NVBIO_VAR_UNUSED const uint32 BLOCKDIM = 128;
        NVBIO_VAR_UNUSED const uint32 BAND_LEN = 31;
        NVBIO_VAR_UNUSED const uint32 M = 150;
        NVBIO_VAR_UNUSED const uint32 N = 150 + 31;

        thrust::host_vector<uint8> str_hvec( M );
        thrust::host_vector<uint8> ref_hvec( N );

        uint8* str_hptr = nvbio::raw_pointer( str_hvec );
        uint8* ref_hptr = nvbio::raw_pointer( ref_hvec );
        string_to_dna("TTATGTAGGTGGTCTGGTTTTTGCCTTTTAAGCTTCTGCAAAAAACAACAACAAACTTGTGGTATTACACTGACTCTACAGATCAATTTGGGGACAACTTCCATGTGTTCCACCACCAATACTGAATCTTTCAATCGACTGACGTGGTAT", str_hptr);
        string_to_dna("ATCGGATTCTTTCTTACTTGTAGGTGGTCTGGTTTTTGCCTTTTAAGCTTCTGCAAAAAACAACAACAAACTTGTGGTATTACACTGACTCTACAGATCAATTTGGGGACAACTTCCATGTGTTCCACCACCAATACTGAATCTTTCAATCGACTGACGTGGTATCTCTCTCTCCATCTAT", ref_hptr);

        aln::SimpleGotohScheme scoring;
        scoring.m_match    =  0;
        scoring.m_mismatch = -5;
        scoring.m_gap_open = -8;
        scoring.m_gap_ext  = -3;

        SingleTest test;
        nvbio::cuda::thrust_copy_vector(test.str_hvec, str_hvec);
        nvbio::cuda::thrust_copy_vector(test.ref_hvec, ref_hvec);
        nvbio::cuda::thrust_copy_vector(test.str_dvec, str_hvec);
        nvbio::cuda::thrust_copy_vector(test.ref_dvec, ref_hvec);

        test.banded<BLOCKDIM, BAND_LEN, N, M>( "banded-semi-global", make_gotoh_aligner<aln::SEMI_GLOBAL>( scoring ), "147M2D3M" );
    }

    // This code is for debugging purposes, useful to plug-in and analyze real problems coming from an app
    if (TEST_MASK & FUNCTIONAL)
    {
        fprintf(stderr,"  testing real full-matrix Gotoh problem...\n");
        NVBIO_VAR_UNUSED const uint32 BLOCKDIM = 128;
        NVBIO_VAR_UNUSED const uint32 M = 144;
        NVBIO_VAR_UNUSED const uint32 N = 500;

        thrust::host_vector<uint8> str_hvec( M );
        thrust::host_vector<uint8> ref_hvec( N );

        uint8* str_hptr = nvbio::raw_pointer( str_hvec );
        uint8* ref_hptr = nvbio::raw_pointer( ref_hvec );

        const char* str_ascii =
            "TAGGAGGTAACATGTATGGAGCATTTACCATAGGCCAAGCACTGTTCTAAGAACTTCGGACATGTTATCTCACTTGTATAAGTACTTAGGTGCCTACAACATAAGCAGCACCTGGTAAATTAAGTATTGAAAAAATGCAGATCG";
        const char* ref_ascii =
            "CAGCACTGACCGGTGAGCATAAACCCTGGGGATGCCCAGAGCTGGTACAGCCAGGAGCTCCAGAAGCGTGGGATTCTCAGAGGGAAGTGGAGCTCACTGCTCTACAGGTCCTATTCAAGTTAGAAAGTAAGATACAATGCACACAAAGCCAAATTGTC"
            "ATCATTCAGCTCCTATTACAGGGGAACTAAGAGCTGCATTGAAAATTATTTGCAAAGCTTGTAAGTGGTTCTGCCACTTATTAGCCGTGTGAACCTTAGCAAATTACCTAGCGTCTCTGAGTTTCAACTTCCTCATCTACAAAATAGAAATGATAATAAT"
            "AACCGCATCGCAAGAGTTGTTGGAAAAATGAAAATGAGGTATCATAGGAGGTAACATGTATGGAGCATTTACCATAGGCCAAGCACTGTTCTAAGAACTTCGGACATGTTATCTCACTTGTATAAGTACTTAGGTGCCTACAACATAAACAGCACCTGGT"
            "AAATTAAGTATTGAAAAAATGC";

        string_to_dna( str_ascii, str_hptr );
        string_to_dna( ref_ascii, ref_hptr );

        aln::SimpleGotohScheme scoring;
        scoring.m_match    =  0;
        scoring.m_mismatch = -5;
        scoring.m_gap_open = -8;
        scoring.m_gap_ext  = -3;

        aln::GotohAligner<aln::SEMI_GLOBAL, aln::SimpleGotohScheme> aligner( scoring );

        SingleTest test;
        nvbio::cuda::thrust_copy_vector(test.str_hvec, str_hvec);
        nvbio::cuda::thrust_copy_vector(test.ref_hvec, ref_hvec);
        nvbio::cuda::thrust_copy_vector(test.str_dvec, str_hvec);
        nvbio::cuda::thrust_copy_vector(test.ref_dvec, ref_hvec);

        test.full<BLOCKDIM,N,M>( "semi-global", aligner, "6I138M" );
    }

    // This code is for debugging purposes, useful to plug-in and analyze real problems coming from an app
    if (TEST_MASK & FUNCTIONAL)
    {
        fprintf(stderr,"  testing real full-matrix Edit Distance problem...\n");
        NVBIO_VAR_UNUSED const uint32 BLOCKDIM = 128;
        NVBIO_VAR_UNUSED const uint32 M = 144;
        NVBIO_VAR_UNUSED const uint32 N = 500;

        thrust::host_vector<uint8> str_hvec( M );
        thrust::host_vector<uint8> ref_hvec( N );

        uint8* str_hptr = nvbio::raw_pointer( str_hvec );
        uint8* ref_hptr = nvbio::raw_pointer( ref_hvec );

        const char* str_ascii =
            "TAGGAGGTAACATGTATGGAGCATTTACCATAGGCCAAGCACTGTTCTAAGAACTTCGGACATGTTATCTCACTTGTATAAGTACTTAGGTGCCTACAACATAAGCAGCACCTGGTAAATTAAGTATTGAAAAAATGCAGATCG";
        const char* ref_ascii =
            "CAGCACTGACCGGTGAGCATAAACCCTGGGGATGCCCAGAGCTGGTACAGCCAGGAGCTCCAGAAGCGTGGGATTCTCAGAGGGAAGTGGAGCTCACTGCTCTACAGGTCCTATTCAAGTTAGAAAGTAAGATACAATGCACACAAAGCCAAATTGTC"
            "ATCATTCAGCTCCTATTACAGGGGAACTAAGAGCTGCATTGAAAATTATTTGCAAAGCTTGTAAGTGGTTCTGCCACTTATTAGCCGTGTGAACCTTAGCAAATTACCTAGCGTCTCTGAGTTTCAACTTCCTCATCTACAAAATAGAAATGATAATAAT"
            "AACCGCATCGCAAGAGTTGTTGGAAAAATGAAAATGAGGTATCATAGGAGGTAACATGTATGGAGCATTTACCATAGGCCAAGCACTGTTCTAAGAACTTCGGACATGTTATCTCACTTGTATAAGTACTTAGGTGCCTACAACATAAACAGCACCTGGT"
            "AAATTAAGTATTGAAAAAATGC";

        string_to_dna( str_ascii, str_hptr );
        string_to_dna( ref_ascii, ref_hptr );

        aln::EditDistanceAligner<aln::SEMI_GLOBAL> aligner;

        SingleTest test;
        nvbio::cuda::thrust_copy_vector(test.str_hvec, str_hvec);
        nvbio::cuda::thrust_copy_vector(test.ref_hvec, ref_hvec);
        nvbio::cuda::thrust_copy_vector(test.str_dvec, str_hvec);
        nvbio::cuda::thrust_copy_vector(test.ref_dvec, ref_hvec);

        test.full<BLOCKDIM,N,M>( "semi-global", aligner, "1I1M2I1M3I136M" );
    }

    // do a larger speed test of the Gotoh alignment
    if (TEST_MASK & (ED | SW | GOTOH))
    {
        const uint32 N_TASKS = N_THREAD_TASKS;
        const uint32 M = 150;
        const uint32 N = 500;

        const uint32 M_WORDS = (M + 7)  >> 3;
        const uint32 N_WORDS = (N + 15) >> 4;

        thrust::host_vector<uint32> str( M_WORDS * N_TASKS );
        thrust::host_vector<uint32> ref( N_WORDS * N_TASKS );

        LCG_random rand;
        fill_packed_stream<4u>( rand, 4u, M * N_TASKS, nvbio::raw_pointer( str ) );
        fill_packed_stream<2u>( rand, 4u, N * N_TASKS, nvbio::raw_pointer( ref ) );

        thrust::device_vector<uint32> str_dvec( str );
        thrust::device_vector<uint32> ref_dvec( ref );
        thrust::device_vector<int16>  score_dvec( N_TASKS );

        if (TEST_MASK & ED)
        {
            fprintf(stderr,"  testing Edit Distance scoring speed...\n");
            fprintf(stderr,"    %15s : ", "global");
            {
                batch_score_profile_all<N,M>(
                    make_edit_distance_aligner<aln::GLOBAL>(),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "semi-global");
            {
                batch_score_profile_all<N,M>(
                    make_edit_distance_aligner<aln::SEMI_GLOBAL>(),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "local");
            {
                batch_score_profile_all<N,M>(
                    make_edit_distance_aligner<aln::LOCAL>(),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
        }
        if (TEST_MASK & ED)
        {
            aln::SimpleSmithWatermanScheme scoring;
            scoring.m_match     =  2;
            scoring.m_mismatch  = -1;

            fprintf(stderr,"  testing Hamming Distance scoring speed...\n");
            fprintf(stderr,"    %15s : ", "semi-global");
            {
                batch_score_profile_all<N,M>(
                    make_hamming_distance_aligner<aln::SEMI_GLOBAL>( scoring ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "local");
            {
                batch_score_profile_all<N,M>(
                    make_hamming_distance_aligner<aln::LOCAL>( scoring ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
        }
        if (TEST_MASK & SW)
        {
            aln::SimpleSmithWatermanScheme scoring;
            scoring.m_match     =  2;
            scoring.m_mismatch  = -1;
            scoring.m_deletion  = -1;
            scoring.m_insertion = -1;

            fprintf(stderr,"  testing Smith-Waterman scoring speed...\n");
            fprintf(stderr,"    %15s : ", "global");
            {
                batch_score_profile_all<N,M>(
                    make_smith_waterman_aligner<aln::GLOBAL>( scoring ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "semi-global");
            {
                batch_score_profile_all<N,M>(
                    make_smith_waterman_aligner<aln::SEMI_GLOBAL>( scoring ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "local");
            {
                batch_score_profile_all<N,M>(
                    make_smith_waterman_aligner<aln::LOCAL>( scoring ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
        }
        if (TEST_MASK & GOTOH)
        {
            aln::SimpleGotohScheme scoring;
            scoring.m_match    =  2;
            scoring.m_mismatch = -1;
            scoring.m_gap_open = -1;
            scoring.m_gap_ext  = -1;

            fprintf(stderr,"  testing Gotoh scoring speed...\n");
            fprintf(stderr,"    %15s : ", "global");
            {
                batch_score_profile_all<N,M>(
                    make_gotoh_aligner<aln::GLOBAL>( scoring ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "semi-global");
            {
                batch_score_profile_all<N,M>(
                    make_gotoh_aligner<aln::SEMI_GLOBAL>( scoring ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "local");
            {
                batch_score_profile_all<N,M>(
                    make_gotoh_aligner<aln::LOCAL>( scoring ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
        }
    }
    // do a larger speed test of the banded SW alignment
    if (TEST_MASK & (ED_BANDED | SW_BANDED | GOTOH_BANDED))
    {
        const uint32 BAND_LEN = 15u;
        const uint32 N_TASKS  = N_THREAD_TASKS;
        const uint32 M = 150;
        const uint32 N = M+BAND_LEN;

        const uint32 M_WORDS = (M + 7)  >> 3;
        const uint32 N_WORDS = (N + 15) >> 4;

        thrust::host_vector<uint32> str( M_WORDS * N_TASKS );
        thrust::host_vector<uint32> ref( N_WORDS * N_TASKS );

        LCG_random rand;
        fill_packed_stream<4u>( rand, 4u, M * N_TASKS, nvbio::raw_pointer( str ) );
        fill_packed_stream<2u>( rand, 4u, N * N_TASKS, nvbio::raw_pointer( ref ) );

        thrust::device_vector<uint32> str_dvec( str );
        thrust::device_vector<uint32> ref_dvec( ref );
        thrust::device_vector<int16>  score_dvec( N_TASKS );

        if (TEST_MASK & ED_BANDED)
        {
            fprintf(stderr,"  testing banded Edit Distance scoring speed...\n");
            fprintf(stderr,"    %15s : ", "global");
            {
                batch_banded_score_profile_all<BAND_LEN,N,M>(
                    make_edit_distance_aligner<aln::GLOBAL>(),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "semi-global");
            {
                batch_banded_score_profile_all<BAND_LEN,N,M>(
                    make_edit_distance_aligner<aln::SEMI_GLOBAL>(),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "local");
            {
                batch_banded_score_profile_all<BAND_LEN,N,M>(
                    make_edit_distance_aligner<aln::LOCAL>(),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
        }
        if (TEST_MASK & SW_BANDED)
        {
            fprintf(stderr,"  testing banded Smith-Waterman scoring speed...\n");
            fprintf(stderr,"    %15s : ", "global");
            {
                batch_banded_score_profile_all<BAND_LEN,N,M>(
                    make_smith_waterman_aligner<aln::GLOBAL>( aln::SimpleSmithWatermanScheme(2,-1,-1,-1) ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "semi-global");
            {
                batch_banded_score_profile_all<BAND_LEN,N,M>(
                    make_smith_waterman_aligner<aln::SEMI_GLOBAL>( aln::SimpleSmithWatermanScheme(2,-1,-1,-1) ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "local");
            {
                batch_banded_score_profile_all<BAND_LEN,N,M>(
                    make_smith_waterman_aligner<aln::LOCAL>( aln::SimpleSmithWatermanScheme(2,-1,-1,-1) ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
        }
        if (TEST_MASK & GOTOH_BANDED)
        {
            fprintf(stderr,"  testing banded Gotoh scoring speed...\n");
            fprintf(stderr,"    %15s : ", "global");
            {
                batch_banded_score_profile_all<BAND_LEN,N,M>(
                    make_gotoh_aligner<aln::GLOBAL>( aln::SimpleGotohScheme(2,-1,-1,-1) ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "semi-global");
            {
                batch_banded_score_profile_all<BAND_LEN,N,M>(
                    make_gotoh_aligner<aln::SEMI_GLOBAL>( aln::SimpleGotohScheme(2,-1,-1,-1) ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
            fprintf(stderr,"    %15s : ", "local");
            {
                batch_banded_score_profile_all<BAND_LEN,N,M>(
                    make_gotoh_aligner<aln::LOCAL>( aln::SimpleGotohScheme(2,-1,-1,-1) ),
                    n_tests,
                    N_TASKS,
                    str_dvec,
                    ref_dvec,
                    score_dvec );
            }
        }
    }
    fprintf(stderr,"testing alignment... done\n");
}

} // namespace sw
} // namespace nvbio
