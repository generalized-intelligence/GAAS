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

#define NVBIO_CUDA_DEBUG

#include <cub/cub.cuh>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <crc/crc.h>
#ifdef _OPENMP
#include <omp.h>
#endif

#include <nvbio/sufsort/sufsort.h>
#include <nvbio/sufsort/sufsort_utils.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/timer.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/io/fmindex/fmindex.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/basic/dna.h>
#include <nvbio/fmindex/bwt.h>
#include <thrust/device_vector.h>

namespace nvbio {

namespace sufsort {

template <uint32 SYMBOL_SIZE, typename offset_type>
void make_test_string_set(
    const uint64                        N_strings,
    const uint32                        N,
    thrust::host_vector<uint32>&        h_string,
    thrust::host_vector<offset_type>&   h_offsets)
{
    for (uint64 i = 0; i < N_strings; ++i)
        h_offsets[i] = offset_type( uint64(N)*i );

    LCG_random rand;
    for (uint64 i = 0; i < h_string.size(); ++i)
        h_string[i] = rand.next();

    h_offsets[N_strings] = N*N_strings;
}

struct SuffixHandler
{
    void process(
        const uint32  n_suffixes,
        const uint32* suffix_array,
        const uint32* string_ids,
        const uint32* cum_lengths)
    {
        output.resize( n_suffixes );
        thrust::copy(
            thrust::device_ptr<const uint32>( suffix_array ),
            thrust::device_ptr<const uint32>( suffix_array ) + n_suffixes,
            output.begin() );
    }

    thrust::device_vector<uint32> output;
};

} // namespace sufsort

int sufsort_test(int argc, char* argv[])
{
    enum Test
    {
        kGPU_SA             = 1u,
        kGPU_BWT            = 2u,
        kCPU_BWT            = 4u,
        kGPU_BWT_FUNCTIONAL = 8u,
        kGPU_BWT_GENOME     = 16u,
        kGPU_BWT_SET        = 32u,
        kCPU_BWT_SET        = 64u,
        kGPU_SA_SET         = 128u,
    };
    uint32 TEST_MASK = 0xFFFFFFFFu;

    uint32 gpu_bwt_size = 50u;
    uint32 cpu_bwt_size = 100u;
#ifdef _OPENMP
    uint32 threads      = omp_get_num_procs();
#else
    uint32 threads      = 1;
#endif
    bool   store_output = true;

    const char* index_name = "data/human.NCBI36/Homo_sapiens.NCBI36.53.dna.toplevel.fa";

    BWTParams params;

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp( argv[i], "-v" )          == 0 ||
            strcmp( argv[i], "-verbosity" )  == 0 ||
            strcmp( argv[i], "--verbosity" ) == 0)
        {
            set_verbosity( Verbosity( atoi( argv[++i] ) ) );
        }
        else if (strcmp( argv[i], "-cpu-mem" ) == 0)
        {
            params.host_memory = atoi( argv[++i] ) * uint64(1024u*1024u);
        }
        else if (strcmp( argv[i], "-gpu-mem" ) == 0)
        {
            params.device_memory = atoi( argv[++i] ) * uint64(1024u*1024u);
        }
        else if (strcmp( argv[i], "-cpu-bwt-size" ) == 0)
        {
            cpu_bwt_size = atoi( argv[++i] );
        }
        else if (strcmp( argv[i], "-gpu-bwt-size" ) == 0)
        {
            gpu_bwt_size = atoi( argv[++i] );
        }
        else if (strcmp( argv[i], "-threads" ) == 0)
        {
            threads = atoi( argv[++i] );
        }
        else if (strcmp( argv[i], "-no-output" ) == 0)
        {
            store_output = false;
        }
        else if ((strcmp( argv[i], "-genome" ) == 0) ||
                 (strcmp( argv[i], "-index" )  == 0))
        {
            index_name = argv[++i];
        }
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

                if (strcmp( temp, "gpu-sa" ) == 0)
                    TEST_MASK |= kGPU_SA;
                else if (strcmp( temp, "gpu-bwt" ) == 0)
                    TEST_MASK |= kGPU_BWT;
                else if (strcmp( temp, "gpu-bwt-func" ) == 0)
                    TEST_MASK |= kGPU_BWT_FUNCTIONAL;
                else if (strcmp( temp, "gpu-bwt-genome" ) == 0)
                    TEST_MASK |= kGPU_BWT_GENOME;
                else if (strcmp( temp, "cpu-bwt" ) == 0)
                    TEST_MASK |= kCPU_BWT;
                else if (strcmp( temp, "gpu-set-bwt" ) == 0)
                    TEST_MASK |= kGPU_BWT_SET;
                else if (strcmp( temp, "cpu-set-bwt" ) == 0)
                    TEST_MASK |= kCPU_BWT_SET;

                if (*end == '\0')
                    break;

                ++end; begin = end;
            }
        }
    }

#ifdef _OPENMP
    // Now set the number of threads
    omp_set_num_threads( threads );
#endif

    log_info(stderr, "nvbio/sufsort test... started (%u threads)\n", threads);
    #pragma omp parallel
    {
        log_info(stderr, "  running on multiple threads\n");
    }
    const uint32 N           = 100;
    const uint32 SYMBOL_SIZE = 2;
    const uint32 SYMBOLS_PER_WORD = (8u*sizeof(uint32)) / SYMBOL_SIZE;

    if (TEST_MASK & kGPU_SA)
    {
        typedef uint32                                                  index_type;
        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,true,index_type> packed_stream_type;

        const index_type N_symbols  = 8u*1024u*1024u;
        const index_type N_words    = (N_symbols + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

        log_info(stderr, "  gpu sa test\n");
        log_info(stderr, "    %5.1f M symbols\n",  (1.0e-6f*float(N_symbols)));
        log_info(stderr, "    %5.2f GB\n",         (float(N_words)*sizeof(uint32))/float(1024*1024*1024));

        thrust::host_vector<uint32> h_string( N_words );

        LCG_random rand;
        for (index_type i = 0; i < N_words; ++i)
            h_string[i] = rand.next();

        for (uint32 lcp = 100; lcp <= 100000; lcp *= 10)
        {
            // insert some long common prefixes
            for (uint32 i = 50; i < 50 + lcp; ++i)
                h_string[i] = 0;

            thrust::device_vector<uint32>  d_string( h_string );
            thrust::device_vector<uint32>  d_sa( N_symbols+1 );

            cudaDeviceSynchronize();

            packed_stream_type d_packed_string( nvbio::plain_view( d_string ) );

            log_info(stderr, "\n  sa... started (LCP: %u)\n", lcp*16u);

            Timer timer;
            timer.start();

            cuda::suffix_sort(
                N_symbols,
                d_packed_string,
                d_sa.begin(),
                &params );

            cudaDeviceSynchronize();
            timer.stop();

            log_info(stderr, "  sa... done: %.2fs (%.1fM suffixes/s)\n", timer.seconds(), 1.0e-6f*float(N_symbols)/float(timer.seconds()));

            if (1)
            {
                log_info(stderr, "  sa-is... started\n");
                timer.start();

                std::vector<int32> sa_ref( N_symbols+1 );
                gen_sa( N_symbols, packed_stream_type( nvbio::plain_view( h_string ) ), &sa_ref[0] );

                timer.stop();
                log_info(stderr, "  sa-is... done: %.2fs (%.1fM suffixes/s)\n", timer.seconds(), 1.0e-6f*float(N_symbols)/float(timer.seconds()));

                thrust::host_vector<uint32> h_sa( d_sa );
                for (uint32 i = 0; i < N_symbols; ++i)
                {
                    const uint32 s = h_sa[i];
                    const uint32 r = sa_ref[i];
                    if (s != r)
                    {
                        log_error(stderr, "  mismatch at %u: expected %u, got %u\n", i, r, s);
                        return 0u;
                    }
                }
            }
        }

        FILE* file = fopen("./data/howto", "r" );
        if (file == NULL)
            log_warning(stderr, "  unable to open \"howto\" file\n");
        else
        {
            log_info(stderr, "\n  loading \"howto\" text benchmark\n");
            fseek( file, 0, SEEK_END );
            const uint32 N_symbols = uint32( ftell( file ) );
            thrust::host_vector<uint8> h_text( N_symbols );
            rewind( file );
            fread( &h_text[0], 1, N_symbols, file );
            fclose( file );

            thrust::device_vector<uint8>   d_text( h_text );
            thrust::device_vector<uint32>  d_sa( N_symbols+1 );

            cudaDeviceSynchronize();

            log_info(stderr, "  sa... started (%u bytes)\n", N_symbols);

            Timer timer;
            timer.start();

            cuda::suffix_sort(
                N_symbols,
                d_text.begin(),
                d_sa.begin(),
                &params );

            cudaDeviceSynchronize();
            timer.stop();

            log_info(stderr, "  sa... done: %.2fs (%.1fM suffixes/s)\n", timer.seconds(), 1.0e-6f*float(N_symbols)/float(timer.seconds()));

            if (1)
            {
                log_info(stderr, "  sa-is... started\n");
                timer.start();

                std::vector<int32> sa_ref( N_symbols+1 );
                sa_ref[0] = N_symbols;
                saisxx( nvbio::plain_view( h_text ), &sa_ref[0] + 1, int32(N_symbols), 256 );

                timer.stop();
                log_info(stderr, "  sa-is... done: %.2fs (%.1fM suffixes/s)\n", timer.seconds(), 1.0e-6f*float(N_symbols)/float(timer.seconds()));

                thrust::host_vector<uint32> h_sa( d_sa );
                for (uint32 i = 0; i < N_symbols; ++i)
                {
                    const uint32 s = h_sa[i];
                    const uint32 r = sa_ref[i];
                    if (s != r)
                    {
                        log_error(stderr, "  mismatch at %u: expected %u, got %u\n", i, r, s);
                        return 0u;
                    }
                }
            }
        }
    }
    if (TEST_MASK & kGPU_SA_SET)
    {
        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,false>           packed_stream_type;
        typedef ConcatenatedStringSet<packed_stream_type,uint32*>       string_set;

        const uint32 N_strings  = 1024*1024;
        const uint32 N_tests    = 10;
        const uint32 N_words    = uint32((uint64(N_strings)*N + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD);

        thrust::host_vector<uint32>  h_string( N_words );
        thrust::host_vector<uint32>  h_offsets( N_strings+1 );

        sufsort::make_test_string_set<SYMBOL_SIZE>(
            N_strings,
            N,
            h_string,
            h_offsets );

        thrust::device_vector<uint32>  d_string( h_string );
        thrust::device_vector<uint32>  d_offsets( h_offsets );

        packed_stream_type d_packed_string( nvbio::plain_view( d_string ) );

        string_set d_string_set(
            N_strings,
            d_packed_string,
            nvbio::plain_view( d_offsets ) );

        cudaDeviceSynchronize();

        log_info(stderr, "  gpu SA test\n");
        log_info(stderr, "    %5.1f M strings\n",  (1.0e-6f*float(N_strings)));
        log_info(stderr, "    %5.1f M suffixes\n", (1.0e-6f*float(N_strings*(N+1))));
        log_info(stderr, "    %5.1f G symbols\n",  (1.0e-9f*float(uint64(N_strings)*(N+1)*(N+1)/2)));
        log_info(stderr, "    %5.2f GB\n",         (float(N_words)*sizeof(uint32))/float(1024*1024*1024));

        // copy a sparse string set into a packed concatenated one
        {
            sufsort::SuffixHandler suffix_hander;

            Timer timer;
            timer.start();

            // sort the suffixes
            for (uint32 i = 0; i < N_tests; ++i)
                cuda::suffix_sort( d_string_set, suffix_hander, &params );

            cudaDeviceSynchronize();
            timer.stop();

            log_info(stderr, "  sorting time: %.2fs\n", timer.seconds()/float(N_tests));
            log_info(stderr, "    %5.1f M strings/s\n",  (1.0e-6f*float(N_strings))               * (float(N_tests)/timer.seconds()));
            log_info(stderr, "    %5.1f M suffixes/s\n", (1.0e-6f*float(N_strings*(N+1)))         * (float(N_tests)/timer.seconds()));
            log_info(stderr, "    %5.1f G symbols/s\n",  (1.0e-9f*float(uint64(N_strings)*(N+1)*(N+1)/2)) * (float(N_tests)/timer.seconds()));
        }
    }
    if (TEST_MASK & kGPU_BWT_FUNCTIONAL)
    {
        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,true,uint32>     packed_stream_type;

        const uint32 N_words    = 8;
        const uint32 N_symbols  = N_words * SYMBOLS_PER_WORD - 13u;

        char char_string[N_symbols+1];

        log_info(stderr, "  gpu bwt test\n");

        thrust::host_vector<uint32>  h_string( N_words );
        thrust::host_vector<uint32>  h_bwt( N_words+1 );
        thrust::host_vector<uint32>  h_bwt_ref( N_words+1 );
        uint32                       primary_ref;

        LCG_random rand;
        for (uint32 i = 0; i < N_words; ++i)
            h_string[i] = rand.next();

        dna_to_string(
            packed_stream_type( nvbio::plain_view( h_string ) ),
            N_symbols,
            char_string );

        log_info(stderr, "    str     : %s\n", char_string );
        {
            // generate the SA using SA-IS
            int32 sa[N_symbols+1];
            gen_sa( N_symbols, packed_stream_type( nvbio::plain_view( h_string ) ), &sa[0] );

            // generate the BWT from the SA
            primary_ref = gen_bwt_from_sa( N_symbols, packed_stream_type( nvbio::plain_view( h_string ) ), sa, packed_stream_type( nvbio::plain_view( h_bwt_ref ) ) );

            dna_to_string(
                packed_stream_type( nvbio::plain_view( h_bwt_ref ) ),
                N_symbols,
                char_string );

            log_info(stderr, "    primary : %u\n", primary_ref );
            log_info(stderr, "    bwt     : %s\n", char_string );
        }

        thrust::device_vector<uint32>  d_string( h_string );
        thrust::device_vector<uint32>  d_bwt( N_words+1 );

        cudaDeviceSynchronize();

        packed_stream_type d_packed_string( nvbio::plain_view( d_string ) );
        packed_stream_type d_packed_bwt( nvbio::plain_view( d_bwt ) );

        log_info(stderr, "  bwt... started\n");

        Timer timer;
        timer.start();

        const uint32 primary = cuda::bwt(
            N_symbols,
            d_packed_string,
            d_packed_bwt,
            &params );

        timer.stop();

        log_info(stderr, "  bwt... done: %.2fs\n", timer.seconds());

        h_bwt = d_bwt;
        {
            // check whether the results match our expectations
            packed_stream_type h_packed_bwt_ref( nvbio::plain_view( h_bwt_ref ) );
            packed_stream_type h_packed_bwt( nvbio::plain_view( h_bwt ) );

            bool check = (primary_ref == primary);
            for (uint32 i = 0; i < N_symbols; ++i)
            {
                if (h_packed_bwt[i] != h_packed_bwt_ref[i])
                    check = false;
            }

            if (check == false)
            {
                dna_to_string(
                    packed_stream_type( nvbio::plain_view( h_bwt ) ),
                    N_symbols,
                    char_string );

                log_error(stderr, "mismatching results!\n" );
                log_error(stderr, "    primary : %u\n", primary );
                log_error(stderr, "    bwt     : %s\n", char_string );
                return 0u;
            }
        }
    }
    if (TEST_MASK & kGPU_BWT_GENOME)
    {
        // load a genome
        io::SequenceDataHost h_ref;
        io::FMIndexDataHost  h_fmi;

        if (io::load_sequence_file( DNA, &h_ref, index_name ) == false)
            return 0;

        if (h_fmi.load( index_name, io::FMIndexData::FORWARD ) == false)
            return 0;

        // copy it to the gpu
        io::SequenceDataDevice d_ref( h_ref );
        io::FMIndexDataDevice d_fmi( h_fmi, 0u );

        typedef io::SequenceDataAccess<DNA,io::ConstSequenceDataView> const_reference_access_type;
        typedef io::SequenceDataEdit<DNA,io::SequenceDataView>              reference_access_type;
        typedef const_reference_access_type::sequence_stream_type     const_packed_stream_type;
        typedef       reference_access_type::sequence_stream_type           packed_stream_type;

        const uint32 N_symbols = d_ref.bps();
        const uint32 N_words   = d_ref.words();

        log_info(stderr, "  gpu bwt test\n");
        log_info(stderr, "    %5.1f G symbols\n",  (1.0e-6f*float(N_symbols)));
        log_info(stderr, "    %5.2f GB\n",         (float(N_words)*sizeof(uint32))/float(1024*1024*1024));

        thrust::device_vector<uint32> d_bwt_storage( N_words+1 );

        const const_reference_access_type d_ref_access( d_ref );
        const_packed_stream_type d_packed_string( d_ref_access.sequence_stream() );
              packed_stream_type d_packed_bwt( nvbio::plain_view( d_bwt_storage ) );

        const uint32 primary_ref = cuda::find_primary( N_symbols, d_packed_string );
        log_info(stderr, "    primary: %u\n", primary_ref);
        {
            const const_reference_access_type h_ref_access( h_ref );
            const_packed_stream_type h_packed_string( h_ref_access.sequence_stream() );
            const uint32 crc = crcCalc( h_packed_string, N_symbols );
            log_info(stderr, "    crc    : %u\n", crc);
        }

        log_info(stderr, "  bwt... started\n");

        Timer timer;
        timer.start();

        const uint32 primary = cuda::bwt(
            N_symbols,
            d_packed_string,
            d_packed_bwt,
            &params );

        timer.stop();

        log_info(stderr, "  bwt... done: %.2fs\n", timer.seconds());

        bool check = primary == primary_ref;
        if (check == false)
        {
            log_error(stderr, "mismatching results!\n" );
            log_error(stderr, "    primary : %u\n", primary );
            return 0u;
        }

        log_info(stderr, "  testing correctness... started\n");
        thrust::host_vector<uint32>            h_bwt_storage( d_bwt_storage );
        const const_packed_stream_type         h_packed_bwt( nvbio::plain_view( h_bwt_storage ) );
        const io::FMIndexData::bwt_stream_type h_ref_bwt( h_fmi.bwt_iterator() );
        for (uint32 i = 0; i < N_symbols; ++i)
        {
            const uint8 c0 = h_ref_bwt[i];
            const uint8 c1 = h_packed_bwt[i];

            if (c0 != c1)
            {
                log_error(stderr, "mismatching results!\n" );
                log_error(stderr, "    at %u, expected %c, got %c\n", i, dna_to_char(c0), dna_to_char(c1) );
                return 0u;
            }
        }
        log_info(stderr, "  testing correctness... done\n");
        {
            const uint32 crc = crcCalc( h_packed_bwt, N_symbols );
            log_info(stderr, "    crc: %u\n", crc);
        }
    }
    if (TEST_MASK & kGPU_BWT)
    {
        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,true,uint64>     packed_stream_type;

        const uint64 N_symbols  = 4llu*1024u*1024u*1024u - 1u;
        const uint64 N_words    = (N_symbols + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

        log_info(stderr, "  gpu bwt test\n");
        log_info(stderr, "    %5.1f G symbols\n",  (1.0e-9f*float(N_symbols)));
        log_info(stderr, "    %5.2f GB\n",         (float(N_words)*sizeof(uint32))/float(1024*1024*1024));

        thrust::host_vector<uint32>  h_string( N_words );

        LCG_random rand;
        for (uint64 i = 0; i < N_words; ++i)
            h_string[i] = rand.next();

        // insert some long common prefixes
        for (uint32 i = 50; i < 100; ++i)
            h_string[i] = 0;

        thrust::device_vector<uint32>  d_string( h_string );
        thrust::device_vector<uint32>  d_bwt( N_words );

        cudaDeviceSynchronize();

        packed_stream_type d_packed_string( nvbio::plain_view( d_string ) );
        packed_stream_type d_packed_bwt( nvbio::plain_view( d_bwt ) );

        log_info(stderr, "  bwt... started\n");

        Timer timer;
        timer.start();

        cuda::bwt(
            N_symbols,
            d_packed_string,
            d_packed_bwt,
            &params );

        timer.stop();

        log_info(stderr, "  bwt... done: %.2fs\n", timer.seconds());
    }
    if (TEST_MASK & kGPU_BWT_SET)
    {
        typedef uint32 word_type;
        typedef cuda::load_pointer<word_type,cuda::LOAD_DEFAULT> storage_type;

        typedef PackedStream<word_type*,uint8,SYMBOL_SIZE,true,uint64>      packed_stream_type;

        typedef PackedStream<storage_type,uint8,SYMBOL_SIZE,true,uint64>    mod_packed_stream_type;
        typedef ConcatenatedStringSet<mod_packed_stream_type,uint64*>       string_set;

        const uint32 N_strings   = gpu_bwt_size*1000*1000;
        const uint64 N_words     = util::divide_ri( uint64(N_strings)*(N+0), SYMBOLS_PER_WORD );
        const uint64 N_bwt_words = util::divide_ri( uint64(N_strings)*(N+1), SYMBOLS_PER_WORD );

        log_info(stderr, "  gpu set-bwt test\n");
        log_info(stderr, "    %5.1f M strings\n",  (1.0e-6f*float(N_strings)));
        log_info(stderr, "    %5.1f G suffixes\n", (1.0e-9f*float(uint64(N_strings)*uint64(N+1))));
        log_info(stderr, "    %5.2f GB\n",         (float(N_words)*sizeof(uint32))/float(1024*1024*1024));

        thrust::host_vector<uint32>  h_string( N_words );
        thrust::host_vector<uint64>  h_offsets( N_strings+1 );

        sufsort::make_test_string_set<SYMBOL_SIZE>(
            N_strings,
            N,
            h_string,
            h_offsets );

        thrust::device_vector<uint32>  d_string( h_string );
        thrust::device_vector<uint64>  d_offsets( h_offsets );

        cudaDeviceSynchronize();

        mod_packed_stream_type d_packed_string( storage_type( (word_type*)nvbio::plain_view( d_string ) ) );

        string_set d_string_set(
            N_strings,
            d_packed_string,
            nvbio::plain_view( d_offsets ) );

        log_info(stderr, "  bwt... started\n");

        Timer timer;

        if (store_output)
        {
            thrust::device_vector<uint32> d_bwt( N_bwt_words );
            packed_stream_type            d_packed_bwt( (word_type*)nvbio::plain_view( d_bwt ) );

            DeviceBWTHandler<packed_stream_type> output_handler( d_packed_bwt );

            timer.start();

            cuda::bwt<SYMBOL_SIZE,true>(
                d_string_set,
                output_handler,
                &params );

            timer.stop();
        }
        else
        {
            DiscardBWTHandler output_handler;

            timer.start();

            cuda::bwt<SYMBOL_SIZE,true>(
                d_string_set,
                output_handler,
                &params );

            timer.stop();
        }

        log_info(stderr, "  bwt... done: %.2fs\n", timer.seconds());
    }
    if (TEST_MASK & kCPU_BWT_SET)
    {
        typedef uint32 word_type;

        typedef PackedStream<word_type*,uint8,SYMBOL_SIZE,true,uint64>  packed_stream_type;
        typedef ConcatenatedStringSet<packed_stream_type,uint64*>       string_set;

        const uint32 N_strings   = cpu_bwt_size*1000*1000;
        const uint64 N_words     = util::divide_ri( uint64(N_strings)*(N+0), SYMBOLS_PER_WORD );
        const uint64 N_bwt_words = util::divide_ri( uint64(N_strings)*(N+1), SYMBOLS_PER_WORD );

        log_info(stderr, "  cpu set-bwt test\n");
        log_info(stderr, "    %5.1f M strings\n",  (1.0e-6f*float(N_strings)));
        log_info(stderr, "    %5.1f G suffixes\n", (1.0e-9f*float(uint64(N_strings)*uint64(N+1))));
        log_info(stderr, "    %5.2f GB\n",         (float(N_words)*sizeof(uint32))/float(1024*1024*1024));

        thrust::host_vector<uint32>  h_string( N_words );
        thrust::host_vector<uint64>  h_offsets( N_strings+1 );

        sufsort::make_test_string_set<SYMBOL_SIZE>(
            N_strings,
            N,
            h_string,
            h_offsets );

        packed_stream_type h_packed_string( (word_type*)nvbio::plain_view( h_string ) );

        string_set h_string_set(
            N_strings,
            h_packed_string,
            nvbio::plain_view( h_offsets ) );

        log_info(stderr, "  bwt... started\n");

        Timer timer;

        if (store_output)
        {
            thrust::host_vector<uint32>  h_bwt( N_bwt_words );
            packed_stream_type           h_packed_bwt( (word_type*)nvbio::plain_view( h_bwt ) );

            HostBWTHandler<packed_stream_type> output_handler( h_packed_bwt );

            timer.start();

            large_bwt<SYMBOL_SIZE,true>(
                h_string_set,
                output_handler,
                &params );

            timer.stop();
        }
        else
        {
            DiscardBWTHandler output_handler;

            timer.start();

            large_bwt<SYMBOL_SIZE,true>(
                h_string_set,
                output_handler,
                &params );

            timer.stop();
        }

        log_info(stderr, "  bwt... done: %.2fs\n", timer.seconds());
    }
    log_info(stderr, "nvbio/sufsort test... done\n");
    return 0;
}

} // namespace nvbio

using namespace nvbio;

int main(int argc, char* argv[])
{
    crcInit();

    int cuda_device = -1;
    int device_count;
    cudaGetDeviceCount(&device_count);
    log_verbose(stderr, "  cuda devices : %d\n", device_count);

    int arg = 1;
    if (argc > 1)
    {
        if (strcmp( argv[arg], "-device" ) == 0)
        {
            cuda_device = atoi(argv[++arg]);
            ++arg;
        }
    }

    // inspect and select cuda devices
    if (device_count)
    {
        if (cuda_device == -1)
        {
            int            best_device = 0;
            cudaDeviceProp best_device_prop;
            cudaGetDeviceProperties( &best_device_prop, best_device );

            for (int device = 0; device < device_count; ++device)
            {
                cudaDeviceProp device_prop;
                cudaGetDeviceProperties( &device_prop, device );
                log_verbose(stderr, "  device %d has compute capability %d.%d\n", device, device_prop.major, device_prop.minor);
                log_verbose(stderr, "    SM count          : %u\n", device_prop.multiProcessorCount);
                log_verbose(stderr, "    SM clock rate     : %u Mhz\n", device_prop.clockRate / 1000);
                log_verbose(stderr, "    memory clock rate : %.1f Ghz\n", float(device_prop.memoryClockRate) * 1.0e-6f);

                if (device_prop.major >= best_device_prop.major &&
                    device_prop.minor >= best_device_prop.minor)
                {
                    best_device_prop = device_prop;
                    best_device      = device;
                }
            }
            cuda_device = best_device;
        }
        log_verbose(stderr, "  chosen device %d\n", cuda_device);
        {
            cudaDeviceProp device_prop;
            cudaGetDeviceProperties( &device_prop, cuda_device );
            log_verbose(stderr, "    device name        : %s\n", device_prop.name);
            log_verbose(stderr, "    compute capability : %d.%d\n", device_prop.major, device_prop.minor);
        }
        cudaSetDevice( cuda_device );
    }

    // allocate some heap
    cudaDeviceSetLimit( cudaLimitMallocHeapSize, 128*1024*1024 );

    argc = argc >= arg ? argc-arg : 0;

    try
    {
        nvbio::sufsort_test( argc, argv+arg );
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
        log_error(stderr,"unknown exception caught!\n");
        exit(1);
    }

    cudaDeviceReset();
	return 0;
}
