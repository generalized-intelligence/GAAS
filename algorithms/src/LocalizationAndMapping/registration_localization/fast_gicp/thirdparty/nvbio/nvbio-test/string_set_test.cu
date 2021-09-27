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

// string_set_test.cu
//

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/strings/string_set.h>
#include <thrust/device_vector.h>

namespace nvbio {

namespace string_set_private {

enum StringSetTest
{
    ALL                                 = 0x0FFFFFFFu,
    CPU                                 = 0xF0000000u,
    SPARSE_TO_CONCAT                    = 1u,
    SPARSE_TO_PACKED_CONCAT             = 2u,
    CONCAT_TO_PACKED_CONCAT             = 4u,
    CONCAT_TO_STRIDED                   = 8u,
    SPARSE_TO_STRIDED                   = 16u,
    PACKED_CONCAT_TO_STRIDED            = 32u,
    PACKED_SPARSE_TO_STRIDED            = 64u,
    STRIDED_PACKED_TO_STRIDED           = 128u,
    CONCAT_TO_STRIDED_PACKED            = 256u,
    PACKED_CONCAT_TO_STRIDED_PACKED     = 512u,
    PACKED_SPARSE_TO_STRIDED_PACKED     = 1024u,
};

}

using namespace string_set_private;

template <typename input_set, typename output_set>
void check(const input_set& in_string_set, const output_set& out_string_set)
{
    if (in_string_set.size() != out_string_set.size())
    {
        fprintf(stderr, "    \nerror: input set has size %u, output has %u\n", in_string_set.size(), out_string_set.size() );
        exit(1);
    }

    // check that the string sets match
    for (uint32 i = 0; i < in_string_set.size(); ++i)
    {
        typename input_set::string_type  in_string  = in_string_set[i];
        typename output_set::string_type out_string = out_string_set[i];

        const uint32 in_len  = in_string.size();
        const uint32 out_len = out_string.size();

        if (in_len != out_len)
        {
            fprintf(stderr, "    \nerror: input string[%u] has length %u, output has length %u\n", i, in_len, out_len );
            exit(1);
        }

        for (uint32 j = 0; j < in_len; ++j)
        {
            const uint8 in_c  = in_string[j];
            const uint8 out_c = out_string[j];
            if (in_c != out_c)
            {
                fprintf(stderr, "    \nerror: at string[%u][%u] expected : %u, got %u\n", i, j, uint32(in_c), uint32(out_c) );
                exit(1);
            }
        }
    }
}

void make_test_string_set(
    const uint32                  SYMBOL_SIZE,
    const uint32                  N_strings,
    const uint32                  N,
    const uint32                  N_spacing,
    thrust::host_vector<uint8>&   h_string,
    thrust::host_vector<uint2>&   h_ranges)
{
    h_string.resize( N_strings * N_spacing );
    h_ranges.resize( N_strings );

    LCG_random rand;
    for (uint32 i = 0; i < N_strings; ++i)
    {
        h_ranges[i] = make_uint2( N_spacing*i, N_spacing*i + N );
        for (uint32 j = 0; j < N_spacing; ++j)
            h_string[ i * N_spacing + j ] = rand.next() & ((1u << SYMBOL_SIZE)-1);
    }
}

int string_set_test(int argc, char* argv[])
{
    fprintf(stderr, "nvbio/basic/string_set test... started\n");
    const uint32 N           = 128;
    const uint32 N_spacing   = 150;
    const uint32 SYMBOL_SIZE = 4;
    const uint32 SYMBOLS_PER_WORD = (8u*sizeof(uint32)) / SYMBOL_SIZE;
    const uint32 N_words     = (N + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

    uint32 N_strings   = 256*1024;
    uint32 N_tests     = 20;

    uint32 TEST_MASK = ALL;
    uint32 CPU_MASK  = CPU;

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp( argv[i], "-N" ) == 0)
            N_strings = atoi( argv[++i] );
        else if (strcmp( argv[i], "-N-tests" ) == 0)
            N_tests = atoi( argv[++i] );
        else if (strcmp( argv[i], "-cpu" ) == 0)
            CPU_MASK = atoi( argv[++i] );
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

                if (strcmp( temp, "packed-sparse-to-strided" ) == 0)
                    TEST_MASK |= PACKED_SPARSE_TO_STRIDED;
                else if (strcmp( temp, "packed-sparse-to-strided-packed" ) == 0)
                    TEST_MASK |= PACKED_SPARSE_TO_STRIDED_PACKED;
                else if (strcmp( temp, "sparse-to-concat" ) == 0)
                    TEST_MASK |= SPARSE_TO_CONCAT;
                else if (strcmp( temp, "sparse-to-packed-concat" ) == 0)
                    TEST_MASK |= SPARSE_TO_PACKED_CONCAT;
                else if (strcmp( temp, "concat-to-packed-concat" ) == 0)
                    TEST_MASK |= CONCAT_TO_PACKED_CONCAT;
                else if (strcmp( temp, "sparse-to-strided" ) == 0)
                    TEST_MASK |= SPARSE_TO_STRIDED;
                else if (strcmp( temp, "packed-concat-to-strided" ) == 0)
                    TEST_MASK |= PACKED_CONCAT_TO_STRIDED;
                else if (strcmp( temp, "packed-sparse-to-strided" ) == 0)
                    TEST_MASK |= PACKED_SPARSE_TO_STRIDED;
                else if (strcmp( temp, "strided-packed-to-strided" ) == 0)
                    TEST_MASK |= STRIDED_PACKED_TO_STRIDED;
                else if (strcmp( temp, "concat-to-strided-packed" ) == 0)
                    TEST_MASK |= CONCAT_TO_STRIDED_PACKED;
                else if (strcmp( temp, "packed-concat-to-strided-packed" ) == 0)
                    TEST_MASK |= PACKED_CONCAT_TO_STRIDED_PACKED;
                else if (strcmp( temp, "packed-sparse-to-strided-packed" ) == 0)
                    TEST_MASK |= PACKED_SPARSE_TO_STRIDED_PACKED;

                if (end[i] == '\0')
                    break;

                ++end; begin = end;
            }
        }
    }
    TEST_MASK |= CPU_MASK;

    typedef SparseStringSet<uint8*,uint2*> base_string_set;

    thrust::host_vector<uint8>  h_base_string;
    thrust::host_vector<uint2>  h_base_ranges;

    make_test_string_set(
        SYMBOL_SIZE,
        N_strings,
        N,
        N_spacing,
        h_base_string,
        h_base_ranges );

    thrust::device_vector<uint8>   d_base_string( h_base_string );
    thrust::device_vector<uint2>   d_base_ranges( h_base_ranges );

    base_string_set h_base_string_set(
        N_strings,
        thrust::raw_pointer_cast( &h_base_string.front() ),
        thrust::raw_pointer_cast( &h_base_ranges.front() ) );

    base_string_set d_base_string_set(
        N_strings,
        thrust::raw_pointer_cast( &d_base_string.front() ),
        thrust::raw_pointer_cast( &d_base_ranges.front() ) );

    // copy a packed sparse string set into a strided packed string set
    if ((TEST_MASK & PACKED_SPARSE_TO_STRIDED) && (TEST_MASK & CPU))
    {
        fprintf(stderr, "  test cpu packed-sparse  -> strided        copy... started\n");
        const uint32 N_words = (N_spacing + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,false> packed_stream_type;

        typedef SparseStringSet<packed_stream_type,const uint2*> input_set;
        typedef StridedStringSet<
            uint8*,
            uint32*>                                                  output_set;

        thrust::host_vector<uint32>  h_in_string( N_strings * N_words );
        thrust::host_vector<uint2>   h_in_ranges( N_strings );

        packed_stream_type h_packed_stream(
            thrust::raw_pointer_cast( &h_in_string.front() ) );

        LCG_random rand;
        for (uint32 i = 0; i < N_strings; ++i)
        {
            h_in_ranges[i] = make_uint2( N_spacing*i, N_spacing*i + N );
            for (uint32 j = 0; j < N_spacing; ++j)
                h_packed_stream[ i * N_spacing + j ] = rand.next() & ((1u << SYMBOL_SIZE) - 1u);
        }

        input_set h_in_string_set(
            N_strings,
            h_packed_stream,
            thrust::raw_pointer_cast( &h_in_ranges.front() ) );

        // build the host output string set
        thrust::host_vector<uint8>   h_out_stream( N_strings * N );
        thrust::host_vector<uint32>  h_out_lengths( N_strings );

        output_set h_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &h_out_stream.front() ),
            thrust::raw_pointer_cast( &h_out_lengths.front() ) );

        Timer timer;
        timer.start();

        // copy intput set into the output set
        for (uint32 i = 0; i < N_tests; ++i)
            copy( h_in_string_set, h_out_string_set );

        timer.stop();
        fprintf(stderr, "  test cpu packed-sparse  -> strided        copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }
    // copy a packed sparse string set into a strided packed string set
    if ((TEST_MASK & PACKED_SPARSE_TO_STRIDED_PACKED) && (TEST_MASK & CPU))
    {
        fprintf(stderr, "  test cpu packed-sparse  -> strided-packed copy... started\n");
        const uint32 N_words = (N_spacing + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,false> packed_stream_type;

        typedef SparseStringSet<packed_stream_type,const uint2*> input_set;
        typedef StridedPackedStringSet<
            uint32*,
            uint8,
            SYMBOL_SIZE,
            false,
            uint32*>                                                  output_set;

        thrust::host_vector<uint32>  h_in_string( N_strings * N_words );
        thrust::host_vector<uint2>   h_in_ranges( N_strings );

        packed_stream_type h_packed_stream(
            thrust::raw_pointer_cast( &h_in_string.front() ) );

        LCG_random rand;
        for (uint32 i = 0; i < N_strings; ++i)
        {
            h_in_ranges[i] = make_uint2( N_spacing*i, N_spacing*i + N );
            for (uint32 j = 0; j < N_spacing; ++j)
                h_packed_stream[ i * N_spacing + j ] = rand.next() & ((1u << SYMBOL_SIZE) - 1u);
        }

        input_set h_in_string_set(
            N_strings,
            h_packed_stream,
            thrust::raw_pointer_cast( &h_in_ranges.front() ) );

        // build the output string set
        thrust::host_vector<uint32>  h_out_stream( N_strings * N_words );
        thrust::host_vector<uint32>  h_out_lengths( N_strings );

        output_set h_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &h_out_stream.front() ),
            thrust::raw_pointer_cast( &h_out_lengths.front() ) );

        Timer timer;
        timer.start();

        // copy intput set into the output set
        for (uint32 i = 0; i < N_tests; ++i)
            copy( h_in_string_set, h_out_string_set );

        timer.stop();

        fprintf(stderr, "  test cpu packed-sparse  -> strided-packed copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }

    // copy a sparse string set into a concatenated one
    if (TEST_MASK & SPARSE_TO_CONCAT)
    {
        fprintf(stderr, "  test sparse         -> concat         copy... started\n");

        typedef base_string_set                                     input_set;
        typedef ConcatenatedStringSet<uint8*,uint32*>               output_set;

        // build the device output string set
        thrust::device_vector<uint8>   d_out_string( N_strings * N );
        thrust::device_vector<uint32>  d_out_offsets( N_strings+1 );

        output_set d_out_string_set(
            N_strings,
            thrust::raw_pointer_cast( &d_out_string.front() ),
            thrust::raw_pointer_cast( &d_out_offsets.front() ) );

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_base_string_set, d_out_string_set );

        timer.stop();

        // build the host output string set
        thrust::host_vector<uint8>  h_out_string( d_out_string );
        thrust::host_vector<uint32> h_out_offsets( d_out_offsets );

        output_set h_out_string_set(
            N_strings,
            thrust::raw_pointer_cast( &h_out_string.front() ),
            thrust::raw_pointer_cast( &h_out_offsets.front() ) );

        // check that the string sets match
        check( h_base_string_set, h_out_string_set );
        fprintf(stderr, "  test sparse         -> concat         copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }
    // copy a sparse string set into a packed concatenated one
    if (TEST_MASK & SPARSE_TO_PACKED_CONCAT)
    {
        fprintf(stderr, "  test sparse         -> packed-concat  copy... started\n");

        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,false> packed_stream_type;

        typedef base_string_set                                         input_set;
        typedef ConcatenatedStringSet<packed_stream_type,uint32*>       output_set;

        // build the device output string set
        thrust::device_vector<uint32>  d_out_string( N_strings * N_words );
        thrust::device_vector<uint32>  d_out_offsets( N_strings+1 );

        packed_stream_type d_packed_stream(
            thrust::raw_pointer_cast( &d_out_string.front() ) );

        output_set d_out_string_set(
            N_strings,
            d_packed_stream,
            thrust::raw_pointer_cast( &d_out_offsets.front() ) );

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_base_string_set, d_out_string_set );

        timer.stop();

        // build the host output string set
        thrust::host_vector<uint32> h_out_string( d_out_string );
        thrust::host_vector<uint32> h_out_offsets( d_out_offsets );

        packed_stream_type h_packed_stream(
            thrust::raw_pointer_cast( &h_out_string.front() ) );

        output_set h_out_string_set(
            N_strings,
            h_packed_stream,
            thrust::raw_pointer_cast( &h_out_offsets.front() ) );

        // check that the string sets match
        check( h_base_string_set, h_out_string_set );
        fprintf(stderr, "  test sparse         -> packed-concat  copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }
    // copy a concatenated string set into a packed concatenated one
    if (TEST_MASK & CONCAT_TO_PACKED_CONCAT)
    {
        fprintf(stderr, "  test concat         -> packed-concat  copy... started\n");

        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,false> packed_stream_type;

        typedef ConcatenatedStringSet<uint8*,uint32*>               input_set;
        typedef ConcatenatedStringSet<packed_stream_type,uint32*>   output_set;

        // build the device input string set
        thrust::device_vector<uint8>   d_in_string( N_strings * N );
        thrust::device_vector<uint32>  d_in_offsets( N_strings+1 );

        input_set d_in_string_set(
            N_strings,
            thrust::raw_pointer_cast( &d_in_string.front() ),
            thrust::raw_pointer_cast( &d_in_offsets.front() ) );

        // copy the base string set into the input one
        cuda::copy( d_base_string_set, d_in_string_set );

        // build the device output string set
        thrust::device_vector<uint32>  d_out_string( N_strings * N_words );
        thrust::device_vector<uint32>  d_out_offsets( N_strings+1 );

        packed_stream_type d_packed_stream(
            thrust::raw_pointer_cast( &d_out_string.front() ) );

        output_set d_out_string_set(
            N_strings,
            d_packed_stream,
            thrust::raw_pointer_cast( &d_out_offsets.front() ) );

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_in_string_set, d_out_string_set );

        timer.stop();

        // build the host output string set
        thrust::host_vector<uint32> h_out_string( d_out_string );
        thrust::host_vector<uint32> h_out_offsets( d_out_offsets );

        packed_stream_type h_packed_stream(
            thrust::raw_pointer_cast( &h_out_string.front() ) );

        output_set h_out_string_set(
            N_strings,
            h_packed_stream,
            thrust::raw_pointer_cast( &h_out_offsets.front() ) );

        // check that the string sets match
        check( h_base_string_set, h_out_string_set );
        fprintf(stderr, "  test concat         -> packed-concat  copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }
    // copy a sparse string set into a strided one
    if (TEST_MASK & SPARSE_TO_STRIDED)
    {
        fprintf(stderr, "  test sparse         -> strided        copy... started\n");

        typedef base_string_set                                input_set;
        typedef StridedStringSet<uint8*,uint32*>               output_set;

        // build the device output string set
        thrust::device_vector<uint8>   d_out_string( N_strings * N );
        thrust::device_vector<uint32>  d_out_lengths( N_strings+1 );

        output_set d_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &d_out_string.front() ),
            thrust::raw_pointer_cast( &d_out_lengths.front() ) );

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_base_string_set, d_out_string_set );

        timer.stop();

        // build the host output string set
        thrust::host_vector<uint8>  h_out_string( d_out_string );
        thrust::host_vector<uint32> h_out_lengths( d_out_lengths );

        output_set h_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &h_out_string.front() ),
            thrust::raw_pointer_cast( &h_out_lengths.front() ) );

        // check that the string sets match
        check( h_base_string_set, h_out_string_set );
        fprintf(stderr, "  test sparse         -> strided        copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }
    // copy a packed sparse string set into a strided packed string set
    if (TEST_MASK & PACKED_CONCAT_TO_STRIDED)
    {
        fprintf(stderr, "  test packed-concat  -> strided        copy... started\n");
        const uint32 N_words = (N + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,false> packed_stream_type;

        typedef ConcatenatedStringSet<packed_stream_type,uint32*>   input_set;
        typedef StridedStringSet<
            uint8*,
            uint32*>                                                output_set;

        // build the device input string set
        thrust::device_vector<uint32>  d_in_string( N_strings * N_words );
        thrust::device_vector<uint32>  d_in_offsets( N_strings+1 );

        packed_stream_type d_packed_stream(
            thrust::raw_pointer_cast( &d_in_string.front() ) );

        input_set d_in_string_set(
            N_strings,
            d_packed_stream,
            thrust::raw_pointer_cast( &d_in_offsets.front() ) );

        // copy the base string set into the input set
        cuda::copy( d_base_string_set, d_in_string_set );

        // build the device output string set
        thrust::device_vector<uint8>   d_out_stream( N_strings * N );
        thrust::device_vector<uint32>  d_out_lengths( N_strings );

        output_set d_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &d_out_stream.front() ),
            thrust::raw_pointer_cast( &d_out_lengths.front() ) );

        Timer timer;
        timer.start();

        // copy intput set into the output set
        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_in_string_set, d_out_string_set );

        timer.stop();

        // build the host output string set
        thrust::host_vector<uint8>  h_out_stream( d_out_stream );
        thrust::host_vector<uint32> h_out_lengths( d_out_lengths );

        output_set h_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &h_out_stream.front() ),
            thrust::raw_pointer_cast( &h_out_lengths.front() ) );

        // check that the string sets match
        check( h_base_string_set, h_out_string_set );
        fprintf(stderr, "  test packed-concat  -> strided        copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }
    // copy a packed sparse string set into a strided packed string set
    if (TEST_MASK & PACKED_SPARSE_TO_STRIDED)
    {
        fprintf(stderr, "  test packed-sparse  -> strided        copy... started\n");
        const uint32 N_words = (N_spacing + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,false>                   packed_stream_type;

        typedef PackedStream<cuda::ldg_pointer<uint32>,uint8,SYMBOL_SIZE,false> tex_packed_stream_type;

        typedef SparseStringSet<packed_stream_type,const uint2*>         input_set;
        typedef SparseStringSet<tex_packed_stream_type,const uint2*> tex_input_set;
        typedef StridedStringSet<
            uint8*,
            uint32*>                                                        output_set;

        thrust::host_vector<uint32>  h_in_string( N_strings * N_words );
        thrust::host_vector<uint2>   h_in_ranges( N_strings );

        packed_stream_type h_packed_stream(
            thrust::raw_pointer_cast( &h_in_string.front() ) );

        LCG_random rand;
        for (uint32 i = 0; i < N_strings; ++i)
        {
            h_in_ranges[i] = make_uint2( N_spacing*i, N_spacing*i + N );
            for (uint32 j = 0; j < N_spacing; ++j)
                h_packed_stream[ i * N_spacing + j ] = rand.next() & ((1u << SYMBOL_SIZE) - 1u);
        }

        // build the device input string set
        thrust::device_vector<uint32>  d_in_string( h_in_string );
        thrust::device_vector<uint2>   d_in_ranges( h_in_ranges );

        packed_stream_type d_packed_stream(
            thrust::raw_pointer_cast( &d_in_string.front() ) );

        input_set d_in_string_set(
            N_strings,
            d_packed_stream,
            thrust::raw_pointer_cast( &d_in_ranges.front() ) );

        // build the device output string set
        thrust::device_vector<uint8>   d_out_stream( N_strings * N );
        thrust::device_vector<uint32>  d_out_lengths( N_strings );

        output_set d_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &d_out_stream.front() ),
            thrust::raw_pointer_cast( &d_out_lengths.front() ) );

        Timer timer;
        timer.start();

        // copy intput set into the output set
        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_in_string_set, d_out_string_set );

        timer.stop();

        // build the host input string set
        input_set h_in_string_set(
            N_strings,
            h_packed_stream,
            thrust::raw_pointer_cast( &h_in_ranges.front() ) );

        // build the host output string set
        thrust::host_vector<uint8>  h_out_stream( d_out_stream );
        thrust::host_vector<uint32> h_out_lengths( d_out_lengths );

        output_set h_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &h_out_stream.front() ),
            thrust::raw_pointer_cast( &h_out_lengths.front() ) );

        // check that the string sets match
        check( h_in_string_set, h_out_string_set );
        fprintf(stderr, "  test packed-sparse  -> strided        copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
#if 1
        fprintf(stderr, "  test packed-sparse (tex) -> strided   copy... started\n");

        // bind the texture
        tex_packed_stream_type d_tex_packed_stream( cuda::ldg_pointer<uint32>( thrust::raw_pointer_cast( &d_in_string.front() ) ) );

        tex_input_set d_tex_in_string_set(
            N_strings,
            d_tex_packed_stream,
            thrust::raw_pointer_cast( &d_in_ranges.front() ) );

        timer.start();

        // copy intput set into the output set
        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_tex_in_string_set, d_out_string_set );

        timer.stop();
        fprintf(stderr, "  test packed-sparse (tex) -> strided   copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
#endif
    }
    // copy a strided-packed string set into a strided one
    if (TEST_MASK & STRIDED_PACKED_TO_STRIDED)
    {
        fprintf(stderr, "  test strided-packed -> strided        copy... started\n");

        typedef StridedPackedStringSet<
            uint32*,
            uint8,
            SYMBOL_SIZE,
            false,
            uint32*>                                           input_set;
        typedef StridedStringSet<uint8*,uint32*>               output_set;

        // first build the input set
        thrust::device_vector<uint32>  d_in_stream( N_strings * N_words );
        thrust::device_vector<uint32>  d_in_lengths( N_strings+1 );

        input_set d_in_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &d_in_stream.front() ),
            thrust::raw_pointer_cast( &d_in_lengths.front() ) );

        // copy the base string set into the input set
        cuda::copy( d_base_string_set, d_in_string_set );

        // build the device output string set
        thrust::device_vector<uint8>   d_out_string( N_strings * N );
        thrust::device_vector<uint32>  d_out_lengths( N_strings );

        output_set d_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &d_out_string.front() ),
            thrust::raw_pointer_cast( &d_out_lengths.front() ) );

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_in_string_set, d_out_string_set );

        timer.stop();

        // build the host output string set
        thrust::host_vector<uint8>  h_out_string( d_out_string );
        thrust::host_vector<uint32> h_out_lengths( d_out_lengths );

        output_set h_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &h_out_string.front() ),
            thrust::raw_pointer_cast( &h_out_lengths.front() ) );

        // check that the string sets match
        check( h_base_string_set, h_out_string_set );
        fprintf(stderr, "  test strided-packed -> strided        copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }
    // copy a simple concatenated string set into a strided packed string set
    if (TEST_MASK & CONCAT_TO_STRIDED_PACKED)
    {
        fprintf(stderr, "  test concat         -> strided-packed copy... started\n");

        typedef ConcatenatedStringSet<uint8*,uint32*>               input_set;
        typedef StridedPackedStringSet<
            uint32*,
            uint8,
            SYMBOL_SIZE,
            false,
            uint32*>                                                output_set;

        // first build the input set
        thrust::device_vector<uint8>   d_in_string( N_strings * N );
        thrust::device_vector<uint32>  d_in_offsets( N_strings+1 );

        input_set d_in_string_set(
            N_strings,
            thrust::raw_pointer_cast( &d_in_string.front() ),
            thrust::raw_pointer_cast( &d_in_offsets.front() ) );

        // copy the base string set into the input set
        cuda::copy( d_base_string_set, d_in_string_set );

        // build the device output string set
        thrust::device_vector<uint32>  d_out_stream( N_strings * N_words );
        thrust::device_vector<uint32>  d_out_lengths( N_strings );

        output_set d_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &d_out_stream.front() ),
            thrust::raw_pointer_cast( &d_out_lengths.front() ) );

        Timer timer;
        timer.start();

        // copy intput set into the output set
        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_in_string_set, d_out_string_set );

        timer.stop();

        // build the host output string set
        thrust::host_vector<uint32> h_out_stream( d_out_stream );
        thrust::host_vector<uint32> h_out_lengths( d_out_lengths );

        output_set h_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &h_out_stream.front() ),
            thrust::raw_pointer_cast( &h_out_lengths.front() ) );

        // check that the string sets match
        check( h_base_string_set, h_out_string_set );
        fprintf(stderr, "  test concat         -> strided-packed copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }
    // copy a packed concatenated string set into a strided packed string set
    if (TEST_MASK & PACKED_CONCAT_TO_STRIDED_PACKED)
    {
        fprintf(stderr, "  test packed-concat  -> strided-packed copy... started\n");
        const uint32 N_words    = (N + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,false> packed_stream_type;

        typedef ConcatenatedStringSet<packed_stream_type,uint32*>   input_set;
        typedef StridedPackedStringSet<
            uint32*,
            uint8,
            SYMBOL_SIZE,
            false,
            uint32*>                                                 output_set;

        // build the device input string set
        thrust::device_vector<uint32>  d_in_string( N_strings * N_words );
        thrust::device_vector<uint32>  d_in_offsets( N_strings+1 );

        packed_stream_type d_packed_stream(
            thrust::raw_pointer_cast( &d_in_string.front() ) );

        input_set d_in_string_set(
            N_strings,
            d_packed_stream,
            thrust::raw_pointer_cast( &d_in_offsets.front() ) );

        // copy the base string set into the input set
        cuda::copy( d_base_string_set, d_in_string_set );

        // build the device output string set
        thrust::device_vector<uint32>  d_out_stream( N_strings * N_words );
        thrust::device_vector<uint32>  d_out_lengths( N_strings );

        output_set d_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &d_out_stream.front() ),
            thrust::raw_pointer_cast( &d_out_lengths.front() ) );

        Timer timer;
        timer.start();

        // copy intput set into the output set
        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_in_string_set, d_out_string_set );

        timer.stop();

        // build the host output string set
        thrust::host_vector<uint32> h_out_stream( d_out_stream );
        thrust::host_vector<uint32> h_out_lengths( d_out_lengths );

        output_set h_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &h_out_stream.front() ),
            thrust::raw_pointer_cast( &h_out_lengths.front() ) );

        // check that the string sets match
        check( h_base_string_set, h_out_string_set );
        fprintf(stderr, "  test packed-concat  -> strided-packed copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }
    // copy a packed sparse string set into a strided packed string set
    if (TEST_MASK & PACKED_SPARSE_TO_STRIDED_PACKED)
    {
        fprintf(stderr, "  test packed-sparse  -> strided-packed copy... started\n");
        const uint32 N_words = (N_spacing + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

        typedef PackedStream<uint32*,uint8,SYMBOL_SIZE,false> packed_stream_type;

        typedef PackedStream<cuda::ldg_pointer<uint32>,uint8,SYMBOL_SIZE,false> tex_packed_stream_type;

        typedef SparseStringSet<packed_stream_type,const uint2*>         input_set;
        typedef SparseStringSet<tex_packed_stream_type,const uint2*> tex_input_set;
        typedef StridedPackedStringSet<
            uint32*,
            uint8,
            SYMBOL_SIZE,
            false,
            uint32*>                                                        output_set;

        thrust::host_vector<uint32>  h_in_string( N_strings * N_words );
        thrust::host_vector<uint2>   h_in_ranges( N_strings );

        packed_stream_type h_packed_stream(
            thrust::raw_pointer_cast( &h_in_string.front() ) );

        LCG_random rand;
        for (uint32 i = 0; i < N_strings; ++i)
        {
            h_in_ranges[i] = make_uint2( N_spacing*i, N_spacing*i + N );
            for (uint32 j = 0; j < N_spacing; ++j)
                h_packed_stream[ i * N_spacing + j ] = rand.next() & ((1u << SYMBOL_SIZE) - 1u);
        }

        // build the device input string set
        thrust::device_vector<uint32>  d_in_string( h_in_string );
        thrust::device_vector<uint2>   d_in_ranges( h_in_ranges );

        packed_stream_type d_packed_stream(
            thrust::raw_pointer_cast( &d_in_string.front() ) );

        input_set d_in_string_set(
            N_strings,
            d_packed_stream,
            thrust::raw_pointer_cast( &d_in_ranges.front() ) );

        // build the device output string set
        thrust::device_vector<uint32>  d_out_stream( N_strings * N_words );
        thrust::device_vector<uint32>  d_out_lengths( N_strings );

        output_set d_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &d_out_stream.front() ),
            thrust::raw_pointer_cast( &d_out_lengths.front() ) );

        Timer timer;
        timer.start();

        // copy intput set into the output set
        for (uint32 i = 0; i < N_tests; ++i)
            cuda::copy( d_in_string_set, d_out_string_set );

        timer.stop();

        // build the host input string set
        input_set h_in_string_set(
            N_strings,
            h_packed_stream,
            thrust::raw_pointer_cast( &h_in_ranges.front() ) );

        // build the host output string set
        thrust::host_vector<uint32> h_out_stream( d_out_stream );
        thrust::host_vector<uint32> h_out_lengths( d_out_lengths );

        output_set h_out_string_set(
            N_strings,
            N_strings,
            thrust::raw_pointer_cast( &h_out_stream.front() ),
            thrust::raw_pointer_cast( &h_out_lengths.front() ) );

        // check that the string sets match
        check( h_in_string_set, h_out_string_set );
        fprintf(stderr, "  test packed-sparse  -> strided-packed copy... done:   %.2f GSYMS\n", (1.0e-9f*float(N_strings*N))*(float(N_tests)/timer.seconds()));
    }
    fprintf(stderr, "nvbio/basic/string_set test... done\n");
    return 0;
}

} // namespace nvbio
