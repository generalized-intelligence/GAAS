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

// proteinsw.cu
//

#include <nvbio/basic/console.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/strings/string.h>
#include <nvbio/strings/alphabet.h>
#include <nvbio/alignment/alignment.h>
#include <nvbio/alignment/batched.h>
#include <thrust/sequence.h>
#include <stdio.h>
#include <stdlib.h>

using namespace nvbio;

int8 s_blosum62[] =
{
 4,  0, -2, -1, -2,  0, -2, -1, -1, -1, -1, -2, -4, -1, -1, -1,  1,  0,  0, -3, -2, -2, -1,  0,
 0,  9, -3, -4, -2, -3, -3, -1, -3, -1, -1, -3, -4, -3, -3, -3, -1, -1, -1, -2, -2, -3, -3, -2,
-2, -3,  6,  2, -3, -1, -1, -3, -1, -4, -3,  1, -4, -1,  0, -2,  0, -1, -3, -4, -3,  4,  1, -1,
-1, -4,  2,  5, -3, -2,  0, -3,  1, -3, -2,  0, -4, -1,  2,  0,  0, -1, -2, -3, -2,  1,  4, -1,
-2, -2, -3, -3,  6, -3, -1,  0, -3,  0,  0, -3, -4, -4, -3, -3, -2, -2, -1,  1,  3, -3, -3, -1,
 0, -3, -1, -2, -3,  6, -2, -4, -2, -4, -3,  0, -4, -2, -2, -2,  0, -2, -3, -2, -3, -1, -2, -1,
-2, -3, -1,  0, -1, -2,  8, -3, -1, -3, -2,  1, -4, -2,  0,  0, -1, -2, -3, -2,  2,  0,  0, -1,
-1, -1, -3, -3,  0, -4, -3,  4, -3,  2,  1, -3, -4, -3, -3, -3, -2, -1,  3, -3, -1, -3, -3, -1,
-1, -3, -1,  1, -3, -2, -1, -3,  5, -2, -1,  0, -4, -1,  1,  2,  0, -1, -2, -3, -2,  0,  1, -1,
-1, -1, -4, -3,  0, -4, -3,  2, -2,  4,  2, -3, -4, -3, -2, -2, -2, -1,  1, -2, -1, -4, -3, -1,
-1, -1, -3, -2,  0, -3, -2,  1, -1,  2,  5, -2, -4, -2,  0, -1, -1, -1,  1, -1, -1, -3, -1, -1,
-2, -3,  1,  0, -3,  0,  1, -3,  0, -3, -2,  6, -4, -2,  0,  0,  1,  0, -3, -4, -2,  3,  0, -1,
-4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,  1, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,
-1, -3, -1, -1, -4, -2, -2, -3, -1, -3, -2, -2, -4,  7, -1, -2, -1, -1, -2, -4, -3, -2, -1, -2,
-1, -3,  0,  2, -3, -2,  0, -3,  1, -2,  0,  0, -4, -1,  5,  1,  0, -1, -2, -2, -1,  0,  3, -1,
-1, -3, -2,  0, -3, -2,  0, -3,  2, -2, -1,  0, -4, -2,  1,  5, -1, -1, -3, -3, -2, -1,  0, -1,
 1, -1,  0,  0, -2,  0, -1, -2,  0, -2, -1,  1, -4, -1,  0, -1,  4,  1, -2, -3, -2,  0,  0,  0,
 0, -1, -1, -1, -2, -2, -2, -1, -1, -1, -1,  0, -4, -1, -1, -1,  1,  5,  0, -2, -2, -1, -1,  0,
 0, -1, -3, -2, -1, -3, -3,  3, -2,  1,  1, -3, -4, -2, -2, -3, -2,  0,  4, -3, -1, -3, -2, -1,
-3, -2, -4, -3,  1, -2, -2, -3, -3, -2, -1, -4, -4, -4, -2, -3, -3, -2, -3, 11,  2, -4, -3, -2,
-2, -2, -3, -2,  3, -3,  2, -1, -2, -1, -1, -2, -4, -3, -1, -2, -2, -2, -1,  2,  7, -3, -2, -1,
-2, -3,  4,  1, -3, -1,  0, -3,  0, -4, -3,  3, -4, -2,  0, -1,  0, -1, -3, -4, -3,  4,  1, -1,
-1, -3,  1,  4, -3, -2,  0, -3,  1, -3, -1,  0, -4, -1,  3,  0,  0, -1, -2, -3, -2,  1,  4, -1,
 0, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -4, -2, -1, -1,  0,  0, -1, -2, -1, -1, -1, -1,
};

// A scoring scheme class that will be used in conjunction with the \ref GotohAligner;
//
template <typename matrix_iterator>
struct BlosumScheme
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE BlosumScheme(const matrix_iterator m, const int32 gap_open, const int32 gap_ext) :
        m_matrix(m), m_gap_open(gap_open), m_gap_ext(gap_ext) {}

    // return the maximum match bonus at the given quality score
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 match(const uint8 q = 0) const { return 11; };

    // return the substitution score at the given (reference,query) position (r_i,q_j),
    // with values (r,q) and quality score qq
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 substitution(
        const uint32 r_i, const uint32 q_j,
        const uint8  r,  const uint8   q,
        const uint8  qq = 0) const { return int8( m_matrix[ r + q*24 ] ); };

    // return gap open and extension penalties for the pattern (query) and the text (reference)
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 pattern_gap_open()      const { return m_gap_open; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 pattern_gap_extension() const { return m_gap_ext; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 text_gap_open()         const { return m_gap_open; };
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE int32 text_gap_extension()    const { return m_gap_ext; };

    const matrix_iterator   m_matrix;
    const int32             m_gap_open;
    const int32             m_gap_ext;
};

// main test entry point
//
int main(int argc, char* argv[])
{
    log_info(stderr, "protein SW... started\n");
    const uint32 n_tests   = 10;
    const uint32 n_strings = 50000;
    const uint32 P         = 128;
    const uint32 T         = 1024;

    // alloc a device vector for holding the Blosum-62 scoring matrix
    nvbio::vector<device_tag,uint8> d_blosum62( 24*24 );

    // copy the matrix to the device
    thrust::copy( s_blosum62, s_blosum62 + 24*24, d_blosum62.begin() );

    // alloc the storage for the host strings
    nvbio::vector<host_tag,uint8>  h_pattern_strings( P * n_strings );
    nvbio::vector<host_tag,uint8>  h_text_strings( T * n_strings );

    // fill the strings with random characters
    LCG_random rand;
    for (uint32 i = 0; i < P * n_strings; ++i)
        h_pattern_strings[i] = nvbio::min( uint8( rand.next() * 24.0f ), (uint8)23u );
    for (uint32 i = 0; i < T * n_strings; ++i)
        h_text_strings[i] = nvbio::min( uint8( rand.next() * 24.0f ), (uint8)23u );

    // copy to the device
    nvbio::vector<device_tag,uint8>  d_pattern_strings( h_pattern_strings );
    nvbio::vector<device_tag,uint8>  d_text_strings( h_text_strings );
    nvbio::vector<device_tag,uint32> d_pattern_offsets( n_strings + 1 );
    nvbio::vector<device_tag,uint32> d_text_offsets( n_strings + 1 );

    // build the string offsets
    thrust::sequence( d_pattern_offsets.begin(), d_pattern_offsets.end(), 0u, P );
    thrust::sequence( d_text_offsets.begin(),    d_text_offsets.end(),    0u, T );

    // prepare a vector of alignment sinks
    nvbio::vector< device_tag, aln::BestSink< uint32 > > sinks( n_strings );

    {
        const aln::SimpleGotohScheme scoring( 1, -1, -5, -3 );

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
        {
            // and execute the batch alignment, on a GPU device
            aln::batch_alignment_score(
                aln::make_gotoh_aligner<aln::LOCAL,aln::PatternBlockingTag>( scoring ),
                make_concatenated_string_set( n_strings, (const uint8*)raw_pointer( d_pattern_strings ), (const uint32*)raw_pointer( d_pattern_offsets ) ),
                make_concatenated_string_set( n_strings, (const uint8*)raw_pointer( d_text_strings ),    (const uint32*)raw_pointer( d_text_offsets ) ),
                sinks.begin(),
                aln::DeviceThreadScheduler(),
                P, T );
        }
        cudaDeviceSynchronize();

        timer.stop();
        log_info(stderr, "  GCUPS (Constant/P): %.1f\n", (1.0e-9f * float(P*T) * float(n_strings) * float(n_tests))/timer.seconds());
    }
    {
        const aln::SimpleGotohScheme scoring( 1, -1, -5, -3 );

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
        {
            // and execute the batch alignment, on a GPU device
            aln::batch_alignment_score(
                aln::make_gotoh_aligner<aln::LOCAL,aln::TextBlockingTag>( scoring ),
                make_concatenated_string_set( n_strings, (const uint8*)raw_pointer( d_pattern_strings ), (const uint32*)raw_pointer( d_pattern_offsets ) ),
                make_concatenated_string_set( n_strings, (const uint8*)raw_pointer( d_text_strings ),    (const uint32*)raw_pointer( d_text_offsets ) ),
                sinks.begin(),
                aln::DeviceThreadScheduler(),
                P, T );
        }
        cudaDeviceSynchronize();

        timer.stop();
        log_info(stderr, "  GCUPS (Constant/T): %.1f\n", (1.0e-9f * float(P*T) * float(n_strings) * float(n_tests))/timer.seconds());
    }
    {
        const BlosumScheme< cuda::ldg_pointer<uint8> > scoring( cuda::make_ldg_pointer( raw_pointer( d_blosum62 ) ), -5, -3 );

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
        {
            // and execute the batch alignment, on a GPU device
            aln::batch_alignment_score(
                aln::make_gotoh_aligner<aln::LOCAL,aln::PatternBlockingTag>( scoring ),
                make_concatenated_string_set( n_strings, (const uint8*)raw_pointer( d_pattern_strings ), (const uint32*)raw_pointer( d_pattern_offsets ) ),
                make_concatenated_string_set( n_strings, (const uint8*)raw_pointer( d_text_strings ),    (const uint32*)raw_pointer( d_text_offsets ) ),
                sinks.begin(),
                aln::DeviceThreadBlockScheduler<128,1>(),
                P, T );
        }
        cudaDeviceSynchronize();

        timer.stop();
        log_info(stderr, "  GCUPS (Blosum62/P): %.1f\n", (1.0e-9f * float(P*T) * float(n_strings) * float(n_tests))/timer.seconds());
    }
    {
        const BlosumScheme< cuda::ldg_pointer<uint8> > scoring( cuda::make_ldg_pointer( raw_pointer( d_blosum62 ) ), -5, -3 );

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
        {
            // and execute the batch alignment, on a GPU device
            aln::batch_alignment_score(
                aln::make_gotoh_aligner<aln::LOCAL,aln::TextBlockingTag>( scoring ),
                make_concatenated_string_set( n_strings, (const uint8*)raw_pointer( d_pattern_strings ), (const uint32*)raw_pointer( d_pattern_offsets ) ),
                make_concatenated_string_set( n_strings, (const uint8*)raw_pointer( d_text_strings ),    (const uint32*)raw_pointer( d_text_offsets ) ),
                sinks.begin(),
                aln::DeviceThreadBlockScheduler<128,1>(),
                P, T );
        }
        cudaDeviceSynchronize();

        timer.stop();
        log_info(stderr, "  GCUPS (Blosum62/T): %.1f\n", (1.0e-9f * float(P*T) * float(n_strings) * float(n_tests))/timer.seconds());
    }

    log_info(stderr, "protein SW... done\n");
    return 0;
}
