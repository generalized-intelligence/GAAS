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

// rank_test.cu
//

#define MOD_NAMESPACE
#define MOD_NAMESPACE_NAME fmitest
#define MOD_NAMESPACE_BEGIN namespace fmitest {
#define MOD_NAMESPACE_END   }

//#define CUFMI_CUDA_DEBUG
//#define CUFMI_CUDA_ASSERTS

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/dna.h>
#include <nvbio/basic/cached_iterator.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/deinterleaved_iterator.h>
#include <nvbio/fmindex/bwt.h>
#include <nvbio/fmindex/rank_dictionary.h>

namespace nvbio {
namespace { // anonymous namespace

template <typename rank_dict_type, typename index_type>
void do_test(const index_type LEN, const rank_dict_type& dict)
{
    typedef StaticVector<index_type,4> vec4;

    vec4 counts(0u);
    for (index_type i = 0; i < LEN; ++i)
    {
        counts[ dict.text()[i] ]++;

        for (uint8 c = 0; c < 4; ++c)
        {
            const index_type r = rank( dict, i, c );

            if (r != counts[c])
            {
                log_error(stderr, "  rank mismatch at [%u:%u]: expected %u, got %u\n", uint32(i), uint32(c), uint32(counts[c]), uint32(r));
                exit(1);
            }
        }

        vec4 r4 = rank_all( dict, i );

        if (r4 != counts)
        {
            log_error(stderr, "  rank mismatch at [%u]: expected (%u,%u,%u,%u), got (%u,%u,%u,%u)\n", uint32(i),
                (uint32)counts[0], (uint32)counts[1], (uint32)counts[2], (uint32)counts[3],
                (uint32)r4[0], (uint32)r4[1], (uint32)r4[2], (uint32)r4[3]);
            exit(1);
        }
    }
}

void synthetic_test(const uint32 LEN)
{
    // 32-bits test
    {
        fprintf(stderr, "  32-bit test\n");
        const uint32 OCC_INT   = 64;
        const uint32 WORDS     = (LEN+15)/16;
        const uint32 OCC_WORDS = ((LEN+OCC_INT-1) / OCC_INT) * 4;

        Timer timer;

        const uint64 memory_footprint =
            sizeof(uint32)*WORDS +
            sizeof(uint32)*OCC_WORDS;

        fprintf(stderr, "    memory  : %.1f MB\n", float(memory_footprint)/float(1024*1024));

        thrust::host_vector<uint32> text_storage( align<4>(WORDS), 0u );
        thrust::host_vector<uint32> occ(  align<4>(WORDS), 0u );
        thrust::host_vector<uint32> count_table( 256 );

        // initialize the text
        {
            typedef PackedStream<uint32*,uint8,2,true> stream_type;
            stream_type text( &text_storage[0] );

            for (uint32 i = 0; i < LEN; ++i)
                text[i] = (rand() % 4);

            // print the string
            if (LEN < 64)
            {
                char string[64];
                dna_to_string(
                    text.begin(),
                    text.begin() + LEN,
                    string );

                fprintf(stderr, "  string : %s\n", string);
            }

            uint32 L2[5];

            // build the occurrence table
            build_occurrence_table<2u,OCC_INT>(
                text.begin(),
                text.begin() + LEN,
                &occ[0],
                &L2[1] );
        }

        // generate the count table
        gen_bwt_count_table( &count_table[0] );

        // test uint32 support
        {
            typedef PackedStream<const uint32*,uint8,2,true> stream_type;
            stream_type text( &text_storage[0] );

            typedef rank_dictionary<2u, OCC_INT, stream_type, const uint32*, const uint32*> rank_dict_type;
            rank_dict_type dict(
                text,
                &occ[0],
                &count_table[0] );

            do_test( LEN, dict );
        }
        // test uint4 support
        {
            typedef PackedStream<const uint4*,uint8,2,true> stream_type;
            stream_type text( (const uint4*)&text_storage[0] );

            typedef rank_dictionary<2u, OCC_INT, stream_type, const uint4*, const uint32*> rank_dict_type;
            rank_dict_type dict(
                text,
                (const uint4*)&occ[0],
                &count_table[0] );

            do_test( LEN, dict );
        }
    }
    // 64-bits test
    {
        fprintf(stderr, "  64-bit test\n");
        const uint32 OCC_INT   = 128;
        const uint32 WORDS     = (LEN+31)/32;
        const uint32 OCC_WORDS = ((LEN+OCC_INT-1) / OCC_INT) * 4;

        Timer timer;

        const uint64 memory_footprint =
            sizeof(uint64)*WORDS +
            sizeof(uint64)*OCC_WORDS;

        fprintf(stderr, "    memory  : %.1f MB\n", float(memory_footprint)/float(1024*1024));

        thrust::host_vector<uint64> text_storage( align<4>(WORDS), 0u );
        thrust::host_vector<uint64> occ(  align<4>(WORDS), 0u );
        thrust::host_vector<uint32> count_table( 256 );

        // initialize the text
        {
            typedef PackedStream<uint64*,uint8,2,true,uint64> stream_type;
            stream_type text( &text_storage[0] );

            for (uint32 i = 0; i < LEN; ++i)
                text[i] = (rand() % 4);

            // print the string
            if (LEN < 64)
            {
                char string[64];
                dna_to_string(
                    text.begin(),
                    text.begin() + LEN,
                    string );

                fprintf(stderr, "  string : %s\n", string);
            }

            uint64 L2[5];

            // build the occurrence table
            build_occurrence_table<2u,OCC_INT>(
                text.begin(),
                text.begin() + LEN,
                &occ[0],
                &L2[1] );
        }

        // generate the count table
        gen_bwt_count_table( &count_table[0] );

        // test uint64 support
        {
            typedef PackedStream<const uint64*,uint8,2,true,uint64> stream_type;
            stream_type text( &text_storage[0] );

            typedef rank_dictionary<2u, OCC_INT, stream_type, const uint64*, const uint32*> rank_dict_type;
            rank_dict_type dict(
                text,
                &occ[0],
                &count_table[0] );

            do_test( uint64(LEN), dict );
        }
    }
}

} // anonymous namespace

int rank_test(int argc, char* argv[])
{
    uint32 len = 10000000;

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp( argv[i], "-length" ) == 0)
            len = atoi( argv[++i] )*1000;
    }

    fprintf(stderr, "rank test... started\n");

    synthetic_test( len );

    fprintf(stderr, "rank test... done\n");
    return 0;
}

} // namespace nvbio
