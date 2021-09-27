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

// packedstream_test.cpp
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <nvbio/basic/types.h>
#include <nvbio/basic/cached_iterator.h>
#include <nvbio/basic/packedstream.h>
#include <sais.h>

using namespace nvbio;

static const uint32 LEN = 100;

template <typename CompStream>
bool check_stream(const uint8* uncomp_stream, const CompStream& comp_stream)
{
    for (uint32 i = 0; i < LEN; ++i)
    {
        if (uncomp_stream[i] != comp_stream[i])
        {
            fprintf(stderr, "  error at %u : found %u, expected %u\n", i, uint32( comp_stream[i] ), uint32( uncomp_stream[i] ));
            return false;
        }
    }
    return true;
}

template <typename CompStream>
bool check_forward_stream(const uint8* uncomp_stream, CompStream comp_stream)
{
    for (uint32 i = 0; i < LEN; ++i)
    {
        if (uncomp_stream[i] != *comp_stream)
        {
            fprintf(stderr, "  forward iterator error at %u : found %u, expected %u\n", i, uint32( *comp_stream ), uint32( uncomp_stream[i] ));
            return false;
        }
        ++comp_stream;
    }
    return true;
}

int packedstream_test()
{
    {
        uint32 base_stream[LEN] = { 0u };
        uint8  uncomp_stream[2*LEN] = { 0u };

        typedef PackedStream<uint32*,uint8,2,true> Stream;
        Stream stream( base_stream );

        fprintf(stderr, "2-bit stream test... started\n");

        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 0, uint8(3) );
        uncomp_stream[0] = uint8(3);

        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 2, uint8(2) );
        uncomp_stream[2] = uint8(2);

        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 16, uint8(3) );
        uncomp_stream[16] = uint8(3);

        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 17, uint8(1) );
        uncomp_stream[17] = uint8(1);

        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        // randomized test
        for (uint32 i = 0; i < 1000; ++i)
        {
            const uint32 j = rand() % LEN;
            const uint32 s = rand() % 4;
            stream[j] = s;
            uncomp_stream[j] = s;

            if (check_stream( uncomp_stream, stream ) == false)
                exit(1);
        }
        // randomized test
        Stream::iterator it = stream.begin();
        for (uint32 i = 0; i < 1000; ++i)
        {
            const uint32 j = rand() % LEN;
            const uint32 s = rand() % 4;
            it[j] = s;
            uncomp_stream[j] = s;

            if (check_stream( uncomp_stream, stream ) == false)
                exit(1);
        }

        std::swap( *uncomp_stream, *(uncomp_stream+1) );
        std::swap( *stream.begin(), *(stream.begin()+1) );
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        std::sort( uncomp_stream, uncomp_stream + LEN );
        std::sort( stream.begin(), stream.begin() + LEN );
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        typedef PackedStream<const_cached_iterator<uint32*>,uint8,2,true> CachedPackedStream;
        const_cached_iterator<uint32*> cached_base_stream( base_stream );
        CachedPackedStream cached_stream( cached_base_stream );
        for (uint32 i = 0; i < LEN; ++i)
        {
            if (uncomp_stream[i] != cached_stream[i])
            {
                fprintf(stderr, "  error at %u : found %u, expected %u\n", i, uint32( cached_stream[i] ), uint32( uncomp_stream[i] ));
                return false;
            }
        }

        // test the SAIS module
        stream[0] = 1; // B
        stream[1] = 0; // A
        stream[2] = 2; // N
        stream[3] = 0; // A
        stream[4] = 2; // N
        stream[5] = 0; // A

        int32 SA[6];
        saisxx( stream.begin(), SA, 6, 4 );
        if (SA[0] != 5 ||
            SA[1] != 3 ||
            SA[2] != 1 ||
            SA[3] != 0 ||
            SA[4] != 4 ||
            SA[5] != 2)
        {
            fprintf(stderr, "  wrong suffix tree for \"BANANA\"\n");
            for (uint32 i = 0; i < 6; ++i)
                fprintf(stderr, "%u : %u\n", i, SA[i]);
            exit(1);
        }
        uint32 base_bwt_stream[LEN] = { 0u };
        Stream bwt_stream( base_bwt_stream );

        saisxx_bwt( stream.begin(), bwt_stream.begin(), SA, 6, 4 );
        char bwt[7];
        for (uint32 i = 0; i < 6; ++i)
        {
            const uint8 c = bwt_stream[i];
            bwt[i] =
                c == 0 ? 'A' :
                c == 1 ? 'B' :
                c == 2 ? 'N' :
                         'T';
        }
        bwt[6] = '\0';
        if (strcmp( bwt, "ANNBAA" ) != 0)
        {
            fprintf(stderr, "  wrong bwt: expected \"ANNBAA\", got \"%s\"\n", bwt);
            exit(1);
        }

        fprintf(stderr, "2-bit stream test... done\n");
    }
    {
        uint32 base_stream[LEN] = { 0u };
        uint8  uncomp_stream[4*LEN] = { 0u };

        fprintf(stderr, "4-bit uint32-stream test... started\n");
        PackedStream<uint32*,uint8,4,true> stream( base_stream );

        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 9, uint8(5) );
        uncomp_stream[9] = uint8(5);
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 10, uint8(15) );
        uncomp_stream[10] = uint8(15);
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        // randomized test
        for (uint32 i = 0; i < 1000; ++i)
        {
            const uint32 j = rand() % LEN;
            const uint32 s = rand() % 16;
            stream[j] = s;
            uncomp_stream[j] = s;

            if (check_stream( uncomp_stream, stream ) == false)
                exit(1);
        }

        std::sort( uncomp_stream, uncomp_stream + LEN );
        std::sort( stream.begin(), stream.begin() + LEN );
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        ForwardPackedStream<const uint32*,uint8,4,true> forward_stream( stream );
        if (check_forward_stream( uncomp_stream, forward_stream ) == false)
            exit(1);

        const_cached_iterator<const uint32*> cached_base_stream( base_stream );
        PackedStream<const_cached_iterator<const uint32*>,uint8,4,true> cached_stream( cached_base_stream );
        if (check_stream( uncomp_stream, cached_stream ) == false)
            exit(1);

        fprintf(stderr, "4-bit uint32-stream test... done\n");
    }
    {
        uint4  base_stream[LEN] = { { 0u } };
        uint8  uncomp_stream[4*LEN] = { 0u };

        fprintf(stderr, "4-bit uint4-stream test... started\n");
        PackedStream<uint4*,uint8,4,true> stream( base_stream );

        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 9, uint8(5) );
        uncomp_stream[9] = uint8(5);
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 10, uint8(15) );
        uncomp_stream[10] = uint8(15);
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        // randomized test
        for (uint32 i = 0; i < 1000; ++i)
        {
            const uint32 j = rand() % LEN;
            const uint32 s = rand() % 16;
            stream[j] = s;
            uncomp_stream[j] = s;

            if (check_stream( uncomp_stream, stream ) == false)
                exit(1);
        }

        std::sort( uncomp_stream, uncomp_stream + LEN );
        std::sort( stream.begin(), stream.begin() + LEN );
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        ForwardPackedStream<const uint4*,uint8,4,true> forward_stream( stream );
        if (check_forward_stream( uncomp_stream, forward_stream ) == false)
            exit(1);

        const_cached_iterator<const uint4*> cached_base_stream( base_stream );
        PackedStream<const_cached_iterator<const uint4*>,uint8,4,true> cached_stream( cached_base_stream );
        if (check_stream( uncomp_stream, cached_stream ) == false)
            exit(1);

        fprintf(stderr, "4-bit uint4-stream test... done\n");
    }
    {
        uint32 base_stream[LEN] = { 0u };
        uint8  uncomp_stream[3*LEN] = { 0u };

        fprintf(stderr, "3-bit uint32-stream test... started\n");
        PackedStream<uint32*,uint8,3,true> stream( base_stream );

        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 9, uint8(5) );
        uncomp_stream[9] = uint8(5);
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 10, uint8(7) );
        uncomp_stream[10] = uint8(7);
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        // randomized test
        for (uint32 i = 0; i < 1000; ++i)
        {
            const uint32 j = rand() % LEN;
            const uint32 s = rand() % 8;
            stream[j] = s;
            uncomp_stream[j] = s;

            if (check_stream( uncomp_stream, stream ) == false)
                exit(1);
        }

        std::sort( uncomp_stream, uncomp_stream + LEN );
        std::sort( stream.begin(), stream.begin() + LEN );
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        fprintf(stderr, "3-bit uint32-stream test... done\n");
    }
    {
        uint8  base_stream[LEN] = { 0u };
        uint8  uncomp_stream[2*LEN] = { 0u };

        fprintf(stderr, "2-bit byte-stream test... started\n");
        PackedStream<uint8*,uint8,2,true> stream( base_stream );

        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 9, uint8(3) );
        uncomp_stream[9] = uint8(3);
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 10, uint8(2) );
        uncomp_stream[10] = uint8(2);
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        // randomized test
        for (uint32 i = 0; i < 1000; ++i)
        {
            const uint32 j = rand() % LEN;
            const uint32 s = rand() % 4;
            stream[j] = s;
            uncomp_stream[j] = s;

            if (check_stream( uncomp_stream, stream ) == false)
                exit(1);
        }

        std::sort( uncomp_stream, uncomp_stream + LEN );
        std::sort( stream.begin(), stream.begin() + LEN );
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        ForwardPackedStream<const uint8*,uint8,2,true> forward_stream( stream );
        if (check_forward_stream( uncomp_stream, forward_stream ) == false)
            exit(1);

        fprintf(stderr, "2-bit byte-stream test... done\n");
    }
    {
        uint4  base_stream[LEN] = { { 0u } };
        uint8  uncomp_stream[4*LEN] = { 0u };

        fprintf(stderr, "2-bit uint4-stream test... started\n");
        PackedStream<uint4*,uint8,2,true> stream( base_stream );

        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 9, uint8(3) );
        uncomp_stream[9] = uint8(3);
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        stream.set( 10, uint8(2) );
        uncomp_stream[10] = uint8(2);
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        // randomized test
        for (uint32 i = 0; i < 1000; ++i)
        {
            const uint32 j = rand() % LEN;
            const uint32 s = rand() % 4;
            stream[j] = s;
            uncomp_stream[j] = s;

            if (check_stream( uncomp_stream, stream ) == false)
                exit(1);
        }

        std::sort( uncomp_stream, uncomp_stream + LEN );
        std::sort( stream.begin(), stream.begin() + LEN );
        if (check_stream( uncomp_stream, stream ) == false)
            exit(1);

        fprintf(stderr, "2-bit uint4-stream test... done\n");
    }

	return 0;
}
