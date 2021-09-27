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

// fmindex_test.cpp
//

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/dna.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/fmindex/bwt.h>

using namespace nvbio;

int bwt_test()
{
    fprintf(stderr, "bwt test... started\n");
    const int32  LEN = 10000000;
    const uint32 WORDS = (LEN+16)/16;

    const uint64 memory_footprint =
        sizeof(uint32)*uint64(WORDS) +
        sizeof(uint32)*uint64(LEN);

    fprintf(stderr, "  arch    : %lu bit\n", sizeof(void*)*8u);
    fprintf(stderr, "  length  : %.2f M bps\n", float(LEN)/1.0e6f);
    fprintf(stderr, "  memory  : %.1f MB\n", float(memory_footprint)/float(1024*1024));

    std::vector<int32> buffer( LEN+1 + WORDS, 0u );
    int32*  bwt_temp    = &buffer[0];
    uint32* base_stream = (uint32*)&buffer[0] + LEN+1;

    typedef PackedStream<uint32*,uint8,2,true> stream_type;
    stream_type stream( base_stream );

    srand(0);
    for (int32 i = 0; i < LEN; ++i)
    {
        const uint32 s = rand() % 4;
        stream[i] = s;
    }

    fprintf(stderr, "  construction... started\n");

    Timer timer;
    timer.start();

    gen_bwt( LEN, stream.begin(), &bwt_temp[0], stream.begin() );

    timer.stop();
    fprintf(stderr, "  construction... done: %um:%us\n", uint32(timer.seconds()/60), uint32(timer.seconds())%60);

    fprintf(stderr, "bwt test... done\n");
    return 0;
}