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

#include <nvbio/fasta/fasta.h>

using namespace nvbio;

namespace {

struct Writer
{
    Writer() : n(0) {}

    void push_back(const char* id, const uint32 read_len, const uint8* bp)
    {
#if 0
        if ((n & 0x00FF) == 0)
        {
            char read[16*1024];
            for (uint32 i = 0; i < read_len; ++i)
                read[i] = bp[i];
            read[read_len] = '\0';

            fprintf( stderr, "  len: %u, read: %s\n", read_len, read );
        }
#endif
        n++;
    }

    uint32 n;
};

} // anonymous namespace

int fasta_test(const char* filename)
{
    fprintf(stderr, "FASTA test... started\n");

    FASTA_reader fasta( filename );
    if (fasta.valid() == false)
    {
        fprintf(stderr, "*** error *** : file \"%s\" not found\n", filename);
        return 1;
    }

    Writer writer;

    int n;

    while ((n = fasta.read( 100u, writer )))
    {
        if (n < 0)
        {
            fprintf(stderr, "*** parsing error ***\n");
            return 1;
        }
    }

    fprintf(stderr, "FASTA test... done: %u reads\n", writer.n);
    return 0;
}
