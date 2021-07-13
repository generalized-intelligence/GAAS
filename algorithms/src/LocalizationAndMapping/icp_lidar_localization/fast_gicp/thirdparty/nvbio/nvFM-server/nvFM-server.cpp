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

// nvFM-server.cpp : Defines the entry point for the console application.
//

#include <nvbio/io/fmindex/fmindex.h>
#include <nvbio/io/sequence/sequence_mmap.h>
#include <nvbio/basic/mmap.h>
#include <string.h>
#include <string>

using namespace nvbio;

int main(int argc, char* argv[])
{
    if (argc == 1)
    {
        fprintf(stderr, "nvFM-server genome-prefix mapped-name\n");
        exit(1);
    }

    fprintf(stderr, "nvFM-server started\n");

    const char* file_name = argv[1];
    const char* mapped_name = argc == 3 ? argv[2] : argv[1];

    io::SequenceDataMMAPServer reference_driver;
    reference_driver.load( DNA, file_name, mapped_name );

    io::FMIndexDataMMAPServer fmindex_driver;
    fmindex_driver.load( file_name, mapped_name );

    getc(stdin);
    return 0;
}

