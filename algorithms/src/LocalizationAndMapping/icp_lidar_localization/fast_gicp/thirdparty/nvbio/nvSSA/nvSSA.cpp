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

// cuFMIndex.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <nvbio/basic/console.h>
#include <nvbio/io/fmindex/fmindex.h>

void crcInit();

using namespace nvbio;

int main(int argc, char* argv[])
{
    cudaSetDeviceFlags( cudaDeviceMapHost );

    crcInit();

    if (argc == 1)
    {
        log_info(stderr,"nvSSA [-gpu] input-prefix [output-prefix]\n");
        exit(0);
    }

    int base_arg = 0;
    const char* input;
    const char* output;
    if (strcmp( argv[1], "-gpu" ) == 0)
        base_arg = 2;
    else
        base_arg = 1;

    input = argv[base_arg];
    if (argc == base_arg+2)
        output = argv[base_arg+1];
    else
        output = argv[base_arg];

    //
    // Save sampled suffix array in a format compatible with BWA's
    //
    nvbio::io::FMIndexDataHost driver_data;
    if (!driver_data.load( input ))
        return 1;

    nvbio::io::FMIndexData::ssa_storage_type ssa, rssa;

    if (strcmp( argv[1], "-gpu" ) == 0)
    {
        nvbio::io::FMIndexDataDevice driver_data_cuda(
            driver_data,
            nvbio::io::FMIndexDataDevice::FORWARD | nvbio::io::FMIndexDataDevice::REVERSE );

        nvbio::io::FMIndexDataDevice::ssa_storage_type ssa_cuda, rssa_cuda;

        init_ssa( driver_data_cuda, ssa_cuda, rssa_cuda );

        ssa  = ssa_cuda;
        rssa = rssa_cuda;
    }
    else
        init_ssa( driver_data, ssa, rssa );

    const uint32 sa_intv = nvbio::io::FMIndexData::SA_INT;
    const uint32 ssa_len = (driver_data.m_seq_length + sa_intv) / sa_intv;

    log_info(stderr, "saving SSA... started\n");
    {
        std::string file_name = std::string( output ) + std::string(".sa");
        FILE* file = fopen( file_name.c_str(), "wb" );

        fwrite( &driver_data.m_primary,     sizeof(uint32), 1u, file );
        fwrite( &driver_data.m_L2+1,        sizeof(uint32), 4u, file );
        fwrite( &sa_intv,                   sizeof(uint32), 1u, file );
        fwrite( &driver_data.m_seq_length,  sizeof(uint32), 1u, file );
        fwrite( &ssa.m_ssa[1],              sizeof(uint32), ssa_len-1, file );
        fclose( file );
    }
    {
        std::string file_name = std::string( output ) + std::string(".rsa");
        FILE* file = fopen( file_name.c_str(), "wb" );

        fwrite( &driver_data.m_rprimary,    sizeof(uint32), 1u, file );
        fwrite( &driver_data.m_L2+1,        sizeof(uint32), 4u, file );
        fwrite( &sa_intv,                   sizeof(uint32), 1u, file );
        fwrite( &driver_data.m_seq_length,  sizeof(uint32), 1u, file );
        fwrite( &rssa.m_ssa[1],             sizeof(uint32), ssa_len-1, file );
        fclose( file );
    }
    log_info(stderr, "saving SSA... done\n");
    return 0;
}

