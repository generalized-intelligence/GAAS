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

// alloc_test.cu
//

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/cuda/arch.h>

namespace nvbio {

int alloc_test()
{
    log_info( stderr, "alloc test... started\n" );
    const uint32 N_TESTS = 32;

    for (size_t size = 1024*1024; size <= size_t(1u << 30); size *= 4)
    {
        Timer timer;

        float cuda_malloc_time = 0.0f;
        float cuda_free_time   = 0.0f;

        float malloc_time = 0.0f;
        float free_time   = 0.0f;

        for (uint32 i = 0; i < N_TESTS; ++i)
        {
            void* ptr;

            // cuda
            timer.start();
            cudaMalloc( &ptr, size );
            timer.stop();

            cuda_malloc_time += timer.seconds();

            timer.start();
            cudaFree( ptr );
            timer.stop();

            cuda_free_time += timer.seconds();

            // cpu
            timer.start();
            ptr = malloc( size );
            timer.stop();

            malloc_time += timer.seconds();

            timer.start();
            free( ptr );
            timer.stop();

            free_time += timer.seconds();
        }

        const float GB = float(1024*1024*1024);

        cuda_malloc_time /= N_TESTS;
        cuda_free_time /= N_TESTS;
        malloc_time /= N_TESTS;
        free_time /= N_TESTS;

        log_info( stderr, "  %u MB:\n", size/(1024*1024) );
        log_info( stderr, "    cuda malloc : %.2f ms, %.3f GB/s\n", cuda_malloc_time*1000.0f, (float(size)/(cuda_malloc_time)) / GB );
        log_info( stderr, "    cuda free   : %.2f ms, %.3f GB/s\n", cuda_free_time*1000.0f,   (float(size)/(cuda_free_time)) / GB );
        log_info( stderr, "    malloc      : %.2f ms, %.3f GB/s\n", malloc_time*1000.0f,      (float(size)/(malloc_time)) / GB );
        log_info( stderr, "    free        : %.2f ms, %.3f GB/s\n", free_time*1000.0f,        (float(size)/(free_time)) / GB );
    }
    log_info( stderr, "alloc test... done\n" );
    return 0;
}

} // namespace nvbio
