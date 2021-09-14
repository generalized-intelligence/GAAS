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

#include <stdio.h>

#include "util.h"

// pick the best GPU to run on
void gpu_init(void)
{
    int device_count = 0;
    cudaError_t err;

    cudaSetDeviceFlags(cudaDeviceMapHost | cudaDeviceLmemResizeToMax);

    err = cudaGetDeviceCount(&device_count);
    if (err != cudaSuccess)
    {
        fprintf(stderr, "gpu_init: error enumerating GPUs (%d)\n", err);
        exit(1);
    }

    if (device_count == 0)
    {
        fprintf(stderr, "gpu_init: no devices found\n");
        exit(1);
    }

    // pick the best device to run on
    cudaDeviceProp best_dev_prop;
    int best_dev = 0;
    int dev;

    cudaGetDeviceProperties(&best_dev_prop, 0);

    for(dev = 0; dev < device_count; dev++)
    {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, dev);

        if (prop.major >= best_dev_prop.major &&
            prop.minor >= best_dev_prop.minor)
        {
            best_dev_prop = prop;
            best_dev = dev;
        }
    }

    cudaSetDevice(best_dev);
    fprintf(stderr, "Running on %s (%d MB)\n", best_dev_prop.name, best_dev_prop.totalGlobalMem / 1024 / 1024);
}
