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

// nvbio-test.cpp
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/cuda/arch.h>
#include <cuda_runtime_api.h>

int cache_test();
int packedstream_test();
int bwt_test();
int fmindex_test(int argc, char* argv[]);
int fastq_test(const char* filename);

void crcInit();

namespace nvbio {

int alloc_test();
int syncblocks_test();
int condition_test();
int rank_test(int argc, char* argv[]);
int work_queue_test(int argc, char* argv[]);
int string_set_test(int argc, char* argv[]);
int sum_tree_test();
int qgram_test(int argc, char* argv[]);
int sequence_test(int argc, char* argv[]);
int wavelet_test(int argc, char* argv[]);
int bloom_filter_test(int argc, char* argv[]);

namespace cuda { void scan_test(); }
namespace aln { void test(int argc, char* argv[]); }
namespace html { void test(); }

} // namespace nvbio

using namespace nvbio;

enum Tests {
    kStringSet      = 1u,
    kScan           = 2u,
    kSumTree        = 16u,
    kHTML           = 32u,
    kCache          = 64u,
    kPackedStream   = 128u,
    kBWT            = 256u,
    kFMIndex        = 512u,
    kAlloc          = 1024u,
    kSyncblocks     = 2048u,
    kCondition      = 4096u,
    kWorkQueue      = 8192u,
    kAlignment      = 16384u,
    kRank           = 32768u,
    kQGram          = 65536u,
    kSequence       = 131072u,
    kWaveletTree    = 262144u,
    kBloomFilter    = 524288u,
    kALL            = 0xFFFFFFFFu
};

int main(int argc, char* argv[])
{
    try
    {
        crcInit();

        int cuda_device = -1;
        int device_count;
        cudaGetDeviceCount(&device_count);
        cuda::check_error("cuda-check");

        log_verbose(stderr, "  cuda devices : %d\n", device_count);

        uint32 tests = kALL;

        int arg = 1;
        if (argc > 1)
        {
            if (strcmp( argv[arg], "-device" ) == 0)
            {
                cuda_device = atoi(argv[++arg]);
                ++arg;
            }

            if (arg < argc)
            {
                if (strcmp( argv[arg], "-string-set" ) == 0)
                    tests = kStringSet;
                else if (strcmp( argv[arg], "-scan" ) == 0)
                    tests = kScan;
                else if (strcmp( argv[arg], "-sum-tree" ) == 0)
                    tests = kSumTree;
                else if (strcmp( argv[arg], "-aln" ) == 0)
                    tests = kAlignment;
                else if (strcmp( argv[arg], "-html" ) == 0)
                    tests = kHTML;
                else if (strcmp( argv[arg], "-cache" ) == 0)
                    tests = kCache;
                else if (strcmp( argv[arg], "-packed-stream" ) == 0)
                    tests = kPackedStream;
                else if (strcmp( argv[arg], "-bwt" ) == 0)
                    tests = kBWT;
                else if (strcmp( argv[arg], "-rank" ) == 0)
                    tests = kRank;
                else if (strcmp( argv[arg], "-fm-index" ) == 0)
                    tests = kFMIndex;
                else if (strcmp( argv[arg], "-qgram" ) == 0)
                    tests = kQGram;
                else if (strcmp( argv[arg], "-alloc" ) == 0)
                    tests = kAlloc;
                else if (strcmp( argv[arg], "-syncblocks" ) == 0)
                    tests = kSyncblocks;
                else if (strcmp( argv[arg], "-condition" ) == 0)
                    tests = kCondition;
                else if (strcmp( argv[arg], "-work-queue" ) == 0)
                    tests = kWorkQueue;
                else if (strcmp( argv[arg], "-sequence" ) == 0)
                    tests = kSequence;
                else if (strcmp( argv[arg], "-wavelet" ) == 0)
                    tests = kWaveletTree;
                else if (strcmp( argv[arg], "-bloom-filter" ) == 0)
                    tests = kBloomFilter;

                ++arg;
            }
        }

        // inspect and select cuda devices
        if (device_count)
        {
            if (cuda_device == -1)
            {
                int            best_device = 0;
                cudaDeviceProp best_device_prop;
                cudaGetDeviceProperties( &best_device_prop, best_device );

                for (int device = 0; device < device_count; ++device)
                {
                    cudaDeviceProp device_prop;
                    cudaGetDeviceProperties( &device_prop, device );
                    log_verbose(stderr, "  device %d has compute capability %d.%d\n", device, device_prop.major, device_prop.minor);
                    log_verbose(stderr, "    SM count          : %u\n", device_prop.multiProcessorCount);
                    log_verbose(stderr, "    SM clock rate     : %u Mhz\n", device_prop.clockRate / 1000);
                    log_verbose(stderr, "    memory clock rate : %.1f Ghz\n", float(device_prop.memoryClockRate) * 1.0e-6f);

                    if (device_prop.major >= best_device_prop.major &&
                        device_prop.minor >= best_device_prop.minor)
                    {
                        best_device_prop = device_prop;
                        best_device      = device;
                    }
                }
                cuda_device = best_device;
            }
            log_verbose(stderr, "  chosen device %d\n", cuda_device);
            {
                cudaDeviceProp device_prop;
                cudaGetDeviceProperties( &device_prop, cuda_device );
                log_verbose(stderr, "    device name        : %s\n", device_prop.name);
                log_verbose(stderr, "    compute capability : %d.%d\n", device_prop.major, device_prop.minor);
            }
            cudaSetDevice( cuda_device );
        }

        // allocate some heap
        cudaDeviceSetLimit( cudaLimitMallocHeapSize, 128*1024*1024 );

        argc = argc >= arg ? argc-arg : 0;

        if (tests & kAlloc)         alloc_test();
        if (tests & kSyncblocks)    syncblocks_test();
        if (tests & kCondition)     condition_test();
        if (tests & kWorkQueue)     work_queue_test( argc, argv+arg );
        if (tests & kStringSet)     string_set_test( argc, argv+arg );
        if (tests & kScan)          cuda::scan_test();
        if (tests & kAlignment)     aln::test( argc, argv+arg );
        if (tests & kSumTree)       sum_tree_test();
        if (tests & kHTML)          html::test();
        if (tests & kCache)         cache_test();
        if (tests & kPackedStream)  packedstream_test();
        if (tests & kBWT)           bwt_test();
        if (tests & kRank)          rank_test( argc, argv+arg );
        if (tests & kFMIndex)       fmindex_test( argc, argv+arg );
        if (tests & kQGram)         qgram_test( argc, argv+arg );
        if (tests & kSequence)      sequence_test( argc, argv+arg );
        if (tests & kWaveletTree)   wavelet_test( argc, argv+arg );
        if (tests & kBloomFilter)   bloom_filter_test( argc, argv+arg );

        cudaDeviceReset();
    	return 0;
    }
    catch (nvbio::cuda_error e)
    {
        log_error(stderr, "caught a nvbio::cuda_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (...)
    {
        log_error(stderr, "caught an unknown exception!\n");
        return 1;
    }
}

