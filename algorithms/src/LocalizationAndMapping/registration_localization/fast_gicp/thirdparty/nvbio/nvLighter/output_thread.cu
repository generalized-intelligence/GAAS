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

// output_thread.cu
//

#include "output_thread.h"
#include "utils.h"
#include <nvbio/basic/pipeline_context.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/threads.h>
#include <nvbio/io/sequence/sequence.h>
#include <zlib/zlib.h>
#include <stdio.h>
#include <stdlib.h>

using namespace nvbio;

// fill the next batch
//
bool OutputStage::process(PipelineContext& context)
{
    try
    {
        // fetch the h_read_data
        io::SequenceDataHost* h_read_data = context.input<io::SequenceDataHost>( 0 );

        log_debug(stderr, "    writing... started (%u, %u)\n", h_read_data->size(), h_read_data->bps() );

        Timer timer;
        timer.start();

        data->m_mutex.lock();
        data->m_file->next( *h_read_data );

        if (data->m_file->is_ok() == false)
        {
            log_error(stderr, "[OutputStage] failed writing output\n");
            exit(1);
        }

        timer.stop();
        data->m_time += timer.seconds();

        log_verbose(stderr, "\r  written reads [%u, %u] (%.1fM / %.2fG bps, %.1fK reads/s, %.1fM bps/s)                ",
            data->m_reads,
            data->m_reads + h_read_data->size(),
            1.0e-6f * (h_read_data->bps()),
            1.0e-9f * (data->m_bps + h_read_data->bps()),
            data->m_time ? (1.0e-3f * (data->m_reads + h_read_data->size())) / data->m_time : 0.0f,
            data->m_time ? (1.0e-6f * (data->m_bps   + h_read_data->bps() )) / data->m_time : 0.0f );
        log_debug_cont(stderr, "\n");

        data->m_reads += h_read_data->size();
        data->m_bps   += h_read_data->bps();
        data->m_mutex.unlock();

        log_debug(stderr, "    writing... done (%.1f Mbps/s)\n", data->m_time ? (1.0e-6f * float(data->m_bps)) / data->m_time : 0.0f);
    }
    catch (nvbio::cuda_error e)
    {
        log_error(stderr, "[OutputStage] caught a nvbio::cuda_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "[OutputStage] caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "[OutputStage] caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "[OutputStage] caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (thrust::system::system_error e)
    {
        log_error(stderr, "[OutputStage] caught a thrust::system_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "[OutputStage] caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "[OutputStage] caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "[OutputStage] caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (...)
    {
        log_error(stderr, "[OutputStage] caught an unknown exception!\n");
        exit(1);
    }
    return true;
}
