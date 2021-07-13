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

// input_thread.cu
//

#include "input_thread.h"
#include <nvbio/basic/pipeline_context.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/threads.h>
#include <nvbio/io/sequence/sequence.h>
#include <stdio.h>
#include <stdlib.h>

using namespace nvbio;

// fill the next batch
//
bool InputStage::process(nvbio::PipelineContext& context)
{
    m_data->m_mutex.lock();

    nvbio::Timer timer;
    timer.start();

    // fetch the output
    nvbio::io::SequenceDataHost* h_read_data = context.output<nvbio::io::SequenceDataHost>();

    const int ret = nvbio::io::next( DNA_N, h_read_data, m_data->m_file, m_data->m_max_strings, m_data->m_max_bps );

    timer.stop();
    m_data->m_time  += timer.seconds();
    m_data->m_reads += h_read_data->size();
    m_data->m_bps   += h_read_data->bps();

    if (h_read_data->max_sequence_len() > MAX_READ_LENGTH)
    {
        log_error(stderr, "  maximum read length exceeded: %u > %u\n", h_read_data->max_sequence_len(), MAX_READ_LENGTH);
        return false;
    }

    log_verbose(stderr, "\r  loaded reads [%llu, %llu] (%.1fM / %.2fG bps, %.1fK reads/s, %.1fM bps/s)                ",
        m_data->m_reads,
        m_data->m_reads + h_read_data->size(),
        1.0e-6f * (h_read_data->bps()),
        1.0e-9f * (m_data->m_bps + h_read_data->bps()),
        m_data->m_time ? (1.0e-3f * (m_data->m_reads + h_read_data->size())) / m_data->m_time : 0.0f,
        m_data->m_time ? (1.0e-6f * (m_data->m_bps   + h_read_data->bps() )) / m_data->m_time : 0.0f );
    log_debug_cont(stderr, "\n");

    m_data->m_mutex.unlock();

    return ret;
}
