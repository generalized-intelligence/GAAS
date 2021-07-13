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

// input_thread.h
//

#pragma once

#include <nvbio/basic/pipeline_context.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/threads.h>
#include <nvbio/io/sequence/sequence.h>
#include <stdio.h>
#include <stdlib.h>

using namespace nvbio;

///
/// A small class implementing a Pipeline stage reading sequence batches from a file
///
struct InputStage
{
    typedef void                   argument_type;
    typedef io::SequenceDataHost   return_type;

    /// constructor
    ///
    ///\param file          input sequence file
    ///\param max_strings   maximum number of strings per batch
    ///\param max_bps       maximum number of base pairs per batch
    ///
    InputStage(io::SequenceDataStream* file, const uint32 max_strings, const uint32 max_bps) :
        m_file          ( file ),
        m_max_strings   ( max_strings ),
        m_max_bps       ( max_bps )
    {}

    /// fill the next batch
    ///
    bool process(PipelineContext& context)
    {
        Timer timer;
        timer.start();

        // fetch the output
        io::SequenceDataHost* output = context.output<io::SequenceDataHost>();

        log_debug(stderr, "    loading.. started (%u, %u)\n", m_max_strings, m_max_bps);
        const int ret = io::next( DNA, output, m_file, m_max_strings, m_max_bps );
        log_debug(stderr, "    loading.. done\n");

        timer.stop();
        m_time += timer.seconds();
        return ret;
    }

    io::SequenceDataStream*         m_file;
    uint32                          m_max_strings;
    uint32                          m_max_bps;
    float                           m_time;
};
