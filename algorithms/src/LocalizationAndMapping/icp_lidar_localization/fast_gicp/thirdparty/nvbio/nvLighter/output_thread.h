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

// output_thread.h
//

#pragma once

#include "utils.h"
#include <nvbio/basic/pipeline_context.h>
#include <nvbio/io/sequence/sequence.h>
#include <zlib/zlib.h>

///@addtogroup nvLighterModule
///@{

/// A small class encapsulating the core data needed by all output threads,
/// i.e. the output file and its relevant statistics
///
struct OutputStageData : public SequenceStats
{
    /// constructor
    ///
    ///\param file          input sequence file
    ///\param max_strings   maximum number of strings per batch
    ///\param max_bps       maximum number of base pairs per batch
    ///
    OutputStageData(nvbio::io::SequenceDataOutputStream* file) : m_file( file ) {}

    nvbio::io::SequenceDataOutputStream* m_file;
};

///
/// A small class implementing an nvbio::Pipeline stage reading sequence batches from a file
///
struct OutputStage
{
    typedef nvbio::io::SequenceDataHost   argument_type;

    /// empty constructor
    ///
    OutputStage() : data(NULL) {}

    /// constructor
    ///
    ///\param file          input sequence file
    ///
    OutputStage(OutputStageData* _data) : data(_data) {}

    /// fill the next batch
    ///
    bool process(nvbio::PipelineContext& context);

    OutputStageData* data;
};

///@}  // group nvLighterModule
