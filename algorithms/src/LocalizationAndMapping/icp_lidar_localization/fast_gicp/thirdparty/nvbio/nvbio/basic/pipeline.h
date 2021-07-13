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

// pipeline.h
//

#pragma once

#include <nvbio/basic/pipeline_context.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/threads.h>
#include <stdio.h>
#include <stdlib.h>

namespace nvbio {

namespace priv { struct PipelineThreadBase; }

///@addtogroup Basic
///@{

///@addtogroup Threads
///@{

///
/// A class implementing a parallel CPU task-pipeline.
/// The pipeline can be composed by any number of user-defined stages connected
/// as a DAG and terminating into a sink (i.e. a task with no output).
/// At run-time, each stage of the pipeline can be executed in parallel by
/// separate threads, and the run-time takes care of managing the dependencies
/// and performing multiple-buffering for each of the stages.
///
struct Pipeline
{
    /// constructor
    ///
    Pipeline() {}

    /// destructor
    ///
    ~Pipeline();

    /// append a new pipeline stage
    ///
    ///\param stage     the stage to be added
    ///\param buffers   the number of output buffers for multiple buffering
    ///\return          the stage id
    ///
    template <typename StageType>
    uint32 append_stage(StageType* stage, const uint32 buffers = 4);

    /// append the pipeline sink
    ///
    ///\param sink      the sink stage
    ///
    template <typename SinkType>
    uint32 append_sink(SinkType* sink);

    /// add a dependency
    ///
    ///\param in        the id of the producer stage
    ///\param out       the id of the consumer stage
    ///
    void add_dependency(const uint32 in, const uint32 out);

    /// run the pipeline to completion
    ///
    void run();

    std::vector<priv::PipelineThreadBase*> m_stages;
};

///@} Threads
///@} Basic

} // namespace nvbio

#include <nvbio/basic/pipeline_inl.h>
