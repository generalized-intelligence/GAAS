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

#pragma once

#include <nvBowtie/bowtie2/cuda/stats.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/io/fmindex/fmindex.h>
#include <nvbio/basic/threads.h>
#include <map>
#include <string>

namespace nvbio { namespace io { struct OutputFile; } }

namespace nvbio {
namespace bowtie2 {
namespace cuda {

struct InputThreadSE;
struct InputThreadPE;
struct Aligner;
struct Stats;

struct ComputeThreadSE : public Thread<ComputeThreadSE>
{
    /// constructor
    ///
    ComputeThreadSE(
        const uint32                             _thread_id,
        const uint32                             _device_id,
        const io::SequenceData&                  _reference_data,
        const io::FMIndexData&                   _driver_data,
        const std::map<std::string,std::string>& _options,
        const Params&                            _params,
              Stats&                             _stats);

    /// gauge the favourite batch size
    ///
    uint32 gauge_batch_size();

    /// setup the input thread
    ///
    void set_input(InputThreadSE* _input_thread) { input_thread = _input_thread; }

    /// setup the input thread
    ///
    void set_output(io::OutputFile* _output_file) { output_file = _output_file; }

    void run();

    void do_run();

    const uint32                             thread_id;
    const uint32                             device_id;
    const io::SequenceData&                  reference_data_host;
    const io::FMIndexData&                   driver_data_host;
    const std::map<std::string,std::string>& options;
          InputThreadSE*                     input_thread;
          io::OutputFile*                    output_file;
          Params                             params;
          Stats&                             stats;
    SharedPointer<Aligner>                   aligner;
    SharedPointer<io::SequenceDataDevice>    reference_data_device;
    SharedPointer<io::FMIndexDataDevice>     driver_data_device;
};

struct ComputeThreadPE : public Thread<ComputeThreadPE>
{
    /// constructor
    ///
    ComputeThreadPE(
        const uint32                             _thread_id,
        const uint32                             _device_id,
        const io::SequenceData&                  _reference_data,
        const io::FMIndexData&                   _driver_data,
        const std::map<std::string,std::string>& _options,
        const Params&                            _params,
              Stats&                             _stats);

    /// gauge the favourite batch size
    ///
    uint32 gauge_batch_size();

    /// setup the input thread
    ///
    void set_input(InputThreadPE* _input_thread) { input_thread = _input_thread; }

    /// setup the input thread
    ///
    void set_output(io::OutputFile* _output_file) { output_file = _output_file; }

    void run();

    void do_run();

    const uint32                             thread_id;
    const uint32                             device_id;
    const io::SequenceData&                  reference_data_host;
    const io::FMIndexData&                   driver_data_host;
    const std::map<std::string,std::string>& options;
          InputThreadPE*                     input_thread;
          io::OutputFile*                    output_file;
          Params                             params;
          Stats&                             stats;
    SharedPointer<Aligner>                   aligner;
    SharedPointer<io::SequenceDataDevice>    reference_data_device;
    SharedPointer<io::FMIndexDataDevice>     driver_data_device;
};

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
