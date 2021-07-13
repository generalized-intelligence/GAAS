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

#include <nvBowtie/bowtie2/cuda/aligner_inst.h>
#include <nvBowtie/bowtie2/cuda/aligner_all.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

void all_ed(
          Aligner&                  aligner,
    const Params&                   params,
    const FMIndexDef::type          fmi,
    const FMIndexDef::type          rfmi,
    const UberScoringScheme&        scoring_scheme,
    const io::SequenceDataDevice&   reference_data,
    const io::FMIndexDataDevice&    driver_data,
    io::SequenceDataDevice&         read_data,
    io::HostOutputBatchSE&          cpu_batch,
    Stats&                          stats)
{
    aligner.all<edit_distance_scoring_tag>(
        params,
        fmi,
        rfmi,
        scoring_scheme,
        reference_data,
        driver_data,
        read_data,
        cpu_batch,
        stats );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
