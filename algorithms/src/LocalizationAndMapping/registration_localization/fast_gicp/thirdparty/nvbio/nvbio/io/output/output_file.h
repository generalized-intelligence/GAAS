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

#include <nvbio/io/output/output_types.h>
#include <nvbio/io/output/output_stats.h>
#include <nvbio/io/sequence/sequence.h>

#include <stdio.h>
#include <string>

namespace nvbio {
namespace io {

/**
   @addtogroup IO
   @{
   @addtogroup Output
   @{
*/

/**
   The output file interface.

   This class is the base class for all file output classes. By itself, it
   implements a "null" output, which discards all data sent into it. Other
   classes derive from this to implement various file output formats.

   After each alignment pass in the pipeline, the aligner should call
   OutputFile::process. This call takes a handle to the alignment buffers in
   GPU memory (a DeviceOutputBatchSE structure). The implementation ensures that
   any data required is copied out before the call returns, allowing the
   caller to invalidate the GPU memory immediately.

   Sequence alignment is generally done in batches of reads; for each batch multiple
   OutputFile::process calls can be made. Two extra enum parameters serve to
   identify which alignment pass we're handling at each call. Each batch
   should be preceded by a OutputFile::start_batch call; at the end of each
   batch, OutputFile::end_batch should be called. In most cases, data is only
   written to disk after OutputFile::end_batch is called.

   The factory method OutputFile::open is used to create OutputFile
   objects. It parses the file name extension to determine the file format for
   the output.
*/
struct OutputFile
{

protected:
    OutputFile(const char *file_name, AlignmentType alignment_type, BNT bnt);

public:
    virtual ~OutputFile();

    void set_program(
        const char* _pg_id,
        const char* _pg_name,
        const char* _pg_version,
        const char* _pg_args)
    {
        pg_id      = _pg_id      ? _pg_id      : "";
        pg_name    = _pg_name    ? _pg_name    : "";
        pg_version = _pg_version ? _pg_version : "";
        pg_args    = _pg_args    ? _pg_args    : "";
    }

    void set_rg(
        const char* _rg_id,
        const char* _rg_string)
    {
        rg_id      = _rg_id      ? _rg_id      : "";
        rg_string  = _rg_string  ? _rg_string  : "";
    }

    /// write the header out
    ///
    virtual void header() {}

    /// Configure the MapQ evaluator. Must be called prior to any batch processing.
    ///
    virtual void configure_mapq_evaluator(int mapq_filter);

    /// Process a set of alignment results for the current batch.
    ///
    /// \param batch    Handle to the buffers containing the alignment results
    ///
    virtual void process(struct HostOutputBatchSE& batch) {}

    /// Process a set of alignment results for the current batch.
    ///
    /// \param batch    Handle to the buffers containing the alignment results
    ///
    virtual void process(struct HostOutputBatchPE& batch) {}

    /// Flush and close the output file
    virtual void close(void);

    /// Returns aggregate I/O statistics for this object
    virtual IOStats& get_aggregate_statistics(void);

protected:
    /// Name of the file we're writing
    const char *file_name;
    /// The type of alignment we're running (single or paired-end)
    AlignmentType alignment_type;
    /// Reference genome data handle
    BNT bnt;

    /// The current mapping quality filter: reads with a mapq below this value will be marked as not aligned
    int mapq_filter;

    /// I/O statistics
    IOStats iostats;

    std::string pg_id;
    std::string pg_name;
    std::string pg_version;
    std::string pg_args;

    std::string rg_id;
    std::string rg_string;

public:
    /// Factory method to create OutputFile objects
    /// \param [in] file_name The name of the file to create (will be silently overwritten if it already exists).
    ///             This method parses out the extension from the file name to determine what kind of file format to write.
    /// \param [in] aln_type The type of alignment (single or paired-end)
    /// \param [in] bnt A handle to the reference genome
    /// \return A pointer to an OutputFile object, or NULL if an error occurs.
    static OutputFile *open(const char *file_name, AlignmentType aln_type, BNT bnt);
};

/**
   @} // Output
   @} // IO
*/

} // namespace io
} // namespace nvbio
