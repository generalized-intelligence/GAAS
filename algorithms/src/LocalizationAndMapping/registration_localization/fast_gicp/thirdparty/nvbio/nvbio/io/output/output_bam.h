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
#include <nvbio/io/output/output_utils.h>

#include <nvbio/io/output/output_file.h>
#include <nvbio/io/output/output_batch.h>
#include <nvbio/io/output/output_priv.h>
#include <nvbio/io/output/output_databuffer.h>
#include <nvbio/io/output/output_gzip.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/io/bam_format.h>
#include <nvbio/basic/threads.h>
#include <stdio.h>

namespace nvbio {
namespace io {

struct BamOutput : public OutputFile
{
private:
    // BAM alignment flags
    // these are meant to be bitwised OR'ed together
    typedef enum {
        BAM_FLAGS_PAIRED        = 1 << 16,
        BAM_FLAGS_PROPER_PAIR   = 2 << 16,
        BAM_FLAGS_UNMAPPED      = 4 << 16,
        BAM_FLAGS_MATE_UNMAPPED = 8 << 16,
        BAM_FLAGS_REVERSE       = 16 << 16,
        BAM_FLAGS_MATE_REVERSE  = 32 << 16,
        BAM_FLAGS_READ_1        = 64 << 16,
        BAM_FLAGS_READ_2        = 128 << 16,
        BAM_FLAGS_SECONDARY     = 256 << 16,
        BAM_FLAGS_QC_FAILED     = 512 << 16,
        BAM_FLAGS_DUPLICATE     = 1024 << 16
    } BamAlignmentFlags;

public:
    BamOutput(const char *file_name, AlignmentType alignment_type, BNT bnt);
    ~BamOutput();

    void header() { output_header(); }

    /// Process a set of alignment results for the current batch.
    ///
    /// \param batch    Handle to the buffers containing the alignment results
    ///
    void process(struct HostOutputBatchSE& batch);

    /// Process a set of alignment results for the current batch.
    ///
    /// \param batch    Handle to the buffers containing the alignment results
    ///
    void process(struct HostOutputBatchPE& batch);

    void close(void);

private:
    void output_header(void);
    uint32 process_one_alignment(AlignmentData& alignment, AlignmentData& mate);

    void write_block();
    void flush_blocks();

    uint32 generate_cigar(struct BAM_alignment& alnh,
                          struct BAM_alignment_data_block& alnd,
                          const AlignmentData& alignment);
    uint32 generate_md_string(BAM_alignment& alnh, BAM_alignment_data_block& alnd,
                              const AlignmentData& alignment);

    void output_tag_uint32(DataBuffer& out, const char *tag, uint32 val);
    void output_tag_uint8(DataBuffer& out, const char *tag, uint8 val);
    void output_tag_string(DataBuffer& out, const char *tag, const char *val);

    void output_alignment(BAM_alignment& alnh, BAM_alignment_data_block& alnd);

    static uint8 encode_bp(uint8 bp);

    // our file pointer
    FILE *fp;
    // CPU copy of the current alignment batch
    HostOutputBatchPE cpu_output;

    // text buffer that we're filling with data
    static const uint32     BUFFERS = 64;
    DataBuffer              data_buffers[BUFFERS];
    DataBuffer              compressed_buffers[BUFFERS];
    int32                   buffer_id;

    // our BGZF compressors
    BGZFCompressor bgzf[BUFFERS];

    Mutex mutex;
};

} // namespace io
} // namespace nvbio
