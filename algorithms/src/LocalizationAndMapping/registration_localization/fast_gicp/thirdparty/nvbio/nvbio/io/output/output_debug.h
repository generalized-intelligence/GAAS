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
#include <nvbio/io/sequence/sequence.h>

#include <zlib/zlib.h>

namespace nvbio {
namespace io {

struct DebugOutput : public OutputFile
{
    // structure that describes one read in the debug output format
    struct DbgAlignment
    {
        uint32 read_id;
        uint32 read_len;
        uint32 alignment_pos;   // if alignment_pos is 0, nothing else is written for this alignment

        // if alignment_pos is valid, struct Info follows
    };

    struct DbgInfo
    {
        enum {
            PAIRED        = 1,
            PROPER_PAIR   = 2,
            UNMAPPED      = 4,
            MATE_UNMAPPED = 8,
            REVERSE       = 16,
            MATE_REVERSE  = 32,
            READ_1        = 64,
            READ_2        = 128,
            SECONDARY     = 256,
            QC_FAILED     = 512,
            DUPLICATE     = 1024
        };

        uint32 flag;                // from enum above
        uint32 ref_id;
        // 8-bytes
        uint8  mate;
        uint8  mapQ;
        uint8  ed;
        uint8  pad;
        // 12-bytes
        uint16 subs:16;
        uint8  ins;
        uint8  dels;
        // 16-bytes
        uint8  mms;
        uint8  gapo;
        uint8  gape;
        uint8  has_second;
        // 20-bytes
        int32  score;
        int32  sec_score;
        // 24-bytes
    };

public:
    DebugOutput(const char *file_name, AlignmentType alignment_type, BNT bnt);
    ~DebugOutput();

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
    void output_alignment(gzFile& fp, const struct DbgAlignment& al, const struct DbgInfo& info);
    void process_one_alignment(const AlignmentData& alignment, const AlignmentData& mate);
    void process_one_mate(DbgAlignment& al,
                          DbgInfo& info,
                          const AlignmentData& alignment,
                          const AlignmentData& mate,
                          const uint32 mapq);

    // our file pointers
    gzFile fp;
    gzFile fp_opposite_mate;
};

} // namespace io
} // namespace nvbio
