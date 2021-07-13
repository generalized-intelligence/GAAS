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

#include <zlib/zlib.h>

#include <nvbio/io/bam_format.h>
#include <nvbio/io/reads/reads.h>
#include <nvbio/io/reads/reads_priv.h>
#include <nvbio/basic/console.h>

namespace nvbio {
namespace io {

///@addtogroup IO
///@{

///@addtogroup ReadsIO
///@{

///@addtogroup ReadsIODetail
///@{

/// ReadDataFile from a BAM file
///
struct ReadDataFile_BAM : public ReadDataFile
{
    /// constructor
    ///
    ReadDataFile_BAM(const char *read_file_name,
                     const uint32 max_reads,
                     const uint32 max_read_len,
                     const ReadEncoding flags);

    /// read the next chunk
    ///
    virtual int nextChunk(ReadDataRAM *output, uint32 max_reads, uint32 max_bps);

    /// initialize the stream
    ///
    bool init(void);

private:
    /// small utility function to read data from the gzip stream
    ///
    bool readData(void *output, unsigned int len);

    // our file pointer
    gzFile fp;
};

///@} // ReadsIODetail
///@} // ReadsIO
///@} // IO

} // namespace io
} // namespace nvbio
