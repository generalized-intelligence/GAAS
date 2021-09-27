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

#include <nvbio/io/sequence/sequence.h>
#include <nvbio/io/sequence/sequence_priv.h>
#include <nvbio/basic/console.h>

namespace nvbio {
namespace io {

///@addtogroup IO
///@{

///@addtogroup SequenceIO
///@{

///@addtogroup SequenceIODetail
///@{

// SAM format description: http://samtools.sourceforge.net/SAM1.pdf

/// flag comments come from SAMtools spec
/// a better explanation is available at:
/// http://genome.sph.umich.edu/wiki/SAM#What_Information_Does_SAM.2FBAM_Have_for_an_Alignment
enum AlignmentFlags
{
    // SAMtools: template having multiple segments in sequencing
    SAMFlag_MultipleSegments = 0x1,
    // each segment properly aligned according to the aligner
    SAMFlag_AllSegmentsAligned = 0x2,
    // segment unmapped
    SAMFlag_SegmentUnmapped = 0x4,
    // next segment in the template unmapped
    SAMFlag_NextSegmentUnmapped = 0x8,
    // SEQ being reverse complemented
    SAMFlag_ReverseComplemented = 0x10,
    // SEQ of the next segment in the template being reversed
    SAMFlag_NextSegmentReverseComplemented = 0x20,
    // the first segment in the template
    SAMFlag_FirstSegment = 0x40,
    // the last segment in the template
    SAMFlag_LastSegment = 0x80,
    // secondary alignment
    SAMFlag_SecondaryAlignment = 0x100,
    // not passing quality controls
    SAMFlag_FailedQC = 0x200,
    // PCR or optical duplicate
    SAMFlag_Duplicate = 0x400,
};


/// SequenceDataFile from a BAM file
///
struct SequenceDataFile_SAM : public SequenceDataFile
{
    enum { LINE_BUFFER_INIT_SIZE = 1024 };

    enum SortOrder
    {
        SortOrder_unknown,
        SortOrder_unsorted,
        SortOrder_queryname,
        SortOrder_coordinate,
    };

    /// constructor
    ///
    SequenceDataFile_SAM(
        const char*                      read_file_name,
        const SequenceDataFile::Options& options);

    /// load the next chunk
    ///
    virtual int nextChunk(struct SequenceDataEncoder *output, uint32 max_reads, uint32 max_bps);

    /// rewind the file
    ///
    virtual bool rewind();

    bool init(void);

private:
    bool readLine(void);
    void rewindLine(void);
    bool parseHeaderLine(char *start);
    bool parseReferenceSequenceLine(char *start);

    gzFile  fp;                             ///< file pointer
    char*   linebuf;                        ///< a buffer for a full line; this will grow as needed
    int     linebuf_size;                   ///< current size of the buffer
    int     line_length;                    ///< length of the current line in the buffer
    int     numLines;                       ///< how many lines we parsed so far

    // info from the header
    char *version;                          ///< SAM version
    SortOrder sortOrder;                    ///< sorting order

public:
    // reference sequence info
    std::vector<std::string> sq_names;
    std::vector<uint64> sq_lengths;
};

}
}
