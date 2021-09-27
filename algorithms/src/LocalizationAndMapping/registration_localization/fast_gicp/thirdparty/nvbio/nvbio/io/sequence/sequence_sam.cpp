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

#include <stdlib.h>
#include <string.h>
#include <zlib/zlib.h>

#include <nvbio/basic/console.h>
#include <nvbio/io/sequence/sequence_sam.h>
#include <nvbio/io/sequence/sequence_encoder.h>

namespace nvbio {
namespace io {

SequenceDataFile_SAM::SequenceDataFile_SAM(
    const char*                      read_file_name,
    const SequenceDataFile::Options& options)
  : SequenceDataFile( options )
{
    fp = gzopen(read_file_name, "rt");
    if (fp == Z_NULL)
    {
        // this will cause init() to fail below
        log_error(stderr, "unable to open SAM file %s\n", read_file_name);
        m_file_state = FILE_OPEN_FAILED;
    } else {
        m_file_state = FILE_OK;
    }

    linebuf      = NULL;
    linebuf_size = 0;
    line_length  = 0;
    numLines     = 0;
    version      = NULL;
    sortOrder    = SortOrder_unknown;
}

bool SequenceDataFile_SAM::readLine(void)
{
    char *ret;
    int start_file_pos;

    // current position in linebuf that we're writing into
    // used when resizing the buffer
    int cur_buf_pos = 0;

    if (!linebuf)
    {
        // allocate initial buffer
        linebuf_size = LINE_BUFFER_INIT_SIZE;
        linebuf = (char *) malloc(linebuf_size);

        if (linebuf == NULL)
        {
            log_error(stderr, "out of memory reading SAM file\n");
            m_file_state = FILE_STREAM_ERROR;
            return false;
        }
    }

    // stash the (uncompressed) stream position we started reading from
    start_file_pos = gztell(fp);

    for(;;)
    {
        // mark the next-to-last byte with a \0
        // this serves to detect when a buffer is too small for a full line
        linebuf[linebuf_size - 2] = '\0';

        // try to read a full line
        ret = gzgets(fp, &linebuf[cur_buf_pos], linebuf_size - cur_buf_pos);
        if (ret == Z_NULL)
        {
            // EOF
            m_file_state = FILE_EOF;
            return false;
        }

        if (linebuf[linebuf_size - 2] == '\0')
        {
            break;
        } else {
            // buffer is too small, double it
            char *tmp;

            cur_buf_pos = linebuf_size - 1;

            linebuf_size *= 2;
            tmp = (char *) realloc(linebuf, linebuf_size);
            if (tmp == NULL)
            {
                log_error(stderr, "out of memory reading SAM file\n");
                m_file_state = FILE_STREAM_ERROR;
                return false;
            }

            linebuf = tmp;
        }
    }

    // store the byte size of the line we read
    line_length = gztell(fp) - start_file_pos;

    // chop off the newline character if any
    if (linebuf[line_length - 1] == '\n')
    {
        assert(linebuf[line_length] == '\0');
        linebuf[line_length - 1] = '\0';
    }

    numLines++;
    return true;
}

void SequenceDataFile_SAM::rewindLine(void)
{
    assert(line_length);
    gzseek(fp, -line_length, SEEK_CUR);
}

// initializes a SAM file
// will consume the SAM header and prepare for fetching reads from the file
// returns false on failure
bool SequenceDataFile_SAM::init(void)
{
    bool ret;

    if (m_file_state != FILE_OK)
    {
        // file failed to open
        return false;
    }

    // read the header section
    do {
        ret = readLine();
        if (!ret)
        {
            return false;
        }

        if (linebuf[0] != '@')
        {
            break;
        }

        char *delim;
        delim = strchr(linebuf, '\t');

        if (delim)
        {
            if (strncmp(linebuf, "@HD\t", strlen("@HD\t")) == 0)
            {
                ret = parseHeaderLine(delim + 1);
                if (!ret)
                {
                    return false;
                }
            } else if (strncmp(linebuf, "@SQ\t", strlen("@SQ\t")) == 0)
            {
                ret = parseReferenceSequenceLine(delim + 1);
                if (!ret)
                {
                    return false;
                }
            } else if (strncmp(linebuf, "@RG\t", strlen("@RG\t")) == 0) {
                // ignored
                continue;
            } else if (strncmp(linebuf, "@PG\t", strlen("@PG\t")) == 0) {
                // ignored
                continue;
            } else if (strncmp(linebuf, "@CO\t", strlen("@CO\t")) == 0) {
                // ignored
                continue;
            } else {
                log_warning(stderr, "SAM file warning: unknown header at line %d\n", numLines);
            }
        } else {
            log_warning(stderr, "SAM file warning: malformed line %d\n", numLines);
        }
    } while(linebuf[0] == '@');

    // rewind the last line
    rewindLine();

    return true;
}

// parse a @HD line from the SAM file
// start points at the first tag of the line
bool SequenceDataFile_SAM::parseHeaderLine(char *start)
{
    char *version = NULL;
    char *delim;

    if (numLines != 1)
    {
        log_warning(stderr, "SAM file warning (line %d): @HD not the first line in the header section\n", numLines);
    }

    for(;;)
    {
        // look for the next delimiter
        delim = strchr(start, '\t');

        // zero out the next delimiter if found
        if (delim)
        {
            *delim = 0;
        }

        if (strncmp(start, "VN:", strlen("VN:")) == 0)
        {
            version = &start[3];
        } else if (strncmp(start, "SO:", strlen("SO:")) == 0) {
            if(strcmp(&start[3], "unknown") == 0)
            {
                sortOrder = SortOrder_unknown;
            } else if (strcmp(&start[3], "unsorted") == 0) {
                sortOrder = SortOrder_unsorted;
            } else if (strcmp(&start[3], "queryname") == 0) {
                sortOrder = SortOrder_queryname;
            } else if (strcmp(&start[3], "coordinate") == 0) {
                sortOrder = SortOrder_coordinate;
            } else {
                log_warning(stderr, "SAM file warning (line %d): invalid sort order %s\n", numLines, &start[3]);
            }
        } else {
            log_warning(stderr, "SAM file warning (line %d): invalid tag %s in @HD\n", numLines, start);
        }

        if (!delim)
        {
            // this was the last token
            break;
        }

        // advance to next token
        start = delim + 1;
    }

    if (version == NULL)
    {
        log_warning(stderr, "SAM file warning (line %d): header does not contain a version tag\n", numLines);
    }

    return true;
}

// parse a @SQ line from the SAM file
// start points at the first tag of the line
bool SequenceDataFile_SAM::parseReferenceSequenceLine(char *start)
{
    char *seq_name = NULL;
    char *seq_len = NULL;
    char *delim;

    for(;;)
    {
        // look for the next delimiter
        delim = strchr(start, '\t');

        // zero out the next delimiter if found
        if (delim)
        {
            *delim = 0;
        }

        if (strncmp(start, "SN:", strlen("SN:")) == 0)
        {
            if (seq_name != NULL)
            {
                log_warning(stderr, "SAM file warning (line %d): multiple SN tags in @SQ record\n", numLines);
            } else {
                seq_name = &start[3];
            }
        } else if (strncmp(start, "LN:", strlen("LN:")) == 0) {
            if (seq_len != NULL)
            {
                log_warning(stderr, "SAM file warning (line %d): multiple LN tags in @SQ record\n", numLines);
            } else {
                seq_len = &start[3];
            }
        }

        if (!delim)
        {
            // this was the last token
            break;
        }

        // advance to next token
        start = delim + 1;
    }

    if (seq_name == NULL || seq_len == NULL)
    {
        log_warning(stderr, "SAM file warning (line %d): missing required tags in @SQ record\n", numLines);
        return true;
    }

    char *endptr = NULL;
#if WIN32
    uint64 len = strtol(seq_len, &endptr, 10);
#else
    uint64 len = strtoll(seq_len, &endptr, 10);
#endif
    if (!endptr || endptr == seq_len || *endptr != '\0')
    {
        log_warning(stderr, "SAM file warning (line %d): invalid sequence length in @SQ record\n", numLines);
    }

    sq_names.push_back(std::string(seq_name));
    sq_lengths.push_back(len);

    return true;
}

// rewind
//
bool SequenceDataFile_SAM::rewind()
{
    if (fp == NULL)
        return false;

    gzrewind( fp );

    linebuf      = NULL;
    linebuf_size = 0;
    line_length  = 0;
    numLines     = 0;
    version      = NULL;
    sortOrder    = SortOrder_unknown;

    m_file_state = FILE_OK;
    return init();
}

// fetch the next chunk of reads (up to max_reads) from the file and push it into output
int SequenceDataFile_SAM::nextChunk(SequenceDataEncoder *output, uint32 max_reads, uint32 max_bps)
{
    if (max_bps < SequenceDataFile::LONG_READ)
        return 0;

    char *name;
    char *flag;
    char *rname;
    char *pos;
    char *mapq;
    char *cigar;
    char *rnext;
    char *pnext;
    char *tlen;
    char *seq;
    char *qual;

    uint32 read_flags;

    // find the next primary alignment from the file
    do {
        // get next line from file
        if (readLine() == false)
        {
            return 0;
        }

// ugly macro to tokenize the string based on strchr
#define NEXT(prev, next)                        \
    {                                           \
        next = strchr(prev, '\t');              \
        if (!next) {                                                    \
            log_error(stderr, "Error parsing SAM file (line %d): incomplete alignment section\n", numLines); \
            m_file_state = FILE_PARSE_ERROR;                            \
            return -1;                                                  \
        }                                                               \
        *next = '\0';                                                   \
        next++;                                                         \
    }

        // first token is just the start of the string
        name = linebuf;

        // for all remaining tokens, locate the next token based on the previous
        NEXT(name, flag);
        NEXT(flag, rname);
        NEXT(rname, pos);
        NEXT(pos, mapq);
        NEXT(mapq, cigar);
        NEXT(cigar, rnext);
        NEXT(rnext, pnext);
        NEXT(pnext, tlen);
        NEXT(tlen, seq);
        NEXT(seq, qual);

#undef NEXT

        // figure out what the flag value is
        read_flags = strtol(flag, NULL, 0);
    } while(read_flags & SAMFlag_SecondaryAlignment);

    if (m_options.flags & FORWARD)
    {
        const SequenceDataEncoder::StrandOp op = (read_flags & SAMFlag_ReverseComplemented) ?
              SequenceDataEncoder::REVERSE_COMPLEMENT_OP : SequenceDataEncoder::NO_OP;

        // add the read
        output->push_back(uint32(strlen(seq)),
                          name,
                          (uint8*)seq,
                          (uint8*)qual,
                          Phred33,
                          m_options.max_sequence_len,
                          m_options.trim3,
                          m_options.trim5,
                          op );
    }
    if (m_options.flags & REVERSE)
    {
        const SequenceDataEncoder::StrandOp op = (read_flags & SAMFlag_ReverseComplemented) ?
              SequenceDataEncoder::COMPLEMENT_OP : SequenceDataEncoder::REVERSE_OP;

        // add the read
        output->push_back(uint32(strlen(seq)),
                          name,
                          (uint8*)seq,
                          (uint8*)qual,
                          Phred33,
                          m_options.max_sequence_len,
                          m_options.trim3,
                          m_options.trim5,
                          op );
    }
    if (m_options.flags & FORWARD_COMPLEMENT)
    {
        const SequenceDataEncoder::StrandOp op = (read_flags & SAMFlag_ReverseComplemented) ?
              SequenceDataEncoder::REVERSE_OP : SequenceDataEncoder::COMPLEMENT_OP;

        // add the read
        output->push_back(uint32(strlen(seq)),
                          name,
                          (uint8*)seq,
                          (uint8*)qual,
                          Phred33,
                          m_options.max_sequence_len,
                          m_options.trim3,
                          m_options.trim5,
                          op );
    }
    if (m_options.flags & REVERSE_COMPLEMENT)
    {
        const SequenceDataEncoder::StrandOp op = (read_flags & SAMFlag_ReverseComplemented) ?
              SequenceDataEncoder::NO_OP : SequenceDataEncoder::REVERSE_COMPLEMENT_OP;

        // add the read
        output->push_back(uint32(strlen(seq)),
                          name,
                          (uint8*)seq,
                          (uint8*)qual,
                          Phred33,
                          m_options.max_sequence_len,
                          m_options.trim3,
                          m_options.trim5,
                          op );
    }

    // we always input 1 read at a time here
    return 1;
}

} // namespace io
} // namespace nvbio
