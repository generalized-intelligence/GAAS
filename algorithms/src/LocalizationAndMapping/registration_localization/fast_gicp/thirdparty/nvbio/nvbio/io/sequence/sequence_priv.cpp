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

#include <nvbio/io/sequence/sequence_priv.h>
#include <nvbio/io/sequence/sequence_encoder.h>

#include <nvbio/io/sequence/sequence_fasta.h>
#include <nvbio/io/sequence/sequence_fastq.h>
#include <nvbio/io/sequence/sequence_txt.h>
#include <nvbio/io/sequence/sequence_sam.h>
#include <nvbio/io/sequence/sequence_bam.h>
#include <nvbio/io/sequence/sequence_pac.h>

#include <nvbio/basic/shared_pointer.h>

namespace nvbio {
namespace io {

// grab the next batch of reads into a host memory buffer
int SequenceDataFile::next(SequenceDataEncoder* encoder, const uint32 batch_size, const uint32 batch_bps)
{
    const uint32 reads_to_load = std::min(m_options.max_seqs - m_loaded, batch_size);

    if (!is_ok() || reads_to_load == 0)
        return 0;

    // a default average read length used to reserve enough space
    const uint32 AVG_READ_LENGTH = 100;

    encoder->begin_batch();
    encoder->reserve(
        batch_size,
        batch_bps == uint32(-1) ? batch_size * AVG_READ_LENGTH : batch_bps ); // try to use a default read length

    // fetch the sequence info
    const SequenceDataInfo* info = encoder->info();

    while (info->size() < reads_to_load &&
           info->bps()  < batch_bps)
    {
        // load 100 at a time if possible
        const uint32 chunk_reads = nvbio::min(reads_to_load - info->size(), uint32(100));
        const uint32 chunk_bps   = batch_bps - info->bps();

        const int n = nextChunk( encoder , chunk_reads, chunk_bps );
        assert(n <= (int) chunk_reads);
        if (n == 0)
            break;

        assert(info->size() <= reads_to_load);
    }

    m_loaded += info->size();

    encoder->end_batch();

    return info->size();
}

// factory method to open a read file, tries to detect file type based on file name
SequenceDataStream *open_sequence_file(
    const char *             sequence_file_name,
    const QualityEncoding    qualities,
    const uint32             max_seqs,
    const uint32             max_sequence_len,
    const SequenceEncoding   flags,
    const uint32             trim3,
    const uint32             trim5)
{
    // parse out file extension; look for .fastq.gz, .fastq suffixes
    uint32 len = uint32( strlen(sequence_file_name) );
    bool is_gzipped = false;

    SequenceDataFile::Options options;
    options.qualities        = qualities;
    options.max_seqs         = max_seqs;
    options.max_sequence_len = max_sequence_len;
    options.flags            = flags;
    options.trim3            = trim3;
    options.trim5            = trim5;

    if (len == 0)
    {
        // read from FASTQ from stdin
        return new SequenceDataFile_FASTQ(
            NULL,
            options );
    }

    // do we have a .gz suffix?
    if (len >= strlen(".gz"))
    {
        if (strcmp(&sequence_file_name[len - strlen(".gz")], ".gz") == 0)
        {
            is_gzipped = true;
            len = uint32(len - strlen(".gz"));
        }
    }

    // check for fasta suffix
    if (len >= strlen(".fasta"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".fasta")], ".fasta", strlen(".fasta")) == 0)
        {
            return new SequenceDataFile_FASTA_gz(
                sequence_file_name,
                options );
        }
    }
    // check for fastq suffix
    if (len >= strlen(".fa"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".fa")], ".fa", strlen(".fa")) == 0)
        {
            return new SequenceDataFile_FASTA_gz(
                sequence_file_name,
                options );
        }
    }

    // check for fastq suffix
    if (len >= strlen(".fastq"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".fastq")], ".fastq", strlen(".fastq")) == 0)
        {
            return new SequenceDataFile_FASTQ_gz(
                sequence_file_name,
                options );
        }
    }

    // check for fastq suffix
    if (len >= strlen(".fq"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".fq")], ".fq", strlen(".fq")) == 0)
        {
            return new SequenceDataFile_FASTQ_gz(
                sequence_file_name,
                options );
        }
    }

    // check for txt suffix
    if (len >= strlen(".txt"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".txt")], ".txt", strlen(".txt")) == 0)
        {
            return new SequenceDataFile_TXT_gz(
                sequence_file_name,
                options );
        }
    }

    // check for sam suffix
    if (len >= strlen(".sam"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".sam")], ".sam", strlen(".sam")) == 0)
        {
            SequenceDataFile_SAM *ret;

            ret = new SequenceDataFile_SAM(
                sequence_file_name,
                options );

            if (ret->init() == false)
            {
                delete ret;
                return NULL;
            }

            return ret;
        }
    }

    // check for bam suffix
    if (len >= strlen(".bam"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".bam")], ".bam", strlen(".bam")) == 0)
        {
            SequenceDataFile_BAM *ret;

            ret = new SequenceDataFile_BAM(
                sequence_file_name,
                options );

            if (ret->init() == false)
            {
                delete ret;
                return NULL;
            }

            return ret;
        }
    }

    // we don't actually know what this is; guess fastq
    log_warning(stderr, "could not determine file type for %s; guessing %sfastq\n", sequence_file_name, is_gzipped ? "compressed " : "");
    return new SequenceDataFile_FASTQ_gz(
        sequence_file_name,
        options );
}

// load a sequence file
//
// \param alphabet             the alphabet used to encode the sequence data
// \param sequence_data        the output sequence data
// \param sequence_file_name   the file to open
// \param load_flags           a set of flags indicating what to load
// \param qualities            the encoding of the qualities
//
bool load_sequence_file(
    const Alphabet              alphabet,
    SequenceDataHost*           sequence_data,
    const char*                 sequence_file_name,
    const SequenceFlags         load_flags,
    const QualityEncoding       qualities)
{
    // check whether this is a pac archive
    if (is_pac_archive( sequence_file_name ))
        return load_pac( alphabet, sequence_data, sequence_file_name, load_flags, qualities );

    // open a regular stream
    SharedPointer<SequenceDataStream> sequence_file( open_sequence_file( sequence_file_name, qualities ) );
    if (sequence_file == NULL || sequence_file->is_ok() == false)
        return false;

    // load as many sequences as possible in one go
    return io::next( alphabet, sequence_data, sequence_file.get(), uint32(-1), uint32(-1) ) > 0;
}


/// load a sequence file
///
/// \param alphabet             the alphabet used to encode the sequence data
/// \param sequence_file_name   the file to open
/// \param load_flags           a set of flags indicating what to load
/// \param qualities            the encoding of the qualities
///
SequenceDataHost* load_sequence_file(
    const Alphabet              alphabet,
    const char*                 sequence_file_name,
    const SequenceFlags         load_flags,
    const QualityEncoding       qualities)
{
    SequenceDataHost* ret = new SequenceDataHost;
    if (load_sequence_file( alphabet, ret, sequence_file_name, load_flags, qualities ) == false)
    {
        delete ret;
        return NULL;
    }
    return ret;
}

//\relates SequenceDataOutputStream
// factory method to open a read file for writing
//
// \param sequence_file_name   the file to open
// \param compression          compression options
//
SequenceDataOutputStream* open_output_sequence_file(
    const char* sequence_file_name,
    const char* options)
{
    // parse out file extension; look for .fastq.gz, .fastq suffixes
    uint32 len = uint32( strlen(sequence_file_name) );
    const char* gz  = "gz";
    const char* lz4 = "lz4";
    const char* compressor = NULL;

    // do we have a .gz suffix?
    if (len >= strlen(".gz"))
    {
        if (strcmp(&sequence_file_name[len - strlen(".gz")], ".gz") == 0)
        {
            compressor = gz;
            len = uint32(len - strlen(".gz"));
        }
    }

    // do we have a .gz suffix?
    if (len >= strlen(".lz4"))
    {
        if (strcmp(&sequence_file_name[len - strlen(".lz4")], ".lz4") == 0)
        {
            compressor = lz4;
            len = uint32(len - strlen(".lz4"));
        }
    }

    // check for fastq suffix
    if (len >= strlen(".fastq"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".fastq")], ".fastq", strlen(".fastq")) == 0)
        {
            return new SequenceDataOutputFile_FASTQ(
                sequence_file_name,
                compressor,
                options );
        }
    }

    // check for fastq suffix
    if (len >= strlen(".fq"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".fq")], ".fq", strlen(".fq")) == 0)
        {
            return new SequenceDataOutputFile_FASTQ(
                sequence_file_name,
                compressor,
                options );
        }
    }

    // check for fastq suffix
    if (len >= strlen(".fasta"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".fasta")], ".fasta", strlen(".fasta")) == 0)
        {
            return new SequenceDataOutputFile_FASTA(
                sequence_file_name,
                compressor,
                options );
        }
    }

    // check for fastq suffix
    if (len >= strlen(".fa"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".fa")], ".fa", strlen(".fa")) == 0)
        {
            return new SequenceDataOutputFile_FASTA(
                sequence_file_name,
                compressor,
                options );
        }
    }

    // check for fastq suffix
    if (len >= strlen(".txt"))
    {
        if (strncmp(&sequence_file_name[len - strlen(".txt")], ".txt", strlen(".txt")) == 0)
        {
            return new SequenceDataOutputFile_TXT(
                sequence_file_name,
                compressor,
                options );
        }
    }

    // we don't actually know what this is; guess fastq
    log_warning(stderr, "could not determine file type for %s; guessing fastq\n", sequence_file_name);
    return new SequenceDataOutputFile_FASTQ(
        sequence_file_name,
        compressor,
        options );
}

} // namespace io
} // namespace nvbio
