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

#include <nvbio/io/sequence/sequence.h>

namespace nvbio {
namespace io {

///
/// Base class to encode a host-side SequenceData object.
/// The alphabet is provided to this class as a run-time parameter.
///
struct SequenceDataEncoder
{
    /// a set of flags describing the operators to apply to a given strand
    ///
    enum StrandOp
    {
        NO_OP                 = 0x0000,     ///< default, no operator applied
        REVERSE_OP            = 0x0001,     ///< reverse operator
        COMPLEMENT_OP         = 0x0002,     ///< complement operator
        REVERSE_COMPLEMENT_OP = 0x0003,     ///< convenience definition, same as StrandOp( REVERSE_OP | COMPLEMENT_OP )
    };

    /// constructor
    ///
    SequenceDataEncoder(const Alphabet alphabet) : m_alphabet( alphabet ) {}

    /// destructor
    ///
    virtual ~SequenceDataEncoder() {}

    /// reserve enough storage for a given number of reads and bps
    ///
    virtual void reserve(const uint32 n_reads, const uint32 n_bps) {}

    /// add a read to the end of this batch
    ///
    /// \param sequence_len                 input read length
    /// \param name                         read name
    /// \param base_pairs                   list of base pairs
    /// \param quality                      list of base qualities
    /// \param quality_encoding             quality encoding scheme
    /// \param max_sequence_len             truncate the read if longer than this
    /// \param conversion_flags             conversion operators applied to the strand
    ///
    virtual void push_back(
        const uint32            in_sequence_len,
        const char*             name,
        const uint8*            base_pairs,
        const uint8*            quality,
        const QualityEncoding   quality_encoding,
        const uint32            max_sequence_len,
        const uint32            trim3,
        const uint32            trim5,
        const StrandOp          conversion_flags)
    {
        const uint32 trimmed_len = in_sequence_len > trim3 + trim5 ?
                                   in_sequence_len - trim3 - trim5 : 0u;

        // truncate sequence
        const uint32 sequence_len = nvbio::min( trimmed_len, max_sequence_len );

        // keep stats, needed for the implementation of io::skip()
        m_info.m_sequence_stream_len += sequence_len;
        m_info.m_n_seqs++;
    }

    /// signals that a batch is to begin
    ///
    virtual void begin_batch(void) { m_info = SequenceDataInfo(); }

    /// signals that the batch is complete
    ///
    virtual void end_batch(void) {}

    /// return the sequence data info
    ///
    virtual const SequenceDataInfo* info() const { return &m_info; }

    /// get the alphabet
    ///
    Alphabet         alphabet() const { return m_alphabet; }

private:
    Alphabet         m_alphabet;
    SequenceDataInfo m_info;
};

/// create a sequence encoder
///
SequenceDataEncoder* create_encoder(const Alphabet         alphabet, SequenceDataHost* data);

} // namespace io
} // namespace nvbio
