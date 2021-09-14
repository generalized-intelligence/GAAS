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

/// check whether the file name points to a pac archive
///
bool is_pac_archive(const char* sequence_file_name);

/// load a sequence file
///
/// \param sequence_file_name   the file to open
/// \param qualities            the encoding of the qualities
/// \param max_seqs             maximum number of reads to input
/// \param max_sequence_len     maximum read length - reads will be truncated
/// \param flags                a set of flags indicating which strands to encode
///                             in the batch for each read.
///                             For example, passing FORWARD | REVERSE_COMPLEMENT
///                             will result in a stream containing BOTH the forward
///                             and reverse-complemented strands.
///
bool load_pac(
    const Alphabet              alphabet,
    SequenceDataHost*           sequence_data,
    const char*                 prefix,
    const SequenceFlags         load_flags,
    const QualityEncoding       qualities);

/// load a sequence file
///
/// \param sequence_file_name   the file to open
/// \param qualities            the encoding of the qualities
/// \param max_seqs             maximum number of reads to input
/// \param max_sequence_len     maximum read length - reads will be truncated
/// \param flags                a set of flags indicating which strands to encode
///                             in the batch for each read.
///                             For example, passing FORWARD | REVERSE_COMPLEMENT
///                             will result in a stream containing BOTH the forward
///                             and reverse-complemented strands.
///
bool load_pac(
    const Alphabet                  alphabet,
    struct SequenceDataMMAPServer*  sequence_data,
    const char*                     prefix,
    const char*                     mapped_name,
    const SequenceFlags             load_flags,
    const QualityEncoding           qualities);

///@} // SequenceIO
///@} // IO

} // namespace io
} // namespace nvbio
