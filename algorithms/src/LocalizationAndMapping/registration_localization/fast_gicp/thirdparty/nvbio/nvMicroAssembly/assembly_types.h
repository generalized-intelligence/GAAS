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

#include <nvbio/basic/types.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/packed_vector.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/strings/infix.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/io/sequence/sequence_access.h>
#include <nvbio/basic/dna.h>

using namespace nvbio;

#define DNA_SYMBOL_BITS 2U
#define DNA_SYMBOL_MASK ((1u << DNA_SYMBOL_BITS) - 1u)

typedef nvbio::vector<device_tag, uint8> D_VectorU8;
typedef nvbio::vector<host_tag, uint8> H_VectorU8;
typedef nvbio::vector<device_tag, uint16> D_VectorU16;
typedef nvbio::vector<host_tag, uint16> H_VectorU16;
typedef nvbio::vector<device_tag, uint32> D_VectorU32;
typedef nvbio::vector<host_tag, uint32> H_VectorU32;
typedef nvbio::vector<device_tag, uint64> D_VectorU64;
typedef nvbio::vector<host_tag, uint64> H_VectorU64;
typedef nvbio::vector<device_tag, int32> D_VectorI32;
typedef nvbio::vector<host_tag, int32> H_VectorI32;
typedef nvbio::vector<device_tag, ulonglong2> D_VectorU64_2;
typedef nvbio::vector<host_tag, ulonglong2> H_VectorU64_2;

typedef nvbio::PackedVector<host_tag, 2, false, uint64> H_VectorDNA;
typedef nvbio::PackedVector<device_tag, 2, false, uint64> D_VectorDNA;
typedef H_VectorDNA::stream_type H_StreamDNA;
typedef H_VectorDNA::const_stream_type H_ConstStreamDNA;
typedef D_VectorDNA::stream_type D_StreamDNA;
typedef D_VectorDNA::const_stream_type D_ConstStreamDNA;

typedef ConcatenatedStringSet<H_StreamDNA, uint64*> H_SequenceSet;
typedef ConcatenatedStringSet<D_StreamDNA, uint64*> D_SequenceSet;
typedef string_set_infix_coord_type SequenceSetKmerCoord;
typedef nvbio::vector<host_tag, SequenceSetKmerCoord> H_VectorSetKmerCoord;
typedef nvbio::vector<device_tag, SequenceSetKmerCoord> D_VectorSetKmerCoord;

typedef nvbio::vector<host_tag, SequenceSetKmerCoord*>::iterator H_KmerIterator;
typedef nvbio::vector<device_tag, SequenceSetKmerCoord*>::iterator D_KmerIterator;

typedef nvbio::io::SequenceDataAccess<DNA>::sequence_string_set_type RefSequenceSet;

typedef const ulong2* ranges_iterator;
typedef nvbio::SparseStringSet<H_VectorDNA, ranges_iterator> H_RefRangesSet;
typedef nvbio::SparseStringSet<D_VectorDNA, ranges_iterator> D_RefRangesSet;


/* BAM IO */
typedef nvbio::PackedVector<host_tag, 4, false, uint64> H_VectorDNA16;
typedef nvbio::PackedVector<device_tag, 4, false, uint64> D_VectorDNA16;
struct cigar_op
{
    uint32 len:24, op:4;

    enum
    {
        OP_M     = 0,
        OP_I     = 1,
        OP_D     = 2,
        OP_N     = 3,
        OP_S     = 4,
        OP_H     = 5,
        OP_P     = 6,
        OP_MATCH = 7,
        OP_X     = 8,
    };

    NVBIO_HOST_DEVICE char ascii_op(void) const
    {
        return op == 0 ? 'M' :
               op == 1 ? 'I' :
               op == 2 ? 'D' :
               op == 3 ? 'N' :
               op == 4 ? 'S' :
               op == 5 ? 'H' :
               op == 6 ? 'P' :
               op == 7 ? '=' :
                         'X';
    }
};

typedef nvbio::vector<device_tag, cigar_op> D_VectorCigarOp;
typedef nvbio::vector<host_tag, cigar_op> H_VectorCigarOp;

