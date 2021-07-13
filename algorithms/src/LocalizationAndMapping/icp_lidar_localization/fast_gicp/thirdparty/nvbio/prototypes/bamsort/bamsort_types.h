/*
 * Copyright (c) 2012-14, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 *
 *
 *
 *
 *
 *
 *
 *
 */

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/packed_vector.h>

using namespace nvbio;


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

typedef nvbio::PackedVector<host_tag, 4, false, uint64> H_VectorDNA16;
typedef nvbio::PackedVector<device_tag, 4, false, uint64> D_VectorDNA16;
typedef H_VectorDNA16::stream_type H_StreamDNA16;
typedef D_VectorDNA16::stream_type D_StreamDNA16;

typedef nvbio::PackedVector<host_tag, 1> H_PackedVector_1b;
typedef nvbio::PackedVector<device_tag, 1> D_PackedVector_1b;

typedef nvbio::PackedVector<host_tag, 2> H_PackedVector_2b;
typedef nvbio::PackedVector<device_tag, 2> D_PackedVector_2b;

// this is evil: endianess between reference data and sequence data doesn't seem to match...
typedef nvbio::PackedVector<host_tag, 2, true>::stream_type H_PackedReference;
typedef nvbio::PackedVector<device_tag, 2, true>::stream_type D_PackedReference;

typedef nvbio::vector<device_tag, uint2> D_VectorU32_2;
typedef nvbio::vector<host_tag, uint2> H_VectorU32_2;
typedef nvbio::vector<device_tag, ushort2> D_VectorU16_2;
typedef nvbio::vector<host_tag, ushort2> H_VectorU16_2;

// we only use 1 bit per entry on the active location list
// however, because we write to this using multiple threads that are scanning reads concurrently,
// we need to make sure we don't access the same dword from different threads
// sizing this to 4 bits per symbol ensures that, because input reads are padded to a dword boundary
typedef D_VectorDNA16 D_ActiveLocationList;
typedef H_VectorDNA16 H_ActiveLocationList;
typedef D_ActiveLocationList::plain_view_type D_ActiveLocationStream;
typedef H_ActiveLocationList::plain_view_type H_ActiveLocationStream;

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
