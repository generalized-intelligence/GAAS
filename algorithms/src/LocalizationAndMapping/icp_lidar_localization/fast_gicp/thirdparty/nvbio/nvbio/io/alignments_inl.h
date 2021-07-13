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

namespace nvbio {
namespace io {

// check whether two alignments are distinct
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const uint32 pos1,
    const bool   rc1,
    const uint32 pos2,
    const bool   rc2,
    const uint32 dist)
{
    return rc1 != rc2 ? 
        true :
            pos1 >= pos2 - nvbio::min( pos2, dist ) &&
            pos1 <= pos2 + dist ? false :
                                  true;
}

// check whether two paired-end alignments are distinct
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const uint32 apos1,
    const uint32 opos1,
    const bool   arc1,
    const bool   orc1,
    const uint32 apos2,
    const uint32 opos2,
    const bool   arc2,
    const bool   orc2)
{
    return (arc1 != arc2) || (orc1 != orc2) || (apos1 != apos2) || (opos1 != opos2);
}

// check whether two paired-end alignments are distinct
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const uint32 apos1,
    const uint32 opos1,
    const bool   arc1,
    const bool   orc1,
    const uint32 apos2,
    const uint32 opos2,
    const bool   arc2,
    const bool   orc2,
    const uint32 dist)
{
    return arc1 != arc2 || orc1 != orc2 ? 
        true :
            (apos1 >= apos2 - nvbio::min( apos2, dist ) &&  apos1 <= apos2 + dist) &&
            (opos1 >= opos2 - nvbio::min( opos2, dist ) &&  opos1 <= opos2 + dist) ?
                false :
                true;
}

// check whether two alignments are distinct
//
//   NOTE: this uses Alignment::sink() which is only valid during alignment,
//   as sink() is unioned with ed().
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const Alignment& p1,
    const Alignment& p2,
    const uint32 dist)
{
    return distinct_alignments(
        p1.alignment() + p1.sink(),
        p1.is_rc(),
        p2.alignment() + p2.sink(),
        p2.is_rc(),
        dist );
}

// check whether two paired-end alignments are distinct
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const PairedAlignments& p1,
    const PairedAlignments& p2)
{
    return distinct_alignments(
        p1.mate(0).alignment() + p1.mate(0).sink(),
        p1.mate(1).alignment() + p1.mate(1).sink(),
        p1.mate(0).is_rc(),
        p1.mate(1).is_rc(),
        p2.mate(0).alignment() + p2.mate(0).sink(),
        p2.mate(1).alignment() + p2.mate(1).sink(),
        p2.mate(0).is_rc(),
        p2.mate(1).is_rc() );
}
// check whether two paired-end alignments are distinct
//
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool distinct_alignments(
    const PairedAlignments& p1,
    const PairedAlignments& p2,
    const uint32            dist)
{
    return distinct_alignments(
        p1.mate(0).alignment() + p1.mate(0).sink(),
        p1.mate(1).alignment() + p1.mate(1).sink(),
        p1.mate(0).is_rc(),
        p1.mate(1).is_rc(),
        p2.mate(0).alignment() + p2.mate(0).sink(),
        p2.mate(1).alignment() + p2.mate(1).sink(),
        p2.mate(0).is_rc(),
        p2.mate(1).is_rc(),
        dist );
}

} // namespace io
} // namespace nvbio
