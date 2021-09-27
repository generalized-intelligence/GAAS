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

#include <nvbio/fmindex/fmindex.h>

namespace nvbio {

///
/// helper class to hold fm index device data
///
template <uint32 OCC_INT>
struct fm_index_device_data
{
    /// constructor
    ///
    fm_index_device_data(
        const uint32  len,
        const uint32* bwt,
        const uint32* occ,
        const uint32* L2)
    {
        const uint32 WORDS = (len+16)/16;

        cudaMalloc( &m_L2,   sizeof(uint32)*5 );
        cudaMalloc( &m_bwt,  sizeof(uint32)*WORDS );
        cudaMalloc( &m_occ,  uint64(sizeof(uint32))*4*uint64(len+OCC_INT-1)/OCC_INT );

        cudaMemcpy( m_L2,   L2,   sizeof(uint32)*5,                                         cudaMemcpyHostToDevice );
        cudaMemcpy( m_occ,  occ,  uint64(sizeof(uint32))*4*uint64(len+OCC_INT-1)/OCC_INT,   cudaMemcpyHostToDevice );
        cudaMemcpy( m_bwt,  bwt,  sizeof(uint32)*WORDS,                                     cudaMemcpyHostToDevice );
    }
    /// destructor
    ////
    ~fm_index_device_data()
    {
        cudaFree( m_L2 );
        cudaFree( m_bwt );
        cudaFree( m_occ );
    }

    uint32* m_bwt;
    uint32* m_occ;
    uint32* m_L2;
};

} // namespace nvbio
