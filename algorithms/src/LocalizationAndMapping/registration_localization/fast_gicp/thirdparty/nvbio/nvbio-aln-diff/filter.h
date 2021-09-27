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

#include <nvbio-aln-diff/alignment.h>
#include <stdio.h>

namespace nvbio {
namespace alndiff {

struct Filter
{
    enum Flags      { DISTANT = 1u, DISCORDANT = 2u, DIFFERENT_REF = 4u, ALL = 0xFFFFFFFFu };
    enum Statistics { ED = 1u, MAPQ = 2u, MMS = 4u, INS = 8u, DELS = 16u, SCORE = 32u };

    // empty constructor
    //
    Filter() : m_file(NULL), m_filtered(0) {}

    // constructor
    //
    // \param file_name     output file name
    // \param flags         read flags (DISTANT | DISCORDANT | DIFFERENT_REF | ALL) accepted by the filter
    // \param stats         statistics accepted by the filter
    // \param delta         filtering threshold
    //
    Filter(const char* file_name, const uint32 flags, const uint32 stats, const int32 delta) :
        m_file( NULL ),
        m_flags( flags ),
        m_stats( stats ),
        m_delta( delta ),
        m_filtered(0)
    {
        if (file_name)
        {
            log_verbose(stderr, "opening filter file \"%s\"... done\n", file_name);
            m_file = fopen( file_name, "wb" );
            if (m_file == NULL)
                log_warning( stderr, "unable to open filter file \"%s\"\n", file_name );
        }
    }
    // destructor
    //
    ~Filter()
    {
        if (m_file)
        {
            fclose( m_file );
            log_verbose(stderr, "closing filter file... done\n");
        }
    }

    // push a statistic into the filter
    //
    void operator() (const int32 delta, const uint32 flags, const Statistics stat, const uint32 read_id)
    {
        if (m_file == NULL)
            return;

        if ((m_flags & flags) &&
            (m_stats & stat) &&
            (m_delta > 0 ? delta >= m_delta : delta <= m_delta))
        {
            fwrite( &read_id, sizeof(uint32), 1u, m_file );

            ++m_filtered;
        }
    }

    // get filtered count
    //
    uint32 filtered() const { return m_filtered; }

    // flush the file
    //
    void flush() { if (m_file) fflush( m_file ); }

    FILE*  m_file;
    uint32 m_flags;
    uint32 m_stats;
    int32  m_delta;
    uint32 m_filtered;
};

} // namespace alndiff
} // namespace nvbio
