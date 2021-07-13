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

#include <nvbio-aln-diff/alignment.h>
#include <contrib/bamtools/BamReader.h>
#include <nvbio/basic/console.h>
#include <crc/crc.h>

namespace nvbio {
namespace alndiff {

struct BAMAlignmentStream : public AlignmentStream
{
    BAMAlignmentStream(const char* file_name)
    {
        log_verbose(stderr, "opening BAM file \"%s\"... started\n", file_name);
        m_bam_reader.Open( file_name );
        m_offset = 0;
        log_verbose(stderr, "opening BAM file \"%s\"... done\n", file_name);
    }

    // return if the stream is ok
    //
    bool is_ok() { return true; } // TODO: add a mechanism to bamtools to know whether the file opened correctly

    // get the next batch
    //
    uint32 next_batch(
        const uint32    count,
        Alignment*      batch)
    {
        uint32 n_read = 0;

        while (n_read < count)
        {
            Alignment* aln = batch + n_read;

            // clean the alignment
            *aln = Alignment();

            BamTools::BamAlignment bam_aln;

            if (m_bam_reader.GetNextAlignment( bam_aln ) == false)
                break;

            aln->read_id  = uint32( crcCalc( bam_aln.Name.c_str(), uint32(bam_aln.Name.length()) ) );
            aln->read_len = bam_aln.Length;
            aln->mate     = bam_aln.IsFirstMate() ? 0u : 1u;
            aln->flag     = bam_aln.AlignmentFlag;
            aln->pos      = bam_aln.Position;
            if (aln->is_mapped())
            {
                aln->ref_id   = bam_aln.RefID;
                aln->mapQ     = uint8( bam_aln.MapQuality );
                bam_aln.GetEditDistance( aln->ed );

                analyze_cigar( bam_aln.CigarData, aln );

                bam_aln.GetTag( "AS", aln->score );
                aln->has_second = bam_aln.GetTag( "XS", aln->sec_score );
                bam_aln.GetTag( "XM", aln->n_mm );
                bam_aln.GetTag( "XO", aln->n_gapo );
                bam_aln.GetTag( "XG", aln->n_gape );

                const char* md = bam_aln.GetTag( "MD" );
                if (md)
                    analyze_md( md, aln );
            }

            ++n_read;
        }
        m_offset += n_read;
        return n_read;
    }

    void analyze_cigar(const std::vector<BamTools::CigarOp>& cigar, Alignment* aln)
    {
        aln->subs = aln->ins = aln->dels = 0;

        for (uint32 i = 0; i < cigar.size(); ++i)
        {
            const BamTools::CigarOp op = cigar[i];

            if (op.Type == 'X')
                ++aln->n_mm;

            if (op.Type == 'M' || op.Type == 'X' || op.Type == '=')
                aln->subs += op.Length;
            else if (op.Type == 'I')
                aln->ins  += op.Length;
            else if (op.Type == 'D')
                aln->dels += op.Length;
        }
    }
    void analyze_md(const char* md, Alignment* aln)
    {
        aln->n_mm = 0;

        for (; *md != '\0'; ++md)
        {
            const char c = *md;

            if (c >= '0' &&
                c <= '9')
                continue;

            if (c >= 'A' &&
                c <= 'Z')
                ++aln->n_mm;

            if (c == '^')
            {
                // a deletion, skip it
                for (++md; *md != '\0' && (*md <= '0' || *md >= '9'); ++md) {}
            }
        }
    }

    BamTools::BamReader m_bam_reader;
    uint32              m_offset;
};

AlignmentStream* open_bam_file(const char* file_name)
{
    return new BAMAlignmentStream( file_name );
}

} // alndiff namespace
} // nvbio namespace


