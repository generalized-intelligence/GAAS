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

#include <nvbio-aln-diff/se_analyzer.h>
#include <nvbio-aln-diff/pe_analyzer.h>
#include <nvbio-aln-diff/alignment.h>
#include <nvbio-aln-diff/utils.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/html.h>
#include <nvbio/basic/shared_pointer.h>
#include <cuda_runtime_api.h>
#include <vector_types.h>
#include <vector_functions.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <string>

void crcInit();

using namespace nvbio;
using namespace alndiff;


int main(int argc, char* argv[])
{
    crcInit();

    int cuda_device = -1;
    int device_count;
    cudaGetDeviceCount(&device_count);
    log_verbose(stderr, "  cuda devices : %d\n", device_count);

    const char* report_name = NULL;

    const char* filter_name = NULL;
    uint32 filter_flags = Filter::ALL;
    uint32 filter_stats = Filter::ALL;
    int32  filter_delta = 5;
    bool   paired = false;
    bool   id_check = true;

    int arg = 1;
    while (arg < argc)
    {
        if (strcmp( argv[arg], "-device" ) == 0)
        {
            cuda_device = atoi(argv[++arg]);
            ++arg;
        }
        else if (strcmp( argv[arg], "-paired" ) == 0)
        {
            paired = true;
            ++arg;
        }
        else if (strcmp( argv[arg], "-report" ) == 0)
        {
            report_name = argv[++arg];
            ++arg;
        }
        else if (strcmp( argv[arg], "-no-ids" ) == 0)
        {
            id_check = false;
            ++arg;
        }
        else if (strcmp( argv[arg], "-filter" ) == 0)
        {
            filter_name = argv[++arg];

            const std::string flags_string( argv[++arg] );
            const std::string stats_string( argv[++arg] );
            filter_delta = atoi(argv[++arg]);
            ++arg;

            char temp[256];

            // parse the filter flags
            const char* begin = flags_string.c_str();
            const char* end   = begin;
            filter_flags      = 0u;

            while (1)
            {
                while (*end != '|' && *end != '\0')
                {
                    temp[end - begin] = *end;
                    end++;
                }

                temp[end - begin] = '\0';

                if (strcmp( temp, "distant" ) == 0)
                    filter_flags |= Filter::DISTANT;
                else if (strcmp( temp, "discordant" ) == 0)
                    filter_flags |= Filter::DISCORDANT;
                else if (strcmp( temp, "diff-ref" ) == 0)
                    filter_flags |= Filter::DIFFERENT_REF;

                if (*end == '\0')
                    break;

                ++end; begin = end;
            }

            // parse the filter stats
            begin        = stats_string.c_str();
            end          = begin;
            filter_stats = 0u;

            while (1)
            {
                while (*end != '|' && *end != '\0')
                {
                    temp[end - begin] = *end;
                    end++;
                }

                temp[end - begin] = '\0';

                if (strcmp( temp, "ed" ) == 0)
                    filter_stats |= Filter::ED;
                else if (strcmp( temp, "mapQ" ) == 0)
                    filter_stats |= Filter::MAPQ;
                else if (strcmp( temp, "mms" ) == 0)
                    filter_stats |= Filter::MMS;
                else if (strcmp( temp, "ins" ) == 0)
                    filter_stats |= Filter::INS;
                else if (strcmp( temp, "dels" ) == 0)
                    filter_stats |= Filter::DELS;

                if (*end == '\0')
                    break;

                ++end; begin = end;
            }
        }
        else
            break;
    }

    // inspect and select cuda devices
    if (device_count)
    {
        if (cuda_device == -1)
        {
            int            best_device = 0;
            cudaDeviceProp best_device_prop;
            cudaGetDeviceProperties( &best_device_prop, best_device );

            for (int device = 0; device < device_count; ++device)
            {
                cudaDeviceProp device_prop;
                cudaGetDeviceProperties( &device_prop, device );
                log_verbose(stderr, "  device %d has compute capability %d.%d\n", device, device_prop.major, device_prop.minor);
                log_verbose(stderr, "    SM count          : %u\n", device_prop.multiProcessorCount);
                log_verbose(stderr, "    SM clock rate     : %u Mhz\n", device_prop.clockRate / 1000);
                log_verbose(stderr, "    memory clock rate : %.1f Ghz\n", float(device_prop.memoryClockRate) * 1.0e-6f);

                if (device_prop.major >= best_device_prop.major &&
                    device_prop.minor >= best_device_prop.minor)
                {
                    best_device_prop = device_prop;
                    best_device      = device;
                }
            }
            cuda_device = best_device;
        }
        log_verbose(stderr, "  chosen device %d\n", cuda_device);
        {
            cudaDeviceProp device_prop;
            cudaGetDeviceProperties( &device_prop, cuda_device );
            log_verbose(stderr, "    device name        : %s\n", device_prop.name);
            log_verbose(stderr, "    compute capability : %d.%d\n", device_prop.major, device_prop.minor);
        }
        cudaSetDevice( cuda_device );
    }

    if (argc < arg + 2)
    {
        log_info(stderr, "nvbio-aln-diff [OPTIONS] <file1> <file2>\n");
        log_info(stderr, "OPTIONS:\n");
        log_info(stderr, "  -paired                   # paired-end input\n" );
        log_info(stderr, "  -no-ids                   # do not perform id checks\n" );
        log_info(stderr, "  -report <file-name>       # HTML report\n" );
        log_info(stderr, "  -filter <file-name>\n" );
        log_info(stderr, "          <flags={distant|discordant|diff-ref}>\n" );
        log_info(stderr, "          <stats={ed|mapQ|mms|ins|dels}>\n" );
        log_info(stderr, "          int:delta\n" );
        return 0;
    }

    if (filter_name)
    {
        std::string filter_flags_string;
        if (filter_flags & Filter::DISTANT)
            filter_flags_string += std::string(filter_flags_string.length() ? "|" : "") + "distant";
        if (filter_flags & Filter::DISCORDANT)
            filter_flags_string += std::string(filter_flags_string.length() ? "|" : "") + "discordant";
        if (filter_flags & Filter::DIFFERENT_REF)
            filter_flags_string += std::string(filter_flags_string.length() ? "|" : "") + "diff-ref";

        std::string filter_stats_string;
        if (filter_stats & Filter::ED)
            filter_stats_string += std::string(filter_stats_string.length() ? "|" : "") + "ed";
        if (filter_stats & Filter::MAPQ)
            filter_stats_string += std::string(filter_stats_string.length() ? "|" : "") + "mapQ";
        if (filter_stats & Filter::MMS)
            filter_stats_string += std::string(filter_stats_string.length() ? "|" : "") + "mms";
        if (filter_stats & Filter::INS)
            filter_stats_string += std::string(filter_stats_string.length() ? "|" : "") + "ins";
        if (filter_stats & Filter::DELS)
            filter_stats_string += std::string(filter_stats_string.length() ? "|" : "") + "dels";

        log_info(stderr, "filter file : %s\n", filter_name);
        log_info(stderr, "  flags : %s\n", filter_flags_string.c_str());
        log_info(stderr, "  stats : %s\n", filter_stats_string.c_str());
        log_info(stderr, "  delta : %d\n", filter_delta);
    }

    if (argc == arg + 2)
    {
        const char *aln_file_nameL = argv[arg];
        const char *aln_file_nameR = argv[arg+1];

        if (paired)
        {
            const char *aln_file_nameL = argv[arg];
            const char *aln_file_nameR = argv[arg+1];

            SharedPointer<AlignmentStream> aln_streamL = SharedPointer<AlignmentStream>( open_alignment_file( aln_file_nameL ) );
            SharedPointer<AlignmentStream> aln_streamR = SharedPointer<AlignmentStream>( open_alignment_file( aln_file_nameR ) );

            if (aln_streamL == NULL || aln_streamL->is_ok() == false) { log_error(stderr, "failed opening \"%s\"\n", aln_file_nameL); exit(1); }
            if (aln_streamR == NULL || aln_streamR->is_ok() == false) { log_error(stderr, "failed opening \"%s\"\n", aln_file_nameR); exit(1); }

            const uint32 BATCH_SIZE = 500000;
            std::vector<Alignment> batchL( BATCH_SIZE );
            std::vector<Alignment> batchR( BATCH_SIZE );

            Filter filter( filter_name, filter_flags, filter_stats, filter_delta );
            SharedPointer<PEAnalyzer> analyzer = SharedPointer<PEAnalyzer>( new PEAnalyzer( filter, id_check ) );

            uint32 n_batch = 0;
            while (1)
            {
                const uint32 batch_sizeL = aln_streamL->next_batch( BATCH_SIZE, &batchL[0] );
                const uint32 batch_sizeR = aln_streamR->next_batch( BATCH_SIZE, &batchR[0] );

                const uint32 min_batch_size = nvbio::min( batch_sizeL, batch_sizeR );
                const uint32 max_batch_size = nvbio::max( batch_sizeL, batch_sizeR );

                if (min_batch_size != max_batch_size)
                    log_warning(stderr, "alignment files have different size\n");

                log_info(stderr, "analizing batch[%u]: %u alignments (%.1f M)\n", n_batch, min_batch_size, float(n_batch * BATCH_SIZE + min_batch_size)*1.0e-6f);

                for (uint32 i = 0; i < min_batch_size; i += 2)
                {
                    const Alignment* alnL1 = &batchL[i];
                    const Alignment* alnL2 = &batchL[i+1];
                    const Alignment* alnR1 = &batchR[i];
                    const Alignment* alnR2 = &batchR[i+1];

                    if (alnL1->is_mapped() && alnL1->mate == alnL2->mate)
                    {
                        log_error(stderr, "alignments %u and %u in \"%s\" refer to the same mate, must come from different reads\n", i, i+1, aln_file_nameL);
                        return 1;
                    }
                    if (alnR1->is_mapped() && alnR1->mate == alnR2->mate)
                    {
                        log_error(stderr, "alignments %u and %u in \"%s\" refer to the same mate, must come from different reads\n", i, i+1, aln_file_nameR);
                        return 1;
                    }

                    if (alnL1->mate) std::swap( alnL1, alnL2 );
                    if (alnR1->mate) std::swap( alnR1, alnR2 );

                    analyzer->push(
                        AlignmentPair( *alnL1, *alnL2 ),
                        AlignmentPair( *alnR1, *alnR2 ));
                }

                if (report_name)
                    analyzer->generate_report( aln_file_nameL, aln_file_nameR, report_name );

                analyzer->flush();

                log_verbose(stderr, "  mismatched          : %5.2f%%\n", 100.0f * analyzer->mismatched());
                log_verbose(stderr, "  mapped [L]          : %5.2f%%\n", 100.0f * analyzer->mapped.avg_L());
                log_verbose(stderr, "  mapped [R]          : %5.2f%%\n", 100.0f * analyzer->mapped.avg_R());
                log_verbose(stderr, "  mapped [L&R]        : %5.2f%%\n", 100.0f * analyzer->mapped.avg_L_and_R());
                log_verbose(stderr, "  mapped/unmapped [L] : %5.2f%%\n", 100.0f * analyzer->mapped.avg_L_not_R());
                log_verbose(stderr, "  mapped/unmapped [R] : %5.2f%%\n", 100.0f * analyzer->mapped.avg_R_not_L());
                log_verbose(stderr, "  paired [L]          : %5.2f%%\n", 100.0f * analyzer->paired.avg_L());
                log_verbose(stderr, "  paired [R]          : %5.2f%%\n", 100.0f * analyzer->paired.avg_R());
                log_verbose(stderr, "  paired [L&R]        : %5.2f%%\n", 100.0f * analyzer->paired.avg_L_and_R());
                log_verbose(stderr, "  paired/unpaired [L] : %5.2f%%\n", 100.0f * analyzer->paired.avg_L_not_R());
                log_verbose(stderr, "  paired/unpaired [R] : %5.2f%%\n", 100.0f * analyzer->paired.avg_R_not_L());
                log_verbose(stderr, "  different ref       : %5.2f%%\n", 100.0f * analyzer->different_ref());
                log_verbose(stderr, "  distant             : %5.2f%%\n", 100.0f * analyzer->distant());
                log_verbose(stderr, "  discordant          : %5.2f%%\n", 100.0f * analyzer->discordant());
                log_verbose(stderr, "  filtered            : %u\n", analyzer->filtered());

                if (min_batch_size < BATCH_SIZE)
                    break;

                ++n_batch;
            }
        }
        else
        {
            SharedPointer<AlignmentStream> aln_streamL = SharedPointer<AlignmentStream>( open_alignment_file( aln_file_nameL ) );
            SharedPointer<AlignmentStream> aln_streamR = SharedPointer<AlignmentStream>( open_alignment_file( aln_file_nameR ) );

            if (aln_streamL == NULL || aln_streamL->is_ok() == false) { log_error(stderr, "failed opening \"%s\"\n", aln_file_nameL); exit(1); }
            if (aln_streamR == NULL || aln_streamR->is_ok() == false) { log_error(stderr, "failed opening \"%s\"\n", aln_file_nameR); exit(1); }

            const uint32 BATCH_SIZE = 500000;
            std::vector<Alignment> batchL( BATCH_SIZE );
            std::vector<Alignment> batchR( BATCH_SIZE );

            Filter filter( filter_name, filter_flags, filter_stats, filter_delta );
            SharedPointer<SEAnalyzer> analyzer = SharedPointer<SEAnalyzer>( new SEAnalyzer( filter ) );

            uint32 n_batch = 0;
            while (1)
            {
                const uint32 batch_sizeL = aln_streamL->next_batch( BATCH_SIZE, &batchL[0] );
                const uint32 batch_sizeR = aln_streamR->next_batch( BATCH_SIZE, &batchR[0] );

                if (batch_sizeL != batch_sizeR)
                    log_warning(stderr, "alignment files have different size\n");

                const uint32 min_batch_size = nvbio::min( batch_sizeL, batch_sizeR );

                log_info(stderr, "analizing batch[%u]: %u alignments (%.1f M)\n", n_batch, min_batch_size, float(n_batch * BATCH_SIZE + min_batch_size)*1.0e-6f);

                for (uint32 i = 0; i < min_batch_size; ++i)
                    analyzer->push( batchL[i], batchR[i] );

                if (report_name)
                    analyzer->generate_report( aln_file_nameL, aln_file_nameR, report_name );

                analyzer->flush();

                log_verbose(stderr, "  mismatched          : %5.2f%%\n", 100.0f * analyzer->mismatched());
                log_verbose(stderr, "  mapped [L]          : %5.2f%%\n", 100.0f * analyzer->mapped.avg_L());
                log_verbose(stderr, "  mapped [R]          : %5.2f%%\n", 100.0f * analyzer->mapped.avg_R());
                log_verbose(stderr, "  mapped [L&R]        : %5.2f%%\n", 100.0f * analyzer->mapped.avg_L_and_R());
                log_verbose(stderr, "  mapped/unmapped [L] : %5.2f%%\n", 100.0f * analyzer->mapped.avg_L_not_R());
                log_verbose(stderr, "  mapped/unmapped [R] : %5.2f%%\n", 100.0f * analyzer->mapped.avg_R_not_L());
                log_verbose(stderr, "  different ref       : %5.2f%%\n", 100.0f * analyzer->different_ref());
                log_verbose(stderr, "  distant             : %5.2f%%\n", 100.0f * analyzer->distant());
                log_verbose(stderr, "  discordant          : %5.2f%%\n", 100.0f * analyzer->discordant());
                log_verbose(stderr, "  filtered            : %u\n", analyzer->filtered());

                if (min_batch_size < BATCH_SIZE)
                    break;

                ++n_batch;
            }
        }
    }
    else if (argc == arg + 4)
    {
        const char *aln_file_nameL1 = argv[arg];
        const char *aln_file_nameL2 = argv[arg+1];
        const char *aln_file_nameR1 = argv[arg+2];
        const char *aln_file_nameR2 = argv[arg+3];

        const char* aln_file_nameL = aln_file_nameL1;
        const char* aln_file_nameR = aln_file_nameR1;

        SharedPointer<AlignmentStream> aln_streamL1 = SharedPointer<AlignmentStream>( open_alignment_file( aln_file_nameL1 ) );
        SharedPointer<AlignmentStream> aln_streamL2 = SharedPointer<AlignmentStream>( open_alignment_file( aln_file_nameL2 ) );
        SharedPointer<AlignmentStream> aln_streamR1 = SharedPointer<AlignmentStream>( open_alignment_file( aln_file_nameR1 ) );
        SharedPointer<AlignmentStream> aln_streamR2 = SharedPointer<AlignmentStream>( open_alignment_file( aln_file_nameR2 ) );

        if (aln_streamL1 == NULL || aln_streamL1->is_ok() == false) { log_error(stderr, "failed opening \"%s\"\n", aln_file_nameL1); exit(1); }
        if (aln_streamL2 == NULL || aln_streamL2->is_ok() == false) { log_error(stderr, "failed opening \"%s\"\n", aln_file_nameL2); exit(1); }

        if (aln_streamR2 == NULL || aln_streamR1->is_ok() == false) { log_error(stderr, "failed opening \"%s\"\n", aln_file_nameR1); exit(1); }
        if (aln_streamR2 == NULL || aln_streamR2->is_ok() == false) { log_error(stderr, "failed opening \"%s\"\n", aln_file_nameR2); exit(1); }

        const uint32 BATCH_SIZE = 500000;
        std::vector<Alignment> batchL1( BATCH_SIZE );
        std::vector<Alignment> batchL2( BATCH_SIZE );
        std::vector<Alignment> batchR1( BATCH_SIZE );
        std::vector<Alignment> batchR2( BATCH_SIZE );

        Filter filter( filter_name, filter_flags, filter_stats, filter_delta );
        SharedPointer<PEAnalyzer> analyzer = SharedPointer<PEAnalyzer>( new PEAnalyzer( filter, id_check ) );

        uint32 n_batch = 0;
        while (1)
        {
            const uint32 batch_sizeL1 = aln_streamL1->next_batch( BATCH_SIZE, &batchL1[0] );
            const uint32 batch_sizeL2 = aln_streamL2->next_batch( BATCH_SIZE, &batchL2[0] );
            const uint32 batch_sizeR1 = aln_streamR1->next_batch( BATCH_SIZE, &batchR1[0] );
            const uint32 batch_sizeR2 = aln_streamR2->next_batch( BATCH_SIZE, &batchR2[0] );

            const uint32 min_batch_size = nvbio::min(
                nvbio::min( batch_sizeL1, batch_sizeL2 ),
                nvbio::min( batch_sizeR1, batch_sizeR2 ) );

            const uint32 max_batch_size = nvbio::min(
                nvbio::max( batch_sizeL1, batch_sizeL2 ),
                nvbio::max( batch_sizeR1, batch_sizeR2 ) );

            if (min_batch_size != max_batch_size)
                log_warning(stderr, "alignment files have different size\n");

            log_info(stderr, "analizing batch[%u]: %u alignments (%.1f M)\n", n_batch, min_batch_size, float(n_batch * BATCH_SIZE + min_batch_size)*1.0e-6f);

            for (uint32 i = 0; i < min_batch_size; ++i)
                analyzer->push(
                    AlignmentPair( batchL1[i], batchL2[i] ),
                    AlignmentPair( batchR1[i], batchR2[i] ));

            if (report_name)
                analyzer->generate_report( aln_file_nameL, aln_file_nameR, report_name );

            analyzer->flush();

            log_verbose(stderr, "  mismatched          : %5.2f%%\n", 100.0f * analyzer->mismatched());
            log_verbose(stderr, "  mapped [L]          : %5.2f%%\n", 100.0f * analyzer->mapped.avg_L());
            log_verbose(stderr, "  mapped [R]          : %5.2f%%\n", 100.0f * analyzer->mapped.avg_R());
            log_verbose(stderr, "  mapped [L&R]        : %5.2f%%\n", 100.0f * analyzer->mapped.avg_L_and_R());
            log_verbose(stderr, "  mapped/unmapped [L] : %5.2f%%\n", 100.0f * analyzer->mapped.avg_L_not_R());
            log_verbose(stderr, "  mapped/unmapped [R] : %5.2f%%\n", 100.0f * analyzer->mapped.avg_R_not_L());
            log_verbose(stderr, "  paired [L]          : %5.2f%%\n", 100.0f * analyzer->paired.avg_L());
            log_verbose(stderr, "  paired [R]          : %5.2f%%\n", 100.0f * analyzer->paired.avg_R());
            log_verbose(stderr, "  paired [L&R]        : %5.2f%%\n", 100.0f * analyzer->paired.avg_L_and_R());
            log_verbose(stderr, "  paired/unpaired [L] : %5.2f%%\n", 100.0f * analyzer->paired.avg_L_not_R());
            log_verbose(stderr, "  paired/unpaired [R] : %5.2f%%\n", 100.0f * analyzer->paired.avg_R_not_L());
            log_verbose(stderr, "  different ref       : %5.2f%%\n", 100.0f * analyzer->different_ref());
            log_verbose(stderr, "  distant             : %5.2f%%\n", 100.0f * analyzer->distant());
            log_verbose(stderr, "  discordant          : %5.2f%%\n", 100.0f * analyzer->discordant());
            log_verbose(stderr, "  filtered            : %u\n", analyzer->filtered());

            if (min_batch_size < BATCH_SIZE)
                break;

            ++n_batch;
        }
    }

    cudaDeviceReset();
	return 0;
}

