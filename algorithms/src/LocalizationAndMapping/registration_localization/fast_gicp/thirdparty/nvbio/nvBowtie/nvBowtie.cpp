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

// nvBowtie.cpp : Defines the entry point for the console application.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <nvbio/basic/console.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/shared_pointer.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/io/fmindex/fmindex.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/io/sequence/sequence_mmap.h>
#include <nvbio/io/output/output_file.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvBowtie/bowtie2/cuda/stats.h>
#include <nvBowtie/bowtie2/cuda/input_thread.h>
#include <nvBowtie/bowtie2/cuda/compute_thread.h>
#include <nvbio/basic/omp.h>

void crcInit();

namespace nvbio {
namespace bowtie2 {
namespace cuda {

    void test_seed_hit_deques();
    void test_scoring_queues();

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio

using namespace nvbio;

// bogus implementation of a function to check if a string is a number
bool is_number(const char* str, uint32 len = uint32(-1))
{
    if (str[0] == '-')
        ++str;

    for (uint32 l = 0; *str != '\0' && l < len; ++l)
    {
        const char c = *str; ++str;
        if (c == '.')             continue;
        if (c >= '0' && c <= '9') continue;
        return false;
    }
    return true;
}

void log_ed(nvbio::bowtie2::cuda::AlignmentStats& stats)
{
    if (stats.n_mapped == 0)
        return;

    const std::vector<uint32>& mapped = stats.mapped_log_ed_histogram;
    log_stats(stderr, "    ed   :");
    for (uint32 i = 0; i < mapped.size(); ++i)
    {
        if (float(mapped[i])/float(stats.n_mapped) > 1.0e-3f)
            log_stats_cont(stderr, " %5u  ", i ? 1u << (i-1) : 0u );
    }
    log_stats_cont(stderr,"\n");
    log_stats(stderr, "         :");
    for (uint32 i = 0; i < mapped.size(); ++i)
    {
        if (float(mapped[i])/float(stats.n_mapped) > 1.0e-3f)
            log_stats_cont(stderr, " %6.2f%%", 100.0f * float(mapped[i])/float(stats.n_mapped) );
    }
    log_stats_cont(stderr,"\n");
}
void log_mapq(nvbio::bowtie2::cuda::AlignmentStats& stats)
{
    if (stats.n_mapped == 0)
        return;

    log_stats(stderr, "    mapq :");
    for (uint32 i = 0; i < 8; ++i)
        log_stats_cont(stderr, " %5u  ", i ? 1u << (i-1) : 0u );
    log_stats_cont(stderr,"\n");
    log_stats(stderr, "         :");
    for (uint32 i = 0; i < 8; ++i)
        log_stats_cont(stderr, " %6.2f%%", 100.0f * float(stats.mapq_log_bins[i])/float(stats.n_mapped) );
    log_stats_cont(stderr,"\n");
}

int main(int argc, char* argv[])
{
    //cudaSetDeviceFlags( cudaDeviceMapHost | cudaDeviceLmemResizeToMax );

    crcInit();

    if (argc == 1 ||
        (argc == 2 && strcmp( argv[1], "--help" ) == 0) ||
        (argc == 2 && strcmp( argv[1], "-h" ) == 0))
    {
        log_info(stderr,"nvBowtie [options]\n");
        log_info(stderr,"options:\n");
        log_info(stderr,"  General:\n");
        log_info(stderr,"    -U                  file-name      unpaired reads\n");
        log_info(stderr,"    -1                  file-name      first mate reads\n");
        log_info(stderr,"    -2                  file-name      second mate reads\n");
        log_info(stderr,"    -S                  file-name      output file (.sam|.bam)\n");
        log_info(stderr,"    -x                  file-name      reference index\n");
        log_info(stderr,"    --verbosity         int [5]        verbosity level\n");
        log_info(stderr,"    --upto       | -u   int [-1]       maximum number of reads to process\n");
        log_info(stderr,"    --trim3      | -3   int [0]        trim the first N bases of 3'\n");
        log_info(stderr,"    --trim5      | -5   int [0]        trim the first N bases of 5'\n");
        log_info(stderr,"    --nofw                             do not align the forward strand\n");
        log_info(stderr,"    --norc                             do not align the reverse-complemented strand\n");
        log_info(stderr,"    --device            int [0]        select the given cuda device(s) (e.g. --device 0 --device 1 ...)\n");
        log_info(stderr,"    --file-ref                         load reference from file\n");
        log_info(stderr,"    --server-ref                       load reference from server\n");
        log_info(stderr,"    --phred33                          qualities are ASCII characters equal to Phred quality + 33\n");
        log_info(stderr,"    --phred64                          qualities are ASCII characters equal to Phred quality + 64\n");
        log_info(stderr,"    --solexa-quals                     qualities are in the Solexa format\n");
        log_info(stderr,"    --rg-id             string         add the RG-ID field of the SAM output header\n");
        log_info(stderr,"    --rg                string,val     add an RG-TAG field of the SAM output header\n");
        log_info(stderr,"  Paired-End:\n");
        log_info(stderr,"    --ff                               paired mates are forward-forward\n");
        log_info(stderr,"    --fr                               paired mates are forward-reverse\n");
        log_info(stderr,"    --rf                               paired mates are reverse-forward\n");
        log_info(stderr,"    --rr                               paired mates are reverse-reverse\n");
        log_info(stderr,"    --minins            int [0]        minimum insert length\n");
        log_info(stderr,"    --minins            int [500]      maximum insert length\n");
        log_info(stderr,"    --overlap                          allow overlapping mates\n");
        log_info(stderr,"    --dovetail                         allow dovetailing mates\n");
        log_info(stderr,"    --no-mixed                         only report paired alignments\n");
        log_info(stderr,"    --ungapped-mates | -ug             perform ungapped mate alignment\n");
        log_info(stderr,"  Seeding:\n");
        log_info(stderr,"    --seed-len   | -L   int   [22]     seed lengths\n");
        log_info(stderr,"    --seed-freq  | -i   {G|L|S},x,y    seed interval, as x + y*func(read-len) (G=log,L=linear,S=sqrt)\n");
        log_info(stderr,"    --max-hits          int   [100]    maximum amount of seed hits\n");
        log_info(stderr,"    --max-reseed | -R   int   [2]      number of reseeding rounds\n");
        log_info(stderr,"  Extension:\n");
        log_info(stderr,"    --all        | -a                  perform all-mapping (i.e. find and report all alignments)\n");
        log_info(stderr,"    --local                            perform local alignment\n");
        log_info(stderr,"    --rand                             randomized seed selection\n");
        log_info(stderr,"    --no-rand                          do not randomize seed hit selection\n");
        log_info(stderr,"    --max-dist          int [15]       maximum edit distance\n");
        log_info(stderr,"    --max-effort-init   int [15]       initial maximum number of consecutive extension failures\n");
        log_info(stderr,"    --max-effort | -D   int [15]       maximum number of consecutive extension failures\n");
        log_info(stderr,"    --min-ext           int [30]       minimum number of extensions per read\n");
        log_info(stderr,"    --max-ext           int [400]      maximum number of extensions per read\n");
        log_info(stderr,"    --fast                             apply the fast presets\n");
        log_info(stderr,"    --very-fast                        apply the very-fast presets\n");
        log_info(stderr,"    --sensitive                        apply the sensitive presets\n");
        log_info(stderr,"    --very-sensitive                   apply the very-sensitive presets\n");
        log_info(stderr,"    --fast-local                       apply the fast presets\n");
        log_info(stderr,"    --very-fast-local                  apply the very-fast presets\n");
        log_info(stderr,"    --sensitive-local                  apply the sensitive presets\n");
        log_info(stderr,"    --very-sensitive-local             apply the very-sensitive presets\n");
        log_info(stderr,"  Scoring:\n");
        log_info(stderr,"    --scoring           {sw|ed}        Smith-Waterman / Edit-Distance scoring\n");
        log_info(stderr,"    --score-min         {G|L|S},x,y    minimum score function, as x + y*func(read-len)\n");
        log_info(stderr,"    --ma                int            match bonus\n");
        log_info(stderr,"    --mp                int,int        mismatch min/max penalties\n");
        log_info(stderr,"    --np                int            N penalty\n");
        log_info(stderr,"    --rdg               int,int        read open/extension gap penalties\n");
        log_info(stderr,"    --rfg               int,int        reference open/extension gap penalties\n");
        log_info(stderr,"  Reporting:\n");
        log_info(stderr,"    --mapQ-filter | -Q  int [0]        minimum mapQ threshold\n");
        exit(0);
    }
    else if (argc == 2 && strcmp( argv[1], "-test" ) == 0)
    {
        log_visible(stderr, "nvBowtie tests... started\n");
        nvbio::bowtie2::cuda::test_seed_hit_deques();
        nvbio::bowtie2::cuda::test_scoring_queues();
        log_visible(stderr, "nvBowtie tests... done\n");
        exit(0);
    }
    else if (argc == 2 && strcmp( argv[1], "--version" ) == 0)
    {
        fprintf(stderr, "nvBowtie version %s\n", NVBIO_VERSION_STRING);
        exit(0);
    }

    // setup the number of OMP threads
    omp_set_num_threads( omp_get_num_procs() );

    uint32 max_reads    = uint32(-1);
    uint32 max_read_len = uint32(-1);
    uint32 trim3        = 0;
    uint32 trim5        = 0;
    //bool   debug        = false;
    bool   from_file    = false;
    bool   paired_end   = false;
    io::PairedEndPolicy pe_policy = io::PE_POLICY_FR;
    io::QualityEncoding qencoding = io::Phred33;
    std::vector<int> cuda_devices;

    std::map<std::string,std::string> string_options;

    std::string argstr;
    for (int32 i = 1; i < argc; ++i)
    {
        argstr += " ";
        argstr += argv[i];
    }

    std::string rg_id;
    std::string rg_string;

    bool legacy_cmdline = true;

    const char* read_name1      = "";
    const char* read_name2      = "";
    const char* reference_name  = "";
    const char* output_name     = "";

    for (int32 i = 1; i < argc; ++i)
    {
        if (strcmp( argv[i], "--pe" ) == 0 ||
            strcmp( argv[i], "-paired-ends" ) == 0 ||
            strcmp( argv[i], "--paired-ends" ) == 0)
            paired_end = true;
        else if (strcmp( argv[i], "--ff" ) == 0)
            pe_policy = io::PE_POLICY_FF;
        else if (strcmp( argv[i], "--fr" ) == 0)
            pe_policy = io::PE_POLICY_FR;
        else if (strcmp( argv[i], "--rf" ) == 0)
            pe_policy = io::PE_POLICY_RF;
        else if (strcmp( argv[i], "--rr" ) == 0)
            pe_policy = io::PE_POLICY_RR;
        else if (strcmp( argv[i], "-max-reads" )  == 0 ||
                 strcmp( argv[i], "--max-reads" ) == 0 ||
                 strcmp( argv[i], "-u" )          == 0 ||
                 strcmp( argv[i], "--upto" )      == 0)
            max_reads = (uint32)atoi( argv[++i] );
        else if (strcmp( argv[i], "-max-read-len" )  == 0 ||
                 strcmp( argv[i], "--max-read-len" ) == 0)
            max_read_len = (uint32)atoi( argv[++i] );
        else if (strcmp( argv[i], "-3") == 0 ||
                 strcmp( argv[i], "--trim3") == 0)
            trim3 = (uint32)atoi( argv[++i] );
        else if (strcmp( argv[i], "-5") == 0 ||
                 strcmp( argv[i], "--trim5") == 0)
            trim5 = (uint32)atoi( argv[++i] );
        else if (strcmp( argv[i], "-file-ref" )  == 0 ||
                 strcmp( argv[i], "--file-ref" ) == 0)
            from_file = true;
        else if (strcmp( argv[i], "-server-ref" )  == 0 ||
                 strcmp( argv[i], "--server-ref" ) == 0)
            from_file = false;
        else if (strcmp( argv[i], "-input" )  == 0 ||
                 strcmp( argv[i], "--input" ) == 0)
        {
            if (strcmp( argv[i+1], "file" ) == 0)
                from_file = true;
            else if (strcmp( argv[i+1], "server" ) == 0)
                from_file = false;
            else
                log_warning(stderr, "unknown \"%s\" input, skipping\n", argv[i+1]);

            ++i;
        }
        else if (strcmp( argv[i], "-phred33" ) == 0 ||
                 strcmp( argv[i], "--phred33" ) == 0)
            qencoding = io::Phred33;
        else if (strcmp( argv[i], "-phred64" ) == 0 ||
                 strcmp( argv[i], "--phred64" ) == 0)
            qencoding = io::Phred64;
        else if (strcmp( argv[i], "-solexa-quals" ) == 0 ||
                 strcmp( argv[i], "--solexa-quals" ) == 0)
            qencoding = io::Solexa;
        else if (strcmp( argv[i], "-device" ) == 0 ||
                 strcmp( argv[i], "--device" ) == 0)
            cuda_devices.push_back( atoi( argv[++i] ) );
        else if (strcmp( argv[i], "-verbosity" ) == 0 ||
                 strcmp( argv[i], "--verbosity" ) == 0)
            set_verbosity( Verbosity( atoi( argv[++i] ) ) );
        else if (strcmp( argv[i], "-rg-id" )  == 0 ||
                 strcmp( argv[i], "--rg-id" ) == 0)
            rg_id = argv[++i];
        else if (strcmp( argv[i], "-rg" )  == 0 ||
                 strcmp( argv[i], "--rg" ) == 0)
        {
            rg_string += "\t";
            rg_string += argv[++i];
        }
        else if (strcmp( argv[i], "-1") == 0)
        {
            legacy_cmdline = false;
            paired_end     = true;
            read_name1     = argv[++i];
        }
        else if (strcmp( argv[i], "-2") == 0)
        {
            legacy_cmdline = false;
            paired_end     = true;
            read_name2     = argv[++i];
        }
        else if (strcmp( argv[i], "-U") == 0)
        {
            legacy_cmdline = false;
            paired_end     = false;
            read_name1     = argv[++i];
        }
        else if (strcmp( argv[i], "-S") == 0)
        {
            legacy_cmdline = false;
            output_name    = argv[++i];
        }
        else if (strcmp( argv[i], "-x") == 0)
        {
            legacy_cmdline = false;
            reference_name = argv[++i];
        }
        else if (argv[i][0] == '-')
        {
            // add unknown option to the string options
            const std::string key = std::string( argv[i][1] == '-' ? argv[i] + 2 : argv[i] + 1 );
            const char* next = argv[i+1];

            if (is_number(next) || next[0] != '-')
            {
                const std::string val = std::string( next ); ++i;
                string_options.insert( std::make_pair( key, val ) );
            }
            else
                string_options.insert( std::make_pair( key, "1" ) );
        }
    }

    if (strlen( read_name1 ) == 0 &&
        strlen( read_name2 ) == 0)
    {
        log_error(stderr, "must specify at least one read input with -U/-1/-2\n");
        return 1;
    }

    log_info(stderr, "nvBowtie... started\n");
    log_debug(stderr, "  %-16s : %d\n", "max-reads",  max_reads);
    log_debug(stderr, "  %-16s : %d\n", "max-length", max_read_len);
    log_debug(stderr, "  %-16s : %s\n", "quals", qencoding == io::Phred33 ? "phred33" :
                                                 qencoding == io::Phred64 ? "phred64" :
                                                                            "solexa");
    if (paired_end)
    {
        log_debug(stderr, "  %-16s : %s\n", "pe-policy",
            pe_policy == io::PE_POLICY_FF ? "ff" :
            pe_policy == io::PE_POLICY_FR ? "fr" :
            pe_policy == io::PE_POLICY_RF ? "rf" :
                                            "rr" );
    }
    if (string_options.empty() == false)
    {
        for (std::map<std::string,std::string>::const_iterator it = string_options.begin(); it != string_options.end(); ++it)
            log_debug(stderr, "  %-16s : %s\n", it->first.c_str(), it->second.c_str());
    }
    log_debug(stderr, "\n");

    try
    {
        int device_count;
        cudaGetDeviceCount(&device_count);
        cuda::check_error("cuda-check");

        log_verbose(stderr, "  cuda devices : %d\n", device_count);

        // inspect and select cuda devices
        if (device_count > 0)
        {
            if (cuda_devices.empty())
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
                cuda_devices.push_back( best_device );
            }

            for (size_t i = 0; i < cuda_devices.size(); ++i)
            {
                cudaDeviceProp device_prop;
                cudaGetDeviceProperties( &device_prop, cuda_devices[i] );
                log_verbose(stderr, "  chosen device %d\n", cuda_devices[i]);
                log_verbose(stderr, "    device name        : %s\n", device_prop.name);
                log_verbose(stderr, "    compute capability : %d.%d\n", device_prop.major, device_prop.minor);
            }
        }
        else
        {
            log_error(stderr, "no available CUDA devices\n");
            exit(1);
        }

        if (legacy_cmdline)
        {
            const uint32 arg_offset = paired_end ? argc-4 : argc-3;
            reference_name = argv[arg_offset];
            if (paired_end)
            {
                read_name1 = argv[arg_offset+1];
                read_name2 = argv[arg_offset+2];
            }
            else
                read_name1 = argv[arg_offset+1];

            output_name = argv[argc-1];
        }

        //
        // Parse program options
        //

        // WARNING: we don't do any error checking on passed parameters!
        bowtie2::cuda::Params params;
        params.pe_policy = pe_policy;
        {
            bool init = true;
            std::string config = string_option( string_options, "config", "" );
            if (config != "") { bowtie2::cuda::parse_options( params, bowtie2::cuda::load_options( config.c_str() ), init ); init = false; }
                                bowtie2::cuda::parse_options( params, string_options,                                init );

        }
        if (params.alignment_type == bowtie2::cuda::LocalAlignment &&
            params.scoring_mode   == bowtie2::cuda::EditDistanceMode)
        {
            log_warning(stderr, "edit-distance scoring is incompatible with local alignment, switching to Smith-Waterman\n");
            params.scoring_mode = bowtie2::cuda::SmithWatermanMode;
        }

        //
        // Load the reference
        //

        SharedPointer<nvbio::io::SequenceData> reference_data;
        SharedPointer<nvbio::io::FMIndexData>  driver_data;
        if (from_file)
        {
            log_visible(stderr, "loading reference index... started\n");
            log_info(stderr, "  file: \"%s\"\n", reference_name);

            // load the reference data
            reference_data = io::load_sequence_file( DNA, reference_name );
            if (reference_data == NULL)
            {
                log_error(stderr, "unable to load reference index \"%s\"\n", reference_name);
                return 1;
            }

            log_visible(stderr, "loading reference index... done\n");

            nvbio::io::FMIndexDataHost* loader = new nvbio::io::FMIndexDataHost;
            if (!loader->load( reference_name ))
            {
                log_error(stderr, "unable to load reference index \"%s\"\n", reference_name);
                return 1;
            }

            driver_data = loader;
        }
        else
        {
            log_visible(stderr, "mapping reference index... started\n");
            log_info(stderr, "  file: \"%s\"\n", reference_name);

            // map the reference data
            reference_data = io::map_sequence_file( reference_name );
            if (reference_data == NULL)
            {
                log_visible(stderr, "mapping reference index... failed\n");
                log_visible(stderr, "loading reference index... started\n");
                log_info(stderr, "  file: \"%s\"\n", reference_name);

                // load the reference data
                reference_data = io::load_sequence_file( DNA, reference_name );
                if (reference_data == NULL)
                {
                    log_error(stderr, "unable to load reference index \"%s\"\n", reference_name);
                    return 1;
                }

                log_visible(stderr, "loading reference index... done\n");

                nvbio::io::FMIndexDataHost* loader = new nvbio::io::FMIndexDataHost;
                if (!loader->load( reference_name ))
                {
                    log_error(stderr, "unable to load reference index \"%s\"\n", reference_name);
                    return 1;
                }

                driver_data = loader;
            }
            else
            {
                log_visible(stderr, "mapping reference index... done\n");

                nvbio::io::FMIndexDataMMAP* loader = new nvbio::io::FMIndexDataMMAP;
                if (!loader->load( reference_name ))
                {
                    log_error(stderr, "unable to load reference index \"%s\"\n", reference_name);
                    return 1;
                }

                driver_data = loader;
            }
        }

        //
        // Setup the output file
        //

        SharedPointer<io::OutputFile> output_file( io::OutputFile::open(
                                                    output_name,
                                                    paired_end ? io::PAIRED_END : io::SINGLE_END,
                                                    io::BNT(*reference_data) ) );

        output_file->set_rg( rg_id.c_str(), rg_string.c_str() );
        output_file->set_program(
            "nvBowtie",
            "nvBowtie",
            NVBIO_VERSION_STRING,
            argstr.c_str() );

        output_file->configure_mapq_evaluator(params.mapq_filter);
        output_file->header();

        if (paired_end)
        {
            //
            // Open the input read files
            //

            log_visible(stderr, "opening read file [1] \"%s\"\n", read_name1);
            SharedPointer<nvbio::io::SequenceDataStream> read_data_file1(
                nvbio::io::open_sequence_file(
                    strcmp( read_name1, "-" ) == 0 ? "" : read_name1,
                    qencoding,
                    max_reads,
                    max_read_len,
                    io::REVERSE,
                    trim3,
                    trim5 )
            );

            if (read_data_file1 == NULL || read_data_file1->is_ok() == false)
            {
                log_error(stderr, "unable to open read file \"%s\"\n", read_name1);
                return 1;
            }

            log_visible(stderr, "opening read file [2] \"%s\"\n", read_name2);
            SharedPointer<nvbio::io::SequenceDataStream> read_data_file2(
                nvbio::io::open_sequence_file(
                    strcmp( read_name2, "-" ) == 0 ? "" : read_name2,
                    qencoding,
                    max_reads,
                    max_read_len,
                    io::REVERSE,
                    trim3,
                    trim5 )
            );

            if (read_data_file2 == NULL || read_data_file2->is_ok() == false)
            {
                log_error(stderr, "unable to open read file \"%s\"\n", read_name2);
                return 1;
            }


            // print the command line options
            {
                const bowtie2::cuda::SimpleFunc& score_min = bowtie2::cuda::EditDistanceMode ?
                    params.scoring_scheme.ed.m_score_min :
                    params.scoring_scheme.sw.m_score_min;

                log_verbose(stderr, "  mode           = %s\n", bowtie2::cuda::mapping_mode( params.mode ));
                log_verbose(stderr, "  scoring        = %s\n", bowtie2::cuda::scoring_mode( params.scoring_mode ));
                log_verbose(stderr, "  score-min      = %s:%.2f:%.2f\n", score_min.type_string(), score_min.k, score_min.m);
                log_verbose(stderr, "  alignment type = %s\n", params.alignment_type == bowtie2::cuda::LocalAlignment ? "local" : "end-to-end");
                log_verbose(stderr, "  pe-policy      = %s\n",
                                                               pe_policy == io::PE_POLICY_FF ? "ff" :
                                                               pe_policy == io::PE_POLICY_FR ? "fr" :
                                                               pe_policy == io::PE_POLICY_RF ? "rf" :
                                                                                               "rr" );
                log_verbose(stderr, "  seed length    = %u\n", params.seed_len);
                log_verbose(stderr, "  seed interval  = (%s, %.3f, %.3f)\n", params.seed_freq.type_symbol(), params.seed_freq.k, params.seed_freq.m);
                log_verbose(stderr, "  seed rounds    = %u\n", params.max_reseed);
                log_verbose(stderr, "  max hits       = %u\n", params.max_hits);
                log_verbose(stderr, "  max edit dist  = %u\n", params.max_dist);
                log_verbose(stderr, "  max effort     = %u\n", params.max_effort);
                log_verbose(stderr, "  substitutions  = %u\n", params.allow_sub);
                log_verbose(stderr, "  mapq filter    = %u\n", params.mapq_filter);
                log_verbose(stderr, "  randomized     = %s\n", params.randomized ? "yes" : "no");
                if (params.allow_sub)
                    log_verbose(stderr, "  subseed length = %u\n", params.subseed_len);
            }

            //
            // Setup the compute threads
            //

            uint32 batch_size = uint32(-1);

            std::vector< bowtie2::cuda::Stats > device_stats( cuda_devices.size() );
            std::vector< SharedPointer<bowtie2::cuda::ComputeThreadPE> > compute_threads( cuda_devices.size() );

            for (uint32 i = 0; i < cuda_devices.size(); ++i)
            {
                device_stats[i] = bowtie2::cuda::Stats( params );

                compute_threads[i] = SharedPointer<bowtie2::cuda::ComputeThreadPE>(
                    new bowtie2::cuda::ComputeThreadPE(
                        i,
                        cuda_devices[i],
                        *reference_data,
                        *driver_data,
                        string_options,
                        params,
                        device_stats[i] ) );

                batch_size = nvbio::min( batch_size, compute_threads[i]->gauge_batch_size() );
            }

            //
            // Setup the input thread
            //

            Timer timer;
            timer.start();

            bowtie2::cuda::Stats input_stats( params );

            bowtie2::cuda::InputThreadPE input_thread( read_data_file1.get(),  read_data_file2.get(), input_stats, batch_size, params.avg_read_length );
            input_thread.create();

            for (uint32 i = 0; i < cuda_devices.size(); ++i)
            {
                compute_threads[i]->set_input( &input_thread );
                compute_threads[i]->set_output( output_file.get() );
                compute_threads[i]->create();
            }

            uint32                        n_reads = 0;
            bowtie2::cuda::AlignmentStats concordant;
            bowtie2::cuda::AlignmentStats discordant;
            bowtie2::cuda::AlignmentStats mate1;
            bowtie2::cuda::AlignmentStats mate2;

            for (uint32 i = 0; i < cuda_devices.size(); ++i)
            {
                compute_threads[i]->join();

                n_reads += device_stats[i].n_reads;
                concordant.merge( device_stats[i].concordant );
                discordant.merge( device_stats[i].discordant );
                mate1.merge( device_stats[i].mate1 );
                mate2.merge( device_stats[i].mate2 );
            }

            input_thread.join();

            timer.stop();

            log_verbose( stderr, "  compute threads joined\n" );

            // close the output file
            output_file->close();

            //
            // Print statistics
            //

            // transfer I/O statistics
            io::IOStats iostats = output_file->get_aggregate_statistics();
            const bowtie2::cuda::KernelStats& io = iostats.output_process_timings;

            log_stats(stderr, "  total         : %.2f sec (avg: %.3fK reads/s).\n", timer.seconds(), 1.0e-3f * float(n_reads) / timer.seconds());
            log_stats(stderr, "  reads   I/O   : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", input_stats.read_io.time, 1.0e-6f * input_stats.read_io.avg_speed(), 1.0e-6f * input_stats.read_io.max_speed);
            log_stats(stderr, "  results I/O   : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", io.time, 1.0e-6f * io.avg_speed(), 1.0e-6f * io.max_speed);

            uint32&              n_mapped       = concordant.n_mapped;
            uint32&              n_unique       = concordant.n_unique;
            uint32&              n_ambiguous    = concordant.n_ambiguous;
            uint32&              n_nonambiguous = concordant.n_unambiguous;
            uint32&              n_multiple     = concordant.n_multiple;
            {
                log_stats(stderr, "  concordant reads : %.2f %% - of these:\n", 100.0f * float(n_mapped)/float(n_reads) );
                log_stats(stderr, "    aligned uniquely      : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(n_unique)/float(n_mapped), 100.0f * float(n_mapped - n_multiple)/float(n_reads) );
                log_stats(stderr, "    aligned unambiguously : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(n_nonambiguous)/float(n_mapped), 100.0f * float(n_nonambiguous)/float(n_reads) );
                log_stats(stderr, "    aligned ambiguously   : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(n_ambiguous)/float(n_mapped), 100.0f * float(n_ambiguous)/float(n_reads) );
                log_stats(stderr, "    aligned multiply      : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(n_multiple)/float(n_mapped), 100.0f * float(n_multiple)/float(n_reads) );
                log_ed( concordant );
                log_mapq( concordant );

                log_stats(stderr, "  discordant reads : %.2f %%\n", 100.0f * float(discordant.n_mapped)/float(n_reads) );
                log_ed( discordant );
                log_mapq( discordant );

                log_stats(stderr, "  mate1 : %.2f %% - of these:\n", 100.0f * float(mate1.n_mapped)/float(n_reads) );
                if (mate1.n_mapped)
                {
                    log_stats(stderr, "    aligned uniquely      : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(mate1.n_unique)/float(mate1.n_mapped), 100.0f * float(mate1.n_mapped - mate1.n_multiple)/float(n_reads) );
                    log_stats(stderr, "    aligned unambiguously : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(mate1.n_unambiguous)/float(mate1.n_mapped), 100.0f * float(mate1.n_unambiguous)/float(n_reads) );
                    log_stats(stderr, "    aligned ambiguously   : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(mate1.n_ambiguous)/float(mate1.n_mapped), 100.0f * float(mate1.n_ambiguous)/float(n_reads) );
                    log_stats(stderr, "    aligned multiply      : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(mate1.n_multiple)/float(mate1.n_mapped), 100.0f * float(mate1.n_multiple)/float(n_reads) );
                }

                log_stats(stderr, "  mate2 : %.2f %% - of these:\n", 100.0f * float(mate2.n_mapped)/float(n_reads) );
                if (mate2.n_mapped)
                {
                    log_stats(stderr, "    aligned uniquely      : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(mate2.n_unique)/float(mate2.n_mapped), 100.0f * float(mate2.n_mapped - mate2.n_multiple)/float(n_reads) );
                    log_stats(stderr, "    aligned unambiguously : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(mate2.n_unambiguous)/float(mate2.n_mapped), 100.0f * float(mate2.n_unambiguous)/float(n_reads) );
                    log_stats(stderr, "    aligned ambiguously   : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(mate2.n_ambiguous)/float(mate2.n_mapped), 100.0f * float(mate2.n_ambiguous)/float(n_reads) );
                    log_stats(stderr, "    aligned multiply      : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(mate2.n_multiple)/float(mate2.n_mapped), 100.0f * float(mate2.n_multiple)/float(n_reads) );
                }
            }

            // generate an html report
            if (params.report.length())
                bowtie2::cuda::generate_report_header( n_reads, params, concordant, (uint32)cuda_devices.size(), &device_stats[0], params.report.c_str() );
        }
        else
        {
            //
            // Open the input read file
            //

            log_visible(stderr, "opening read file \"%s\"\n", read_name1);
            SharedPointer<io::SequenceDataStream> read_data_file(
                io::open_sequence_file(
                    strcmp( read_name1, "-" ) == 0 ? "" : read_name1,
                    qencoding,
                    max_reads,
                    max_read_len,
                    io::REVERSE,
                    trim3,
                    trim5 )
            );

            if (read_data_file == NULL || read_data_file->is_ok() == false)
            {
                log_error(stderr, "unable to open read file \"%s\"\n", read_name1);
                return 1;
            }

            // print the command line options
            {
                const bowtie2::cuda::SimpleFunc& score_min = params.scoring_mode == bowtie2::cuda::EditDistanceMode ?
                                              params.scoring_scheme.ed.m_score_min :
                                              params.scoring_scheme.sw.m_score_min;
                
                log_verbose(stderr, "  mode           = %s\n", bowtie2::cuda::mapping_mode( params.mode ));
                log_verbose(stderr, "  scoring        = %s\n", bowtie2::cuda::scoring_mode( params.scoring_mode ));
                log_verbose(stderr, "  score-min      = %s:%.2f:%.2f\n", score_min.type_string(), score_min.k, score_min.m);
                log_verbose(stderr, "  alignment type = %s\n", params.alignment_type == bowtie2::cuda::LocalAlignment ? "local" : "end-to-end");
                log_verbose(stderr, "  seed length    = %u\n", params.seed_len);
                log_verbose(stderr, "  seed interval  = (%s, %.3f, %.3f)\n", params.seed_freq.type_symbol(), params.seed_freq.k, params.seed_freq.m);
                log_verbose(stderr, "  seed rounds    = %u\n", params.max_reseed);
                log_verbose(stderr, "  max hits       = %u\n", params.max_hits);
                log_verbose(stderr, "  max edit dist  = %u\n", params.max_dist);
                log_verbose(stderr, "  max effort     = %u\n", params.max_effort);
                log_verbose(stderr, "  substitutions  = %u\n", params.allow_sub);
                log_verbose(stderr, "  mapq filter    = %u\n", params.mapq_filter);
                log_verbose(stderr, "  randomized     = %s\n", params.randomized ? "yes" : "no");
                if (params.allow_sub)
                    log_verbose(stderr, "  subseed length = %u\n", params.subseed_len);
            }

            //
            // Setup the compute threads
            //

            uint32 batch_size = uint32(-1);

            std::vector< bowtie2::cuda::Stats > device_stats( cuda_devices.size() );
            std::vector< SharedPointer<bowtie2::cuda::ComputeThreadSE> > compute_threads( cuda_devices.size() );

            for (uint32 i = 0; i < cuda_devices.size(); ++i)
            {
                device_stats[i] = bowtie2::cuda::Stats( params );

                compute_threads[i] = SharedPointer<bowtie2::cuda::ComputeThreadSE>(
                    new bowtie2::cuda::ComputeThreadSE(
                        i,
                        cuda_devices[i],
                        *reference_data,
                        *driver_data,
                        string_options,
                        params,
                        device_stats[i] ) );

                batch_size = nvbio::min( batch_size, compute_threads[i]->gauge_batch_size() );
            }

            //
            // Setup the input thread
            //

            Timer timer;
            timer.start();

            bowtie2::cuda::Stats input_stats( params );

            bowtie2::cuda::InputThreadSE input_thread( read_data_file.get(), input_stats, batch_size, params.avg_read_length );
            input_thread.create();

            for (uint32 i = 0; i < cuda_devices.size(); ++i)
            {
                compute_threads[i]->set_input( &input_thread );
                compute_threads[i]->set_output( output_file.get() );
                compute_threads[i]->create();
            }

            uint32                        n_reads = 0;
            bowtie2::cuda::AlignmentStats mate1;

            for (uint32 i = 0; i < cuda_devices.size(); ++i)
            {
                compute_threads[i]->join();

                n_reads += device_stats[i].n_reads;
                mate1.merge( device_stats[i].mate1 );
            }

            input_thread.join();

            timer.stop();

            log_verbose( stderr, "  compute threads joined\n" );

            // close the output file
            output_file->close();

            //
            // Print statistics
            //

            // transfer I/O statistics
            io::IOStats iostats = output_file->get_aggregate_statistics();
            const bowtie2::cuda::KernelStats& io = iostats.output_process_timings;

            log_stats(stderr, "  total         : %.2f sec (avg: %.3fK reads/s).\n", timer.seconds(), 1.0e-3f * float(n_reads) / timer.seconds());
            log_stats(stderr, "  reads   I/O   : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", input_stats.read_io.time, 1.0e-6f * input_stats.read_io.avg_speed(), 1.0e-6f * input_stats.read_io.max_speed);
            log_stats(stderr, "  results I/O   : %.2f sec (avg: %.3fM reads/s, max: %.3fM reads/s).\n", io.time, 1.0e-6f * io.avg_speed(), 1.0e-6f * io.max_speed);

            uint32&              n_mapped       = mate1.n_mapped;
            uint32&              n_unique       = mate1.n_unique;
            uint32&              n_ambiguous    = mate1.n_ambiguous;
            uint32&              n_nonambiguous = mate1.n_unambiguous;
            uint32&              n_multiple     = mate1.n_multiple;
            {
                log_stats(stderr, "  mapped reads : %.2f %% - of these:\n", 100.0f * float(n_mapped)/float(n_reads) );
                log_stats(stderr, "    aligned uniquely      : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(n_unique)/float(n_mapped), 100.0f * float(n_mapped - n_multiple)/float(n_reads) );
                log_stats(stderr, "    aligned unambiguously : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(n_nonambiguous)/float(n_mapped), 100.0f * float(n_nonambiguous)/float(n_reads) );
                log_stats(stderr, "    aligned ambiguously   : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(n_ambiguous)/float(n_mapped), 100.0f * float(n_ambiguous)/float(n_reads) );
                log_stats(stderr, "    aligned multiply      : %4.1f%% (%4.1f%% of total)\n", 100.0f * float(n_multiple)/float(n_mapped), 100.0f * float(n_multiple)/float(n_reads) );
                log_ed( mate1 );
                log_mapq( mate1 );
            }

            // generate an html report
            if (params.report.length())
                bowtie2::cuda::generate_report_header( n_reads, params, mate1, (uint32)cuda_devices.size(), &device_stats[0], params.report.c_str() );
        }

        log_info( stderr, "nvBowtie... done\n" );
    }
    catch (nvbio::cuda_error e)
    {
        log_error(stderr, "caught a nvbio::cuda_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
    }
    catch (...)
    {
        log_error(stderr, "caught an unknown exception!\n");
    }

	return 0;
}

