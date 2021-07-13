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

// nvBWT.cu
//

#define NVBIO_CUDA_DEBUG

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
#include <algorithm>
#include <crc/crc.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/bnt.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/thrust_view.h>
#include <nvbio/basic/dna.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/fmindex/bwt.h>
#include <nvbio/fasta/fasta.h>
#include <nvbio/io/fmindex/fmindex.h>
#include <nvbio/sufsort/sufsort.h>
#include "filelist.h"

// PAC File Type
enum PacType { BPAC = 0, WPAC = 1 };

using namespace nvbio;

unsigned char nst_nt4_table[256] = {
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 5 /*'-'*/, 4, 4,
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 0, 4, 1,  4, 4, 4, 2,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  3, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 0, 4, 1,  4, 4, 4, 2,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  3, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4, 
	4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4
};


#define RAND    0
#define RAND48  1

#if (GENERATOR == RAND) || ((GENERATOR == RAND48) && defined(WIN32))

// generate random base pairs using rand()
inline void  srand_bp(const unsigned int s) { srand(s); }
inline float frand() { return float(rand()) / float(RAND_MAX); }
inline uint8 rand_bp() { return uint8( frand() * 4 ) & 3; }

#elif (GENERATOR == RAND48)

// generate random base pairs using rand48()
inline void  srand_bp(const unsigned int s) { srand48(s); }
inline uint8 rand_bp() { return uint8( drand48() * 4 ) & 3; }

#endif

struct Counter
{
    Counter() : m_size(0), m_reads(0) {}

    void begin_read() { m_reads++; }
    void end_read() {}

    void id(const uint8 c) {}
    void read(const uint8 c) { m_size++; }

    uint64 m_size;
    uint32 m_reads;
};

template <typename stream_type>
struct Writer
{
    Writer(stream_type stream, const uint32 reads, const uint64 max_size) :
        m_max_size(max_size), m_size(0), m_stream( stream )
    {
        m_bntseq.seed = 11;
        m_bntseq.anns_data.resize( reads );
        m_bntseq.anns_info.resize( reads );

        srand_bp( m_bntseq.seed );

        for (uint32 i = 0; i < 4; ++i)
            m_freq[i] = 0;
    }

    void begin_read()
    {
        BNTAnnData& ann_data = m_bntseq.anns_data[ m_bntseq.n_seqs ];
        ann_data.len    = 0;
        ann_data.gi     = 0;
        ann_data.offset = m_size;
        ann_data.n_ambs = 0;

        BNTAnnInfo& ann_info = m_bntseq.anns_info[ m_bntseq.n_seqs ];
        ann_info.anno   = "null";

        m_lasts = 0;
    }
    void end_read()
    {
        m_bntseq.n_seqs++;
    }

    void id(const uint8 c)
    {
        m_bntseq.anns_info[ m_bntseq.n_seqs ].name.push_back(char(c));
    }
    void read(const uint8 s)
    {
        if (m_size < m_max_size)
        {
            const uint8 c = nst_nt4_table[s];

            const uint8 sc = c < 4 ? c : rand_bp();

            m_stream[ m_size ] = sc;

            // keep track of the symbol frequencies
            ++m_freq[sc];

            if (c >= 4) // we have an N
            {
                if (m_lasts == s) // contiguous N
                {
                    // increment length of the last hole
                    ++m_bntseq.ambs.back().len;
                }
                else
                {
                    // beginning of a new hole
                    BNTAmb amb;
                    amb.len    = 1;
                    amb.offset = m_size;
                    amb.amb    = s;

                    m_bntseq.ambs.push_back( amb );

                    ++m_bntseq.anns_data[ m_bntseq.n_seqs ].n_ambs;
                    ++m_bntseq.n_holes;
                }
            }
            // save last symbol
            m_lasts = s;

            // update sequence length
            BNTAnnData& ann_data = m_bntseq.anns_data[ m_bntseq.n_seqs ];
            ann_data.len++;
        }

        m_bntseq.l_pac++;

        m_size++;
    }

    uint64      m_max_size;
    uint64      m_size;
    stream_type m_stream;

    BNTSeq      m_bntseq;
    uint8       m_lasts;

    uint32      m_freq[4];
};

template <typename StreamType>
bool save_stream(FILE* output_file, const uint64 seq_words, const StreamType* stream)
{
    for (uint64 words = 0; words < seq_words; words += 1024)
    {
        const uint32 n_words = (uint32)nvbio::min( uint64(1024u), uint64(seq_words - words) );
        if (fwrite( stream + words, sizeof(StreamType), n_words, output_file ) != n_words)
            return false;
    }
    return true;
}

//
// .wpac file
//
void save_wpac(const uint32 seq_length, const uint32* string_storage, const char* pac_name)
{
    log_info(stderr, "\nwriting \"%s\"... started\n", pac_name);

    const uint32 seq_words = util::divide_ri( seq_length, 16 );

    FILE* output_file = fopen( pac_name, "wb" );
    if (output_file == NULL)
    {
        log_error(stderr, "  could not open output file \"%s\"!\n", pac_name );
        exit(1);
    }

    // write the sequence length as a uint64
    const uint64 len = seq_length;
    fwrite( &len, sizeof(len), 1u, output_file );

    // save the uint32 stream
    if (save_stream( output_file, seq_words, string_storage ) == false)
    {
        log_error(stderr, "  writing failed!\n");
        exit(1);
    }

    fclose( output_file );
    log_info(stderr, "writing \"%s\"... done\n", pac_name);
}

//
// .pac file
//
void save_bpac(const uint32 seq_length, const uint32* string_storage, const char* pac_name)
{
    typedef PackedStream<const uint32*,uint8,2,true,int64>       stream_type;
    typedef PackedStream<      uint8*, uint8,2,true,int64>   pac_stream_type;

    log_info(stderr, "\nwriting \"%s\"... started\n", pac_name);

    const uint32 bps_per_byte = 4u;
    const uint64 seq_bytes    = (seq_length + bps_per_byte - 1u) / bps_per_byte;

    FILE* output_file = fopen( pac_name, "wb" );
    if (output_file == NULL)
    {
        log_error(stderr, "  could not open output file \"%s\"!\n", pac_name );
        exit(1);
    }

    // copy the uint32 packed stream into a uint8 pac stream
    thrust::host_vector<uint8> pac_storage( seq_bytes );
    pac_stream_type pac_string( nvbio::plain_view( pac_storage ) );
        stream_type     string( string_storage );

    for (uint32 i = 0; i < seq_length; ++i)
        pac_string[i] = string[i];

    // save the uint8 stream
    if (save_stream( output_file, seq_bytes, nvbio::raw_pointer( pac_storage ) ) == false)
    {
        log_error(stderr, "  writing failed!\n");
        exit(1);
    }
	// the following code makes the pac file size always (l_pac/4+1+1)
    if (seq_length % 4 == 0)
    {
	    const uint8 ct = 0;
	    fwrite( &ct, 1, 1, output_file );
    }
    {
        const uint8 ct = seq_length % 4;
        fwrite( &ct, 1, 1, output_file );
    }

    fclose( output_file );
    log_info(stderr, "writing \"%s\"... done\n", pac_name);
}

//
// .pac | .wpac file
//
void save_pac(const uint32 seq_length, const uint32* string_storage, const char* pac_name, const PacType pac_type)
{
    if (pac_type == BPAC)
        save_bpac( seq_length, string_storage, pac_name );
    else
        save_wpac( seq_length, string_storage, pac_name );
}

//
// .bwt file
//
void save_bwt(const uint32 seq_length, const uint32 seq_words, const uint32 primary, const uint32* cumFreq, const uint32* h_bwt_storage, const char* bwt_name)
{
    log_info(stderr, "\nwriting \"%s\"... started\n", bwt_name);
    FILE* output_file = fopen( bwt_name, "wb" );
    if (output_file == NULL)
    {
        log_error(stderr, "  could not open output file \"%s\"!\n", bwt_name );
        exit(1);
    }
    fwrite( &primary, sizeof(uint32), 1, output_file );
    fwrite( cumFreq,  sizeof(uint32), 4, output_file );
    if (save_stream( output_file, seq_words, h_bwt_storage ) == false)
    {
        log_error(stderr, "  writing failed!\n");
        exit(1);
    }
    fclose( output_file );
    log_info(stderr, "writing \"%s\"... done\n", bwt_name);
}

//
// .sa file
//
void save_ssa(const uint32 seq_length, const uint32 sa_intv, const uint32 ssa_len, const uint32 primary, const uint32* cumFreq, const uint32* h_ssa, const char* sa_name)
{
    log_info(stderr, "\nwriting \"%s\"... started\n", sa_name);
    FILE* output_file = fopen( sa_name, "wb" );
    if (output_file == NULL)
    {
        log_error(stderr, "  could not open output file \"%s\"!\n", sa_name );
        exit(1);
    }

    fwrite( &primary,       sizeof(uint32),     1u,         output_file );
    fwrite( &cumFreq,       sizeof(uint32),     4u,         output_file );
    fwrite( &sa_intv,       sizeof(uint32),     1u,         output_file );
    fwrite( &seq_length,    sizeof(uint32),     1u,         output_file );
    fwrite( &h_ssa[1],      sizeof(uint32),     ssa_len-1,  output_file );
    fclose( output_file );
    log_info(stderr, "writing \"%s\"... done\n", sa_name);
}

int build(
    const char*  input_name,
    const char*  output_name,
    const char*  pac_name,
    const char*  rpac_name,
    const char*  bwt_name,
    const char*  rbwt_name,
    const char*  sa_name,
    const char*  rsa_name,
    const uint64 max_length,
    const PacType pac_type,
    const bool    compute_crc)
{
    std::vector<std::string> sortednames;
    list_files(input_name, sortednames);

    uint32 n_inputs = (uint32)sortednames.size();
    log_info(stderr, "\ncounting bps... started\n");
    // count entire sequence length
    Counter counter;

    for (uint32 i = 0; i < n_inputs; ++i)
    {
        log_info(stderr, "  counting \"%s\"\n", sortednames[i].c_str());

        FASTA_inc_reader fasta( sortednames[i].c_str() );
        if (fasta.valid() == false)
        {
            log_error(stderr, "  unable to open file\n");
            exit(1);
        }

        while (fasta.read( 1024, counter ) == 1024);
    }
    log_info(stderr, "counting bps... done\n");

    const uint64 seq_length   = nvbio::min( (uint64)counter.m_size, (uint64)max_length );
    const uint32 bps_per_word = sizeof(uint32)*4u;
    const uint64 seq_words    = (seq_length + bps_per_word - 1u) / bps_per_word;

    log_info(stderr, "\nstats:\n");
    log_info(stderr, "  reads           : %u\n", counter.m_reads );
    log_info(stderr, "  sequence length : %llu bps (%.1f MB)\n",
        seq_length,
        float(seq_words*sizeof(uint32))/float(1024*1024));
    log_info(stderr, "  buffer size     : %.1f MB\n",
        2*seq_words*sizeof(uint32)/1.0e6f );

    const uint32 sa_intv = nvbio::io::FMIndexData::SA_INT;
    const uint32 ssa_len = (seq_length + sa_intv) / sa_intv;

    // allocate the actual storage
    thrust::host_vector<uint32> h_string_storage( seq_words+1 );
    thrust::host_vector<uint32> h_bwt_storage( seq_words+1 );
    thrust::host_vector<uint32> h_ssa( ssa_len );

    typedef PackedStream<const uint32*,uint8,io::FMIndexData::BWT_BITS,io::FMIndexData::BWT_BIG_ENDIAN> const_stream_type;
    typedef PackedStream<      uint32*,uint8,io::FMIndexData::BWT_BITS,io::FMIndexData::BWT_BIG_ENDIAN>       stream_type;

    stream_type h_string( nvbio::plain_view( h_string_storage ) );

    uint32 cumFreq[4] = { 0, 0, 0, 0 };

    log_info(stderr, "\nbuffering bps... started\n");
    // read all files
    {
        Writer<stream_type> writer( h_string, counter.m_reads, seq_length );

        for (uint32 i = 0; i < n_inputs; ++i)
        {
            log_info(stderr, "  buffering \"%s\"\n", sortednames[i].c_str());

            FASTA_inc_reader fasta( sortednames[i].c_str() );
            if (fasta.valid() == false)
            {
                log_error(stderr, "  unable to open file!\n");
                exit(1);
            }

            while (fasta.read( 1024, writer ) == 1024);
        }

        save_bns( writer.m_bntseq, output_name );

        // compute the cumulative symbol frequencies
        cumFreq[0] = writer.m_freq[0];
        cumFreq[1] = writer.m_freq[1] + cumFreq[0];
        cumFreq[2] = writer.m_freq[2] + cumFreq[1];
        cumFreq[3] = writer.m_freq[3] + cumFreq[2];

        if (cumFreq[3] != seq_length)
        {
            log_error(stderr, "  mismatching symbol frequencies!\n");
            log_error(stderr, "    (%u, %u, %u, %u)\n", cumFreq[0], cumFreq[1], cumFreq[2], cumFreq[3]);
            exit(1);
        }
    }
    log_info(stderr, "buffering bps... done\n");

    if (compute_crc)
    {
        const uint32 crc = crcCalc( h_string, uint32(seq_length) );
        log_info(stderr, "  crc: %u\n", crc);
    }

    try
    {
        BWTParams params;
        uint32    primary;

        thrust::device_vector<uint32> d_string_storage( h_string_storage );
        thrust::device_vector<uint32> d_bwt_storage( seq_words+1 );

        const_stream_type d_string( nvbio::plain_view( d_string_storage ) );
              stream_type d_bwt(    nvbio::plain_view( d_bwt_storage ) );

        Timer timer;

        log_info(stderr, "\nbuilding forward BWT... started\n");
        timer.start();
        {
            StringBWTSSAHandler<const_stream_type,stream_type,uint32*> output(
                seq_length,                         // string length
                d_string,                           // string
                sa_intv,                            // SSA sampling interval
                d_bwt,                              // output bwt iterator
                nvbio::plain_view( h_ssa ) );       // output ssa iterator

            cuda::blockwise_suffix_sort(
                seq_length,
                d_string,
                output,
                &params );

            // remove the dollar symbol
            output.remove_dollar();

            primary = output.primary();
        }
        timer.stop();
        log_info(stderr, "building forward BWT... done: %um:%us\n", uint32(timer.seconds()/60), uint32(timer.seconds())%60);
        log_info(stderr, "  primary: %u\n", primary);

        // save everything to disk
        {
            // copy to the host
            thrust::copy( d_bwt_storage.begin(),
                          d_bwt_storage.begin() + seq_words,
                          h_bwt_storage.begin() );

            if (compute_crc)
            {
                const_stream_type h_bwt( nvbio::plain_view( h_bwt_storage ) );
                const uint32 crc = crcCalc( h_bwt, uint32(seq_length) );
                log_info(stderr, "  crc: %u\n", crc);
            }

            save_pac( seq_length, nvbio::plain_view( h_string_storage ),                           pac_name, pac_type );
            save_bwt( seq_length, seq_words, primary, cumFreq, nvbio::plain_view( h_bwt_storage ), bwt_name );
            save_ssa( seq_length, sa_intv, ssa_len, primary, cumFreq, nvbio::plain_view( h_ssa ),  sa_name );
        }

        // reverse the string in h_string_storage
        {
            // reuse the bwt storage to build the reverse
            uint32* h_rbase_stream = nvbio::plain_view( h_bwt_storage );
            stream_type h_rstring( h_rbase_stream );

            // reverse the string
            for (uint32 i = 0; i < seq_length; ++i)
                h_rstring[i] = h_string[ seq_length - i - 1u ];

            // and now swap the vectors
            h_bwt_storage.swap( h_string_storage );
            h_string = stream_type( nvbio::plain_view( h_string_storage ) );

            // and copy back the new string to the device
            d_string_storage = h_string_storage;
        }

        log_info(stderr, "\nbuilding reverse BWT... started\n");
        timer.start();
        {
            StringBWTSSAHandler<const_stream_type,stream_type,uint32*> output(
                seq_length,                         // string length
                d_string,                           // string
                sa_intv,                            // SSA sampling interval
                d_bwt,                              // output bwt iterator
                nvbio::plain_view( h_ssa ) );       // output ssa iterator

            cuda::blockwise_suffix_sort(
                seq_length,
                d_string,
                output,
                &params );

            // remove the dollar symbol
            output.remove_dollar();

            primary = output.primary();
        }
        timer.stop();
        log_info(stderr, "building reverse BWT... done: %um:%us\n", uint32(timer.seconds()/60), uint32(timer.seconds())%60);
        log_info(stderr, "  primary: %u\n", primary);

        // save everything to disk
        {
            // copy to the host
            thrust::copy( d_bwt_storage.begin(),
                          d_bwt_storage.begin() + seq_words,
                          h_bwt_storage.begin() );

            if (compute_crc)
            {
                const_stream_type h_bwt( nvbio::plain_view( h_bwt_storage ) );
                const uint32 crc = crcCalc( h_bwt, uint32(seq_length) );
                log_info(stderr, "  crc: %u\n", crc);
            }

            save_pac( seq_length, nvbio::plain_view( h_string_storage ),                           rpac_name, pac_type );
            save_bwt( seq_length, seq_words, primary, cumFreq, nvbio::plain_view( h_bwt_storage ), rbwt_name );
            save_ssa( seq_length, sa_intv, ssa_len, primary, cumFreq, nvbio::plain_view( h_ssa ),  rsa_name );
        }
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
        log_error(stderr,"unknown exception caught!\n");
        exit(1);
    }
    return 0;
}

int main(int argc, char* argv[])
{
    crcInit();

    if (argc < 2)
    {
        log_info(stderr, "please specify input and output file names, e.g:\n");
        log_info(stderr, "  nvBWT [options] myinput.*.fa output-prefix\n");
        log_info(stderr, "  options:\n");
        log_info(stderr, "    -v | --verbosity      select verbosity\n");
        log_info(stderr, "    -m | --max-length     clamp input to max_length\n");
        log_info(stderr, "    -b | --byte-packing   output byte packed .pac\n");
        log_info(stderr, "    -w | --word-packing   output word packed .wpac\n");
        log_info(stderr, "    -c | --crc            compute crcs\n");
        log_info(stderr, "    -d | --device         cuda device\n");
        exit(0);
    }

    const char* file_names[2] = { NULL, NULL };
    uint64  max_length  = uint64(-1);
    PacType pac_type    = BPAC;
    bool    crc         = false;
    int     cuda_device = -1;

    uint32 n_files = 0;
    for (int32 i = 1; i < argc; ++i)
    {
        const char* arg = argv[i];

        if ((strcmp( arg, "-m" )                    == 0) ||
            (strcmp( arg, "--max-length" )          == 0))
        {
            max_length = atoi( argv[++i] );
        }
        else if ((strcmp( argv[i], "-v" )           == 0) ||
                 (strcmp( argv[i], "-verbosity" )   == 0) ||
                 (strcmp( argv[i], "--verbosity" )  == 0))
        {
            set_verbosity( Verbosity( atoi( argv[++i] ) ) );
        }
        else if ((strcmp( arg, "-b" )               == 0) ||
                 (strcmp( arg, "--byte-packing" )   == 0))
        {
            pac_type = BPAC;
        }
        else if ((strcmp( arg, "-w" )               == 0) ||
                 (strcmp( arg, "--word-packing" )   == 0))
        {
            pac_type = WPAC;
        }
        else if ((strcmp( arg, "-c" )               == 0) ||
                 (strcmp( arg, "--crc" )            == 0))
        {
            crc = true;
        }
        else if ((strcmp( arg, "-d" )               == 0) ||
                 (strcmp( arg, "--device" )         == 0))
        {
            cuda_device = atoi( argv[++i] );
        }
        else
            file_names[ n_files++ ] = argv[i];
    }

    const char* input_name  = file_names[0];
    const char* output_name = file_names[1];
    std::string pac_string  = std::string( output_name ) + (pac_type == BPAC ? ".pac" : ".wpac");
    const char* pac_name    = pac_string.c_str();
    std::string rpac_string = std::string( output_name ) + (pac_type == BPAC ? ".rpac" : ".rwpac");
    const char* rpac_name   = rpac_string.c_str();
    std::string bwt_string  = std::string( output_name ) + ".bwt";
    const char* bwt_name    = bwt_string.c_str();
    std::string rbwt_string = std::string( output_name ) + ".rbwt";
    const char* rbwt_name   = rbwt_string.c_str();
    std::string sa_string   = std::string( output_name ) + ".sa";
    const char* sa_name     = sa_string.c_str();
    std::string rsa_string  = std::string( output_name ) + ".rsa";
    const char* rsa_name    = rsa_string.c_str();

    log_info(stderr, "max length : %lld\n", max_length);
    log_info(stderr, "input      : \"%s\"\n", input_name);
    log_info(stderr, "output     : \"%s\"\n", output_name);

    try
    {
        int device_count;
        cudaGetDeviceCount(&device_count);
        cuda::check_error("cuda-check");

        log_verbose(stderr, "  cuda devices : %d\n", device_count);

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

        size_t free, total;
        cudaMemGetInfo(&free, &total);
        NVBIO_CUDA_DEBUG_STATEMENT( log_info(stderr,"device mem : total: %.1f GB, free: %.1f GB\n", float(total)/float(1024*1024*1024), float(free)/float(1024*1024*1024)) );

        cuda::check_error("cuda-memory-check");

        return build( input_name, output_name, pac_name, rpac_name, bwt_name, rbwt_name, sa_name, rsa_name, max_length, pac_type, crc );
    }
    catch (nvbio::cuda_error e)
    {
        log_error(stderr, "caught a nvbio::cuda_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (thrust::system::system_error e)
    {
        log_error(stderr, "caught a thrust::system_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        return 1;
    }
    catch (...)
    {
        log_error(stderr, "caught an unknown exception!\n");
        return 1;
    }
}

