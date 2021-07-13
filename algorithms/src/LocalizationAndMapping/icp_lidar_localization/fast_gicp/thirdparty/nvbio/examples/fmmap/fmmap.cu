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

// fmmap.cu
//

#include <stdio.h>
#include <stdlib.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/shared_pointer.h>
#include <nvbio/basic/dna.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/strings/infix.h>
#include <nvbio/strings/seeds.h>
#include <nvbio/fmindex/filter.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/io/sequence/sequence_encoder.h>
#include <nvbio/io/fmindex/fmindex.h>
#include <nvbio/alignment/alignment.h>
#include <nvbio/alignment/batched.h>

#include "util.h"

using namespace nvbio;

// alignment params
//
struct Params
{
    uint32 seed_len;
    uint32 seed_intv;
    uint32 merge_intv;
};

// query stats
//
struct Stats
{
    Stats() :
        time(0),
        extract_time(0),
        rank_time(0),
        locate_time(0),
        align_time(0),
        reads(0),
        aligned(0),
        queries(0),
        occurrences(0) {}

    float   time;
    float   extract_time;
    float   rank_time;
    float   locate_time;
    float   align_time;
    uint64  reads;
    uint64  aligned;
    uint64  queries;
    uint64  occurrences;
};

// the pipeline state
//
struct Pipeline
{
    typedef io::FMIndexDataDevice::fm_index_type        fm_index_type;
    typedef FMIndexFilterDevice<fm_index_type>          fm_filter_type;

    Params                                  params;    // program options
    SharedPointer<io::SequenceDataDevice>   ref_data;  // reference data
    SharedPointer<io::FMIndexDataDevice>    fm_data;   // fm-index data
    fm_filter_type                          fm_filter; // fm-index filter
};

// transform an (index-pos,seed-id) hit into a diagonal (text-pos = index-pos - seed-pos, read-id)
struct hit_to_diagonal
{
    typedef uint2  argument_type;
    typedef uint2  result_type;

    // constructor
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    hit_to_diagonal(const string_set_infix_coord_type* _seed_coords) : seed_coords(_seed_coords) {}

    // functor operator
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint2 operator() (const uint2 hit) const
    {
        const uint32 index_pos = hit.x;
        const uint32 seed_id   = hit.y;

        const string_set_infix_coord_type seed = seed_coords[ seed_id ];

        const uint32 read_pos = infix_begin( seed );
        const uint32 read_id  =   string_id( seed );

        return make_uint2( index_pos - read_pos, read_id );
    }

    const string_set_infix_coord_type* seed_coords;
};

// extract a set of uniformly spaced seeds from a string-set and return it as an InfixSet
//
template <typename system_tag, typename string_set_type>
InfixSet<string_set_type, const string_set_infix_coord_type*>
extract_seeds(
    const string_set_type                                   string_set,         // the input string-set
    const uint32                                            seed_len,           // the seeds length
    const uint32                                            seed_interval,      // the spacing between seeds
    nvbio::vector<system_tag,string_set_infix_coord_type>&  seed_coords)        // the output vector of seed coordinates
{
    // enumerate all seeds
    const uint32 n_seeds = enumerate_string_set_seeds(
        string_set,
        uniform_seeds_functor<>( seed_len, seed_interval ),
        seed_coords );

    // and build the output infix-set
    return InfixSet<string_set_type, const string_set_infix_coord_type*>(
        n_seeds,
        string_set,
        nvbio::plain_view( seed_coords ) );
}

// a functor to extract the read infixes from the hit diagonals
//
struct read_infixes
{
    // constructor
    NVBIO_HOST_DEVICE
    read_infixes(const io::ConstSequenceDataView reads) :
        m_reads( reads ) {}

    // functor operator
    NVBIO_HOST_DEVICE
    string_infix_coord_type operator() (const uint2 diagonal) const
    {
        const io::SequenceDataAccess<DNA_N> reads( m_reads );

        const uint32 read_id = diagonal.y;

        // fetch the read range
        return reads.get_range( read_id );
    }

    const io::ConstSequenceDataView m_reads;
};

// a functor to extract the genome infixes from the hit diagonals
//
template <uint32 BAND_LEN>
struct genome_infixes
{
    typedef const io::SequenceDataAccess<DNA_N,io::ConstSequenceDataView> read_access_type;

    // constructor
    NVBIO_HOST_DEVICE
    genome_infixes(const uint32 genome_len, const io::ConstSequenceDataView reads) :
        m_genome_len( genome_len ),
        m_reads( reads ) {}

    // functor operator
    NVBIO_HOST_DEVICE
    string_infix_coord_type operator() (const uint2 diagonal) const
    {
        const io::SequenceDataAccess<DNA_N> reads( m_reads );

        const uint32 read_id  = diagonal.y;
        const uint32 text_pos = diagonal.x;

        // fetch the read range
        const uint2  read_range = reads.get_range( read_id );
        const uint32 read_len   = read_range.y - read_range.x;

        // compute the segment of text to align to
        const uint32 genome_begin = text_pos > BAND_LEN/2 ? text_pos - BAND_LEN/2 : 0u;
        const uint32 genome_end   = nvbio::min( genome_begin + read_len + BAND_LEN, m_genome_len );

        return make_uint2( genome_begin, genome_end );
    }

    const uint32                    m_genome_len;
    const io::ConstSequenceDataView m_reads;
};

// a functor to extract the score from a sink
//
struct sink_score
{
    typedef aln::BestSink<int16> argument_type;
    typedef int16                result_type;

    // functor operator
    NVBIO_HOST_DEVICE
    int16 operator() (const aln::BestSink<int16>& sink) const { return sink.score; }
};

// perform q-gram index mapping
//
void map(
    Pipeline&                               pipeline,
    const io::SequenceDataDevice&           reads,
    nvbio::vector<device_tag,int16>&        best_scores,
          Stats&                            stats)
{
    typedef io::SequenceDataAccess<DNA>                                         genome_access_type;
    typedef genome_access_type::sequence_stream_type                            genome_string;
    typedef io::SequenceDataAccess<DNA_N>                                       read_access_type;
    typedef read_access_type::sequence_string_set_type                          read_string_set_type;
    typedef read_access_type::sequence_stream_type                              read_stream;
    typedef string_set_infix_coord_type                                         infix_coord_type;
    typedef nvbio::vector<device_tag,infix_coord_type>                          infix_vector_type;
    typedef InfixSet<read_string_set_type, const string_set_infix_coord_type*>  seed_string_set_type;

    // fetch the program options
    const Params& params = pipeline.params;

    // fetch the genome string
    const genome_access_type genome_access( *pipeline.ref_data );
    const uint32             genome_len = genome_access.bps();
    const genome_string      genome( genome_access.sequence_stream() );

    // fetch an fm-index view
    const Pipeline::fm_index_type fm_index = pipeline.fm_data->index();

    // fetch the fm-index filter
    Pipeline::fm_filter_type& fm_filter = pipeline.fm_filter;

    // prepare some vectors to store the query qgrams
    infix_vector_type seed_coords;

    Timer timer;
    timer.start();

    const read_access_type reads_access( reads );
    const read_string_set_type read_string_set = reads_access.sequence_string_set();
    const seed_string_set_type seed_string_set = extract_seeds(
        read_string_set,
        params.seed_len,
        params.seed_intv,
        seed_coords );

    cudaDeviceSynchronize();
    timer.stop();
    const float extract_time = timer.seconds();

    stats.queries       += seed_string_set.size();
    stats.extract_time  += extract_time;

    //
    // search the sorted seeds with the FM-index filter
    //

    const uint32 batch_size = 16*1024*1024;

    typedef uint2 hit_type; // each hit will be an (index-pos,seed-id) coordinate pair

    // prepare storage for the output hits
    nvbio::vector<device_tag,hit_type>      hits( batch_size );
    nvbio::vector<device_tag,uint32>        out_reads( batch_size );
    nvbio::vector<device_tag,int16>         out_scores( batch_size );
    nvbio::vector<device_tag,uint8>         temp_storage;

    timer.start();

    // first step: rank the query seeds
    const uint64 n_hits = fm_filter.rank( fm_index, seed_string_set );

    cudaDeviceSynchronize();
    timer.stop();
    stats.rank_time   += timer.seconds();
    stats.occurrences += n_hits;

    nvbio::vector<device_tag, aln::BestSink<int16> >  sinks( batch_size );
    nvbio::vector<device_tag,string_infix_coord_type> genome_infix_coords( batch_size );
    nvbio::vector<device_tag,string_infix_coord_type> read_infix_coords( batch_size );

    static const uint32 BAND_LEN = 31;

    // loop through large batches of hits and locate & merge them
    for (uint64 hits_begin = 0; hits_begin < n_hits; hits_begin += batch_size)
    {
        const uint64 hits_end = nvbio::min( hits_begin + batch_size, n_hits );

        timer.start();

        fm_filter.locate(
            hits_begin,
            hits_end,
            hits.begin() );

        cudaDeviceSynchronize();
        timer.stop();
        stats.locate_time += timer.seconds();

        // transform the (index-pos,seed-id) hit coordinates into diagonals, (text-pos = index-pos - seed-pos, read-id)
        thrust::transform(
            hits.begin(),
            hits.begin() + hits_end - hits_begin,
            hits.begin(),
            hit_to_diagonal( nvbio::plain_view( seed_coords ) ) );

        timer.start();

        // build the set of read infixes
        thrust::transform(
            hits.begin(),
            hits.begin() + hits_end - hits_begin,
            read_infix_coords.begin(),
            read_infixes( nvbio::plain_view( reads ) ) );

        // build the set of genome infixes
        thrust::transform(
            hits.begin(),
            hits.begin() + hits_end - hits_begin,
            genome_infix_coords.begin(),
            genome_infixes<BAND_LEN>( genome_len, nvbio::plain_view( reads ) ) );

        typedef nvbio::vector<device_tag,string_infix_coord_type>::const_iterator infix_iterator;

        const SparseStringSet<read_stream,infix_iterator> read_infix_set(
            hits_end - hits_begin,
            reads_access.sequence_stream(),
            read_infix_coords.begin() );

        const SparseStringSet<genome_string,infix_iterator> genome_infix_set(
            hits_end - hits_begin,
            genome,
            genome_infix_coords.begin() );

        typedef aln::MyersTag<5u> myers_dna5_tag;
        //const aln::SimpleGotohScheme gotoh( 2, -2, -5, -3 );
        aln::batch_banded_alignment_score<BAND_LEN>(
            aln::make_edit_distance_aligner<aln::SEMI_GLOBAL, myers_dna5_tag>(),
            //aln::make_gotoh_aligner<aln::LOCAL>( gotoh ),
            read_infix_set,
            genome_infix_set,
            sinks.begin(),
            aln::DeviceThreadScheduler(),
            reads.max_sequence_len(),
            reads.max_sequence_len() + BAND_LEN );

        cudaDeviceSynchronize();
        timer.stop();
        stats.align_time += timer.seconds();

        // compute the best score for each read in this batch;
        // note that we divide the string-id by 2 to merge results coming from the forward
        // and reverse-complemented strands
        cuda::reduce_by_key(
            hits_end - hits_begin,
            thrust::make_transform_iterator(
                hits.begin(),
                make_composition_functor( divide_by_two(), component_functor<hit_type>( 1u ) ) ), // take the second component divided by 2
            thrust::make_transform_iterator( sinks.begin(), sink_score() ),
            out_reads.begin(),
            out_scores.begin(),
            thrust::maximum<int16>(),
            temp_storage );

        // and keep track of the global best
        update_scores(
            hits_end - hits_begin,
            nvbio::plain_view( out_reads ),
            nvbio::plain_view( out_scores ),
            nvbio::plain_view( best_scores ) );

        log_verbose(stderr, "\r  processed %6.2f %% reads", 100.0f * float( hits_end ) / float( n_hits ));
    }
    log_verbose_cont(stderr, "\n");
}

// main test entry point
//
int main(int argc, char* argv[])
{
    //
    // perform some basic option parsing
    //

    const uint32 batch_reads   =   1*1024*1024;
    const uint32 batch_bps     = 100*1024*1024;

    const char* reads = argv[argc-1];
    const char* index = argv[argc-2];

    Params params;
    params.seed_len         = 22;
    params.seed_intv        = 10;
    params.merge_intv       = 16;
    uint32 max_reads        = uint32(-1);
    int16  score_threshold  = -20;

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp( argv[i], "-s" ) == 0)
        {
            params.seed_len  = uint32( atoi( argv[++i] ) );
            params.seed_intv = uint32( atoi( argv[++i] ) );
        }
        if (strcmp( argv[i], "-m" ) == 0)
            params.merge_intv = uint32( atoi( argv[++i] ) );
        else if (strcmp( argv[i], "-max-reads" ) == 0)
            max_reads = uint32( atoi( argv[++i] ) );
        else if (strcmp( argv[i], "-t" ) == 0)
            score_threshold = int16( atoi( argv[++i] ) );
    }

    io::SequenceDataHost h_ref;
    if (!io::load_sequence_file( DNA, &h_ref, index ))
    {
        log_error(stderr, "    failed loading reference \"%s\"\n", index);
        return 1u;
    }

    io::FMIndexDataHost h_fmi;
    if (!h_fmi.load( index, io::FMIndexData::FORWARD | io::FMIndexData::SA ))
    {
        log_error(stderr, "    failed loading index \"%s\"\n", index);
        return 1u;
    }

    Pipeline pipeline;

    // store the program options
    pipeline.params = params;

    // build its device version
    pipeline.ref_data = new io::SequenceDataDevice( h_ref );

    // build its device version
    pipeline.fm_data = new io::FMIndexDataDevice( h_fmi, io::FMIndexData::FORWARD |
                                                         io::FMIndexData::SA );

    // open a read file
    log_info(stderr, "  opening reads file... started\n");

    SharedPointer<io::SequenceDataStream> read_data_file(
        io::open_sequence_file(
            reads,
            io::Phred33,
            2*max_reads,
            uint32(-1),
            io::SequenceEncoding( io::FORWARD | io::REVERSE_COMPLEMENT ) ) );

    // check whether the file opened correctly
    if (read_data_file == NULL || read_data_file->is_ok() == false)
    {
        log_error(stderr, "    failed opening file \"%s\"\n", reads);
        return 1u;
    }
    log_info(stderr, "  opening reads file... done\n");

    // keep stats
    Stats stats;

    io::SequenceDataHost h_read_data;

    while (1)
    {
        // load a batch of reads
        if (io::next( DNA_N, &h_read_data, read_data_file.get(), batch_reads, batch_bps ) == 0)
            break;

        log_info(stderr, "  loading reads... started\n");

        // copy it to the device
        const io::SequenceDataDevice d_read_data = h_read_data;

        const uint32 n_reads = d_read_data.size() / 2;

        log_info(stderr, "  loading reads... done\n");
        log_info(stderr, "    %u reads\n", n_reads);

        const int16 worst_score = Field_traits<int16>::min();
        nvbio::vector<device_tag,int16> best_scores( n_reads, worst_score );
        nvbio::vector<device_tag,uint8> temp_storage;

        Timer timer;
        timer.start();

        map(
            pipeline,
            d_read_data,
            best_scores,
            stats );

        timer.stop();
        const float time = timer.seconds();

        // accumulate the number of aligned reads
        stats.reads += n_reads;
        stats.time  += time;

        // count how many reads have a score >= score_threshold
        const uint32 n_aligned = cuda::reduce(
            n_reads,
            thrust::make_transform_iterator( best_scores.begin(), above_threshold( score_threshold ) ),
            thrust::plus<uint32>(),
            temp_storage );

        stats.aligned += n_aligned;

        log_info(stderr, "  aligned %6.2f %% reads (%6.2f K reads/s)\n", 100.0f * float( stats.aligned ) / float( stats.reads ), (1.0e-3f * float( stats.reads )) / stats.time);
        log_verbose(stderr, "  breakdown:\n");
        log_verbose(stderr, "    extract throughput : %.2f B seeds/s\n",  (1.0e-9f * float( stats.queries )) / stats.extract_time);
        log_verbose(stderr, "    rank throughput    : %6.2f K reads/s\n", (1.0e-3f * float( stats.reads )) / stats.rank_time);
        log_verbose(stderr, "                       : %6.2f B seeds/s\n", (1.0e-9f * float( stats.queries )) / stats.rank_time);
        log_verbose(stderr, "    locate throughput  : %6.2f K reads/s\n", (1.0e-3f * float( stats.reads )) / stats.locate_time);
        log_verbose(stderr, "    align throughput   : %6.2f K reads/s\n", (1.0e-3f * float( stats.reads )) / stats.align_time);
        log_verbose(stderr, "                       : %6.2f M hits/s\n",  (1.0e-6f * float( stats.occurrences )) / stats.align_time);
        log_verbose(stderr, "    occurrences        : %.3f B\n", 1.0e-9f * float( stats.occurrences ) );
    }
    return 0;
}
