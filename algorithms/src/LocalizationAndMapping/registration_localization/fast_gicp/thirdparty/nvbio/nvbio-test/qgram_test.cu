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

// qgram_test.cu
//
//#define CUFMI_CUDA_DEBUG
//#define CUFMI_CUDA_ASSERTS

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/strings/seeds.h>
#include <nvbio/basic/shared_pointer.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/qgram/qgram.h>
#include <nvbio/qgram/qgroup.h>
#include <nvbio/qgram/filter.h>
#if defined(_OPENMP)
#include <omp.h>
#endif

namespace nvbio {

// return the size of a given range
struct range_size
{
    typedef uint2  argument_type;
    typedef uint32 result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint2 range) const { return range.y - range.x; }
};

// return 1 for non-empty ranges, 0 otherwise
struct valid_range
{
    typedef uint2  argument_type;
    typedef uint32 result_type;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 operator() (const uint2 range) const { return range.y - range.x > 0 ? 1u : 0u; }
};

// query stats
//
struct Stats
{
    Stats() :
        build_time(0),
        unsorted_time(0),
        sorted_time(0),
        filter_time(0),
        merge_time(0),
        queries(0),
        matches(0),
        occurrences(0),
        merged(0) {}

    float   build_time;
    float   unsorted_time;
    float   sorted_time;
    float   filter_time;
    float   merge_time;
    uint64  queries;
    uint64  matches;
    uint64  occurrences;
    uint64  merged;
};

// build a set of q-grams from a given string, together with their sorted counterpart
//
template <typename genome_string, typename qgram_vector_type, typename index_vector_type>
void build_qgrams(
    const uint32                    Q,
    const uint32                    genome_len,
    const uint32                    genome_offset,
    const genome_string             genome,
    const uint32                    n_queries,
    qgram_vector_type&              qgrams,
    qgram_vector_type&              sorted_qgrams,
    index_vector_type&              sorted_indices)
{
    // build the q-grams
    qgrams.resize( n_queries );
    generate_qgrams( Q, 2u, genome_len, genome, n_queries, thrust::make_counting_iterator<uint32>(genome_offset), qgrams.begin() );

    // sort the q-grams
    sorted_qgrams = qgrams;
    sorted_indices.resize( n_queries );
    thrust::copy(
        thrust::make_counting_iterator<uint32>(genome_offset),
        thrust::make_counting_iterator<uint32>(genome_offset) + n_queries,
        sorted_indices.begin() );

    thrust::sort_by_key( sorted_qgrams.begin(), sorted_qgrams.end(), sorted_indices.begin() );
}

// build a q-gram index from a string
//
template <typename string_type>
void test_qgram_index_build(
    const uint32            Q,
    const uint32            string_len,
    const string_type       string,
          QGramIndexDevice& qgram_index)
{
    log_verbose(stderr, "  building q-gram index... started\n");

    Timer timer;
    timer.start();

    // build the q-gram index
    qgram_index.build(
        Q,              // q-gram size
        2u,             // implicitly convert N to A
        string_len,
        string,
        12u );

    cudaDeviceSynchronize();
    timer.stop();
    const float time = timer.seconds();

    log_verbose(stderr, "  building q-gram index... done\n");
    log_verbose(stderr, "    indexed q-grams : %6.2f M q-grams\n", 1.0e-6f * float( qgram_index.n_qgrams ));
    log_verbose(stderr, "    unique q-grams  : %6.2f M q-grams\n", 1.0e-6f * float( qgram_index.n_unique_qgrams ));
    log_verbose(stderr, "    throughput      : %5.1f M q-grams/s\n", 1.0e-6f * float( string_len ) / time);
    log_verbose(stderr, "    memory usage    : %5.1f MB\n", float( qgram_index.used_device_memory() ) / float(1024*1024) );

    log_verbose(stderr, "  querying q-gram index... started\n");
}

// build a q-gram set-index from a string-set
//
template <typename string_set_type>
void test_qgram_set_index_build(
    const uint32            Q,
    const string_set_type   string_set,
    QGramSetIndexDevice&    qgram_index)
{
    log_verbose(stderr, "  building q-gram set-index... started\n");

    Timer timer;
    timer.start();

    // build the q-gram set index
    qgram_index.build(
        Q,              // q-gram size
        2u,             // implicitly convert N to A
        string_set,
        uniform_seeds_functor<>( Q, 10u ),
        12u );

    cudaDeviceSynchronize();
    timer.stop();
    const float time = timer.seconds();

    log_verbose(stderr, "  building q-gram set-index... done\n");
    log_verbose(stderr, "    indexed q-grams : %6.2f M q-grams\n", 1.0e-6f * float( qgram_index.n_qgrams ));
    log_verbose(stderr, "    unique q-grams  : %6.2f M q-grams\n", 1.0e-6f * float( qgram_index.n_unique_qgrams ));
    log_verbose(stderr, "    throughput      : %5.1f M q-grams/s\n", 1.0e-6f * float( qgram_index.n_qgrams ) / time);
    log_verbose(stderr, "    memory usage    : %5.1f MB\n", float( qgram_index.used_device_memory() ) / float(1024*1024) );
}

// build a q-group index from a string
//
template <typename string_type>
void test_qgroup_index_build(
    const uint32            Q,
    const uint32            string_len,
    const string_type       string,
    QGroupIndexDevice&      qgram_index)
{
    log_verbose(stderr, "  building q-group index... started\n");

    Timer timer;
    timer.start();

    // build the q-group index
    qgram_index.build(
        Q,              // q-group size
        2u,             // implicitly convert N to A
        string_len,
        string );

    cudaDeviceSynchronize();
    timer.stop();
    const float time = timer.seconds();

    log_verbose(stderr, "  building q-group index... done\n");
    log_verbose(stderr, "    indexed q-grams : %6.2f M q-grams\n", 1.0e-6f * float( qgram_index.n_qgrams ));
    log_verbose(stderr, "    unique q-grams  : %6.2f M q-grams\n", 1.0e-6f * float( qgram_index.n_unique_qgrams ));
    log_verbose(stderr, "    throughput      : %5.1f M q-grams/s\n", 1.0e-6f * float( string_len ) / time);
    log_verbose(stderr, "    memory usage    : %5.1f MB\n", float( qgram_index.used_device_memory() ) / float(1024*1024) );

    log_verbose(stderr, "  querying q-group index... started\n");
}

// test a generic q-gram index query, both using plain queries and with a q-gram filter
//
template <typename qgram_index_type, typename genome_string>
void test_qgram_index_query(
          qgram_index_type& qgram_index,
    const uint32            n_queries,
    const uint32            genome_len,
    const uint32            genome_offset,
    const genome_string     genome,
          Stats&            stats)
{
    const uint32 Q = qgram_index.Q;

    typedef typename qgram_index_type::system_tag system_tag;

    // prepare some vectors to store the query qgrams
    nvbio::vector<system_tag,uint64>  qgrams( n_queries );
    nvbio::vector<system_tag,uint64>  sorted_qgrams( n_queries );
    nvbio::vector<system_tag,uint32>  sorted_indices( n_queries );

    build_qgrams(
        Q,
        genome_len,
        genome_offset,
        genome,
        n_queries,
        qgrams,
        sorted_qgrams,
        sorted_indices );

    // prepare a vector to store the query results
    nvbio::vector<system_tag,uint2>  ranges( n_queries );

    log_verbose(stderr, "  querying q-gram index... started\n");

    Timer timer;
    timer.start();

    // search the query q-grams in the index
    thrust::transform(
        qgrams.begin(),
        qgrams.begin() + n_queries,
        ranges.begin(),
        nvbio::plain_view( qgram_index ) );

    cudaDeviceSynchronize();
    timer.stop();
    const float unsorted_time = timer.seconds();

    timer.start();

    // and now repeat the same operation with the sorted q-grams
    thrust::transform(
        sorted_qgrams.begin(),
        sorted_qgrams.begin() + n_queries,
        ranges.begin(),
        nvbio::plain_view( qgram_index ) );

    cudaDeviceSynchronize();
    timer.stop();
    const float sorted_time = timer.seconds();

    const uint32 n_occurrences = thrust::reduce(
        thrust::make_transform_iterator( ranges.begin(), range_size() ),
        thrust::make_transform_iterator( ranges.begin(), range_size() ) + n_queries );

    const uint32 n_matches = thrust::reduce(
        thrust::make_transform_iterator( ranges.begin(), valid_range() ),
        thrust::make_transform_iterator( ranges.begin(), valid_range() ) + n_queries );

    stats.queries       += n_queries;
    stats.unsorted_time += unsorted_time;
    stats.sorted_time   += sorted_time;
    stats.matches       += n_matches;
    stats.occurrences   += n_occurrences;

    log_verbose(stderr, "  querying q-gram index... done\n");
    log_verbose(stderr, "    unsorted throughput : %.2f B q-grams/s\n", (1.0e-9f * float( stats.queries )) / stats.unsorted_time);
    log_verbose(stderr, "    sorted   throughput : %.2f B q-grams/s\n", (1.0e-9f * float( stats.queries )) / stats.sorted_time);
    log_verbose(stderr, "    matches             : %.2f M\n", 1.0e-6f * float( stats.matches ) );
    log_verbose(stderr, "    occurrences         : %.3f B\n", 1.0e-9f * float( stats.occurrences ) );

    log_verbose(stderr, "  q-gram filter... started\n");

    //
    // search the sorted query q-grams with a q-gram filter
    //

    const uint32 batch_size = 16*1024*1024;

    typedef QGramFilter<system_tag,qgram_index_type,const uint64*,const uint32*> qgram_filter_type;

    typedef typename qgram_filter_type::hit_type        hit_type;
    typedef typename qgram_filter_type::diagonal_type   diagonal_type;

    // prepare storage for the output hits
    nvbio::vector<system_tag,hit_type>      hits( batch_size );
    nvbio::vector<system_tag,diagonal_type> merged_hits( batch_size );
    nvbio::vector<system_tag,uint16>        merged_counts( batch_size );

    qgram_filter_type qgram_filter;

    timer.start();

    // first step: rank the query q-grams
    const uint32 n_hits = qgram_filter.rank(
        qgram_index,
        n_queries,
        nvbio::raw_pointer( sorted_qgrams ),
        nvbio::raw_pointer( sorted_indices ) );

    if (n_hits != n_occurrences)
    {
        log_error(stderr, "  mismatching number of hits: expected %u, got %u\n", n_occurrences, n_hits);
        exit(1);
    }

    // loop through large batches of hits and locate them
    for (uint32 hits_begin = 0; hits_begin < n_hits; hits_begin += batch_size)
    {
        const uint32 hits_end = nvbio::min( hits_begin + batch_size, n_hits );

        qgram_filter.locate(
            hits_begin,
            hits_end,
            hits.begin() );
    }

    cudaDeviceSynchronize();
    timer.stop();
    const float filter_time = timer.seconds();
    stats.filter_time += filter_time;

    timer.start();

    // loop through large batches of hits and locate & merge them
    for (uint32 hits_begin = 0; hits_begin < n_hits; hits_begin += batch_size)
    {
        const uint32 hits_end = nvbio::min( hits_begin + batch_size, n_hits );

        qgram_filter.locate(
            hits_begin,
            hits_end,
            hits.begin() );

        const uint32 n_merged = qgram_filter.merge(
            16u,
            hits_end - hits_begin,
            hits.begin(),
            merged_hits.begin(),
            merged_counts.begin() );

        stats.merged += n_merged;
    }

    cudaDeviceSynchronize();
    timer.stop();
    const float merge_time = timer.seconds();
    stats.merge_time += merge_time;

    log_verbose(stderr, "  q-gram filter... done\n");
    log_verbose(stderr, "    filter throughput  : %.2f M q-grams/s\n", (1.0e-6f * float( stats.queries )) / stats.filter_time);
    log_verbose(stderr, "    merge  throughput  : %.2f M q-grams/s\n", (1.0e-6f * float( stats.queries )) / stats.merge_time);
    log_verbose(stderr, "    merged occurrences : %.3f B (%.1f %%)\n", 1.0e-9f * float( stats.merged ), 100.0f * float(stats.merged)/float(stats.occurrences));
}

enum QGramTest
{
    ALL                 = 0xFFFFFFFFu,
    QGRAM_INDEX         = 1u,
    QGRAM_SET_INDEX     = 2u,
    QGROUP_INDEX        = 4u,
};

// main test entry point
//
int qgram_test(int argc, char* argv[])
{
    uint32 TEST_MASK     = 0xFFFFFFFFu;
    uint32 n_qgrams      = 10000000;
    uint32 n_queries     = 10000000;
    uint32 queries_batch = 10000000;
    bool   device_test   = true;
    bool   host_test     = true;
    const char* reads = "./data/SRR493095_1.fastq.gz";
    const char* index = "./data/human.NCBI36/Homo_sapiens.NCBI36.53.dna.toplevel.fa";

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp( argv[i], "-qgrams" ) == 0)
            n_qgrams = uint32( atoi( argv[++i] ) )*1000u;
        else if (strcmp( argv[i], "-queries" ) == 0)
            n_queries = uint32( atoi( argv[++i] ) )*1000u;
        else if (strcmp( argv[i], "-batch" ) == 0)
            queries_batch = uint32( atoi( argv[++i] ) )*1000u;
        else if (strcmp( argv[i], "-reads" ) == 0)
            reads = argv[++i];
        else if (strcmp( argv[i], "-index" ) == 0)
            index = argv[++i];
        else if (strcmp( argv[i], "-no-device" ) == 0)
            device_test = false;
        else if (strcmp( argv[i], "-no-host" ) == 0)
            host_test = false;
        else if (strcmp( argv[i], "-tests" ) == 0)
        {
            const std::string tests_string( argv[++i] );

            char temp[256];
            const char* begin = tests_string.c_str();
            const char* end   = begin;

            TEST_MASK = 0u;

            while (1)
            {
                while (*end != ':' && *end != '\0')
                {
                    temp[end - begin] = *end;
                    end++;
                }

                temp[end - begin] = '\0';

                if (strcmp( temp, "qgram" ) == 0)
                    TEST_MASK |= QGRAM_INDEX;
                else if (strcmp( temp, "qgram-set" ) == 0)
                    TEST_MASK |= QGRAM_SET_INDEX;
                else if (strcmp( temp, "qgroup" ) == 0)
                    TEST_MASK |= QGROUP_INDEX;

                if (*end == '\0')
                    break;

                ++end; begin = end;
            }
        }
    }

  #if defined(_OPENMP)
    // Now set the number of threads
    omp_set_num_threads( omp_get_num_procs() );
  #endif

    log_info(stderr, "q-gram test... started\n");

    const io::QualityEncoding qencoding = io::Phred33;

    log_info(stderr, "  loading reads... started\n");

    SharedPointer<io::SequenceDataStream> read_data_file(
        io::open_sequence_file(
            reads,
            qencoding,
            uint32(-1),
            uint32(-1) ) );

    if (read_data_file == NULL || read_data_file->is_ok() == false)
    {
        log_error(stderr, "    failed opening file \"%s\"\n", reads);
        return 1u;
    }

    const uint32 batch_size = uint32(-1);
    const uint32 batch_bps  = n_qgrams;

    // load a batch of reads
    io::SequenceDataHost h_read_data;

    if (io::next( DNA_N, &h_read_data, read_data_file.get(), batch_size, batch_bps ) == 0)
    {
        log_error(stderr, "  unable to read input sequences\n");
        return 1;
    }
    
    // build its device version
    const io::SequenceDataDevice d_read_data( h_read_data );
    const io::SequenceDataAccess<DNA_N> d_read_access( d_read_data );

    log_info(stderr, "  loading reads... done\n");

    // fetch the actual string
    typedef io::SequenceDataAccess<DNA_N> read_access_type;

    typedef read_access_type::sequence_stream_type        string_type;
    typedef read_access_type::sequence_string_set_type    string_set_type;

    const uint32          n_strings      = d_read_access.size();
    const uint32          string_len     = d_read_access.bps();
    const string_type     string         = d_read_access.sequence_stream();
    const string_set_type string_set     = d_read_access.sequence_string_set();

    log_info(stderr, "    strings: %u\n", n_strings);
    log_info(stderr, "    symbols: %.3f M\n", 1.0e-6f * float(string_len));

    io::SequenceDataHost ref;
    if (!io::load_sequence_file( DNA, &ref, index ))
    {
        log_error(stderr, "    failed loading index \"%s\"\n", index);
        return 1u;
    }

    // build its device version
    const io::SequenceDataDevice ref_cuda( ref );

    typedef io::SequenceDataAccess<DNA>                       genome_access_type;
    typedef genome_access_type::sequence_stream_type          genome_type;

    const uint32                genome_len = ref.bps();

    const genome_access_type    h_genome_access( ref );
    const genome_type           h_genome( h_genome_access.sequence_stream() );

    const genome_access_type    d_genome_access( ref_cuda );
    const genome_type           d_genome( d_genome_access.sequence_stream() );

    // clamp the total number of queries
    n_queries = nvbio::min( n_queries, genome_len );

    // test q-gram index
    if (TEST_MASK & QGRAM_INDEX)
    {
        log_visible(stderr, "  testing q-gram index (device)... started\n");

        QGramIndexDevice qgram_index;

        test_qgram_index_build(
            20u,
            string_len,
            string,
            qgram_index );

        if (device_test)
        {
            Stats stats;

            for (uint32 genome_begin = 0; genome_begin < n_queries; genome_begin += queries_batch)
            {
                const uint32 genome_end = nvbio::min( genome_begin + queries_batch, n_queries );

                test_qgram_index_query(
                    qgram_index,
                    genome_end - genome_begin,
                    genome_len,
                    genome_begin,
                    d_genome,
                    stats );
            }

            log_visible(stderr, "  testing q-gram index (device)... done\n");
            const float genome_ratio = float(genome_len)/float(stats.queries);
            log_info(stderr, "    sorted throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.sorted_time * genome_ratio) );
            log_info(stderr, "    sorted throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.sorted_time * genome_ratio) );
            log_info(stderr, "    filter throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.filter_time * genome_ratio) );
            log_info(stderr, "    filter throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.filter_time * genome_ratio) );
            log_info(stderr, "    merge  throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.merge_time  * genome_ratio) );
            log_info(stderr, "    merge  throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.merge_time  * genome_ratio) );
        }
        if (host_test)
        {
            log_visible(stderr, "  testing q-gram index (host)... started\n");
            QGramIndexHost h_qgram_index;
        
            h_qgram_index = qgram_index;

            Stats stats;

            for (uint32 genome_begin = 0; genome_begin < n_queries; genome_begin += queries_batch)
            {
                const uint32 genome_end = nvbio::min( genome_begin + queries_batch, n_queries );

                test_qgram_index_query(
                    h_qgram_index,
                    genome_end - genome_begin,
                    genome_len,
                    genome_begin,
                    h_genome,
                    stats );
            }
            log_visible(stderr, "  testing q-gram index (host)... done\n");
            const float genome_ratio = float(genome_len)/float(stats.queries);
            log_info(stderr, "    sorted throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.sorted_time * genome_ratio) );
            log_info(stderr, "    sorted throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.sorted_time * genome_ratio) );
            log_info(stderr, "    filter throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.filter_time * genome_ratio) );
            log_info(stderr, "    filter throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.filter_time * genome_ratio) );
            log_info(stderr, "    merge  throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.merge_time  * genome_ratio) );
            log_info(stderr, "    merge  throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.merge_time  * genome_ratio) );
        }
    }

    // test q-gram set-index
    if (TEST_MASK & QGRAM_SET_INDEX)
    {
        log_visible(stderr, "  testing q-gram set-index (device)... started\n");

        QGramSetIndexDevice qgram_index;

        test_qgram_set_index_build(
            22u,
            string_set,
            qgram_index );

        if (device_test)
        {
            Stats stats;

            for (uint32 genome_begin = 0; genome_begin < n_queries; genome_begin += queries_batch)
            {
                const uint32 genome_end = nvbio::min( genome_begin + queries_batch, n_queries );

                test_qgram_index_query(
                    qgram_index,
                    genome_end - genome_begin,
                    genome_len,
                    genome_begin,
                    d_genome,
                    stats );
            }

            log_visible(stderr, "  testing q-gram set-index (device)... done\n");
            const float genome_ratio = float(genome_len)/float(stats.queries);
            log_info(stderr, "    sorted throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.sorted_time * genome_ratio) );
            log_info(stderr, "    sorted throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.sorted_time * genome_ratio) );
            log_info(stderr, "    filter throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.filter_time * genome_ratio) );
            log_info(stderr, "    filter throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.filter_time * genome_ratio) );
            log_info(stderr, "    merge  throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.merge_time  * genome_ratio) );
            log_info(stderr, "    merge  throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.merge_time  * genome_ratio) );
        }
        if (host_test)
        {
            log_visible(stderr, "  testing q-gram set-index (host)... started\n");
            QGramSetIndexHost h_qgram_index;
        
            h_qgram_index = qgram_index;

            Stats stats;

            for (uint32 genome_begin = 0; genome_begin < n_queries; genome_begin += queries_batch)
            {
                const uint32 genome_end = nvbio::min( genome_begin + queries_batch, n_queries );

                test_qgram_index_query(
                    h_qgram_index,
                    genome_end - genome_begin,
                    genome_len,
                    genome_begin,
                    h_genome,
                    stats );
            }
            log_visible(stderr, "  testing q-gram set-index (host)... done\n");
            const float genome_ratio = float(genome_len)/float(stats.queries);
            log_info(stderr, "    sorted throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.sorted_time * genome_ratio) );
            log_info(stderr, "    sorted throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.sorted_time * genome_ratio) );
            log_info(stderr, "    filter throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.filter_time * genome_ratio) );
            log_info(stderr, "    filter throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.filter_time * genome_ratio) );
            log_info(stderr, "    merge  throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.merge_time  * genome_ratio) );
            log_info(stderr, "    merge  throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.merge_time  * genome_ratio) );
        }
    }

    // test q-group index
    if (TEST_MASK & QGROUP_INDEX)
    {
        log_visible(stderr, "  testing q-group index (device)... started\n");

        QGroupIndexDevice qgram_index;

        test_qgroup_index_build(
            16u,
            string_len,
            string,
            qgram_index );

        if (device_test)
        {
            Stats stats;

            for (uint32 genome_begin = 0; genome_begin < n_queries; genome_begin += queries_batch)
            {
                const uint32 genome_end = nvbio::min( genome_begin + queries_batch, n_queries );

                test_qgram_index_query(
                    qgram_index,
                    genome_end - genome_begin,
                    genome_len,
                    genome_begin,
                    d_genome,
                    stats );
            }

            log_visible(stderr, "  testing q-group index (device)... done\n");
            const float genome_ratio = float(genome_len)/float(stats.queries);
            log_info(stderr, "    sorted throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.sorted_time * genome_ratio) );
            log_info(stderr, "    sorted throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.sorted_time * genome_ratio) );
            log_info(stderr, "    filter throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.filter_time * genome_ratio) );
            log_info(stderr, "    filter throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.filter_time * genome_ratio) );
            log_info(stderr, "    merge  throughput: %7.2f K reads/s\n", 1.0e-3f * float(n_strings)  / (stats.merge_time  * genome_ratio) );
            log_info(stderr, "    merge  throughput: %7.2f M bases/s\n", 1.0e-6f * float(string_len) / (stats.merge_time  * genome_ratio) );
        }
    }

    log_info(stderr, "q-gram test... done\n" );
    return 0;
}

} // namespace nvbio
