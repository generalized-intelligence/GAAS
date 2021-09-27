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

// seeding.cu
//

#include <stdio.h>
#include <stdlib.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/shared_pointer.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/strings/infix.h>
#include <nvbio/strings/seeds.h>
#include <nvbio/io/sequence/sequence.h>

using namespace nvbio;

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

// main test entry point
//
int main(int argc, char* argv[])
{
    //
    // perform some basic option parsing
    //

    uint32      n_bps = 10000000;
    const char* reads = "./data/SRR493095_1.fastq.gz";

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp( argv[i], "-bps" ) == 0)
            n_bps = uint32( atoi( argv[++i] ) )*1000u;
        else if (strcmp( argv[i], "-reads" ) == 0)
            reads = argv[++i];
    }

    // start our program
    log_info(stderr, "seeding... started\n");

    // open a read file
    log_info(stderr, "  loading reads... started\n");

    SharedPointer<io::SequenceDataStream> read_data_file(
        io::open_sequence_file(
            reads,
            io::Phred33,
            uint32(-1),
            uint32(-1) ) );

    // check whether the file opened correctly
    if (read_data_file == NULL || read_data_file->is_ok() == false)
    {
        log_error(stderr, "    failed opening file \"%s\"\n", reads);
        return 1u;
    }

    const uint32 batch_size = uint32(-1);
    const uint32 batch_bps  = n_bps;

    // load a batch of reads
    io::SequenceDataHost h_read_data;

    io::next( DNA_N, &h_read_data, read_data_file.get(), batch_size, batch_bps );

    // copy it to the device
    const io::SequenceDataDevice d_read_data( h_read_data );

    log_info(stderr, "  loading reads... done\n");
    log_info(stderr, "    %u reads\n", d_read_data.size());

    // prepare some typedefs for the involved string-sets and infixes
    typedef io::SequenceDataAccess<DNA_N>                                   read_access_type;
    typedef read_access_type::sequence_string_set_type                      string_set_type;    // the read string-set
    typedef string_set_infix_coord_type                                     infix_coord_type;   // the infix coordinate type, for string-sets
    typedef nvbio::vector<device_tag,infix_coord_type>                      infix_vector_type;  // the device vector type for infix coordinates
    typedef InfixSet<string_set_type, const string_set_infix_coord_type*>   seed_set_type;      // the infix-set type for representing seeds

    // build a read accessor
    const read_access_type d_read_access( d_read_data );

    // fetch the actual read string-set
    const string_set_type d_read_string_set = d_read_access.sequence_string_set();

    // prepare enough storage for the seed coordinates
    infix_vector_type d_seed_coords;

    // extract the seeds and get the corresponding string-set representation
    const seed_set_type d_seed_set = extract_seeds(
        d_read_string_set,
        20u,
        10u,
        d_seed_coords );

    // output some stats
    log_info(stderr, "seeding... done\n");
    log_info(stderr, "  %u seeds\n", d_seed_set.size());

    return 0;
}
