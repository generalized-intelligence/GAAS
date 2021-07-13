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

// hello_world2.cu
//

#include <stdio.h>
#include <stdlib.h>
#include <nvbio/basic/packed_vector.h>
#include <nvbio/strings/alphabet.h>
#include <nvbio/strings/infix.h>
#include <nvbio/strings/seeds.h>


using namespace nvbio;

template <typename string_set_type>
struct print_strings
{
    typedef typename string_set_type::string_type string_type;

    NVBIO_HOST_DEVICE
    void operator() (const string_type& string) const
    {
        char seed[4];
        dna_to_string(
            string,
            length( string ),
            seed );

        printf("%s\n", seed);
    }
};

// main test entry point
//
int main(int argc, char* argv[])
{
    // our hello world ASCII string
    const char dna_string[] = "ACGTTGCA";
    const uint32 len = (uint32)strlen( dna_string );

    // our DNA alphabet size
    const uint32 ALPHABET_SIZE = AlphabetTraits<DNA>::SYMBOL_SIZE;

    // instantiate a packed host vector
    nvbio::PackedVector<host_tag,ALPHABET_SIZE> h_dna( len );

    // pack our ASCII string
    nvbio::assign( len, nvbio::from_string<DNA>( dna_string ), h_dna.begin() );

    // instantiate a packed host vector
    nvbio::PackedVector<device_tag,ALPHABET_SIZE> d_dna( h_dna );

    // prepare a vector to store the coordinates of the resulting infixes
    nvbio::vector<device_tag,string_infix_coord_type> d_seed_coords;

    const uint32 n_seeds = enumerate_string_seeds(
		len,                                    // the input string length
        uniform_seeds_functor<>( 3u, 1u ),      // a seeding functor, specifying to extract all 3-mers offset by 1 base each
        d_seed_coords );                        // the output infix coordinates

    // define an infix-set to represent the resulting infixes
    typedef nvbio::PackedVector<device_tag,ALPHABET_SIZE>::iterator             packed_iterator_type;
    typedef nvbio::vector<device_tag,string_infix_coord_type>::const_iterator   infix_iterator_type;
    typedef InfixSet<packed_iterator_type, infix_iterator_type> infix_set_type;

    const infix_set_type seeds(
        n_seeds,                                // the number of infixes in the set
        d_dna.begin(),                          // the underlying string
        d_seed_coords.begin() );                // the iterator to the infix coordinates

    thrust::for_each(
        seeds.begin(),
        seeds.end(),
        print_strings<infix_set_type>() );

    cudaDeviceSynchronize();

    return 0;
}
