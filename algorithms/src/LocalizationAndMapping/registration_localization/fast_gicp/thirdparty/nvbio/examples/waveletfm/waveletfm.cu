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

// waveletfm.cu
//
// Protein Search using a Wavelet Tree and an FM-index
//

#include <stdio.h>
#include <stdlib.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/vector.h>
#include <nvbio/basic/packed_vector.h>
#include <nvbio/strings/alphabet.h>
#include <nvbio/strings/wavelet_tree.h>
#include <nvbio/sufsort/sufsort.h>
#include <nvbio/fmindex/fmindex.h>

using namespace nvbio;

// main test entry point
//
int main(int argc, char* argv[])
{
    log_info(stderr, "waveletfm... started\n");

    const char* proteins = "ACDEFGHIKLMNOPQRSTVWYBZX";
    const uint32 text_len = 24;
    const uint32 alphabet_bits = AlphabetTraits<PROTEIN>::SYMBOL_SIZE;
    const uint32 alphabet_size = 1u << alphabet_bits;

    // print the text
    log_info(stderr, "  text: %s\n", proteins);

    // allocate a host packed vector
    PackedVector<host_tag,alphabet_bits,true> h_text( text_len );

    // pack the string
    from_string<PROTEIN>( proteins, proteins + text_len, h_text.begin() );

    // copy it to the device
    PackedVector<device_tag,alphabet_bits,true> d_text( h_text );

    // allocate a vector for the BWT
    PackedVector<device_tag,alphabet_bits,true> d_bwt( text_len + 1 );

    BWTParams bwt_params;

    // build the BWT
    const uint32 primary = cuda::bwt( text_len, d_text.begin(), d_bwt.begin(), &bwt_params );

    // print the BWT
    {
        char proteins_bwt[ 25 ];

        to_string<PROTEIN>( d_bwt.begin(), d_bwt.begin() + text_len, proteins_bwt );

        log_info(stderr, "  bwt: %s (primary %u)\n", proteins_bwt, primary);
    }

    // define our wavelet tree storage type and its plain view
    typedef WaveletTreeStorage<device_tag>                          wavelet_tree_type;
    typedef WaveletTreeStorage<device_tag>::const_plain_view_type   wavelet_tree_view_type;

    // build a wavelet tree
    wavelet_tree_type wavelet_bwt;

    // setup the wavelet tree
    setup( text_len, d_bwt.begin(), wavelet_bwt );

    typedef nvbio::vector<device_tag,uint32>::const_iterator            l2_iterator;
    typedef fm_index<wavelet_tree_view_type, null_type, l2_iterator>    fm_index_type;

    // take the const view of the wavelet tree
    const wavelet_tree_view_type wavelet_bwt_view = plain_view( (const wavelet_tree_type&)wavelet_bwt );

    // build the L2 vector
    nvbio::vector<device_tag,uint32> L2(alphabet_size+1);

    // note we perform this on the host, even though the wavelet tree is on the device -
    // gonna be slow, but it's not much stuff, and this is just an example anyway...
    // and we just want to show that NVBIO is designed to make everything work!
    L2[0] = 0;
    for (uint32 i = 1; i <= alphabet_size; ++i)
        L2[i] = L2[i-1] + rank( wavelet_bwt_view, text_len, i-1u );

    // build the FM-index
    const fm_index_type fmi(
        text_len,
        primary,
        L2.begin(),
        wavelet_bwt_view,
        null_type() );

    // do some string matching using our newly built FM-index - once again
    // we are doing it on the host, though all data is on the device: the entire
    // loop would be better moved to the device in a real app.
    log_info(stderr, "  string matching:\n");
    for (uint32 i = 0; i < text_len; ++i)
    {
        // match the i-th suffix of the text
        const uint32 pattern_len = text_len - i;

        // compute the SA range containing the occurrences of the pattern we are after
        const uint2 range = match( fmi, d_text.begin() + i, pattern_len );

        // print the number of occurrences of our pattern, equal to the SA range size
        log_info(stderr, "    rank(%s): %u\n", proteins + i, 1u + range.y - range.x);
    }

    log_info(stderr, "waveletfm... done\n");
    return 0;
}
