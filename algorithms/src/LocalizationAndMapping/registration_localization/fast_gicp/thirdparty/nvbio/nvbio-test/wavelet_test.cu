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

// wavelet_test.cu
//

#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/strings/wavelet_tree.h>
#include <stdio.h>
#include <stdlib.h>

namespace nvbio {

template <typename BitStreamIterator, typename IndexIterator>
struct text_functor
{
    typedef uint32 argument_type;
    typedef uint8  result_type;

    // constructor
    NVBIO_HOST_DEVICE
    text_functor(
        WaveletTree<BitStreamIterator,IndexIterator> _tree) : tree(_tree) {}

    // unary operator
    NVBIO_HOST_DEVICE
    uint8 operator() (const uint32 i) const { return text( tree, i ); }

    WaveletTree<BitStreamIterator,IndexIterator> tree;
};

template <typename BitStreamIterator, typename IndexIterator>
text_functor<BitStreamIterator,IndexIterator> make_text_functor(WaveletTree<BitStreamIterator,IndexIterator> _tree)
{
    return text_functor<BitStreamIterator,IndexIterator>( _tree );
}

int wavelet_test(int argc, char* argv[])
{
    uint32 text_len = 100000;

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp( argv[i], "-length" ) == 0)
            text_len = atoi( argv[++i] )*1000;
    }

    try
    {
        log_info(stderr, "wavelet test... started\n");

        nvbio::vector<host_tag,uint8> h_text( text_len );

        h_text[0] = 41;
        h_text[1] = 59;
        h_text[2] = 59;
        h_text[3] = 41;
        h_text[4] = 59;

        for (uint32 i = 5; i < text_len; ++i)
            h_text[i] = (rand() & 255);

        nvbio::vector<device_tag,uint8> d_text( h_text );

        typedef WaveletTreeStorage<device_tag>  wavelet_tree_type;

        // build a wavelet tree
        wavelet_tree_type wavelet_tree;

        // setup the wavelet tree
        setup( text_len, d_text.begin(), wavelet_tree );

        nvbio::vector<host_tag,uint32> occ( wavelet_tree.m_occ );

        // extract the text
        nvbio::transform<device_tag>(
            text_len,
            thrust::make_counting_iterator<uint32>(0),
            d_text.begin(),
            make_text_functor( plain_view(wavelet_tree) ) );

        // and copy it back to the host
        nvbio::vector<host_tag,uint8> h_extracted_text( d_text );

        for (uint32 i = 0; i < text_len; ++i)
        {
            const uint32 c = h_extracted_text[i];
            const uint32 r = h_text[i];

            if (c != r)
            {
                log_error(stderr, "error in text(%u): expected %u, got %u!\n", i, r, c);
                return 1;
            }
        }

        // do a quick ranking test against a bunch of known results
        const uint32 p[7] = { 0,   1,  2,  2,  3,  4,  5 };
        const uint32 c[7] = { 41, 59, 59, 41, 41, 59,  0 };
        const uint32 r[7] = { 1,   1,  2,  1,  2,  3,  0 };

        for (uint32 i = 0; i < 7; ++i)
        {
            const uint32 n = rank( plain_view(wavelet_tree), p[i], c[i] );
            if (n != r[i])
            {
                log_error(stderr, "error in rank(%u,%u): expected %u, got %u!\n", p[i], c[i], r[i], n);
                return 1;
            }
        }

        log_info(stderr, "wavelet test... done\n");
    }
    catch (...)
    {
        log_error(stderr, "error: unknown exception caught!\n");
    }

    return 0;
}

} // namespace nvbio
