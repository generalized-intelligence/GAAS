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

// sum_tree_test.cpp
//

#include <nvbio/basic/sum_tree.h>
#include <nvbio/basic/console.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

namespace nvbio {

int sum_tree_test()
{
    printf("sum tree... started\n");
    // 128 leaves
    {
        const uint32 n_leaves = 128;

        std::vector<int32> vec( SumTree<uint32*>::node_count( n_leaves ) );

        SumTree<int32*> sum_tree( n_leaves, &vec[0] );

        // test 1
        for (uint32 i = 0; i < n_leaves; ++i)
            vec[i] = i;
        {
            sum_tree.setup();

            uint32 c0 = sample( sum_tree, 0.0f );
            uint32 c1 = sample( sum_tree, 0.5f );
            uint32 c2 = sample( sum_tree, 1.0f );

            if (c0 != 1 || c1 != 90 || c2 != 127)
            {
                log_error( stderr, "error in test(1):\n  c(0.0) = %u (!= 1)\n  c(0.5) = %u (!= 90)  c(1.0) = %u (!= 127)\n", c0, c1, c2 );
                exit(1);
            }
        }

        // test 2
        for (uint32 i = 0; i < n_leaves; ++i)
            vec[i] = i < n_leaves/2 ? 0 : 1;
        {
            sum_tree.setup();

            uint32 c0 = sample( sum_tree, 0.0f );
            uint32 c1 = sample( sum_tree, 0.5f );
            uint32 c2 = sample( sum_tree, 1.0f );

            if (c0 != 64 || c1 != 96 || c2 != 127)
            {
                log_error( stderr, "error in test(2):\n  c(0.0) = %u (!= 64)\n  c(0.5) = %u (!= 90)  c(1.0) = %u (!= 127)\n", c0, c1, c2 );
                return 1;
            }
        }

        // test 3
        for (uint32 i = 0; i < n_leaves; ++i)
            vec[i] = i < n_leaves/2 ? 1 : 0;
        {
            sum_tree.setup();

            uint32 c0 = sample( sum_tree, 0.0f );
            uint32 c1 = sample( sum_tree, 0.5f );
            uint32 c2 = sample( sum_tree, 1.0f );

            if (c0 != 0 || c1 != 32 || c2 != 63)
            {
                log_error( stderr, "error in test(3):\n  c(0.0) = %u (!= 0)\n  c(0.5) = %u (!= 32)  c(1.0) = %u (!= 63)\n", c0, c1, c2 );
                exit(1);
            }
        }
    }
    // 80 leaves
    {
        const uint32 n_leaves = 80;

        std::vector<int32> vec( SumTree<uint32*>::node_count( n_leaves ) );

        SumTree<int32*> sum_tree( n_leaves, &vec[0] );

        if (sum_tree.padded_size() != 128)
        {
            log_error( stderr, "error: wrong padded size: %u != 128\n", sum_tree.padded_size() );
            exit(1);
        }

        // test 4
        for (uint32 i = 0; i < n_leaves; ++i)
            vec[i] = i < n_leaves/2 ? 0 : 1;
        {
            sum_tree.setup();

            uint32 c0 = sample( sum_tree, 0.0f );
            uint32 c1 = sample( sum_tree, 0.5f );
            uint32 c2 = sample( sum_tree, 1.0f );

            if (c0 != 40 || c1 != 60 || c2 != 79)
            {
                log_error( stderr, "error in test(3):\n  c(0.0) = %u (!= 40)\n  c(0.5) = %u (!= 60)  c(1.0) = %u (!= 79)\n", c0, c1, c2 );
                exit(1);
            }
        }

        // test 5
        for (uint32 i = 0; i < n_leaves; ++i)
            vec[i] = i < n_leaves/2 ? 1 : 0;
        {
            sum_tree.setup();

            uint32 c0 = sample( sum_tree, 0.0f );
            uint32 c1 = sample( sum_tree, 0.5f );
            uint32 c2 = sample( sum_tree, 1.0f );

            if (c0 != 0 || c1 != 20 || c2 != 39)
            {
                log_error( stderr, "error in test(5):\n  c(0.0) = %u (!= 0)\n  c(0.5) = %u (!= 20)  c(1.0) = %u (!= 39)\n", c0, c1, c2 );
                exit(1);
            }
        }

        // remove the last leaf
        sum_tree.add( 39, -1 );

        // test 6
        uint32 c = sample( sum_tree, 1.0f );
        if (c != 38)
        {
            log_error( stderr, "error in test(6):\n  c(1.0) = %u (!= 38)\n", c );
            exit(1);
        }

        // remove one more leaf
        sum_tree.add( 38, -1 );

        c = sample( sum_tree, 1.0f );
        if (c != 37)
        {
            log_error( stderr, "error in test(7):\n  c(1.0) = %u (!= 37)\n", c );
            exit(1);
        }

        // add it back
        sum_tree.add( 38, 1 );

        // and remove it using set
        sum_tree.set( 38, 0 );

        c = sample( sum_tree, 1.0f );
        if (c != 37)
        {
            log_error( stderr, "error in test(8):\n  c(1.0) = %u (!= 37)\n", c );
            exit(1);
        }

        // remove the first ten leaves
        for (uint32 i = 0; i < 10; ++i)
        {
            sum_tree.set( i, 0 );

            c = sample( sum_tree, 0.0f );
            if (c != i+1)
            {
                log_error( stderr, "error in test(9):\n  c(0.0) = %u (!= %u)\n", c, i );
                exit(1);
            }
        }
    }
    printf("sum tree... done\n");

    return 0;
}

}