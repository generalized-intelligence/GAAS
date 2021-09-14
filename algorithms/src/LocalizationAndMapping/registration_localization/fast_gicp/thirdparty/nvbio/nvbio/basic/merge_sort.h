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

#pragma once

#include <nvbio/basic/types.h>
#include <iterator>

namespace nvbio {

namespace mergesort {

/// merge two contiguous sorted sequences into a single sorted output sequence.
/// NOTE: the output buffer should not overlap the input buffer.
///
/// \param A            input buffer
/// \param int_left     beginning of the first interval
/// \param int_right    beginning of the second interval
/// \param int_end      end of the second interval
/// \param B            output buffer
///
template <typename Iterator, typename Compare>
NVBIO_HOST_DEVICE
void merge(Iterator A, uint32 int_left, uint32 int_right, uint32 int_end, Iterator B, const Compare cmp)
{
    uint32 i0 = int_left;
    uint32 i1 = int_right;

    // while there are elements in the left or right lists
    for (uint32 j = int_left; j < int_end; ++j)
    {
        // if left list head exists and is <= existing right list head
        if (i0 < int_right && (i1 >= int_end || (!(cmp( A[i1], A[i0] )))))
        {
            B[j] = A[i0];
            i0 = i0 + 1;
        }
        else
        {
            B[j] = A[i1];
            i1 = i1 + 1;
        }
    }
}

} // namespace merge_sort

///
/// Merge sort
///
/// \param n        number of entries
/// \param A        array to sort
/// \param B        temporary buffer
///
/// \return         true if the results are in B, false otherwise
///
template <typename Iterator, typename Compare>
NVBIO_HOST_DEVICE
bool merge_sort(uint32 n, Iterator A, Iterator B, const Compare cmp)
{
    if (n == 1)
        return false;

    // each 1-element run in A is already "sorted":
    // make successively longer sorted runs of length 2, 4, 8, 16...
    // until whole array is sorted

    // sort pairs in place to avoid unnecessary memory traffic
    const uint32 nn = (n & 1) ? n-1 : n;
    for (uint32 i = 0; i < nn; i += 2u)
    {
        if (cmp( A[i+1], A[i] ))
        {
            typename std::iterator_traits<Iterator>::value_type tmp = A[i];
            A[i]   = A[i+1];
            A[i+1] = tmp;
        }
    }

    // merge longer and longer runs in a loop
    bool swap = 0;
    for (uint32 width = 2u; width < n; width *= 2u, swap = !swap)
    {
        // array A is full of runs of length width
        for (uint32 i = 0; i < n; i += 2u * width)
        {
            // merge two runs: A[i:i+width-1] and A[i+width:i+2*width-1] to B[]
            //  or copy A[i:n-1] to B[] ( if(i+width >= n) )
            mergesort::merge( A, i, nvbio::min(i+width, n), nvbio::min(i+2*width, n), B, cmp );
        }

        // now work array B is full of runs of length 2*width, swap A and B.
        Iterator tmp = A;
        A = B;
        B = tmp;
    }
    return swap;
}

} // namespace nvbio
