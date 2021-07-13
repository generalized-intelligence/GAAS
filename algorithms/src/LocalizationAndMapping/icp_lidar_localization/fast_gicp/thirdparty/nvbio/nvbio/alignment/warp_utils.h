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

namespace nvbio {
namespace aln {
namespace priv {

// helper structure to hold a score + sink and allow for comparisons during a warp-scan
template<typename score_type>
struct alignment_result
{
    score_type score;
    uint2 sink;

    __device__ alignment_result(score_type score, uint2 sink)
    {
        this->score = score;
        this->sink = sink;
    };

    __device__ alignment_result()
    {
    };

    __device__ alignment_result(const volatile alignment_result<score_type>& other)
    {
        *this = other;
    };

    __device__ alignment_result(const alignment_result<score_type>& other)
    {
        *this = other;
    };

    __device__ alignment_result<score_type>& operator=(const alignment_result<score_type>& other)
    {
        score = other.score;
        sink.x = other.sink.x;
        sink.y = other.sink.y;
        return *this;
    };

    __device__ volatile alignment_result<score_type>& operator=(const volatile alignment_result<score_type>& other) volatile
    {
        score = other.score;
        sink.x = other.sink.x;
        sink.y = other.sink.y;
        return *this;
    };

    __device__ static alignment_result<score_type> minimum_value()
    {
        alignment_result<score_type> ret;
        ret.score = Field_traits<score_type>::min();
        return ret;
    };

    // comparison functor
    struct max_operator
    {
        __device__ const volatile alignment_result<score_type>& operator() (const alignment_result<score_type>& s1,
                                                                            const volatile alignment_result<score_type>& s2) const
        {
            if (s1.score > s2.score)
            {
                return s1;
            } else {
                return s2;
            }
        }
    };
};

} // namespace priv
} // namespace aln
} // namespace nvbio
