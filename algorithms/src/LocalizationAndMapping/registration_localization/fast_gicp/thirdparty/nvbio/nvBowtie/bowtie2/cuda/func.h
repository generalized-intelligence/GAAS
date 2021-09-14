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
namespace bowtie2 {
namespace cuda {

///@addtogroup nvBowtie
///@{

struct SimpleFunc
{
    enum Type { LinearFunc = 0, LogFunc = 1, SqrtFunc = 2 };

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    SimpleFunc(const Type _type = LinearFunc, const float _k = 0.0f, const float _m = 1.0f) : type(_type), k(_k), m(_m) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    int32 operator() (const int32 x) const
    {
        return int32( k + m * (type == LogFunc  ?  logf(float(x)) :
                               type == SqrtFunc ? sqrtf(float(x)) :
                                                        float(x)) );
    }

    const char* type_string() const
    {
        return type == LogFunc  ? "log" :
               type == SqrtFunc ? "sqrt" :
                                  "linear";
    }
    const char* type_symbol() const
    {
        return type == LogFunc  ? "G" :
               type == SqrtFunc ? "S" :
                                  "L";
    }

    Type  type;
    float k;
    float m;
};

///@}  // group nvBowtie

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
