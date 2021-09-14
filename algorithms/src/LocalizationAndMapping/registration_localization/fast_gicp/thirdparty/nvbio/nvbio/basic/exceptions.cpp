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

/*! \file cached_iterator.h
 *   \brief CUDA-compatible iterator wrappers allowing to cache the dereferenced
 *   value of generic iterators
 */

#include <nvbio/basic/exceptions.h>
#include <string.h>
#include <stdio.h>

#if WIN32
#include <windows.h>
#else
#include <stdarg.h>
#endif

namespace nvbio
{

char cuda_error::s_error[4096];
char bad_alloc::s_error[4096];
char runtime_error::s_error[4096];
char logic_error::s_error[4096];

cuda_error::cuda_error(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsprintf(s_error, format, args);
    va_end(args);
}

bad_alloc::bad_alloc(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsprintf(s_error, format, args);
    va_end(args);
}

runtime_error::runtime_error(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsprintf(s_error, format, args);
    va_end(args);
}

logic_error::logic_error(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsprintf(s_error, format, args);
    va_end(args);
}

} // namespace nvbio
