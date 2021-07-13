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

// packedstream_test.cpp
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/cache.h>

using namespace nvbio;

struct CacheManager
{
    // constructor
    CacheManager() : m_size(0) {}

    // acquire element i
    bool acquire(const uint32 i)
    {
        if (m_size >= 100u)
            return false;

        ++m_size;
        return true;
    }

    // release element i
    void release(const uint32 i)
    {
        --m_size;
    }

    // is cache usage below the low-watermark?
    bool low_watermark() const
    {
        return (m_size < 75u);
    }

    uint32 m_size;
};

int cache_test()
{
    printf("cache test... started\n");
    CacheManager manager;

    LRU<CacheManager> cache( manager );
    for (uint32 i = 0; i < 1000; ++i)
    {
        cache.pin(i);
        cache.unpin(i);
    }

    printf("  test overflow... started\n");
    bool overflow = false;

    try
    {
        for (uint32 i = 0; i < 200; ++i)
            cache.pin(i);
    }
    catch (cache_overflow)
    {
        overflow = true;
    }
    if (overflow == false)
        printf("  error: overflow was expected, but did not occurr!\n");
    else
        printf("  test overflow... done\n");
    printf("cache test... done\n");
    return 0u;
}