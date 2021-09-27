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
#include <map>
#include <vector>
#include <stack>

namespace nvbio {

///
/// LRU cache.
/// The template parameter CacheManager should supply the following interface:
///
/// bool acquire(const uint32 item);
///     try to acquire/load an element, returning false in case of failure:
///     the latter will trigger a cache release cycle.
///
/// void release(const uint32 item);
///     release an element, freeing any of the relative resources.
///
/// bool low_watermark() const
///     return true when the cache usage is below the low-watermark
///
struct cache_overflow {};

template <typename CacheManager>
struct LRU
{
    typedef CacheManager cache_manager_type;

    /// cache constructor
    ///
    LRU(CacheManager& manager);

    /// pin a given element, marking it as non-releasable
    ///
    void pin(const uint32 item);

    /// unpin a given element, marking it as releasable
    ///
    void unpin(const uint32 item);

private:
    struct List
    {
        List() {}
        List(const uint32 item, const uint32 next, const uint32 prev) :
            m_item( item ), m_next( next ), m_prev( prev ), m_pinned(true) {}

        uint32 m_item;
        uint32 m_next;
        uint32 m_prev;
        bool   m_pinned;
    };

    void touch(const uint32 list_idx, List& list);
    void release_cycle(const uint32 item);

    uint32 m_first;
    uint32 m_last;

    CacheManager*           m_manager;
    std::map<uint32,uint32> m_cache_map;
    std::vector<List>       m_cache_list;
    std::stack<uint32>      m_cache_pool;
};

} // namespace nvbio

#include <nvbio/basic/cache_inl.h>
