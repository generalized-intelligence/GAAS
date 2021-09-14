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

namespace nvbio {

template <typename CacheManager>
LRU<CacheManager>::LRU(CacheManager& manager) : m_first(0xFFFFFFFFu), m_last(0xFFFFFFFFu), m_manager(&manager) {}

template <typename CacheManager>
void LRU<CacheManager>::pin(const uint32 item)
{
    typename std::map<uint32,uint32>::iterator it = m_cache_map.find( item );
    if (it == m_cache_map.end())
    {
        uint32 list_idx;
        if (m_cache_pool.size())
        {
            list_idx = m_cache_pool.top();
            m_cache_pool.pop();
        }
        else
        {
            list_idx = uint32( m_cache_list.size() );
            m_cache_list.push_back( List() );
        }

        m_cache_map.insert( std::make_pair( item, list_idx ) );

        List& list = m_cache_list[ list_idx ];
        list.m_item   = item;
        list.m_next   = m_first;
        list.m_prev   = 0xFFFFFFFFu;
        list.m_pinned = true;

        // insert at the beginning of the LRU list
        if (m_first != 0xFFFFFFFFu)
        {
            List& first = m_cache_list[ m_first ];
            first.m_prev = list_idx;
        }

        m_first = list_idx;
        if (m_last == 0xFFFFFFFFu)
            m_last  = m_first;

        if (m_manager->acquire( item ) == false)
            release_cycle( item );
    }
    else
    {
        // pin element
        List& list = m_cache_list[ it->second ];
        list.m_pinned = true;

        // move at the beginning of the LRU list
        touch( it->second, list );
    }
}

template <typename CacheManager>
void LRU<CacheManager>::unpin(const uint32 item)
{
    typename std::map<uint32,uint32>::iterator it = m_cache_map.find( item );
    const uint32 list_idx = it->second;

    List& list = m_cache_list[ list_idx ];
    list.m_pinned = false;

    // move at the beginning of the LRU list
    touch( list_idx, list );
}

template <typename CacheManager>
void LRU<CacheManager>::touch(const uint32 list_idx, List& list)
{
    // check whether this element is already at the beginning of the LRU list
    if (m_first == list_idx)
        return;

    // extract element from the LRU list
    if (list.m_prev != 0xFFFFFFFFu)
    {
        List& prev = m_cache_list[ list.m_prev ];
        prev.m_next = list.m_next;
    }
    if (list.m_next != 0xFFFFFFFFu)
    {
        List& next = m_cache_list[ list.m_next ];
        next.m_next = list.m_prev;
    }
    else // mark the new end of list
        m_last = list.m_prev;

    // re-insert at the beginning of the LRU list
    m_first = list_idx;
}

template <typename CacheManager>
void LRU<CacheManager>::release_cycle(const uint32 item_to_acquire)
{
    // walk the list from the end
    uint32 list_idx = m_last;
    bool acquired = false;

    do
    {
        if (list_idx == 0xFFFFFFFFu)
        {
            if (acquired)
                break;
            else
                throw cache_overflow();
        }

        List& list = m_cache_list[ list_idx ];
        if (list.m_pinned)
        {
            list_idx = list.m_prev;
            continue;
        }

        const uint32 prev = list.m_prev;

        // extract element from the LRU list
        if (list.m_prev != 0xFFFFFFFFu)
        {
            List& prev = m_cache_list[ list.m_prev ];
            prev.m_next = list.m_next;
        }
        else // mark the list as empty
            m_first = 0xFFFFFFFFu;

        if (list.m_next != 0xFFFFFFFFu)
        {
            List& next = m_cache_list[ list.m_next ];
            next.m_next = list.m_prev;
        }
        else // mark the new end of list
            m_last = list.m_prev;

        const uint32 item = list.m_item;

        // release this list entry
        m_cache_pool.push( list_idx );

        // and remove from the map
        m_cache_map.erase( m_cache_map.find( item ) );

        m_manager->release( item );
        if (acquired == false && m_manager->acquire( item_to_acquire ))
            acquired  = true;

        // jump to the previous
        list_idx = prev;
    }
    while (m_manager->low_watermark() == false);
}

} // namespace nvbio
