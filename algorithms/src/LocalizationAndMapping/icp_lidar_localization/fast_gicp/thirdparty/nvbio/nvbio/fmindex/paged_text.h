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

#include <nvbio/basic/vector.h>
#include <nvbio/basic/packedstream.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/basic/popcount.h>
#include <nvbio/basic/algorithms.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/cast_iterator.h>
#include <thrust/adjacent_difference.h>
#include <thrust/iterator/counting_iterator.h>
#include <thrust/iterator/permutation_iterator.h>
#include <thrust/iterator/transform_iterator.h>
#include <stack>
#include <numeric>

namespace nvbio {

/// build a set of buckets pointing to the lower/upper bounds of a sequence of keys
///
void build_buckets(const uint64 key_range, const uint32 n_keys, const uint64* keys, const uint32 bucket_size, nvbio::vector<host_tag,uint32>& buckets, const bool upper = true);

///
/// This class represents a packed paged text and rank-dictionary supporting parallel bulk insertions.
///
/// \tparam SYMBOL_SIZE_T   the size of the symbols, in bits
/// \tparam BIG_ENDIAN_T    the endianness with which the packed symbols are stored within a word:
///                         true  = the first symbols are stored in the most significant bits,
///                         false = the first symbols are stored in the least signficant bits
///
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
struct PagedText
{
    typedef uint64                                                           word_type;
    typedef PackedStream<const word_type*,uint8,SYMBOL_SIZE_T,BIG_ENDIAN_T>  const_packed_page_type;
    typedef PackedStream<      word_type*,uint8,SYMBOL_SIZE_T,BIG_ENDIAN_T>        packed_page_type;

    static const uint32 SYMBOL_SIZE      = SYMBOL_SIZE_T;
    static const uint32 SYMBOL_COUNT     = 1u << SYMBOL_SIZE;
    static const uint32 WORD_SIZE        = (8u*sizeof(word_type));
    static const uint32 SYMBOLS_PER_WORD = WORD_SIZE / SYMBOL_SIZE;
    static const uint32 BIG_ENDIAN       = BIG_ENDIAN_T;

    static const uint32 LOG_BUCKET_SIZE = 20u;
    static const uint32     BUCKET_SIZE = 1u << LOG_BUCKET_SIZE;

    /// constructor
    ///
    PagedText(
        const uint32 page_size    = 512 * 1024,         ///< page size, in bytes
        const uint32 segment_size = 128 * 1024 * 1024,  ///< segment size, in bytes
        const uint32 occ_intv     = 128);               ///< occurrence intervals, in symbols

    /// destructor
    ///
    ~PagedText();

    /// return the current page count
    ///
    uint32 page_count() const { return uint32( m_offsets.size() - 1u ); }

    /// return the current size
    ///
    uint64 size() const { return m_offsets.size() ? m_offsets.back() : 0u; }

    /// alloc new pages
    ///
    void grow();

    /// alloc a new page
    ///
    word_type* alloc_page();

    /// release a page
    ///
    void release_page(word_type* page);

    /// return the i-th page
    ///
    const word_type* get_page(const uint32 i) const { return m_pages[i]; }

    /// return the size of the i-th page
    ///
    uint32 get_page_size(const uint32 i) const { return uint32( m_offsets[i+1] - m_offsets[i] ); }

    /// return the size of the i-th page
    ///
    uint64 get_page_offset(const uint32 i) const { return m_offsets[i]; }

    /// return the i-th occurrence table page
    ///
    const uint32* get_occ_page(const uint32 i) const { return (uint32*)( m_pages[i] + m_page_size ); }

    /// find the page containing the i-th entry
    ///
    uint32 find_page(const uint64 i) const;

    /// indexing operator - return the i-th symbol
    ///
    uint8 operator[] (const uint64 i) const;

    /// compute the rank of c in [0,i]
    ///
    uint64 rank(const uint64 i, const uint8 c) const;

    /// compute the rank of c in [0,i]
    ///
    uint64 rank(const uint32 page_idx, const uint64 i, const uint8 c) const;

    /// reserve
    ///
    void reserve_pages(const uint32 n_pages);

    /// reserve free pages
    ///
    void reserve_free_pages(const uint32 n_pages);

    /// reserve
    ///
    void reserve(const uint64 n);

    /// needed host memory
    ///
    uint64 needed_host_memory(const uint64 n) const;

    /// needed device memory
    ///
    uint64 needed_device_memory(const uint64 n) const { return 0u; }

    /// resize and optionally copy a given vector
    ///
    void resize(const uint64 n, const uint8* c = NULL);

    /// perform a batch of parallel insertions
    ///
    void insert(const uint32 n, const uint64* g, const uint8* c);

    /// global symbol frequencies
    ///
    const uint64* symbol_frequencies() const;

    /// global symbol frequency
    ///
    uint64 symbol_frequency(const uint8 c) const { return symbol_frequencies()[c]; }

    /// defragment the pages
    ///
    void defrag();

    uint32                              m_page_size;        ///< page size, in words
    uint32                              m_segment_size;     ///< segment size, in words
    uint32                              m_occ_intv;         ///< occurrence interval, in symbols
    uint32                              m_occ_intv_w;       ///< occurrence interval, in words
    uint32                              m_occ_intv_log;     ///< occurrence interval's logarithm, in symbols
    uint32                              m_page_count;       ///< total number of allocated pages
    nvbio::vector<host_tag,word_type*>  m_segments;
    nvbio::vector<host_tag,word_type*>  m_pages;
    nvbio::vector<host_tag,word_type*>  m_new_pages;
    nvbio::vector<host_tag,uint64>      m_offsets;
    nvbio::vector<host_tag,uint64>      m_new_offsets;
    nvbio::vector<host_tag,uint64>      m_counters;
    nvbio::vector<host_tag,uint64>      m_new_counters;
    nvbio::vector<host_tag,uint32>      m_buckets;
    std::vector<word_type*>             m_pool;
    uint32                              m_pool_size;
    omp_lock_t                          m_lock;
    uint32                              m_count_table[256];
};

///
/// A data structure to hold a sparse subset of symbols from a larger set/string
///
struct SparseSymbolSet
{
    static const uint32 LOG_BUCKET_SIZE = 20u;
    static const uint32     BUCKET_SIZE = 1u << LOG_BUCKET_SIZE;

    /// constructor
    ///
    /// \param c            the special symbol recorded into this set
    ///
    SparseSymbolSet() : m_n( 0 ), m_n_special( 0 ) {}

    /// return the number of special symbols collected so far
    ///
    uint32 size() const { return m_n_special; }

    /// reserve enough storage for n special symbols
    ///
    void reserve(const uint64 n, const uint32 n_special);

    /// set the initial set of symbols
    ///
    /// \param range        total number of symbols in the virtual string
    ///
    /// \param n_special    number of symbols in the block which are special
    /// \param p_special    the positions of the special symbols in the block
    /// \param g_special    the insertion positions of the special symbols
    ///
    void set(const uint64 range, const uint32 n_special, const uint32* p, const uint32* id);

    /// simulates the insertion of a set of n_block symbols at positions g in a string,
    /// n_special of which are special and will be recorded in this set.
    /// Note that the actual symbols in the block don't need to be known, but in order to adjust the
    /// positions of the previously inserted symbols we need to know _all_ their insertion
    /// positions, for the entire block, not just for the special symbols.
    ///
    /// \param range        total number of symbols in the virtual string
    /// \param n_block      total number of inserted symbols
    /// \param g            insertion positions for the entire block
    /// \param n_special    number of symbols in the block which are special
    /// \param p_special    the positions of the special symbols in the block
    /// \param g_special    the insertion positions of the special symbols
    /// \param id_special   the ids associated to the special symbols
    /// \param n_special    number of symbols in c which are special
    ///
    void insert(
        const uint64    range,
        const uint32    n_block,
        const uint64*   g,
        const uint32    n_special,
        const uint32*   p_special,
        const uint64*   g_special,
        const uint32*   id_special);

    /// extend the range of string indices
    ///
    void set_range(const uint64 n);

    /// find how many special symbols there are in the range [0,i] (inclusive)
    ///
    uint32 rank(const uint64 i) const
    {
        if (i < m_n)
        {
            const uint64 ii = i+1;
            const uint32 b  = uint32( ii >> LOG_BUCKET_SIZE );
            const uint32 lo = m_buckets[b];
            const uint32 hi = m_buckets[b+1];
            return lower_bound_index( ii, &m_pos[lo], hi - lo ) + lo;
        }
        else if (i == uint64(-1))
            return 0;
        else // i >= m_n
            return m_n_special;
    }

    const uint64* pos() const { return raw_pointer( m_pos ); }
    const uint64* ids() const { return raw_pointer( m_id ); }

    uint64                              m_n;
    uint32                              m_n_special;
    nvbio::vector<host_tag,uint64>      m_pos;
    nvbio::vector<host_tag,uint64>      m_new_pos;
    nvbio::vector<host_tag,uint64>      m_id;
    nvbio::vector<host_tag,uint64>      m_new_id;
    nvbio::vector<host_tag,uint32>      m_buckets;
};

} // namespace nvbio

#include <nvbio/fmindex/paged_text_inl.h>
